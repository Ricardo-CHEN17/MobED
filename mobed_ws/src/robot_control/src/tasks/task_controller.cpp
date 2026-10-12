#include "robot_control/tasks/task_controller.hpp"
#include <cmath>

namespace robot_control {
namespace tasks {

TaskController::TaskController(const TaskControllerParams& params)
    : params_(params)
{
    reset();
}

void TaskController::reset() {
    state_ = ClimbState::IDLE;
    ecc_overrides_ = Eigen::Vector4d::Zero();
    override_mask_.fill(false);
    wheel_speed_overrides_.setZero();
    wheel_override_mask_.fill(false);
    active_lift_mask_.fill(false);
    forward_vel_override_ = 0.0;
    state_timer_ = 0.0;
    contact_debounce_ = 0.0;
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 0.0;
    cooldown_timer_ = 0.0;
    travel_distance_ = 0.0;
    lift_start_angles_ = Eigen::Vector4d::Zero();
    front_target_angles_ = Eigen::Vector2d::Zero();
    rear_target_angles_ = Eigen::Vector2d::Zero();
    has_climbed_ = false;
}

void TaskController::abort() {
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 1.0;
    cooldown_timer_ = params_.cooldown_duration;
    travel_distance_ = 0.0;
    forward_vel_override_ = 0.0;
    wheel_speed_overrides_.setZero();
    wheel_override_mask_.fill(false);
    active_lift_mask_.fill(false);
    transitionTo(ClimbState::IDLE);
}

void TaskController::transitionTo(ClimbState new_state) {
    state_ = new_state;
    state_timer_ = 0.0;
    travel_distance_ = 0.0;
}

double TaskController::computeAdaptiveLiftAngle([[maybe_unused]] double q_start) const {
    const double l_ecc = 0.075;
    if (params_.climb_mode == ClimbMode::HIGH_200MM || params_.target_curb_height >= 0.18) {
        return 2.80;
    }
    double ratio = std::clamp(params_.target_curb_height / (2.0 * l_ecc), 0.0, 1.0);
    double delta_theta = std::asin(ratio) * 1.6;
    return std::clamp(delta_theta, 0.40, 2.60);
}

double TaskController::computeRearLevelingAngle(double q_front_avg, double h_curb) const {
    const double l_ecc = 0.075;
    // Condition for chassis pitch == 0:
    // Front pivot height above ground: h_curb + l_ecc * cos(q_front_avg)
    // Rear pivot height above ground: l_ecc * cos(q_rear)
    // l_ecc * cos(q_rear) = h_curb + l_ecc * cos(q_front_avg)
    double cos_target = std::cos(q_front_avg) + (h_curb / l_ecc);
    if (cos_target >= std::cos(MIN_SINGULARITY_INCLINATION)) {
        // Enforce natural inclination limit (~6.9 deg, 0.12 rad) to avoid the 0 rad singularity
        // while achieving ~99.3% maximum ground lift and preserving a ~1.1cm impact lever arm.
        return -MIN_SINGULARITY_INCLINATION;
    }
    cos_target = std::clamp(cos_target, -1.0, 1.0);
    return -std::acos(cos_target);
}

std::string TaskController::getStateName() const {
    switch (state_) {
        case ClimbState::IDLE:           return "IDLE";
        case ClimbState::FRONT_CONTACT:  return "FRONT_CONTACT";
        case ClimbState::FRONT_LIFT:     return "FRONT_LIFT";
        case ClimbState::FRONT_PLACED:   return "FRONT_PLACED";
        case ClimbState::REAR_CONTACT:   return "REAR_CONTACT";
        case ClimbState::REAR_LIFT:      return "REAR_LIFT";
        case ClimbState::REAR_PLACED:    return "REAR_PLACED";
        default:                         return "UNKNOWN";
    }
}

void TaskController::planFrontLiftTrajectories(const Eigen::Vector4d& current_ecc_angles) {
    // ================================================================
    // Front leg step-over trajectory (3 waypoints):
    // Front legs swing forward-upward (+X direction, delta_q > 0).
    // In Mode 1: adaptive angle calculated from curb height.
    // In Mode 2: full rotation up to +2.80 rad (~160 deg) for 200mm climb.
    // ================================================================

    double seg_time = params_.front_lift_duration / 3.0;
    lift_start_angles_ = current_ecc_angles;

    bool is_high_mode = (params_.climb_mode == ClimbMode::HIGH_200MM || params_.target_curb_height >= 0.18);

    // Front-Left trajectory
    if (active_lift_mask_[FL]) {
        double start = current_ecc_angles(FL);
        double fl_peak = 0.0;
        double fl_settle = 0.0;
        if (is_high_mode) {
            fl_peak = 3.05;
            fl_settle = 2.80;
        } else {
            double fl_lift = computeAdaptiveLiftAngle(start);
            fl_peak = start + fl_lift + params_.lift_peak_angle_offset;
            fl_settle = start + fl_lift;
        }
        std::vector<TrajectoryWaypoint> waypoints = {
            {fl_peak,   seg_time},   // lift to peak
            {fl_peak,   seg_time},   // hold at peak
            {fl_settle, seg_time}    // settle onto curb
        };
        front_left_traj_.plan(start, waypoints);
        front_target_angles_(0) = fl_settle;
    } else {
        front_target_angles_(0) = current_ecc_angles(FL);
    }

    // Front-Right trajectory
    if (active_lift_mask_[FR]) {
        double start = current_ecc_angles(FR);
        double fr_peak = 0.0;
        double fr_settle = 0.0;
        if (is_high_mode) {
            fr_peak = 3.05;
            fr_settle = 2.80;
        } else {
            double fr_lift = computeAdaptiveLiftAngle(start);
            fr_peak = start + fr_lift + params_.lift_peak_angle_offset;
            fr_settle = start + fr_lift;
        }
        std::vector<TrajectoryWaypoint> waypoints = {
            {fr_peak,   seg_time},
            {fr_peak,   seg_time},
            {fr_settle, seg_time}
        };
        front_right_traj_.plan(start, waypoints);
        front_target_angles_(1) = fr_settle;
    } else {
        front_target_angles_(1) = current_ecc_angles(FR);
    }
}

void TaskController::planRearLiftTrajectories(const Eigen::Vector4d& current_ecc_angles) {
    // ================================================================
    // Rear leg step-over trajectory.
    // Rear legs swing backward-upward (-X direction, negative angles).
    // In Mode 1: swings backward-upward and lands at stable outward stance.
    // In Mode 2: full rotation to -2.80 rad (-pi) atop 200mm curb.
    // ================================================================

    double seg_time = params_.rear_lift_duration / 3.0;
    lift_start_angles_ = current_ecc_angles;

    bool is_high_mode = (params_.climb_mode == ClimbMode::HIGH_200MM || params_.target_curb_height >= 0.18);

    double rl_peak = 0.0;
    double rl_settle = 0.0;
    double rr_peak = 0.0;
    double rr_settle = 0.0;

    if (is_high_mode) {
        rl_peak = -3.05;
        rl_settle = -2.80;
        rr_peak = -3.05;
        rr_settle = -2.80;
    } else {
        // Mode 1: curb height <= 0.15m (e.g. 100mm curb)
        // Nominal support stance atop flat obstacle is -0.65 rad.
        rl_settle = -0.65;
        rr_settle = -0.65;
        // Peak angle swings backward-upward to ensure rear wheels clear curb lip
        double lift_ang = computeAdaptiveLiftAngle(current_ecc_angles(RL));
        rl_peak = -0.65 - lift_ang - params_.lift_peak_angle_offset;
        rr_peak = -0.65 - lift_ang - params_.lift_peak_angle_offset;
        rl_peak = std::clamp(rl_peak, -2.50, -1.60);
        rr_peak = std::clamp(rr_peak, -2.50, -1.60);
    }

    // Rear-Left
    if (active_lift_mask_[RL]) {
        double start = current_ecc_angles(RL);
        std::vector<TrajectoryWaypoint> waypoints = {
            {rl_peak,   seg_time},  // lift backward-upward
            {rl_peak,   seg_time},  // hold
            {rl_settle, seg_time}   // settle at stable support angle
        };
        rear_left_traj_.plan(start, waypoints);
        rear_target_angles_(0) = rl_settle;
    } else {
        rear_target_angles_(0) = current_ecc_angles(RL);
    }

    // Rear-Right
    if (active_lift_mask_[RR]) {
        double start = current_ecc_angles(RR);
        std::vector<TrajectoryWaypoint> waypoints = {
            {rr_peak,   seg_time},
            {rr_peak,   seg_time},
            {rr_settle, seg_time}
        };
        rear_right_traj_.plan(start, waypoints);
        rear_target_angles_(1) = rr_settle;
    } else {
        rear_target_angles_(1) = current_ecc_angles(RR);
    }
}

void TaskController::update(
    const estimation::ContactDetector& contact_detector,
    const Eigen::Vector4d& current_ecc_angles,
    const Eigen::Vector4d& current_ecc_efforts,
    double dt,
    double body_vx)
{
    if (dt <= 0.0 || dt > 0.5) return;

    state_timer_ += dt;

    // Clear overrides by default
    override_mask_.fill(false);
    forward_vel_override_ = 0.0;
    wheel_speed_overrides_.setZero();
    wheel_override_mask_.fill(false);

    switch (state_) {
        // ============================================================
        // IDLE: Normal driving, monitoring for front wheel impact
        // ============================================================
        case ClimbState::IDLE: {
            // Refractory cooldown timer: inhibit detection after touchdown or abort
            if (cooldown_timer_ > 0.0) {
                cooldown_timer_ = std::max(0.0, cooldown_timer_ - dt);
                break;
            }

            // Two-stage detection handles shock transient + sustained load confirmation.
            // When front impact is confirmed, lift both front legs symmetrically.
            if (contact_detector.frontImpactDetected()) {
                active_lift_mask_[FL] = true;
                active_lift_mask_[FR] = true;
                active_lift_mask_[RL] = false;
                active_lift_mask_[RR] = false;
                transitionTo(ClimbState::FRONT_CONTACT);
            }
            break;
        }

        // ============================================================
        // FRONT_CONTACT: Front wheels hit stair, prepare to lift
        // Freeze all motion, plan front leg trajectories
        // ============================================================
        case ClimbState::FRONT_CONTACT: {
            // Stop all wheel motion
            forward_vel_override_ = 0.0;
            active_lift_mask_[FL] = true;
            active_lift_mask_[FR] = true;

            // Plan the step-over trajectories for active legs
            planFrontLiftTrajectories(current_ecc_angles);

            // Immediately transition to lifting
            transitionTo(ClimbState::FRONT_LIFT);
            break;
        }

        // ============================================================
        // FRONT_LIFT: Execute front leg step-over trajectory
        // Override only active front legs; remaining legs stay in BalanceController
        // ============================================================
        case ClimbState::FRONT_LIFT: {
            if (active_lift_mask_[FL]) {
                override_mask_[FL] = true;
                double q_fl, dq_fl, ddq_fl;
                front_left_traj_.evaluate(state_timer_, q_fl, dq_fl, ddq_fl);
                ecc_overrides_(FL) = q_fl;
            }
            if (active_lift_mask_[FR]) {
                override_mask_[FR] = true;
                double q_fr, dq_fr, ddq_fr;
                front_right_traj_.evaluate(state_timer_, q_fr, dq_fr, ddq_fr);
                ecc_overrides_(FR) = q_fr;
            }

            // Active pitch counter-balancing during FRONT_LIFT:
            // As front legs ascend along trajectory, rear legs synchronously extend downward
            // to elevate rear chassis and eliminate pitch tilt in real time.
            if (params_.active_pitch_counter_balance) {
                double q_f_current = 0.5 * (ecc_overrides_(FL) + ecc_overrides_(FR));
                double q_f_start = 0.5 * (lift_start_angles_(FL) + lift_start_angles_(FR));
                double q_f_target = 0.5 * (front_target_angles_(0) + front_target_angles_(1));
                double q_r_start = 0.5 * (lift_start_angles_(RL) + lift_start_angles_(RR));

                double lift_progress = 1.0;
                if (std::abs(q_f_target - q_f_start) > 1e-3) {
                    lift_progress = std::clamp((q_f_current - q_f_start) / (q_f_target - q_f_start), 0.0, 1.0);
                }
                double q_rear_target = computeRearLevelingAngle(q_f_target, params_.target_curb_height);
                double q_rear_level = (1.0 - lift_progress) * q_r_start + lift_progress * q_rear_target;

                override_mask_[RL] = true;
                override_mask_[RR] = true;
                ecc_overrides_(RL) = q_rear_level;
                ecc_overrides_(RR) = q_rear_level;
            }

            // Traction assist on vertical obstacles (Mode 2)
            if (params_.front_roll_speed > 1e-3) {
                if (active_lift_mask_[FL]) {
                    wheel_override_mask_[FL] = true;
                    wheel_speed_overrides_(FL) = params_.front_roll_speed;
                }
                if (active_lift_mask_[FR]) {
                    wheel_override_mask_[FR] = true;
                    wheel_speed_overrides_(FR) = params_.front_roll_speed;
                }
                wheel_override_mask_[RL] = true;
                wheel_override_mask_[RR] = true;
                double rear_push_speed = (params_.climb_mode == ClimbMode::HIGH_200MM) ? 0.06 : params_.forward_drive_speed;
                wheel_speed_overrides_(RL) = rear_push_speed / 0.065;
                wheel_speed_overrides_(RR) = rear_push_speed / 0.065;
                forward_vel_override_ = rear_push_speed;
            }

            // Check if planned trajectories are complete
            bool traj_complete = true;
            if (active_lift_mask_[FL] && !front_left_traj_.isComplete(state_timer_)) {
                traj_complete = false;
            }
            if (active_lift_mask_[FR] && !front_right_traj_.isComplete(state_timer_)) {
                traj_complete = false;
            }

            // Closed-loop touchdown detection:
            // During descent phase (t >= 0.8 * duration), resistive torque on eccentric joint
            // (|tau_ecc| >= 5.0 Nm) confirms solid contact with the curb top.
            bool fl_touchdown = !active_lift_mask_[FL] ||
                (state_timer_ >= 0.8 * params_.front_lift_duration &&
                 std::abs(current_ecc_efforts(FL)) >= 5.0);

            bool fr_touchdown = !active_lift_mask_[FR] ||
                (state_timer_ >= 0.8 * params_.front_lift_duration &&
                 std::abs(current_ecc_efforts(FR)) >= 5.0);

            if (traj_complete || (fl_touchdown && fr_touchdown)) {
                transitionTo(ClimbState::FRONT_PLACED);
            }
            break;
        }

        // ============================================================
        // FRONT_PLACED: Front wheels on top, drive body forward
        // Wait for rear wheels to reach the obstacle
        // Active pitch counter-balance: extend rear legs downward (toward 0 rad)
        // ============================================================
        case ClimbState::FRONT_PLACED: {
            // Maintain only active front wheels on top of curb
            if (active_lift_mask_[FL]) {
                override_mask_[FL] = true;
                ecc_overrides_(FL) = front_target_angles_(0);
            }
            if (active_lift_mask_[FR]) {
                override_mask_[FR] = true;
                ecc_overrides_(FR) = front_target_angles_(1);
            }

            // Active pitch counter-balancing:
            // Extend rear legs downward (toward 0 rad) to elevate rear chassis
            if (params_.active_pitch_counter_balance) {
                double q_f_avg = 0.5 * (ecc_overrides_(FL) + ecc_overrides_(FR));
                double q_rear_level = computeRearLevelingAngle(q_f_avg, params_.target_curb_height);
                override_mask_[RL] = true;
                override_mask_[RR] = true;
                ecc_overrides_(RL) = q_rear_level;
                ecc_overrides_(RR) = q_rear_level;
            }

            // Command forward motion to advance the body
            forward_vel_override_ = params_.forward_drive_speed;
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(params_.forward_drive_speed / 0.065);

            // Calculate instantaneous effective distance between front and rear ground contact points:
            // L_eff = wheelbase + l_ecc * (sin(q_front) - sin(q_rear))
            double q_f_curr = 0.5 * (ecc_overrides_(FL) + ecc_overrides_(FR));
            double q_r_curr = 0.5 * (ecc_overrides_(RL) + ecc_overrides_(RR));
            double effective_wheelbase = params_.wheelbase + 0.075 * (std::sin(q_f_curr) - std::sin(q_r_curr));

            // Closed-loop odometry travel integration (using measured or commanded speed)
            double effective_vx = (std::abs(body_vx) > 1e-3) ? std::abs(body_vx) : params_.forward_drive_speed;
            travel_distance_ += effective_vx * dt;

            // Monitor for rear wheel impact:
            // Rear wheels cannot reach the curb until the robot has driven forward across the curb
            bool rear_can_reach_curb = (state_timer_ > 1.2 && travel_distance_ >= (effective_wheelbase - 0.06));
            if (rear_can_reach_curb && (contact_detector.rearImpactDetected() ||
                                        travel_distance_ >= (effective_wheelbase + 0.08) ||
                                        state_timer_ > params_.forward_drive_duration)) {
                active_lift_mask_[RL] = true;
                active_lift_mask_[RR] = true;
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_CONTACT);
            }
            break;
        }

        // ============================================================
        // REAR_CONTACT: Rear wheels hit stair, prepare to lift
        // ============================================================
        case ClimbState::REAR_CONTACT: {
            // Maintain front wheels on top of curb
            override_mask_[FL] = true;
            override_mask_[FR] = true;
            ecc_overrides_(FL) = front_target_angles_(0);
            ecc_overrides_(FR) = front_target_angles_(1);

            if (params_.active_pitch_counter_balance) {
                override_mask_[RL] = true;
                override_mask_[RR] = true;
                ecc_overrides_(RL) = -MIN_SINGULARITY_INCLINATION;
                ecc_overrides_(RR) = -MIN_SINGULARITY_INCLINATION;
            }

            active_lift_mask_[RL] = true;
            active_lift_mask_[RR] = true;

            forward_vel_override_ = 0.0;
            planRearLiftTrajectories(current_ecc_angles);
            transitionTo(ClimbState::REAR_LIFT);
            break;
        }

        // ============================================================
        // REAR_LIFT: Execute rear leg step-over trajectory
        // ============================================================
        case ClimbState::REAR_LIFT: {
            bool is_high_mode = (params_.climb_mode == ClimbMode::HIGH_200MM || params_.target_curb_height >= 0.18);

            // Front wheels stay on top of curb
            // In Mode 1: smoothly interpolate front legs from landing crouch back to nominal support stance (0.65 rad)
            // This actively elevates the chassis by ~8cm, granting ample clearance for rear steering columns to clear curb edge.
            // In Mode 2: maintain high-stair posture (front_target_angles_ = 2.80 rad).
            if (active_lift_mask_[FL]) {
                override_mask_[FL] = true;
                if (is_high_mode) {
                    ecc_overrides_(FL) = front_target_angles_(0);
                } else {
                    double progress = std::clamp(state_timer_ / params_.rear_lift_duration, 0.0, 1.0);
                    ecc_overrides_(FL) = (1.0 - progress) * front_target_angles_(0) + progress * 0.65;
                }
            }
            if (active_lift_mask_[FR]) {
                override_mask_[FR] = true;
                if (is_high_mode) {
                    ecc_overrides_(FR) = front_target_angles_(1);
                } else {
                    double progress = std::clamp(state_timer_ / params_.rear_lift_duration, 0.0, 1.0);
                    ecc_overrides_(FR) = (1.0 - progress) * front_target_angles_(1) + progress * 0.65;
                }
            }

            if (active_lift_mask_[RL]) {
                override_mask_[RL] = true;
                double q_rl, dq_rl, ddq_rl;
                rear_left_traj_.evaluate(state_timer_, q_rl, dq_rl, ddq_rl);
                ecc_overrides_(RL) = q_rl;
            }
            if (active_lift_mask_[RR]) {
                override_mask_[RR] = true;
                double q_rr, dq_rr, ddq_rr;
                rear_right_traj_.evaluate(state_timer_, q_rr, dq_rr, ddq_rr);
                ecc_overrides_(RR) = q_rr;
            }

            // Maintain positive traction while rear legs lift:
            // Front wheels pull body forward across curb
            wheel_override_mask_[FL] = true;
            wheel_override_mask_[FR] = true;
            wheel_speed_overrides_(FL) = params_.forward_drive_speed / 0.065;
            wheel_speed_overrides_(FR) = params_.forward_drive_speed / 0.065;
            forward_vel_override_ = params_.forward_drive_speed;

            if (params_.rear_roll_speed > 1e-3) {
                if (active_lift_mask_[RL]) {
                    wheel_override_mask_[RL] = true;
                    wheel_speed_overrides_(RL) = params_.rear_roll_speed;
                }
                if (active_lift_mask_[RR]) {
                    wheel_override_mask_[RR] = true;
                    wheel_speed_overrides_(RR) = params_.rear_roll_speed;
                }
            } else {
                if (active_lift_mask_[RL]) {
                    wheel_override_mask_[RL] = true;
                    wheel_speed_overrides_(RL) = params_.forward_drive_speed / 0.065;
                }
                if (active_lift_mask_[RR]) {
                    wheel_override_mask_[RR] = true;
                    wheel_speed_overrides_(RR) = params_.forward_drive_speed / 0.065;
                }
            }

            bool traj_complete = true;
            if (active_lift_mask_[RL] && !rear_left_traj_.isComplete(state_timer_)) {
                traj_complete = false;
            }
            if (active_lift_mask_[RR] && !rear_right_traj_.isComplete(state_timer_)) {
                traj_complete = false;
            }

            // Closed-loop touchdown detection for rear legs
            // During descent phase (t >= 0.85 * duration), resistive torque on eccentric joint
            // (|tau_ecc| >= 5.0 Nm) confirms solid contact with the curb top.
            bool rl_touchdown = !active_lift_mask_[RL] ||
                (state_timer_ >= 0.85 * params_.rear_lift_duration &&
                 std::abs(current_ecc_efforts(RL)) >= 5.0);

            bool rr_touchdown = !active_lift_mask_[RR] ||
                (state_timer_ >= 0.85 * params_.rear_lift_duration &&
                 std::abs(current_ecc_efforts(RR)) >= 5.0);

            if (traj_complete || (rl_touchdown && rr_touchdown)) {
                transitionTo(ClimbState::REAR_PLACED);
            }
            break;
        }

        // ============================================================
        // REAR_PLACED: All wheels on top, advance body then blend-out to IDLE
        // ============================================================
        case ClimbState::REAR_PLACED: {
            bool is_high_mode = (params_.climb_mode == ClimbMode::HIGH_200MM || params_.target_curb_height >= 0.18);
            if (active_lift_mask_[FL]) {
                override_mask_[FL] = true;
                ecc_overrides_(FL) = is_high_mode ? front_target_angles_(0) : 0.65;
            }
            if (active_lift_mask_[FR]) {
                override_mask_[FR] = true;
                ecc_overrides_(FR) = is_high_mode ? front_target_angles_(1) : 0.65;
            }
            if (active_lift_mask_[RL]) {
                override_mask_[RL] = true;
                ecc_overrides_(RL) = rear_target_angles_(0);
            }
            if (active_lift_mask_[RR]) {
                override_mask_[RR] = true;
                ecc_overrides_(RR) = rear_target_angles_(1);
            }

            // Command forward motion to advance the body fully onto the obstacle
            forward_vel_override_ = params_.forward_drive_speed;
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(params_.forward_drive_speed / 0.065);

            if (state_timer_ >= params_.settling_duration) {
                // Begin blend-out: start fading FSM override back to IK control
                blend_out_timer_ = 0.0;
                blend_out_weight_ = 1.0;
                cooldown_timer_ = params_.cooldown_duration; // Start 1.0s refractory cooldown
                contact_debounce_ = 0.0;
                has_climbed_ = true;
                if (!is_high_mode) {
                    front_target_angles_(0) = 0.65;
                    front_target_angles_(1) = 0.65;
                }
                transitionTo(ClimbState::IDLE);
            }
            break;
        }
    }

    // ================================================================
    // Post-switch: blend-out fade (runs after transition to IDLE)
    // ================================================================
    if (state_ == ClimbState::IDLE && blend_out_weight_ > 0.0) {
        blend_out_timer_ += dt;
        double progress = blend_out_timer_ / params_.blend_out_duration;
        blend_out_weight_ = std::max(0.0, 1.0 - progress);

        if (blend_out_weight_ > 0.0) {
            // Keep publishing the last known override angles with fading weight for active legs
            for (int i = 0; i < NUM_LEGS; ++i) {
                if (active_lift_mask_[i]) {
                    override_mask_[i] = true;
                }
            }
        } else {
            // Blend-out complete — fully release control
            override_mask_.fill(false);
            active_lift_mask_.fill(false);
        }
    }
}

std::array<bool, NUM_LEGS> TaskController::getContactValid() const {
    std::array<bool, NUM_LEGS> valid = {true, true, true, true};
    if (state_ == ClimbState::FRONT_LIFT) {
        if (active_lift_mask_[FL]) valid[FL] = false;
        if (active_lift_mask_[FR]) valid[FR] = false;
    } else if (state_ == ClimbState::REAR_LIFT) {
        if (active_lift_mask_[RL]) valid[RL] = false;
        if (active_lift_mask_[RR]) valid[RR] = false;
    }
    return valid;
}

}  // namespace tasks
}  // namespace robot_control
