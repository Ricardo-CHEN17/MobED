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
    active_lift_mask_.fill(false);
    forward_vel_override_ = 0.0;
    state_timer_ = 0.0;
    contact_debounce_ = 0.0;
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 0.0;
    cooldown_timer_ = 0.0;
    lift_start_angles_ = Eigen::Vector4d::Zero();
    front_target_angles_ = Eigen::Vector2d::Zero();
    rear_target_angles_ = Eigen::Vector2d::Zero();
}

void TaskController::abort() {
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 1.0;
    cooldown_timer_ = params_.cooldown_duration;
    forward_vel_override_ = 0.0;
    active_lift_mask_.fill(false);
    transitionTo(ClimbState::IDLE);
}

void TaskController::transitionTo(ClimbState new_state) {
    state_ = new_state;
    state_timer_ = 0.0;
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
    //
    // The paper states: "rotate the eccentric joints until they reach
    // the top of the curb."  For front legs (positive q_ecc convention):
    //
    //   Waypoint 1 — PEAK: start + step_over_angle + peak_offset
    //                Lifts wheel above obstacle edge with margin.
    //
    //   Waypoint 2 — EXTEND: start + step_over_angle + peak_offset (hold)
    //                Brief hold at peak to let wheel pass over edge.
    //
    //   Waypoint 3 — SETTLE: start + step_over_angle
    //                Leg lands on top of curb at the target support angle.
    //
    // step_over_angle = 0.65 rad (~37°) covering 5~10 cm curbs
    // (geometric derivation in TaskControllerParams).
    // ================================================================

    double peak = params_.step_over_angle + params_.lift_peak_angle_offset;
    double settle = params_.step_over_angle;
    double seg_time = params_.front_lift_duration / 3.0;

    lift_start_angles_ = current_ecc_angles;

    // Front-Left trajectory
    if (active_lift_mask_[FL]) {
        double start = current_ecc_angles(FL);
        std::vector<TrajectoryWaypoint> waypoints = {
            {start + peak,   seg_time},   // lift to peak
            {start + peak,   seg_time},   // hold at peak
            {start + settle, seg_time}    // settle onto curb
        };
        front_left_traj_.plan(start, waypoints);
        front_target_angles_(0) = start + settle;
    } else {
        front_target_angles_(0) = current_ecc_angles(FL);
    }

    // Front-Right trajectory
    if (active_lift_mask_[FR]) {
        double start = current_ecc_angles(FR);
        std::vector<TrajectoryWaypoint> waypoints = {
            {start + peak,   seg_time},
            {start + peak,   seg_time},
            {start + settle, seg_time}
        };
        front_right_traj_.plan(start, waypoints);
        front_target_angles_(1) = start + settle;
    } else {
        front_target_angles_(1) = current_ecc_angles(FR);
    }
}

void TaskController::planRearLiftTrajectories(const Eigen::Vector4d& current_ecc_angles) {
    // ================================================================
    // Rear leg step-over trajectory.
    //
    // Rear legs operate at NEGATIVE eccentric angles (outward-down config).
    // To lift the wheel upward, we must DECREASE the angle magnitude,
    // i.e. bring q_ecc toward zero (or slightly positive at peak).
    //
    //   Waypoint 1 — PEAK: start + step_over_angle + peak_offset
    //                (less negative = leg swings upward / inward)
    //
    //   Waypoint 2 — EXTEND: hold at peak briefly
    //
    //   Waypoint 3 — SETTLE: rear_settle_angle (−0.80 rad)
    //                Leg returns to a stable negative angle pushing DOWN
    //                onto the curb surface for solid ground support.
    // ================================================================

    double seg_time = params_.rear_lift_duration / 3.0;

    lift_start_angles_ = current_ecc_angles;

    // Rear-Left
    if (active_lift_mask_[RL]) {
        double start = current_ecc_angles(RL);
        std::vector<TrajectoryWaypoint> waypoints = {
            {start + params_.step_over_angle + params_.lift_peak_angle_offset, seg_time},  // lift
            {start + params_.step_over_angle + params_.lift_peak_angle_offset, seg_time},  // hold
            {params_.rear_settle_angle, seg_time}   // settle at stable support angle
        };
        rear_left_traj_.plan(start, waypoints);
        rear_target_angles_(0) = params_.rear_settle_angle;
    } else {
        rear_target_angles_(0) = current_ecc_angles(RL);
    }

    // Rear-Right
    if (active_lift_mask_[RR]) {
        double start = current_ecc_angles(RR);
        std::vector<TrajectoryWaypoint> waypoints = {
            {start + params_.step_over_angle + params_.lift_peak_angle_offset, seg_time},
            {start + params_.step_over_angle + params_.lift_peak_angle_offset, seg_time},
            {params_.rear_settle_angle, seg_time}
        };
        rear_right_traj_.plan(start, waypoints);
        rear_target_angles_(1) = params_.rear_settle_angle;
    } else {
        rear_target_angles_(1) = current_ecc_angles(RR);
    }
}

void TaskController::update(
    const estimation::ContactDetector& contact_detector,
    const Eigen::Vector4d& current_ecc_angles,
    const Eigen::Vector4d& current_ecc_efforts,
    double dt)
{
    if (dt <= 0.0 || dt > 0.5) return;

    state_timer_ += dt;

    // Clear overrides by default
    override_mask_.fill(false);
    forward_vel_override_ = 0.0;

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

            // Two-stage detection handles shock transient + 30ms sustained load confirmation.
            // When front impact is confirmed, populate active_lift_mask_ and transition to FRONT_CONTACT.
            if (contact_detector.frontImpactDetected()) {
                auto impacts = contact_detector.getImpactFlags();
                active_lift_mask_[FL] = impacts[FL];
                active_lift_mask_[FR] = impacts[FR];
                active_lift_mask_[RL] = false;
                active_lift_mask_[RR] = false;

                // Fallback: if frontImpactDetected() was true but individual flags clear, lift both
                if (!active_lift_mask_[FL] && !active_lift_mask_[FR]) {
                    active_lift_mask_[FL] = true;
                    active_lift_mask_[FR] = true;
                }
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

            // In case the other front wheel also contacts during this initial tick
            auto impacts = contact_detector.getImpactFlags();
            if (impacts[FL]) active_lift_mask_[FL] = true;
            if (impacts[FR]) active_lift_mask_[FR] = true;

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

            // Check if planned trajectories are complete
            bool traj_complete = true;
            if (active_lift_mask_[FL] && !front_left_traj_.isComplete(state_timer_)) {
                traj_complete = false;
            }
            if (active_lift_mask_[FR] && !front_right_traj_.isComplete(state_timer_)) {
                traj_complete = false;
            }

            // Closed-loop touchdown detection:
            // During descent phase (t >= 0.6 * duration), resistive torque on eccentric joint
            // (|tau_ecc| > touchdown_torque_threshold) confirms solid contact with the curb top.
            bool fl_touchdown = !active_lift_mask_[FL] ||
                (state_timer_ >= 0.6 * params_.front_lift_duration &&
                 std::abs(current_ecc_efforts(FL)) >= params_.touchdown_torque_threshold);

            bool fr_touchdown = !active_lift_mask_[FR] ||
                (state_timer_ >= 0.6 * params_.front_lift_duration &&
                 std::abs(current_ecc_efforts(FR)) >= params_.touchdown_torque_threshold);

            if (traj_complete || (fl_touchdown && fr_touchdown)) {
                transitionTo(ClimbState::FRONT_PLACED);
            }
            break;
        }

        // ============================================================
        // FRONT_PLACED: Front wheels on top, drive body forward
        // Wait for rear wheels to reach the obstacle
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

            // Command forward motion to advance the body
            forward_vel_override_ = params_.forward_drive_speed;

            // Monitor for rear wheel impact
            if (contact_detector.rearImpactDetected()) {
                auto impacts = contact_detector.getImpactFlags();
                active_lift_mask_[RL] = impacts[RL];
                active_lift_mask_[RR] = impacts[RR];

                // Fallback: if rear impact detected but individual flags clear,
                // mirror front active lift mask (curb side correspondence)
                if (!active_lift_mask_[RL] && !active_lift_mask_[RR]) {
                    active_lift_mask_[RL] = active_lift_mask_[FL];
                    active_lift_mask_[RR] = active_lift_mask_[FR];
                }
                transitionTo(ClimbState::REAR_CONTACT);
            } else if (state_timer_ > params_.forward_drive_duration) {
                // Safety timeout: if driving completed without rear wheel collision,
                // the obstacle was cleared, or it was a single bump/ramp lip.
                // Do NOT blindly lift rear legs! Safely abort with smooth blend-out.
                blend_out_timer_ = 0.0;
                blend_out_weight_ = 1.0;
                cooldown_timer_ = params_.cooldown_duration;
                forward_vel_override_ = 0.0;
                transitionTo(ClimbState::IDLE);
            }
            break;
        }

        // ============================================================
        // REAR_CONTACT: Rear wheels hit stair, prepare to lift
        // ============================================================
        case ClimbState::REAR_CONTACT: {
            // Maintain front wheels on top of curb
            if (active_lift_mask_[FL]) {
                override_mask_[FL] = true;
                ecc_overrides_(FL) = front_target_angles_(0);
            }
            if (active_lift_mask_[FR]) {
                override_mask_[FR] = true;
                ecc_overrides_(FR) = front_target_angles_(1);
            }

            // Accumulate any impact on rear legs
            auto impacts = contact_detector.getImpactFlags();
            if (impacts[RL]) active_lift_mask_[RL] = true;
            if (impacts[RR]) active_lift_mask_[RR] = true;

            forward_vel_override_ = 0.0;
            planRearLiftTrajectories(current_ecc_angles);
            transitionTo(ClimbState::REAR_LIFT);
            break;
        }

        // ============================================================
        // REAR_LIFT: Execute rear leg step-over trajectory
        // ============================================================
        case ClimbState::REAR_LIFT: {
            // Front wheels stay on top of curb
            if (active_lift_mask_[FL]) {
                override_mask_[FL] = true;
                ecc_overrides_(FL) = front_target_angles_(0);
            }
            if (active_lift_mask_[FR]) {
                override_mask_[FR] = true;
                ecc_overrides_(FR) = front_target_angles_(1);
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

            bool traj_complete = true;
            if (active_lift_mask_[RL] && !rear_left_traj_.isComplete(state_timer_)) {
                traj_complete = false;
            }
            if (active_lift_mask_[RR] && !rear_right_traj_.isComplete(state_timer_)) {
                traj_complete = false;
            }

            // Closed-loop touchdown detection for rear legs
            bool rl_touchdown = !active_lift_mask_[RL] ||
                (state_timer_ >= 0.6 * params_.rear_lift_duration &&
                 std::abs(current_ecc_efforts(RL)) >= params_.touchdown_torque_threshold);

            bool rr_touchdown = !active_lift_mask_[RR] ||
                (state_timer_ >= 0.6 * params_.rear_lift_duration &&
                 std::abs(current_ecc_efforts(RR)) >= params_.touchdown_torque_threshold);

            if (traj_complete || (rl_touchdown && rr_touchdown)) {
                transitionTo(ClimbState::REAR_PLACED);
            }
            break;
        }

        // ============================================================
        // REAR_PLACED: All wheels on top, advance body then blend-out to IDLE
        // ============================================================
        case ClimbState::REAR_PLACED: {
            if (active_lift_mask_[FL]) {
                override_mask_[FL] = true;
                ecc_overrides_(FL) = front_target_angles_(0);
            }
            if (active_lift_mask_[FR]) {
                override_mask_[FR] = true;
                ecc_overrides_(FR) = front_target_angles_(1);
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

            if (state_timer_ >= params_.settling_duration) {
                // Begin blend-out: start fading FSM override back to IK control
                blend_out_timer_ = 0.0;
                blend_out_weight_ = 1.0;
                cooldown_timer_ = params_.cooldown_duration; // Start 1.0s refractory cooldown
                contact_debounce_ = 0.0;
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
