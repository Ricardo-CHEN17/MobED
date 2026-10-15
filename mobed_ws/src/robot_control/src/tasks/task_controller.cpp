#include "robot_control/tasks/task_controller.hpp"
#include <cmath>
#include <algorithm>

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
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 0.0;
    cooldown_timer_ = 0.0;
    travel_distance_ = 0.0;
    stall_jam_timer_ = 0.0;
    current_front_target_ = 0.0;
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
    stall_jam_timer_ = 0.0;
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
    stall_jam_timer_ = 0.0;
}

double TaskController::computeFrontTargetAngle(double h_curb) const {
    double ratio = std::clamp((h_curb - 0.04) / (0.20 - 0.04), 0.0, 1.0);
    // Maps [0.04m, 0.20m] smoothly to [1.65 rad, 1.95 rad] (95° ~ 111.7°)
    // 5cm: 1.65 rad (120mm ground clearance)
    // 10cm: 1.80 rad (103mm ground clearance)
    // 20cm: 1.95 rad (88mm clearance, preserves >68mm steer column clearance above curb plateau!)
    // Strictly capped at 1.95 rad to prevent steer column and chassis collision box from bottoming out (触底).
    return 1.65 + ratio * (1.95 - 1.65);
}

double TaskController::computeAdaptiveLiftAngle([[maybe_unused]] double q_start) const {
    return computeFrontTargetAngle(params_.target_curb_height);
}

double TaskController::computeRearLevelingAngle([[maybe_unused]] double q_front_avg, double h_curb) const {
    double ratio = std::clamp((h_curb - 0.04) / (0.20 - 0.04), 0.0, 1.0);
    // Smooth transition from -0.55 rad (gentle 40mm) to -0.15 rad (extreme 200mm high clearance)
    return -0.55 + ratio * (-0.15 - (-0.55));
}

double TaskController::computeRearClimbTargetAngle(double h_curb) const {
    double ratio = std::clamp((h_curb - 0.04) / (0.20 - 0.04), 0.0, 1.0);
    // Sweeps backward and upward into open space behind robot to avoid vertical wall collision:
    // For 5cm: -1.85 rad
    // For 10cm: -2.15 rad
    // For 20cm: -2.40 rad (sweeps cleanly above curb lip)
    return -1.85 - ratio * (2.40 - 1.85);
}

std::string TaskController::getStateName() const {
    switch (state_) {
        case ClimbState::IDLE:              return "IDLE";
        case ClimbState::FRONT_WALL_CLIMB:  return "FRONT_WALL_CLIMB";
        case ClimbState::STRADDLE_LEVELING: return "STRADDLE_LEVELING";
        case ClimbState::STRADDLE_HOLD:     return "STRADDLE_HOLD";
        case ClimbState::REAR_WALL_CLIMB:   return "REAR_WALL_CLIMB";
        case ClimbState::REAR_CREST_FOLD:   return "REAR_CREST_FOLD";
        case ClimbState::REAR_PLACED:       return "REAR_PLACED";
        default:                            return "UNKNOWN";
    }
}

void TaskController::update(
    const estimation::ContactDetector& contact_detector,
    const estimation::ClimbGeometry& geometry,
    const Eigen::Vector4d& current_ecc_angles,
    const Eigen::Vector4d& current_ecc_efforts,
    double dt,
    double body_vx,
    double cmd_vx)
{
    if (dt <= 0.0 || dt > 0.5) return;
    (void)current_ecc_efforts;

    state_timer_ += dt;

    // Clear overrides by default
    override_mask_.fill(false);
    forward_vel_override_ = 0.0;
    wheel_speed_overrides_.setZero();
    wheel_override_mask_.fill(false);

    switch (state_) {
        // ============================================================
        // 0. IDLE: Normal driving, monitoring for front wheel impact or wall-climbing
        // ============================================================
        case ClimbState::IDLE: {
            if (cooldown_timer_ > 0.0) {
                cooldown_timer_ = std::max(0.0, cooldown_timer_ - dt);
                if (cooldown_timer_ <= 0.0) {
                    has_climbed_ = false; // Auto-clear lockout after cooldown expires
                }
                break;
            }

            bool front_impact = contact_detector.frontImpactDetected();
            bool front_already_on_curb = (geometry.front_height >= 0.035) &&
                                         (geometry.delta_height >= 0.035) &&
                                         geometry.front_edge_crested && (std::abs(body_vx) > 0.02);

            if (front_impact) {
                lift_start_angles_ = current_ecc_angles;
                current_front_target_ = 0.5 * (current_ecc_angles(FL) + current_ecc_angles(FR));
                stall_jam_timer_ = 0.0;

                // Stage 1: Moderate initial climb posture (lift wheel without premature deep crouch)
                front_target_angles_(0) = 1.25;
                front_target_angles_(1) = 1.25;

                double target_r = computeRearLevelingAngle(1.25, params_.target_curb_height);
                rear_target_angles_(0) = target_r;
                rear_target_angles_(1) = target_r;

                active_lift_mask_[FL] = true;
                active_lift_mask_[FR] = true;
                active_lift_mask_[RL] = false;
                active_lift_mask_[RR] = false;
                transitionTo(ClimbState::FRONT_WALL_CLIMB);
            } else if (front_already_on_curb) {
                lift_start_angles_ = current_ecc_angles;
                current_front_target_ = 0.5 * (current_ecc_angles(FL) + current_ecc_angles(FR));
                stall_jam_timer_ = 0.0;

                double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
                double target_f = computeFrontTargetAngle(active_h);
                front_target_angles_(0) = target_f;
                front_target_angles_(1) = target_f;

                double target_r = computeRearLevelingAngle(target_f, active_h);
                rear_target_angles_(0) = target_r;
                rear_target_angles_(1) = target_r;

                active_lift_mask_[FL] = true;
                active_lift_mask_[FR] = true;
                active_lift_mask_[RL] = true;
                active_lift_mask_[RR] = true;
                transitionTo(ClimbState::STRADDLE_LEVELING);
            }
            break;
        }

        // ============================================================
        // 1. FRONT_WALL_CLIMB: Front wheels roll up wall; eccentric arms fold moderately with pitch feedback
        // ============================================================
        case ClimbState::FRONT_WALL_CLIMB: {
            // Two-Stage Front Decoupled Folding:
            // Stage 1 (Wall Climbing): Lift wheels and absorb pitch, but DO NOT deep-crouch to bottom out chassis!
            // Fold moderately up to 1.28 rad (~73 deg), ensuring belly clearance > 70mm while wheels bite the wall.
            double theta_ref = 0.080; // nominal nose-up pitch reference during climb
            double pitch_err = geometry.nose_up_pitch - theta_ref;
            double q_dot_fold = 1.2; // steady adaptive climbing rate
            if (geometry.nose_up_pitch > theta_ref) {
                q_dot_fold = 1.2 + 5.0 * pitch_err + 0.3 * geometry.nose_up_pitch_rate;
                q_dot_fold = std::clamp(q_dot_fold, 0.4, 3.0);
            }

            current_front_target_ += q_dot_fold * dt;
            current_front_target_ = std::clamp(current_front_target_,
                                               0.5 * (lift_start_angles_(FL) + lift_start_angles_(FR)),
                                               1.28); // Moderate wall-climb ceiling (NOT 1.95 rad yet!)

            // Override front legs
            override_mask_[FL] = true;
            override_mask_[FR] = true;
            ecc_overrides_(FL) = current_front_target_;
            ecc_overrides_(FR) = current_front_target_;

            // Rear legs: remain under Balance Controller control (In Tandem with Balance Controller)
            // Balance controller dynamically manages ground reaction forces and prevents backward flipping!
            override_mask_[RL] = false;
            override_mask_[RR] = false;

            // Traction drive: front wheels spin forward to grip wall, rear wheels push forward
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = 3.5; // active wall roll speed
            wheel_speed_overrides_(FR) = 3.5;
            double rear_push_vx = 0.15;
            wheel_speed_overrides_(RL) = rear_push_vx / params_.wheel_radius;
            wheel_speed_overrides_(RR) = rear_push_vx / params_.wheel_radius;
            forward_vel_override_ = rear_push_vx;

            // Dynamic Progress Watchdog (replacing rigid 2.0s timer):
            // Check if active climb progress is occurring:
            // 1. Vertical climb progress: front_z_vel > 0.010 m/s OR delta_height increasing
            // 2. Arm folding progress: current_front_target_ < 1.25 rad
            // 3. Chassis moving or operator actively pushing stick forward (cmd_vx > 0.08)
            bool active_climb_motion = (geometry.front_z_vel > 0.010) || (geometry.delta_height > 0.040);
            bool active_arm_folding = (current_front_target_ < 1.25);
            bool operator_driving = (cmd_vx > 0.08);

            if (active_climb_motion || (active_arm_folding && operator_driving)) {
                stall_jam_timer_ = 0.0; // Active progress: pet the watchdog
            } else {
                stall_jam_timer_ += dt;
            }

            // Safety abort conditions:
            // 1. Physical hard jam: zero climb progress for > 2.50s continuously while stalled
            // 2. Dangerous pitch: chassis pitch > 28 deg (0.49 rad) backward tipping hazard
            // 3. Operator reverse: cmd_vx < -0.15
            bool physical_jam = (stall_jam_timer_ >= 2.50);
            bool pitch_hazard = (geometry.nose_up_pitch > 0.49);
            bool operator_abort = (cmd_vx < -0.15);

            if (physical_jam || pitch_hazard || operator_abort) {
                cooldown_timer_ = params_.cooldown_duration;
                transitionTo(ClimbState::IDLE);
                break;
            }

            // Transition to STRADDLE_LEVELING once front wheels have crested the top edge
            // Evaluated autonomously by the tri-feature edge observer
            if (geometry.front_edge_crested) {
                double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
                double target_f = computeFrontTargetAngle(active_h);
                front_target_angles_(0) = target_f;
                front_target_angles_(1) = target_f;

                double target_r = computeRearLevelingAngle(target_f, active_h);
                rear_target_angles_(0) = target_r;
                rear_target_angles_(1) = target_r;

                travel_distance_ = 0.0;
                transitionTo(ClimbState::STRADDLE_LEVELING);
            }
            break;
        }

        // ============================================================
        // 2. STRADDLE_LEVELING: Front wheels on plateau; deep-crouch to plant chassis & skid wheels
        // ============================================================
        case ClimbState::STRADDLE_LEVELING: {
            // Operator intent check: if operator releases throttle, pause into STRADDLE_HOLD immediately
            if (cmd_vx < 0.05) {
                transitionTo(ClimbState::STRADDLE_HOLD);
                break;
            }

            double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
            double target_f_adaptive = computeFrontTargetAngle(active_h);
            front_target_angles_(0) = target_f_adaptive;
            front_target_angles_(1) = target_f_adaptive;

            // Stage 2 (Deep Crouch & Skid Landing):
            // Smoothly fold front legs from moderate climb angle (~1.25) to full adaptive crouch target_f_adaptive (1.65 ~ 1.95 rad)
            // in 0.35s, lowering the chassis so mid-rear auxiliary support wheels land firmly on the plateau.
            double crouch_progress = std::clamp(state_timer_ / 0.35, 0.0, 1.0);
            double q_f_base = (1.0 - crouch_progress) * 1.25 + crouch_progress * target_f_adaptive;
            double pitch_corr = params_.leveling_kp_pitch * std::max(0.0, geometry.nose_up_pitch);
            double q_f = std::clamp(q_f_base + pitch_corr, 1.25, 1.95);

            double q_r = computeRearLevelingAngle(q_f, active_h);
            rear_target_angles_(0) = q_r;
            rear_target_angles_(1) = q_r;

            // Active coordination on all legs
            override_mask_.fill(true);
            ecc_overrides_(FL) = q_f;
            ecc_overrides_(FR) = q_f;
            ecc_overrides_(RL) = q_r;
            ecc_overrides_(RR) = q_r;

            // Traction drive: front wheels pull forward strongly to drag chassis, rear wheels push forward
            wheel_override_mask_.fill(true);
            double pull_spd = 4.5; // rad/s (~0.45 m/s) strong traction pull
            double push_spd = 3.5;
            wheel_speed_overrides_(FL) = pull_spd;
            wheel_speed_overrides_(FR) = pull_spd;
            wheel_speed_overrides_(RL) = push_spd;
            wheel_speed_overrides_(RR) = push_spd;
            forward_vel_override_ = 0.35;

            // Integrate travel distance (only integrate true forward motion)
            double effective_vx = (body_vx > 0.02) ? body_vx : 0.0;
            travel_distance_ += effective_vx * dt;

            // Multi-cue arrival verification:
            // Cue 1: Dual-track contact detector (reaction torque / momentum arrest)
            // Cue 2: Odometry path arrival (>= 0.22m forward travel + rear slip)
            // Cue 3: Physical stall backup while driving forward
            bool rear_impact = contact_detector.rearImpactDetected();
            bool min_safe_straddle = (state_timer_ >= 0.20);
            bool odometry_arrival = (geometry.travel_distance >= 0.22 && geometry.rear_slip_ratio > 0.65);
            bool stall_arrival = (state_timer_ >= 0.60 && std::abs(body_vx) < 0.03 && geometry.travel_distance >= 0.10);

            if (min_safe_straddle && (rear_impact || odometry_arrival || stall_arrival)) {
                active_lift_mask_[RL] = true;
                active_lift_mask_[RR] = true;
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_WALL_CLIMB);
            }
            break;
        }

        // ============================================================
        // 2b. STRADDLE_HOLD: Operator paused during straddle; lock stance & balance closed-loop
        // ============================================================
        case ClimbState::STRADDLE_HOLD: {
            double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
            double target_f = computeFrontTargetAngle(active_h);
            double pitch_corr = params_.leveling_kp_pitch * std::max(0.0, geometry.nose_up_pitch);
            double q_f = std::clamp(target_f + pitch_corr, 1.55, 1.95);
            double q_r = computeRearLevelingAngle(q_f, active_h);

            override_mask_.fill(true);
            ecc_overrides_(FL) = q_f;
            ecc_overrides_(FR) = q_f;
            ecc_overrides_(RL) = q_r;
            ecc_overrides_(RR) = q_r;

            // Brake wheels during pause hold (zero velocity command, holding brakes)
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setZero();
            forward_vel_override_ = 0.0;

            // ZERO TIMEOUT: Permanent hold while paused! (Never timed out to IDLE)
            // Resume forward climbing when user presses forward
            if (cmd_vx > 0.08) {
                transitionTo(ClimbState::STRADDLE_LEVELING);
            }
            // Dismount backwards only when user commands intentional reverse
            else if (cmd_vx < -0.20) {
                cooldown_timer_ = params_.cooldown_duration;
                transitionTo(ClimbState::IDLE);
            }
            break;
        }

        // ============================================================
        // 3. REAR_WALL_CLIMB: Rear wheels climb wall with front pulling assist
        // ============================================================
        case ClimbState::REAR_WALL_CLIMB: {
            double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
            // Front legs STAY in crouch posture during rear wall climb! DO NOT unfold to 0.65!
            // This prevents pitching the chassis nose up into the sky.
            double current_front_q = front_target_angles_(0);

            // Rear legs: fast active lift sweep upward into open air behind robot (-1.85 ~ -2.40 rad)
            // Sweep time accelerated from 0.85s to 0.42s for snappy clearance!
            double q_r_target = computeRearClimbTargetAngle(active_h);
            double rear_sweep_progress = std::clamp(state_timer_ / 0.42, 0.0, 1.0);
            double current_rear_q = (1.0 - rear_sweep_progress) * rear_target_angles_(0) + rear_sweep_progress * q_r_target;

            override_mask_.fill(true);
            ecc_overrides_(FL) = current_front_q;
            ecc_overrides_(FR) = current_front_q;
            ecc_overrides_(RL) = current_rear_q;
            ecc_overrides_(RR) = current_rear_q;

            // Drive commands: all 4 wheels pull/push forward strongly to drag chassis deep into plateau
            wheel_override_mask_.fill(true);
            double pull_vx = 0.45; // Continuous strong forward pull traction
            double wheel_omega = pull_vx / params_.wheel_radius;
            wheel_speed_overrides_.setConstant(wheel_omega);
            forward_vel_override_ = pull_vx;

            double effective_vx = (body_vx > 0.02) ? body_vx : 0.0;
            travel_distance_ += effective_vx * dt;

            // Clearance check: rear sweep must complete AND robot must advance forward into plateau (>= 0.10m)
            double min_sweep_time = 0.42;
            bool sweep_completed = (rear_sweep_progress >= 0.95) && (state_timer_ >= min_sweep_time);
            bool advance_reached = (travel_distance_ >= 0.10);
            bool rear_wheels_lifted = (geometry.rear_height >= (active_h - 0.045));
            bool rear_crested = geometry.rear_edge_crested;

            bool cleared = sweep_completed && (advance_reached || rear_wheels_lifted || rear_crested || state_timer_ >= 0.90);

            if (cleared) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_CREST_FOLD);
            } else if (state_timer_ >= 4.00) { // Generous 4.0s timeout for rear wall climb
                cooldown_timer_ = params_.cooldown_duration;
                transitionTo(ClimbState::IDLE);
            }
            break;
        }

        // ============================================================
        // 4. REAR_CREST_FOLD: Rear hip cleared; fold rear legs smoothly onto plateau
        // ============================================================
        case ClimbState::REAR_CREST_FOLD: {
            double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
            wheel_override_mask_.fill(true);
            double advance_vx = 0.35;
            wheel_speed_overrides_.setConstant(advance_vx / params_.wheel_radius);
            forward_vel_override_ = advance_vx;

            double effective_vx = (std::abs(body_vx) > 0.02) ? std::abs(body_vx) : advance_vx;
            travel_distance_ += effective_vx * dt;

            // Smoothly fold rear legs toward nominal plateau cruising posture (-0.65) in 0.45s
            override_mask_.fill(true);
            double progress = std::clamp(state_timer_ / 0.45, 0.0, 1.0);
            double nominal_r = -0.65;

            double target_q_r_start = computeRearClimbTargetAngle(active_h);
            double q_r = (1.0 - progress) * target_q_r_start + progress * nominal_r;
            // Front legs STILL held in crouch posture during rear crest fold
            double q_f = front_target_angles_(0);

            ecc_overrides_(FL) = q_f;
            ecc_overrides_(FR) = q_f;
            ecc_overrides_(RL) = q_r;
            ecc_overrides_(RR) = q_r;

            if (progress >= 1.0 || state_timer_ >= 0.55) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_PLACED);
            }
            break;
        }

        // ============================================================
        // 5. REAR_PLACED: All wheels on plateau; advance 0.25m into plateau then return to IDLE
        // ============================================================
        case ClimbState::REAR_PLACED: {
            // Settle in plateau cruising posture: smoothly unfold front legs from crouch to nominal 0.65 in 0.60s
            override_mask_.fill(true);
            double unfold_progress = std::clamp(state_timer_ / 0.60, 0.0, 1.0);
            double q_f = (1.0 - unfold_progress) * front_target_angles_(0) + unfold_progress * 0.65;
            double q_r = -0.65;

            ecc_overrides_(FL) = q_f;
            ecc_overrides_(FR) = q_f;
            ecc_overrides_(RL) = q_r;
            ecc_overrides_(RR) = q_r;

            forward_vel_override_ = params_.forward_drive_speed;
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(params_.forward_drive_speed / params_.wheel_radius);

            double effective_vx = (std::abs(body_vx) > 0.02) ? std::abs(body_vx) : params_.forward_drive_speed;
            travel_distance_ += effective_vx * dt;

            // Advance by at least params_.settling_distance (0.25m) so rear wheels are deeply positioned on plateau
            bool distance_settled = travel_distance_ >= params_.settling_distance;
            bool time_settled = state_timer_ >= params_.settling_duration;

            if ((distance_settled && state_timer_ >= 0.60) || time_settled) {
                blend_out_timer_ = 0.0;
                blend_out_weight_ = 1.0;
                cooldown_timer_ = params_.cooldown_duration;
                has_climbed_ = true;
                transitionTo(ClimbState::IDLE);
            }
            break;
        }
    }

    // Blend-out fade back to balance controller
    if (state_ == ClimbState::IDLE && blend_out_weight_ > 0.0) {
        blend_out_timer_ += dt;
        double progress = blend_out_timer_ / params_.blend_out_duration;
        blend_out_weight_ = std::max(0.0, 1.0 - progress);

        if (blend_out_weight_ > 0.0) {
            override_mask_.fill(true);
        } else {
            override_mask_.fill(false);
            active_lift_mask_.fill(false);
        }
    }
}

std::array<bool, NUM_LEGS> TaskController::getContactValid() const {
    std::array<bool, NUM_LEGS> valid = {true, true, true, true};
    if (state_ == ClimbState::FRONT_WALL_CLIMB) {
        valid[FL] = false;
        valid[FR] = false;
    } else if (state_ == ClimbState::REAR_WALL_CLIMB || state_ == ClimbState::REAR_CREST_FOLD) {
        valid[RL] = false;
        valid[RR] = false;
    }
    return valid;
}

}  // namespace tasks
}  // namespace robot_control
