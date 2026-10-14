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
    current_front_target_ = 0.0;
    lift_start_angles_ = Eigen::Vector4d::Zero();
    front_target_angles_ = Eigen::Vector2d::Zero();
    rear_target_angles_ = Eigen::Vector2d::Zero();
    target_roll_bias_ = 0.0;
    has_climbed_ = false;
}

void TaskController::abort() {
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 1.0;
    target_roll_bias_ = 0.0;
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

double TaskController::computeRearSwingTrajectory(double progress, double q_start, double q_land) const {
    double s = std::clamp(progress, 0.0, 1.0);
    // Minimum-jerk cycloid smoothstep: sigma(s) = s - (1/(2*pi))*sin(2*pi*s)
    double sigma = s - (1.0 / (2.0 * M_PI)) * std::sin(2.0 * M_PI * s);
    // Unwrapped continuous sweep clockwise / over-the-top:
    // Rotates: q_start -> -pi -> +pi -> q_land
    double delta_total = (q_land - 2.0 * M_PI) - q_start;
    double q_unwrapped = q_start + delta_total * sigma;
    // Normalize to [-pi, pi]
    while (q_unwrapped > M_PI)  q_unwrapped -= 2.0 * M_PI;
    while (q_unwrapped < -M_PI) q_unwrapped += 2.0 * M_PI;
    return q_unwrapped;
}

std::string TaskController::getStateName() const {
    switch (state_) {
        case ClimbState::IDLE:                return "IDLE";
        case ClimbState::FRONT_WALL_CLIMB:    return "FRONT_WALL_CLIMB";
        case ClimbState::STRADDLE_LEVELING:   return "STRADDLE_LEVELING";
        case ClimbState::STRADDLE_HOLD:       return "STRADDLE_HOLD";
        case ClimbState::REAR_STRIDE_PREPARE: return "REAR_STRIDE_PREPARE";
        case ClimbState::REAR_SWING_LEFT:     return "REAR_SWING_LEFT";
        case ClimbState::REAR_LOAD_LEFT:      return "REAR_LOAD_LEFT";
        case ClimbState::REAR_SHIFT_LEFT:     return "REAR_SHIFT_LEFT";
        case ClimbState::REAR_SWING_RIGHT:    return "REAR_SWING_RIGHT";
        case ClimbState::REAR_LOAD_RIGHT:     return "REAR_LOAD_RIGHT";
        case ClimbState::REAR_SETTLED:        return "REAR_SETTLED";
        default:                              return "UNKNOWN";
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
            bool front_already_on_curb = (geometry.front_height >= (params_.target_curb_height - 0.035)) &&
                                         (geometry.delta_height >= (params_.target_curb_height - 0.045)) &&
                                         geometry.front_edge_crested && (std::abs(body_vx) > 0.02);

            if (front_impact) {
                lift_start_angles_ = current_ecc_angles;
                current_front_target_ = 0.5 * (current_ecc_angles(FL) + current_ecc_angles(FR));

                double target_f = computeFrontTargetAngle(params_.target_curb_height);
                front_target_angles_(0) = target_f;
                front_target_angles_(1) = target_f;

                double target_r = computeRearLevelingAngle(target_f, params_.target_curb_height);
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

                double target_f = computeFrontTargetAngle(params_.target_curb_height);
                front_target_angles_(0) = target_f;
                front_target_angles_(1) = target_f;

                double target_r = computeRearLevelingAngle(target_f, params_.target_curb_height);
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
        // 1. FRONT_WALL_CLIMB: Front wheels roll up wall; eccentric arms fold with pitch feedback
        // ============================================================
        case ClimbState::FRONT_WALL_CLIMB: {
            double h = params_.target_curb_height;
            double theta_ref = 0.060 + (h - 0.04) / (0.20 - 0.04) * (0.130 - 0.060);

            // Closed-loop pitch compliance rate:
            // When front wheels climb vertical wall, nose pitches up.
            // Front eccentric arms fold upward at matching rate to absorb height and prevent backward flipping.
            double pitch_err = geometry.nose_up_pitch - theta_ref;
            double q_dot_fold = 1.4; // steady feedforward climbing rate
            if (geometry.nose_up_pitch > theta_ref - 0.035) {
                q_dot_fold = 1.8 + 6.0 * pitch_err + 0.4 * geometry.nose_up_pitch_rate;
                q_dot_fold = std::clamp(q_dot_fold, 0.5, 3.5);
            }

            current_front_target_ += q_dot_fold * dt;
            current_front_target_ = std::clamp(current_front_target_,
                                               0.5 * (lift_start_angles_(FL) + lift_start_angles_(FR)),
                                               front_target_angles_(0));

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
            wheel_speed_overrides_(FL) = params_.front_roll_speed;
            wheel_speed_overrides_(FR) = params_.front_roll_speed;
            double rear_push_vx = 0.15;
            wheel_speed_overrides_(RL) = rear_push_vx / params_.wheel_radius;
            wheel_speed_overrides_(RR) = rear_push_vx / params_.wheel_radius;
            forward_vel_override_ = rear_push_vx;

            // Early sanity check: if climbing was triggered falsely on flat ground,
            // pitch won't rise within 0.40s and delta_height will remain near zero.
            // Abort immediately back to IDLE with cooldown!
            if (state_timer_ >= 0.40 && geometry.nose_up_pitch < 0.02 && geometry.delta_height < 0.015) {
                cooldown_timer_ = params_.cooldown_duration;
                transitionTo(ClimbState::IDLE);
                break;
            }

            // Check if front wheels have crested curb edge
            bool height_reached = geometry.delta_height >= (params_.target_curb_height - 0.035);
            bool target_reached = current_front_target_ >= (front_target_angles_(0) - 0.10);
            bool front_crested = (height_reached && target_reached) || geometry.front_edge_crested;
            double max_front_climb_time = (h >= 0.15) ? 2.5 : 2.0;

            if (front_crested) {
                double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
                double target_f = computeFrontTargetAngle(active_h);
                front_target_angles_(0) = target_f;
                front_target_angles_(1) = target_f;

                double target_r = computeRearLevelingAngle(target_f, active_h);
                rear_target_angles_(0) = target_r;
                rear_target_angles_(1) = target_r;

                transitionTo(ClimbState::STRADDLE_LEVELING);
            } else if (state_timer_ >= max_front_climb_time) {
                // If front wheels never gained height, this was an abortive attempt or flat ground bump.
                // Abort safely back to IDLE with cooldown!
                cooldown_timer_ = params_.cooldown_duration;
                transitionTo(ClimbState::IDLE);
            }
            break;
        }

        // ============================================================
        // 2. STRADDLE_LEVELING: Front wheels on top of curb; active closed-loop leveling & advance
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

            // Two-stage front crouch:
            // Stage 1: Safe entry buffer (first 0.06m or 0.25s) keeps moderate 1.60 rad fold
            // Stage 2: Adaptive deep crouch + Pitch leveling compliance, strictly capped at 1.95 rad to prevent bottoming out!
            double q_f_base = (travel_distance_ < 0.06 && state_timer_ < 0.25) ? 1.60 : target_f_adaptive;
            double pitch_corr = params_.leveling_kp_pitch * std::max(0.0, geometry.nose_up_pitch);
            double q_f = std::clamp(q_f_base + pitch_corr, 1.55, 1.95);

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
                transitionTo(ClimbState::REAR_STRIDE_PREPARE);
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

            // Resume forward climbing when user presses forward
            if (cmd_vx > 0.08) {
                transitionTo(ClimbState::STRADDLE_LEVELING);
            }
            // Dismount backwards when user commands reverse
            else if (cmd_vx < -0.15) {
                cooldown_timer_ = params_.cooldown_duration;
                transitionTo(ClimbState::IDLE);
            }
            break;
        }

        // ============================================================
        // 3. REAR_STRIDE_PREPARE: Anchor RR support leg (q=0), shift CoM right (roll -> -3.5°)
        // ============================================================
        case ClimbState::REAR_STRIDE_PREPARE: {
            double current_front_q = front_target_angles_(0);

            // Front legs maintain plateau support
            ecc_overrides_(FL) = current_front_q;
            ecc_overrides_(FR) = current_front_q;
            // RR is the vertical ground support pillar (q=0 rad, maximum height & normal force)
            ecc_overrides_(RR) = 0.0;
            // RL begins at pre-swing ground position
            ecc_overrides_(RL) = -0.35;
            override_mask_.fill(true);

            // Pre-shift CoM to the right toward RR (roll -> -3.5 deg = -0.061 rad)
            target_roll_bias_ = -0.061;

            // Traction drive: slow forward traction
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = 0.20 / params_.wheel_radius;
            wheel_speed_overrides_(FR) = 0.20 / params_.wheel_radius;
            wheel_speed_overrides_(RL) = 0.10 / params_.wheel_radius;
            wheel_speed_overrides_(RR) = 0.10 / params_.wheel_radius;
            forward_vel_override_ = 0.15;

            // Stabilization wait: 0.20s
            if (state_timer_ >= 0.20) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_SWING_LEFT);
            }
            break;
        }

        // ============================================================
        // 4. REAR_SWING_LEFT: RL swing leg sweeps along cycloid arc over step lip onto plateau
        // ============================================================
        case ClimbState::REAR_SWING_LEFT: {
            double current_front_q = front_target_angles_(0);
            ecc_overrides_(FL) = current_front_q;
            ecc_overrides_(FR) = current_front_q;
            ecc_overrides_(RR) = 0.0; // RR maintains rigid vertical support

            // RL executes minimum-jerk cycloid swing trajectory (T = 0.45s)
            // Sweeps: -0.35 rad -> -pi -> +pi -> +1.15 rad
            double s = std::clamp(state_timer_ / 0.45, 0.0, 1.0);
            ecc_overrides_(RL) = computeRearSwingTrajectory(s, -0.35, 1.15);
            override_mask_.fill(true);

            target_roll_bias_ = -0.061;

            // Drive: FL & FR pull forward strongly with traction
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = 0.25 / params_.wheel_radius;
            wheel_speed_overrides_(FR) = 0.25 / params_.wheel_radius;
            wheel_speed_overrides_(RR) = 0.10 / params_.wheel_radius;
            wheel_speed_overrides_(RL) = 0.25 / params_.wheel_radius;
            forward_vel_override_ = 0.20;

            // Touchdown detection: once swing is descending (s >= 0.70), check for load acceptance
            bool touchdown = (s >= 0.70 && contact_detector.isLegTouchdown(RL));
            if (touchdown || state_timer_ >= 0.55) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_LOAD_LEFT);
            }
            break;
        }

        // ============================================================
        // 5. REAR_LOAD_LEFT: RL confirms touchdown, wheel speed synchronizes, outputs forward push
        // ============================================================
        case ClimbState::REAR_LOAD_LEFT: {
            double current_front_q = front_target_angles_(0);
            ecc_overrides_(FL) = current_front_q;
            ecc_overrides_(FR) = current_front_q;
            ecc_overrides_(RL) = 1.15; // RL firmly holds step position
            ecc_overrides_(RR) = 0.0;  // RR holds vertical ground support
            override_mask_.fill(true);

            target_roll_bias_ = -0.061;

            // Drive: RL wheel matches front wheels, pushing forward!
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = 0.25 / params_.wheel_radius;
            wheel_speed_overrides_(FR) = 0.25 / params_.wheel_radius;
            wheel_speed_overrides_(RL) = 0.25 / params_.wheel_radius;
            wheel_speed_overrides_(RR) = 0.10 / params_.wheel_radius;
            forward_vel_override_ = 0.20;

            if (state_timer_ >= 0.18) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_SHIFT_LEFT);
            }
            break;
        }

        // ============================================================
        // 6. REAR_SHIFT_LEFT: Shift CoM left (roll -> +3.5°) so △(FL,FR,RL) takes full weight
        // ============================================================
        case ClimbState::REAR_SHIFT_LEFT: {
            double current_front_q = front_target_angles_(0);
            ecc_overrides_(FL) = current_front_q;
            ecc_overrides_(FR) = current_front_q;
            ecc_overrides_(RL) = 1.15;
            ecc_overrides_(RR) = 0.0;
            override_mask_.fill(true);

            // Smooth roll transition from -0.061 to +0.061 rad (+3.5 deg) in 0.25s
            double progress = std::clamp(state_timer_ / 0.25, 0.0, 1.0);
            target_roll_bias_ = -0.061 + progress * (0.061 - (-0.061));

            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = 0.25 / params_.wheel_radius;
            wheel_speed_overrides_(FR) = 0.25 / params_.wheel_radius;
            wheel_speed_overrides_(RL) = 0.25 / params_.wheel_radius;
            wheel_speed_overrides_(RR) = 0.05 / params_.wheel_radius;
            forward_vel_override_ = 0.20;

            if (state_timer_ >= 0.25) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_SWING_RIGHT);
            }
            break;
        }

        // ============================================================
        // 7. REAR_SWING_RIGHT: RR swing leg sweeps along cycloid arc onto plateau
        // ============================================================
        case ClimbState::REAR_SWING_RIGHT: {
            double current_front_q = front_target_angles_(0);
            ecc_overrides_(FL) = current_front_q;
            ecc_overrides_(FR) = current_front_q;
            ecc_overrides_(RL) = 1.15; // RL on plateau supports body

            // RR executes cycloid swing trajectory (T = 0.45s) from 0.0 -> -pi -> +pi -> 1.15 rad
            double s = std::clamp(state_timer_ / 0.45, 0.0, 1.0);
            ecc_overrides_(RR) = computeRearSwingTrajectory(s, 0.0, 1.15);
            override_mask_.fill(true);

            target_roll_bias_ = +0.061;

            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(0.25 / params_.wheel_radius);
            forward_vel_override_ = 0.20;

            bool touchdown = (s >= 0.70 && contact_detector.isLegTouchdown(RR));
            if (touchdown || state_timer_ >= 0.55) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_LOAD_RIGHT);
            }
            break;
        }

        // ============================================================
        // 8. REAR_LOAD_RIGHT: All 4 wheels on plateau, roll bias fades to 0
        // ============================================================
        case ClimbState::REAR_LOAD_RIGHT: {
            double current_front_q = front_target_angles_(0);
            ecc_overrides_(FL) = current_front_q;
            ecc_overrides_(FR) = current_front_q;
            ecc_overrides_(RL) = 1.15;
            ecc_overrides_(RR) = 1.15;
            override_mask_.fill(true);

            // Roll bias fades to zero
            double progress = std::clamp(state_timer_ / 0.15, 0.0, 1.0);
            target_roll_bias_ = (1.0 - progress) * 0.061;

            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(0.25 / params_.wheel_radius);
            forward_vel_override_ = 0.20;

            if (state_timer_ >= 0.15) {
                target_roll_bias_ = 0.0;
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_SETTLED);
            }
            break;
        }

        // ============================================================
        // 9. REAR_SETTLED: Roll/pitch zero out, advance 0.25m into plateau, release to IDLE
        // ============================================================
        case ClimbState::REAR_SETTLED: {
            target_roll_bias_ = 0.0;
            override_mask_.fill(true);

            // Settle in plateau cruising posture:
            // Front legs smoothly unfold from crouch to nominal 0.65 in 0.60s
            // Rear legs smoothly rotate from +1.15 through bottom (0) to nominal -0.65 in 0.60s
            double unfold_progress = std::clamp(state_timer_ / 0.60, 0.0, 1.0);
            double q_f = (1.0 - unfold_progress) * front_target_angles_(0) + unfold_progress * 0.65;
            double q_r = (1.0 - unfold_progress) * 1.15 + unfold_progress * (-0.65);

            ecc_overrides_(FL) = q_f;
            ecc_overrides_(FR) = q_f;
            ecc_overrides_(RL) = q_r;
            ecc_overrides_(RR) = q_r;

            forward_vel_override_ = params_.forward_drive_speed;
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(params_.forward_drive_speed / params_.wheel_radius);

            double effective_vx = (std::abs(body_vx) > 0.02) ? std::abs(body_vx) : params_.forward_drive_speed;
            travel_distance_ += effective_vx * dt;

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
        target_roll_bias_ = 0.0;
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
    } else if (state_ == ClimbState::REAR_SWING_LEFT) {
        valid[RL] = false;
    } else if (state_ == ClimbState::REAR_SWING_RIGHT) {
        valid[RR] = false;
    }
    return valid;
}

}  // namespace tasks
}  // namespace robot_control
