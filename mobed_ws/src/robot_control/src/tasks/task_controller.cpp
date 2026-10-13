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

double TaskController::computeFrontTargetAngle(double h_curb) const {
    double ratio = std::clamp((h_curb - 0.04) / (0.20 - 0.04), 0.0, 1.0);
    // Maps [0.04m, 0.20m] smoothly to [1.45 rad, 2.60 rad] (149°, safely below 180° singularity)
    return 1.45 + ratio * (2.60 - 1.45);
}

double TaskController::computeAdaptiveLiftAngle([[maybe_unused]] double q_start) const {
    return computeFrontTargetAngle(params_.target_curb_height);
}

double TaskController::computeRearLevelingAngle([[maybe_unused]] double q_front_avg, double h_curb) const {
    double ratio = std::clamp((h_curb - 0.04) / (0.20 - 0.04), 0.0, 1.0);
    // Smooth transition from -0.55 rad (gentle 40mm) to -0.21 rad (extreme 200mm high clearance)
    return -0.55 + ratio * (-0.21 - (-0.55));
}

std::string TaskController::getStateName() const {
    switch (state_) {
        case ClimbState::IDLE:              return "IDLE";
        case ClimbState::FRONT_WALL_CLIMB:  return "FRONT_WALL_CLIMB";
        case ClimbState::STRADDLE_LEVELING: return "STRADDLE_LEVELING";
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
    double body_vx)
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

            if (contact_detector.frontImpactDetected()) {
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
            double q_dot_fold = 0.8; // steady feedforward climbing rate
            if (geometry.nose_up_pitch > theta_ref - 0.035) {
                q_dot_fold = 1.2 + 6.0 * pitch_err + 0.4 * geometry.nose_up_pitch_rate;
                q_dot_fold = std::clamp(q_dot_fold, 0.0, 3.5);
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

            // Rear legs active leveling: extend downward (-0.21 rad to -0.55 rad) to raise rear hip
            override_mask_[RL] = true;
            override_mask_[RR] = true;
            ecc_overrides_(RL) = rear_target_angles_(0);
            ecc_overrides_(RR) = rear_target_angles_(1);

            // Traction drive: front wheels spin forward to grip wall, rear wheels push forward
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = params_.front_roll_speed;
            wheel_speed_overrides_(FR) = params_.front_roll_speed;
            double rear_push_vx = 0.15;
            wheel_speed_overrides_(RL) = rear_push_vx / params_.wheel_radius;
            wheel_speed_overrides_(RR) = rear_push_vx / params_.wheel_radius;
            forward_vel_override_ = rear_push_vx;

            // Check if front wheels have crested curb edge
            bool height_reached = geometry.delta_height >= (params_.target_curb_height - 0.025);
            bool target_reached = current_front_target_ >= (front_target_angles_(0) - 0.08);
            bool front_crested = (height_reached && target_reached) || geometry.front_edge_crested;
            double max_front_climb_time = (h >= 0.15) ? 1.6 : 1.2;

            if (front_crested || state_timer_ >= max_front_climb_time) {
                transitionTo(ClimbState::STRADDLE_LEVELING);
            }
            break;
        }

        // ============================================================
        // 2. STRADDLE_LEVELING: Front wheels on top of curb; adaptive near-level differential advance
        // ============================================================
        case ClimbState::STRADDLE_LEVELING: {
            // Front legs held on top of curb
            override_mask_[FL] = true;
            override_mask_[FR] = true;
            ecc_overrides_(FL) = front_target_angles_(0);
            ecc_overrides_(FR) = front_target_angles_(1);

            // Rear legs maintained in near-level extended stance for ample belly clearance (>21mm)
            override_mask_[RL] = true;
            override_mask_[RR] = true;
            ecc_overrides_(RL) = rear_target_angles_(0);
            ecc_overrides_(RR) = rear_target_angles_(1);

            // Command forward drive to advance the body toward the curb
            forward_vel_override_ = params_.forward_drive_speed;
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(params_.forward_drive_speed / params_.wheel_radius);

            // Integrate travel distance
            double effective_vx = (std::abs(body_vx) > 0.02) ? std::abs(body_vx) : params_.forward_drive_speed;
            travel_distance_ += effective_vx * dt;

            // Distance to rear curb arrival:
            // Chassis wheelbase is 0.30m, eccentric offset difference brings rear wheels to face after ~0.35m
            double advance_to_rear_curb = 0.35;
            bool in_rear_contact_window = (travel_distance_ >= 0.22);

            bool rear_arrived = (in_rear_contact_window && contact_detector.rearImpactDetected()) ||
                                (travel_distance_ >= advance_to_rear_curb) ||
                                (in_rear_contact_window && geometry.rear_height > 0.030) ||
                                (contact_detector.rearImpactDetected() && state_timer_ >= 0.8);

            // Anti-stall traction assist: ramp forward drive if advance takes longer than 2.0s
            if (state_timer_ > 2.0) {
                forward_vel_override_ = params_.forward_drive_speed * 1.25;
                wheel_speed_overrides_.setConstant(forward_vel_override_ / params_.wheel_radius);
            }

            if (rear_arrived) {
                active_lift_mask_[RL] = true;
                active_lift_mask_[RR] = true;
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_WALL_CLIMB);
            } else if (state_timer_ >= 8.0) {
                // Safety abort after prolonged stall: never lock has_climbed_ so it can re-trigger
                cooldown_timer_ = params_.cooldown_duration;
                transitionTo(ClimbState::IDLE);
            }
            break;
        }

        // ============================================================
        // 3. REAR_WALL_CLIMB: Rear wheels climb wall with front pulling assist
        // ============================================================
        case ClimbState::REAR_WALL_CLIMB: {
            // Front wheels pull forward on top of plateau
            override_mask_[FL] = true;
            override_mask_[FR] = true;
            ecc_overrides_(FL) = front_target_angles_(0);
            ecc_overrides_(FR) = front_target_angles_(1);

            // Rear legs: active lift sweep upward
            override_mask_[RL] = true;
            override_mask_[RR] = true;
            double q_r_target = -front_target_angles_(0);
            double rear_sweep_progress = std::clamp(state_timer_ / 0.60, 0.0, 1.0);
            double current_rear_q = (1.0 - rear_sweep_progress) * rear_target_angles_(0) + rear_sweep_progress * q_r_target;
            ecc_overrides_(RL) = current_rear_q;
            ecc_overrides_(RR) = current_rear_q;

            // Drive commands: front wheels pull forward strongly, rear wheels assist wall climbing
            wheel_override_mask_.fill(true);
            double pull_vx = 0.20;
            wheel_speed_overrides_(FL) = pull_vx / params_.wheel_radius;
            wheel_speed_overrides_(FR) = pull_vx / params_.wheel_radius;
            wheel_speed_overrides_(RL) = params_.rear_roll_speed;
            wheel_speed_overrides_(RR) = params_.rear_roll_speed;
            forward_vel_override_ = pull_vx;

            // Clearance check
            bool cleared = false;
            if (params_.target_curb_height >= 0.15) {
                cleared = geometry.rear_hip_cleared || (geometry.nose_up_pitch <= 0.05 && state_timer_ >= 0.40) || (state_timer_ >= 0.70);
            } else {
                cleared = (geometry.rear_height >= (params_.target_curb_height - 0.025)) || geometry.rear_edge_crested || (state_timer_ >= 0.60);
            }

            if (cleared || state_timer_ >= 0.95) {
                transitionTo(ClimbState::REAR_CREST_FOLD);
            }
            break;
        }

        // ============================================================
        // 4. REAR_CREST_FOLD: Rear hip cleared; fold rear legs smoothly onto plateau
        // ============================================================
        case ClimbState::REAR_CREST_FOLD: {
            wheel_override_mask_.fill(true);
            double advance_vx = 0.20;
            wheel_speed_overrides_.setConstant(advance_vx / params_.wheel_radius);
            forward_vel_override_ = advance_vx;

            // Smoothly fold all legs toward nominal plateau cruising posture (FL/FR: +0.65, RL/RR: -0.65)
            override_mask_.fill(true);
            double progress = std::clamp(state_timer_ / 0.50, 0.0, 1.0);
            double nominal_f = 0.65;
            double nominal_r = -0.65;
            double q_f = (1.0 - progress) * front_target_angles_(0) + progress * nominal_f;
            double q_r = (1.0 - progress) * (-front_target_angles_(0)) + progress * nominal_r;
            ecc_overrides_(FL) = q_f;
            ecc_overrides_(FR) = q_f;
            ecc_overrides_(RL) = q_r;
            ecc_overrides_(RR) = q_r;

            if (progress >= 1.0 || state_timer_ >= 0.60) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_PLACED);
            }
            break;
        }

        // ============================================================
        // 5. REAR_PLACED: All wheels on plateau; advance 0.25m into plateau then return to IDLE
        // ============================================================
        case ClimbState::REAR_PLACED: {
            // Settle in plateau cruising posture
            override_mask_.fill(true);
            ecc_overrides_(FL) = 0.65;
            ecc_overrides_(FR) = 0.65;
            ecc_overrides_(RL) = -0.65;
            ecc_overrides_(RR) = -0.65;

            forward_vel_override_ = params_.forward_drive_speed;
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(params_.forward_drive_speed / params_.wheel_radius);

            double effective_vx = (std::abs(body_vx) > 0.02) ? std::abs(body_vx) : params_.forward_drive_speed;
            travel_distance_ += effective_vx * dt;

            // Advance by at least params_.settling_distance (0.25m) so rear wheels are deeply positioned on plateau
            bool distance_settled = travel_distance_ >= params_.settling_distance;
            bool time_settled = state_timer_ >= params_.settling_duration;

            if (distance_settled || time_settled) {
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
