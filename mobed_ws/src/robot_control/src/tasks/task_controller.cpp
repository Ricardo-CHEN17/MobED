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

double TaskController::computeAdaptiveLiftAngle([[maybe_unused]] double q_start) const {
    if (params_.climb_mode == ClimbMode::HIGH_200MM || params_.target_curb_height >= 0.18) {
        return 2.80;
    }
    return 1.65;
}

double TaskController::computeRearLevelingAngle(double q_front_avg, double h_curb) const {
    const double l_ecc = 0.075;
    double cos_target = std::cos(q_front_avg) + (h_curb / l_ecc);
    if (cos_target >= std::cos(MIN_SINGULARITY_INCLINATION)) {
        return -MIN_SINGULARITY_INCLINATION;
    }
    cos_target = std::clamp(cos_target, -1.0, 1.0);
    return -std::acos(cos_target);
}

std::string TaskController::getStateName() const {
    switch (state_) {
        case ClimbState::IDLE:              return "IDLE";
        case ClimbState::FRONT_WALL_CLIMB:  return "FRONT_WALL_CLIMB";
        case ClimbState::FRONT_PLACED:      return "FRONT_PLACED";
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
            if (has_climbed_) {
                break;
            }

            if (cooldown_timer_ > 0.0) {
                cooldown_timer_ = std::max(0.0, cooldown_timer_ - dt);
                break;
            }

            if (contact_detector.frontImpactDetected()) {
                lift_start_angles_ = current_ecc_angles;
                current_front_target_ = 0.5 * (current_ecc_angles(FL) + current_ecc_angles(FR));

                double target_f = 2.80;
                front_target_angles_(0) = target_f;
                front_target_angles_(1) = target_f;

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
            bool is_high_mode = (params_.climb_mode == ClimbMode::HIGH_200MM || params_.target_curb_height >= 0.18);
            double theta_ref = is_high_mode ? 0.130 : 0.070; // 7.5 deg for 200mm, 4.0 deg for 100mm

            // Closed-loop pitch compliance rate:
            // When front wheels climb vertical wall, nose pitches up (geometry.nose_up_pitch > theta_ref).
            // Front eccentric arms fold upward at a matching rate to absorb the height and prevent backward flipping.
            double pitch_err = geometry.nose_up_pitch - theta_ref;
            double q_dot_fold = 0.0;
            if (geometry.nose_up_pitch > theta_ref - 0.035) {
                q_dot_fold = 1.2 + 6.0 * pitch_err + 0.4 * geometry.nose_up_pitch_rate;
                q_dot_fold = std::clamp(q_dot_fold, 0.0, 3.0);
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

            // Rear legs active leveling: extend downward (-0.12 rad) to raise rear of chassis
            override_mask_[RL] = true;
            override_mask_[RR] = true;
            ecc_overrides_(RL) = -MIN_SINGULARITY_INCLINATION;
            ecc_overrides_(RR) = -MIN_SINGULARITY_INCLINATION;

            // Traction drive: front wheels spin forward to grip wall, rear wheels push forward
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = params_.front_roll_speed;
            wheel_speed_overrides_(FR) = params_.front_roll_speed;
            double rear_push_vx = 0.15;
            wheel_speed_overrides_(RL) = rear_push_vx / 0.065;
            wheel_speed_overrides_(RR) = rear_push_vx / 0.065;
            forward_vel_override_ = rear_push_vx;

            // Check if front wheels have crested the curb edge
            bool height_reached = geometry.delta_height >= (params_.target_curb_height - 0.025);
            bool target_reached = current_front_target_ >= (front_target_angles_(0) - 0.08);
            bool front_crested = (height_reached && target_reached) || geometry.front_edge_crested;

            if (front_crested || state_timer_ >= 1.2) {
                transitionTo(ClimbState::FRONT_PLACED);
            }
            break;
        }

        // ============================================================
        // 2. FRONT_PLACED: Front wheels on top of curb; full-stride forward advance
        // ============================================================
        case ClimbState::FRONT_PLACED: {
            override_mask_[FL] = true;
            override_mask_[FR] = true;
            ecc_overrides_(FL) = front_target_angles_(0);
            ecc_overrides_(FR) = front_target_angles_(1);

            // Maintain rear legs extended (-0.12 rad) for ample belly clearance
            override_mask_[RL] = true;
            override_mask_[RR] = true;
            ecc_overrides_(RL) = -MIN_SINGULARITY_INCLINATION;
            ecc_overrides_(RR) = -MIN_SINGULARITY_INCLINATION;

            // Command forward drive to advance the body toward the curb
            forward_vel_override_ = params_.forward_drive_speed;
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(params_.forward_drive_speed / 0.065);

            // Integrate travel distance
            double effective_vx = (std::abs(body_vx) > 0.02) ? std::abs(body_vx) : params_.forward_drive_speed;
            travel_distance_ += effective_vx * dt;

            // Distance from rear wheels to curb face after front wheels are placed:
            // Mode 1: rear wheels start ~0.50m from curb, needing ~0.52m advance
            // Mode 2: body already advanced partially on high step crest, needing ~0.38m advance
            double advance_to_rear_curb = 0.38;
            bool in_rear_contact_window = (travel_distance_ >= 0.28);

            // Rear wall arrival check (multi-modal: impact detection, physical displacement, or height rise):
            // 1. Direct rear impact detected by contact detector
            // 2. Kinematic travel distance brings rear wheels to the curb face
            // 3. Rear wheel contact height rises within contact window
            // 4. Contact detector registers persistent rear wall collision after initial advance
            bool rear_arrived = (in_rear_contact_window && contact_detector.rearImpactDetected()) ||
                                (travel_distance_ >= advance_to_rear_curb) ||
                                (in_rear_contact_window && geometry.rear_height > 0.030) ||
                                (contact_detector.rearImpactDetected() && state_timer_ >= 0.8);

            // Anti-stall traction assist: if advance is taking longer than 2.0s, slightly ramp forward drive
            if (state_timer_ > 2.0) {
                forward_vel_override_ = params_.forward_drive_speed * 1.25;
                wheel_speed_overrides_.setConstant(forward_vel_override_ / 0.065);
            }

            if (rear_arrived) {
                active_lift_mask_[RL] = true;
                active_lift_mask_[RR] = true;
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_WALL_CLIMB);
            } else if (state_timer_ >= 8.0) {
                // Safety abort after prolonged stall:
                // NEVER set has_climbed_ = true here, so that climbing is not falsely locked out!
                cooldown_timer_ = params_.cooldown_duration;
                transitionTo(ClimbState::IDLE);
            }
            break;
        }

        // ============================================================
        // 3. REAR_WALL_CLIMB (Stage R1): Rear legs LOCKED at -0.12 rad, rear wheels climb wall
        // ============================================================
        case ClimbState::REAR_WALL_CLIMB: {
            // Front wheels pull forward on top of plateau
            override_mask_[FL] = true;
            override_mask_[FR] = true;
            ecc_overrides_(FL) = front_target_angles_(0);
            ecc_overrides_(FR) = front_target_angles_(1);

            // Stage R1: Keep rear legs LOCKED at -0.12 rad so steering columns stay 56mm behind curb wall
            override_mask_[RL] = true;
            override_mask_[RR] = true;
            ecc_overrides_(RL) = -MIN_SINGULARITY_INCLINATION;
            ecc_overrides_(RR) = -MIN_SINGULARITY_INCLINATION;

            // Drive commands: front wheels haul forward, rear wheels climb wall
            wheel_override_mask_.fill(true);
            double pull_vx = 0.20;
            wheel_speed_overrides_(FL) = pull_vx / 0.065;
            wheel_speed_overrides_(FR) = pull_vx / 0.065;
            wheel_speed_overrides_(RL) = params_.rear_roll_speed;
            wheel_speed_overrides_(RR) = params_.rear_roll_speed;
            forward_vel_override_ = pull_vx;

            // Clearance check:
            // In Mode 1 (100mm): rear wheels climb by 100mm = rw, takes ~0.25s
            // In Mode 2 (200mm): rear hip must clear curb edge (rear_hip_cleared == true or pitch levelled)
            bool is_high_mode = (params_.climb_mode == ClimbMode::HIGH_200MM || params_.target_curb_height >= 0.18);
            bool cleared = false;
            if (is_high_mode) {
                cleared = geometry.rear_hip_cleared || (geometry.nose_up_pitch <= 0.05 && state_timer_ >= 0.25);
            } else {
                cleared = (geometry.rear_height >= 0.06) || geometry.rear_edge_crested || (state_timer_ >= 0.65);
            }

            if (cleared || state_timer_ >= 0.90) {
                transitionTo(ClimbState::REAR_CREST_FOLD);
            }
            break;
        }

        // ============================================================
        // 4. REAR_CREST_FOLD (Stage R2): Rear hip cleared; fold rear legs smoothly onto plateau
        // ============================================================
        case ClimbState::REAR_CREST_FOLD: {
            wheel_override_mask_.fill(true);
            double advance_vx = 0.20;
            wheel_speed_overrides_.setConstant(advance_vx / 0.065);
            forward_vel_override_ = advance_vx;

            override_mask_.fill(true);
            ecc_overrides_(FL) = 2.80;
            ecc_overrides_(FR) = 2.80;
            double progress = std::clamp(state_timer_ / 0.50, 0.0, 1.0);
            double q_r = (1.0 - progress) * (-MIN_SINGULARITY_INCLINATION) + progress * (-2.80);
            ecc_overrides_(RL) = q_r;
            ecc_overrides_(RR) = q_r;

            if (progress >= 1.0 || state_timer_ >= 0.60) {
                transitionTo(ClimbState::REAR_PLACED);
            }
            break;
        }

        // ============================================================
        // 5. REAR_PLACED: All wheels on plateau; brief settling advance then return to IDLE
        // ============================================================
        case ClimbState::REAR_PLACED: {
            override_mask_.fill(true);
            ecc_overrides_(FL) = 2.80;
            ecc_overrides_(FR) = 2.80;
            ecc_overrides_(RL) = -2.80;
            ecc_overrides_(RR) = -2.80;

            forward_vel_override_ = params_.forward_drive_speed;
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(params_.forward_drive_speed / 0.065);

            if (state_timer_ >= params_.settling_duration) {
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
