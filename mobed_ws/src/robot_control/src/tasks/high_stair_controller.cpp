#include "robot_control/tasks/high_stair_controller.hpp"
#include <cmath>
#include <algorithm>

namespace robot_control {
namespace tasks {

HighStairController::HighStairController(const HighStairParams& params)
    : params_(params)
{
    reset();
}

void HighStairController::reset() {
    state_ = HighStairState::IDLE;
    pre_hold_state_ = HighStairState::IDLE;
    ecc_overrides_.setZero();
    override_mask_.fill(false);
    wheel_speed_overrides_.setZero();
    wheel_override_mask_.fill(false);
    forward_vel_override_ = 0.0;
    state_timer_ = 0.0;
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 0.0;
    advance_distance_ = 0.0;
    front_flip_angle_ = 0.0;
    rear_flip_angle_ = 0.0;
    front_climb_retry_count_ = 0;
    front_stall_timer_ = 0.0;
    base_standing_angles_.setZero();
    hold_ecc_angles_.setZero();
}

void HighStairController::abort() {
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 1.0;
    forward_vel_override_ = 0.0;
    wheel_speed_overrides_.setZero();
    wheel_override_mask_.fill(false);
    override_mask_.fill(false);
    front_stall_timer_ = 0.0;
    transitionTo(HighStairState::IDLE);
}

void HighStairController::trigger(const Eigen::Vector4d& current_ecc_angles) {
    if (state_ == HighStairState::IDLE) {
        base_standing_angles_ = current_ecc_angles;
        front_climb_retry_count_ = 0;
        transitionTo(HighStairState::F_PRELOAD);
    }
}

void HighStairController::transitionTo(HighStairState new_state) {
    state_ = new_state;
    state_timer_ = 0.0;
}

std::string HighStairController::getStateName() const {
    switch (state_) {
        case HighStairState::IDLE:         return "IDLE";
        case HighStairState::HOLD:         return "HOLD";
        case HighStairState::F_PRELOAD:    return "F_PRELOAD";
        case HighStairState::F_WALL_ROLL:  return "F_WALL_ROLL";
        case HighStairState::F_LAND:       return "F_LAND";
        case HighStairState::STRADDLE:     return "STRADDLE";
        case HighStairState::R_PRELOAD:    return "R_PRELOAD";
        case HighStairState::R_WALL_ROLL:  return "R_WALL_ROLL";
        case HighStairState::R_LAND:       return "R_LAND";
        case HighStairState::RESET_SETTLE: return "RESET_SETTLE";
        default:                           return "UNKNOWN";
    }
}

void HighStairController::update(
    const estimation::ContactDetector& contact_detector,
    const estimation::ClimbGeometry& geometry,
    const Eigen::Vector4d& current_ecc_angles,
    const Eigen::Vector4d& current_ecc_efforts,
    const Eigen::Vector4d& current_wheel_efforts,
    const Eigen::Vector4d& current_wheel_vels,
    double user_cmd_vx,
    bool mode_enabled,
    double dt,
    double body_vx,
    double body_pitch)
{
    if (dt <= 0.0 || dt > 0.5) return;
    (void)current_ecc_efforts;

    state_timer_ += dt;
    forward_vel_override_ = 0.0;

    // Mode guard: abort if mode disabled
    if (!mode_enabled && state_ != HighStairState::IDLE) {
        abort();
        return;
    }

    double r_w = params_.wheel_radius > 1e-4 ? params_.wheel_radius : 0.10;
    bool user_pushing_forward = (user_cmd_vx > 0.05);
    bool user_backing_off = (user_cmd_vx < -0.05);

    // Dead-man's switch logic:
    // If user releases forward drive while actively climbing -> HOLD
    if (state_ != HighStairState::IDLE && state_ != HighStairState::HOLD) {
        if (!user_pushing_forward) {
            pre_hold_state_ = state_;
            hold_ecc_angles_ = current_ecc_angles;
            override_mask_.fill(true);
            ecc_overrides_ = hold_ecc_angles_;
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setZero();
            forward_vel_override_ = 0.0;
            transitionTo(HighStairState::HOLD);
            return;
        }
    }

    // If user backs off before straddle, cleanly retreat and regain full teleop
    if (user_backing_off) {
        if (state_ == HighStairState::F_PRELOAD || state_ == HighStairState::F_WALL_ROLL) {
            abort();
            return;
        }
        if (state_ == HighStairState::HOLD &&
            (pre_hold_state_ == HighStairState::F_PRELOAD || pre_hold_state_ == HighStairState::F_WALL_ROLL)) {
            abort();
            return;
        }
    }

    switch (state_) {
        // ============================================================
        // 0. IDLE: Completely manual, monitoring for forward obstacle collision
        // ============================================================
        case HighStairState::IDLE: {
            override_mask_.fill(false);
            wheel_override_mask_.fill(false);
            forward_vel_override_ = 0.0;

            if (mode_enabled && user_pushing_forward) {
                // Front impact shock transient from ContactDetector
                bool front_impact = contact_detector.frontImpactDetected();

                // Physical stall detection against vertical obstacle face:
                // Front wheels must be stalled (|omega| < stall_vel_threshold) under high resistance torque (> stall_torque_threshold),
                // while actively commanded forward and chassis blocked (< 0.06 m/s).
                bool fl_stalled = (std::abs(current_wheel_efforts(FL)) > params_.stall_torque_threshold) &&
                                  (std::abs(current_wheel_vels(FL)) < params_.stall_vel_threshold);
                bool fr_stalled = (std::abs(current_wheel_efforts(FR)) > params_.stall_torque_threshold) &&
                                  (std::abs(current_wheel_vels(FR)) < params_.stall_vel_threshold);
                bool chassis_blocked = (body_vx < 0.08) && (std::abs(body_vx) < 0.15);

                if ((fl_stalled || fr_stalled) && chassis_blocked) {
                    front_stall_timer_ += dt;
                } else {
                    front_stall_timer_ = std::max(0.0, front_stall_timer_ - dt);
                }

                // Confirm collision if either shock onset detected OR sustained stall > 0.15s
                if (front_impact || front_stall_timer_ >= params_.stall_confirm_duration) {
                    front_stall_timer_ = 0.0;
                    base_standing_angles_ = current_ecc_angles;
                    front_climb_retry_count_ = 0;
                    transitionTo(HighStairState::F_PRELOAD);
                }
            } else {
                front_stall_timer_ = 0.0;
            }
            break;
        }

        // ============================================================
        // HOLD: User released forward key -> freeze in place (safe damping)
        // ============================================================
        case HighStairState::HOLD: {
            override_mask_.fill(true);
            ecc_overrides_ = hold_ecc_angles_;
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setZero();
            forward_vel_override_ = 0.0;

            if (user_pushing_forward && pre_hold_state_ != HighStairState::IDLE) {
                transitionTo(pre_hold_state_);
            } else if (user_backing_off &&
                       (pre_hold_state_ == HighStairState::F_PRELOAD ||
                        pre_hold_state_ == HighStairState::F_WALL_ROLL)) {
                abort();
            }
            break;
        }

        // ============================================================
        // 1. F_PRELOAD: Tuck rear legs to shift CoM rearward, pin rear wheels
        // ============================================================
        case HighStairState::F_PRELOAD: {
            override_mask_.fill(true);
            ecc_overrides_(FL) = base_standing_angles_(FL);
            ecc_overrides_(FR) = base_standing_angles_(FR);
            ecc_overrides_(RL) = base_standing_angles_(RL) + params_.rear_tuck_angle;
            ecc_overrides_(RR) = base_standing_angles_(RR) + params_.rear_tuck_angle;

            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = 0.0;
            wheel_speed_overrides_(FR) = 0.0;
            wheel_speed_overrides_(RL) = params_.rear_pin_speed / r_w;
            wheel_speed_overrides_(RR) = params_.rear_pin_speed / r_w;
            forward_vel_override_ = params_.rear_pin_speed;

            bool rear_pin_ok = (std::abs(current_wheel_efforts(RL)) > 2.0 || std::abs(current_wheel_efforts(RR)) > 2.0);
            if (state_timer_ > 0.20 && (rear_pin_ok || state_timer_ > 0.60)) {
                front_flip_angle_ = 0.0;
                transitionTo(HighStairState::F_WALL_ROLL);
            } else if (state_timer_ > params_.preload_timeout) {
                abort();
            }
            break;
        }

        // ============================================================
        // 2. F_WALL_ROLL: Front wheels roll up wall under closed-loop pitch control
        // ============================================================
        case HighStairState::F_WALL_ROLL: {
            override_mask_.fill(true);
            double pitch_err = body_pitch - params_.pitch_target;
            double flip_rate = params_.base_flip_speed + params_.k_pitch * std::max(0.0, pitch_err);
            front_flip_angle_ -= flip_rate * dt;
            front_flip_angle_ = std::clamp(front_flip_angle_, -M_PI, 0.0);

            ecc_overrides_(FL) = base_standing_angles_(FL) + front_flip_angle_;
            ecc_overrides_(FR) = base_standing_angles_(FR) + front_flip_angle_;
            ecc_overrides_(RL) = base_standing_angles_(RL) + params_.rear_tuck_angle;
            ecc_overrides_(RR) = base_standing_angles_(RR) + params_.rear_tuck_angle;

            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = params_.front_roll_speed;
            wheel_speed_overrides_(FR) = params_.front_roll_speed;
            wheel_speed_overrides_(RL) = params_.rear_pin_speed / r_w;
            wheel_speed_overrides_(RR) = params_.rear_pin_speed / r_w;
            forward_vel_override_ = params_.rear_pin_speed;

            bool crested = geometry.front_edge_crested || (front_flip_angle_ <= -2.70 && geometry.delta_height >= 0.14);
            if (crested) {
                transitionTo(HighStairState::F_LAND);
            } else if (state_timer_ > params_.wall_roll_timeout) {
                if (geometry.delta_height < 0.08 && front_climb_retry_count_ < 2) {
                    front_climb_retry_count_++;
                    transitionTo(HighStairState::F_PRELOAD);
                } else if (front_flip_angle_ <= -2.50) {
                    transitionTo(HighStairState::F_LAND);
                } else {
                    abort();
                }
            }
            break;
        }

        // ============================================================
        // 3. F_LAND: Front legs landed on curb top, stabilize
        // ============================================================
        case HighStairState::F_LAND: {
            override_mask_.fill(true);
            ecc_overrides_(FL) = base_standing_angles_(FL) - M_PI;
            ecc_overrides_(FR) = base_standing_angles_(FR) - M_PI;
            ecc_overrides_(RL) = base_standing_angles_(RL);
            ecc_overrides_(RR) = base_standing_angles_(RR);

            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = params_.advance_speed / r_w;
            wheel_speed_overrides_(FR) = params_.advance_speed / r_w;
            wheel_speed_overrides_(RL) = params_.advance_speed / r_w;
            wheel_speed_overrides_(RR) = params_.advance_speed / r_w;
            forward_vel_override_ = params_.advance_speed;

            if (state_timer_ > 0.40 && (geometry.delta_height >= 0.14 || state_timer_ > 1.0)) {
                advance_distance_ = 0.0;
                transitionTo(HighStairState::STRADDLE);
            }
            break;
        }

        // ============================================================
        // 4. STRADDLE: Synchronized advance until rear wheels hit curb face
        // ============================================================
        case HighStairState::STRADDLE: {
            override_mask_.fill(true);
            ecc_overrides_(FL) = base_standing_angles_(FL) - M_PI;
            ecc_overrides_(FR) = base_standing_angles_(FR) - M_PI;
            ecc_overrides_(RL) = base_standing_angles_(RL);
            ecc_overrides_(RR) = base_standing_angles_(RR);

            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(params_.advance_speed / r_w);
            forward_vel_override_ = params_.advance_speed;

            double effective_vx = std::abs(body_vx) > 0.02 ? std::abs(body_vx) : params_.advance_speed;
            advance_distance_ += effective_vx * dt;

            bool rear_stalled = (std::abs(current_wheel_vels(RL)) < params_.stall_vel_threshold ||
                                 std::abs(current_wheel_vels(RR)) < params_.stall_vel_threshold) &&
                                (std::abs(current_wheel_efforts(RL)) > params_.stall_torque_threshold ||
                                 std::abs(current_wheel_efforts(RR)) > params_.stall_torque_threshold);

            bool rear_hit = (contact_detector.rearImpactDetected() || rear_stalled) && (advance_distance_ >= 0.28);

            if (rear_hit || advance_distance_ >= 0.38 || state_timer_ > 3.0) {
                transitionTo(HighStairState::R_PRELOAD);
            }
            break;
        }

        // ============================================================
        // 5. R_PRELOAD: Front wheels pull to firmly pin rear against curb face
        // ============================================================
        case HighStairState::R_PRELOAD: {
            override_mask_.fill(true);
            ecc_overrides_(FL) = base_standing_angles_(FL) - M_PI;
            ecc_overrides_(FR) = base_standing_angles_(FR) - M_PI;
            ecc_overrides_(RL) = base_standing_angles_(RL);
            ecc_overrides_(RR) = base_standing_angles_(RR);

            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = params_.pull_speed / r_w;
            wheel_speed_overrides_(FR) = params_.pull_speed / r_w;
            wheel_speed_overrides_(RL) = 0.20 / r_w;
            wheel_speed_overrides_(RR) = 0.20 / r_w;
            forward_vel_override_ = params_.pull_speed;

            if (state_timer_ > 0.30) {
                rear_flip_angle_ = 0.0;
                transitionTo(HighStairState::R_WALL_ROLL);
            }
            break;
        }

        // ============================================================
        // 6. R_WALL_ROLL: Rear flip with pitch compliance + front haul traction
        // ============================================================
        case HighStairState::R_WALL_ROLL: {
            override_mask_.fill(true);
            double flip_rate = params_.base_flip_speed;
            rear_flip_angle_ -= flip_rate * dt;
            rear_flip_angle_ = std::clamp(rear_flip_angle_, -M_PI, 0.0);

            ecc_overrides_(FL) = base_standing_angles_(FL) - M_PI;
            ecc_overrides_(FR) = base_standing_angles_(FR) - M_PI;
            ecc_overrides_(RL) = base_standing_angles_(RL) + rear_flip_angle_;
            ecc_overrides_(RR) = base_standing_angles_(RR) + rear_flip_angle_;

            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = params_.pull_speed / r_w;
            wheel_speed_overrides_(FR) = params_.pull_speed / r_w;
            wheel_speed_overrides_(RL) = params_.rear_roll_speed;
            wheel_speed_overrides_(RR) = params_.rear_roll_speed;
            forward_vel_override_ = params_.pull_speed;

            bool rear_crested = geometry.rear_edge_crested || (rear_flip_angle_ <= -2.80 && std::abs(geometry.delta_height) < 0.06);
            if (rear_crested || state_timer_ >= 2.5) {
                transitionTo(HighStairState::R_LAND);
            }
            break;
        }

        // ============================================================
        // 7. R_LAND: All 4 wheels on curb plateau -> prepare settle
        // ============================================================
        case HighStairState::R_LAND: {
            override_mask_.fill(true);
            ecc_overrides_(FL) = base_standing_angles_(FL) - M_PI;
            ecc_overrides_(FR) = base_standing_angles_(FR) - M_PI;
            ecc_overrides_(RL) = base_standing_angles_(RL) - M_PI;
            ecc_overrides_(RR) = base_standing_angles_(RR) - M_PI;

            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(params_.settle_speed / r_w);
            forward_vel_override_ = params_.settle_speed;

            settle_traj_.plan(0.0, 1.0, params_.settle_duration);
            transitionTo(HighStairState::RESET_SETTLE);
            break;
        }

        // ============================================================
        // 8. RESET_SETTLE: S-curve restore all eccentric joints to nominal standing
        // ============================================================
        case HighStairState::RESET_SETTLE: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(params_.settle_speed / r_w);
            forward_vel_override_ = params_.settle_speed;

            double s, ds, dds;
            settle_traj_.evaluate(state_timer_, s, ds, dds);
            s = std::clamp(s, 0.0, 1.0);
            double smooth_s = 0.5 * (1.0 - std::cos(M_PI * s));

            ecc_overrides_(FL) = (1.0 - smooth_s) * (base_standing_angles_(FL) - M_PI) + smooth_s * base_standing_angles_(FL);
            ecc_overrides_(FR) = (1.0 - smooth_s) * (base_standing_angles_(FR) - M_PI) + smooth_s * base_standing_angles_(FR);
            ecc_overrides_(RL) = (1.0 - smooth_s) * (base_standing_angles_(RL) - M_PI) + smooth_s * base_standing_angles_(RL);
            ecc_overrides_(RR) = (1.0 - smooth_s) * (base_standing_angles_(RR) - M_PI) + smooth_s * base_standing_angles_(RR);

            if (s >= 1.0 || state_timer_ >= params_.settle_duration) {
                blend_out_timer_ = 0.0;
                blend_out_weight_ = 1.0;
                wheel_speed_overrides_.setZero();
                wheel_override_mask_.fill(false);
                transitionTo(HighStairState::IDLE);
            }
            break;
        }
    }

    // Post-switch blend-out fade
    if (state_ == HighStairState::IDLE && blend_out_weight_ > 0.0) {
        blend_out_timer_ += dt;
        double progress = blend_out_timer_ / params_.blend_out_duration;
        blend_out_weight_ = std::max(0.0, 1.0 - progress);

        if (blend_out_weight_ > 0.0) {
            override_mask_.fill(true);
            ecc_overrides_ = base_standing_angles_;
        } else {
            override_mask_.fill(false);
        }
    }
}

std::array<bool, NUM_LEGS> HighStairController::getContactValid() const {
    std::array<bool, NUM_LEGS> valid = {true, true, true, true};
    if (state_ == HighStairState::F_WALL_ROLL) {
        valid[FL] = false;
        valid[FR] = false;
    } else if (state_ == HighStairState::R_WALL_ROLL) {
        valid[RL] = false;
        valid[RR] = false;
    }
    return valid;
}

}  // namespace tasks
}  // namespace robot_control
