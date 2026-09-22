#include "robot_control/controllers/driving_controller.hpp"
#include <algorithm>
#include <cmath>

namespace robot_control {
namespace controllers {

DrivingController::DrivingController(const DrivingControllerParams& params,
                                     std::shared_ptr<kinematics::MobedKinematics> kinematics)
    : params_(params), kinematics_(kinematics) {
    prev_steer_angles_.setZero();
    prev_wheel_speeds_.setZero();
}

double DrivingController::computeBankAngle(const Eigen::Vector3d& cmd_vel) const {
    // ================================================================
    // Bank Angle Calculator (Paper Section III.D & Eq 8)
    // ================================================================

    double vx = cmd_vel(0); // Forward velocity
    double wz = cmd_vel(2); // Yaw rate

    // Lateral centrifugal acceleration to counteract roll during turning:
    // When driving forward (vx) and turning (wz), lateral acceleration is wz * vx.
    double alpha_c_lateral = wz * vx;

    // Desired bank angle theta_b^d = arctan(alpha_c / g) (Eq 8)
    double bank = std::atan2(alpha_c_lateral, 9.81);

    // Clamp to maximum allowable bank
    bank = std::clamp(bank, -params_.max_bank_angle, params_.max_bank_angle);

    return bank;
}

void DrivingController::applySteerConstraint(
    int leg_index, double& steer, double& speed, double current_ecc) const {
    // Dynamic inward steering limit based on current eccentric posture angle (Section III.D)
    // When the eccentric arm is more horizontally extended (|current_ecc| is large),
    // the wheel contact point extends further, so turning inward brings it even closer
    // to the chassis boundary. We dynamically tighten the inward limit accordingly.
    double max_inward = params_.max_inward_steer_angle;
    double ecc_mag = std::abs(current_ecc);
    if (ecc_mag > 0.8) {
        max_inward = std::max(0.78, params_.max_inward_steer_angle - 0.3 * (ecc_mag - 0.8));
    }
    double max_outward = params_.max_outward_steer_angle;

    // Safe steering angle range [min_steer, max_steer] for each leg:
    // - FL (leg 0): turning left (>0) is outward; turning right (<0) is inward towards chassis.
    //               safe range: [-max_inward, +max_outward]
    // - FR (leg 1): turning right (<0) is outward; turning left (>0) is inward towards chassis.
    //               safe range: [-max_outward, +max_inward]
    // - RL (leg 2, q_ecc < 0 in outward config): turning right (<0) is outward; turning left (>0) is inward.
    //               safe range: [-max_outward, +max_inward]
    // - RR (leg 3, q_ecc < 0 in outward config): turning left (>0) is outward; turning right (<0) is inward.
    //               safe range: [-max_inward, +max_outward]
    double min_steer, max_steer;
    if (leg_index == 0 || leg_index == 3) {
        // FL (0) or RR (3)
        min_steer = -max_inward;
        max_steer =  max_outward;
    } else {
        // FR (1) or RL (2)
        min_steer = -max_outward;
        max_steer =  max_inward;
    }

    double orig_steer = steer;
    steer = std::clamp(steer, min_steer, max_steer);

    // Project wheel speed onto the clamped steering heading
    double angle_error = steer - orig_steer;
    double proj_factor = std::cos(angle_error);
    if (proj_factor <= 0.0) {
        speed = 0.0;
    } else {
        speed *= proj_factor;
    }
}

std::tuple<Eigen::Vector4d, Eigen::Vector4d> DrivingController::update(
    const Eigen::Vector3d& cmd_vel,
    const Eigen::Vector4d& current_steer_angles,
    const Eigen::Vector4d& current_ecc_angles,
    double dt,
    bool e_stop_active,
    bool is_homing) {

    // First time initialization to current hardware state
    if (!initialized_) {
        prev_steer_angles_ = current_steer_angles;
        prev_wheel_speeds_.setZero();
        initialized_ = true;
    }

    if (e_stop_active) {
        // E-STOP: Lock steering, stop wheels immediately
        prev_wheel_speeds_.setZero();
        filtered_bank_angle_ = 0.0;
        return {prev_steer_angles_, prev_wheel_speeds_};
    }

    // ================================================================
    // 1. Compute Bank Angle (centripetal force compensation)
    // ================================================================
    double raw_bank = is_homing ? 0.0 : computeBankAngle(cmd_vel);
    // Low-pass filter for smooth transitions
    filtered_bank_angle_ += params_.bank_angle_filter * (raw_bank - filtered_bank_angle_);

    // ================================================================
    // 2. Swerve Drive Inverse Kinematics (Eq 7 in paper)
    // ================================================================
    Eigen::Vector4d raw_steer_angles;
    Eigen::Vector4d raw_wheel_speeds;
    if (is_homing) {
        raw_steer_angles.setZero();
        raw_wheel_speeds.setZero();
        zero_cmd_duration_ = 0.0;
    } else {
        if (cmd_vel.norm() < 1e-3) {
            zero_cmd_duration_ += dt;
        } else {
            zero_cmd_duration_ = 0.0;
        }

        // Use prev_steer_angles_ (last commanded) instead of current_steer_angles (physical)
        // Pass current_ecc_angles for exact contact point ^B r_i (Section III.D)
        auto [s, w] = kinematics_->computeDrivingIK(cmd_vel, prev_steer_angles_, current_ecc_angles);
        raw_steer_angles = s;
        raw_wheel_speeds = w;

        // If stopped for > 0.3s, gently guide steering back to neutral (0.0 rad)
        // This prevents steering angles from remaining stuck sideways (+-90 deg) after rotation/strafe
        if (zero_cmd_duration_ > 0.3) {
            raw_steer_angles.setZero();
        }
    }

    Eigen::Vector4d final_steer = prev_steer_angles_;
    Eigen::Vector4d final_speed = prev_wheel_speeds_;

    for (int i = 0; i < 4; ++i) {
        // ============================================================
        // 3. Steering Constraint Function (geometric anti-collision)
        // ============================================================
        applySteerConstraint(i, raw_steer_angles(i), raw_wheel_speeds(i), current_ecc_angles(i));

        // ============================================================
        // 4. Slew Rate Limiting (Speed/Accel)
        // ============================================================
        double speed_delta = raw_wheel_speeds(i) - prev_wheel_speeds_(i);
        double max_speed_delta = params_.max_wheel_accel * dt;
        speed_delta = std::clamp(speed_delta, -max_speed_delta, max_speed_delta);
        final_speed(i) = prev_wheel_speeds_(i) + speed_delta;

        // Rate Limiting (Steering) with shortest-path angular normalization
        double steer_delta = kinematics_->normalize_angle(raw_steer_angles(i) - prev_steer_angles_(i));
        double max_steer_delta = params_.max_steer_vel * dt;
        steer_delta = std::clamp(steer_delta, -max_steer_delta, max_steer_delta);
        final_steer(i) = kinematics_->normalize_angle(prev_steer_angles_(i) + steer_delta);
    }

    // 5. Update state and return
    prev_steer_angles_ = final_steer;
    prev_wheel_speeds_ = final_speed;

    return {final_steer, final_speed};
}

}  // namespace controllers
}  // namespace robot_control
