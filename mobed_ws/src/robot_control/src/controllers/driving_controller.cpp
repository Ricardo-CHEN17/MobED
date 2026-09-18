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
    // Bank Angle Calculator (Paper Eq 8)
    // ================================================================

    double vx = cmd_vel(0);
    double vy = cmd_vel(1);
    double wz = cmd_vel(2);

    // Centrifugal acceleration x-component (alpha_{c,x})
    // alpha_c = - omega x v
    // omega = (0, 0, wz), v = (vx, vy, 0)
    // omega x v = (-wz * vy, wz * vx, 0)
    // alpha_c = (wz * vy, -wz * vx, 0)
    double alpha_cx = wz * vy;

    // Bank angle: atan(alpha_cx / g)
    double bank = std::atan2(alpha_cx, 9.81);

    // Clamp to maximum allowable bank
    bank = std::clamp(bank, -params_.max_bank_angle, params_.max_bank_angle);

    return bank;
}

double DrivingController::clampSteerSafe(int leg_index, double target_steer, double current_ecc) const {
    // If the eccentric arm is nearly vertical, collision is impossible
    if (std::abs(current_ecc) < params_.ecc_collision_threshold) {
        return target_steer;
    }

    auto kParams = kinematics_->getParams();
    double px = (leg_index == 0 || leg_index == 1) ? kParams.length_x : -kParams.length_x;
    double py = (leg_index == 0 || leg_index == 2) ? kParams.width_y : -kParams.width_y;

    double l_ecc = kParams.l_ecc; 
    double D_xy = l_ecc * std::sin(current_ecc);

    // The forbidden direction is the direction from the corner (px, py) towards the chassis center (0,0).
    // Center is at (-px, -py) relative to the corner.
    double center_angle = std::atan2(-py, -px);
    
    // The actual direction the wheel extends from the corner is target_steer if D_xy > 0, 
    // or target_steer + PI if D_xy < 0.
    double extension_angle = target_steer;
    if (D_xy < 0) {
        extension_angle = target_steer + M_PI;
    }
    
    // Normalize extension_angle to [-PI, PI]
    while (extension_angle > M_PI) extension_angle -= 2.0 * M_PI;
    while (extension_angle <= -M_PI) extension_angle += 2.0 * M_PI;
    
    // The forbidden zone for the extension angle is [center_angle - clearance, center_angle + clearance]
    double diff = extension_angle - center_angle;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff <= -M_PI) diff += 2.0 * M_PI;
    
    if (std::abs(diff) < params_.steer_ecc_clearance) {
        // It's inside the forbidden zone! Clamp it to the nearest edge.
        if (diff >= 0) {
            extension_angle = center_angle + params_.steer_ecc_clearance;
        } else {
            extension_angle = center_angle - params_.steer_ecc_clearance;
        }
        
        // Recover target_steer
        double clamped_steer = extension_angle;
        if (D_xy < 0) {
            clamped_steer -= M_PI;
        }
        
        while (clamped_steer > M_PI) clamped_steer -= 2.0 * M_PI;
        while (clamped_steer <= -M_PI) clamped_steer += 2.0 * M_PI;
        
        return clamped_steer;
    }
    
    return target_steer;
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
    // 2. Swerve Drive Inverse Kinematics
    // ================================================================
    Eigen::Vector4d raw_steer_angles;
    Eigen::Vector4d raw_wheel_speeds;
    if (is_homing) {
        raw_steer_angles.setZero();
        raw_wheel_speeds.setZero();
    } else {
        // Use prev_steer_angles_ (last commanded) instead of current_steer_angles (physical)
        // This prevents a positive feedback loop with the PD controller noise when stationary.
        auto [s, w] = kinematics_->computeDrivingIK(cmd_vel, prev_steer_angles_);
        raw_steer_angles = s;
        raw_wheel_speeds = w;
    }

    Eigen::Vector4d final_steer = prev_steer_angles_;
    Eigen::Vector4d final_speed = prev_wheel_speeds_;

    for (int i = 0; i < 4; ++i) {
        // ============================================================
        // 3. Steering Constraint Function (geometric anti-collision)
        // ============================================================
        raw_steer_angles(i) = clampSteerSafe(i, raw_steer_angles(i), current_ecc_angles(i));

        // ============================================================
        // 4. Slew Rate Limiting (Speed/Accel)
        // ============================================================
        double speed_delta = raw_wheel_speeds(i) - prev_wheel_speeds_(i);
        double max_speed_delta = params_.max_wheel_accel * dt;
        speed_delta = std::clamp(speed_delta, -max_speed_delta, max_speed_delta);
        final_speed(i) = prev_wheel_speeds_(i) + speed_delta;

        // Rate Limiting (Steering)
        double steer_delta = raw_steer_angles(i) - prev_steer_angles_(i);
        double max_steer_delta = params_.max_steer_vel * dt;
        steer_delta = std::clamp(steer_delta, -max_steer_delta, max_steer_delta);
        final_steer(i) = prev_steer_angles_(i) + steer_delta;
    }

    // 5. Update state and return
    prev_steer_angles_ = final_steer;
    prev_wheel_speeds_ = final_speed;

    return {final_steer, final_speed};
}

}  // namespace controllers
}  // namespace robot_control
