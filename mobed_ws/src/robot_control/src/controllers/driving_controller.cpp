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
    double vy = cmd_vel(1); // Lateral velocity
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
    // If the eccentric arm is nearly vertical, collision is impossible
    if (std::abs(current_ecc) < params_.ecc_collision_threshold) {
        return;
    }

    auto kParams = kinematics_->getParams();
    double px = (leg_index == 0 || leg_index == 1) ? kParams.length_x : -kParams.length_x;
    double py = (leg_index == 0 || leg_index == 2) ? kParams.width_y : -kParams.width_y;

    double D_xy = kParams.l_ecc * std::sin(current_ecc);

    // The forbidden direction is the direction from the corner (px, py) towards the chassis center (0,0).
    double center_angle = std::atan2(-py, -px);
    
    // The actual direction the wheel extends from the corner is steer if D_xy > 0, 
    // or steer + PI if D_xy < 0.
    double extension_angle = steer;
    if (D_xy < 0.0) {
        extension_angle = steer + M_PI;
    }
    
    // Normalize extension_angle to [-PI, PI]
    while (extension_angle > M_PI) extension_angle -= 2.0 * M_PI;
    while (extension_angle <= -M_PI) extension_angle += 2.0 * M_PI;
    
    // The forbidden zone for the extension angle is [center_angle - clearance, center_angle + clearance]
    double diff = extension_angle - center_angle;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff <= -M_PI) diff += 2.0 * M_PI;
    
    if (std::abs(diff) < params_.steer_ecc_clearance) {
        double orig_steer = steer;

        // Clamp to the nearest safe edge of the forbidden sector
        if (diff >= 0.0) {
            extension_angle = center_angle + params_.steer_ecc_clearance;
        } else {
            extension_angle = center_angle - params_.steer_ecc_clearance;
        }
        
        // Recover steer angle
        double clamped_steer = extension_angle;
        if (D_xy < 0.0) {
            clamped_steer -= M_PI;
        }
        
        while (clamped_steer > M_PI) clamped_steer -= 2.0 * M_PI;
        while (clamped_steer <= -M_PI) clamped_steer += 2.0 * M_PI;
        
        steer = clamped_steer;

        // Project wheel speed onto the clamped steering heading
        double angle_error = steer - orig_steer;
        double proj_factor = std::cos(angle_error);
        if (proj_factor <= 0.0) {
            speed = 0.0;
        } else {
            speed *= proj_factor;
        }
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
    } else {
        // Use prev_steer_angles_ (last commanded) instead of current_steer_angles (physical)
        // Pass current_ecc_angles for exact contact point ^B r_i (Section III.D)
        auto [s, w] = kinematics_->computeDrivingIK(cmd_vel, prev_steer_angles_, current_ecc_angles);
        raw_steer_angles = s;
        raw_wheel_speeds = w;
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
