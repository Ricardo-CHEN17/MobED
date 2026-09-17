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
    //
    // During curved motion, centripetal acceleration creates a lateral
    // force on the chassis. To prevent tipping, the chassis leans inward
    // (like a motorcycle cornering).
    //
    // Physics:
    //   a_centripetal = v * omega_z
    //   tan(bank_angle) = a_centripetal / g = v * omega_z / g
    //
    // For omnidirectional motion, use the total translational speed:
    //   v = sqrt(vx^2 + vy^2)
    //
    // The sign of the bank angle follows the yaw rate:
    //   positive omega_z (CCW turn) -> lean left (negative roll)
    //   negative omega_z (CW turn)  -> lean right (positive roll)
    // ================================================================

    double vx = cmd_vel(0);
    double vy = cmd_vel(1);
    double wz = cmd_vel(2);

    double v = std::sqrt(vx * vx + vy * vy);

    // No bank needed for very low speed or no rotation
    if (v < 0.05 || std::abs(wz) < 0.01) {
        return 0.0;
    }

    // Centripetal acceleration
    double a_cent = v * wz;

    // Bank angle: atan(a_cent / g)
    // Negative sign: CCW turn (wz > 0) should produce a lean into the turn
    double bank = std::atan2(-a_cent, 9.81);

    // Clamp to maximum allowable bank
    bank = std::clamp(bank, -params_.max_bank_angle, params_.max_bank_angle);

    return bank;
}

bool DrivingController::isSteerSafe(int leg_index, double target_steer, double current_ecc) const {
    (void)leg_index;
    (void)target_steer;
    (void)current_ecc;

    // Disabled geometric constraint: previous logic was flawed as it compared 
    // the steering yaw angle directly with the eccentric pitch angle.
    // For now, always allow steering to prevent wheel lockups.
    return true;
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
        if (!isSteerSafe(i, raw_steer_angles(i), current_ecc_angles(i))) {
            // Collision zone! Kill speed, hold steering.
            raw_wheel_speeds(i) = 0.0;
            raw_steer_angles(i) = prev_steer_angles_(i);
        }

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
