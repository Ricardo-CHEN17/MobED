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

    // Lateral centripetal acceleration in body frame: a_y = wz * vx.
    // The resulting centrifugal inertial force on the body points in -Y (for wz*vx > 0).
    // In ISO frame (X fwd, Y left, Z up), leaning inward into a left turn requires
    // negative roll (left side drops, right side rises).
    // Eq (8): theta_b^d = -arctan(alpha_c_lateral / g)
    double alpha_c_lateral = wz * vx;

    // Desired inward bank angle
    double bank = -std::atan2(alpha_c_lateral, 9.81);

    // Clamp to maximum allowable bank
    bank = std::clamp(bank, -params_.max_bank_angle, params_.max_bank_angle);

    return bank;
}

std::pair<double, double> DrivingController::optimizeSwerveModule(
    int leg_index, double des_steer, double des_speed, double curr_steer, double curr_ecc) const {
    
    if (std::abs(des_speed) < 1e-3) {
        return {curr_steer, 0.0};
    }

    // Candidate State A: (des_steer, des_speed)
    double s_A = kinematics_->normalize_angle(des_steer);
    double v_A = des_speed;

    // Inverted State B: (des_steer +- pi, -des_speed)
    // Exact physical equivalence: produces identical ground traction vector
    double s_B = kinematics_->normalize_angle(des_steer + M_PI);
    double v_B = -des_speed;

    // Query dynamic geometric forbidden inward sector from kinematics
    auto [forbid_min, forbid_max] = kinematics_->getForbiddenSteerSector(leg_index, curr_ecc);
    bool has_forbid = (forbid_min != forbid_max);

    // Check collision safety for both states
    bool safe_A = true;
    bool safe_B = true;
    if (has_forbid) {
        safe_A = (s_A < forbid_min || s_A > forbid_max);
        safe_B = (s_B < forbid_min || s_B > forbid_max);
    }

    double diff_A = std::abs(kinematics_->normalize_angle(s_A - curr_steer));
    double diff_B = std::abs(kinematics_->normalize_angle(s_B - curr_steer));

    if (safe_A && safe_B) {
        // Both safe: choose shortest angular travel (|delta| <= 90 deg)
        return (diff_A <= diff_B) ? std::make_pair(s_A, v_A) : std::make_pair(s_B, v_B);
    } else if (safe_A) {
        return {s_A, v_A};
    } else if (safe_B) {
        // Active evasion: flip 180 deg to free outward sector with 0% force loss
        return {s_B, v_B};
    } else {
        // Extremely rare double-restriction fallback: clamp to nearest boundary
        double clamped_s = std::clamp(s_A, forbid_min, forbid_max);
        return {clamped_s, 0.0};
    }
}

std::tuple<Eigen::Vector4d, Eigen::Vector4d> DrivingController::update(
    const Eigen::Vector3d& cmd_vel,
    const Eigen::Vector4d& current_steer_angles,
    const Eigen::Vector4d& current_ecc_angles,
    double dt,
    bool e_stop_active,
    bool is_homing,
    const Eigen::Vector4d& current_steer_vels,
    const Eigen::Vector4d& current_wheel_vels) {

    // First time initialization to current hardware state
    if (!initialized_) {
        prev_steer_angles_ = current_steer_angles;
        prev_wheel_speeds_.setZero();
        steer_torques_.setZero();
        wheel_torques_.setZero();
        wheel_speed_integrals_.setZero();
        initialized_ = true;
    }

    if (e_stop_active) {
        // E-STOP: Lock steering, stop wheels immediately
        prev_wheel_speeds_.setZero();
        filtered_bank_angle_ = 0.0;
        steer_torques_.setZero();
        wheel_torques_.setZero();
        wheel_speed_integrals_.setZero();
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

    // ============================================================
    // 3. Two-Stage Swerve Optimization (Shortest-Path + 180-deg Inversion Escape)
    // ============================================================
    Eigen::Vector4d opt_steer = prev_steer_angles_;
    Eigen::Vector4d opt_speed = prev_wheel_speeds_;

    for (int i = 0; i < 4; ++i) {
        auto [s, w] = optimizeSwerveModule(
            i, raw_steer_angles(i), raw_wheel_speeds(i),
            prev_steer_angles_(i), current_ecc_angles(i));
        opt_steer(i) = s;
        opt_speed(i) = w;
    }

    // ============================================================
    // 4. Chassis-Level Coherent Drive Scaling (RoboMaster Swerve Principle)
    // Scale driving wheel speeds by cos(max_error) so wheels don't fight while rotating into alignment!
    // ============================================================
    double max_steer_error = 0.0;
    for (int i = 0; i < 4; ++i) {
        double err = std::abs(kinematics_->normalize_angle(opt_steer(i) - current_steer_angles(i)));
        if (err > max_steer_error) {
            max_steer_error = err;
        }
    }

    double coherent_drive_scale = 1.0;
    if (params_.enable_coherent_drive_scaling) {
        if (max_steer_error > params_.steer_alignment_deadband) {
            coherent_drive_scale = std::max(0.0, std::cos(max_steer_error));
        }
    }

    // ============================================================
    // 5. Slew Rate Limiting (Speed/Accel & Steering velocity)
    // ============================================================
    Eigen::Vector4d final_steer = prev_steer_angles_;
    Eigen::Vector4d final_speed = prev_wheel_speeds_;

    for (int i = 0; i < 4; ++i) {
        double target_speed = opt_speed(i) * coherent_drive_scale;
        double speed_delta = target_speed - prev_wheel_speeds_(i);
        double max_speed_delta = params_.max_wheel_accel * dt;
        speed_delta = std::clamp(speed_delta, -max_speed_delta, max_speed_delta);
        final_speed(i) = prev_wheel_speeds_(i) + speed_delta;

        // Rate Limiting (Steering) with shortest-path angular normalization
        double steer_delta = kinematics_->normalize_angle(opt_steer(i) - prev_steer_angles_(i));
        double max_steer_delta = params_.max_steer_vel * dt;
        steer_delta = std::clamp(steer_delta, -max_steer_delta, max_steer_delta);
        final_steer(i) = kinematics_->normalize_angle(prev_steer_angles_(i) + steer_delta);
    }

    // ============================================================
    // 5. Low-Level Actuator Torque Loops (Paper Fig. 8 & Dual-Layer Mode)
    // ============================================================
    for (int i = 0; i < 4; ++i) {
        // Steering angle PD torque
        double e_str = kinematics_->normalize_angle(final_steer(i) - current_steer_angles(i));
        double tau_str = params_.kp_steer * e_str - params_.kd_steer * current_steer_vels(i);
        steer_torques_(i) = std::clamp(tau_str, -params_.max_steer_torque, params_.max_steer_torque);

        // Wheel speed PI torque with anti-windup & traction saturation
        double e_whl = final_speed(i) - current_wheel_vels(i);
        wheel_speed_integrals_(i) += e_whl * dt;
        wheel_speed_integrals_(i) = std::clamp(wheel_speed_integrals_(i), -10.0, 10.0);
        double tau_whl = params_.kp_wheel * e_whl + params_.ki_wheel * wheel_speed_integrals_(i);
        wheel_torques_(i) = std::clamp(tau_whl, -params_.max_wheel_torque, params_.max_wheel_torque);
    }

    // 6. Update state and return
    prev_steer_angles_ = final_steer;
    prev_wheel_speeds_ = final_speed;

    return {final_steer, final_speed};
}

}  // namespace controllers
}  // namespace robot_control
