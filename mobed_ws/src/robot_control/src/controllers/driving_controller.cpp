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
    int leg_index, double des_steer, double des_speed, double curr_steer, double /*curr_ecc*/,
    bool force_align) {
    
    if (std::abs(des_speed) < 1e-3) {
        if (!force_align) {
            return {curr_steer, 0.0};
        }
        // Force-alignment requested (homing or neutral centering):
        // Steer directly towards des_steer, clamping outside forbidden sector
        double s_target = kinematics_->normalize_angle(des_steer);
        s_target = kinematics_->clampToSafeSteerSector(leg_index, s_target);
        return {s_target, 0.0};
    }

    // Candidate State A: (des_steer, des_speed)
    double s_A = kinematics_->normalize_angle(des_steer);
    double v_A = des_speed;

    // Inverted State B: (des_steer +- pi, -des_speed)
    // Produces identical ground traction vector while keeping wheel in outward safe sector
    double s_B = kinematics_->normalize_angle(des_steer + M_PI);
    double v_B = -des_speed;

    // Safe Sector Check: evaluated against each leg's individual safe outward sector
    bool safe_A = !kinematics_->isSteerInForbiddenSector(leg_index, s_A);
    bool safe_B = !kinematics_->isSteerInForbiddenSector(leg_index, s_B);

    auto cfg = kinematics_->getLegSafeSectorConfig(leg_index);
    double u_curr = kinematics_->normalize_angle(curr_steer - cfg.center_angle);
    double u_A    = kinematics_->normalize_angle(s_A - cfg.center_angle);
    double u_B    = kinematics_->normalize_angle(s_B - cfg.center_angle);
    double diff_A = std::abs(u_A - u_curr);
    double diff_B = std::abs(u_B - u_curr);

    // Stance bias: slight preference for states closer to widest stance angle
    double opt_stance = kinematics_->getOptimalOutwardStanceAngle(leg_index);
    double stance_dist_A = std::abs(kinematics_->normalize_angle(s_A - opt_stance));
    double stance_dist_B = std::abs(kinematics_->normalize_angle(s_B - opt_stance));

    std::pair<double, double> result;

    if (safe_A && safe_B) {
        // Both safe (typically around +-90 deg pure strafe boundary):
        // Apply switching hysteresis to prevent chattering
        bool was_inverted = prev_swerve_inverted_[leg_index];
        double hyst = params_.swerve_invert_hysteresis; // ~0.175 rad (10 deg)
        
        bool choose_B;
        if (was_inverted) {
            // Favor staying in B unless A is noticeably shorter
            choose_B = (diff_A > diff_B - hyst);
        } else {
            // Favor staying in A unless B is noticeably shorter
            choose_B = (diff_B < diff_A - hyst);
        }

        // If diffs are very close, break tie towards wider track stance
        if (std::abs(diff_A - diff_B) < hyst) {
            choose_B = (stance_dist_B < stance_dist_A);
        }

        if (choose_B) {
            prev_swerve_inverted_[leg_index] = true;
            result = {s_B, v_B};
        } else {
            prev_swerve_inverted_[leg_index] = false;
            result = {s_A, v_A};
        }
    } else if (safe_A) {
        prev_swerve_inverted_[leg_index] = false;
        result = {s_A, v_A};
    } else if (safe_B) {
        // Active evasion: 180 deg inversion keeps wheel safely outward!
        // For spin, FL/RL/FR/RR will reliably select this when direct target points inward.
        prev_swerve_inverted_[leg_index] = true;
        result = {s_B, v_B};
    } else {
        // Fallback: when neither candidate is strictly within the safe sector (e.g. at transition boundaries),
        // pick whichever candidate is closer to the safe sector center, clamp it to the safe boundary,
        // and project the drive velocity onto the clamped heading rather than zeroing it out.
        bool choose_B = (std::abs(u_B) < std::abs(u_A));
        double target_s = choose_B ? s_B : s_A;
        double base_v   = choose_B ? v_B : v_A;
        double safe_s   = kinematics_->clampToSafeSteerSector(leg_index, target_s);
        double v_proj   = base_v * std::cos(safe_s - target_s);
        prev_swerve_inverted_[leg_index] = choose_B;
        result = {safe_s, v_proj};
    }

    return result;
}

double DrivingController::enforceForbiddenSectorTransit(
    int leg_index, double prev_s, double opt_s, double max_delta) const {

    // 1. Both endpoints are strictly confined inside this leg's safe outward sector
    opt_s  = kinematics_->clampToSafeSteerSector(leg_index, opt_s);
    prev_s = kinematics_->clampToSafeSteerSector(leg_index, prev_s);

    // 2. Continuous 1D monotonic transit in unwrapped safe coordinate centered at cfg.center_angle.
    // In this coordinate frame, the safe sector is [-half_width, +half_width], which is strictly convex.
    // Stepping between any two points is guaranteed to stay within the safe sector and never cross into the forbidden zone.
    auto cfg = kinematics_->getLegSafeSectorConfig(leg_index);
    double u_opt  = kinematics_->normalize_angle(opt_s - cfg.center_angle);
    double u_prev = kinematics_->normalize_angle(prev_s - cfg.center_angle);
    double delta  = u_opt - u_prev;
    double step   = std::clamp(delta, -max_delta, max_delta);
    double u_next = u_prev + step;
    double next_s = kinematics_->normalize_angle(cfg.center_angle + u_next);

    // 3. Guaranteed barrier defense: clamp to safe sector
    return kinematics_->clampToSafeSteerSector(leg_index, next_s);
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
    // 0. Velocity Saturation Limits
    // ================================================================
    Eigen::Vector3d active_cmd = cmd_vel;
    double v_planar = active_cmd.head<2>().norm();
    if (v_planar > params_.max_linear_vel && v_planar > 1e-4) {
        active_cmd.head<2>() *= (params_.max_linear_vel / v_planar);
    }
    active_cmd.z() = std::clamp(active_cmd.z(), -params_.max_angular_vel, params_.max_angular_vel);

    // ================================================================
    // 1. Compute Bank Angle (centripetal force compensation)
    // ================================================================
    double raw_bank = is_homing ? 0.0 : computeBankAngle(active_cmd);
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
        if (active_cmd.norm() < 1e-3) {
            zero_cmd_duration_ += dt;
        } else {
            zero_cmd_duration_ = 0.0;
        }

        // Use prev_steer_angles_ (last commanded) instead of current_steer_angles (physical)
        // Pass current_ecc_angles for exact contact point ^B r_i (Section III.D)
        auto [s, w] = kinematics_->computeDrivingIK(active_cmd, prev_steer_angles_, current_ecc_angles);
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

    bool align_to_neutral = is_homing || (zero_cmd_duration_ > 0.3);

    for (int i = 0; i < 4; ++i) {
        auto [s, w] = optimizeSwerveModule(
            i, raw_steer_angles(i), raw_wheel_speeds(i),
            prev_steer_angles_(i), current_ecc_angles(i),
            align_to_neutral);
        opt_steer(i) = s;
        opt_speed(i) = w;
    }

    // ============================================================
    // 3.5 Ground Traction Vector Projection & Alignment
    // Industry standard practice (NASA/Hyundai e-Corner/Swerve Drive):
    // Computes drive speed via dot product projection: v_module = (v_contact · u_hat) / r_wheel
    // Ensures that for any module heading, the contact point push vector strictly aligns
    // with the commanded body velocity, completely eliminating opposing front/rear wheel forces.
    // ============================================================
    if (active_cmd.norm() >= 1e-3) {
        double r_w = kinematics_->getParams().r_wheel;
        if (r_w < 1e-4) r_w = 0.10;

        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d B_r_i = kinematics_->computeFootPositionInBase(i, opt_steer(i), current_ecc_angles(i));
            double v_ix = active_cmd.x() - active_cmd.z() * B_r_i.y();
            double v_iy = active_cmd.y() + active_cmd.z() * B_r_i.x();

            double u_x = std::cos(opt_steer(i));
            double u_y = std::sin(opt_steer(i));
            double v_dot = (v_ix * u_x + v_iy * u_y) / r_w;

            // Lateral strafe traction alignment:
            // Ensure the lateral traction component (v_module * sin(theta_steer)) strictly matches
            // the commanded lateral direction sign(active_cmd.y()).
            if (std::abs(active_cmd.y()) > 1e-3 && std::abs(u_y) > 0.1) {
                double sign_cmd_y = (active_cmd.y() >= 0.0) ? 1.0 : -1.0;
                double sign_steer_y = (u_y >= 0.0) ? 1.0 : -1.0;
                // Base-frame lateral push on chassis is: F_y ~ v_wheel * sin(theta_steer).
                // To guarantee F_y has sign_cmd_y: v_wheel must have sign (sign_cmd_y * sign_steer_y).
                double desired_sign = sign_cmd_y * sign_steer_y;
                double speed_mag = std::abs(v_dot) > 1e-4 ? std::abs(v_dot) : std::abs(opt_speed(i));
                v_dot = speed_mag * desired_sign;
            }

            opt_speed(i) = v_dot;
        }
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
        double sat_speed = prev_wheel_speeds_(i) + speed_delta;
        final_speed(i) = std::clamp(sat_speed, -params_.max_wheel_speed, params_.max_wheel_speed);

        // Rate Limiting (Steering) with continuous forbidden sector avoidance
        double max_steer_delta = params_.max_steer_vel * dt;
        final_steer(i) = enforceForbiddenSectorTransit(
            i, prev_steer_angles_(i), opt_steer(i), max_steer_delta);
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
