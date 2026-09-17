#include "robot_control/controllers/balance_controller.hpp"
#include <algorithm>
#include <cmath>

namespace robot_control {
namespace controllers {

BalanceController::BalanceController(const BalanceControllerParams& params,
                                     std::shared_ptr<kinematics::MobedKinematics> kinematics)
    : params_(params),
      kinematics_(kinematics),
      srbd_model_(RobotParams()),
      grf_optimizer_(RobotParams())
{
    prev_ecc_angles_.setZero();
}

// ============================================================================
// Original position-only interface (backward compatible)
// ============================================================================
Eigen::Vector4d BalanceController::update(
    double target_height,
    double target_roll,
    double target_pitch,
    const Eigen::Vector4d& current_steer_angles,
    const Eigen::Vector4d& current_ecc_angles,
    double dt,
    bool e_stop_active) {

    if (e_stop_active) {
        // E-STOP: Hold current physical state and reset initialization
        // so it smoothly recovers via the cubic trajectory when released
        initialized_ = false;
        prev_ecc_angles_ = current_ecc_angles;
        return current_ecc_angles;
    }

    // First time initialization to current hardware state
    if (!initialized_) {
        start_ecc_angles_ = current_ecc_angles;
        prev_ecc_angles_ = current_ecc_angles;
        time_since_init_ = 0.0;
        initialized_ = true;
    }

    // 1. Safety Input Clamp (Prevent impossible target commands)
    double safe_roll = std::clamp(target_roll, -params_.max_roll, params_.max_roll);
    double safe_pitch = std::clamp(target_pitch, -params_.max_pitch, params_.max_pitch);

    // 2. Math Kinematics (Exact non-linear projection)
    Eigen::Vector4d raw_ecc_angles = kinematics_->computePostureIK(
        target_height, safe_roll, safe_pitch, current_steer_angles, false);

    Eigen::Vector4d final_ecc = prev_ecc_angles_;

    for (int i = 0; i < 4; ++i) {
        // 3. Mechanical Limit Clamp
        raw_ecc_angles(i) = std::clamp(raw_ecc_angles(i), params_.min_ecc_angle, params_.max_ecc_angle);
    }

    // 4. Synchronized Smooth Startup Trajectory
    if (time_since_init_ < STARTUP_DURATION) {
        time_since_init_ += dt;
        double progress = std::min(1.0, time_since_init_ / STARTUP_DURATION);

        // Cubic easing: 3p^2 - 2p^3 (Smooth start and stop)
        double smooth_progress = progress * progress * (3.0 - 2.0 * progress);

        for (int i = 0; i < 4; ++i) {
            final_ecc(i) = start_ecc_angles_(i) + (raw_ecc_angles(i) - start_ecc_angles_(i)) * smooth_progress;
        }
    } else {
        // 5. Normal Trajectory Smoothing (Low Pass Filter) for teleop
        for (int i = 0; i < 4; ++i) {
            double smoothed = params_.filter_alpha * raw_ecc_angles(i) +
                              (1.0 - params_.filter_alpha) * prev_ecc_angles_(i);

            // Rate Limiter
            double delta = smoothed - prev_ecc_angles_(i);
            double max_delta = params_.max_ecc_vel * dt;
            delta = std::clamp(delta, -max_delta, max_delta);

            final_ecc(i) = prev_ecc_angles_(i) + delta;
        }
    }

    // 6. Update state and return
    prev_ecc_angles_ = final_ecc;

    return final_ecc;
}

// ============================================================================
// Gain Scheduling (Paper Eq 16-17)
// ============================================================================
void BalanceController::computeGainSchedule(
    double current_height, double& w_grf, double& w_ik) const
{
    // ================================================================
    // The paper uses a height-dependent blending:
    //
    // At maximum height (legs nearly vertical):
    //   - The eccentric arm Jacobian J_z approaches zero (singularity)
    //   - GRF-to-torque mapping is unreliable
    //   - -> IK position control dominates (w_ik → 1, w_grf → 0)
    //
    // At lower heights (legs angled):
    //   - Jacobian has good condition number
    //   - Dynamics-based GRF provides superior balance
    //   - -> GRF torque control dominates (w_grf → 1, w_ik → 0)
    //
    // height_ratio ∈ [0, 1]: 0 = min_height, 1 = max_height
    // ================================================================

    double min_h = 0.08;  // min operational height
    double max_h = 0.22;  // max operational height

    double height_ratio = (current_height - min_h) / (max_h - min_h);
    height_ratio = std::clamp(height_ratio, 0.0, 1.0);

    // Smooth transition using cosine blending
    // At height_ratio = 0 (low): w_grf = 1.0, w_ik = 0.0
    // At height_ratio = 1 (max): w_grf = 0.0, w_ik = 1.0
    double blend = 0.5 * (1.0 - std::cos(M_PI * height_ratio));

    w_ik  = blend * params_.gain_ik_scale;
    w_grf = (1.0 - blend) * params_.gain_grf_scale;

    // Normalize so weights sum to 1
    double total = w_grf + w_ik;
    if (total > 1e-6) {
        w_grf /= total;
        w_ik  /= total;
    } else {
        w_grf = 0.0;
        w_ik  = 1.0;
    }
}

// ============================================================================
// PD Controller for desired body acceleration
// ============================================================================
void BalanceController::computeDesiredAcceleration(
    double target_height, double target_roll, double target_pitch,
    const BodyState& body_state,
    Eigen::Vector3d& desired_linear_accel,
    Eigen::Vector3d& desired_angular_accel) const
{
    // ================================================================
    // Simple PD control to generate reference accelerations
    // for the SRBD model.
    //
    // Linear (Z-axis only for height control):
    //   a_z = kp * (h_target - h_current) + kd * (0 - v_z)
    //
    // Angular (roll and pitch):
    //   alpha_x = kp_roll  * (roll_target  - roll_current)  + kd_roll  * (0 - omega_x)
    //   alpha_y = kp_pitch * (pitch_target - pitch_current) + kd_pitch * (0 - omega_y)
    // ================================================================

    double h_current = body_state.position.z();
    double h_error = target_height - h_current;
    double vz = body_state.linear_velocity.z();

    desired_linear_accel = Eigen::Vector3d::Zero();
    desired_linear_accel.z() = params_.kp_height * h_error - params_.kd_height * vz;

    double roll_current  = body_state.roll();
    double pitch_current = body_state.pitch();
    double omega_x = body_state.angular_velocity.x();
    double omega_y = body_state.angular_velocity.y();

    desired_angular_accel = Eigen::Vector3d::Zero();
    desired_angular_accel.x() = params_.kp_roll  * (target_roll  - roll_current)  - params_.kd_roll  * omega_x;
    desired_angular_accel.y() = params_.kp_pitch * (target_pitch - pitch_current) - params_.kd_pitch * omega_y;
}

// ============================================================================
// Hybrid force-position control (Plan 3 upgrade)
// ============================================================================
ControlOutput BalanceController::updateHybrid(
    double target_height,
    double target_roll,
    double target_pitch,
    const Eigen::Vector4d& current_steer_angles,
    const Eigen::Vector4d& current_ecc_angles,
    const BodyState& body_state,
    const std::array<Eigen::Vector3d, NUM_LEGS>& contact_points,
    const std::array<bool, NUM_LEGS>& contact_valid,
    double dt,
    bool e_stop_active)
{
    ControlOutput output;

    if (e_stop_active) {
        initialized_ = false;
        prev_ecc_angles_ = current_ecc_angles;
        output.ecc_angles = current_ecc_angles;
        output.ecc_torques = Eigen::Vector4d::Zero();
        output.ecc_mode = ControlOutput::EccMode::POSITION;
        return output;
    }

    // ================================================================
    // 1. Compute IK position target (same as original update())
    // ================================================================
    double safe_roll = std::clamp(target_roll, -params_.max_roll, params_.max_roll);
    double safe_pitch = std::clamp(target_pitch, -params_.max_pitch, params_.max_pitch);

    Eigen::Vector4d ik_target = kinematics_->computePostureIK(
        target_height, safe_roll, safe_pitch, current_steer_angles, false);

    for (int i = 0; i < 4; ++i) {
        ik_target(i) = std::clamp(ik_target(i), params_.min_ecc_angle, params_.max_ecc_angle);
    }

    // ================================================================
    // 2. Compute GRF-based torque command
    // ================================================================
    // 2a. PD controller → desired acceleration
    Eigen::Vector3d desired_lin_accel, desired_ang_accel;
    computeDesiredAcceleration(target_height, safe_roll, safe_pitch,
                               body_state, desired_lin_accel, desired_ang_accel);

    // 2b. Set SRBD state and solve QP for optimal GRFs
    srbd_model_.setState(body_state, contact_points, contact_valid);

    Eigen::Vector4d optimal_grf = grf_optimizer_.computeOptimalGRF(
        srbd_model_, desired_lin_accel, desired_ang_accel);

    // 2c. Map GRFs to eccentric joint torques via Jacobian
    Eigen::Vector4d grf_torques = grf_optimizer_.grfToEccTorque(
        optimal_grf, current_ecc_angles, current_steer_angles,
        safe_roll, safe_pitch);

    // ================================================================
    // 3. Gain Scheduling — blend IK and GRF (Paper Eq 16-17)
    // ================================================================
    double w_grf, w_ik;
    computeGainSchedule(target_height, w_grf, w_ik);

    // ================================================================
    // 4. Apply startup trajectory (same smoothing as position mode)
    // ================================================================
    if (!initialized_) {
        start_ecc_angles_ = current_ecc_angles;
        prev_ecc_angles_ = current_ecc_angles;
        time_since_init_ = 0.0;
        initialized_ = true;
    }

    Eigen::Vector4d smoothed_ik = prev_ecc_angles_;

    if (time_since_init_ < STARTUP_DURATION) {
        time_since_init_ += dt;
        double progress = std::min(1.0, time_since_init_ / STARTUP_DURATION);
        double smooth_progress = progress * progress * (3.0 - 2.0 * progress);

        for (int i = 0; i < 4; ++i) {
            smoothed_ik(i) = start_ecc_angles_(i) +
                (ik_target(i) - start_ecc_angles_(i)) * smooth_progress;
        }
        // During startup, use pure IK (no GRF torques for safety)
        w_grf = 0.0;
        w_ik = 1.0;
    } else {
        for (int i = 0; i < 4; ++i) {
            double smoothed = params_.filter_alpha * ik_target(i) +
                              (1.0 - params_.filter_alpha) * prev_ecc_angles_(i);
            double delta = smoothed - prev_ecc_angles_(i);
            double max_delta = params_.max_ecc_vel * dt;
            delta = std::clamp(delta, -max_delta, max_delta);
            smoothed_ik(i) = prev_ecc_angles_(i) + delta;
        }
    }

    prev_ecc_angles_ = smoothed_ik;

    // ================================================================
    // 5. Assemble hybrid output
    // ================================================================
    output.ecc_angles  = smoothed_ik;
    output.ecc_torques = grf_torques;

    // Determine output mode based on gain scheduling
    if (w_grf < 0.01) {
        output.ecc_mode = ControlOutput::EccMode::POSITION;
    } else if (w_ik < 0.01) {
        output.ecc_mode = ControlOutput::EccMode::TORQUE;
    } else {
        output.ecc_mode = ControlOutput::EccMode::HYBRID;
    }

    return output;
}

}  // namespace controllers
}  // namespace robot_control
