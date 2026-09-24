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
    prev_compliance_shift_.setZero();
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
    const Eigen::Matrix3d& R_T,
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
        target_height, safe_roll, safe_pitch, current_steer_angles, R_T);

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
    // Gain Scheduling (Paper Eq 16-17)
    //
    // w_grf = sqrt(|h_max - h_cmd| / (h_max - h_mid))     (16)
    // w_ik = (1 + Kh * max(0, h_cmd - h_mid) / (h_max - h_mid)) * (1 + m_payload / m_body)    (17)
    // ================================================================

    // Eq 16-17 requires RobotParams
    RobotParams rp;
    double h_max = rp.max_height;
    double h_mid = 0.5 * (rp.min_height + rp.max_height);
    double h_cmd = std::clamp(current_height, rp.min_height, h_max);

    // To prevent division by zero, ensure h_max - h_mid > 0
    double den = std::max(1e-6, h_max - h_mid);

    // Eq 16: Augmented with baseline compliance floor (0.35) so shock
    // absorption is active even at max height (0.22m)
    double base_grf = 0.35;
    w_grf = base_grf + (1.0 - base_grf) * std::sqrt(std::abs(h_max - h_cmd) / den);
    w_grf = std::clamp(w_grf, 0.0, 1.0);

    // Eq 17
    double Kh = 7.0; // from paper
    double mass_ratio = 1.0 + (rp.payload_mass / rp.body_mass);
    w_ik = (1.0 + Kh * std::max(0.0, h_cmd - h_mid) / den) * mass_ratio;

    // The paper does not explicitly normalize, but they act as weights.
    // In Eq 9: tau = w_grf * tau_grf + w_ik * tau_ik.
    // So we apply them as scales directly. No normalization here.
}

// ============================================================================
// PD Controller for desired body acceleration
// ============================================================================
void BalanceController::computeDesiredAcceleration(
    double target_height, double target_roll, double target_pitch,
    const Eigen::Vector4d& current_ecc_angles,
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
    // Physical ground clearance is computed directly from forward kinematics
    // of the 4 legs: H_i = posture_z_offset + l_ecc * cos(q_ecc,i) + r_wheel.
    // This provides exact, drift-free height feedback and completely prevents
    // open-loop EKF acceleration integration drift from driving height error
    // to infinity and pegging compliance shifts.
    // ================================================================

    double h_current = 0.0;
    for (int i = 0; i < 4; ++i) {
        h_current += robot_params_.posture_z_offset + robot_params_.l_ecc * std::cos(current_ecc_angles(i)) + robot_params_.r_wheel;
    }
    h_current /= 4.0;

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

double BalanceController::computeAdaptiveNominalHeight(
    double target_height, double pitch_slope, double roll_slope) const
{
    // Required vertical difference to maintain level chassis
    double delta_z_req = robot_params_.length_x * std::abs(std::sin(pitch_slope))
                       + robot_params_.width_y  * std::abs(std::sin(roll_slope));

    // Slope Headroom Management (Phase 2.7):
    // Rather than subtracting delta_z_req directly (which caused a double-subtraction
    // when computePostureIK also subtracts delta_z_req via R_T), we define the admissible
    // envelope for the chassis reference height:
    //   safe_ceiling = max_safe_leg_length (0.205m) - delta_z_req (to avoid singularity at q->0)
    //   safe_floor   = min_safe_leg_length (0.115m) + delta_z_req (to keep crouch |q| <= 115 deg)
    double safe_ceiling = 0.205 - delta_z_req;
    double safe_floor   = 0.115 + delta_z_req;

    double h_min_bound = std::min(safe_floor, safe_ceiling);
    double h_max_bound = std::max(safe_floor, safe_ceiling);
    double h_adapt_raw = std::clamp(target_height, h_min_bound, h_max_bound);

    // Low-pass filter to prevent high-frequency height bobbing on micro-bumps
    double alpha_h = 0.15;
    double h_adapt = alpha_h * h_adapt_raw + (1.0 - alpha_h) * prev_h_adapt_;
    prev_h_adapt_ = h_adapt;
    return h_adapt;
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
    const Eigen::Vector4d& current_ecc_efforts,
    const BodyState& body_state,
    const std::array<Eigen::Vector3d, NUM_LEGS>& contact_points,
    const std::array<bool, NUM_LEGS>& contact_valid,
    const TerrainState& terrain_state,
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
    // 1. Slope Headroom Management & Fast Residual Attitude PD (Phase 2)
    // ================================================================
    double safe_roll  = std::clamp(target_roll,  -params_.max_roll,  params_.max_roll);
    double safe_pitch = std::clamp(target_pitch, -params_.max_pitch, params_.max_pitch);

    // Compute dynamic adaptive height to preserve downhill leg stroke headroom
    double h_adapt = computeAdaptiveNominalHeight(
        target_height, terrain_state.pitch_slope, terrain_state.roll_slope);

    // Fast IMU Attitude Residual Feedback:
    // Terrain slope feedforward is already handled geometrically via terrain_state.rotation in computePostureIK.
    // The attitude PD loop only eliminates dynamic disturbances and residual unmodeled error.
    double roll_current  = body_state.roll();
    double pitch_current = body_state.pitch();
    double omega_x = body_state.angular_velocity.x();
    double omega_y = body_state.angular_velocity.y();

    double roll_err  = roll_current - safe_roll;
    double pitch_err = pitch_current - safe_pitch;

    // Command residual corrections (expanded to ±0.18 rad ≈ ±10.3 deg) to ensure adequate leveling authority
    double roll_corr  = -(params_.kp_attitude_roll  * roll_err  + params_.kd_attitude_roll  * omega_x);
    double pitch_corr = -(params_.kp_attitude_pitch * pitch_err + params_.kd_attitude_pitch * omega_y);
    roll_corr  = std::clamp(roll_corr,  -0.18, 0.18);
    pitch_corr = std::clamp(pitch_corr, -0.18, 0.18);

    double active_ik_roll  = std::clamp(safe_roll  + roll_corr,  -params_.max_roll,  params_.max_roll);
    double active_ik_pitch = std::clamp(safe_pitch + pitch_corr, -params_.max_pitch, params_.max_pitch);

    Eigen::Vector4d ik_target = kinematics_->computePostureIK(
        h_adapt, active_ik_roll, active_ik_pitch, current_steer_angles, terrain_state.rotation);

    for (int i = 0; i < 4; ++i) {
        ik_target(i) = std::clamp(ik_target(i), params_.min_ecc_angle, params_.max_ecc_angle);
        // Singularity avoidance: enforce minimum bend of 15 deg (0.26 rad)
        if (i == FL || i == FR) {
            ik_target(i) = std::max(0.26, ik_target(i));
        } else {
            ik_target(i) = std::min(-0.26, ik_target(i));
        }
    }

    // ================================================================
    // 2. Compute GRF-based torque command
    // ================================================================
    // 2a. PD controller → desired acceleration (using h_adapt)
    Eigen::Vector3d desired_lin_accel, desired_ang_accel;
    computeDesiredAcceleration(h_adapt, safe_roll, safe_pitch,
                               current_ecc_angles, body_state,
                               desired_lin_accel, desired_ang_accel);

    // 2b. Set SRBD state and solve QP for optimal GRFs
    srbd_model_.setState(body_state, contact_points, contact_valid);

    Eigen::Vector4d optimal_grf = grf_optimizer_.computeOptimalGRF(
        srbd_model_, desired_lin_accel, desired_ang_accel);

    // 2c. Map GRFs to eccentric joint torques via Jacobian
    Eigen::Vector4d grf_torques = grf_optimizer_.grfToEccTorque(
        optimal_grf, current_ecc_angles, current_steer_angles,
        safe_roll, safe_pitch);

    // ================================================================
    // 3. Gain Scheduling — blend IK and GRF (using h_adapt)
    // ================================================================
    double w_grf, w_ik;
    computeGainSchedule(h_adapt, w_grf, w_ik);

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
    // 5. Hybrid Control Strategy (Eq 9 in paper) & Admittance Compliance
    //    tau_ecc,i^* = w_grf * tau_grf,i + w_ik * tau_ik,i
    // ================================================================
    Eigen::Vector4d tau_ik = Eigen::Vector4d::Zero();
    Eigen::Vector4d tau_hybrid = Eigen::Vector4d::Zero();
    Eigen::Vector4d compliant_ecc_pos = smoothed_ik;

    for (int i = 0; i < NUM_LEGS; ++i) {
        // tau_ik: PD position tracking torque (Section III.E.1 & III.E.3)
        double pos_err = smoothed_ik(i) - current_ecc_angles(i);
        tau_ik(i) = params_.kp_ik * pos_err;

        // Eq 9: Hybrid torque combination
        tau_hybrid(i) = w_grf * grf_torques(i) + w_ik * tau_ik(i);

        // ----------------------------------------------------------------
        // Bilateral Force-Balanced Admittance Compliance (Phase 2.7 Upgrade)
        //
        // Ground Interaction:
        // Use the QP-optimized normal force optimal_grf(i) to compute the exact
        // dynamic equilibrium supporting torque for leg i:
        //   nom_torque = |J_z,i * optimal_grf(i)|
        //
        // 1. When running over an obstacle/bump:
        //    delta_tau = |tau_meas| - nom_torque > compliance_deadband
        //    -> Retract leg (raw_shift in sign_leg direction) to absorb shock.
        //
        // 2. When wheel is unloaded or airborne (cleared bump, dip, or step-off):
        //    delta_tau < -compliance_deadband
        //    -> Gently extend leg DOWNWARDS (-sign_leg direction) to seek ground contact!
        //
        // 3. Fast Touch-Loss Reset:
        //    If wheel is completely airborne (|tau_meas| < 0.35 * nom_torque),
        //    increase filter alpha to touch_loss_decay_alpha to rapidly drop the suspended wheel.
        // ----------------------------------------------------------------
        double sign_leg = (smoothed_ik(i) >= 0.0) ? 1.0 : -1.0;
        double J_z = grf_optimizer_.computeVerticalJacobian(
            i, current_ecc_angles(i), current_steer_angles(i), safe_roll, safe_pitch);
        double dyn_fz = (optimal_grf(i) > 1e-3) ? optimal_grf(i) : (robot_params_.total_mass * 9.81 / NUM_LEGS);
        double nom_torque = std::abs(J_z * dyn_fz);
        double meas_torque = std::abs(current_ecc_efforts(i));
        double delta_tau = meas_torque - nom_torque;

        double raw_shift = 0.0;
        if (delta_tau > params_.compliance_deadband) {
            // Impact / excess load -> Retract
            double excess = delta_tau - params_.compliance_deadband;
            raw_shift = sign_leg * (w_grf * excess) / (params_.compliance_k + 1e-4);
            raw_shift = std::clamp(raw_shift, -params_.max_compliance_shift, params_.max_compliance_shift);
        } else if (delta_tau < -params_.compliance_deadband) {
            // Airborne / lost load -> Extend downwards to seek ground
            double deficit = -delta_tau - params_.compliance_deadband;
            raw_shift = -sign_leg * (w_grf * deficit) / (params_.ground_seeking_k + 1e-4);
            raw_shift = std::clamp(raw_shift, -params_.ground_seeking_max_shift, params_.ground_seeking_max_shift);
        }

        // Virtual damping with touch-loss reset acceleration
        double effective_alpha = params_.compliance_filter_alpha;
        if (meas_torque < 0.35 * nom_torque) {
            effective_alpha = params_.touch_loss_decay_alpha;
        }

        prev_compliance_shift_(i) = effective_alpha * raw_shift
                                  + (1.0 - effective_alpha) * prev_compliance_shift_(i);

        compliant_ecc_pos(i) = std::clamp(smoothed_ik(i) + prev_compliance_shift_(i),
                                          params_.min_ecc_angle, params_.max_ecc_angle);
    }

    output.ecc_angles  = compliant_ecc_pos;
    output.ecc_torques = tau_hybrid;
    output.ecc_mode    = ControlOutput::EccMode::HYBRID;

    return output;
}

}  // namespace controllers
}  // namespace robot_control
