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

void BalanceController::setPayloadMass(double mass) {
    robot_params_.payload_mass = mass;
    robot_params_.total_mass = robot_params_.body_mass + mass;
    srbd_model_.setPayload(mass, robot_params_.payload_com_z_offset);
    grf_optimizer_.setRobotParams(robot_params_);
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

    // 2. Math Kinematics (Exact non-linear projection with posture configuration)
    Eigen::Vector4d raw_ecc_angles = kinematics_->computePostureIK(
        target_height, safe_roll, safe_pitch, current_steer_angles, R_T, current_config_);

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

    double h_max = robot_params_.max_height;
    double h_mid = 0.5 * (robot_params_.min_height + robot_params_.max_height);
    double h_cmd = std::clamp(current_height, robot_params_.min_height, h_max);

    // To prevent division by zero, ensure h_max - h_mid > 0
    double den = std::max(1e-6, h_max - h_mid);

    // Eq 16: Exact paper linear formula. Range is [0, 2.0]
    // At h_cmd = h_max (0.22m): w_grf = 0.0 (converges smoothly to 0 near singularity)
    // At h_cmd = h_mid (0.15m): w_grf = 1.0
    // At h_cmd = h_min (0.08m): w_grf = 2.0
    w_grf = std::abs(h_max - h_cmd) / den;

    // Eq 17: Kh = 7.0 from paper, scaled by dynamic payload mass ratio
    double Kh = 7.0; // from paper
    double mass_ratio = 1.0 + (robot_params_.payload_mass / robot_params_.body_mass);
    w_ik = (1.0 + Kh * std::max(0.0, h_cmd - h_mid) / den) * mass_ratio;
}

// ============================================================================
// PD Controller for desired body acceleration
// ============================================================================
void BalanceController::computeDesiredAcceleration(
    double target_height, double target_roll, double target_pitch,
    const Eigen::Vector4d& current_ecc_angles,
    const BodyState& body_state,
    Eigen::Vector3d& desired_linear_accel,
    Eigen::Vector3d& desired_angular_accel,
    const Eigen::Vector3d& cmd_vel,
    double dt) const
{
    // ================================================================
    // Reference acceleration generation for SRBD model (Paper Eq 11-12)
    //
    // Linear (Z-axis only for height control):
    //   a_z = kp * (h_target - h_current) + kd * (0 - v_z)
    //
    // Physical ground clearance is computed directly from forward kinematics
    // of the 4 legs: H_i = posture_z_offset + l_ecc * cos(q_ecc,i) + r_wheel.
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

    // Active longitudinal and centripetal acceleration feedforward (Paper Eq 11):
    // Preemptively cancels pitch load transfer from d(vx)/dt and roll load transfer from wz * vx
    double ax_cmd = 0.0;
    if (dt > 1e-4) {
        ax_cmd = (cmd_vel.x() - prev_cmd_vel_.x()) / dt;
        ax_cmd = std::clamp(ax_cmd, -3.0, 3.0);
    }
    prev_cmd_vel_ = cmd_vel;

    double ay_cmd = cmd_vel.z() * cmd_vel.x();
    ay_cmd = std::clamp(ay_cmd, -3.0, 3.0);

    // Transform planar acceleration from vehicle heading frame to world frame
    double yaw_curr = body_state.yaw();
    Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(yaw_curr, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d a_planar_heading(ax_cmd, ay_cmd, 0.0);
    desired_linear_accel.head<2>() = (R_yaw * a_planar_heading).head<2>();

    // ================================================================
    // SO(3) Lie Algebra Logarithmic Map (Paper Eq 12):
    //   alpha^d = K_{P,T} * log(R^d * R^T) + K_{D,T} * (omega^d - omega)
    //
    // Eliminates gimbal lock and non-commutative Euler angle coupling
    // across the full 19 deg roll x 23 deg pitch workspace.
    // ================================================================
    // Desired orientation R^d (ZYX Tait-Bryan: keeps current yaw heading, commands target roll & pitch)
    Eigen::Matrix3d R_curr = body_state.orientation.toRotationMatrix();
    Eigen::Matrix3d R_d = (Eigen::AngleAxisd(yaw_curr, Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd(target_pitch, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(target_roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

    // Relative rotation error in world frame: R_err = R^d * R^T
    Eigen::Matrix3d R_err = R_d * R_curr.transpose();

    // Matrix logarithm map: e_R = log(R_err) \in so(3)
    double trace = R_err.trace();
    double cos_theta = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
    double theta = std::acos(cos_theta);
    Eigen::Vector3d vee(
        R_err(2, 1) - R_err(1, 2),
        R_err(0, 2) - R_err(2, 0),
        R_err(1, 0) - R_err(0, 1)
    );
    Eigen::Vector3d e_R;
    if (theta < 1e-5) {
        e_R = 0.5 * (1.0 + theta * theta / 6.0) * vee;
    } else {
        e_R = (0.5 * theta / std::sin(theta)) * vee;
    }

    // Express rotation error and angular velocity in robot heading frame (R_z(yaw))
    // so gains kp_roll, kp_pitch apply along vehicle longitudinal and lateral axes
    Eigen::Vector3d e_R_heading = R_yaw.transpose() * e_R;
    Eigen::Vector3d omega_heading = R_yaw.transpose() * body_state.angular_velocity;

    Eigen::Vector3d alpha_heading = Eigen::Vector3d::Zero();
    alpha_heading.x() = params_.kp_roll  * e_R_heading.x() - params_.kd_roll  * omega_heading.x();
    alpha_heading.y() = params_.kp_pitch * e_R_heading.y() - params_.kd_pitch * omega_heading.y();

    // Transform desired angular acceleration back to world frame for SRBD
    desired_angular_accel = R_yaw * alpha_heading;
}

double BalanceController::computeAdaptiveNominalHeight(
    double target_height, double pitch_slope, double roll_slope, double commanded_roll) const
{
    // Required vertical difference to maintain level chassis and accommodate active banking:
    // Take the maximum of terrain roll slope and commanded active bank roll
    double effective_roll = std::max(std::abs(std::sin(roll_slope)), std::abs(std::sin(commanded_roll)));
    double delta_z_req = robot_params_.length_x * std::abs(std::sin(pitch_slope))
                       + robot_params_.width_y  * effective_roll;

    // Slope & Bank Headroom Management (Phase 2.7):
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
    bool e_stop_active,
    const Eigen::Vector3d& cmd_vel)
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

    // Compute dynamic adaptive height to preserve downhill leg stroke headroom and bank angle stroke
    double h_adapt = computeAdaptiveNominalHeight(
        target_height, terrain_state.pitch_slope, terrain_state.roll_slope, safe_roll);

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
        h_adapt, active_ik_roll, active_ik_pitch, current_steer_angles, terrain_state.rotation, current_config_);

    for (int i = 0; i < 4; ++i) {
        ik_target(i) = std::clamp(ik_target(i), params_.min_ecc_angle, params_.max_ecc_angle);
        // Singularity avoidance: enforce minimum bend of 15 deg (0.26 rad) along the configuration sign branch
        bool is_front = (i == FL || i == FR);
        bool target_positive = true;
        if (current_config_ == PostureConfiguration::OUTWARD) {
            target_positive = is_front;
        } else if (current_config_ == PostureConfiguration::INWARD) {
            target_positive = !is_front;
        } else if (current_config_ == PostureConfiguration::SPIN) {
            target_positive = (i == FL || i == RL);
        }

        if (target_positive) {
            ik_target(i) = std::max(0.26, ik_target(i));
        } else {
            ik_target(i) = std::min(-0.26, ik_target(i));
        }
    }

    // ================================================================
    // 2. Compute GRF-based torque command
    // ================================================================
    // 2a. PD controller → desired acceleration (using h_adapt) + inertial feedforward
    Eigen::Vector3d desired_lin_accel, desired_ang_accel;
    computeDesiredAcceleration(h_adapt, safe_roll, safe_pitch,
                               current_ecc_angles, body_state,
                               desired_lin_accel, desired_ang_accel,
                               cmd_vel, dt);

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

        // Decouple virtual compliance authority from w_grf so shock absorption is maintained
        // even when w_grf approaches 0 near max height (0.22m)
        double compliance_scale = std::clamp(w_grf, 0.35, 1.0);

        double raw_shift = 0.0;
        if (delta_tau > params_.compliance_deadband) {
            // Impact / excess load -> Retract
            double excess = delta_tau - params_.compliance_deadband;
            raw_shift = sign_leg * (compliance_scale * excess) / (params_.compliance_k + 1e-4);
            raw_shift = std::clamp(raw_shift, -params_.max_compliance_shift, params_.max_compliance_shift);
        } else if (delta_tau < -params_.compliance_deadband) {
            // Airborne / lost load -> Extend downwards to seek ground
            double deficit = -delta_tau - params_.compliance_deadband;
            raw_shift = -sign_leg * (compliance_scale * deficit) / (params_.ground_seeking_k + 1e-4);
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
