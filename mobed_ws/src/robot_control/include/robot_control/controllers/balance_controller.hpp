#pragma once

#include <Eigen/Dense>
#include <memory>
#include "robot_control/kinematics/mobed_kinematics.hpp"
#include "robot_control/core/robot_params.hpp"
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/dynamics/srbd_model.hpp"
#include "robot_control/dynamics/grf_optimizer.hpp"

namespace robot_control {
namespace controllers {

struct BalanceControllerParams {
    double min_ecc_angle = -1.745; // ~ -100 deg
    double max_ecc_angle = 1.745;  // ~ 100 deg

    double max_roll = 0.331;       // ~ 19 deg
    double max_pitch = 0.401;      // ~ 23 deg

    double filter_alpha = 0.02;    // LPF coefficient for normal operation
    double max_ecc_vel = 2.0;      // rad/s

    // ---- Gain Scheduling (Paper Eq 16-17) ----
    // Controls the blend between dynamics-based torque (GRF) and
    // kinematics-based position (IK) commands.
    //
    // At max height (legs vertical), IK dominates because the Jacobian
    // is near-singular and GRF mapping is unreliable.
    // At lower heights, GRF dominates for superior dynamic balance.
    //
    // w_grf = (1 - height_ratio) * gain_grf_scale
    // w_ik  = height_ratio * gain_ik_scale
    // where height_ratio = (current_height - min_height) / (max_height - min_height)
    double gain_grf_scale = 1.0;   // scaling factor for GRF weight
    double gain_ik_scale  = 1.0;   // scaling factor for IK weight

    // ---- PD gains for desired body acceleration ----
    // Used to compute the desired acceleration fed into the SRBD/GRF system
    double kp_height = 200.0;  // N/m (stiffness for height tracking)
    double kd_height = 40.0;   // N·s/m (damping for height tracking)
    double kp_roll   = 100.0;  // Nm/rad
    double kd_roll   = 20.0;   // Nm·s/rad
    double kp_pitch  = 100.0;  // Nm/rad
    double kd_pitch  = 20.0;   // Nm·s/rad

    // ---- IK position PD gains for tau_ik in Eq 9 ----
    double kp_ik = 150.0;      // Nm/rad
    double kd_ik = 15.0;       // Nm·s/rad

    // ---- Admittance / Compliance parameters (Section III.E & III.F) ----
    // Complies with dynamic disturbance force: delta_f_z = f_z - (m*g/4)
    double compliance_k = 1000.0;       // N/rad (admittance stiffness: delta_q = w_grf * delta_f_z / compliance_k)
    double max_compliance_shift = 0.35; // rad (~20 deg), maximum compliance adjustment
};

class BalanceController {
public:
    BalanceController(const BalanceControllerParams& params,
                      std::shared_ptr<kinematics::MobedKinematics> kinematics);

    /**
     * @brief Update the balance controller step (position-only mode, backward compatible).
     *
     * This is the original interface that outputs pure position commands.
     * Used when the system is still in position-control mode.
     *
     * @param target_height Target base height
     * @param target_roll Target roll angle (includes bank angle from driving)
     * @param target_pitch Target pitch angle
     * @param current_steer_angles Required for exact posture projection
     * @param current_ecc_angles Required for initialization and E-Stop freezing
     * @param dt Time step
     * @param e_stop_active Emergency stop flag
     * @return Eigen::Vector4d Target eccentric angles (position command)
     */
    Eigen::Vector4d update(
        double target_height,
        double target_roll,
        double target_pitch,
        const Eigen::Vector4d& current_steer_angles,
        const Eigen::Vector4d& current_ecc_angles,
        const Eigen::Matrix3d& R_T,
        double dt,
        bool e_stop_active = false);

    ControlOutput updateHybrid(
        double target_height,
        double target_roll,
        double target_pitch,
        const Eigen::Vector4d& current_steer_angles,
        const Eigen::Vector4d& current_ecc_angles,
        const BodyState& body_state,
        const std::array<Eigen::Vector3d, NUM_LEGS>& contact_points,
        const std::array<bool, NUM_LEGS>& contact_valid,
        const Eigen::Matrix3d& R_T,
        double dt,
        bool e_stop_active = false);

    void reset() { initialized_ = false; }

private:
    BalanceControllerParams params_;
    RobotParams robot_params_;
    std::shared_ptr<kinematics::MobedKinematics> kinematics_;

    // Dynamics modules
    dynamics::SrbdModel srbd_model_;
    dynamics::GrfOptimizer grf_optimizer_;

    Eigen::Vector4d prev_ecc_angles_;
    Eigen::Vector4d start_ecc_angles_;
    bool initialized_ = false;
    double time_since_init_ = 0.0;
    const double STARTUP_DURATION = 3.0; // 3 seconds to smoothly stand up

    /**
     * @brief Compute gain scheduling weights (Paper Eq 16-17).
     *
     * @param current_height  Current chassis height (m)
     * @param[out] w_grf  Weight for GRF-based torque command
     * @param[out] w_ik   Weight for IK-based position command
     */
    void computeGainSchedule(double current_height, double& w_grf, double& w_ik) const;

    /**
     * @brief Compute desired body acceleration from PD control.
     *
     * Generates the reference acceleration for the SRBD model based on
     * the tracking error between target and estimated body state.
     *
     * @param target_height Target height
     * @param target_roll Target roll
     * @param target_pitch Target pitch
     * @param body_state Current estimated body state
     * @param[out] desired_linear_accel Linear acceleration command (world frame)
     * @param[out] desired_angular_accel Angular acceleration command (world frame)
     */
    void computeDesiredAcceleration(
        double target_height, double target_roll, double target_pitch,
        const BodyState& body_state,
        Eigen::Vector3d& desired_linear_accel,
        Eigen::Vector3d& desired_angular_accel) const;
};

}  // namespace controllers
}  // namespace robot_control
