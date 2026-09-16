#pragma once

#include <Eigen/Dense>
#include <array>
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"
#include "robot_control/dynamics/srbd_model.hpp"

namespace robot_control {
namespace dynamics {

/**
 * @brief Ground Reaction Force (GRF) optimizer for MobED.
 *
 * Implements the QP-based GRF optimization from the MobED paper (Eq 13-14).
 * Given the desired body acceleration (from a PD controller), computes the
 * optimal vertical ground reaction forces for each leg, then maps them to
 * eccentric joint torques via the leg Jacobian transpose.
 *
 * QP formulation:
 *   min  sum_i w_i * (f_z,i - f_ref)^2 + alpha * sum_i f_z,i^2
 *   s.t. A * f_z = b                    (Newton-Euler dynamics)
 *        0 <= f_z,i <= f_max            (physical bounds)
 *
 * where:
 *   f_ref = m*g / n_contact (equal weight distribution reference)
 *   w_i   = weighting per leg (can prioritize certain legs)
 *   alpha = regularization weight (small, prevents extreme forces)
 */
class GrfOptimizer {
public:
    explicit GrfOptimizer(const RobotParams& params = RobotParams());

    /**
     * @brief Compute optimal vertical GRFs for all legs.
     *
     * @param srbd      SRBD model with current state already set
     * @param desired_linear_accel   Desired body linear acceleration (m/s^2)
     * @param desired_angular_accel  Desired body angular acceleration (rad/s^2)
     * @return Eigen::Vector4d  Optimal vertical GRF per leg [FL, FR, RL, RR] (N).
     *         Non-contact legs will have f_z = 0.
     */
    Eigen::Vector4d computeOptimalGRF(
        const SrbdModel& srbd,
        const Eigen::Vector3d& desired_linear_accel,
        const Eigen::Vector3d& desired_angular_accel) const;

    /**
     * @brief Map vertical GRFs to eccentric joint torques via Jacobian transpose.
     *
     * For each leg, computes:
     *   tau_ecc,i = J_z,i^T * f_z,i
     *
     * where J_z,i = dz_foot / dq_ecc,i is the vertical component of the
     * leg Jacobian w.r.t. the eccentric joint angle.
     *
     * @param grf              Vertical GRF per leg [FL, FR, RL, RR] (N)
     * @param ecc_angles       Current eccentric joint angles (rad)
     * @param steer_angles     Current steering joint angles (rad)
     * @param target_roll      Current body roll (rad)
     * @param target_pitch     Current body pitch (rad)
     * @return Eigen::Vector4d  Eccentric joint torques [FL, FR, RL, RR] (Nm)
     */
    Eigen::Vector4d grfToEccTorque(
        const Eigen::Vector4d& grf,
        const Eigen::Vector4d& ecc_angles,
        const Eigen::Vector4d& steer_angles,
        double target_roll,
        double target_pitch) const;

    /**
     * @brief Compute the vertical Jacobian dz/dq_ecc for a single leg.
     *
     * @param leg_index     Which leg (FL, FR, RL, RR)
     * @param q_ecc         Eccentric joint angle (rad)
     * @param q_steer       Steering joint angle (rad)
     * @param roll          Body roll angle (rad)
     * @param pitch         Body pitch angle (rad)
     * @return double       dz_foot / dq_ecc (m/rad)
     */
    double computeVerticalJacobian(
        int leg_index,
        double q_ecc,
        double q_steer,
        double roll,
        double pitch) const;

    // Tuning parameters
    double regularization_weight = 0.001;  // alpha: prevents extreme force solutions
    double max_grf_per_leg = 500.0;        // N, maximum force per leg

private:
    RobotParams params_;
};

}  // namespace dynamics
}  // namespace robot_control
