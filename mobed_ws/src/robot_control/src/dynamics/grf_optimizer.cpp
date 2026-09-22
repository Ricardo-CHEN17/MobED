#include "robot_control/dynamics/grf_optimizer.hpp"
#include "robot_control/dynamics/qp_solver.hpp"
#include <cmath>

namespace robot_control {
namespace dynamics {

GrfOptimizer::GrfOptimizer(const RobotParams& params)
    : params_(params)
{
}

Eigen::Vector4d GrfOptimizer::computeOptimalGRF(
    const SrbdModel& srbd,
    const Eigen::Vector3d& desired_linear_accel,
    const Eigen::Vector3d& desired_angular_accel)
{
    Eigen::Vector4d grf_result = Eigen::Vector4d::Zero();

    auto contact_indices = srbd.getContactLegIndices();
    int n_contact = static_cast<int>(contact_indices.size());

    if (n_contact == 0) {
        return grf_result;  // all legs airborne — no GRF possible
    }

    // ================================================================
    // Build dynamics constraints: A * f_z = b
    // ================================================================
    Eigen::MatrixXd A_dyn;
    Eigen::VectorXd b_dyn;
    srbd.buildDynamicsConstraints(desired_linear_accel, desired_angular_accel,
                                  A_dyn, b_dyn);

    // Extract f_prev for the active contact legs
    Eigen::VectorXd f_prev(n_contact);
    for (int i = 0; i < n_contact; ++i) {
        f_prev(i) = prev_grf_result_(contact_indices[i]);
    }

    // ================================================================
    // Build QP cost function (Eq 13 in the paper)
    //
    //   min  || A_dyn * f - b_dyn ||_Q^2 + || f ||_R^2 + || f - f_prev ||_P^2
    //
    // This is equivalent to min 0.5 * f^T * H * f + f_cost^T * f
    // where H = 2 * (weight_Q * A_dyn^T * A_dyn + (weight_R + weight_P) * I)
    //       f_cost = 2 * (-weight_Q * A_dyn^T * b_dyn - weight_P * f_prev)
    // ================================================================
    Eigen::MatrixXd H = 2.0 * (weight_Q * A_dyn.transpose() * A_dyn + 
                               (weight_R + weight_P) * Eigen::MatrixXd::Identity(n_contact, n_contact));
    
    Eigen::VectorXd f_cost = 2.0 * (-weight_Q * A_dyn.transpose() * b_dyn - weight_P * f_prev);

    // ================================================================
    // Assemble QP problem
    // ================================================================
    QpProblem qp;
    qp.H  = H;
    qp.f  = f_cost;
    qp.lb = Eigen::VectorXd::Zero(n_contact);                           // f_z >= 0 (no pull)
    qp.ub = Eigen::VectorXd::Constant(n_contact, max_grf_per_leg);      // f_z <= f_max
    
    // Dynamics are now soft constraints in the cost function
    qp.A_eq = Eigen::MatrixXd(0, n_contact);
    qp.b_eq = Eigen::VectorXd(0);

    // ================================================================
    // Solve QP
    // ================================================================
    QpResult result = solveQP(qp, 100, 1e-6);

    // Map contact-indexed solution back to 4-leg array
    if (result.converged) {
        for (int j = 0; j < n_contact; ++j) {
            int leg_idx = contact_indices[j];
            grf_result(leg_idx) = std::max(0.0, result.x(j));
        }
    } else {
        // Fallback: equal weight distribution if QP fails
        double f_fallback = params_.total_mass * params_.gravity / n_contact;
        for (int leg_idx : contact_indices) {
            grf_result(leg_idx) = f_fallback;
        }
    }

    prev_grf_result_ = grf_result;
    return grf_result;
}

double GrfOptimizer::computeVerticalJacobian(
    int leg_index,
    double q_ecc,
    double q_steer,
    double roll,
    double pitch) const
{
    // ================================================================
    // Vertical Jacobian: dz_foot / dq_ecc
    //
    // From the posture IK (mobed_kinematics.cpp), the height equation is:
    //   height = -r31*px - r32*py + r33*r_wheel + z_offset + A*cos(q) + B*sin(q)
    //
    // where:
    //   r31 = -sin(pitch)
    //   r32 = sin(roll)*cos(pitch)
    //   r33 = cos(roll)*cos(pitch)
    //   A = l_ecc * (r31*cos(q_steer) + r32*sin(q_steer))
    //   B = -l_ecc * r33
    //
    // z_world = const + A*sin(q_ecc) + B*cos(q_ecc)
    // Therefore:
    //   dz/dq_ecc = d(A*sin(q) + B*cos(q))/dq = A*cos(q_ecc) - B*sin(q_ecc)
    // ================================================================

    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cr = std::cos(roll);
    double sr = std::sin(roll);

    // leg_index reserved for future per-leg geometric offsets
    (void)leg_index;

    double r31 = -sp;
    double r32 = sr * cp;
    double r33 = cr * cp;

    double A = params_.l_ecc * (r31 * std::cos(q_steer) + r32 * std::sin(q_steer));
    double B = -params_.l_ecc * r33;

    // dz/dq_ecc = A*cos(q_ecc) - B*sin(q_ecc)
    double J_z = A * std::cos(q_ecc) - B * std::sin(q_ecc);

    return J_z;
}

Eigen::Vector4d GrfOptimizer::grfToEccTorque(
    const Eigen::Vector4d& grf,
    const Eigen::Vector4d& ecc_angles,
    const Eigen::Vector4d& steer_angles,
    double target_roll,
    double target_pitch) const
{
    // ================================================================
    // Map vertical GRF to eccentric joint torque via Jacobian transpose:
    //   tau_ecc,i = J_z,i^T * f_z,i
    //
    // The sign convention: a positive f_z (upward ground push) requires
    // the eccentric joint to exert a torque that resists downward motion.
    // ================================================================

    Eigen::Vector4d tau_ecc = Eigen::Vector4d::Zero();

    for (int i = 0; i < NUM_LEGS; ++i) {
        if (std::abs(grf(i)) < 1e-6) continue;  // skip non-contact legs

        double J_z = computeVerticalJacobian(
            i, ecc_angles(i), steer_angles(i), target_roll, target_pitch);

        // tau = J^T * f  (scalar for single-DOF eccentric)
        tau_ecc(i) = J_z * grf(i);
    }

    return tau_ecc;
}

}  // namespace dynamics
}  // namespace robot_control
