#include "robot_control/dynamics/srbd_model.hpp"

namespace robot_control {
namespace dynamics {

SrbdModel::SrbdModel(const RobotParams& params)
    : params_(params)
{
    contact_valid_.fill(true);
    for (auto& cp : contact_points_) {
        cp.setZero();
    }
}

Eigen::Matrix3d SrbdModel::skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<    0.0, -v.z(),  v.y(),
          v.z(),    0.0, -v.x(),
         -v.y(),  v.x(),    0.0;
    return S;
}

void SrbdModel::setState(
    const BodyState& body_state,
    const std::array<Eigen::Vector3d, NUM_LEGS>& contact_points,
    const std::array<bool, NUM_LEGS>& contact_valid)
{
    body_state_ = body_state;
    contact_points_ = contact_points;
    contact_valid_ = contact_valid;
}

int SrbdModel::getNumContactLegs() const {
    int count = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (contact_valid_[i]) ++count;
    }
    return count;
}

std::vector<int> SrbdModel::getContactLegIndices() const {
    std::vector<int> indices;
    indices.reserve(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (contact_valid_[i]) {
            indices.push_back(i);
        }
    }
    return indices;
}

void SrbdModel::buildDynamicsConstraints(
    const Eigen::Vector3d& desired_linear_accel,
    const Eigen::Vector3d& desired_angular_accel,
    Eigen::MatrixXd& A,
    Eigen::VectorXd& b) const
{
    // ================================================================
    // Newton-Euler Equations (Paper Eq 10-12)
    //
    // Newton (linear):
    //   m * a_d = sum_i(f_i) + m * g
    //   => sum_i(f_i) = m * (a_d - g)
    //
    // Since we only optimize vertical GRF (f_z,i), each contact force
    // is f_i = [0, 0, f_z,i]^T (assuming flat contact, vertical only).
    //
    // Euler (angular):
    //   I * alpha_d + omega x (I * omega) = sum_i(r_i x f_i)
    //   => sum_i(r_i x f_i) = I * alpha_d + omega x (I * omega)
    //
    // where r_i = p_contact,i - p_CoM (vector from CoM to contact point)
    //
    // Assemble into: A * f_z = b
    //   A is 6 x n_contact
    //   b is 6 x 1
    // ================================================================

    auto contact_indices = getContactLegIndices();
    int n_contact = static_cast<int>(contact_indices.size());

    A.resize(6, n_contact);
    b.resize(6);

    if (n_contact == 0) {
        A.setZero();
        b.setZero();
        return;
    }

    double m = params_.total_mass;
    Eigen::Vector3d gravity(0.0, 0.0, -params_.gravity);
    Eigen::Matrix3d R = body_state_.orientation.toRotationMatrix();
    Eigen::Matrix3d I_world = R * params_.inertia * R.transpose();
    Eigen::Vector3d omega = body_state_.angular_velocity;

    // ---- Build b vector (desired wrench) ----
    // Linear part: m * (a_desired - g)
    b.head<3>() = m * (desired_linear_accel - gravity);

    // Angular part: I * alpha_desired + omega x (I * omega)
    b.tail<3>() = I_world * desired_angular_accel + omega.cross(I_world * omega);

    // ---- Build A matrix (force-to-wrench mapping) ----
    // For each contact leg i with vertical force f_z,i:
    //   Force contribution: [0, 0, f_z,i]^T
    //   Torque contribution: r_i x [0, 0, f_z,i]^T = [r_iy * f_z,i, -r_ix * f_z,i, 0]
    //
    // Column j of A (for contact leg j):
    //   A(0,j) = 0        (fx contribution from f_z)
    //   A(1,j) = 0        (fy contribution from f_z)
    //   A(2,j) = 1        (fz contribution from f_z)
    //   A(3,j) = r_iy     (torque_x from r x [0,0,fz])
    //   A(4,j) = -r_ix    (torque_y from r x [0,0,fz])
    //   A(5,j) = 0        (torque_z from r x [0,0,fz])

    Eigen::Vector3d p_com = body_state_.position;

    for (int j = 0; j < n_contact; ++j) {
        int leg_idx = contact_indices[j];
        Eigen::Vector3d r_i = contact_points_[leg_idx] - p_com;

        A(0, j) =  0.0;
        A(1, j) =  0.0;
        A(2, j) =  1.0;
        A(3, j) =  r_i.y();   // r_iy
        A(4, j) = -r_i.x();   // -r_ix
        A(5, j) =  0.0;
    }
}

}  // namespace dynamics
}  // namespace robot_control
