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
    current_total_mass_ = params_.total_mass;
    current_inertia_ = params_.inertia;
    setPayload(params_.payload_mass, params_.payload_com_z_offset);
}

void SrbdModel::setPayload(double payload_mass, double payload_z_offset) {
    payload_mass_ = std::max(0.0, payload_mass);
    payload_z_offset_ = payload_z_offset;

    double m_robot = params_.total_mass;
    current_total_mass_ = m_robot + payload_mass_;

    // Center of mass shift along Z (Section II.B & III.E.2)
    com_z_shift_ = (payload_mass_ * payload_z_offset_) / current_total_mass_;

    // Parallel axis theorem shift:
    // Distance from robot base CoM (0,0,0) to composite CoM (0,0,com_z_shift_) is com_z_shift_
    // Distance from payload CoM (0,0,payload_z_offset_) to composite CoM is (payload_z_offset_ - com_z_shift_)
    double d_robot = com_z_shift_;
    double d_payload = payload_z_offset_ - com_z_shift_;

    // Additional moment of inertia about composite CoM (pitch & roll axes X and Y)
    double delta_I_xy = m_robot * (d_robot * d_robot) + payload_mass_ * (d_payload * d_payload);

    current_inertia_ = params_.inertia;
    current_inertia_(0, 0) += delta_I_xy;  // Roll inertia
    current_inertia_(1, 1) += delta_I_xy;  // Pitch inertia
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
    auto contact_indices = getContactLegIndices();
    int n_contact = static_cast<int>(contact_indices.size());

    A.resize(6, n_contact);
    b.resize(6);

    if (n_contact == 0) {
        A.setZero();
        b.setZero();
        return;
    }

    double m = current_total_mass_;
    Eigen::Vector3d gravity(0.0, 0.0, -params_.gravity);
    Eigen::Matrix3d R = body_state_.orientation.toRotationMatrix();
    Eigen::Matrix3d I_world = R * current_inertia_ * R.transpose();
    Eigen::Vector3d omega = body_state_.angular_velocity;

    // ---- Build b vector (desired wrench) ----
    // Linear part: m * (a_desired - g) (Paper Eq 11)
    b.head<3>() = m * (desired_linear_accel - gravity);

    // Active load transfer feedforward: planar accelerations (ax, ay) acting at CoM height
    // produce inertial pitching and rolling moments that vertical GRFs must counteract.
    // tau_load_transfer = r_com x (m * a_desired_planar)
    double z_com = body_state_.position.z() + (R * Eigen::Vector3d(0.0, 0.0, com_z_shift_)).z();
    Eigen::Vector3d a_planar(desired_linear_accel.x(), desired_linear_accel.y(), 0.0);
    Eigen::Vector3d tau_inertial = Eigen::Vector3d(0.0, 0.0, z_com).cross(m * a_planar);

    // Angular part: I * alpha_desired + omega x (I * omega) + tau_inertial
    b.tail<3>() = I_world * desired_angular_accel + omega.cross(I_world * omega) + tau_inertial;

    // Composite CoM in world frame
    Eigen::Vector3d p_com = body_state_.position + R * Eigen::Vector3d(0.0, 0.0, com_z_shift_);

    for (int j = 0; j < n_contact; ++j) {
        int leg_idx = contact_indices[j];
        Eigen::Vector3d r_i = contact_points_[leg_idx] - p_com;

        // A_i * ^B f_i = [ I ; [r_i]x ] * R * [0, 0, f_z,i]^T (Eq 11 in paper)
        Eigen::Vector3d f_dir = R.col(2);
        Eigen::Vector3d tau_dir = r_i.cross(f_dir);

        A.block<3, 1>(0, j) = f_dir;
        A.block<3, 1>(3, j) = tau_dir;
    }
}

}  // namespace dynamics
}  // namespace robot_control
