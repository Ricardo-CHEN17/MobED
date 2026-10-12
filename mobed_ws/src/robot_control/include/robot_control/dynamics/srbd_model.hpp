#pragma once

#include <Eigen/Dense>
#include <array>
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"

namespace robot_control {
namespace dynamics {

/**
 * @brief Single Rigid Body Dynamics (SRBD) model for MobED.
 *
 * Implements the dynamics equations from the MobED paper (Eq 10-12).
 * Treats the entire robot as a single rigid body and formulates the
 * Newton-Euler equations as linear constraints on the ground reaction
 * forces (GRF) at each leg's contact point.
 *
 * Newton:  m * a_desired = sum(f_i) + m * g
 * Euler:   I * alpha_desired + omega x (I * omega) = sum(r_i x f_i)
 *
 * These are assembled into the matrix form:
 *   A * f = b
 * where f = [f_z1, f_z2, f_z3, f_z4]^T (vertical GRF per leg)
 *       A is the dynamics constraint matrix (6 x 4)
 *       b is the desired dynamics wrench (6 x 1)
 */
class SrbdModel {
public:
    explicit SrbdModel(const RobotParams& params = RobotParams());

    /**
     * @brief Set the current robot state for dynamics computation.
     *
     * @param body_state  Current body state (position, orientation, velocities)
     * @param contact_points  World-frame positions of the 4 wheel contact points
     * @param contact_valid   Which legs are currently in ground contact
     */
    void setState(const BodyState& body_state,
                  const std::array<Eigen::Vector3d, NUM_LEGS>& contact_points,
                  const std::array<bool, NUM_LEGS>& contact_valid);

    /**
     * @brief Build the dynamics constraint matrix A and target vector b.
     *
     * Given the desired body acceleration (from a PD controller), constructs
     * the linear system A * f_z = b where f_z are the vertical GRFs.
     *
     * @param desired_linear_accel   Desired body linear acceleration (world frame, m/s^2)
     * @param desired_angular_accel  Desired body angular acceleration (world frame, rad/s^2)
     * @param[out] A  Dynamics constraint matrix (6 x num_contact_legs)
     * @param[out] b  Desired dynamics wrench vector (6 x 1)
     */
    void buildDynamicsConstraints(
        const Eigen::Vector3d& desired_linear_accel,
        const Eigen::Vector3d& desired_angular_accel,
        Eigen::MatrixXd& A,
        Eigen::VectorXd& b) const;

    /**
     * @brief Get the number of legs currently in contact.
     */
    int getNumContactLegs() const;

    /**
     * @brief Get the indices of legs currently in contact.
     */
    std::vector<int> getContactLegIndices() const;

    /**
     * @brief Set the payload mass and geometry (IEEE RA-L 2026 Paper Section II.B & III.E.2).
     *
     * Dynamically updates total mass m and inertia tensor I using parallel axis theorem.
     *
     * @param payload_mass      Payload mass in kg (up to 56 kg)
     * @param payload_z_offset  Payload CoM height above chassis origin in meters (default 0.15m)
     */
    void setPayload(double payload_mass, double payload_z_offset = 0.15);

    double getTotalMass() const { return current_total_mass_; }
    double getPayloadMass() const { return payload_mass_; }
    double getComZShift() const { return com_z_shift_; }
    Eigen::Matrix3d getCompositeInertia() const { return current_inertia_; }

private:
    RobotParams params_;

    // Payload state (Section II.B & III.E.2)
    double payload_mass_ = 0.0;
    double payload_z_offset_ = 0.15;
    double current_total_mass_ = 28.4;
    double com_z_shift_ = 0.0;
    Eigen::Matrix3d current_inertia_ = Eigen::Matrix3d::Identity();

    // Current state
    BodyState body_state_;
    std::array<Eigen::Vector3d, NUM_LEGS> contact_points_;
    std::array<bool, NUM_LEGS> contact_valid_;

    /**
     * @brief Compute the skew-symmetric matrix [v]_x for cross product.
     * @param v  3D vector
     * @return 3x3 skew-symmetric matrix such that [v]_x * w = v x w
     */
    static Eigen::Matrix3d skew(const Eigen::Vector3d& v);
};

}  // namespace dynamics
}  // namespace robot_control
