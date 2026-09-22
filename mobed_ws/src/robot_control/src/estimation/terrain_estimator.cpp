#include "robot_control/estimation/terrain_estimator.hpp"
#include <Eigen/SVD>
#include <cmath>
#include <algorithm>

namespace robot_control {
namespace estimation {

TerrainEstimator::TerrainEstimator(const RobotParams& params)
    : params_(params)
{
    reset();
}

void TerrainEstimator::reset() {
    terrain_state_.rotation = Eigen::Matrix3d::Identity();
    terrain_state_.normal   = Eigen::Vector3d(0.0, 0.0, 1.0);
    terrain_state_.slope_angle = 0.0;
    terrain_state_.is_valid = false;
}

void TerrainEstimator::update(
    const std::array<Eigen::Vector3d, NUM_LEGS>& contact_points,
    const std::array<bool, NUM_LEGS>& contact_valid,
    bool is_stopped)
{
    // Collect valid contact points
    std::vector<Eigen::Vector3d> valid_points;
    valid_points.reserve(NUM_LEGS);

    for (int i = 0; i < NUM_LEGS; ++i) {
        if (contact_valid[i]) {
            valid_points.push_back(contact_points[i]);
        }
    }

    // Need at least 3 points to define a plane
    if (valid_points.size() < 3) {
        // Not enough contact points — keep the previous estimate
        return;
    }

    // Fit plane normal via SVD
    Eigen::Vector3d raw_normal = fitPlaneNormal(valid_points);

    // Ensure normal points upward (positive Z component)
    if (raw_normal.z() < 0.0) {
        raw_normal = -raw_normal;
    }

    // Apply low-pass filter to prevent transient spikes from inducing latching
    Eigen::Vector3d normal = raw_normal;
    if (terrain_state_.is_valid) {
        normal = (filter_alpha_ * raw_normal + (1.0 - filter_alpha_) * terrain_state_.normal).normalized();
    }

    // Compute overall slope angle: angle between terrain normal and world Z
    Eigen::Vector3d world_up(0.0, 0.0, 1.0);
    double cos_angle = std::clamp(normal.dot(world_up), -1.0, 1.0);
    double slope_angle = std::acos(cos_angle);

    // Anti-latch: when the robot is stopped, if the estimated slope is nearly flat
    // (< 3.0 deg), snap to world up to prevent small sensor/backlash offsets from accumulating.
    // On actual slopes (>= 3.0 deg, e.g. ramps, steps), preserve the full terrain slope
    // so automatic leveling and balance control remain active while stationary.
    const double FLAT_GROUND_THRESHOLD = 0.052; // ~3.0 deg
    if (is_stopped && slope_angle < FLAT_GROUND_THRESHOLD) {
        normal = world_up;
        slope_angle = 0.0;
    }

    // Clamp slope angle to physical limit
    if (slope_angle > max_slope_angle_) {
        Eigen::Vector3d horizontal = (normal - normal.dot(world_up) * world_up).normalized();
        normal = (std::cos(max_slope_angle_) * world_up + std::sin(max_slope_angle_) * horizontal).normalized();
        slope_angle = max_slope_angle_;
    }

    // Compute terrain rotation matrix
    terrain_state_.normal   = normal;
    terrain_state_.rotation = normalToRotation(normal);
    terrain_state_.slope_angle = slope_angle;
    terrain_state_.is_valid = true;
}

Eigen::Vector3d TerrainEstimator::fitPlaneNormal(
    const std::vector<Eigen::Vector3d>& points) const
{
    // ================================================================
    // Plane fitting using pseudo-inverse (Eq 4-6 in the paper)
    // a = W^+ p^z
    // W = [1 p^x p^y]_{Nx3}
    // ================================================================

    size_t n = points.size();

    Eigen::MatrixXd W(n, 3);
    Eigen::VectorXd pz(n);

    for (size_t i = 0; i < n; ++i) {
        W(i, 0) = 1.0;
        W(i, 1) = points[i].x();
        W(i, 2) = points[i].y();
        pz(i) = points[i].z();
    }

    // Solve for a = [a0, a1, a2]^T using least squares (pseudo-inverse equivalent)
    Eigen::Vector3d a = W.colPivHouseholderQr().solve(pz);

    // The plane equation is z = a0 + a1*x + a2*y
    // Thus, -a1*x - a2*y + z - a0 = 0
    // Normal vector is proportional to (-a1, -a2, 1)
    Eigen::Vector3d normal(-a(1), -a(2), 1.0);

    return normal.normalized();
}

Eigen::Matrix3d TerrainEstimator::normalToRotation(
    const Eigen::Vector3d& normal) const
{
    // ================================================================
    // Construct rotation matrix R_T from terrain normal
    //
    // The terrain frame has:
    //   Z_T = terrain normal (given)
    //   X_T = projection of world X onto the terrain plane, normalized
    //   Y_T = Z_T × X_T (right-hand rule)
    //
    // This gives us a rotation from world frame to terrain-aligned frame.
    // ================================================================

    Eigen::Vector3d z_t = normal.normalized();

    // Choose world X as reference. If normal is nearly parallel to X,
    // fall back to world Y to avoid degenerate cross products.
    Eigen::Vector3d world_x(1.0, 0.0, 0.0);
    Eigen::Vector3d world_y(0.0, 1.0, 0.0);

    Eigen::Vector3d ref = (std::abs(z_t.dot(world_x)) < 0.9) ? world_x : world_y;

    // X_T: project ref onto the terrain plane and normalize
    Eigen::Vector3d x_t = (ref - ref.dot(z_t) * z_t).normalized();

    // Y_T: complete the right-handed frame
    Eigen::Vector3d y_t = z_t.cross(x_t).normalized();

    // Assemble rotation matrix: columns are the terrain frame axes in world coords
    Eigen::Matrix3d R_T;
    R_T.col(0) = x_t;
    R_T.col(1) = y_t;
    R_T.col(2) = z_t;

    return R_T;
}

}  // namespace estimation
}  // namespace robot_control
