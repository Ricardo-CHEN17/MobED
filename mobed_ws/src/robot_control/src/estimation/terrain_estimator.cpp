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
    const std::array<bool, NUM_LEGS>& contact_valid)
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
    Eigen::Vector3d normal = fitPlaneNormal(valid_points);

    // Ensure normal points upward (positive Z component)
    if (normal.z() < 0.0) {
        normal = -normal;
    }

    // Compute terrain rotation matrix
    terrain_state_.normal   = normal;
    terrain_state_.rotation = normalToRotation(normal);

    // Compute overall slope angle: angle between terrain normal and world Z
    Eigen::Vector3d world_up(0.0, 0.0, 1.0);
    double cos_angle = std::clamp(normal.dot(world_up), -1.0, 1.0);
    terrain_state_.slope_angle = std::acos(cos_angle);

    terrain_state_.is_valid = true;
}

Eigen::Vector3d TerrainEstimator::fitPlaneNormal(
    const std::vector<Eigen::Vector3d>& points) const
{
    // ================================================================
    // SVD-based plane fitting (Eq 4-6 in the paper)
    //
    // 1. Compute centroid of contact points
    // 2. Center the points by subtracting the centroid
    // 3. Form the data matrix A (N x 3)
    // 4. Compute SVD: A = U * S * V^T
    // 5. The plane normal is the column of V corresponding to the
    //    smallest singular value (last column of V)
    // ================================================================

    size_t n = points.size();

    // Step 1: Compute centroid
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        centroid += p;
    }
    centroid /= static_cast<double>(n);

    // Step 2-3: Build centered data matrix
    Eigen::MatrixXd A(n, 3);
    for (size_t i = 0; i < n; ++i) {
        A.row(i) = (points[i] - centroid).transpose();
    }

    // Step 4: SVD decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);

    // Step 5: Normal = last column of V (smallest singular value direction)
    Eigen::Vector3d normal = svd.matrixV().col(2);

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
