#pragma once

#include <Eigen/Dense>
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"

namespace robot_control {
namespace estimation {

/**
 * @brief Terrain plane estimator using wheel contact points.
 *
 * Implements the terrain estimation algorithm from the MobED paper (Eq 4-6).
 * Given the 3D positions of the 4 wheel-ground contact points, fits a plane
 * using SVD-based least squares and computes the terrain rotation matrix R_T.
 *
 * This matrix is used by the Balance Controller to compensate for slopes and
 * uneven terrain, ensuring the chassis remains level relative to the ground.
 */
class TerrainEstimator {
public:
    explicit TerrainEstimator(const RobotParams& params = RobotParams());

    /**
     * @brief Update the terrain estimate using 4 wheel contact points.
     *
     * Fits a plane to the 4 contact points via SVD, extracts the surface
     * normal, and constructs the terrain rotation matrix R_T.
     *
     * @param contact_points  Array of 4 contact positions in world frame
     * @param contact_valid   Array of 4 booleans indicating which legs are in contact
     * @param is_stopped      Whether robot is stationary (applies anti-latch flat decay)
     */
    void update(const std::array<Eigen::Vector3d, NUM_LEGS>& contact_points,
                const std::array<bool, NUM_LEGS>& contact_valid,
                bool is_stopped = false);

    /**
     * @brief Get the current terrain state.
     * @return TerrainState with rotation matrix, surface normal, and slope angle.
     */
    TerrainState getState() const { return terrain_state_; }

    /**
     * @brief Reset terrain estimator to flat ground assumption.
     */
    void reset();

private:
    RobotParams params_;
    TerrainState terrain_state_;

    double filter_alpha_ = 0.08;        // LPF smoothing coefficient for normal vector
    double max_slope_angle_ = 0.2618;   // rad (~15 deg), maximum allowed terrain slope angle

    /**
     * @brief Fit a plane to a set of 3D points using SVD.
     *
     * Computes the best-fit plane normal by centering the points and
     * performing SVD on the centered data matrix. The normal is the
     * left singular vector corresponding to the smallest singular value.
     *
     * @param points  Vector of 3D points (at least 3 required)
     * @return Plane normal vector (unit length, pointing upward)
     */
    Eigen::Vector3d fitPlaneNormal(const std::vector<Eigen::Vector3d>& points) const;

    /**
     * @brief Construct the terrain rotation matrix R_T from a surface normal.
     *
     * Builds a rotation matrix that transforms from the world frame to the
     * terrain-aligned frame. The Z-axis of R_T aligns with the terrain normal.
     *
     * @param normal  Terrain surface normal (unit vector)
     * @return 3x3 rotation matrix R_T
     */
    Eigen::Matrix3d normalToRotation(const Eigen::Vector3d& normal) const;
};

}  // namespace estimation
}  // namespace robot_control
