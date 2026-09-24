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
    terrain_state_.pitch_slope = 0.0;
    terrain_state_.roll_slope  = 0.0;
    terrain_state_.is_valid = false;
    gravity_normal_prev_ = Eigen::Vector3d(0.0, 0.0, 1.0);
}

void TerrainEstimator::update(
    const std::array<Eigen::Vector3d, NUM_LEGS>& foot_pos_in_base,
    const Eigen::Matrix3d& R_base,
    const std::array<bool, NUM_LEGS>& contact_valid,
    bool is_stopped)
{
    // Collect valid contact points transformed into gravity-aligned frame:
    // p_{i,G} = R_base * ^B r_i (relative to chassis center, free of base_pos drift)
    std::vector<Eigen::Vector3d> valid_points;
    valid_points.reserve(NUM_LEGS);

    for (int i = 0; i < NUM_LEGS; ++i) {
        if (contact_valid[i]) {
            valid_points.push_back(R_base * foot_pos_in_base[i]);
        }
    }

    // Need at least 3 points to define a plane
    if (valid_points.size() < 3) {
        // Not enough contact points — keep the previous estimate
        return;
    }

    // Fit plane normal via least-squares pseudo-inverse in gravity-aligned frame
    Eigen::Vector3d raw_normal = fitPlaneNormal(valid_points);

    // Ensure normal points upward (positive Z component)
    if (raw_normal.z() < 0.0) {
        raw_normal = -raw_normal;
    }

    // ================================================================
    // 1. Diagonal Torsion Warping Test (Phase 2.7):
    // For a rectangular 4-wheel robot on any planar macro-slope (pitch or roll),
    // the height relationship satisfies: (z_FL - z_FR) - (z_RL - z_RR) == 0.
    // When a single wheel runs over a cobblestone or is suspended in the air,
    // the four points form a hyperbolic warp with torsion == delta_z (15~30mm).
    // If torsion exceeds max_torsion_threshold (10mm), it is guaranteed to be an
    // asymmetric micro-bump or single-leg lift.
    // ================================================================
    bool is_torsion_warped = false;
    if (contact_valid[FL] && contact_valid[FR] && contact_valid[RL] && contact_valid[RR]) {
        Eigen::Vector3d p_FL = R_base * foot_pos_in_base[FL];
        Eigen::Vector3d p_FR = R_base * foot_pos_in_base[FR];
        Eigen::Vector3d p_RL = R_base * foot_pos_in_base[RL];
        Eigen::Vector3d p_RR = R_base * foot_pos_in_base[RR];

        double torsion = std::abs((p_FL.z() - p_FR.z()) - (p_RL.z() - p_RR.z()));
        if (torsion > params_.max_torsion_threshold) {
            is_torsion_warped = true;
        }
    }

    // ================================================================
    // 2. Coplanarity Residual Test (Micro-Bump Decoupling):
    // In true macro-slopes (ramps, hills), all contact points lie
    // on a single geometric plane (coplanar residual RMS < 8mm).
    // On cobblestones, speed bumps, or warped terrain,
    // the points form a warped, non-coplanar set (RMS residual > 8mm).
    // For non-coplanar micro-bumps or warped shapes, force raw_normal to world up (flat ground),
    // allowing the active suspension admittance to absorb the bump locally
    // without corrupting the kinematic terrain rotation matrix R_T!
    // ================================================================
    Eigen::Vector3d mean_point = Eigen::Vector3d::Zero();
    for (const auto& pt : valid_points) {
        mean_point += pt;
    }
    mean_point /= static_cast<double>(valid_points.size());

    double sum_sq_residual = 0.0;
    for (const auto& pt : valid_points) {
        double dist = raw_normal.dot(pt - mean_point);
        sum_sq_residual += dist * dist;
    }
    double rms_residual = std::sqrt(sum_sq_residual / static_cast<double>(valid_points.size()));

    if (is_torsion_warped || rms_residual > params_.terrain_coplanar_residual_threshold) {
        // High warping / non-coplanar: this is micro-terrain (cobblestones/bumps/single-leg lift).
        // Snap to flat ground so kinematic IK does NOT rock the chassis or latch lifted leg!
        raw_normal = Eigen::Vector3d(0.0, 0.0, 1.0);
    }

    // Apply low-pass filter to prevent transient spikes from inducing latching
    Eigen::Vector3d normal_gravity = raw_normal;
    if (terrain_state_.is_valid) {
        normal_gravity = (filter_alpha_ * raw_normal + (1.0 - filter_alpha_) * gravity_normal_prev_).normalized();
    }
    gravity_normal_prev_ = normal_gravity;

    // Compute overall slope angle: angle between terrain normal and world Z
    Eigen::Vector3d world_up(0.0, 0.0, 1.0);
    double cos_angle = std::clamp(normal_gravity.dot(world_up), -1.0, 1.0);
    double slope_angle = std::acos(cos_angle);

    // Anti-latch: when the robot is stopped, if the estimated slope is nearly flat
    // (< 3.0 deg), snap to world up to prevent small sensor/backlash offsets from accumulating.
    const double FLAT_GROUND_THRESHOLD = 0.052; // ~3.0 deg
    if (is_stopped && slope_angle < FLAT_GROUND_THRESHOLD) {
        normal_gravity = world_up;
        slope_angle = 0.0;
    }

    // Clamp slope angle to physical limit
    if (slope_angle > max_slope_angle_) {
        Eigen::Vector3d horizontal = (normal_gravity - normal_gravity.dot(world_up) * world_up).normalized();
        normal_gravity = (std::cos(max_slope_angle_) * world_up + std::sin(max_slope_angle_) * horizontal).normalized();
        slope_angle = max_slope_angle_;
    }

    // Project terrain normal onto robot's horizontal heading axes (u_fwd, u_lat):
    // R_base.col(0) is the robot body X axis in gravity-aligned frame
    Eigen::Vector3d body_x = R_base.col(0);
    Eigen::Vector3d u_fwd(body_x.x(), body_x.y(), 0.0);
    if (u_fwd.norm() < 1e-4) {
        u_fwd = Eigen::Vector3d(1.0, 0.0, 0.0);
    } else {
        u_fwd.normalize();
    }
    Eigen::Vector3d u_lat(-u_fwd.y(), u_fwd.x(), 0.0);

    double n_fwd  = normal_gravity.dot(u_fwd);
    double n_lat  = normal_gravity.dot(u_lat);
    double n_vert = std::max(0.01, normal_gravity.z());

    double pitch_slope = -std::atan2(n_fwd, n_vert);
    double roll_slope  = std::atan2(n_lat, n_vert);

    // Apply deadbands to reject micro-angle noise:
    if (std::abs(roll_slope) < params_.roll_slope_deadband) {
        roll_slope = 0.0;
    }
    if (std::abs(pitch_slope) < params_.pitch_slope_deadband) {
        pitch_slope = 0.0;
    }

    // Reconstruct clean heading-aligned normal from deadbanded slopes
    double clean_fwd = -std::tan(pitch_slope);
    double clean_lat = std::tan(roll_slope);
    Eigen::Vector3d normal_robot(clean_fwd, clean_lat, 1.0);
    normal_robot.normalize();

    double clean_cos = std::clamp(normal_robot.z(), -1.0, 1.0);
    slope_angle = std::acos(clean_cos);

    // Compute heading-aligned terrain rotation matrix R_T
    terrain_state_.normal      = normal_robot;
    terrain_state_.rotation    = normalToRotation(normal_robot);
    terrain_state_.slope_angle = slope_angle;
    terrain_state_.pitch_slope = pitch_slope;
    terrain_state_.roll_slope  = roll_slope;
    terrain_state_.is_valid    = true;
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
