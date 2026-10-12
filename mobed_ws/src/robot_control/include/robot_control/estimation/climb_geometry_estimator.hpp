#pragma once

#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"

namespace robot_control {
namespace estimation {

/**
 * @brief Real-time geometric and physical state during step climbing.
 */
struct ClimbGeometry {
    double front_height = 0.0;       // Average front wheel contact height in world frame (m)
    double rear_height = 0.0;        // Average rear wheel contact height in world frame (m)
    double delta_height = 0.0;       // front_height - rear_height (m)
    double body_pitch = 0.0;         // Chassis pitch angle (rad, >0 is nose-up)
    double body_roll = 0.0;          // Chassis roll angle (rad)
    double com_support_margin = 0.0; // Distance from CoM projection to rear support boundary (m)
    bool front_edge_crested = false; // Front wheels crested curb edge and transitioned to rolling
    bool rear_edge_crested = false;  // Rear wheels crested curb edge and transitioned to rolling
};

/**
 * @brief Pure geometric and kinematic state estimator for 200mm and adaptive curb climbing.
 */
class ClimbGeometryEstimator {
public:
    explicit ClimbGeometryEstimator(const RobotParams& params = RobotParams());

    /**
     * @brief Update the geometric estimation each control cycle.
     */
    void update(const BodyState& body_state,
                const std::array<Eigen::Vector3d, NUM_LEGS>& contact_points_world,
                const Eigen::Vector4d& current_wheel_vels,
                const Eigen::Vector4d& current_wheel_efforts,
                const Eigen::Vector4d& target_wheel_vels,
                double dt);

    const ClimbGeometry& getGeometry() const { return geometry_; }
    void reset();

private:
    RobotParams params_;
    ClimbGeometry geometry_;

    double filtered_delta_height_ = 0.0;
    double front_crest_timer_ = 0.0;
    double rear_crest_timer_ = 0.0;
};

}  // namespace estimation
}  // namespace robot_control
