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
    double body_pitch = 0.0;         // Chassis pitch angle (rad)
    double body_roll = 0.0;          // Chassis roll angle (rad)
    double nose_up_pitch = 0.0;      // rad, >0 is nose-up elevation angle (arcsin(R(2, 0)))
    double nose_up_pitch_rate = 0.0; // rad/s, >0 is pitching up (-omega_body.y())
    double rear_hip_z = 0.0;         // World Z height of rear posture joint (m)
    double com_support_margin = 0.0; // Distance from CoM projection to rear support boundary (m)
    bool front_wall_climbing = false;// Front wheels climbing vertical face
    bool front_edge_crested = false; // Front wheels crested curb edge and transitioned to rolling
    bool rear_hip_cleared = false;   // Rear hip higher than step edge (+ margin)
    bool rear_edge_crested = false;  // Rear wheels crested curb edge and transitioned to rolling
    double latched_curb_height = 0.08; // Auto-latched physical curb height (m)
    bool has_latched_height = false;   // True once front cresting locks the physical height
    double travel_distance = 0.0;        // Accumulated forward travel distance onto plateau (m)
    double rear_slip_ratio = 0.0;        // Rear drive wheel slip ratio [0.0, 1.0]
    double belly_clearance_margin = 0.0; // Estimated clearance between chassis bottom and curb edge (m)
    bool gate_swing_permitted = false;   // True if rear hip is sufficiently high for swing leg to clear curb
    double stability_margin_y = 0.0;     // Lateral stability margin (m)
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
                const Eigen::Vector4d& current_ecc_angles,
                const Eigen::Vector4d& current_wheel_vels,
                const Eigen::Vector4d& current_wheel_efforts,
                const Eigen::Vector4d& target_wheel_vels,
                const Eigen::Vector3d& omega_body,
                double target_step_height,
                double dt);

    void update(const BodyState& body_state,
                const std::array<Eigen::Vector3d, NUM_LEGS>& contact_points_world,
                const Eigen::Vector4d& current_wheel_vels,
                const Eigen::Vector4d& current_wheel_efforts,
                const Eigen::Vector4d& target_wheel_vels,
                double dt) {
        update(body_state, contact_points_world, Eigen::Vector4d::Zero(),
               current_wheel_vels, current_wheel_efforts, target_wheel_vels,
               Eigen::Vector3d::Zero(), 0.10, dt);
    }

    const ClimbGeometry& getGeometry() const { return geometry_; }
    bool hasLatchedHeight() const { return height_latched_; }
    double getLatchedHeight() const { return latched_curb_height_; }
    void unlatchHeight() { height_latched_ = false; }
    void reset();

private:
    RobotParams params_;
    ClimbGeometry geometry_;

    double latched_curb_height_ = 0.08;
    bool height_latched_ = false;

    double filtered_delta_height_ = 0.0;
    double front_crest_timer_ = 0.0;
    double rear_crest_timer_ = 0.0;
    double ground_baseline_z_ = 0.0;
    bool baseline_initialized_ = false;

    double travel_distance_ = 0.0;
    double rear_slip_ratio_ = 0.0;
};

}  // namespace estimation
}  // namespace robot_control
