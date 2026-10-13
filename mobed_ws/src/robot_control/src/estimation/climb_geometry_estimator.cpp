#include "robot_control/estimation/climb_geometry_estimator.hpp"
#include <cmath>

namespace robot_control {
namespace estimation {

ClimbGeometryEstimator::ClimbGeometryEstimator(const RobotParams& params)
    : params_(params)
{
    reset();
}

void ClimbGeometryEstimator::reset() {
    geometry_ = ClimbGeometry();
    filtered_delta_height_ = 0.0;
    front_crest_timer_ = 0.0;
    rear_crest_timer_ = 0.0;
}

void ClimbGeometryEstimator::update(
    const BodyState& body_state,
    const std::array<Eigen::Vector3d, NUM_LEGS>& contact_points_world,
    const Eigen::Vector4d& current_ecc_angles,
    const Eigen::Vector4d& current_wheel_vels,
    const Eigen::Vector4d& current_wheel_efforts,
    const Eigen::Vector4d& target_wheel_vels,
    const Eigen::Vector3d& omega_body,
    double target_step_height,
    double dt)
{
    if (dt <= 0.0 || dt > 0.5) return;

    // 1. Contact heights in gravity-aligned world frame
    double f_h = 0.5 * (contact_points_world[FL].z() + contact_points_world[FR].z());
    double r_h = 0.5 * (contact_points_world[RL].z() + contact_points_world[RR].z());
    geometry_.front_height = f_h;
    geometry_.rear_height = r_h;

    double raw_delta_h = f_h - r_h;
    // Low-pass filter for smooth, robust height difference
    double alpha = std::clamp(dt / (dt + 0.05), 0.0, 1.0);
    filtered_delta_height_ += alpha * (raw_delta_h - filtered_delta_height_);
    geometry_.delta_height = filtered_delta_height_;

    // 2. Chassis Euler angles & Standardized Nose-Up Pitch
    geometry_.body_pitch = body_state.pitch();
    geometry_.body_roll = body_state.roll();

    Eigen::Matrix3d R = body_state.orientation.toRotationMatrix();
    // R(2, 0) is world Z component of body +X (forward nose) vector
    geometry_.nose_up_pitch = std::asin(std::clamp(R(2, 0), -1.0, 1.0));
    // When nose is pitching up, omega_body.y() < 0 (right-hand rule around body +Y left axis)
    geometry_.nose_up_pitch_rate = -omega_body.y();

    // 3. Rear hip height in world frame (posture joint is l_ecc*cos(q_r) above wheel axle)
    double q_r_avg = 0.5 * (current_ecc_angles(RL) + current_ecc_angles(RR));
    geometry_.rear_hip_z = r_h + params_.r_wheel + params_.l_ecc * std::cos(q_r_avg);
    geometry_.rear_hip_cleared = (geometry_.rear_hip_z >= target_step_height + 0.015);

    // 4. CoM longitudinal support margin (distance from CoM to rear wheel contact boundary)
    double rear_contact_x = 0.5 * (contact_points_world[RL].x() + contact_points_world[RR].x());
    geometry_.com_support_margin = body_state.position.x() - rear_contact_x;

    // 5. Front wall climbing detection:
    // When front wheels are climbing vertical face, nose pitches up sharply with positive wheel height delta
    geometry_.front_wall_climbing = (geometry_.delta_height > 0.035 &&
                                     geometry_.nose_up_pitch > 0.070 &&
                                     geometry_.nose_up_pitch_rate > 0.15);

    // 6. Edge cresting detection
    bool fl_spinning = std::abs(current_wheel_vels(FL)) > 0.3 * std::max(0.5, std::abs(target_wheel_vels(FL)));
    bool fr_spinning = std::abs(current_wheel_vels(FR)) > 0.3 * std::max(0.5, std::abs(target_wheel_vels(FR)));
    bool fl_effort_dropped = std::abs(current_wheel_efforts(FL)) < 4.0;
    bool fr_effort_dropped = std::abs(current_wheel_efforts(FR)) < 4.0;

    double crest_h_thresh = std::max(0.06, target_step_height - 0.04);
    if (geometry_.delta_height > crest_h_thresh && (fl_spinning || fr_spinning) && (fl_effort_dropped || fr_effort_dropped)) {
        front_crest_timer_ += dt;
        if (front_crest_timer_ >= 0.05) {
            geometry_.front_edge_crested = true;
        }
    } else {
        front_crest_timer_ = std::max(0.0, front_crest_timer_ - dt);
        if (geometry_.delta_height < 0.05) {
            geometry_.front_edge_crested = false;
        }
    }

    // Rear edge cresting: rear wheels have climbed onto the plateau
    bool rl_spinning = std::abs(current_wheel_vels(RL)) > 0.3 * std::max(0.5, std::abs(target_wheel_vels(RL)));
    bool rr_spinning = std::abs(current_wheel_vels(RR)) > 0.3 * std::max(0.5, std::abs(target_wheel_vels(RR)));

    if (geometry_.front_height > crest_h_thresh && geometry_.rear_height > crest_h_thresh &&
        std::abs(geometry_.delta_height) < 0.05 && (rl_spinning || rr_spinning)) {
        rear_crest_timer_ += dt;
        if (rear_crest_timer_ >= 0.05) {
            geometry_.rear_edge_crested = true;
        }
    } else {
        rear_crest_timer_ = std::max(0.0, rear_crest_timer_ - dt);
        geometry_.rear_edge_crested = false;
    }
}

}  // namespace estimation
}  // namespace robot_control
