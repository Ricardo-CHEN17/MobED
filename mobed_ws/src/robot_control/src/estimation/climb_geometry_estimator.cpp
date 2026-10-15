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
    latched_curb_height_ = 0.08;
    height_latched_ = false;
    filtered_delta_height_ = 0.0;
    front_crest_timer_ = 0.0;
    rear_crest_timer_ = 0.0;
    ground_baseline_z_ = 0.0;
    baseline_initialized_ = false;
    travel_distance_ = 0.0;
    rear_slip_ratio_ = 0.0;
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
    (void)current_wheel_efforts;

    // 1. Contact heights in gravity-aligned world frame
    double f_h = 0.5 * (contact_points_world[FL].z() + contact_points_world[FR].z());
    double r_h = 0.5 * (contact_points_world[RL].z() + contact_points_world[RR].z());

    if (!baseline_initialized_) {
        ground_baseline_z_ = r_h;
        baseline_initialized_ = true;
    }

    geometry_.front_height = f_h - ground_baseline_z_;
    geometry_.rear_height = r_h - ground_baseline_z_;

    double raw_delta_h = f_h - r_h;
    // Low-pass filter for smooth, robust height difference
    double alpha = std::clamp(dt / (dt + 0.05), 0.0, 1.0);
    filtered_delta_height_ += alpha * (raw_delta_h - filtered_delta_height_);
    geometry_.delta_height = filtered_delta_height_;

    // 2. Chassis Euler angles & Standardized Nose-Up Pitch
    geometry_.body_pitch = body_state.pitch();
    geometry_.body_roll = body_state.roll();

    // Standardized positive nose-up elevation angle (strictly > 0 when nose is tilted up)
    geometry_.nose_up_pitch = body_state.nose_up_pitch();
    // When nose is pitching up, omega_body.y() < 0 (right-hand rule around body +Y left axis)
    geometry_.nose_up_pitch_rate = -omega_body.y();

    // 3. Rear hip height in world frame (posture joint is l_ecc*cos(q_r) above wheel axle)
    double q_r_avg = 0.5 * (current_ecc_angles(RL) + current_ecc_angles(RR));
    geometry_.rear_hip_z = geometry_.rear_height + params_.r_wheel + params_.l_ecc * std::cos(q_r_avg);
    geometry_.rear_hip_cleared = (geometry_.rear_hip_z >= target_step_height + 0.015);

    // 4. CoM longitudinal support margin (distance from CoM to rear wheel contact boundary)
    double rear_contact_x = 0.5 * (contact_points_world[RL].x() + contact_points_world[RR].x());
    geometry_.com_support_margin = body_state.position.x() - rear_contact_x;

    // 5. Front wall climbing detection:
    // When front wheels are climbing vertical face, nose pitches up sharply with positive wheel height delta
    geometry_.front_wall_climbing = (geometry_.delta_height > 0.030 &&
                                     geometry_.nose_up_pitch > 0.050 &&
                                     geometry_.nose_up_pitch_rate > 0.10);

    // 6. Edge cresting detection (adaptive threshold for 50mm, 100mm, 200mm)
    bool fl_spinning = std::abs(current_wheel_vels(FL)) > 0.3 * std::max(0.5, std::abs(target_wheel_vels(FL)));
    bool fr_spinning = std::abs(current_wheel_vels(FR)) > 0.3 * std::max(0.5, std::abs(target_wheel_vels(FR)));

    double crest_h_thresh = std::clamp(target_step_height * 0.65, 0.050, 0.16);
    // Cresting requires delta_height > threshold AND nose pitch rate has stabilized (pitch_rate < 0.15 rad/s)
    bool pitch_stabilizing = (geometry_.nose_up_pitch_rate < 0.15);
    if (geometry_.delta_height > crest_h_thresh && (fl_spinning || fr_spinning) && pitch_stabilizing) {
        front_crest_timer_ += dt;
        if (front_crest_timer_ >= 0.06) {
            geometry_.front_edge_crested = true;
        }
    } else {
        front_crest_timer_ = std::max(0.0, front_crest_timer_ - dt);
        if (geometry_.delta_height < 0.035) {
            geometry_.front_edge_crested = false;
        }
    }

    // Rear edge cresting: rear wheels have climbed onto the plateau
    bool rl_spinning = std::abs(current_wheel_vels(RL)) > 0.3 * std::max(0.5, std::abs(target_wheel_vels(RL)));
    bool rr_spinning = std::abs(current_wheel_vels(RR)) > 0.3 * std::max(0.5, std::abs(target_wheel_vels(RR)));

    if (geometry_.front_height > crest_h_thresh && geometry_.rear_height > crest_h_thresh &&
        std::abs(geometry_.delta_height) < 0.05 && (rl_spinning || rr_spinning)) {
        rear_crest_timer_ += dt;
        if (rear_crest_timer_ >= 0.04) {
            geometry_.rear_edge_crested = true;
        }
    } else {
        rear_crest_timer_ = std::max(0.0, rear_crest_timer_ - dt);
        geometry_.rear_edge_crested = false;
    }

    // 7. Auto Curb Height Latch
    // Once front wheels have crested the curb edge and height difference is confirmed (> 0.050m),
    // latch the true physical step height. This adapts all subsequent postures to the real terrain
    // without requiring manual keyboard input.
    if (geometry_.front_edge_crested && !height_latched_ && geometry_.delta_height > 0.050) {
        latched_curb_height_ = std::clamp(geometry_.delta_height, 0.05, 0.22);
        height_latched_ = true;
    }
    geometry_.latched_curb_height = latched_curb_height_;
    geometry_.has_latched_height = height_latched_;

    // 8. Plateau Path Odometry & Rear Drive Wheel Slip Ratio
    double chassis_vx = std::max(0.0, body_state.linear_velocity.x());
    double front_wheel_lin_vel = 0.5 * (std::max(0.0, current_wheel_vels(FL)) +
                                       std::max(0.0, current_wheel_vels(FR))) * params_.r_wheel;
    double effective_travel_vx = std::max(chassis_vx, front_wheel_lin_vel);

    if (geometry_.front_edge_crested || height_latched_) {
        travel_distance_ += effective_travel_vx * dt;
    } else {
        travel_distance_ = 0.0;
    }
    geometry_.travel_distance = travel_distance_;

    double rear_wheel_lin_vel = std::max(std::abs(current_wheel_vels(RL)), std::abs(current_wheel_vels(RR))) * params_.r_wheel;
    if (rear_wheel_lin_vel > 0.05) {
        rear_slip_ratio_ = std::clamp((rear_wheel_lin_vel - chassis_vx) / rear_wheel_lin_vel, 0.0, 1.0);
    } else {
        rear_slip_ratio_ = 0.0;
    }
    geometry_.rear_slip_ratio = rear_slip_ratio_;

    // 9. Belly clearance margin (m) and Reachability Gate
    double effective_h = height_latched_ ? latched_curb_height_ : target_step_height;
    double rear_chassis_z = geometry_.rear_height + params_.posture_z_offset + params_.l_ecc * std::cos(q_r_avg) + params_.r_wheel;
    geometry_.belly_clearance_margin = rear_chassis_z - effective_h;

    // 10. Reachability Gate for Crawl Gait:
    // To safely step over the curb lip without hitting it, rear hip z must be high enough
    double min_hip_z = effective_h - params_.l_ecc + params_.r_wheel + 0.015;
    geometry_.gate_swing_permitted = (geometry_.rear_hip_z >= min_hip_z) && (geometry_.delta_height >= 0.035);
    geometry_.stability_margin_y = body_state.position.y();
}

}  // namespace estimation
}  // namespace robot_control
