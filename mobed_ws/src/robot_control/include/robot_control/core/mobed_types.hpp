#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstdint>

namespace robot_control {

// ============================================================================
// Leg Index Convention (consistent across the entire codebase)
// ============================================================================
enum LegIndex : int {
    FL = 0,  // Front-Left
    FR = 1,  // Front-Right
    RL = 2,  // Rear-Left
    RR = 3,  // Rear-Right
    NUM_LEGS = 4
};

// ============================================================================
// Sensor Data — raw inputs from hardware / simulation
// ============================================================================
struct ImuData {
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();  // m/s^2, body frame
    Eigen::Vector3d angular_velocity    = Eigen::Vector3d::Zero();  // rad/s, body frame
    Eigen::Quaterniond orientation      = Eigen::Quaterniond::Identity();
    bool has_orientation = false;  // true if IMU provides fused orientation
};

struct JointFeedback {
    Eigen::Vector4d steer_positions  = Eigen::Vector4d::Zero();  // rad (math frame)
    Eigen::Vector4d ecc_positions    = Eigen::Vector4d::Zero();  // rad (math frame)
    Eigen::Vector4d wheel_velocities = Eigen::Vector4d::Zero();  // rad/s
    Eigen::Vector4d steer_efforts    = Eigen::Vector4d::Zero();  // Nm
    Eigen::Vector4d ecc_efforts      = Eigen::Vector4d::Zero();  // Nm
    Eigen::Vector4d wheel_efforts    = Eigen::Vector4d::Zero();  // Nm
};

// ============================================================================
// Robot State — estimated / computed state of the chassis
// ============================================================================
struct BodyState {
    // Position & Orientation
    Eigen::Vector3d position    = Eigen::Vector3d::Zero();  // m, world frame
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    // Velocity
    Eigen::Vector3d linear_velocity  = Eigen::Vector3d::Zero();  // m/s, world frame
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();  // rad/s, world frame

    // Convenience accessors for Euler angles (from orientation quaternion)
    double roll()  const;
    double pitch() const;
    double yaw()   const;
};

// Inline implementations of Euler angle extraction (standard Tait-Bryan ZYX convention)
inline double BodyState::roll() const {
    Eigen::Matrix3d R = orientation.toRotationMatrix();
    return std::atan2(R(2, 1), R(2, 2));
}

inline double BodyState::pitch() const {
    Eigen::Matrix3d R = orientation.toRotationMatrix();
    return std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
}

inline double BodyState::yaw() const {
    Eigen::Matrix3d R = orientation.toRotationMatrix();
    return std::atan2(R(1, 0), R(0, 0));
}

// ============================================================================
// Terrain State — estimated ground plane under the robot
// ============================================================================
struct TerrainState {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();  // R_T: terrain rotation matrix in robot heading frame
    Eigen::Vector3d normal   = Eigen::Vector3d(0.0, 0.0, 1.0);  // terrain surface normal in robot heading frame
    double slope_angle = 0.0;  // rad, overall inclination angle
    double pitch_slope = 0.0;  // rad, longitudinal slope relative to robot heading (+ uphill)
    double roll_slope  = 0.0;  // rad, lateral slope relative to robot heading
    bool is_valid = false;     // true after first successful estimation
};

// ============================================================================
// Contact State — per-leg contact information
// ============================================================================
struct ContactState {
    std::array<bool, NUM_LEGS> in_contact = {true, true, true, true};
    std::array<bool, NUM_LEGS> impact_detected = {false, false, false, false};

    // Wheel-ground contact point positions in world frame
    std::array<Eigen::Vector3d, NUM_LEGS> contact_points = {
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()
    };
};

// ============================================================================
// Control Output — unified command structure
// ============================================================================
struct ControlOutput {
    Eigen::Vector4d steer_angles  = Eigen::Vector4d::Zero();  // rad
    Eigen::Vector4d ecc_angles    = Eigen::Vector4d::Zero();  // rad (position mode)
    Eigen::Vector4d ecc_torques   = Eigen::Vector4d::Zero();  // Nm  (torque mode)
    Eigen::Vector4d wheel_speeds  = Eigen::Vector4d::Zero();  // rad/s

    enum class EccMode { POSITION, TORQUE, HYBRID } ecc_mode = EccMode::POSITION;
};

}  // namespace robot_control
