#pragma once

#include <Eigen/Dense>

namespace robot_control {

// ============================================================================
// Robot Physical Parameters — all constants derived from CAD and the paper
// ============================================================================
struct RobotParams {
    // -----------------------------------------------
    // Mass & Inertia (for future SRBD / QP in Plan 2)
    // -----------------------------------------------
    double total_mass = 70.0;  // kg, total robot mass including payload
    double body_mass = 14.0;   // kg, base body mass without payload/wheels
    double payload_mass = 56.0; // kg, payload mass
    // Body inertia tensor about CoM, in body frame (kg·m²)
    // Approximate values — must be refined from CAD or system identification
    Eigen::Matrix3d inertia = (Eigen::Matrix3d() <<
        0.5,  0.0,  0.0,
        0.0,  0.8,  0.0,
        0.0,  0.0,  0.9).finished();

    // -----------------------------------------------
    // Chassis Geometry
    // -----------------------------------------------
    double length_x = 0.15;   // m, half-length from center to steer axis (X)
    double width_y  = 0.15;   // m, half-width from center to steer axis (Y)

    // -----------------------------------------------
    // Eccentric Mechanism (Posture Control)
    // -----------------------------------------------
    double l_ecc = 0.075;     // m, eccentric arm length (from posture joint to wheel axle)

    // -----------------------------------------------
    // Wheel
    // -----------------------------------------------
    double r_wheel = 0.10;    // m, wheel radius

    // -----------------------------------------------
    // CAD Mechanical Offsets
    // -----------------------------------------------
    double posture_z_offset = 0.045;  // m, posture joint is 4.5cm below the steer joint

    // -----------------------------------------------
    // Height Limits
    // -----------------------------------------------
    double max_height = 0.22;  // m, maximum chassis height (legs fully vertical)
    double min_height = 0.08;  // m, minimum chassis height

    // -----------------------------------------------
    // Gravity
    // -----------------------------------------------
    double gravity = 9.81;  // m/s^2

    // -----------------------------------------------
    // EKF Tuning Parameters
    // -----------------------------------------------
    // Process noise covariance diagonal elements
    double ekf_process_noise_pos    = 0.01;   // position uncertainty growth rate
    double ekf_process_noise_vel    = 0.1;    // velocity uncertainty growth rate
    double ekf_process_noise_orient = 0.001;  // orientation uncertainty growth rate

    // Measurement noise covariance diagonal elements
    double ekf_meas_noise_accel     = 0.5;    // accelerometer noise (m/s^2)
    double ekf_meas_noise_gyro      = 0.01;   // gyroscope noise (rad/s)
    double ekf_meas_noise_wheel_vel = 0.05;   // wheel odometry noise (m/s)

    // -----------------------------------------------
    // Contact Detection Parameters
    // -----------------------------------------------
    double contact_torque_threshold  = 5.0;   // Nm, effort spike threshold for impact detection
    double contact_torque_rate_threshold = 50.0;  // Nm/s, dτ/dt threshold for impact detection
};

}  // namespace robot_control
