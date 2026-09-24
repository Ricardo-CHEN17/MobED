#pragma once

#include <Eigen/Dense>

namespace robot_control {

// ============================================================================
// Robot Physical Parameters — all constants derived from CAD and the paper
// ============================================================================
struct RobotParams {
    // -----------------------------------------------
    // Mass & Inertia (Aligned with Paper Section II.B & III.E)
    // -----------------------------------------------
    // Body platform weight derived from MuJoCo inertial entries:
    //   chassis: 14.997 kg, 4x steering: 4x1.537=6.148 kg,
    //   4x eccentric arm: 4x1.062=4.248 kg, 4x wheel: 4x0.752=3.008 kg
    //   Total: 14.997 + 6.148 + 4.248 + 3.008 = 28.401 kg
    double body_mass = 21.4;     // kg, chassis + steering + eccentric arms (no wheels)
    // Current payload mass carried by the robot (up to 56.0 kg per paper)
    double payload_mass = 0.0;   // kg, default to unladen
    // Total mass m = body_mass + wheel_mass (Eq 11)
    double total_mass = 28.4;    // kg, calibrated against MuJoCo physical model
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
    // Contact Detection Parameters (Multi-Modal Model)
    // -----------------------------------------------
    double contact_torque_threshold      = 14.0;   // Nm, sustained effort load for obstacle collision confirmation
    double contact_torque_rate_threshold = 120.0;  // Nm/s, shock onset rate threshold to arm observation window
    double shock_window_duration         = 0.20;   // s, transient shock observation window
    double sustain_confirm_duration          = 0.18;   // s, required sustained load duration to confirm impact (9 frames at 50Hz, filters 100ms small bump roll-over)
    double wheel_stall_torque_threshold      = 8.0;    // Nm, drive wheel resistance confirmation (flat ground is < 2.0 Nm)
    double wheel_stall_vel_threshold         = 0.4;    // rad/s (~0.04 m/s), maximum wheel rotation speed under true curb stall
    double ramp_inhibit_slope_threshold      = 0.078;  // rad (~4.5 deg), slopes steeper than this inhibit stair climbing
    double ramp_inhibit_pitch_threshold      = 0.045;  // rad (~2.5 deg), instantaneous IMU pitch angle threshold
    double ramp_inhibit_pitch_rate_threshold = 0.12;   // rad/s (~7 deg/s), instantaneous IMU pitch rate threshold
    double chassis_blocked_vel_threshold     = 0.06;   // m/s, forward chassis speed threshold indicating true rigid blockage

    // -----------------------------------------------
    // Terrain Filter & Micro-Bump Decoupling Parameters
    // -----------------------------------------------
    double terrain_coplanar_residual_threshold = 0.008; // m (8mm), maximum RMS residual to qualify as a planar macro-slope
    double max_torsion_threshold               = 0.010; // m (10mm), diagonal torsion warping |(z_FL-z_FR) - (z_RL-z_RR)| to reject single-leg bumps
    double roll_slope_deadband                 = 0.061; // rad (~3.5 deg), deadband on roll slope to eliminate micro-bump rocking
    double pitch_slope_deadband                = 0.045; // rad (~2.5 deg), deadband on pitch slope for flat-ground stability
};

}  // namespace robot_control
