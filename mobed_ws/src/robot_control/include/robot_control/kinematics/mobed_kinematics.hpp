#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <tuple>
#include <iostream>
#include "robot_control/core/mobed_types.hpp"

namespace robot_control {
namespace kinematics {

struct MobedParameters {
    // Distance from base center to steer axes
    double length_x = 0.15;  // half-length (approx 150mm based on CAD)
    double width_y = 0.15;   // half-width (approx 150mm based on CAD)
    
    // Eccentric mechanism parameters
    // Adjusted from 0.10m to 0.075m to match actual MuJoCo CAD offsets
    double l_ecc = 0.075;    
    
    // Wheel parameters
    double r_wheel = 0.10;   // wheel radius
    
    // CAD Mechanical Offsets
    double posture_z_offset = 0.045; // Posture joint is 4.5cm lower than Steer joint

    // Chassis collision box dimensions (CAD half-dimensions)
    double chassis_box_x = 0.18; // m
    double chassis_box_y = 0.13; // m

    // Limits
    double max_height = 0.22;
    double min_height = 0.08;
};

class MobedKinematics {
public:
    MobedKinematics(const MobedParameters& params = MobedParameters());

    /**
     * @brief Compute Driving Inverse Kinematics (Eq 7 in paper)
     * 
     * @param v_body [v_x, v_y, omega_z] target velocity of the body
     * @param current_steer_angles [FL, FR, RL, RR] Current steering angles for shortest-path optimization
     * @param current_ecc_angles [FL, FR, RL, RR] Current eccentric angles for exact contact point ^B r_i
     * @return std::tuple<Eigen::Vector4d, Eigen::Vector4d> 
     *         Tuple of (steer_angles [FL, FR, RL, RR], wheel_speeds [FL, FR, RL, RR])
     */
    std::tuple<Eigen::Vector4d, Eigen::Vector4d> computeDrivingIK(
        const Eigen::Vector3d& v_body,
        const Eigen::Vector4d& current_steer_angles,
        const Eigen::Vector4d& current_ecc_angles) const;

    /**
     * @brief Compute Posture Inverse Kinematics (Exact Non-linear Solution, Section III.E.3)
     * 
     * @param target_height Target distance from ground to base origin
     * @param target_roll Target roll angle (radians)
     * @param target_pitch Target pitch angle (radians)
     * @param current_steer_angles [FL, FR, RL, RR] Steering angles needed for exact geometric projection
     * @param R_T Terrain plane rotation matrix
     * @param config Posture configuration (OUTWARD, INWARD, SPIN per Section II.B & Fig 5)
     * @return Eigen::Vector4d Target eccentric angles [FL, FR, RL, RR]
     */
    Eigen::Vector4d computePostureIK(
        double target_height, 
        double target_roll, 
        double target_pitch,
        const Eigen::Vector4d& current_steer_angles,
        const Eigen::Matrix3d& R_T,
        PostureConfiguration config = PostureConfiguration::OUTWARD) const;

    // Backward-compatible overload for boolean outward_config
    Eigen::Vector4d computePostureIK(
        double target_height, 
        double target_roll, 
        double target_pitch,
        const Eigen::Vector4d& current_steer_angles,
        const Eigen::Matrix3d& R_T,
        bool outward_config) const {
        return computePostureIK(target_height, target_roll, target_pitch,
                                current_steer_angles, R_T,
                                outward_config ? PostureConfiguration::OUTWARD : PostureConfiguration::INWARD);
    }

    /**
     * @brief Compute the 3D position of wheel contact point in Base frame (Eq 2)
     * 
     * @param leg_index Leg index (FL, FR, RL, RR)
     * @param q_str Steering joint angle (rad)
     * @param q_ecc Eccentric posture joint angle (rad)
     * @return Eigen::Vector3d Contact point position in Base frame [x, y, z]^T
     */
    Eigen::Vector3d computeFootPositionInBase(
        int leg_index, double q_str, double q_ecc) const;

    /**
     * @brief Get the Steer Joint positions in Base frame
     * 
     * @return Eigen::Matrix<double, 4, 3> 4 rows of [x, y, 0] for FL, FR, RL, RR
     */
    Eigen::Matrix<double, 4, 3> getSteerPositions() const;

    /**
     * @brief Get the robot parameters
     */
    const MobedParameters& getParams() const { return params_; }

    /**
     * @brief Maximum safe steer angle from neutral (0.0).
     * Safe outward sector for all wheels is [-MAX_SAFE_STEER_ANGLE, +MAX_SAFE_STEER_ANGLE].
     * Fully spans all 360-deg planar traction vectors using 180-deg wheel speed inversion
     * without ever drilling into the under-belly collision zone (|steer| > 92 deg).
     */
    static constexpr double MAX_SAFE_STEER_ANGLE = 1.60569; // ~92.0 deg (1.60569 rad)

    /**
     * @brief Structure defining individual leg safe outward steer sector
     */
    struct SteerSectorConfig {
        double center_angle; // Safe outward center angle in rad
        double half_width;   // Half width in rad
    };

    /**
     * @brief Structure defining forbidden inward steer sector
     */
    struct SteerForbiddenSector {
        double center_angle; // Center angle in [-pi, pi]
        double half_width;   // Half width in radians
        bool active;         // True if sector is active
    };

    /**
     * @brief Get individual safe steering sector parameters for each leg
     * FL: center = +0.785 rad (+45 deg), half_width = 1.571 rad (90 deg) -> [-45 deg, +135 deg]
     * RL: center = -0.785 rad (-45 deg), half_width = 1.571 rad (90 deg) -> [-135 deg, +45 deg]
     * FR & RR: center = 0.0 rad, half_width = 1.605 rad (~92 deg) -> [-92 deg, +92 deg]
     */
    SteerSectorConfig getLegSafeSectorConfig(int leg_index) const;

    /**
     * @brief Get forbidden inward steering angle sector parameters
     * 
     * @param leg_index Leg index (FL, FR, RL, RR)
     * @return SteerForbiddenSector Sector info
     */
    SteerForbiddenSector getForbiddenSteerSectorInfo(int leg_index) const;

    /**
     * @brief Check if a steer angle is inside the forbidden under-belly sector for this leg
     * 
     * @param leg_index Leg index (FL, FR, RL, RR)
     * @param steer_angle Steering angle in radians
     * @param margin Safety buffer in radians (default 0.0)
     * @return true if inside forbidden sector
     */
    bool isSteerInForbiddenSector(int leg_index, double steer_angle, double margin = 0.0) const;

    /**
     * @brief Clamp/project an angle to this leg's safe outward sector
     * 
     * @param leg_index Leg index (FL, FR, RL, RR)
     * @param steer_angle Steering angle in radians
     * @param margin Safety buffer in radians (default 0.0)
     * @return Clamped safe steering angle in [-pi, pi]
     */
    double clampToSafeSteerSector(int leg_index, double steer_angle, double margin = 0.0) const;

    /**
     * @brief Get the optimal outward wide-track stance angle for a leg
     * 
     * @param leg_index Leg index (FL, FR, RL, RR)
     * @return Optimal angle in radians (FL/RR: +35 deg, FR/RL: -35 deg)
     */
    double getOptimalOutwardStanceAngle(int leg_index) const;

    /**
     * @brief Backward-compatible pair [min, max]
     */
    std::pair<double, double> getForbiddenSteerSector(int leg_index, double q_ecc = 0.0) const;

    /**
     * @brief Utility to normalize angle to [-pi, pi]
     */
    double normalize_angle(double angle) const;

private:
    MobedParameters params_;
    
    // Enumeration for leg indices
    enum LegIndex { FL = 0, FR = 1, RL = 2, RR = 3 };
};

} // namespace kinematics
} // namespace robot_control
