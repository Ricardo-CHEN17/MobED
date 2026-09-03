#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <tuple>
#include <iostream>

namespace robot_control {
namespace kinematics {

struct MobedParameters {
    // Distance from base center to steer axes
    double length_x = 0.15;  // half-length
    double width_y = 0.15;   // half-width
    
    // Eccentric mechanism parameters
    double l_ecc = 0.10;     // eccentric link length
    
    // Wheel parameters
    double r_wheel = 0.15;   // wheel radius
    
    // Limits
    double max_height = 0.25;
    double min_height = 0.05;
};

class MobedKinematics {
public:
    MobedKinematics(const MobedParameters& params = MobedParameters());

    /**
     * @brief Compute Driving Inverse Kinematics (Swerve Drive)
     * 
     * @param v_body [v_x, v_y, omega_z] target velocity of the body
     * @return std::tuple<Eigen::Vector4d, Eigen::Vector4d> 
     *         Tuple of (steer_angles [FL, FR, RL, RR], wheel_speeds [FL, FR, RL, RR])
     */
    std::tuple<Eigen::Vector4d, Eigen::Vector4d> computeDrivingIK(
        const Eigen::Vector3d& v_body) const;

    /**
     * @brief Compute Posture Inverse Kinematics
     * 
     * @param target_height Target distance from ground to base origin
     * @param target_roll Target roll angle (radians)
     * @param target_pitch Target pitch angle (radians)
     * @param outward_config If true, uses q_ecc > 0 (wheels outward). Else wheels inward.
     * @return Eigen::Vector4d Target eccentric angles [FL, FR, RL, RR]
     */
    Eigen::Vector4d computePostureIK(
        double target_height, 
        double target_roll, 
        double target_pitch,
        bool outward_config = true) const;

    /**
     * @brief Get the Steer Joint positions in Base frame
     * 
     * @return Eigen::Matrix<double, 4, 3> 4 rows of [x, y, 0] for FL, FR, RL, RR
     */
    Eigen::Matrix<double, 4, 3> getSteerPositions() const;

private:
    MobedParameters params_;
    
    // Enumeration for leg indices
    enum LegIndex { FL = 0, FR = 1, RL = 2, RR = 3 };
};

} // namespace kinematics
} // namespace robot_control
