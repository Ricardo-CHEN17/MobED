#pragma once

#include <Eigen/Dense>
#include <tuple>
#include <memory>
#include "robot_control/kinematics/mobed_kinematics.hpp"

namespace robot_control {
namespace controllers {

struct DrivingControllerParams {
    double max_wheel_accel = 10.0; // rad/s^2
    double max_steer_vel = 5.0;    // rad/s
    double ecc_collision_threshold = 0.5; // radians (approx 28 degrees) - within this angle, wheel tucked inward
};

class DrivingController {
public:
    DrivingController(const DrivingControllerParams& params, 
                      std::shared_ptr<kinematics::MobedKinematics> kinematics);

    /**
     * @brief Update the driving controller step
     * 
     * @param cmd_vel Target body velocity [vx, vy, omega_z]
     * @param current_steer_angles Feedback from hardware [FL, FR, RL, RR]
     * @param current_ecc_angles Feedback from hardware for collision check [FL, FR, RL, RR]
     * @param dt Time step since last call
     * @param e_stop_active Emergency stop flag
     * @return std::tuple<Eigen::Vector4d, Eigen::Vector4d> (target_steer_angles, target_wheel_speeds)
     */
    std::tuple<Eigen::Vector4d, Eigen::Vector4d> update(
        const Eigen::Vector3d& cmd_vel,
        const Eigen::Vector4d& current_steer_angles,
        const Eigen::Vector4d& current_ecc_angles,
        double dt,
        bool e_stop_active = false);

private:
    DrivingControllerParams params_;
    std::shared_ptr<kinematics::MobedKinematics> kinematics_;

    Eigen::Vector4d prev_steer_angles_;
    Eigen::Vector4d prev_wheel_speeds_;
    
    bool initialized_ = false;

    // Helper for collision checking
    bool isSteerSafe(double target_steer, double current_ecc) const;
};

} // namespace controllers
} // namespace robot_control
