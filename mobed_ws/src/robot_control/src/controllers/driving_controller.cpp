#include "robot_control/controllers/driving_controller.hpp"
#include <algorithm>
#include <cmath>

namespace robot_control {
namespace controllers {

DrivingController::DrivingController(const DrivingControllerParams& params, 
                                     std::shared_ptr<kinematics::MobedKinematics> kinematics)
    : params_(params), kinematics_(kinematics) {
    prev_steer_angles_.setZero();
    prev_wheel_speeds_.setZero();
}

bool DrivingController::isSteerSafe(double target_steer, double current_ecc) const {
    // If the eccentric arm is pointing inwards (e.g. current_ecc is negative and large magnitude,
    // or specifically pointing inwards towards the chassis body), steering might hit the chassis.
    // In this simplified logic: if ecc is tucked inwards, limit steer to outward angles.
    // Assuming current_ecc < -threshold means tucked inwards.
    if (current_ecc < -params_.ecc_collision_threshold) {
        // Normalize steer to [-pi, pi] for checking
        double norm_steer = std::atan2(std::sin(target_steer), std::cos(target_steer));
        
        // Define forbidden zone based on chassis geometry. 
        // Example: if tucked inwards, pointing towards chassis center is forbidden.
        // For simplicity, we just say |norm_steer| > pi/2 is forbidden if tucked.
        if (std::abs(norm_steer) > M_PI_2) {
            return false;
        }
    }
    return true;
}

std::tuple<Eigen::Vector4d, Eigen::Vector4d> DrivingController::update(
    const Eigen::Vector3d& cmd_vel,
    const Eigen::Vector4d& current_steer_angles,
    const Eigen::Vector4d& current_ecc_angles,
    double dt,
    bool e_stop_active) {

    // First time initialization to current hardware state
    if (!initialized_) {
        prev_steer_angles_ = current_steer_angles;
        prev_wheel_speeds_.setZero();
        initialized_ = true;
    }

    if (e_stop_active) {
        // E-STOP: Lock steering, stop wheels immediately
        prev_steer_angles_ = current_steer_angles;
        prev_wheel_speeds_.setZero();
        return {prev_steer_angles_, prev_wheel_speeds_};
    }

    // 1. Math Kinematics (Already contains Swerve Heading Optimization)
    auto [raw_steer_angles, raw_wheel_speeds] = kinematics_->computeDrivingIK(cmd_vel, current_steer_angles);

    Eigen::Vector4d final_steer = prev_steer_angles_;
    Eigen::Vector4d final_speed = prev_wheel_speeds_;

    for (int i = 0; i < 4; ++i) {
        // 2. Steering Constraint Function
        if (!isSteerSafe(raw_steer_angles(i), current_ecc_angles(i))) {
            // Collision zone! Kill speed, hold steering.
            raw_wheel_speeds(i) = 0.0;
            raw_steer_angles(i) = prev_steer_angles_(i);
        }

        // 3. Slew Rate Limiting (Speed/Accel)
        double speed_delta = raw_wheel_speeds(i) - prev_wheel_speeds_(i);
        double max_speed_delta = params_.max_wheel_accel * dt;
        speed_delta = std::clamp(speed_delta, -max_speed_delta, max_speed_delta);
        final_speed(i) = prev_wheel_speeds_(i) + speed_delta;

        // Rate Limiting (Steering)
        double steer_delta = raw_steer_angles(i) - prev_steer_angles_(i);
        double max_steer_delta = params_.max_steer_vel * dt;
        steer_delta = std::clamp(steer_delta, -max_steer_delta, max_steer_delta);
        final_steer(i) = prev_steer_angles_(i) + steer_delta;
    }

    // 4. Update state and return
    prev_steer_angles_ = final_steer;
    prev_wheel_speeds_ = final_speed;

    return {final_steer, final_speed};
}

} // namespace controllers
} // namespace robot_control
