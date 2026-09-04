#pragma once

#include <Eigen/Dense>
#include <memory>
#include "robot_control/kinematics/mobed_kinematics.hpp"

namespace robot_control {
namespace controllers {

struct BalanceControllerParams {
    double min_ecc_angle = -1.745; // ~ -100 deg
    double max_ecc_angle = 1.745;  // ~ 100 deg
    
    double max_roll = 0.331;       // ~ 19 deg
    double max_pitch = 0.401;      // ~ 23 deg
    
    double filter_alpha = 0.1;     // LPF coefficient (0.0 to 1.0, 1.0 means no filtering)
    double max_ecc_vel = 2.0;      // rad/s
};

class BalanceController {
public:
    BalanceController(const BalanceControllerParams& params, 
                      std::shared_ptr<kinematics::MobedKinematics> kinematics);

    /**
     * @brief Update the balance controller step
     * 
     * @param target_height Target base height
     * @param target_roll Target roll angle
     * @param target_pitch Target pitch angle
     * @param current_steer_angles Required for exact posture projection
     * @param current_ecc_angles Required for initialization and E-Stop freezing
     * @param dt Time step
     * @param e_stop_active Emergency stop flag
     * @return Eigen::Vector4d Target eccentric angles
     */
    Eigen::Vector4d update(
        double target_height,
        double target_roll,
        double target_pitch,
        const Eigen::Vector4d& current_steer_angles,
        const Eigen::Vector4d& current_ecc_angles,
        double dt,
        bool e_stop_active = false);

private:
    BalanceControllerParams params_;
    std::shared_ptr<kinematics::MobedKinematics> kinematics_;

    Eigen::Vector4d prev_ecc_angles_;
    bool initialized_ = false;
};

} // namespace controllers
} // namespace robot_control
