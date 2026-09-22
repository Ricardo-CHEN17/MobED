#pragma once

#include <Eigen/Dense>
#include <tuple>
#include <memory>
#include "robot_control/kinematics/mobed_kinematics.hpp"
#include "robot_control/core/robot_params.hpp"

namespace robot_control {
namespace controllers {

struct DrivingControllerParams {
    double max_wheel_accel = 10.0; // rad/s^2
    double max_steer_vel = 5.0;    // rad/s

    // ---- Steering Constraint (geometric anti-collision Section III.D) ----
    // Maximum allowable steering angle towards the inward (chassis) direction
    double max_inward_steer_angle = 1.15;   // rad (~66 deg)
    // Maximum allowable steering angle towards the outward direction
    double max_outward_steer_angle = 1.57;  // rad (90 deg)

    // ---- Bank Angle Calculator (Eq 8 in paper) ----
    // Maximum allowable bank angle (roll induced by centripetal force)
    double max_bank_angle = 0.26;  // rad (~15 deg)
    // Height of center of gravity above ground (used for bank angle calc)
    double cog_height = 0.18;     // m, approximate CoG height
    // Low-pass filter coefficient for smoothing bank angle transitions
    double bank_angle_filter = 0.05;  // 0=frozen, 1=instant
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
     * @param is_homing Whether the robot is in homing mode
     * @return std::tuple<Eigen::Vector4d, Eigen::Vector4d> (target_steer_angles, target_wheel_speeds)
     */
    std::tuple<Eigen::Vector4d, Eigen::Vector4d> update(
        const Eigen::Vector3d& cmd_vel,
        const Eigen::Vector4d& current_steer_angles,
        const Eigen::Vector4d& current_ecc_angles,
        double dt,
        bool e_stop_active = false,
        bool is_homing = false);

    /**
     * @brief Get the bank angle (roll) induced by centripetal force during turning.
     *
     * Computes the optimal chassis roll angle to counteract centripetal force
     * during curved motion, similar to a motorcycle leaning into a turn.
     * (Paper Eq 8)
     *
     * @return double  Desired bank roll angle (rad), positive = lean right
     */
    double getBankAngle() const { return filtered_bank_angle_; }

    void reset() { initialized_ = false; filtered_bank_angle_ = 0.0; zero_cmd_duration_ = 0.0; }

private:
    DrivingControllerParams params_;
    std::shared_ptr<kinematics::MobedKinematics> kinematics_;

    Eigen::Vector4d prev_steer_angles_;
    Eigen::Vector4d prev_wheel_speeds_;

    bool initialized_ = false;
    double filtered_bank_angle_ = 0.0;  // smoothed bank angle output
    double zero_cmd_duration_ = 0.0;    // duration in seconds for which cmd_vel has been ~0

    /**
     * @brief Steering Constraint Function (Section III.D in paper)
     * Dynamically limits the steer range based on current eccentric angle
     * and projects wheel speed onto the safe direction.
     * 
     * @param leg_index Leg index (FL, FR, RL, RR)
     * @param[in,out] steer Target steering angle to be constrained (rad)
     * @param[in,out] speed Target wheel speed to be projected (rad/s)
     * @param current_ecc Current eccentric posture angle (rad)
     */
    void applySteerConstraint(int leg_index, double& steer, double& speed, double current_ecc) const;

    /**
     * @brief Compute the raw bank angle from current motion state.
     *
     * Uses the centripetal force formula (Paper Eq 8):
     *   tan(bank_angle) = v^2 / (R * g) = v * omega / g
     *
     * where v = linear speed, omega = yaw rate, g = gravity.
     *
     * @param cmd_vel  Current velocity command [vx, vy, omega_z]
     * @return double  Raw bank angle (rad)
     */
    double computeBankAngle(const Eigen::Vector3d& cmd_vel) const;
};

}  // namespace controllers
}  // namespace robot_control
