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

    // ---- Swerve Steering Optimization & Coherent Drive Gating ----
    // Enables coherent whole-body velocity scaling to preserve instantaneous ICR during steering transitions
    bool enable_coherent_drive_scaling = true;
    // Maximum steer alignment deadband for full drive power (rad, ~4.5 deg)
    double steer_alignment_deadband = 0.08;
    // Switching hysteresis to prevent 100 Hz chattering near 90 deg inversion boundaries (rad, ~10 deg)
    double swerve_invert_hysteresis = 0.175;

    // ---- Bank Angle Calculator (Eq 8 in paper) ----
    // Maximum allowable bank angle (roll induced by centripetal force)
    // 0.20 rad (~11.5 deg) preserves >= 2cm vertical stroke margin at 0.18m nominal height
    double max_bank_angle = 0.20;  // rad (~11.5 deg)
    // Height of center of gravity above ground (used for bank angle calc)
    double cog_height = 0.18;     // m, approximate CoG height
    // Low-pass filter coefficient for smoothing bank angle transitions
    double bank_angle_filter = 0.05;  // 0=frozen, 1=instant

    // ---- Low-Level Actuator Torque Loops (Paper Fig. 8 & Dual-Layer Mode) ----
    double kp_steer = 300.0;        // Nm/rad (steering PD proportional gain)
    double kd_steer = 30.0;         // Nm·s/rad (steering PD derivative gain)
    double kp_wheel = 15.0;         // Nm/(rad/s) (wheel speed PI proportional gain)
    double ki_wheel = 0.5;          // Nm/rad (wheel speed PI integral gain)
    double max_steer_torque = 35.0; // Nm (steering actuator saturation limit)
    double max_wheel_torque = 25.0; // Nm (driving wheel traction saturation limit)

    // ---- Velocity & Speed Saturation Limits ----
    double max_linear_vel  = 1.2;   // m/s (maximum chassis planar translation speed)
    double max_angular_vel = 2.0;   // rad/s (maximum chassis yaw rotation rate)
    double max_wheel_speed = 15.0;  // rad/s (~1.5 m/s maximum wheel rotational speed)
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
     * @param current_steer_vels Feedback steering joint velocities [FL, FR, RL, RR] (rad/s)
     * @param current_wheel_vels Feedback wheel rotational velocities [FL, FR, RL, RR] (rad/s)
     * @return std::tuple<Eigen::Vector4d, Eigen::Vector4d> (target_steer_angles, target_wheel_speeds)
     */
    std::tuple<Eigen::Vector4d, Eigen::Vector4d> update(
        const Eigen::Vector3d& cmd_vel,
        const Eigen::Vector4d& current_steer_angles,
        const Eigen::Vector4d& current_ecc_angles,
        double dt,
        bool e_stop_active = false,
        bool is_homing = false,
        const Eigen::Vector4d& current_steer_vels = Eigen::Vector4d::Zero(),
        const Eigen::Vector4d& current_wheel_vels = Eigen::Vector4d::Zero());

    /**
     * @brief Get synthesized low-level steering torques (Paper Fig. 8)
     */
    Eigen::Vector4d getSteerTorques() const { return steer_torques_; }

    /**
     * @brief Get synthesized low-level driving wheel torques with traction limits (Paper Fig. 8)
     */
    Eigen::Vector4d getWheelTorques() const { return wheel_torques_; }

    /**
     * @brief Get the bank angle (roll) induced by centripetal force during turning.
     *
     * Computes the optimal chassis roll angle to counteract centrifugal force
     * during curved motion, similar to a motorcycle leaning into a turn.
     * (Paper Section III.D & Eq 8)
     *
     * Sign convention (ISO 8855: X forward, Y left, Z up):
     * - Left turn (wz * vx > 0): returns negative roll (leans left / inward)
     * - Right turn (wz * vx < 0): returns positive roll (leans right / inward)
     *
     * @return double  Desired bank roll angle (rad)
     */
    double getBankAngle() const { return filtered_bank_angle_; }

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

    void reset() {
        initialized_ = false;
        filtered_bank_angle_ = 0.0;
        zero_cmd_duration_ = 0.0;
        prev_swerve_inverted_.fill(false);
        steer_torques_.setZero();
        wheel_torques_.setZero();
        wheel_speed_integrals_.setZero();
    }

private:
    DrivingControllerParams params_;
    std::shared_ptr<kinematics::MobedKinematics> kinematics_;

    Eigen::Vector4d prev_steer_angles_;
    Eigen::Vector4d prev_wheel_speeds_;

    // Actuator torque loops
    Eigen::Vector4d steer_torques_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d wheel_torques_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d wheel_speed_integrals_ = Eigen::Vector4d::Zero();

    bool initialized_ = false;
    double filtered_bank_angle_ = 0.0;  // smoothed bank angle output
    double zero_cmd_duration_ = 0.0;    // duration in seconds for which cmd_vel has been ~0
    std::array<bool, 4> prev_swerve_inverted_ = {false, false, false, false};

    /**
     * @brief Two-Stage Swerve Optimization with 180-deg Wheel Inversion Escape
     * Evaluates candidate state A (theta, v) and inverted state B (theta +- pi, -v).
     * Chooses shortest angular distance while actively evading forbidden inward sectors.
     * (Section III.D in paper & RoboMaster Swerve Architecture)
     * 
     * @param leg_index Leg index (FL, FR, RL, RR)
     * @param des_steer Desired steering angle from kinematics (rad)
     * @param des_speed Desired wheel velocity from kinematics (rad/s)
     * @param curr_steer Current actual steering angle (rad)
     * @param curr_ecc Current eccentric posture angle (rad)
     * @return std::pair<double, double> Optimized (steer_angle, wheel_speed)
     */
    std::pair<double, double> optimizeSwerveModule(
        int leg_index, double des_steer, double des_speed, double curr_steer, double curr_ecc,
        bool force_align = false);

    /**
     * @brief Dynamic collision-free transit interpolator.
     * Guarantees intermediate angles during slew rate limiting NEVER enter the forbidden sector.
     */
    double enforceForbiddenSectorTransit(
        int leg_index, double prev_s, double opt_s, double max_delta) const;
};

}  // namespace controllers
}  // namespace robot_control
