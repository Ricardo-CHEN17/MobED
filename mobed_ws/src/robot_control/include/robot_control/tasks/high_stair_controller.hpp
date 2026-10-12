#pragma once

#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"
#include "robot_control/estimation/contact_detector.hpp"
#include "robot_control/estimation/climb_geometry_estimator.hpp"
#include "robot_control/tasks/trajectory_generator.hpp"

namespace robot_control {
namespace tasks {

/**
 * @brief High Stair (200mm) Climbing Closed-Loop FSM States.
 *
 * Implements pure dynamics-driven closed-loop climbing with:
 * 1. Dead-man's switch gating (manual drive to progress, release to HOLD).
 * 2. Closed-loop pitch compliance and edge cresting detection.
 * 3. Reverse lock during straddle and rear haul.
 */
enum class HighStairState {
    IDLE,
    HOLD,          // Frozen in place when user releases forward drive
    F_PRELOAD,     // Rear legs tucked to +0.80 rad to shift CoM rearward, rear wheels pin
    F_WALL_ROLL,   // Front wheels roll up wall under closed-loop pitch control
    F_LAND,        // Front legs crouch to -2.70 rad, verified height diff and stabilized pitch
    STRADDLE,      // Synchronized wheel speed advance until rear wheels contact curb
    R_PRELOAD,     // Front wheels pull to firmly pin rear wheels against curb face
    R_WALL_ROLL,   // Rear wheels roll up curb under closed-loop pitch control with front haul
    R_LAND,        // Rear legs placed on curb top
    RESET_SETTLE   // Smooth Cosine S-curve restore to nominal standing
};

/**
 * @brief Parameters for the 200mm dynamic high stair climbing controller.
 */
struct HighStairParams {
    double wheel_radius = 0.10;            // m wheel radius

    // Speeds
    double approach_speed = 0.15;          // m/s approach speed
    double rear_pin_speed = 0.35;          // m/s rear wheel pin/push speed holding robot against curb
    double front_roll_speed = 3.0;         // rad/s front wheel upward roll speed on curb face
    double advance_speed = 0.25;           // m/s synchronized forward speed during straddle
    double pull_speed = 0.40;              // m/s front wheel pull speed during rear flip
    double rear_roll_speed = 2.5;          // rad/s rear wheel upward roll speed
    double settle_speed = 0.15;            // m/s settle speed

    // Dynamic angles and pitch feedback
    double pitch_target = 0.12;            // rad (~7 deg nose-up pitch setpoint during climb)
    double k_pitch = 3.5;                  // pitch error gain to eccentric rate
    double base_flip_speed = 1.4;          // rad/s base eccentric flip rate
    double rear_tuck_angle = 0.80;         // rad rear eccentric tuck angle (shifting CoM rearward)
    double crouch_front_angle = -2.70;     // rad front eccentric downforce angle
    double impact_torque_threshold = 3.5;  // Nm wheel torque threshold confirming wall collision
    double stall_torque_threshold = 6.0;   // Nm wheel torque threshold under true obstacle stall
    double stall_vel_threshold = 0.5;      // rad/s maximum wheel rotation speed under true curb stall
    double stall_confirm_duration = 0.15;  // seconds sustained stall duration to confirm wall collision

    // Durations & timeouts
    double preload_timeout = 2.5;          // seconds fallback for preload
    double wall_roll_timeout = 3.0;        // seconds fallback for wall roll
    double settle_duration = 1.2;          // seconds to return eccentric joints back to nominal
    double blend_out_duration = 0.4;       // seconds to blend back to balance controller
};

/**
 * @brief Autonomous and user-gated 200mm high-stair climb controller.
 */
class HighStairController {
public:
    explicit HighStairController(const HighStairParams& params = HighStairParams());

    /**
     * @brief Update the FSM each control cycle with dynamic feedback.
     */
    void update(const estimation::ContactDetector& contact_detector,
                const estimation::ClimbGeometry& geometry,
                const Eigen::Vector4d& current_ecc_angles,
                const Eigen::Vector4d& current_ecc_efforts,
                const Eigen::Vector4d& current_wheel_efforts,
                const Eigen::Vector4d& current_wheel_vels,
                double user_cmd_vx,
                bool mode_enabled,
                double dt,
                double body_vx = 0.0,
                double body_pitch = 0.0);

    bool isActive() const { return state_ != HighStairState::IDLE; }
    HighStairState getState() const { return state_; }
    std::string getStateName() const;

    Eigen::Vector4d getEccOverrides() const { return ecc_overrides_; }
    std::array<bool, NUM_LEGS> getOverrideMask() const { return override_mask_; }

    Eigen::Vector4d getWheelSpeedOverrides() const { return wheel_speed_overrides_; }
    std::array<bool, NUM_LEGS> getWheelOverrideMask() const { return wheel_override_mask_; }

    double getForwardVelocityOverride() const { return forward_vel_override_; }
    double getBlendWeight() const { return blend_out_weight_; }

    /**
     * @brief Check whether reverse commands should be blocked to prevent rearward tipping.
     */
    bool isReverseLocked() const {
        return state_ == HighStairState::STRADDLE ||
               state_ == HighStairState::R_PRELOAD ||
               state_ == HighStairState::R_WALL_ROLL ||
               state_ == HighStairState::R_LAND ||
               state_ == HighStairState::RESET_SETTLE ||
               (state_ == HighStairState::HOLD &&
                (pre_hold_state_ == HighStairState::STRADDLE ||
                 pre_hold_state_ == HighStairState::R_PRELOAD ||
                 pre_hold_state_ == HighStairState::R_WALL_ROLL ||
                 pre_hold_state_ == HighStairState::R_LAND ||
                 pre_hold_state_ == HighStairState::RESET_SETTLE));
    }

    std::array<bool, NUM_LEGS> getContactValid() const;

    void trigger(const Eigen::Vector4d& current_ecc_angles);
    void setBaseStandingAngles(const Eigen::Vector4d& angles) { base_standing_angles_ = angles; }
    void abort();
    void reset();

private:
    HighStairParams params_;
    HighStairState state_ = HighStairState::IDLE;
    HighStairState pre_hold_state_ = HighStairState::IDLE;

    Eigen::Vector4d ecc_overrides_ = Eigen::Vector4d::Zero();
    std::array<bool, NUM_LEGS> override_mask_ = {false, false, false, false};

    Eigen::Vector4d wheel_speed_overrides_ = Eigen::Vector4d::Zero();
    std::array<bool, NUM_LEGS> wheel_override_mask_ = {false, false, false, false};

    double forward_vel_override_ = 0.0;

    double state_timer_ = 0.0;
    double blend_out_timer_ = 0.0;
    double blend_out_weight_ = 0.0;
    double advance_distance_ = 0.0;

    // Continuous dynamic joint angle integrators
    double front_flip_angle_ = 0.0;
    double rear_flip_angle_ = 0.0;

    // Retry counter
    int front_climb_retry_count_ = 0;

    // Sustained stall observation timer for wall collision confirmation
    double front_stall_timer_ = 0.0;

    Eigen::Vector4d base_standing_angles_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d hold_ecc_angles_ = Eigen::Vector4d::Zero();

    TrajectoryGenerator settle_traj_;

    void transitionTo(HighStairState new_state);
};

}  // namespace tasks
}  // namespace robot_control
