#pragma once

#include <Eigen/Dense>
#include <array>
#include <string>
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"
#include "robot_control/estimation/contact_detector.hpp"
#include "robot_control/estimation/climb_geometry_estimator.hpp"
#include "robot_control/tasks/trajectory_generator.hpp"

namespace robot_control {
namespace tasks {

/**
 * @brief Climbing FSM states.
 *
 * Implements physics-closed-loop wall-traction climbing with pitch compliance:
 *   IDLE → FRONT_WALL_CLIMB → FRONT_PLACED →
 *   REAR_WALL_CLIMB (Stage R1) → REAR_CREST_FOLD (Stage R2) →
 *   REAR_PLACED → IDLE
 */
enum class ClimbState {
    IDLE,               // Normal driving, monitoring for front wheel impact / wall-roll
    FRONT_WALL_CLIMB,   // Front wheels climb wall, eccentric arms fold with pitch feedback
    STRADDLE_LEVELING,  // Front wheels on curb, adaptive near-level differential stance (40mm~200mm)
    FRONT_PLACED = STRADDLE_LEVELING, // Backward-compatible alias
    STRADDLE_HOLD,      // Operator pause hold: locking stance, wheels braked, closed-loop balance active
    REAR_WALL_CLIMB,    // Rear wheels climb wall with front wheel pulling assist
    REAR_CREST_FOLD,    // Rear hip cleared, fold rear legs onto curb smoothly
    REAR_PLACED,        // All wheels on top of curb, advance 0.25m into plateau then release to IDLE
};

/**
 * @brief Parameters for the climbing task controller.
 */
struct TaskControllerParams {
    double target_curb_height = 0.08;   // m (0.04m ~ 0.22m continuous adaptive range)
    double curb_margin = 0.015;          // m (clearance safety margin)
    double wheelbase = 0.300;            // m (chassis wheelbase: 2 * length_x)
    double wheel_radius = 0.100;         // m (MuJoCo calibrated wheel radius)

    // Mode Profile
    ClimbMode climb_mode = ClimbMode::ADAPTIVE;
    double front_roll_speed = 2.5;       // rad/s (traction roll assist on front wheels)
    double rear_roll_speed = 2.5;        // rad/s (traction roll assist on rear wheels)
    bool active_pitch_counter_balance = true;

    // Durations & speeds
    double forward_drive_speed = 0.20;   // m/s
    double settling_distance = 0.25;     // m (forward advance onto plateau before exiting FSM)
    double settling_duration = 1.50;     // s (max settling duration guard)
    double blend_out_duration = 0.40;    // s
    double cooldown_duration = 1.0;      // s

    // Active pitch leveling during straddle
    double leveling_kp_pitch = 2.0;
    double leveling_kd_pitch = 0.3;
    double max_straddle_time_200mm = 8.0; // s
};

class TaskController {
public:
    static constexpr double MIN_SINGULARITY_INCLINATION = 0.12;

    explicit TaskController(const TaskControllerParams& params = TaskControllerParams());

    void update(const estimation::ContactDetector& contact_detector,
                const estimation::ClimbGeometry& geometry,
                const Eigen::Vector4d& current_ecc_angles,
                const Eigen::Vector4d& current_ecc_efforts,
                double dt,
                double body_vx = 0.0,
                double cmd_vx = 0.20);

    void update(const estimation::ContactDetector& contact_detector,
                const Eigen::Vector4d& current_ecc_angles,
                const Eigen::Vector4d& current_ecc_efforts,
                double dt,
                double body_vx = 0.0) {
        estimation::ClimbGeometry dummy;
        update(contact_detector, dummy, current_ecc_angles, current_ecc_efforts, dt, body_vx, 0.20);
    }

    void update(const estimation::ContactDetector& contact_detector,
                const Eigen::Vector4d& current_ecc_angles,
                double dt) {
        estimation::ClimbGeometry dummy;
        update(contact_detector, dummy, current_ecc_angles, Eigen::Vector4d::Zero(), dt, 0.0);
    }

    void setCurbHeight(double height) {
        params_.target_curb_height = std::clamp(height, 0.04, 0.22);
    }
    double getCurbHeight() const { return params_.target_curb_height; }

    void setClimbMode(ClimbMode mode) {
        params_.climb_mode = mode;
    }
    ClimbMode getClimbMode() const { return params_.climb_mode; }

    /**
     * @brief Continuous analytical front target angle mapping (1.45 rad for 40mm up to 2.60 rad for 200mm).
     * @param h_curb Curb height (m)
     * @return Target eccentric angle for front legs (rad)
     */
    double computeFrontTargetAngle(double h_curb) const;

    /**
     * @brief Backward-compatible alias for computeFrontTargetAngle.
     */
    double computeAdaptiveLiftAngle(double q_start = 0.0) const;

    /**
     * @brief Compute rear leg counter-balancing angle to keep chassis near-level with >20mm belly clearance.
     * @param q_front_avg Average eccentric angle of the front legs resting on the curb
     * @param h_curb Height of the curb (m)
     * @return Target eccentric angle for rear legs (rad)
     */
    double computeRearLevelingAngle(double q_front_avg, double h_curb) const;

    /**
     * @brief Compute rear leg climbing sweep target angle to ensure complete cresting over obstacle lip.
     * @param h_curb Curb height (m)
     * @return Target eccentric angle for rear legs during REAR_WALL_CLIMB (rad, in [-2.75, -2.95])
     */
    double computeRearClimbTargetAngle(double h_curb) const;

    /**
     * @brief Reset lockout flag allowing immediate next climb without hard reboot.
     */
    void clearClimbedLock() { has_climbed_ = false; }

    /**
     * @brief Check if the task controller is actively overriding any leg.
     */
    bool isActive() const { return state_ != ClimbState::IDLE; }

    /**
     * @brief Check if the robot has successfully climbed onto the obstacle.
     */
    bool hasClimbed() const { return has_climbed_; }

    /**
     * @brief Get the current FSM state.
     */
    ClimbState getState() const { return state_; }

    /**
     * @brief Get human-readable name of the current state.
     */
    std::string getStateName() const;

    /**
     * @brief Get the eccentric angle overrides for each leg.
     *
     * Only valid for legs where getOverrideMask()[i] is true.
     * Other legs should continue using BalanceController output.
     *
     * @return Eigen::Vector4d  Override angles [FL, FR, RL, RR] (rad)
     */
    Eigen::Vector4d getEccOverrides() const { return ecc_overrides_; }

    /**
     * @brief Get which legs are being overridden by the task controller.
     * @return Array of 4 booleans. true = this leg is controlled by FSM.
     */
    std::array<bool, NUM_LEGS> getOverrideMask() const { return override_mask_; }

    /**
     * @brief Get drive wheel speed overrides during wall climbing.
     */
    Eigen::Vector4d getWheelSpeedOverrides() const { return wheel_speed_overrides_; }

    /**
     * @brief Get mask of wheels currently overridden.
     */
    std::array<bool, NUM_LEGS> getWheelOverrideMask() const { return wheel_override_mask_; }

    /**
     * @brief Get the desired forward velocity command during climbing.
     *
     * During certain states (e.g., FRONT_PLACED), the FSM requests
     * a specific forward velocity to advance the body.
     *
     * @return double  Desired vx (m/s), 0.0 when not applicable
     */
    double getForwardVelocityOverride() const { return forward_vel_override_; }

    /**
     * @brief Get the blend-out weight for smooth FSM→IDLE transitions.
     *
     * Returns 1.0 when fully in FSM control, fading to 0.0 over blend_out_duration.
     * mobed_control_node uses this to lerp FSM override angles toward IK angles,
     * preventing the chassis from slamming down when climbing completes.
     */
    double getBlendWeight() const { return blend_out_weight_; }

    /**
     * @brief Get contact validity flags for each leg based on FSM state.
     * Airborne legs during FRONT_LIFT or REAR_LIFT report false.
     * @return Array of 4 booleans [FL, FR, RL, RR]
     */
    std::array<bool, NUM_LEGS> getContactValid() const;

    /**
     * @brief Force abort the climbing task and return to IDLE.
     */
    void abort();

    /**
     * @brief Reset the task controller to IDLE state.
     */
    void reset();

private:
    TaskControllerParams params_;
    ClimbState state_ = ClimbState::IDLE;

    // Per-leg override commands and mask
    Eigen::Vector4d ecc_overrides_;
    std::array<bool, NUM_LEGS> override_mask_;
    Eigen::Vector4d wheel_speed_overrides_ = Eigen::Vector4d::Zero();
    std::array<bool, NUM_LEGS> wheel_override_mask_ = {false, false, false, false};
    // Selective lift mask for decoupled single-wheel / dual-wheel climbing (Phase 3)
    std::array<bool, NUM_LEGS> active_lift_mask_ = {false, false, false, false};
    double forward_vel_override_ = 0.0;

    double current_front_target_ = 0.0;

    // Timing
    double state_timer_ = 0.0;        // time spent in current state
    double blend_out_timer_ = 0.0;    // timer counting the FSM→IDLE blend-out phase
    double cooldown_timer_ = 0.0;     // refractory timer inhibiting re-triggers in IDLE
    double travel_distance_ = 0.0;    // integrated travel distance during body advance in FRONT_PLACED
    double stall_jam_timer_ = 0.0;     // watchdog timer accumulating true physical stall duration

    // Blend-out weight: 1.0 = full FSM override; 0.0 = fully handed back to balance controller.
    double blend_out_weight_ = 0.0;
    bool has_climbed_ = false;

    // Stored start angles and target angles
    Eigen::Vector4d lift_start_angles_ = Eigen::Vector4d::Zero();
    Eigen::Vector2d front_target_angles_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d rear_target_angles_ = Eigen::Vector2d::Zero();

    /**
     * @brief Transition to a new state, resetting the state timer.
     */
    void transitionTo(ClimbState new_state);
};

}  // namespace tasks
}  // namespace robot_control
