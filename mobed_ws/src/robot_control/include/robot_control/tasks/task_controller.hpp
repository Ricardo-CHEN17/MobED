#pragma once

#include <Eigen/Dense>
#include <array>
#include <string>
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"
#include "robot_control/estimation/contact_detector.hpp"
#include "robot_control/tasks/trajectory_generator.hpp"

namespace robot_control {
namespace tasks {

/**
 * @brief Climbing FSM states.
 *
 * Implements the 6-step stair climbing procedure from the MobED paper:
 *   IDLE → FRONT_CONTACT → FRONT_LIFT → FRONT_PLACED →
 *   REAR_CONTACT → REAR_LIFT → REAR_PLACED → IDLE
 */
enum class ClimbState {
    IDLE,            // Normal driving, no obstacle detected
    FRONT_CONTACT,   // Front wheels hit obstacle, preparing to lift
    FRONT_LIFT,      // Front legs executing step-over trajectory
    FRONT_PLACED,    // Front wheels placed on top, driving body forward
    REAR_CONTACT,    // Rear wheels hit obstacle, preparing to lift
    REAR_LIFT,       // Rear legs executing step-over trajectory
    REAR_PLACED,     // Rear wheels placed on top, restoring normal posture
};

/**
 * @brief Parameters for the climbing task controller.
 */
struct TaskControllerParams {
    // Duration for the front leg step-over trajectory (seconds)
    double front_lift_duration = 2.0;
    // Duration for the rear leg step-over trajectory (seconds)
    double rear_lift_duration = 2.0;

    // Angular displacement for step-over (rad)
    // ~π radians = 180° rotation to flip the leg over the obstacle
    double step_over_angle = M_PI;

    // Forward drive duration after front legs are placed (seconds)
    // Allows the body to move forward so rear wheels reach the obstacle
    double forward_drive_duration = 3.0;
    // Forward drive speed during body advance (m/s)
    double forward_drive_speed = 0.1;

    // Settling time after rear legs are placed (seconds)
    double settling_duration = 1.0;

    // Intermediate lift height angle for the step-over waypoint (rad)
    // This is the angle at the peak of the lift arc (above the obstacle)
    double lift_peak_angle_offset = 0.3;  // extra overshoot above π

    // Debounce time to confirm contact detection (seconds)
    double contact_debounce_time = 0.2;
};

/**
 * @brief FSM-based task controller for autonomous stair climbing.
 *
 * Sits above the BalanceController and DrivingController in the control
 * hierarchy. When a climbing task is active, this controller overrides
 * the eccentric joint commands for the relevant legs while the balance
 * controller maintains stability on the remaining legs.
 *
 * Usage:
 *   1. Feed sensor data each cycle via update()
 *   2. Check isActive() to know if the FSM is controlling any legs
 *   3. Use getEccOverrides() to get per-leg command overrides
 *   4. Use getOverrideMask() to know which legs are being controlled
 */
class TaskController {
public:
    explicit TaskController(const TaskControllerParams& params = TaskControllerParams());

    /**
     * @brief Update the FSM state machine.
     *
     * Processes contact detector signals, advances the active state,
     * and generates eccentric joint trajectory commands.
     *
     * @param contact_detector  Reference to the contact detector (for impact flags)
     * @param current_ecc_angles  Current eccentric joint angles [FL, FR, RL, RR] (rad)
     * @param dt  Time step (seconds)
     */
    void update(const estimation::ContactDetector& contact_detector,
                const Eigen::Vector4d& current_ecc_angles,
                double dt);

    /**
     * @brief Check if the task controller is actively overriding any leg.
     */
    bool isActive() const { return state_ != ClimbState::IDLE; }

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
     * @brief Get the desired forward velocity command during climbing.
     *
     * During certain states (e.g., FRONT_PLACED), the FSM requests
     * a specific forward velocity to advance the body.
     *
     * @return double  Desired vx (m/s), 0.0 when not applicable
     */
    double getForwardVelocityOverride() const { return forward_vel_override_; }

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
    double forward_vel_override_ = 0.0;

    // Trajectory generators for front and rear leg pairs
    MultiSegmentTrajectory front_left_traj_;
    MultiSegmentTrajectory front_right_traj_;
    MultiSegmentTrajectory rear_left_traj_;
    MultiSegmentTrajectory rear_right_traj_;

    // Timing
    double state_timer_ = 0.0;        // time spent in current state
    double contact_debounce_ = 0.0;   // debounce timer for contact detection

    // Stored start angles for trajectory planning
    Eigen::Vector4d lift_start_angles_;

    /**
     * @brief Transition to a new state, resetting the state timer.
     */
    void transitionTo(ClimbState new_state);

    /**
     * @brief Plan step-over trajectories for the front legs.
     *
     * Creates a 3-waypoint multi-segment trajectory:
     *   current → peak (lift) → target (placed on stair)
     */
    void planFrontLiftTrajectories(const Eigen::Vector4d& current_ecc_angles);

    /**
     * @brief Plan step-over trajectories for the rear legs.
     */
    void planRearLiftTrajectories(const Eigen::Vector4d& current_ecc_angles);
};

}  // namespace tasks
}  // namespace robot_control
