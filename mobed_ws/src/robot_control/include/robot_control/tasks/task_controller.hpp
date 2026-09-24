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
    // ----------------------------------------------------------------
    // Step-over geometry
    //
    // Per the paper (Section III-F): "rotate the eccentric joints until
    // they reach the top of the curb" — NOT a fixed 180° flip.
    //
    // For l_ecc = 75 mm and a target step height of 5~8 cm:
    //   Δθ_lift = asin((h_step + h_margin) / l_ecc)
    //           ≈ asin((0.065 + 0.02) / 0.075) ≈ asin(1.13) → clamped to ~0.65 rad
    //
    // We use 0.65 rad (~37°) as a conservative universal value covering the
    // 5 cm, 7.5 cm, and 10 cm curbs in the test playground.
    // ----------------------------------------------------------------

    // Angular increment to lift the wheel above the curb edge (rad)
    double step_over_angle = 0.65;

    // Extra overshoot angle at the trajectory peak to guarantee clearance (rad)
    double lift_peak_angle_offset = 0.15;  // total peak = step_over_angle + offset = 0.80 rad

    // Target eccentric angle for rear legs after they are placed on top of curb.
    // Rear legs operate at negative angles (outward-down config); after climbing
    // they must return to this negative angle so wheels push DOWN onto the curb.
    // Derived from posture IK at curb height ≈ 0.06 m → q_rear ≈ -0.80 rad.
    double rear_settle_angle = -0.80;   // rad (rear leg stable-support angle on curb)

    // Duration for the front leg step-over trajectory (seconds)
    double front_lift_duration = 2.0;
    // Duration for the rear leg step-over trajectory (seconds)
    double rear_lift_duration = 2.0;

    // Forward drive duration after front legs are placed (seconds)
    double forward_drive_duration = 3.0;
    // Forward drive speed during body advance (m/s)
    double forward_drive_speed = 0.1;

    // Settling time after rear legs are placed (seconds)
    double settling_duration = 1.0;

    // Blend-out duration when returning from FSM to IDLE (seconds).
    // During this window the override_mask weight fades from 1→0 linearly,
    // preventing the chassis from suddenly slamming down to flat-ground height.
    double blend_out_duration = 0.5;   // seconds

    // Debounce time to confirm contact detection (seconds)
    double contact_debounce_time = 0.12;

    // Refractory cooldown period after returning to IDLE (seconds).
    // During cooldown, all impact triggers are inhibited so touchdown vibrations
    // cannot re-trigger the climbing state machine.
    double cooldown_duration = 1.0;

    // Closed-loop touchdown detection threshold (Nm).
    // During descent of step-over trajectory, resistive torque > threshold confirms
    // solid contact with curb top surface, enabling early transition to placed/drive.
    double touchdown_torque_threshold = 2.0;
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
     * @brief Update the FSM state machine with joint efforts for closed-loop touchdown detection.
     *
     * @param contact_detector    Reference to the contact detector (for impact flags)
     * @param current_ecc_angles  Current eccentric joint angles [FL, FR, RL, RR] (rad)
     * @param current_ecc_efforts Current eccentric joint efforts/torques [FL, FR, RL, RR] (Nm)
     * @param dt                  Time step (seconds)
     */
    void update(const estimation::ContactDetector& contact_detector,
                const Eigen::Vector4d& current_ecc_angles,
                const Eigen::Vector4d& current_ecc_efforts,
                double dt);

    void update(const estimation::ContactDetector& contact_detector,
                const Eigen::Vector4d& current_ecc_angles,
                double dt) {
        update(contact_detector, current_ecc_angles, Eigen::Vector4d::Zero(), dt);
    }

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
    // Selective lift mask for decoupled single-wheel / dual-wheel climbing (Phase 3)
    std::array<bool, NUM_LEGS> active_lift_mask_ = {false, false, false, false};
    double forward_vel_override_ = 0.0;

    // Trajectory generators for front and rear leg pairs
    MultiSegmentTrajectory front_left_traj_;
    MultiSegmentTrajectory front_right_traj_;
    MultiSegmentTrajectory rear_left_traj_;
    MultiSegmentTrajectory rear_right_traj_;

    // Timing
    double state_timer_ = 0.0;        // time spent in current state
    double contact_debounce_ = 0.0;   // debounce timer for contact detection
    double blend_out_timer_ = 0.0;    // timer counting the FSM→IDLE blend-out phase
    double cooldown_timer_ = 0.0;     // refractory timer inhibiting re-triggers in IDLE

    // Blend-out weight: 1.0 = full FSM override; 0.0 = fully handed back to balance controller.
    // Exposed via getBlendWeight() so mobed_control_node can lerp between override and IK angles.
    double blend_out_weight_ = 0.0;

    // Stored start angles for trajectory planning and placed angles
    Eigen::Vector4d lift_start_angles_;
    Eigen::Vector2d front_target_angles_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d rear_target_angles_ = Eigen::Vector2d::Zero();

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
