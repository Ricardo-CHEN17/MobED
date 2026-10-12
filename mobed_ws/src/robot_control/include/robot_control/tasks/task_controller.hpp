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
    // ----------------------------------------------------------------
    // Adaptive Step-Over Geometry (Paper Section III-F & Kinematic Bounds)
    //
    // For l_ecc = 75 mm, the absolute mechanical lift ceiling is 2*l_ecc = 0.15 m.
    // The target curb height is parameterized in [0.04m, 0.15m].
    // Dynamic lift angle is computed from forward kinematics:
    //   cos(q_lift) = cos(q_start) - (h_curb + h_margin) / l_ecc
    // ----------------------------------------------------------------
    double target_curb_height = 0.075;   // m (supported range [0.04m, 0.22m])
    double curb_margin = 0.015;          // m (clearance safety margin)
    double wheelbase = 0.432;            // m (chassis wheelbase for travel progression)

    // Mode Profile
    ClimbMode climb_mode = ClimbMode::LOW_80MM;
    double front_roll_speed = 0.0;       // rad/s (traction roll assist on front wheels during lift)
    double rear_roll_speed = 0.0;        // rad/s (traction roll assist on rear wheels during lift)
    bool active_pitch_counter_balance = true; // active rear leg extension to eliminate pitch tilt

    // Angular increment to lift the wheel above the curb edge (rad, fallback default)
    double step_over_angle = 0.65;

    // Extra overshoot angle at the trajectory peak to guarantee clearance (rad)
    double lift_peak_angle_offset = 0.15;  // total peak = step_over_angle + offset

    // Target eccentric angle for rear legs after they are placed on top of curb (rad)
    double rear_settle_angle = -0.80;

    // Duration for the front leg step-over trajectory (seconds)
    double front_lift_duration = 2.0;
    // Duration for the rear leg step-over trajectory (seconds)
    double rear_lift_duration = 2.0;

    // Forward drive duration after front legs are placed (seconds)
    double forward_drive_duration = 3.5;
    // Forward drive speed during body advance (m/s)
    double forward_drive_speed = 0.15;

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
    // Minimum inclination limit (~6.9 deg, 0.12 rad) to preserve a ~1.1cm impact lever arm
    // and prevent mathematical/physical impact singularity when legs extend toward vertical.
    static constexpr double MIN_SINGULARITY_INCLINATION = 0.12;

    explicit TaskController(const TaskControllerParams& params = TaskControllerParams());

    /**
     * @brief Update the FSM state machine with joint efforts and odometry velocity for closed-loop progression.
     *
     * @param contact_detector    Reference to the contact detector (for impact flags)
     * @param current_ecc_angles  Current eccentric joint angles [FL, FR, RL, RR] (rad)
     * @param current_ecc_efforts Current eccentric joint efforts/torques [FL, FR, RL, RR] (Nm)
     * @param dt                  Time step (seconds)
     * @param body_vx             Chassis forward velocity from odometry (m/s)
     */
    void update(const estimation::ContactDetector& contact_detector,
                const Eigen::Vector4d& current_ecc_angles,
                const Eigen::Vector4d& current_ecc_efforts,
                double dt,
                double body_vx = 0.0);

    void update(const estimation::ContactDetector& contact_detector,
                const Eigen::Vector4d& current_ecc_angles,
                double dt) {
        update(contact_detector, current_ecc_angles, Eigen::Vector4d::Zero(), dt, 0.0);
    }

    void setCurbHeight(double height) {
        params_.target_curb_height = std::clamp(height, 0.04, 0.22);
    }
    double getCurbHeight() const { return params_.target_curb_height; }

    void setModeProfile(ClimbMode mode) {
        params_.climb_mode = mode;
        if (mode == ClimbMode::HIGH_200MM) {
            params_.target_curb_height = 0.20;
            params_.front_roll_speed = 2.5;
            params_.rear_roll_speed = 2.5;
            params_.forward_drive_speed = 0.20;
            params_.forward_drive_duration = 4.0;
        } else {
            params_.target_curb_height = 0.08;
            params_.front_roll_speed = 0.0;
            params_.rear_roll_speed = 0.0;
            params_.forward_drive_speed = 0.20;
            params_.forward_drive_duration = 4.0;
        }
    }
    ClimbMode getModeProfile() const { return params_.climb_mode; }

    /**
     * @brief Dynamically compute lift angle based on target curb height and starting posture.
     * @param q_start Starting eccentric angle of the leg (rad)
     * @return Required angular increment (rad)
     */
    double computeAdaptiveLiftAngle(double q_start) const;

    /**
     * @brief Compute rear leg counter-balancing angle to keep chassis level while front legs are on the curb.
     * @param q_front_avg Average eccentric angle of the front legs resting on the curb
     * @param h_curb Height of the curb (m)
     * @return Target eccentric angle for rear legs (rad)
     */
    double computeRearLevelingAngle(double q_front_avg, double h_curb) const;

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
    double travel_distance_ = 0.0;    // integrated travel distance during body advance in FRONT_PLACED

    // Blend-out weight: 1.0 = full FSM override; 0.0 = fully handed back to balance controller.
    // Exposed via getBlendWeight() so mobed_control_node can lerp between override and IK angles.
    double blend_out_weight_ = 0.0;
    bool has_climbed_ = false;

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
