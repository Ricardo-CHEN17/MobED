#include "robot_control/tasks/task_controller.hpp"
#include <cmath>

namespace robot_control {
namespace tasks {

TaskController::TaskController(const TaskControllerParams& params)
    : params_(params)
{
    reset();
}

void TaskController::reset() {
    state_ = ClimbState::IDLE;
    ecc_overrides_ = Eigen::Vector4d::Zero();
    override_mask_.fill(false);
    forward_vel_override_ = 0.0;
    state_timer_ = 0.0;
    contact_debounce_ = 0.0;
    lift_start_angles_ = Eigen::Vector4d::Zero();
    front_target_angles_ = Eigen::Vector2d::Zero();
    rear_target_angles_ = Eigen::Vector2d::Zero();
}

void TaskController::abort() {
    reset();
}

void TaskController::transitionTo(ClimbState new_state) {
    state_ = new_state;
    state_timer_ = 0.0;
}

std::string TaskController::getStateName() const {
    switch (state_) {
        case ClimbState::IDLE:           return "IDLE";
        case ClimbState::FRONT_CONTACT:  return "FRONT_CONTACT";
        case ClimbState::FRONT_LIFT:     return "FRONT_LIFT";
        case ClimbState::FRONT_PLACED:   return "FRONT_PLACED";
        case ClimbState::REAR_CONTACT:   return "REAR_CONTACT";
        case ClimbState::REAR_LIFT:      return "REAR_LIFT";
        case ClimbState::REAR_PLACED:    return "REAR_PLACED";
        default:                         return "UNKNOWN";
    }
}

void TaskController::planFrontLiftTrajectories(const Eigen::Vector4d& current_ecc_angles) {
    // ================================================================
    // Front leg step-over trajectory (3 waypoints):
    //
    // 1. LIFT: Rotate the eccentric arm upward by ~90° to clear the
    //    obstacle edge. This lifts the wheel above the stair.
    //
    // 2. EXTEND: Continue rotating to the full step-over angle (~180°).
    //    The wheel now passes over the obstacle and begins descending
    //    on the other side.
    //
    // 3. PLACE: Settle at the final angle where the wheel rests on
    //    top of the stair.
    //
    // The total angular displacement is params_.step_over_angle (default π).
    // We add a small overshoot (lift_peak_angle_offset) at the peak to
    // ensure the wheel clears the stair edge with margin.
    // ================================================================

    double half_step = params_.step_over_angle * 0.5;
    double peak_extra = params_.lift_peak_angle_offset;
    double seg_time = params_.front_lift_duration / 3.0;

    // Front-Left trajectory
    {
        double start = current_ecc_angles(FL);
        std::vector<TrajectoryWaypoint> waypoints = {
            {start + half_step + peak_extra, seg_time},         // lift to peak
            {start + params_.step_over_angle + peak_extra, seg_time},  // extend over
            {start + params_.step_over_angle, seg_time}         // settle down
        };
        front_left_traj_.plan(start, waypoints);
    }

    // Front-Right trajectory (same pattern)
    {
        double start = current_ecc_angles(FR);
        std::vector<TrajectoryWaypoint> waypoints = {
            {start + half_step + peak_extra, seg_time},
            {start + params_.step_over_angle + peak_extra, seg_time},
            {start + params_.step_over_angle, seg_time}
        };
        front_right_traj_.plan(start, waypoints);
    }

    lift_start_angles_ = current_ecc_angles;
    front_target_angles_(0) = current_ecc_angles(FL) + params_.step_over_angle;
    front_target_angles_(1) = current_ecc_angles(FR) + params_.step_over_angle;
}

void TaskController::planRearLiftTrajectories(const Eigen::Vector4d& current_ecc_angles) {
    // Same logic as front, but for rear legs
    double half_step = params_.step_over_angle * 0.5;
    double peak_extra = params_.lift_peak_angle_offset;
    double seg_time = params_.rear_lift_duration / 3.0;

    // Rear-Left
    {
        double start = current_ecc_angles(RL);
        std::vector<TrajectoryWaypoint> waypoints = {
            {start + half_step + peak_extra, seg_time},
            {start + params_.step_over_angle + peak_extra, seg_time},
            {start + params_.step_over_angle, seg_time}
        };
        rear_left_traj_.plan(start, waypoints);
    }

    // Rear-Right
    {
        double start = current_ecc_angles(RR);
        std::vector<TrajectoryWaypoint> waypoints = {
            {start + half_step + peak_extra, seg_time},
            {start + params_.step_over_angle + peak_extra, seg_time},
            {start + params_.step_over_angle, seg_time}
        };
        rear_right_traj_.plan(start, waypoints);
    }

    lift_start_angles_ = current_ecc_angles;
    rear_target_angles_(0) = current_ecc_angles(RL) + params_.step_over_angle;
    rear_target_angles_(1) = current_ecc_angles(RR) + params_.step_over_angle;
}

void TaskController::update(
    const estimation::ContactDetector& contact_detector,
    const Eigen::Vector4d& current_ecc_angles,
    double dt)
{
    if (dt <= 0.0 || dt > 0.5) return;

    state_timer_ += dt;

    // Clear overrides by default
    override_mask_.fill(false);
    forward_vel_override_ = 0.0;

    switch (state_) {
        // ============================================================
        // IDLE: Normal driving, monitoring for front wheel impact
        // ============================================================
        case ClimbState::IDLE: {
            if (contact_detector.frontImpactDetected()) {
                transitionTo(ClimbState::FRONT_CONTACT);
            }
            break;
        }

        // ============================================================
        // FRONT_CONTACT: Front wheels hit stair, prepare to lift
        // Freeze all motion, plan front leg trajectories
        // ============================================================
        case ClimbState::FRONT_CONTACT: {
            // Stop all wheel motion
            forward_vel_override_ = 0.0;

            // Plan the step-over trajectories
            planFrontLiftTrajectories(current_ecc_angles);

            // Immediately transition to lifting
            transitionTo(ClimbState::FRONT_LIFT);
            break;
        }

        // ============================================================
        // FRONT_LIFT: Execute front leg step-over trajectory
        // Override FL and FR eccentric joints
        // ============================================================
        case ClimbState::FRONT_LIFT: {
            override_mask_[FL] = true;
            override_mask_[FR] = true;

            double q_fl, dq_fl, ddq_fl;
            double q_fr, dq_fr, ddq_fr;
            front_left_traj_.evaluate(state_timer_, q_fl, dq_fl, ddq_fl);
            front_right_traj_.evaluate(state_timer_, q_fr, dq_fr, ddq_fr);

            ecc_overrides_(FL) = q_fl;
            ecc_overrides_(FR) = q_fr;

            // Check if trajectory is complete
            if (front_left_traj_.isComplete(state_timer_) &&
                front_right_traj_.isComplete(state_timer_)) {
                transitionTo(ClimbState::FRONT_PLACED);
            }
            break;
        }

        // ============================================================
        // FRONT_PLACED: Front wheels on top, drive body forward
        // Wait for rear wheels to reach the obstacle
        // ============================================================
        case ClimbState::FRONT_PLACED: {
            // Maintain front wheels on top of curb
            override_mask_[FL] = true;
            override_mask_[FR] = true;
            ecc_overrides_(FL) = front_target_angles_(0);
            ecc_overrides_(FR) = front_target_angles_(1);

            // Command forward motion to advance the body
            forward_vel_override_ = params_.forward_drive_speed;

            // Monitor for rear wheel impact
            if (contact_detector.rearImpactDetected()) {
                transitionTo(ClimbState::REAR_CONTACT);
            } else if (state_timer_ > params_.forward_drive_duration) {
                // Safety timeout: if we've been driving too long without rear contact
                transitionTo(ClimbState::REAR_CONTACT);
            }
            break;
        }

        // ============================================================
        // REAR_CONTACT: Rear wheels hit stair, prepare to lift
        // ============================================================
        case ClimbState::REAR_CONTACT: {
            // Maintain front wheels on top of curb
            override_mask_[FL] = true;
            override_mask_[FR] = true;
            ecc_overrides_(FL) = front_target_angles_(0);
            ecc_overrides_(FR) = front_target_angles_(1);

            forward_vel_override_ = 0.0;
            planRearLiftTrajectories(current_ecc_angles);
            transitionTo(ClimbState::REAR_LIFT);
            break;
        }

        // ============================================================
        // REAR_LIFT: Execute rear leg step-over trajectory
        // Override FL and FR to hold curb height, RL and RR to trajectory
        // ============================================================
        case ClimbState::REAR_LIFT: {
            // Front wheels stay on top of curb
            override_mask_[FL] = true;
            override_mask_[FR] = true;
            ecc_overrides_(FL) = front_target_angles_(0);
            ecc_overrides_(FR) = front_target_angles_(1);

            override_mask_[RL] = true;
            override_mask_[RR] = true;

            double q_rl, dq_rl, ddq_rl;
            double q_rr, dq_rr, ddq_rr;
            rear_left_traj_.evaluate(state_timer_, q_rl, dq_rl, ddq_rl);
            rear_right_traj_.evaluate(state_timer_, q_rr, dq_rr, ddq_rr);

            ecc_overrides_(RL) = q_rl;
            ecc_overrides_(RR) = q_rr;

            if (rear_left_traj_.isComplete(state_timer_) &&
                rear_right_traj_.isComplete(state_timer_)) {
                transitionTo(ClimbState::REAR_PLACED);
            }
            break;
        }

        // ============================================================
        // REAR_PLACED: All wheels on top, move forward a certain distance (Step 6)
        // ============================================================
        case ClimbState::REAR_PLACED: {
            // Maintain all 4 wheels on top of curb
            override_mask_[FL] = true;
            override_mask_[FR] = true;
            ecc_overrides_(FL) = front_target_angles_(0);
            ecc_overrides_(FR) = front_target_angles_(1);

            override_mask_[RL] = true;
            override_mask_[RR] = true;
            ecc_overrides_(RL) = rear_target_angles_(0);
            ecc_overrides_(RR) = rear_target_angles_(1);

            // Command forward motion to advance the body fully onto the obstacle
            forward_vel_override_ = params_.forward_drive_speed;

            if (state_timer_ >= params_.settling_duration) {
                transitionTo(ClimbState::IDLE);
            }
            break;
        }
    }
}

std::array<bool, NUM_LEGS> TaskController::getContactValid() const {
    switch (state_) {
        case ClimbState::FRONT_LIFT:
            return {false, false, true, true};
        case ClimbState::REAR_LIFT:
            return {true, true, false, false};
        default:
            return {true, true, true, true};
    }
}

}  // namespace tasks
}  // namespace robot_control
