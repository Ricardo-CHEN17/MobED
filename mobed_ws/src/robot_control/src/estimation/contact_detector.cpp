#include "robot_control/estimation/contact_detector.hpp"
#include <cmath>
#include <algorithm>

namespace robot_control {
namespace estimation {

ContactDetector::ContactDetector(const RobotParams& params)
    : params_(params)
{
    reset();
}

void ContactDetector::reset() {
    impact_flags_.fill(false);
    shock_window_timer_.fill(0.0);
    sustain_timer_.fill(0.0);
    ecc_torque_rates_ = Eigen::Vector4d::Zero();
    wheel_torque_rates_ = Eigen::Vector4d::Zero();
    prev_ecc_efforts_ = Eigen::Vector4d::Zero();
    prev_wheel_efforts_ = Eigen::Vector4d::Zero();
    initialized_ = false;

    for (int i = 0; i < NUM_LEGS; ++i) {
        ecc_effort_history_[i].clear();
        wheel_effort_history_[i].clear();
    }
}

double ContactDetector::computeSmoothedRate(
    std::deque<double>& history, double new_val, double dt) const
{
    // ================================================================
    // Sliding window derivative computation
    //
    // Instead of a simple (new - old) / dt which is noisy, we maintain
    // a window of recent values and compute the average slope.
    //
    // rate = (newest - oldest) / (window_size * dt)
    //
    // This smooths out sensor noise while still detecting sharp impacts
    // within a few control cycles.
    // ================================================================

    history.push_back(new_val);

    // Maintain window size
    while (static_cast<int>(history.size()) > WINDOW_SIZE) {
        history.pop_front();
    }

    if (history.size() < 2 || dt <= 0.0) {
        return 0.0;
    }

    double oldest = history.front();
    double newest = history.back();
    double window_dt = dt * static_cast<double>(history.size() - 1);

    if (window_dt <= 0.0) return 0.0;

    return (newest - oldest) / window_dt;
}

void ContactDetector::update(
    const Eigen::Vector4d& ecc_efforts,
    const Eigen::Vector4d& wheel_efforts,
    const Eigen::Vector4d& wheel_velocities,
    double chassis_forward_vel,
    double cmd_forward_vel,
    double dt)
{
    if (dt <= 0.0 || dt > 0.5) return;

    // First call: just store the initial values
    if (!initialized_) {
        prev_ecc_efforts_ = ecc_efforts;
        prev_wheel_efforts_ = wheel_efforts;
        initialized_ = true;
        return;
    }

    // Clear impact flags from previous cycle
    impact_flags_.fill(false);

    // Forward command gate: only check impact when driving forward
    bool forward_cmd_active = (cmd_forward_vel > 0.05);

    // Chassis blockage gate:
    // A true obstacle (>=5cm vertical stair/wall) physically halts the robot's forward progress,
    // causing forward chassis velocity to drop to near zero (vx < chassis_blocked_vel_threshold).
    bool chassis_blocked = forward_cmd_active &&
                           (chassis_forward_vel < params_.chassis_blocked_vel_threshold);

    for (int i = 0; i < NUM_LEGS; ++i) {
        // ============================================================
        // 1. Compute smoothed torque rates (dτ/dt)
        // ============================================================
        ecc_torque_rates_(i) = computeSmoothedRate(
            ecc_effort_history_[i], ecc_efforts(i), dt);

        wheel_torque_rates_(i) = computeSmoothedRate(
            wheel_effort_history_[i], wheel_efforts(i), dt);

        // ============================================================
        // 2. Multi-Modal Impact & Blockage Detection:
        //
        // Accounts for wheel slip on vertical obstacle faces where the drive
        // wheel spins (1.3~3.0 rad/s) without dropping below stall velocity threshold.
        // ============================================================
        bool is_front_wheel = (i == FL || i == FR);
        double ecc_thresh = is_front_wheel ? params_.contact_torque_threshold : 2.5;
        bool ecc_loaded = (std::abs(ecc_efforts(i)) > ecc_thresh);

        bool wheel_stalled = (std::abs(wheel_efforts(i)) > params_.wheel_stall_torque_threshold) &&
                             (std::abs(wheel_velocities(i)) < params_.wheel_stall_vel_threshold);

        // Obstacle-blocked slip: forward command active, chassis halted, wheel heavily loaded (>2.8 Nm)
        bool wheel_slipping_blocked = forward_cmd_active && chassis_blocked &&
                                     (std::abs(wheel_efforts(i)) > 2.8);

        // Stage 1 (Shock Onset / Obstacle Face Arrival):
        // Arm observation window if:
        // - Eccentric torque rate spike (|dτ/dt| > 25 Nm/s), OR
        // - Wheel torque rate spike (|dτ/dt| > 35 Nm/s), OR
        // - Drive wheel stalled, OR
        // - Chassis blocked with heavily loaded drive wheel
        bool shock_onset = (std::abs(ecc_torque_rates_(i)) > params_.contact_torque_rate_threshold) ||
                           (std::abs(wheel_torque_rates_(i)) > 35.0) ||
                           (forward_cmd_active && wheel_stalled) ||
                           (forward_cmd_active && wheel_slipping_blocked);

        if (shock_onset) {
            shock_window_timer_[i] = params_.shock_window_duration;
        }

        // ============================================================
        // Stage 2 (Per-Wheel Sustained Load Confirmation):
        // While within the shock window:
        // Accumulate sustain timer when sustained forward driving is verified
        // by stall, eccentric load + drive torque, or blocked slip.
        // ============================================================
        if (shock_window_timer_[i] > 0.0) {
            shock_window_timer_[i] = std::max(0.0, shock_window_timer_[i] - dt);

            bool contact_sustained = forward_cmd_active && (
                wheel_stalled ||
                (ecc_loaded && std::abs(wheel_efforts(i)) > 2.8) ||
                (wheel_slipping_blocked && (ecc_loaded || std::abs(wheel_efforts(i)) > 3.0))
            );

            if (contact_sustained) {
                sustain_timer_[i] += dt;
                if (sustain_timer_[i] >= params_.sustain_confirm_duration) {
                    impact_flags_[i] = true;
                }
            } else {
                sustain_timer_[i] = std::max(0.0, sustain_timer_[i] - dt);
            }
        } else {
            sustain_timer_[i] = 0.0;
        }
    }

    // Store for next cycle
    prev_ecc_efforts_ = ecc_efforts;
    prev_wheel_efforts_ = wheel_efforts;
}

bool ContactDetector::frontImpactDetected() const {
    return impact_flags_[FL] || impact_flags_[FR];
}

bool ContactDetector::rearImpactDetected() const {
    return impact_flags_[RL] || impact_flags_[RR];
}

}  // namespace estimation
}  // namespace robot_control
