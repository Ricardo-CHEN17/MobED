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

    // Chassis blockage gate:
    // A true obstacle (>=5cm vertical stair) physically halts the robot's forward progress,
    // causing forward chassis velocity to drop to near zero (vx < chassis_blocked_vel_threshold).
    // If the chassis is still moving forward (vx > 0.06 m/s), the robot is simply rolling over
    // a 2~3cm bump, thin board lip, or speed bump — suspension compliance absorbs it without climbing!
    bool chassis_blocked = (cmd_forward_vel > 0.08) &&
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
        // 2. Two-Stage Impact Detection (Shock Onset + Sustained Load):
        //
        // Stage 1 (Shock Onset):
        // When a sudden transient spike in eccentric torque rate occurs,
        // arm a 200 ms collision observation window.
        // ============================================================
        if (std::abs(ecc_torque_rates_(i)) > params_.contact_torque_rate_threshold) {
            shock_window_timer_[i] = params_.shock_window_duration;
        }

        // ============================================================
        // Stage 2 (Sustained Load Confirmation):
        // While within the shock window, if:
        //   1. The chassis forward progress is rigidly blocked (chassis_blocked)
        //   2. The eccentric link experiences sustained resistive torque load (|τ_ecc| > contact_torque_threshold)
        //   3. The drive wheel is truly stalled against a vertical face (|τ_wheel| > 8.0 Nm AND |ω_wheel| < 0.4 rad/s)
        // accumulate the sustain confirmation timer.
        //
        // Once sustained for >= sustain_confirm_duration (0.18s / 9 frames),
        // confirm the obstacle impact.
        // ============================================================
        if (shock_window_timer_[i] > 0.0) {
            shock_window_timer_[i] = std::max(0.0, shock_window_timer_[i] - dt);

            bool ecc_loaded = (std::abs(ecc_efforts(i)) > params_.contact_torque_threshold);
            bool wheel_stalled = (std::abs(wheel_efforts(i)) > params_.wheel_stall_torque_threshold) &&
                                 (std::abs(wheel_velocities(i)) < params_.wheel_stall_vel_threshold);

            if (chassis_blocked && ecc_loaded && wheel_stalled) {
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
