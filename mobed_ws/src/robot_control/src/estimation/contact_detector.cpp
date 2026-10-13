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
    startup_timer_ = 0.0;
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
    const ClimbGeometry& geometry,
    double dt)
{
    if (dt <= 0.0 || dt > 0.5) return;

    // First call: just store the initial values
    if (!initialized_) {
        prev_ecc_efforts_ = ecc_efforts;
        prev_wheel_efforts_ = wheel_efforts;
        initialized_ = true;
        startup_timer_ = 0.0;
        return;
    }

    // Clear impact flags from previous cycle
    impact_flags_.fill(false);

    // Forward command gate: only check impact when driving forward
    bool forward_cmd_active = (cmd_forward_vel > 0.05);

    // Startup window tracking to eliminate false alarms during initial acceleration
    if (forward_cmd_active) {
        startup_timer_ += dt;
    } else {
        startup_timer_ = 0.0;
    }

    // In startup acceleration window (first 0.35s), on flat ground (|delta_height| < 35mm)
    // with un-stalled wheels, suppress torque spike false alarms
    bool in_startup_phase = (startup_timer_ < 0.35) && (std::abs(geometry.delta_height) < 0.035);

    // Chassis blockage gate:
    // A true obstacle physically halts forward progress (vx < chassis_blocked_vel_threshold).
    bool chassis_blocked = forward_cmd_active && !in_startup_phase &&
                           (chassis_forward_vel < params_.chassis_blocked_vel_threshold);

    for (int i = 0; i < NUM_LEGS; ++i) {
        // 1. Compute smoothed torque rates (dτ/dt)
        ecc_torque_rates_(i) = computeSmoothedRate(
            ecc_effort_history_[i], ecc_efforts(i), dt);

        wheel_torque_rates_(i) = computeSmoothedRate(
            wheel_effort_history_[i], wheel_efforts(i), dt);

        // 2. Multi-Modal Impact & Blockage Detection:
        bool is_front_wheel = (i == FL || i == FR);
        double ecc_thresh = is_front_wheel ? params_.contact_torque_threshold : 2.5;
        bool ecc_loaded = (std::abs(ecc_efforts(i)) > ecc_thresh);

        bool wheel_stalled = (std::abs(wheel_efforts(i)) > params_.wheel_stall_torque_threshold) &&
                             (std::abs(wheel_velocities(i)) < params_.wheel_stall_vel_threshold);

        // Obstacle-blocked slip: forward command active, chassis halted, wheel heavily loaded (>2.8 Nm)
        bool wheel_slipping_blocked = forward_cmd_active && chassis_blocked &&
                                     (std::abs(wheel_efforts(i)) > 2.8);

        // Stage 1 (Shock Onset / Obstacle Face Arrival):
        bool shock_onset = forward_cmd_active && !in_startup_phase && (
            (std::abs(ecc_torque_rates_(i)) > params_.contact_torque_rate_threshold) ||
            (std::abs(wheel_torque_rates_(i)) > 35.0) ||
            wheel_stalled ||
            wheel_slipping_blocked
        );

        if (shock_onset) {
            shock_window_timer_[i] = params_.shock_window_duration;
        }

        // Stage 2 (Per-Wheel Sustained Load Confirmation):
        if (shock_window_timer_[i] > 0.0) {
            shock_window_timer_[i] = std::max(0.0, shock_window_timer_[i] - dt);

            bool contact_sustained = forward_cmd_active && (
                wheel_stalled ||
                (chassis_blocked && (ecc_loaded || std::abs(wheel_efforts(i)) > 3.0)) ||
                (wheel_slipping_blocked && (ecc_loaded || std::abs(wheel_efforts(i)) > 3.0)) ||
                (std::abs(wheel_efforts(i)) > 6.0)
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

    // Direct geometric wall-climbing confirmation:
    // When front wheels are actively rolling up a vertical face (e.g. 200mm wall where tires don't stall)
    if (forward_cmd_active && !in_startup_phase && geometry.front_wall_climbing) {
        impact_flags_[FL] = true;
        impact_flags_[FR] = true;
    }

    // Rear wall contact confirmation:
    // If front wheels are already on top (delta_height > 0.04m) and rear wheels hit the curb face,
    // rear wheels stall or pitch rate drops sharply as rear rises
    if (forward_cmd_active && geometry.delta_height > 0.04) {
        bool rear_stalled = (std::abs(wheel_velocities(RL)) < params_.wheel_stall_vel_threshold ||
                             std::abs(wheel_velocities(RR)) < params_.wheel_stall_vel_threshold) &&
                            (std::abs(wheel_efforts(RL)) > 2.0 || std::abs(wheel_efforts(RR)) > 2.0);
        if (rear_stalled || (chassis_blocked && std::abs(wheel_efforts(RL)) > 1.8)) {
            impact_flags_[RL] = true;
            impact_flags_[RR] = true;
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
