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

    for (int i = 0; i < NUM_LEGS; ++i) {
        // ============================================================
        // 1. Compute smoothed torque rates (dτ/dt)
        // ============================================================
        ecc_torque_rates_(i) = computeSmoothedRate(
            ecc_effort_history_[i], ecc_efforts(i), dt);

        wheel_torque_rates_(i) = computeSmoothedRate(
            wheel_effort_history_[i], wheel_efforts(i), dt);

        // ============================================================
        // 2. Impact detection: dual-criteria check
        //
        // An impact is flagged if EITHER condition is met:
        //   (a) Absolute torque exceeds threshold (sustained contact)
        //   (b) Torque rate exceeds threshold (sudden collision)
        //
        // We check both eccentric and wheel joints, since the impact
        // signature appears on both depending on the collision geometry.
        // ============================================================

        bool abs_trigger_ecc   = std::abs(ecc_efforts(i))   > params_.contact_torque_threshold;
        bool abs_trigger_wheel = std::abs(wheel_efforts(i)) > params_.contact_torque_threshold;

        bool rate_trigger_ecc   = std::abs(ecc_torque_rates_(i))   > params_.contact_torque_rate_threshold;
        bool rate_trigger_wheel = std::abs(wheel_torque_rates_(i)) > params_.contact_torque_rate_threshold;

        // Impact = (absolute OR rate) on (ecc OR wheel)
        impact_flags_[i] = (abs_trigger_ecc || rate_trigger_ecc ||
                            abs_trigger_wheel || rate_trigger_wheel);
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
