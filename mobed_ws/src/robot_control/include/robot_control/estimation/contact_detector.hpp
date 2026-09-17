#pragma once

#include <Eigen/Dense>
#include <array>
#include <deque>
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"

namespace robot_control {
namespace estimation {

/**
 * @brief Contact and impact detector using joint torque feedback.
 *
 * Monitors the eccentric joint and wheel joint effort (torque) signals
 * to detect sudden impacts — specifically, the moment a wheel collides
 * with a stair or curb edge. Detection is based on two complementary
 * methods:
 *
 * 1. Absolute threshold: |τ| > τ_threshold
 * 2. Rate-of-change threshold: |dτ/dt| > τ_rate_threshold
 *
 * A sliding window is used to compute a smoothed torque derivative,
 * filtering out sensor noise while preserving sharp impact transients.
 */
class ContactDetector {
public:
    explicit ContactDetector(const RobotParams& params = RobotParams());

    /**
     * @brief Feed new joint effort data and update detection state.
     *
     * Should be called at the control loop rate (e.g., 100 Hz).
     *
     * @param ecc_efforts   Eccentric joint torques [FL, FR, RL, RR] (Nm)
     * @param wheel_efforts Wheel joint torques [FL, FR, RL, RR] (Nm)
     * @param dt            Time step since last call (seconds)
     */
    void update(const Eigen::Vector4d& ecc_efforts,
                const Eigen::Vector4d& wheel_efforts,
                double dt);

    /**
     * @brief Check if an impact was detected on any front leg.
     * @return true if FL or FR detected a collision this cycle
     */
    bool frontImpactDetected() const;

    /**
     * @brief Check if an impact was detected on any rear leg.
     * @return true if RL or RR detected a collision this cycle
     */
    bool rearImpactDetected() const;

    /**
     * @brief Get per-leg impact detection flags.
     * @return Array of 4 booleans [FL, FR, RL, RR]
     */
    std::array<bool, NUM_LEGS> getImpactFlags() const { return impact_flags_; }

    /**
     * @brief Get the smoothed torque rate (dτ/dt) for each eccentric joint.
     * @return Eigen::Vector4d torque rates [FL, FR, RL, RR] (Nm/s)
     */
    Eigen::Vector4d getEccTorqueRates() const { return ecc_torque_rates_; }

    /**
     * @brief Reset all detection state and history buffers.
     */
    void reset();

private:
    RobotParams params_;

    // Per-leg impact detection flags (true for one cycle after detection)
    std::array<bool, NUM_LEGS> impact_flags_;

    // Smoothed torque rates
    Eigen::Vector4d ecc_torque_rates_;
    Eigen::Vector4d wheel_torque_rates_;

    // Sliding window history for torque derivative computation
    static constexpr int WINDOW_SIZE = 5;
    std::array<std::deque<double>, NUM_LEGS> ecc_effort_history_;
    std::array<std::deque<double>, NUM_LEGS> wheel_effort_history_;

    // Previous effort values for finite difference
    Eigen::Vector4d prev_ecc_efforts_;
    Eigen::Vector4d prev_wheel_efforts_;
    bool initialized_ = false;

    /**
     * @brief Compute smoothed derivative using sliding window average.
     *
     * Uses a finite difference over the window to reduce noise sensitivity
     * while preserving fast transient response.
     *
     * @param history  Reference to the sliding window deque
     * @param new_val  New measurement value
     * @param dt       Time step
     * @return Smoothed derivative (units/s)
     */
    double computeSmoothedRate(std::deque<double>& history, double new_val, double dt) const;
};

}  // namespace estimation
}  // namespace robot_control
