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
    front_evidence_score_ = 0.0;
    front_evidence_sustain_timer_ = 0.0;
    prev_cmd_vel_ = 0.0;
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
    double imu_accel_x,
    const ClimbGeometry& geometry,
    double dt)
{
    if (dt <= 0.0 || dt > 0.5) return;

    // First call: just store the initial values
    if (!initialized_) {
        prev_ecc_efforts_ = ecc_efforts;
        prev_wheel_efforts_ = wheel_efforts;
        prev_cmd_vel_ = cmd_forward_vel;
        initialized_ = true;
        startup_timer_ = 0.0;
        front_evidence_score_ = 0.0;
        front_evidence_sustain_timer_ = 0.0;
        return;
    }

    // Clear impact flags from previous cycle
    impact_flags_.fill(false);

    // Compute teleop command acceleration rate (for joystick ramping transient inhibit)
    double cmd_accel = (cmd_forward_vel - prev_cmd_vel_) / dt;
    prev_cmd_vel_ = cmd_forward_vel;
    bool manual_accel_transient = (cmd_accel > 0.40);

    // Forward command gate: only check impact when driving forward
    bool forward_cmd_active = (cmd_forward_vel > 0.05);

    // Startup window tracking to eliminate false alarms during initial acceleration from rest
    if (forward_cmd_active) {
        startup_timer_ += dt;
    } else {
        startup_timer_ = 0.0;
    }

    bool in_startup_phase = (startup_timer_ < 0.35) && (std::abs(geometry.delta_height) < 0.035);

    // Chassis blockage gate:
    // A true obstacle physically halts forward progress (vx < chassis_blocked_vel_threshold)
    bool chassis_blocked = forward_cmd_active && !in_startup_phase &&
                           (chassis_forward_vel < params_.chassis_blocked_vel_threshold);

    // Update derivatives and per-wheel torque rates
    for (int i = 0; i < NUM_LEGS; ++i) {
        ecc_torque_rates_(i) = computeSmoothedRate(
            ecc_effort_history_[i], ecc_efforts(i), dt);
        wheel_torque_rates_(i) = computeSmoothedRate(
            wheel_effort_history_[i], wheel_efforts(i), dt);
    }

    // 1. Per-wheel stall checks
    bool fl_stalled = (std::abs(wheel_efforts(FL)) > params_.wheel_stall_torque_threshold) &&
                      (std::abs(wheel_velocities(FL)) < params_.wheel_stall_vel_threshold);
    bool fr_stalled = (std::abs(wheel_efforts(FR)) > params_.wheel_stall_torque_threshold) &&
                      (std::abs(wheel_velocities(FR)) < params_.wheel_stall_vel_threshold);
    bool front_axle_stalled = fl_stalled || fr_stalled;

    // 2. Bilateral front axle torque metrics
    double tau_whl_avg = 0.5 * (std::abs(wheel_efforts(FL)) + std::abs(wheel_efforts(FR)));
    double tau_ecc_avg = 0.5 * (std::abs(ecc_efforts(FL)) + std::abs(ecc_efforts(FR)));
    double tau_ecc_max = std::max(std::abs(ecc_efforts(FL)), std::abs(ecc_efforts(FR)));

    // 3. Kinematic velocity drop shock & deceleration shock
    double vel_drop = cmd_forward_vel - chassis_forward_vel;
    bool vel_drop_shock = forward_cmd_active && (vel_drop > 0.10);
    bool decel_shock = vel_drop_shock || (forward_cmd_active && (imu_accel_x < -params_.kinematic_decel_shock_thresh));

    // 4. Flat ground suppression: if wheels are rolling freely on horizontal ground with normal cruising loads
    bool flat_rolling = (std::abs(geometry.delta_height) < 0.025) &&
                        (geometry.nose_up_pitch < 0.04) &&
                        (!front_axle_stalled) &&
                        (!decel_shock) &&
                        (tau_whl_avg < 2.8) &&
                        (tau_ecc_avg < 3.2);

    // 5. Multi-modal obstacle evidence calculation
    double evidence_in = 0.0;
    if (forward_cmd_active && !in_startup_phase && !flat_rolling) {
        // Evidence from bilateral drive wheel load (nominal flat cruising is ~1.5 Nm, wall obstacle is >3.5 Nm)
        double whl_ev = std::clamp((tau_whl_avg - 2.2) / 3.0, 0.0, 1.0);

        // Evidence from eccentric joint impact load (gravity baseline is ~2.0 Nm, wall collision is >4.5 Nm)
        double ecc_ev = std::clamp((tau_ecc_max - 3.8) / 4.0, 0.0, 1.0);

        // Evidence from velocity drop / deceleration shock
        double shock_ev = decel_shock ? 0.85 : 0.0;

        // Evidence from true wheel stall
        double stall_ev = front_axle_stalled ? 0.90 : 0.0;

        // Evidence from geometric wall climbing
        double geom_ev = geometry.front_wall_climbing ? 1.0 : 0.0;

        // Inhibitor: if user is aggressively throttling up on flat terrain without decel shock or ecc load
        if (manual_accel_transient && !decel_shock && !front_axle_stalled && tau_ecc_max < 3.0) {
            whl_ev *= 0.2;
            ecc_ev *= 0.2;
        }

        // Bilateral coincidence: require at least two distinct modalities or high-confidence stall/geometry
        evidence_in = std::max({
            geom_ev,
            stall_ev,
            0.45 * whl_ev + 0.45 * ecc_ev + 0.35 * shock_ev,
            chassis_blocked ? (0.6 * whl_ev + 0.4 * ecc_ev) : 0.0
        });
    }

    // 6. Continuous Leaky-Integrator Evidence Update
    if (evidence_in > 0.35) {
        // Accumulate evidence quickly (reaches 0.75 threshold in ~0.08s)
        front_evidence_score_ = std::min(1.0, front_evidence_score_ + (evidence_in * 8.0) * dt);
    } else {
        // Rapid exponential decay during unblocked cruising
        front_evidence_score_ = std::max(0.0, front_evidence_score_ - params_.contact_evidence_decay_rate * dt);
    }

    // 7. Confirmed front impact when evidence score sustains threshold
    if (front_evidence_score_ >= params_.contact_evidence_threshold) {
        front_evidence_sustain_timer_ += dt;
        if (front_evidence_sustain_timer_ >= 0.05) {
            impact_flags_[FL] = true;
            impact_flags_[FR] = true;
        }
    } else {
        front_evidence_sustain_timer_ = 0.0;
    }

    // 8. Direct geometric wall-climbing confirmation
    if (forward_cmd_active && !in_startup_phase && geometry.front_wall_climbing) {
        impact_flags_[FL] = true;
        impact_flags_[FR] = true;
    }

    // 9. Rear wall contact confirmation
    // If front wheels are already on top (delta_height > 0.04m) and rear wheels hit curb face
    if (forward_cmd_active && geometry.delta_height > 0.04) {
        bool rear_stalled = (std::abs(wheel_velocities(RL)) < params_.wheel_stall_vel_threshold ||
                             std::abs(wheel_velocities(RR)) < params_.wheel_stall_vel_threshold) &&
                            (std::abs(wheel_efforts(RL)) > 2.5 || std::abs(wheel_efforts(RR)) > 2.5);
        double tau_rear_max = std::max(std::abs(wheel_efforts(RL)), std::abs(wheel_efforts(RR)));
        if (rear_stalled || (chassis_blocked && tau_rear_max > 2.2) || (tau_rear_max > 4.5)) {
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
