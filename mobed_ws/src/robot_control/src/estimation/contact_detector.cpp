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
    ecc_effort_baseline_ = Eigen::Vector4d::Zero();
    delta_tau_ecc_ = Eigen::Vector4d::Zero();
    momentum_residual_ = 0.0;
    momentum_observer_z_ = 0.0;
    rear_impact_confirm_cycles_ = 0;
    impact_confirm_cycles_ = 0;
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
        ecc_effort_baseline_ = ecc_efforts;
        momentum_observer_z_ = params_.total_mass * chassis_forward_vel;
        momentum_residual_ = 0.0;
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

    // Forward command gate: only check impact when driving forward with intentional speed
    bool forward_cmd_active = (cmd_forward_vel > 0.08);

    // Startup window tracking to eliminate false alarms during initial acceleration from rest
    if (forward_cmd_active) {
        startup_timer_ += dt;
    } else {
        startup_timer_ = 0.0;
    }

    // Startup acceleration blanking window:
    // Inhibit impact detection during initial startup/acceleration from rest (first 0.5s)
    // or during aggressive throttle ramping to completely eliminate false positives.
    if (startup_timer_ < 0.50 || manual_accel_transient) {
        front_evidence_score_ = 0.0;
        front_evidence_sustain_timer_ = 0.0;
        impact_confirm_cycles_ = 0;
        impact_flags_.fill(false);
        prev_ecc_efforts_ = ecc_efforts;
        prev_wheel_efforts_ = wheel_efforts;
        return;
    }

    // Update derivatives, per-wheel torque rates, and eccentric baseline
    for (int i = 0; i < NUM_LEGS; ++i) {
        ecc_torque_rates_(i) = computeSmoothedRate(
            ecc_effort_history_[i], ecc_efforts(i), dt);
        wheel_torque_rates_(i) = computeSmoothedRate(
            wheel_effort_history_[i], wheel_efforts(i), dt);

        delta_tau_ecc_(i) = ecc_efforts(i) - ecc_effort_baseline_(i);
        double alpha_ecc = std::clamp(dt / (dt + 0.40), 0.0, 1.0);
        ecc_effort_baseline_(i) += alpha_ecc * (ecc_efforts(i) - ecc_effort_baseline_(i));
    }

    // Update Generalized Momentum Observer:
    // p = M * vx
    // \dot{z} = F_drive + r
    // r = K_O * (p - z)
    double total_drive_force = 0.0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        total_drive_force += wheel_efforts(i) / params_.r_wheel;
    }
    double p_momentum = params_.total_mass * chassis_forward_vel;
    if (forward_cmd_active) {
        double k_obs = 15.0; // Observer gain (1/s)
        momentum_observer_z_ += (total_drive_force + momentum_residual_) * dt;
        momentum_residual_ = k_obs * (p_momentum - momentum_observer_z_);
    } else {
        momentum_observer_z_ = p_momentum;
        momentum_residual_ = 0.0;
    }

    // 1. True physical wheel stall checks
    // When wheels hit a vertical wall face, actual wheel speed drops near zero while motor torque saturates
    double stall_tau_thresh = std::min(wheel_stall_torque_min_, 0.85 * 10.0 * (cmd_forward_vel / params_.r_wheel));
    bool fl_stalled = (std::abs(wheel_efforts(FL)) >= stall_tau_thresh) &&
                      (std::abs(wheel_velocities(FL)) < wheel_stall_vel_max_);
    bool fr_stalled = (std::abs(wheel_efforts(FR)) >= stall_tau_thresh) &&
                      (std::abs(wheel_velocities(FR)) < wheel_stall_vel_max_);
    bool front_axle_stalled = fl_stalled && fr_stalled;
    bool single_wheel_stalled = (fl_stalled || fr_stalled) &&
                                (0.5 * (std::abs(wheel_efforts(FL)) + std::abs(wheel_efforts(FR))) >= stall_tau_thresh) &&
                                (0.5 * (std::abs(wheel_velocities(FL)) + std::abs(wheel_velocities(FR))) < 0.8);

    // 2. Chassis halted & IMU collision shock metrics
    bool chassis_halted = (chassis_forward_vel < 0.08);
    bool imu_impact_shock = (imu_accel_x < imu_decel_shock_threshold_);

    // 3. Multi-modal obstacle evidence calculation (Strict Consistency Gate)
    // Note: Eccentric arm torque is NEVER used as primary impact source because flat-ground pitch stabilization
    // produces normal torque spikes. Obstacle impact is strictly determined by wheel stall + IMU shock + chassis halt.
    double evidence_in = 0.0;
    if (forward_cmd_active) {
        if (geometry.front_wall_climbing) {
            evidence_in = 1.0;
        } else if (front_axle_stalled && (imu_impact_shock || chassis_halted)) {
            evidence_in = 1.0;
        } else if (single_wheel_stalled && imu_impact_shock && chassis_halted) {
            evidence_in = 0.90;
        }
    }

    // 4. Continuous Leaky-Integrator Evidence Update with consecutive cycle counter
    if (evidence_in > 0.5) {
        impact_confirm_cycles_++;
        front_evidence_score_ = std::min(1.0, front_evidence_score_ + (evidence_in * 20.0) * dt);
    } else {
        impact_confirm_cycles_ = 0;
        front_evidence_score_ = std::max(0.0, front_evidence_score_ - 15.0 * dt);
    }

    // 5. Confirmed front impact when evidence score sustains threshold AND consecutive cycles met (30ms debounce)
    if (front_evidence_score_ >= 0.75 && impact_confirm_cycles_ >= REQUIRED_CONFIRM_CYCLES) {
        front_evidence_sustain_timer_ += dt;
        if (front_evidence_sustain_timer_ >= 0.03) {
            impact_flags_[FL] = true;
            impact_flags_[FR] = true;
        }
    } else {
        front_evidence_sustain_timer_ = 0.0;
    }

    // 6. Rear wall contact confirmation (Dual-Track Arrival Observer: Continuous high-speed + Low-speed/Static bump)
    // Front wheels are already crested or high on curb (delta_height > 0.030m or front_edge_crested)
    bool front_on_plateau = (geometry.delta_height > 0.030) || geometry.front_edge_crested;
    if (forward_cmd_active && front_on_plateau) {
        // Drive wheel effort resistance
        double tau_rear_max = std::max(std::abs(wheel_efforts(RL)), std::abs(wheel_efforts(RR)));
        bool rear_wheel_resisting = (tau_rear_max > 2.0);

        // Fast channel: rear eccentric reaction torque step or high rate
        bool rear_reaction_spike = (std::abs(delta_tau_ecc_(RL)) > 1.8 || std::abs(delta_tau_ecc_(RR)) > 1.8) ||
                                   (std::abs(ecc_torque_rates_(RL)) > 12.0 || std::abs(ecc_torque_rates_(RR)) > 12.0);

        // Track 1: Continuous high-speed arrival
        // Robot has advanced at least wheelbase distance onto plateau (>= 0.22m) and rear wheels slip against curb face
        bool continuous_arrival = (geometry.travel_distance >= 0.22) &&
                                  (geometry.rear_slip_ratio > 0.65 || rear_wheel_resisting || chassis_halted);

        // Track 2: Low-speed / Creeping / Static bump arrival
        bool momentum_arrested = (momentum_residual_ < -10.0) || (chassis_forward_vel < 0.06);
        bool rear_stalled = (std::abs(wheel_velocities(RL)) < params_.wheel_stall_vel_threshold ||
                             std::abs(wheel_velocities(RR)) < params_.wheel_stall_vel_threshold) &&
                            rear_wheel_resisting;
        bool static_bump_arrival = (rear_reaction_spike && (momentum_arrested || chassis_halted)) ||
                                   (rear_wheel_resisting && (momentum_arrested || chassis_halted)) ||
                                   rear_stalled ||
                                   (tau_rear_max > 3.2);

        // Fused contact condition
        bool contact_evidence = continuous_arrival || static_bump_arrival;

        if (contact_evidence) {
            rear_impact_confirm_cycles_++;
            if (rear_impact_confirm_cycles_ >= REQUIRED_REAR_CONFIRM_CYCLES) {
                impact_flags_[RL] = true;
                impact_flags_[RR] = true;
            }
        } else {
            rear_impact_confirm_cycles_ = std::max(0, rear_impact_confirm_cycles_ - 1);
        }
    } else {
        rear_impact_confirm_cycles_ = 0;
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
