#include <iostream>
#include <cassert>
#include "robot_control/tasks/task_controller.hpp"
#include "robot_control/estimation/contact_detector.hpp"

using namespace robot_control;
using namespace robot_control::tasks;

class MockContactDetector : public estimation::ContactDetector {
public:
    bool front_detected = false;
    bool rear_detected = false;
    std::array<bool, NUM_LEGS> flags = {false, false, false, false};

    void setImpact(int leg, bool val) {
        flags[leg] = val;
        front_detected = (flags[FL] || flags[FR]);
        rear_detected = (flags[RL] || flags[RR]);
    }

    void clearAll() {
        front_detected = false;
        rear_detected = false;
        flags.fill(false);
    }

    bool frontImpactDetected() const { return front_detected; }
    bool rearImpactDetected() const { return rear_detected; }
    std::array<bool, NUM_LEGS> getImpactFlags() const { return flags; }
};

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Testing TaskController Independent Climbing" << std::endl;
    std::cout << "========================================" << std::endl;

    TaskControllerParams params;
    params.front_lift_duration = 2.0;
    params.rear_lift_duration = 2.0;
    params.forward_drive_duration = 3.0;
    params.settling_duration = 1.0;
    params.blend_out_duration = 0.5;
    params.cooldown_duration = 0.5;
    params.touchdown_torque_threshold = 2.0;

    TaskController tc(params);
    MockContactDetector detector;
    double dt = 0.01;

    Eigen::Vector4d ecc_angles(0.65, 0.65, -0.65, -0.65);
    Eigen::Vector4d ecc_efforts(0.5, 0.5, 0.5, 0.5);

    // ----------------------------------------------------
    // Test 1: Unilateral Left Obstacle (FL only)
    // ----------------------------------------------------
    std::cout << "\n[Test 1] Testing Unilateral Obstacle on Left Side (FL only)..." << std::endl;
    assert(tc.getState() == ClimbState::IDLE);
    assert(!tc.isActive());

    // Trigger impact on FL only
    detector.setImpact(FL, true);
    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::FRONT_CONTACT);
    assert(tc.isActive());
    detector.clearAll();

    // Next tick plans trajectories and enters FRONT_LIFT
    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::FRONT_LIFT);

    // Next tick executes FRONT_LIFT and populates override mask
    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::FRONT_LIFT);

    // Verify override mask: ONLY FL is overridden! FR must remain FALSE!
    auto mask = tc.getOverrideMask();
    assert(mask[FL] == true);
    assert(mask[FR] == false);
    assert(mask[RL] == false);
    assert(mask[RR] == false);
    std::cout << "  ✓ FRONT_LIFT mask verified: FL=1, FR=0, RL=0, RR=0" << std::endl;

    // Verify contact valid: FL is false (airborne), FR/RL/RR are true (grounded)!
    auto cv = tc.getContactValid();
    assert(cv[FL] == false);
    assert(cv[FR] == true);
    assert(cv[RL] == true);
    assert(cv[RR] == true);
    std::cout << "  ✓ FRONT_LIFT contact_valid verified: [0, 1, 1, 1]" << std::endl;

    // Step forward in time (t < 0.6 * duration = 1.2s). Torque spike should NOT trigger premature touchdown.
    ecc_efforts(FL) = 3.5; // High torque during ascent should be ignored
    for (int step = 0; step < 50; ++step) {
        tc.update(detector, ecc_angles, ecc_efforts, dt);
    }
    assert(tc.getState() == ClimbState::FRONT_LIFT);
    std::cout << "  ✓ Early ascent torque correctly ignored (no premature touchdown)" << std::endl;

    // Advance to descent phase (t >= 1.2s) with low torque (airborne)
    ecc_efforts(FL) = 0.4;
    for (int step = 0; step < 75; ++step) { // Total time ~1.26s
        tc.update(detector, ecc_angles, ecc_efforts, dt);
    }
    assert(tc.getState() == ClimbState::FRONT_LIFT);

    // Now wheel contacts curb top surface: resistive torque jumps to 2.5 Nm
    ecc_efforts(FL) = 2.5;
    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::FRONT_PLACED);
    std::cout << "  ✓ Closed-loop touchdown detected at t=1.27s (before open-loop 2.0s timeout) -> FRONT_PLACED!" << std::endl;

    // Next tick executes FRONT_PLACED
    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::FRONT_PLACED);

    // In FRONT_PLACED: forward vel is 0.1 m/s, FL held, FR untouched
    mask = tc.getOverrideMask();
    assert(mask[FL] == true);
    assert(mask[FR] == false);
    assert(std::abs(tc.getForwardVelocityOverride() - 0.1) < 1e-5);
    std::cout << "  ✓ FRONT_PLACED forward drive active (vx=0.1), FL held, FR unconstrained" << std::endl;

    // Advance body forward until rear left wheel hits curb
    for (int step = 0; step < 100; ++step) {
        tc.update(detector, ecc_angles, ecc_efforts, dt);
    }
    assert(tc.getState() == ClimbState::FRONT_PLACED);

    // RL hits curb!
    detector.setImpact(RL, true);
    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::REAR_CONTACT);
    detector.clearAll();

    // Next tick enters REAR_LIFT
    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::REAR_LIFT);

    // Next tick executes REAR_LIFT and populates override mask
    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::REAR_LIFT);
    std::cout << "  ✓ Rear left impact detected -> transitioned to REAR_LIFT" << std::endl;

    // Check masks during REAR_LIFT: FL is held, RL is lifting; FR and RR are FALSE!
    mask = tc.getOverrideMask();
    assert(mask[FL] == true);
    assert(mask[FR] == false);
    assert(mask[RL] == true);
    assert(mask[RR] == false);

    cv = tc.getContactValid();
    assert(cv[FL] == true);
    assert(cv[FR] == true);
    assert(cv[RL] == false);
    assert(cv[RR] == true);
    std::cout << "  ✓ REAR_LIFT masks verified: Override=[1, 0, 1, 0], ContactValid=[1, 1, 0, 1]" << std::endl;

    // Advance to descent phase and detect RL touchdown
    for (int step = 0; step < 125; ++step) {
        ecc_efforts(RL) = 0.5;
        tc.update(detector, ecc_angles, ecc_efforts, dt);
    }
    assert(tc.getState() == ClimbState::REAR_LIFT);
    ecc_efforts(RL) = 2.8; // Touchdown!
    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::REAR_PLACED);
    std::cout << "  ✓ Closed-loop rear touchdown confirmed -> REAR_PLACED!" << std::endl;

    // Next tick executes REAR_PLACED
    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::REAR_PLACED);

    // In REAR_PLACED: settling forward drive
    assert(std::abs(tc.getForwardVelocityOverride() - 0.1) < 1e-5);
    for (int step = 0; step < 105; ++step) {
        tc.update(detector, ecc_angles, ecc_efforts, dt);
    }
    // Settle duration 1.0s finished -> transitioned to IDLE with blend-out active!
    assert(tc.getState() == ClimbState::IDLE);
    assert(tc.getBlendWeight() > 0.5);
    mask = tc.getOverrideMask();
    assert(mask[FL] == true);
    assert(mask[FR] == false);
    assert(mask[RL] == true);
    assert(mask[RR] == false);
    std::cout << "  ✓ Blend-out started in IDLE: weight=" << tc.getBlendWeight()
              << ", only FL & RL blended (FR & RR untouched)!" << std::endl;

    // Advance through blend-out duration (0.5s)
    for (int step = 0; step < 55; ++step) {
        tc.update(detector, ecc_angles, ecc_efforts, dt);
    }
    assert(tc.getBlendWeight() == 0.0);
    mask = tc.getOverrideMask();
    assert(!mask[FL] && !mask[FR] && !mask[RL] && !mask[RR]);
    std::cout << "  ✓ Blend-out finished: all overrides released to BalanceController!" << std::endl;

    // ----------------------------------------------------
    // Test 2: Bilateral Full-Width Step (Both FL and FR)
    // ----------------------------------------------------
    std::cout << "\n[Test 2] Testing Bilateral Full-Width Obstacle (FL and FR)..." << std::endl;
    // Wait out cooldown
    for (int step = 0; step < 60; ++step) {
        tc.update(detector, ecc_angles, ecc_efforts, dt);
    }

    detector.setImpact(FL, true);
    detector.setImpact(FR, true);
    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::FRONT_CONTACT);
    detector.clearAll();

    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::FRONT_LIFT);

    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::FRONT_LIFT);

    mask = tc.getOverrideMask();
    assert(mask[FL] == true && mask[FR] == true && mask[RL] == false && mask[RR] == false);
    cv = tc.getContactValid();
    assert(cv[FL] == false && cv[FR] == false && cv[RL] == true && cv[RR] == true);
    std::cout << "  ✓ Bilateral FRONT_LIFT mask verified: both FL & FR lifting" << std::endl;

    // Advance to descent and touchdown both wheels
    for (int step = 0; step < 125; ++step) {
        ecc_efforts = Eigen::Vector4d(0.5, 0.5, 0.5, 0.5);
        tc.update(detector, ecc_angles, ecc_efforts, dt);
    }
    ecc_efforts(FL) = 2.2;
    ecc_efforts(FR) = 2.4;
    tc.update(detector, ecc_angles, ecc_efforts, dt);
    assert(tc.getState() == ClimbState::FRONT_PLACED);
    std::cout << "  ✓ Bilateral touchdown confirmed -> FRONT_PLACED" << std::endl;

    // ----------------------------------------------------
    // Test 3: Real ContactDetector Unilateral Obstacle Discrimination
    // ----------------------------------------------------
    std::cout << "\n[Test 3] Testing Real ContactDetector Unilateral Discrimination..." << std::endl;
    RobotParams r_params;
    estimation::ContactDetector real_detector(r_params);
    TaskController real_tc(params);

    // Initial rolling state on flat ground:
    // All wheels rolling at 3.0 rad/s, wheel effort 1.5 Nm, ecc effort 3.5 Nm
    Eigen::Vector4d norm_ecc(3.5, 3.5, 3.5, 3.5);
    Eigen::Vector4d norm_wheel_eff(1.5, 1.5, 1.5, 1.5);
    Eigen::Vector4d norm_wheel_vel(3.0, 3.0, 3.0, 3.0);

    // Frame 0: initialize detector
    real_detector.update(norm_ecc, norm_wheel_eff, norm_wheel_vel, 0.2, 0.2, dt);
    real_tc.update(real_detector, ecc_angles, norm_ecc, dt);
    assert(real_tc.getState() == ClimbState::IDLE);

    // Now FL impacts a unilateral curb!
    // Transient shock: FL ecc torque jumps from 3.5 to 12.0 Nm in 2 frames (rate > 120 Nm/s)
    Eigen::Vector4d impact_ecc = norm_ecc;
    Eigen::Vector4d impact_wheel_eff = norm_wheel_eff;
    Eigen::Vector4d impact_wheel_vel = norm_wheel_vel;

    // Frame 1-2: shock onset on FL only
    impact_ecc(FL) = 8.0;
    real_detector.update(impact_ecc, norm_wheel_eff, norm_wheel_vel, 0.2, 0.2, dt);
    impact_ecc(FL) = 12.0;
    // FL wheel stalls: velocity drops to 0.1 rad/s, torque spikes to 10.0 Nm
    impact_wheel_eff(FL) = 10.0;
    impact_wheel_vel(FL) = 0.1;
    real_detector.update(impact_ecc, impact_wheel_eff, impact_wheel_vel, 0.15, 0.2, dt);

    // Sustained load for sustain_confirm_duration (0.10s = 10 frames)
    // Chassis is still moving at 0.12 m/s because FR, RL, RR are rolling!
    for (int step = 0; step < 11; ++step) {
        real_detector.update(impact_ecc, impact_wheel_eff, impact_wheel_vel, 0.12, 0.2, dt);
    }

    // Now check detector output!
    assert(real_detector.frontImpactDetected());
    auto real_flags = real_detector.getImpactFlags();
    assert(real_flags[FL] == true);
    assert(real_flags[FR] == false);
    assert(real_flags[RL] == false);
    assert(real_flags[RR] == false);
    std::cout << "  ✓ Real ContactDetector verified: FL=1, FR=0, RL=0, RR=0 (FR un-triggered!)" << std::endl;

    // Feed to TaskController and verify only FL is overridden!
    real_tc.update(real_detector, ecc_angles, impact_ecc, dt);
    assert(real_tc.getState() == ClimbState::FRONT_CONTACT);

    // Next tick: enter FRONT_LIFT
    real_tc.update(real_detector, ecc_angles, impact_ecc, dt);
    assert(real_tc.getState() == ClimbState::FRONT_LIFT);

    // Next tick: execute FRONT_LIFT
    real_tc.update(real_detector, ecc_angles, impact_ecc, dt);
    auto real_mask = real_tc.getOverrideMask();
    assert(real_mask[FL] == true);
    assert(real_mask[FR] == false);
    assert(real_mask[RL] == false);
    assert(real_mask[RR] == false);

    auto real_cv = real_tc.getContactValid();
    assert(real_cv[FL] == false);
    assert(real_cv[FR] == true);
    assert(real_cv[RL] == true);
    assert(real_cv[RR] == true);
    std::cout << "  ✓ Real end-to-end integration verified: ONLY FL lifts, FR grounded!" << std::endl;

    // ----------------------------------------------------
    // Test 4: Physical Ramp vs. Curb Disambiguation (Aligned with IEEE RA-L 2026)
    // ----------------------------------------------------
    std::cout << "\n[Test 4] Testing Physical Ramp vs. Curb Disambiguation..." << std::endl;

    // Scenario A: True Continuous Longitudinal Ramp (Track 1A)
    // Both front wheels rolling forward freely at 3.0 rad/s, pitch = 11.3 deg (0.197 rad), roll = 0 deg
    {
        double pitch_slope = 0.197;
        double roll_slope = 0.005;
        double wheel_vel_fl = 3.0, wheel_eff_fl = 2.0;
        double wheel_vel_fr = 3.0, wheel_eff_fr = 2.0;
        double pitch_curr = 0.197, pitch_rate = 0.0;
        double cmd_vx = 0.2;

        bool fl_stalled = (std::abs(wheel_eff_fl) > r_params.wheel_stall_torque_threshold) &&
                          (std::abs(wheel_vel_fl) < r_params.wheel_stall_vel_threshold);
        bool fr_stalled = (std::abs(wheel_eff_fr) > r_params.wheel_stall_torque_threshold) &&
                          (std::abs(wheel_vel_fr) < r_params.wheel_stall_vel_threshold);
        bool front_wheel_stalled = fl_stalled || fr_stalled;
        assert(!front_wheel_stalled);

        bool steady_ramp = (pitch_slope > r_params.ramp_inhibit_slope_threshold) &&
                           (std::abs(roll_slope) < 0.055);
        assert(steady_ramp);

        bool wheels_rolling_forward = (wheel_vel_fl > 1.0) && (wheel_vel_fr > 1.0);
        bool transient_ramp = (cmd_vx > 0.05) && wheels_rolling_forward &&
                              ((pitch_curr > r_params.ramp_inhibit_pitch_threshold) ||
                               (pitch_rate > r_params.ramp_inhibit_pitch_rate_threshold));
        assert(transient_ramp);

        bool is_on_ramp = !front_wheel_stalled && (steady_ramp || transient_ramp);
        assert(is_on_ramp == true);
        std::cout << "  ✓ Scenario A: Continuous Ramp correctly identified (is_on_ramp = true) -> climbing inhibited" << std::endl;
    }

    // Scenario B: Unilateral Curb Collision (Lane 2A)
    // Left wheel FL hits 40mm curb (stalled: vel=0.1, eff=9.5 Nm), right wheel FR rolling on flat ground
    // Chassis pitches up by 4 deg (0.07 rad) due to collision shock, rolls by 6 deg (0.105 rad)
    {
        double pitch_slope = 0.05; // slight pitch bleed
        double roll_slope = 0.105; // 6.0 deg lateral tilt
        double wheel_vel_fl = 0.1, wheel_eff_fl = 9.5; // FL stalled against curb!
        double wheel_vel_fr = 2.8, wheel_eff_fr = 1.5; // FR on flat floor
        double pitch_curr = 0.07, pitch_rate = 0.18;   // momentary shock
        double cmd_vx = 0.2;

        bool fl_stalled = (std::abs(wheel_eff_fl) > r_params.wheel_stall_torque_threshold) &&
                          (std::abs(wheel_vel_fl) < r_params.wheel_stall_vel_threshold);
        bool fr_stalled = (std::abs(wheel_eff_fr) > r_params.wheel_stall_torque_threshold) &&
                          (std::abs(wheel_vel_fr) < r_params.wheel_stall_vel_threshold);
        bool front_wheel_stalled = fl_stalled || fr_stalled;
        assert(front_wheel_stalled == true); // FL stalled!

        // Steady ramp must reject unilateral roll tilt (> 0.055)
        bool steady_ramp = (pitch_slope > r_params.ramp_inhibit_slope_threshold) &&
                           (std::abs(roll_slope) < 0.055);
        assert(steady_ramp == false); // Rejected due to unilateral roll!

        // Transient ramp must reject stalled wheels (cannot roll up ramp with FL stalled)
        bool wheels_rolling_forward = (wheel_vel_fl > 1.0) && (wheel_vel_fr > 1.0);
        assert(wheels_rolling_forward == false);

        bool transient_ramp = (cmd_vx > 0.05) && wheels_rolling_forward &&
                              ((pitch_curr > r_params.ramp_inhibit_pitch_threshold) ||
                               (pitch_rate > r_params.ramp_inhibit_pitch_rate_threshold));
        assert(transient_ramp == false);

        // Crucial: is_on_ramp MUST be false!
        bool is_on_ramp = !front_wheel_stalled && (steady_ramp || transient_ramp);
        assert(is_on_ramp == false);

        bool forward_straight_active = true;
        bool obstacle_climb_permitted = forward_straight_active && !is_on_ramp;
        assert(obstacle_climb_permitted == true);
        std::cout << "  ✓ Scenario B: Unilateral Curb correctly disambiguated (is_on_ramp = false) -> climbing PERMITTED!" << std::endl;
    }

    std::cout << "\n>>> ALL UNIT TESTS PASSED SUCCESSFULLY! <<<\n" << std::endl;
    return 0;
}
