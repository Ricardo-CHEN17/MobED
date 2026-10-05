#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>
#include <Eigen/Dense>

#include "robot_control/estimation/state_estimator.hpp"
#include "robot_control/dynamics/grf_optimizer.hpp"
#include "robot_control/estimation/terrain_estimator.hpp"
#include "robot_control/core/robot_params.hpp"
#include "robot_control/core/mobed_types.hpp"

using namespace robot_control;
using namespace robot_control::estimation;
using namespace robot_control::dynamics;

int main() {
    std::cout << "====================================================" << std::endl;
    std::cout << "  MobED Items 2, 3, 4 Verification Suite            " << std::endl;
    std::cout << "  (Aligned with IEEE RA-L 2026 Paper Core Design)   " << std::endl;
    std::cout << "====================================================" << std::endl;

    RobotParams params;

    // ================================================================
    // Test Item 2: IMU Specific Force & EKF 3D Dynamics
    // ================================================================
    std::cout << "\n[TEST Item 2] Testing IMU Specific Force & EKF 3D Velocity Update..." << std::endl;
    {
        StateEstimator estimator(params);

        ImuData imu;
        imu.orientation = Eigen::Quaterniond::Identity();
        imu.angular_velocity = Eigen::Vector3d::Zero();
        // Physical accelerometer at rest measures specific force = +g upwards
        imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
        imu.has_orientation = true;

        // Step predict 100 times (1 second at dt = 0.01)
        for (int i = 0; i < 100; ++i) {
            estimator.predict(imu, 0.01);
        }

        BodyState state = estimator.getState();
        std::cout << "  After 100 predict steps at rest:" << std::endl;
        std::cout << "    pos_z: " << state.position.z() << " m" << std::endl;
        std::cout << "    vel_z: " << state.linear_velocity.z() << " m/s" << std::endl;

        // Must NOT drift into freefall!
        assert(std::abs(state.position.z()) < 1e-4);
        assert(std::abs(state.linear_velocity.z()) < 1e-4);
        std::cout << "  ✓ PASS: Zero vertical drift when feeding physical specific force [0, 0, 9.81]!" << std::endl;

        // Test 3D velocity correction with odometry
        // Simulate upward chassis motion during stance push (e.g. vz = +0.1 m/s)
        Eigen::Vector4d wheel_vels = Eigen::Vector4d::Zero();
        Eigen::Vector4d steer_angles = Eigen::Vector4d::Zero();
        Eigen::Vector4d ecc_angles(0.65, 0.65, -0.65, -0.65);
        Eigen::Vector4d steer_vels = Eigen::Vector4d::Zero();
        // Negative dq_ecc pushes down -> chassis moves up (vz > 0)
        Eigen::Vector4d ecc_vels(-1.0, -1.0, 1.0, 1.0);

        estimator.correctWithOdometry(wheel_vels, steer_angles, ecc_angles, steer_vels, ecc_vels, Eigen::Vector3d::Zero());
        BodyState updated_state = estimator.getState();
        std::cout << "  After correctWithOdometry with vertical leg motion:" << std::endl;
        std::cout << "    vel_z: " << updated_state.linear_velocity.z() << " m/s" << std::endl;

        // Velocity innovation on Z must be actively incorporated (non-zero)
        assert(std::abs(updated_state.linear_velocity.z()) > 1e-4);
        std::cout << "  ✓ PASS: EKF odometry correction actively updates 3D velocity (including Z)!" << std::endl;
    }

    // ================================================================
    // Test Item 3: Base-Frame Vertical Jacobian (J_z = l_ecc * sin(q_ecc))
    // ================================================================
    std::cout << "\n[TEST Item 3] Testing Base-Frame Vertical GRF Jacobian (Paper Eq 14)..." << std::endl;
    {
        GrfOptimizer optimizer(params);

        double q_ecc = 0.65; // ~37.2 deg
        double expected_Jz = params.l_ecc * std::sin(q_ecc);

        // Test at flat ground
        double Jz_flat = optimizer.computeVerticalJacobian(0, q_ecc, 0.0, 0.0, 0.0);
        assert(std::abs(Jz_flat - expected_Jz) < 1e-9);

        // Test on an 11.5 deg incline (pitch = 0.2 rad) with steering = 45 deg and roll = 5 deg
        double Jz_slope = optimizer.computeVerticalJacobian(0, q_ecc, 0.785, 0.087, 0.200);

        std::cout << "  Jz on flat ground:        " << Jz_flat << std::endl;
        std::cout << "  Jz on 11.5 deg ramp+steer:" << Jz_slope << std::endl;
        std::cout << "  Expected (l_ecc*sin(q)):  " << expected_Jz << std::endl;

        // Must be IDENTICAL: Base-frame Jacobian has zero spurious cross-coupling!
        assert(std::abs(Jz_slope - expected_Jz) < 1e-9);
        assert(std::abs(Jz_slope - Jz_flat) < 1e-9);

        // Verify torque calculation: tau = J_z * f_z
        Eigen::Vector4d grf(70.0, 70.0, 70.0, 70.0); // 70 N per wheel
        Eigen::Vector4d ecc(0.65, 0.65, -0.65, -0.65);
        Eigen::Vector4d steer = Eigen::Vector4d::Zero();

        Eigen::Vector4d tau = optimizer.grfToEccTorque(grf, ecc, steer, 0.087, 0.200);
        std::cout << "  Calculated GRF torques [FL, FR, RL, RR]: ["
                  << tau(0) << ", " << tau(1) << ", " << tau(2) << ", " << tau(3) << "] Nm" << std::endl;

        double expected_tau = expected_Jz * 70.0;
        assert(std::abs(tau(0) - expected_tau) < 1e-4);
        assert(std::abs(tau(1) - expected_tau) < 1e-4);
        std::cout << "  ✓ PASS: GRF Jacobian is strictly base-frame and free of slope/steer distortion!" << std::endl;
    }

    // ================================================================
    // Test Item 4: 3-Point Terrain Plane Estimation & Torsion Protection
    // ================================================================
    std::cout << "\n[TEST Item 4] Testing 3-Point Terrain Plane Estimation & Torsion Protection..." << std::endl;
    {
        TerrainEstimator estimator(params);

        double lx = params.length_x; // 0.15m
        double wy = params.width_y;  // 0.15m

        // Scenario 4A: True Continuous Ramp (11.3 deg incline = slope ~0.2)
        // All 4 wheels lie on the plane: z = -0.2 * x
        std::array<Eigen::Vector3d, NUM_LEGS> ramp_feet = {
            Eigen::Vector3d( lx,  wy, -0.2 * lx), // FL
            Eigen::Vector3d( lx, -wy, -0.2 * lx), // FR
            Eigen::Vector3d(-lx,  wy,  0.2 * lx), // RL
            Eigen::Vector3d(-lx, -wy,  0.2 * lx)  // RR
        };
        std::array<bool, NUM_LEGS> all_valid = {true, true, true, true};
        estimator.update(ramp_feet, Eigen::Matrix3d::Identity(), all_valid, false);

        TerrainState ramp_state = estimator.getState();
        std::cout << "  Scenario 4A (True 11.3 deg Ramp):" << std::endl;
        std::cout << "    Slope angle: " << ramp_state.slope_angle * 180.0 / M_PI << " deg" << std::endl;
        std::cout << "    Pitch slope: " << ramp_state.pitch_slope * 180.0 / M_PI << " deg" << std::endl;
        assert(std::abs(std::abs(ramp_state.pitch_slope * 180.0 / M_PI) - 11.3) < 1.0);
        std::cout << "  ✓ PASS: True 4-point macro-slope is accurately estimated!" << std::endl;

        // Scenario 4B: Flat Ground with 1 Suspended / Stepped Wheel (FL raised 5cm)
        // Wheels: FL is at +0.05m (obstacle or suspended in air). FR, RL, RR are flat at z = 0.
        std::array<Eigen::Vector3d, NUM_LEGS> bump_feet = {
            Eigen::Vector3d( lx,  wy, 0.05), // FL (raised 5cm)
            Eigen::Vector3d( lx, -wy, 0.00), // FR (flat)
            Eigen::Vector3d(-lx,  wy, 0.00), // RL (flat)
            Eigen::Vector3d(-lx, -wy, 0.00)  // RR (flat)
        };

        // Case 1: 4 wheels measured (e.g. stepping on stone), torsion = |0.05 - 0| = 50mm > 10mm
        estimator.reset();
        estimator.update(bump_feet, Eigen::Matrix3d::Identity(), all_valid, false);
        TerrainState bump_state = estimator.getState();
        std::cout << "  Scenario 4B-1 (4 Wheels with FL on 5cm stone):" << std::endl;
        std::cout << "    Estimated slope angle: " << bump_state.slope_angle * 180.0 / M_PI << " deg" << std::endl;
        // Must snap to flat ground (0.0)!
        assert(std::abs(bump_state.slope_angle) < 1e-4);
        std::cout << "  ✓ PASS: 4-wheel asymmetric obstacle correctly snapped to flat ground via torsion!" << std::endl;

        // Case 2: 3 wheels in contact (FL suspended in air: contact_valid[FL] = false!)
        // Previously, this bypassed torsion check and formed a 9.5 deg fake plane!
        std::array<bool, NUM_LEGS> three_valid = {false, true, true, true};
        estimator.reset();
        estimator.update(bump_feet, Eigen::Matrix3d::Identity(), three_valid, false);
        TerrainState suspended_state = estimator.getState();
        std::cout << "  Scenario 4B-2 (3 Wheels in contact, FL suspended in air):" << std::endl;
        std::cout << "    Estimated slope angle: " << suspended_state.slope_angle * 180.0 / M_PI << " deg" << std::endl;
        // Must snap to flat ground (0.0), NOT hallucinate a 9.5 deg slope!
        assert(std::abs(suspended_state.slope_angle) < 1e-4);
        std::cout << "  ✓ PASS: Suspended wheel 3-point plane correctly rejected via 4th-wheel distance!" << std::endl;
    }

    std::cout << "\n====================================================" << std::endl;
    std::cout << ">>> ALL ITEMS 2, 3, 4 TESTS PASSED SUCCESSFULLY! <<<" << std::endl;
    std::cout << "====================================================" << std::endl;

    return 0;
}
