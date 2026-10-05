#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>
#include <Eigen/Dense>

#include "robot_control/controllers/driving_controller.hpp"
#include "robot_control/controllers/balance_controller.hpp"
#include "robot_control/kinematics/mobed_kinematics.hpp"
#include "robot_control/core/robot_params.hpp"

using namespace robot_control;
using namespace robot_control::controllers;
using namespace robot_control::kinematics;

int main() {
    std::cout << "====================================================" << std::endl;
    std::cout << "  MobED Bank Angle & Attitude Control Unit Tests    " << std::endl;
    std::cout << "  (Aligned with IEEE RA-L 2026 Paper Section III.D) " << std::endl;
    std::cout << "====================================================" << std::endl;

    auto kinematics = std::make_shared<MobedKinematics>();

    DrivingControllerParams driving_params;
    driving_params.max_bank_angle = 0.20; // 0.20 rad (~11.5 deg)
    driving_params.bank_angle_filter = 0.05;

    DrivingController driving_controller(driving_params, kinematics);

    // ================================================================
    // Test 1: Forward Left Turn -> Inward Lean to the Left (Roll < 0)
    // ================================================================
    std::cout << "\n[Test 1] Forward Left Turn (vx = 1.0 m/s, wz = +0.8 rad/s)..." << std::endl;
    {
        Eigen::Vector3d cmd_vel(1.0, 0.0, 0.8);
        double bank = driving_controller.computeBankAngle(cmd_vel);

        double alpha_c = 1.0 * 0.8;
        double expected_bank = -std::atan2(alpha_c, 9.81);

        std::cout << "  Measured Bank Angle : " << bank << " rad (" << bank * 180.0 / M_PI << " deg)" << std::endl;
        std::cout << "  Expected Bank Angle : " << expected_bank << " rad (" << expected_bank * 180.0 / M_PI << " deg)" << std::endl;

        // Must be negative roll: left side drops, right side rises (leaning left/inward into turn)
        assert(bank < 0.0);
        assert(std::abs(bank - expected_bank) < 1e-4);
        std::cout << "  ✓ PASS: Left turn produces negative roll (inward bank to the left)!" << std::endl;
    }

    // ================================================================
    // Test 2: Forward Right Turn -> Inward Lean to the Right (Roll > 0)
    // ================================================================
    std::cout << "\n[Test 2] Forward Right Turn (vx = 1.0 m/s, wz = -0.8 rad/s)..." << std::endl;
    {
        Eigen::Vector3d cmd_vel(1.0, 0.0, -0.8);
        double bank = driving_controller.computeBankAngle(cmd_vel);

        double alpha_c = 1.0 * (-0.8);
        double expected_bank = -std::atan2(alpha_c, 9.81);

        std::cout << "  Measured Bank Angle : " << bank << " rad (" << bank * 180.0 / M_PI << " deg)" << std::endl;
        std::cout << "  Expected Bank Angle : " << expected_bank << " rad (" << expected_bank * 180.0 / M_PI << " deg)" << std::endl;

        // Must be positive roll: left side rises, right side drops (leaning right/inward into turn)
        assert(bank > 0.0);
        assert(std::abs(bank - expected_bank) < 1e-4);
        std::cout << "  ✓ PASS: Right turn produces positive roll (inward bank to the right)!" << std::endl;
    }

    // ================================================================
    // Test 3: Reverse Left Turn -> Inward Lean towards Turn Center (Roll > 0)
    // ================================================================
    std::cout << "\n[Test 3] Reverse Left Turn (vx = -1.0 m/s, wz = +0.8 rad/s)..." << std::endl;
    {
        Eigen::Vector3d cmd_vel(-1.0, 0.0, 0.8);
        double bank = driving_controller.computeBankAngle(cmd_vel);

        // In reverse with wz > 0, ICR is on the right (-Y). Leaning into turn means leaning right (bank > 0).
        double alpha_c = (-1.0) * 0.8;
        double expected_bank = -std::atan2(alpha_c, 9.81);

        std::cout << "  Measured Bank Angle : " << bank << " rad" << std::endl;
        assert(bank > 0.0);
        assert(std::abs(bank - expected_bank) < 1e-4);
        std::cout << "  ✓ PASS: Reverse turn correctly banks towards instantaneous rotation center!" << std::endl;
    }

    // ================================================================
    // Test 4: In-Place Spin (Zero Forward Velocity) -> Zero Bank Angle
    // ================================================================
    std::cout << "\n[Test 4] In-Place Spin (vx = 0.0 m/s, wz = 1.5 rad/s)..." << std::endl;
    {
        Eigen::Vector3d cmd_vel(0.0, 0.0, 1.5);
        double bank = driving_controller.computeBankAngle(cmd_vel);
        assert(std::abs(bank) < 1e-9);
        std::cout << "  ✓ PASS: In-place yaw spin produces zero bank angle!" << std::endl;
    }

    // ================================================================
    // Test 5: Pure Straight Driving -> Zero Bank Angle
    // ================================================================
    std::cout << "\n[Test 5] Pure Straight Driving (vx = 2.0 m/s, wz = 0.0 rad/s)..." << std::endl;
    {
        Eigen::Vector3d cmd_vel(2.0, 0.0, 0.0);
        double bank = driving_controller.computeBankAngle(cmd_vel);
        assert(std::abs(bank) < 1e-9);
        std::cout << "  ✓ PASS: Straight line motion produces zero bank angle!" << std::endl;
    }

    // ================================================================
    // Test 6: Max Bank Angle Clamping & Kinematic Stroke Protection
    // ================================================================
    std::cout << "\n[Test 6] Extreme Turn Saturation Clamping..." << std::endl;
    {
        Eigen::Vector3d cmd_vel(4.0, 0.0, 2.0); // alpha_c = 8.0 m/s^2 -> raw bank = -0.684 rad
        double bank = driving_controller.computeBankAngle(cmd_vel);
        assert(std::abs(bank - (-driving_params.max_bank_angle)) < 1e-5);
        std::cout << "  Measured Clamped Bank: " << bank << " rad == -" << driving_params.max_bank_angle << " rad" << std::endl;
        std::cout << "  ✓ PASS: Saturation successfully clamps to max_bank_angle!" << std::endl;
    }

    // ================================================================
    // Test 7: Low-Pass Filter Dynamic Smoothing in Update Loop
    // ================================================================
    std::cout << "\n[Test 7] Dynamic Filter Smoothing (Step Response)..." << std::endl;
    {
        driving_controller.reset();
        Eigen::Vector3d step_cmd(1.0, 0.0, 0.8);
        Eigen::Vector4d curr_steer = Eigen::Vector4d::Zero();
        Eigen::Vector4d curr_ecc = Eigen::Vector4d::Zero();

        double dt = 0.02; // 50 Hz
        double prev_filtered = 0.0;

        for (int i = 0; i < 50; ++i) {
            driving_controller.update(step_cmd, curr_steer, curr_ecc, dt);
            double curr_filtered = driving_controller.getBankAngle();
            assert(curr_filtered <= prev_filtered + 1e-9); // smoothly decreasing (becoming more negative)
            prev_filtered = curr_filtered;
        }

        double expected_steady = -std::atan2(0.8, 9.81);
        std::cout << "  Bank Angle after 1.0s: " << driving_controller.getBankAngle()
                  << " rad (Steady-state target: " << expected_steady << " rad)" << std::endl;
        assert(std::abs(driving_controller.getBankAngle() - expected_steady) < 0.01);
        std::cout << "  ✓ PASS: Bank angle transitions smoothly without discontinuities!" << std::endl;
    }

    // ================================================================
    // Test 8: Posture IK Kinematics Integration (Left Leg Crouching)
    // ================================================================
    std::cout << "\n[Test 8] Posture IK Response to Inward Bank Angle..." << std::endl;
    {
        double target_height = 0.18;
        double bank_roll = -0.15; // Left inward bank (~ -8.6 deg)
        double target_pitch = 0.0;
        Eigen::Vector4d steer_neutral = Eigen::Vector4d::Zero();
        Eigen::Matrix3d R_T = Eigen::Matrix3d::Identity();

        Eigen::Vector4d ecc_target = kinematics->computePostureIK(
            target_height, bank_roll, target_pitch, steer_neutral, R_T);

        std::cout << "  IK Output [FL, FR, RL, RR]: ["
                  << ecc_target(FL) << ", " << ecc_target(FR) << ", "
                  << ecc_target(RL) << ", " << ecc_target(RR) << "] rad" << std::endl;

        // In outward configuration, positive angle for front legs:
        // Larger q_ecc means deeper crouch (shorter leg, lowers chassis).
        // Smaller q_ecc means straighter leg (longer leg, raises chassis).
        // For left turn (bank_roll < 0), FL (left) must be more crouched than FR (right):
        assert(ecc_target(FL) > ecc_target(FR));

        // For rear legs (negative angle in outward config):
        // More negative q_ecc means deeper crouch (shorter leg, lowers chassis).
        // Less negative q_ecc means straighter leg (longer leg, raises chassis).
        // For left turn, RL (left) must be more negative than RR (right):
        assert(ecc_target(RL) < ecc_target(RR));

        std::cout << "  FL crouch angle: " << ecc_target(FL) << " > FR angle: " << ecc_target(FR) << std::endl;
        std::cout << "  RL crouch angle: " << ecc_target(RL) << " < RR angle: " << ecc_target(RR) << std::endl;
        std::cout << "  ✓ PASS: Left wheels crouch and right wheels extend -> chassis tilts left (inward)!" << std::endl;
    }

    // ================================================================
    // Test 9: Adaptive Nominal Height Budgeting for Bank Angle
    // ================================================================
    std::cout << "\n[Test 9] Adaptive Nominal Height Stroke Budgeting..." << std::endl;
    {
        BalanceControllerParams balance_params;
        BalanceController balance_controller(balance_params, kinematics);

        // At nominal 0.18m with flat slope and 0 bank:
        double h_flat = balance_controller.computeAdaptiveNominalHeight(0.18, 0.0, 0.0, 0.0);
        std::cout << "  Nominal height (flat, 0 bank): " << h_flat << " m" << std::endl;

        // With sharp bank angle (-0.20 rad):
        // Re-call over 20 steps to allow alpha_h filter to converge
        double h_banked = h_flat;
        for (int i = 0; i < 30; ++i) {
            h_banked = balance_controller.computeAdaptiveNominalHeight(0.18, 0.0, 0.0, -0.20);
        }
        std::cout << "  Nominal height (banked -0.20 rad): " << h_banked << " m" << std::endl;

        // Height should be slightly lowered to guarantee outside leg does not saturate against ceiling
        assert(h_banked < h_flat);
        assert(h_banked >= 0.15); // stays within comfortable bounds
        std::cout << "  ✓ PASS: Chassis automatically lowers CoG and preserves stroke margin during sharp banking!" << std::endl;
    }

    std::cout << "\n====================================================" << std::endl;
    std::cout << ">>> ALL 9 BANK ANGLE TESTS PASSED SUCCESSFULLY! <<<" << std::endl;
    std::cout << "====================================================" << std::endl;

    return 0;
}
