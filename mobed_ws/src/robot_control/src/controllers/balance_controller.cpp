#include "robot_control/controllers/balance_controller.hpp"
#include <algorithm>
#include <cmath>

namespace robot_control {
namespace controllers {

BalanceController::BalanceController(const BalanceControllerParams& params, 
                                     std::shared_ptr<kinematics::MobedKinematics> kinematics)
    : params_(params), kinematics_(kinematics) {
    prev_ecc_angles_.setZero();
}

Eigen::Vector4d BalanceController::update(
    double target_height,
    double target_roll,
    double target_pitch,
    const Eigen::Vector4d& current_steer_angles,
    const Eigen::Vector4d& current_ecc_angles,
    double dt,
    bool e_stop_active) {

    if (e_stop_active) {
        // E-STOP: Hold current physical state and reset initialization
        // so it smoothly recovers via the cubic trajectory when released
        initialized_ = false;
        prev_ecc_angles_ = current_ecc_angles;
        return current_ecc_angles;
    }

    // First time initialization to current hardware state
    if (!initialized_) {
        start_ecc_angles_ = current_ecc_angles;
        prev_ecc_angles_ = current_ecc_angles;
        time_since_init_ = 0.0;
        initialized_ = true;
    }

    // 1. Safety Input Clamp (Prevent impossible target commands)
    double safe_roll = std::clamp(target_roll, -params_.max_roll, params_.max_roll);
    double safe_pitch = std::clamp(target_pitch, -params_.max_pitch, params_.max_pitch);

    // 2. Math Kinematics (Exact non-linear projection)
    Eigen::Vector4d raw_ecc_angles = kinematics_->computePostureIK(
        target_height, safe_roll, safe_pitch, current_steer_angles, false);

    Eigen::Vector4d final_ecc = prev_ecc_angles_;

    for (int i = 0; i < 4; ++i) {
        // 3. Mechanical Limit Clamp
        raw_ecc_angles(i) = std::clamp(raw_ecc_angles(i), params_.min_ecc_angle, params_.max_ecc_angle);
    }

    // 4. Synchronized Smooth Startup Trajectory
    if (time_since_init_ < STARTUP_DURATION) {
        time_since_init_ += dt;
        double progress = std::min(1.0, time_since_init_ / STARTUP_DURATION);
        
        // Cubic easing: 3p^2 - 2p^3 (Smooth start and stop)
        double smooth_progress = progress * progress * (3.0 - 2.0 * progress);
        
        for (int i = 0; i < 4; ++i) {
            final_ecc(i) = start_ecc_angles_(i) + (raw_ecc_angles(i) - start_ecc_angles_(i)) * smooth_progress;
        }
    } else {
        // 5. Normal Trajectory Smoothing (Low Pass Filter) for teleop
        for (int i = 0; i < 4; ++i) {
            double smoothed = params_.filter_alpha * raw_ecc_angles(i) + 
                              (1.0 - params_.filter_alpha) * prev_ecc_angles_(i);
                              
            // Rate Limiter
            double delta = smoothed - prev_ecc_angles_(i);
            double max_delta = params_.max_ecc_vel * dt;
            delta = std::clamp(delta, -max_delta, max_delta);
            
            final_ecc(i) = prev_ecc_angles_(i) + delta;
        }
    }

    // 6. Update state and return
    prev_ecc_angles_ = final_ecc;

    return final_ecc;
}

} // namespace controllers
} // namespace robot_control
