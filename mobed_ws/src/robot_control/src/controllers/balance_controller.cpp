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

    // First time initialization to current hardware state
    if (!initialized_) {
        prev_ecc_angles_ = current_ecc_angles;
        initialized_ = true;
    }

    if (e_stop_active) {
        // E-STOP: Freeze posture exactly where it currently is physically
        prev_ecc_angles_ = current_ecc_angles;
        return current_ecc_angles;
    }

    // 1. Safety Input Clamp (Prevent impossible target commands)
    double safe_roll = std::clamp(target_roll, -params_.max_roll, params_.max_roll);
    double safe_pitch = std::clamp(target_pitch, -params_.max_pitch, params_.max_pitch);

    // 2. Math Kinematics (Exact non-linear projection)
    Eigen::Vector4d raw_ecc_angles = kinematics_->computePostureIK(
        target_height, safe_roll, safe_pitch, current_steer_angles);

    Eigen::Vector4d final_ecc = prev_ecc_angles_;

    for (int i = 0; i < 4; ++i) {
        // 3. Mechanical Limit Clamp
        raw_ecc_angles(i) = std::clamp(raw_ecc_angles(i), params_.min_ecc_angle, params_.max_ecc_angle);

        // 4. Trajectory Smoothing (Low Pass Filter)
        // This prevents the chassis from jerking violently when a new pose is commanded.
        double smoothed = params_.filter_alpha * raw_ecc_angles(i) + 
                          (1.0 - params_.filter_alpha) * prev_ecc_angles_(i);
                          
        // Optional Rate Limiter check (ensure the smoothed target doesn't require exceeding max motor velocity)
        double delta = smoothed - prev_ecc_angles_(i);
        double max_delta = params_.max_ecc_vel * dt;
        delta = std::clamp(delta, -max_delta, max_delta);
        
        final_ecc(i) = prev_ecc_angles_(i) + delta;
    }

    // 5. Update state and return
    prev_ecc_angles_ = final_ecc;

    return final_ecc;
}

} // namespace controllers
} // namespace robot_control
