#include "robot_control/kinematics/mobed_kinematics.hpp"
#include <algorithm>

namespace robot_control {
namespace kinematics {

MobedKinematics::MobedKinematics(const MobedParameters& params) : params_(params) {}

double MobedKinematics::normalize_angle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::Matrix<double, 4, 3> MobedKinematics::getSteerPositions() const {
    Eigen::Matrix<double, 4, 3> steer_pos;
    steer_pos.row(FL) <<  params_.length_x,  params_.width_y, 0.0;
    steer_pos.row(FR) <<  params_.length_x, -params_.width_y, 0.0;
    steer_pos.row(RL) << -params_.length_x,  params_.width_y, 0.0;
    steer_pos.row(RR) << -params_.length_x, -params_.width_y, 0.0;
    return steer_pos;
}

std::tuple<Eigen::Vector4d, Eigen::Vector4d> MobedKinematics::computeDrivingIK(
    const Eigen::Vector3d& v_body,
    const Eigen::Vector4d& current_steer_angles) const {
    
    Eigen::Vector4d steer_angles;
    Eigen::Vector4d wheel_speeds;
    
    Eigen::Matrix<double, 4, 3> steer_pos = getSteerPositions();
    
    double vx = v_body(0);
    double vy = v_body(1);
    double wz = v_body(2);
    
    for (int i = 0; i < 4; ++i) {
        double v_ix = vx - wz * steer_pos(i, 1);
        double v_iy = vy + wz * steer_pos(i, 0);
        
        double target_steer = std::atan2(v_iy, v_ix);
        double speed = std::sqrt(v_ix * v_ix + v_iy * v_iy) / params_.r_wheel;
        
        // Anti-jitter: if speed is extremely low, keep previous steer angle
        if (std::abs(speed) < 1e-3) {
            target_steer = current_steer_angles(i);
            speed = 0.0;
        } else {
            // Swerve Heading Optimization: Shortest path and wheel reversal
            double diff = normalize_angle(target_steer - current_steer_angles(i));
            
            if (diff > M_PI_2) {
                target_steer -= M_PI;
                speed = -speed;
            } else if (diff < -M_PI_2) {
                target_steer += M_PI;
                speed = -speed;
            }
            
            // Unwrap angle to prevent jumping between -pi and pi
            diff = normalize_angle(target_steer - current_steer_angles(i));
            target_steer = current_steer_angles(i) + diff;
        }
        
        steer_angles(i) = target_steer;
        wheel_speeds(i) = speed;
    }
    
    return {steer_angles, wheel_speeds};
}

Eigen::Vector4d MobedKinematics::computePostureIK(
    double target_height, 
    double target_roll, 
    double target_pitch,
    const Eigen::Vector4d& current_steer_angles,
    bool outward_config) const {
    
    Eigen::Vector4d ecc_angles;
    
    double safe_height = std::max(params_.min_height, std::min(target_height, params_.max_height));
    
    double cp = std::cos(target_pitch);
    double sp = std::sin(target_pitch);
    double cr = std::cos(target_roll);
    double sr = std::sin(target_roll);
    
    // Row 3 of the Base to Terrain Rotation Matrix
    double r31 = -sp;
    double r32 = cp * sr;
    double r33 = cp * cr;
    
    Eigen::Matrix<double, 4, 3> steer_pos = getSteerPositions();
    
    for (int i = 0; i < 4; ++i) {
        double px = steer_pos(i, 0);
        double py = steer_pos(i, 1);
        double q_str = current_steer_angles(i);
        
        // Formulation: A * sin(q_ecc) + B * cos(q_ecc) + C = 0
        double A = params_.l_ecc * (r31 * std::cos(q_str) + r32 * std::sin(q_str));
        double B = -params_.l_ecc * r33;
        double C = safe_height + r31 * px + r32 * py - r33 * params_.r_wheel;
        
        double R = std::sqrt(A * A + B * B);
        
        // Solve: sin(q_ecc + alpha) = -C / R
        double sin_val = -C / R;
        sin_val = std::max(-1.0, std::min(1.0, sin_val)); // Clamp for safety
        
        double alpha = std::atan2(B, A);
        
        // Two analytical solutions for the trigonometric equation
        double q_ecc_1 = std::asin(sin_val) - alpha;
        double q_ecc_2 = M_PI - std::asin(sin_val) - alpha;
        
        q_ecc_1 = normalize_angle(q_ecc_1);
        q_ecc_2 = normalize_angle(q_ecc_2);
        
        // The two solutions correspond to outward (q_ecc > 0) and inward (q_ecc < 0) configurations
        // Pick the appropriate root based on the outward_config flag
        if (outward_config) {
            ecc_angles(i) = std::max(q_ecc_1, q_ecc_2); // Outward is the positive/larger root
        } else {
            ecc_angles(i) = std::min(q_ecc_1, q_ecc_2); // Inward is the negative/smaller root
        }
    }
    
    return ecc_angles;
}

} // namespace kinematics
} // namespace robot_control
