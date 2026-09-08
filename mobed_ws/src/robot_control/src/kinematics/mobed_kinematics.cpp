#include "robot_control/kinematics/mobed_kinematics.hpp"

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
    
    Eigen::Matrix<double, 4, 3> steer_pos = getSteerPositions();
    Eigen::Vector4d target_steer_angles;
    Eigen::Vector4d target_wheel_speeds;

    double vx = v_body.x();
    double vy = v_body.y();
    double wz = v_body.z();

    for (int i = 0; i < 4; ++i) {
        double px = steer_pos(i, 0);
        double py = steer_pos(i, 1);
        
        double v_ix = vx - wz * py;
        double v_iy = vy + wz * px;
        
        double target_steer = std::atan2(v_iy, v_ix);
        double speed = std::sqrt(v_ix * v_ix + v_iy * v_iy) / params_.r_wheel;
        
        if (std::abs(speed) < 1e-3) {
            target_steer = current_steer_angles(i);
            speed = 0.0;
        }

        double diff = normalize_angle(target_steer - current_steer_angles(i));
        
        if (std::abs(diff) > M_PI_2) {
            target_steer = normalize_angle(target_steer + M_PI);
            speed = -speed;
        }

        target_steer_angles(i) = target_steer;
        target_wheel_speeds(i) = speed;
    }

    return {target_steer_angles, target_wheel_speeds};
}

Eigen::Vector4d MobedKinematics::computePostureIK(
    double target_height, 
    double target_roll, 
    double target_pitch,
    const Eigen::Vector4d& current_steer_angles,
    bool outward_config) const {
    
    Eigen::Vector4d target_ecc_angles;
    
    double safe_height = std::max(params_.min_height, std::min(target_height, params_.max_height));
    
    double cp = std::cos(target_pitch);
    double sp = std::sin(target_pitch);
    double cr = std::cos(target_roll);
    double sr = std::sin(target_roll);
    
    double r31 = -sp;
    double r32 = sr * cp;
    double r33 = cr * cp;

    for (int i = 0; i < 4; ++i) {
        double px = (i == FL || i == FR) ? params_.length_x : -params_.length_x;
        double py = (i == FL || i == RL) ? params_.width_y : -params_.width_y;
        
        double q_str = current_steer_angles(i);
        
        double A = params_.l_ecc * (r31 * std::cos(q_str) + r32 * std::sin(q_str));
        double B = -params_.l_ecc * r33;
        // Apply physical z_offset from CAD
        double C = safe_height + r31 * px + r32 * py - r33 * params_.r_wheel - params_.posture_z_offset;
        
        double R = std::sqrt(A * A + B * B);
        double sin_val = -C / R;
        sin_val = std::clamp(sin_val, -1.0, 1.0);
        
        double alpha = std::atan2(B, A);
        
        double sol1 = std::asin(sin_val) - alpha;
        double sol2 = M_PI - std::asin(sin_val) - alpha;
        
        bool is_front = (i == FL || i == FR);
        bool choose_forward = outward_config ? is_front : !is_front;
        
        double q_ecc;
        if (std::sin(sol1) > std::sin(sol2)) {
            q_ecc = choose_forward ? sol1 : sol2;
        } else {
            q_ecc = choose_forward ? sol2 : sol1;
        }
        
        target_ecc_angles(i) = normalize_angle(q_ecc);
    }
    
    return target_ecc_angles;
}

} // namespace kinematics
} // namespace robot_control
