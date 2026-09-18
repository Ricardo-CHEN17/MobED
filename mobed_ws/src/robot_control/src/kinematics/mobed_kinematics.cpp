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

        // 1. Classic Shortest Path Optimization
        double diff = normalize_angle(target_steer - current_steer_angles(i));
        if (std::abs(diff) > M_PI_2) {
            target_steer = normalize_angle(target_steer + M_PI);
            speed = -speed;
        }

        // 2. Forbidden Zone (Self-Collision) Reversal
        double margin = 5.0 * M_PI / 180.0;
        bool is_forbidden = false;
        
        // FL: [-180, -85] 
        if (i == FL && target_steer < (-M_PI_2 + margin) && target_steer > -M_PI) is_forbidden = true;
        
        // FR: [85, 180] 
        if (i == FR && target_steer > (M_PI_2 - margin) && target_steer < M_PI) is_forbidden = true;
        
        // RL: [-95, 0] (as requested)
        if (i == RL && target_steer < 0.0 && target_steer > (-M_PI_2 - margin)) is_forbidden = true;
        
        // RR: [0, 95] (symmetric to RL)
        if (i == RR && target_steer > 0.0 && target_steer < (M_PI_2 + margin)) is_forbidden = true;

        if (is_forbidden) {
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
    const Eigen::Matrix3d& R_T,
    bool outward_config) const {
    
    Eigen::Vector4d target_ecc_angles;
    
    double safe_height = std::max(params_.min_height, std::min(target_height, params_.max_height));
    
    // R_B: Rotation from Base to World
    Eigen::Matrix3d R_B;
    R_B = Eigen::AngleAxisd(target_roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(target_pitch, Eigen::Vector3d::UnitY());
        
    // R_TB: Rotation from Base to Terrain (R_TB = R_T^T * R_B)
    Eigen::Matrix3d R_TB = R_T.transpose() * R_B;
    
    // We need the 3rd row of R_TB for the Z_terrain equation
    double rtb_31 = R_TB(2, 0);
    double rtb_32 = R_TB(2, 1);
    double rtb_33 = R_TB(2, 2);

    for (int i = 0; i < 4; ++i) {
        double px = (i == FL || i == FR) ? params_.length_x : -params_.length_x;
        double py = (i == FL || i == RL) ? params_.width_y : -params_.width_y;
        
        double q_str = current_steer_angles(i);
        
        double A = params_.l_ecc * (rtb_31 * std::cos(q_str) + rtb_32 * std::sin(q_str));
        double B = -params_.l_ecc * rtb_33;
        
        // C = target_height + (R_TB * r_BOC)_z (with ecc angle parts separated into A and B)
        // Corrected physical z_offset from CAD multiplied by rtb_33
        double C = safe_height + rtb_31 * px + rtb_32 * py - rtb_33 * params_.r_wheel - rtb_33 * params_.posture_z_offset;
        
        double R = std::sqrt(A * A + B * B);
        double sin_val = -C / R;
        sin_val = std::clamp(sin_val, -1.0, 1.0);
        
        double alpha = std::atan2(B, A);
        
        double sol1 = normalize_angle(std::asin(sin_val) - alpha);
        double sol2 = normalize_angle(M_PI - std::asin(sin_val) - alpha);
        
        // Strict sign selection as per paper: 
        // "Using the FR wheel as a reference, if the wheel is positioned outside the body, 
        // the solution q_ecc,+ (where q_ecc^d > 0) is selected."
        // Outside body means extending outward.
        // For front wheels, outward means q > 0. For rear wheels, outward means q < 0.
        bool is_front = (i == FL || i == FR);
        bool target_positive = outward_config ? is_front : !is_front;
        
        double q_ecc;
        if (target_positive) {
            // We want the solution that is > 0 (or closest to it)
            q_ecc = (sol1 >= -1e-3) ? sol1 : sol2;
            // If both are negative somehow, we just pick the maximum one
            if (sol1 < 0 && sol2 < 0) q_ecc = std::max(sol1, sol2);
        } else {
            // We want the solution that is < 0 (or closest to it)
            q_ecc = (sol1 <= 1e-3) ? sol1 : sol2;
            if (sol1 > 0 && sol2 > 0) q_ecc = std::min(sol1, sol2);
        }
        
        target_ecc_angles(i) = normalize_angle(q_ecc);
    }
    
    return target_ecc_angles;
}

} // namespace kinematics
} // namespace robot_control
