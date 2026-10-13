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
    const Eigen::Vector4d& current_steer_angles,
    const Eigen::Vector4d& current_ecc_angles) const {
    
    Eigen::Vector4d target_steer_angles;
    Eigen::Vector4d target_wheel_speeds;

    double vx = v_body.x();
    double vy = v_body.y();
    double wz = v_body.z();

    for (int i = 0; i < 4; ++i) {
        double q_str = current_steer_angles(i);
        double q_ecc = current_ecc_angles(i);

        // Fixed-Point Iteration (2 steps) to eliminate kinematic loop between q_str and contact point ^B r_i.
        // Guarantees instantaneous ICR consistency during high-speed spinning (Section III.D & Eq 7)
        // Step 1: Initial estimate using seed steering angle
        Eigen::Vector3d B_r_i = computeFootPositionInBase(i, q_str, q_ecc);
        double v_ix = vx - wz * B_r_i.y();
        double v_iy = vy + wz * B_r_i.x();
        double speed = std::sqrt(v_ix * v_ix + v_iy * v_iy) / params_.r_wheel;

        if (speed < 1e-3) {
            target_steer_angles(i) = current_steer_angles(i);
            target_wheel_speeds(i) = 0.0;
            continue;
        }

        double steer_iter = std::atan2(v_iy, v_ix);

        // Step 2: Refined estimate with steer_iter
        B_r_i = computeFootPositionInBase(i, steer_iter, q_ecc);
        v_ix = vx - wz * B_r_i.y();
        v_iy = vy + wz * B_r_i.x();
        speed = std::sqrt(v_ix * v_ix + v_iy * v_iy) / params_.r_wheel;
        double final_steer = std::atan2(v_iy, v_ix);

        target_steer_angles(i) = normalize_angle(final_steer);
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
    PostureConfiguration config) const {
    
    Eigen::Vector4d target_ecc_angles;
    
    double safe_height = std::max(params_.min_height, std::min(target_height, params_.max_height));
    
    // R_B: Rotation from Base to World (Standard Tait-Bryan ZYX with zero yaw)
    Eigen::Matrix3d R_B;
    R_B = Eigen::AngleAxisd(target_pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(target_roll, Eigen::Vector3d::UnitX());
        
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
        
        // Strict sign selection & feasible workspace filtering (Section II.B & III.E.3, Eq 15)
        // Normal operating range: eccentric arm points downwards/backwards to support the chassis.
        // Expanded to 120 deg (2.094 rad) to provide full deep-crouch stroke for 10-12 deg slope leveling.
        double max_angle = 2.094; // ~120 deg
        bool sol1_feasible = std::abs(sol1) <= max_angle;
        bool sol2_feasible = std::abs(sol2) <= max_angle;

        bool is_front = (i == FL || i == FR);
        bool target_positive = true;
        if (config == PostureConfiguration::OUTWARD) {
            target_positive = is_front;
        } else if (config == PostureConfiguration::INWARD) {
            target_positive = !is_front;
        } else if (config == PostureConfiguration::SPIN) {
            // Tangential alignment for in-place spin (Section II.B & Fig 5a 2)
            target_positive = (i == FL || i == RL);
        }

        double q_ecc;
        if (target_positive) {
            // Prefer positive solution within feasible range [0, max_angle]
            if (sol1_feasible && sol1 >= 0.0) q_ecc = sol1;
            else if (sol2_feasible && sol2 >= 0.0) q_ecc = sol2;
            else if (sol1_feasible) q_ecc = sol1;
            else if (sol2_feasible) q_ecc = sol2;
            else q_ecc = (sol1 >= 0.0) ? sol1 : sol2;
        } else {
            // Prefer negative solution within feasible range [-max_angle, 0]
            if (sol1_feasible && sol1 <= 0.0) q_ecc = sol1;
            else if (sol2_feasible && sol2 <= 0.0) q_ecc = sol2;
            else if (sol1_feasible) q_ecc = sol1;
            else if (sol2_feasible) q_ecc = sol2;
            else q_ecc = (sol1 <= 0.0) ? sol1 : sol2;
        }

        target_ecc_angles(i) = normalize_angle(q_ecc);
    }
    
    return target_ecc_angles;
}

Eigen::Vector3d MobedKinematics::computeFootPositionInBase(
    int leg_index, double q_str, double q_ecc) const {
    double px = (leg_index == FL || leg_index == FR) ? params_.length_x : -params_.length_x;
    double py = (leg_index == FL || leg_index == RL) ? params_.width_y : -params_.width_y;

    // Contact point vector ^B r_i from base origin to contact point C (Section III.A & Eq 2)
    return Eigen::Vector3d(
        px + params_.l_ecc * std::cos(q_str) * std::sin(q_ecc),
        py + params_.l_ecc * std::sin(q_str) * std::sin(q_ecc),
        -params_.posture_z_offset - params_.l_ecc * std::cos(q_ecc) - params_.r_wheel
    );
}

std::pair<double, double> MobedKinematics::getForbiddenSteerSector(int leg_index, double q_ecc) const {
    // Only when eccentric angle lifts the wheel into the chassis Z-plane (q_ecc > 0.45 rad)
    // does any mechanical collision risk exist (Section III.D in paper).
    if (std::abs(q_ecc) < 0.45) {
        return {0.0, 0.0}; // No forbidden sector, full 360 deg safe!
    }

    // Expand collision forbidden sector slightly if eccentric arm is extensively rotated (crouch mode)
    double extra = (std::abs(q_ecc) > 1.0) ? 0.20 : 0.0;

    bool is_left_side = (leg_index == FL || leg_index == RL);
    if (is_left_side) {
        // Left legs (FL, RL): Inward quadrant points towards -Y (clockwise rotation)
        // Forbidden inward sector: approximately [-150 deg, -30 deg]
        return {-2.62 - extra, -0.52 + extra};
    } else {
        // Right legs (FR, RR): Inward quadrant points towards +Y (counter-clockwise rotation)
        // Forbidden inward sector: approximately [+30 deg, +150 deg]
        return {0.52 - extra, 2.62 + extra};
    }
}

} // namespace kinematics
} // namespace robot_control
