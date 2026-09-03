#include "robot_control/kinematics/mobed_kinematics.hpp"

namespace robot_control {
namespace kinematics {

MobedKinematics::MobedKinematics(const MobedParameters& params) : params_(params) {}

Eigen::Matrix<double, 4, 3> MobedKinematics::getSteerPositions() const {
    Eigen::Matrix<double, 4, 3> steer_pos;
    steer_pos.row(FL) <<  params_.length_x,  params_.width_y, 0.0;
    steer_pos.row(FR) <<  params_.length_x, -params_.width_y, 0.0;
    steer_pos.row(RL) << -params_.length_x,  params_.width_y, 0.0;
    steer_pos.row(RR) << -params_.length_x, -params_.width_y, 0.0;
    return steer_pos;
}

std::tuple<Eigen::Vector4d, Eigen::Vector4d> MobedKinematics::computeDrivingIK(
    const Eigen::Vector3d& v_body) const {
    
    Eigen::Vector4d steer_angles;
    Eigen::Vector4d wheel_speeds;
    
    Eigen::Matrix<double, 4, 3> steer_pos = getSteerPositions();
    
    double vx = v_body(0);
    double vy = v_body(1);
    double wz = v_body(2);
    
    for (int i = 0; i < 4; ++i) {
        // v_i = v_body + omega x r_i
        // Cross product [0, 0, wz] x [rx, ry, 0] = [-wz*ry, wz*rx, 0]
        double v_ix = vx - wz * steer_pos(i, 1);
        double v_iy = vy + wz * steer_pos(i, 0);
        
        // Calculate steering angle
        steer_angles(i) = std::atan2(v_iy, v_ix);
        
        // Calculate wheel speed (velocity magnitude / wheel radius)
        double speed = std::sqrt(v_ix * v_ix + v_iy * v_iy);
        wheel_speeds(i) = speed / params_.r_wheel;
    }
    
    return {steer_angles, wheel_speeds};
}

Eigen::Vector4d MobedKinematics::computePostureIK(
    double target_height, 
    double target_roll, 
    double target_pitch,
    bool outward_config) const {
    
    Eigen::Vector4d ecc_angles;
    
    // clamp height to prevent std::acos domain errors (NaNs)
    double safe_height = std::max(params_.min_height, std::min(target_height, params_.max_height));
    
    // Construct rotation matrix from Roll (phi) and Pitch (theta)
    // R_y(pitch) * R_x(roll)
    double cp = std::cos(target_pitch);
    double sp = std::sin(target_pitch);
    double cr = std::cos(target_roll);
    double sr = std::sin(target_roll);
    
    Eigen::Matrix3d R_base_to_terrain;
    R_base_to_terrain << 
        cp,  sp*sr,  sp*cr,
         0,     cr,    -sr,
       -sp,  cp*sr,  cp*cr;
       
    Eigen::Matrix<double, 4, 3> steer_pos = getSteerPositions();
    
    for (int i = 0; i < 4; ++i) {
        // Find the Z-height of the steer joint in the Terrain frame
        Eigen::Vector3d pos_i = steer_pos.row(i).transpose();
        Eigen::Vector3d pos_terrain = R_base_to_terrain * pos_i;
        
        // The total Z drop required from the Steer Joint to the ground 
        // to maintain the target base height
        double z_required = safe_height + pos_terrain(2);
        
        // The eccentric mechanism must provide this Z drop.
        // Z_drop = L_ecc * cos(q_ecc) + R_wheel
        // We approximate that the base's local Z axis is roughly aligned with Terrain Z 
        // for the eccentric linkage calculation to keep it simple.
        double cos_q_ecc = (z_required - params_.r_wheel) / params_.l_ecc;
        
        // Clamp to [-1, 1] to avoid NaN
        cos_q_ecc = std::max(-1.0, std::min(1.0, cos_q_ecc));
        
        double q_ecc = std::acos(cos_q_ecc);
        
        if (!outward_config) {
            q_ecc = -q_ecc; // inward configuration
        }
        
        ecc_angles(i) = q_ecc;
    }
    
    return ecc_angles;
}

} // namespace kinematics
} // namespace robot_control
