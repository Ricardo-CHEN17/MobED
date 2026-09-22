#include "robot_control/estimation/state_estimator.hpp"
#include <cmath>

namespace robot_control {
namespace estimation {

StateEstimator::StateEstimator(const RobotParams& params)
    : params_(params),
      orientation_(Eigen::Quaterniond::Identity())
{
    reset();
}

void StateEstimator::reset() {
    initialized_ = false;

    x_.setZero();
    orientation_ = Eigen::Quaterniond::Identity();

    // Initialize covariance with moderate uncertainty
    P_.setIdentity();
    P_.block<3,3>(0,0) *= 0.1;   // position uncertainty
    P_.block<3,3>(3,3) *= 1.0;   // velocity uncertainty
    P_.block<3,3>(6,6) *= 0.01;  // orientation error uncertainty

    // Process noise: how much uncertainty grows per second
    Q_.setZero();
    Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * params_.ekf_process_noise_pos;
    Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * params_.ekf_process_noise_vel;
    Q_.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * params_.ekf_process_noise_orient;

    // Odometry measurement noise: [vx, vy, wz]
    R_odom_ = Eigen::Matrix3d::Identity() * params_.ekf_meas_noise_wheel_vel;
}

void StateEstimator::predict(const ImuData& imu, double dt) {
    if (dt <= 0.0 || dt > 0.5) return;  // reject invalid time steps

    if (imu.has_orientation) {
        orientation_ = imu.orientation;
        initialized_ = true;
    } else {
        if (!initialized_) {
            initialized_ = true;
        }
        // Integrate orientation using gyroscope (first-order quaternion)
        latest_omega_body_ = imu.angular_velocity;
        Eigen::Vector3d omega = imu.angular_velocity;  // body-frame angular velocity
        double omega_norm = omega.norm();

        if (omega_norm > 1e-8) {
            double half_angle = 0.5 * omega_norm * dt;
            Eigen::Vector3d axis = omega / omega_norm;
            Eigen::Quaterniond dq(
                std::cos(half_angle),
                axis.x() * std::sin(half_angle),
                axis.y() * std::sin(half_angle),
                axis.z() * std::sin(half_angle)
            );
            orientation_ = (orientation_ * dq).normalized();
        }
    }
    latest_omega_body_ = imu.angular_velocity;

    // ================================================================
    // 2. Transform accelerometer reading to world frame and remove gravity
    // ================================================================
    Eigen::Matrix3d R = orientation_.toRotationMatrix();
    Eigen::Vector3d gravity_world(0.0, 0.0, -params_.gravity);
    Eigen::Vector3d accel_world = R * imu.linear_acceleration + gravity_world;

    // ================================================================
    // 3. Propagate state: position and velocity (simple Euler integration)
    // ================================================================
    // x = [px, py, pz, vx, vy, vz, droll, dpitch, dyaw]
    // Position += velocity * dt + 0.5 * accel * dt^2
    x_.segment<3>(0) += x_.segment<3>(3) * dt + 0.5 * accel_world * dt * dt;
    // Velocity += accel * dt
    x_.segment<3>(3) += accel_world * dt;
    // Orientation error is reset to zero after each predict (error-state EKF)
    x_.segment<3>(6).setZero();

    // ================================================================
    // 4. Propagate covariance: P = F * P * F^T + Q * dt
    // ================================================================
    auto F = computeF(dt);
    P_ = F * P_ * F.transpose() + Q_ * dt;
}

void StateEstimator::correctWithOdometry(const Eigen::Vector4d& wheel_velocities,
                                         const Eigen::Vector4d& steer_angles,
                                         const Eigen::Vector4d& ecc_angles,
                                         const Eigen::Vector4d& steer_vels,
                                         const Eigen::Vector4d& ecc_vels,
                                         const Eigen::Vector3d& omega_body) {
    if (!initialized_) return;

    latest_omega_body_ = omega_body;
    Eigen::Matrix3d R = orientation_.toRotationMatrix();

    // ================================================================
    // Eq 3 from paper:
    //   p_dot = 1/4 * sum_{i=1}^4 ( p_dot_C,i - [omega]x R ^B r_i - R ^B r_dot_i )
    // In body frame (multiplying by R^T):
    //   v_body = R^T * p_dot = 1/4 * sum_{i=1}^4 ( - [omega_body]x ^B r_i - ^B r_dot_i )
    // where:
    //   ^B r_i is contact position in base frame
    //   ^B r_dot_i is contact velocity relative to base frame:
    //     ^B r_dot_i = d(^B r_i)/dt_joints + v_wheel_rolling
    // ================================================================

    Eigen::Vector3d v_body_meas = Eigen::Vector3d::Zero();

    for (int i = 0; i < 4; ++i) {
        double px = (i == 0 || i == 1) ? params_.length_x : -params_.length_x;
        double py = (i == 0 || i == 2) ? params_.width_y : -params_.width_y;
        double q_str = steer_angles(i);
        double q_ecc = ecc_angles(i);
        double dq_str = steer_vels(i);
        double dq_ecc = ecc_vels(i);
        double v_wheel = wheel_velocities(i) * params_.r_wheel;

        // 1. Contact position in base frame ^B r_i (Eq 2)
        Eigen::Vector3d B_r_i(
            px + params_.l_ecc * std::cos(q_str) * std::sin(q_ecc),
            py + params_.l_ecc * std::sin(q_str) * std::sin(q_ecc),
            -params_.posture_z_offset - params_.l_ecc * std::cos(q_ecc) - params_.r_wheel
        );

        // 2. Relative velocity ^B r_dot_i (Eq 3)
        // (a) Differential motion from steering and eccentric joints
        Eigen::Vector3d B_r_dot_joints(
            -params_.l_ecc * std::sin(q_str) * std::sin(q_ecc) * dq_str 
                + params_.l_ecc * std::cos(q_str) * std::cos(q_ecc) * dq_ecc,
             params_.l_ecc * std::cos(q_str) * std::sin(q_ecc) * dq_str 
                + params_.l_ecc * std::sin(q_str) * std::cos(q_ecc) * dq_ecc,
             params_.l_ecc * std::sin(q_ecc) * dq_ecc
        );

        // (b) Wheel rolling on ground (contact point moves backward relative to wheel axle)
        Eigen::Vector3d B_r_dot_roll(
            -v_wheel * std::cos(q_str),
            -v_wheel * std::sin(q_str),
            0.0
        );

        Eigen::Vector3d B_r_dot_i = B_r_dot_joints + B_r_dot_roll;

        // Eq 3: v_body = - omega_body x ^B r_i - ^B r_dot_i
        v_body_meas += (-omega_body.cross(B_r_i) - B_r_dot_i);
    }
    v_body_meas /= 4.0;

    // Predicted body-frame velocity from state
    Eigen::Vector3d v_world_pred = x_.segment<3>(3);
    Eigen::Vector3d v_body_pred = R.transpose() * v_world_pred;

    // Innovation (measurement residual): body-frame linear velocity only
    Eigen::Vector3d z_pred(v_body_pred.x(), v_body_pred.y(), 0.0);
    Eigen::Vector3d z_meas(v_body_meas.x(), v_body_meas.y(), 0.0);
    Eigen::Vector3d innovation = z_meas - z_pred;

    // Observation Jacobian H (3x9):
    Eigen::Matrix<double, 3, STATE_DIM> H;
    H.setZero();
    H.block<3,3>(0, 3) = R.transpose();  // ∂(v_body)/∂(v_world) = R^T

    // Kalman gain
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_odom_;
    Eigen::Matrix<double, STATE_DIM, 3> K = P_ * H.transpose() * S.inverse();

    // State correction
    x_ += K * innovation;

    // Covariance correction (Joseph form for numerical stability)
    auto I = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
    auto IKH = I - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R_odom_ * K.transpose();
}

BodyState StateEstimator::getState() const {
    BodyState state;
    state.position = x_.segment<3>(0);
    state.linear_velocity = x_.segment<3>(3);
    state.orientation = orientation_;

    // Angular velocity in world frame: omega_world = R * omega_body
    state.angular_velocity = orientation_.toRotationMatrix() * latest_omega_body_;

    return state;
}

Eigen::Matrix<double, StateEstimator::STATE_DIM, StateEstimator::STATE_DIM>
StateEstimator::computeF(double dt) const {
    // State transition Jacobian for error-state EKF
    // x_next = f(x, u)
    // F = df/dx evaluated at current state
    //
    // State: [pos(3), vel(3), orient_err(3)]
    // pos_next = pos + vel * dt
    // vel_next = vel + R * (a_imu - a_bias) * dt + g * dt  (linearized)
    // orient_err_next = 0  (reset after each step)

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> F;
    F.setIdentity();

    // ∂pos_next/∂vel = I * dt
    F.block<3,3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    return F;
}

}  // namespace estimation
}  // namespace robot_control
