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

    if (!initialized_) {
        // On first call, optionally use IMU orientation if available
        if (imu.has_orientation) {
            orientation_ = imu.orientation;
        }
        initialized_ = true;
    }

    // ================================================================
    // 1. Integrate orientation using gyroscope (first-order quaternion)
    // ================================================================
    Eigen::Vector3d omega = imu.angular_velocity;  // body-frame angular velocity
    double omega_norm = omega.norm();

    if (omega_norm > 1e-8) {
        // Quaternion delta from angular velocity
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
                                         const Eigen::Vector4d& ecc_angles) {
    if (!initialized_) return;

    // ================================================================
    // Eq 3 from paper: v_body = 1/4 * sum( - [omega_body]x B_r_i - B_r_dot_i )
    // Since contact point velocity in world p_dot_C = 0,
    // B_r_dot_i is the contact point velocity relative to the base frame.
    // The wheel rolls forward, so the contact point moves backward relative to the base:
    // B_r_dot_i = - v_wheel * [cos(q_steer), sin(q_steer), 0]^T
    // where v_wheel = wheel_velocity_rad_s * r_wheel.
    // ================================================================

    Eigen::Matrix3d R = orientation_.toRotationMatrix();
    // In our simplified EKF, we don't have angular velocity in the state,
    // so we assume it is zero or could use gyro. For now we just use 0.
    Eigen::Vector3d omega_body = Eigen::Vector3d::Zero();

    Eigen::Vector3d v_body_meas = Eigen::Vector3d::Zero();

    for (int i = 0; i < 4; ++i) {
        double px = (i == 0 || i == 1) ? params_.length_x : -params_.length_x;
        double py = (i == 0 || i == 2) ? params_.width_y : -params_.width_y;
        double q_str = steer_angles(i);
        double q_ecc = ecc_angles(i);
        double v_wheel = wheel_velocities(i) * params_.r_wheel;

        // Contact position in base frame B_r_i
        Eigen::Vector3d B_r_i(
            px + params_.l_ecc * std::cos(q_str) * std::sin(q_ecc),
            py + params_.l_ecc * std::sin(q_str) * std::sin(q_ecc),
            -params_.posture_z_offset - params_.l_ecc * std::cos(q_ecc) - params_.r_wheel
        );

        // Contact velocity relative to base frame B_r_dot_i
        Eigen::Vector3d B_r_dot_i(
            -v_wheel * std::cos(q_str),
            -v_wheel * std::sin(q_str),
            0.0
        );

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

    // Angular velocity from the most recent IMU reading is embedded in the
    // orientation integration. We store it in world frame for consumers.
    // Note: For a full implementation, this would be maintained as part of state.
    state.angular_velocity = Eigen::Vector3d::Zero();

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
