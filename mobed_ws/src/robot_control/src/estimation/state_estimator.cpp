#include "robot_control/estimation/state_estimator.hpp"
#include <cmath>
#include <algorithm>

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
    prev_omega_body_.setZero();
    latest_omega_body_.setZero();
    latest_accel_body_unbiased_.setZero();

    // Initialize 15x15 covariance with moderate uncertainty
    P_.setIdentity();
    P_.block<3,3>(0,0)   *= 0.1;                               // position uncertainty
    P_.block<3,3>(3,3)   *= 1.0;                               // velocity uncertainty
    P_.block<3,3>(6,6)   *= 0.01;                              // orientation error uncertainty
    P_.block<3,3>(9,9)   *= params_.ekf_initial_bias_cov;      // accel bias uncertainty
    P_.block<3,3>(12,12) *= params_.ekf_initial_bias_cov;      // gyro bias uncertainty

    // Process noise: how much uncertainty grows per second (Paper Section III.C.1 Ref [25])
    Q_.setZero();
    Q_.block<3,3>(0,0)   = Eigen::Matrix3d::Identity() * params_.ekf_process_noise_pos;
    Q_.block<3,3>(3,3)   = Eigen::Matrix3d::Identity() * params_.ekf_process_noise_vel;
    Q_.block<3,3>(6,6)   = Eigen::Matrix3d::Identity() * params_.ekf_process_noise_orient;
    Q_.block<3,3>(9,9)   = Eigen::Matrix3d::Identity() * params_.ekf_process_noise_accel_bias;
    Q_.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * params_.ekf_process_noise_gyro_bias;

    // Odometry measurement noise: [vx, vy, vz]
    R_odom_ = Eigen::Matrix3d::Identity() * params_.ekf_meas_noise_wheel_vel;
}

void StateEstimator::predict(const ImuData& imu, double dt) {
    if (dt <= 0.0 || dt > 0.5) return;  // reject invalid time steps

    // Extract current bias estimates:
    Eigen::Vector3d b_a = x_.segment<3>(9);
    Eigen::Vector3d b_g = x_.segment<3>(12);

    // Bias-compensated body angular velocity
    Eigen::Vector3d omega_unbiased = imu.angular_velocity - b_g;
    latest_omega_body_ = omega_unbiased;

    if (imu.has_orientation) {
        orientation_ = imu.orientation;
        initialized_ = true;
    } else {
        if (!initialized_) {
            initialized_ = true;
        }
        // Integrate orientation using bias-compensated gyroscope (first-order quaternion)
        double omega_norm = omega_unbiased.norm();
        if (omega_norm > 1e-8) {
            double half_angle = 0.5 * omega_norm * dt;
            Eigen::Vector3d axis = omega_unbiased / omega_norm;
            Eigen::Quaterniond dq(
                std::cos(half_angle),
                axis.x() * std::sin(half_angle),
                axis.y() * std::sin(half_angle),
                axis.z() * std::sin(half_angle)
            );
            orientation_ = (orientation_ * dq).normalized();
        }
    }

    // Compensate for IMU lever-arm offset r_imu from base origin:
    // a_body = a_imu - omega x (omega x r_imu) - alpha x r_imu
    Eigen::Vector3d a_body = imu.linear_acceleration;
    if (imu_lever_arm_.norm() > 1e-4 && dt > 1e-6) {
        Eigen::Vector3d alpha_body = (omega_unbiased - prev_omega_body_) / dt;
        a_body -= omega_unbiased.cross(omega_unbiased.cross(imu_lever_arm_))
                + alpha_body.cross(imu_lever_arm_);
    }
    prev_omega_body_ = omega_unbiased;

    // Bias-compensated specific force
    Eigen::Vector3d a_body_unbiased = a_body - b_a;
    latest_accel_body_unbiased_ = a_body_unbiased;

    // ================================================================
    // 2. Transform accelerometer reading to world frame and remove gravity
    // ================================================================
    Eigen::Matrix3d R = orientation_.toRotationMatrix();
    Eigen::Vector3d gravity_world(0.0, 0.0, -params_.gravity);
    Eigen::Vector3d accel_world = R * a_body_unbiased + gravity_world;

    // ================================================================
    // 3. Propagate state: position and velocity (Euler integration)
    // ================================================================
    // x = [px, py, pz, vx, vy, vz, droll, dpitch, dyaw, bax, bay, baz, bgx, bgy, bgz]
    // Position += velocity * dt + 0.5 * accel * dt^2
    x_.segment<3>(0) += x_.segment<3>(3) * dt + 0.5 * accel_world * dt * dt;
    // Velocity += accel * dt
    x_.segment<3>(3) += accel_world * dt;
    // Orientation error reset to zero after each predict (error-state EKF)
    x_.segment<3>(6).setZero();
    // Biases remain constant in prediction step (driven by process noise Q)

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
                                         const Eigen::Vector3d& omega_body,
                                         const std::array<bool, NUM_LEGS>& contact_valid) {
    if (!initialized_) return;

    // Use bias-compensated angular velocity for odometry kinematics
    Eigen::Vector3d b_g = x_.segment<3>(12);
    Eigen::Vector3d omega_eff = omega_body - b_g;
    latest_omega_body_ = omega_eff;
    Eigen::Matrix3d R = orientation_.toRotationMatrix();

    // ================================================================
    // Eq 3 from paper with contact gating:
    //   p_dot = 1/N_contact * sum_{i in Contact} ( p_dot_C,i - [omega]x R ^B r_i - R ^B r_dot_i )
    // In body frame:
    //   v_body = 1/N_contact * sum_{i in Contact} ( - [omega_eff]x ^B r_i - ^B r_dot_i )
    // ================================================================

    Eigen::Vector3d v_body_meas = Eigen::Vector3d::Zero();
    int num_contact = 0;

    for (int i = 0; i < 4; ++i) {
        if (!contact_valid[i]) continue;

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

        // Eq 3: v_body = - omega_eff x ^B r_i - ^B r_dot_i
        v_body_meas += (-omega_eff.cross(B_r_i) - B_r_dot_i);
        num_contact++;
    }

    if (num_contact == 0) return;  // All wheels airborne: rely purely on IMU propagation
    v_body_meas /= static_cast<double>(num_contact);

    // Predicted body-frame velocity from state
    Eigen::Vector3d v_world_pred = x_.segment<3>(3);
    Eigen::Vector3d v_body_pred = R.transpose() * v_world_pred;

    // Innovation (measurement residual): full 3D body-frame linear velocity (Paper Eq 2-3)
    Eigen::Vector3d z_pred = v_body_pred;
    Eigen::Vector3d z_meas = v_body_meas;
    Eigen::Vector3d innovation = z_meas - z_pred;

    // Observation Jacobian H (3x15):
    Eigen::Matrix<double, 3, STATE_DIM> H;
    H.setZero();
    // ∂(v_body)/∂(v_world) = R^T
    H.block<3,3>(0, 3) = R.transpose();
    // ∂(v_body)/∂(delta_theta) = [R^T * v_world]x
    Eigen::Vector3d v_b = R.transpose() * v_world_pred;
    Eigen::Matrix3d vb_skew;
    vb_skew <<     0.0, -v_b.z(),  v_b.y(),
               v_b.z(),     0.0, -v_b.x(),
              -v_b.y(),  v_b.x(),     0.0;
    H.block<3,3>(0, 6) = vb_skew;

    // Kalman gain
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_odom_;
    Eigen::Matrix<double, STATE_DIM, 3> K = P_ * H.transpose() * S.inverse();

    // 15-state correction
    Eigen::Matrix<double, STATE_DIM, 1> delta_x = K * innovation;

    x_.segment<3>(0) += delta_x.segment<3>(0);
    x_.segment<3>(3) += delta_x.segment<3>(3);

    // Orientation correction (quaternion integration)
    Eigen::Vector3d dtheta = delta_x.segment<3>(6);
    if (dtheta.norm() > 1e-8) {
        double half_angle = 0.5 * dtheta.norm();
        Eigen::Vector3d axis = dtheta.normalized();
        Eigen::Quaterniond dq(
            std::cos(half_angle),
            axis.x() * std::sin(half_angle),
            axis.y() * std::sin(half_angle),
            axis.z() * std::sin(half_angle)
        );
        orientation_ = (orientation_ * dq).normalized();
    }

    // Bias corrections with safety clamping (accel: +-2.0 m/s^2, gyro: +-0.5 rad/s)
    x_.segment<3>(9) += delta_x.segment<3>(9);
    for (int k = 0; k < 3; ++k) {
        x_(9 + k) = std::clamp(x_(9 + k), -2.0, 2.0);
    }

    x_.segment<3>(12) += delta_x.segment<3>(12);
    for (int k = 0; k < 3; ++k) {
        x_(12 + k) = std::clamp(x_(12 + k), -0.5, 0.5);
    }

    // Covariance correction (Joseph form for numerical stability)
    auto I_mat = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
    auto IKH = I_mat - K * H;
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
    // 15x15 State transition Jacobian for error-state EKF (Section III.C.1 Ref [25])
    // State: [pos(3), vel(3), orient_err(3), accel_bias(3), gyro_bias(3)]
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> F;
    F.setIdentity();

    // ∂pos/∂vel = I * dt
    F.block<3,3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    Eigen::Matrix3d R = orientation_.toRotationMatrix();

    // ∂vel/∂orient_err = -R * [a_body_unbiased]x * dt
    Eigen::Vector3d a_unbiased = latest_accel_body_unbiased_;
    Eigen::Matrix3d a_skew;
    a_skew <<       0.0, -a_unbiased.z(),  a_unbiased.y(),
             a_unbiased.z(),         0.0, -a_unbiased.x(),
            -a_unbiased.y(),  a_unbiased.x(),         0.0;
    F.block<3,3>(3, 6) = -R * a_skew * dt;

    // ∂vel/∂b_a = -R * dt
    F.block<3,3>(3, 9) = -R * dt;

    // ∂orient_err/∂b_g = -I * dt
    F.block<3,3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

    return F;
}

}  // namespace estimation
}  // namespace robot_control
