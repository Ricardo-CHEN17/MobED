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

void StateEstimator::startStaticCalibration(int target_samples) {
    calibrating_ = true;
    is_calibrated_ = false;
    calib_target_samples_ = std::max(10, target_samples);
    calib_sample_count_ = 0;
    calib_accel_sum_.setZero();
    calib_gyro_sum_.setZero();
}

bool StateEstimator::updateStaticCalibration(const ImuData& imu) {
    if (!calibrating_) return is_calibrated_;

    calib_accel_sum_ += imu.linear_acceleration;
    calib_gyro_sum_  += imu.angular_velocity;
    calib_sample_count_++;

    if (calib_sample_count_ >= calib_target_samples_) {
        // Average measurements over stationary period
        Eigen::Vector3d avg_acc = calib_accel_sum_ / static_cast<double>(calib_sample_count_);
        Eigen::Vector3d avg_gyro = calib_gyro_sum_ / static_cast<double>(calib_sample_count_);

        // Gyro bias: on stationary robot, true angular velocity is 0
        static_gyro_bias_ = avg_gyro;

        // Accel bias: on flat stationary ground, body specific force is R^T * [0, 0, g]
        Eigen::Matrix3d R = imu.has_orientation ? imu.orientation.toRotationMatrix()
                                                : orientation_.toRotationMatrix();
        Eigen::Vector3d expected_gravity_body = R.transpose() * Eigen::Vector3d(0.0, 0.0, params_.gravity);
        static_accel_bias_ = avg_acc - expected_gravity_body;

        // Safety clamping for biases
        for (int k = 0; k < 3; ++k) {
            static_accel_bias_(k) = std::clamp(static_accel_bias_(k), -1.5, 1.5);
            static_gyro_bias_(k)  = std::clamp(static_gyro_bias_(k), -0.2, 0.2);
        }

        calibrating_ = false;
        is_calibrated_ = true;
        return true;
    }

    return false;
}

void StateEstimator::reset() {
    initialized_ = false;

    x_.setZero();
    orientation_ = Eigen::Quaterniond::Identity();
    prev_omega_body_.setZero();
    latest_omega_body_.setZero();
    latest_accel_body_unbiased_.setZero();

    // 9x9 covariance matrix: [pos(3), vel(3), orient_err(3)]
    P_.setIdentity();
    P_.block<3,3>(0,0) *= 0.1;   // position uncertainty
    P_.block<3,3>(3,3) *= 0.5;   // velocity uncertainty
    P_.block<3,3>(6,6) *= 0.01;  // orientation error uncertainty

    // Process noise covariance
    Q_.setZero();
    Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * params_.ekf_process_noise_pos;
    Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * params_.ekf_process_noise_vel;
    Q_.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * params_.ekf_process_noise_orient;

    // Odometry measurement noise
    R_odom_ = Eigen::Matrix3d::Identity() * params_.ekf_meas_noise_wheel_vel;
}

void StateEstimator::predict(const ImuData& imu, double dt) {
    if (dt <= 0.0 || dt > 0.5) return;

    // 1. Initial attitude synchronization from first packet
    if (!initialized_) {
        if (imu.has_orientation) {
            orientation_ = imu.orientation;
        }
        initialized_ = true;
    }

    // 2. Bias-compensated body angular velocity using pre-calibrated bias
    Eigen::Vector3d omega_unbiased = imu.angular_velocity - static_gyro_bias_;
    latest_omega_body_ = omega_unbiased;

    // 3. Propagate orientation quaternion via bias-compensated gyro
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

    // 4. Compensate for IMU lever-arm offset
    Eigen::Vector3d a_body = imu.linear_acceleration;
    if (imu_lever_arm_.norm() > 1e-4 && dt > 1e-6) {
        Eigen::Vector3d alpha_body = (omega_unbiased - prev_omega_body_) / dt;
        a_body -= omega_unbiased.cross(omega_unbiased.cross(imu_lever_arm_))
                + alpha_body.cross(imu_lever_arm_);
    }
    prev_omega_body_ = omega_unbiased;

    // 5. Bias-compensated specific force
    Eigen::Vector3d a_body_unbiased = a_body - static_accel_bias_;
    latest_accel_body_unbiased_ = a_body_unbiased;

    // 6. Transform accelerometer reading to world frame and remove gravity
    Eigen::Matrix3d R = orientation_.toRotationMatrix();
    Eigen::Vector3d gravity_world(0.0, 0.0, -params_.gravity);
    Eigen::Vector3d accel_world = R * a_body_unbiased + gravity_world;

    // 7. Propagate position and velocity
    x_.segment<3>(0) += x_.segment<3>(3) * dt + 0.5 * accel_world * dt * dt;
    x_.segment<3>(3) += accel_world * dt;
    // Orientation error in error-state EKF is reset to 0 in nominal state
    x_.segment<3>(6).setZero();

    // 8. Propagate covariance: P = F * P * F^T + Q * dt
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

    Eigen::Vector3d omega_eff = omega_body - static_gyro_bias_;
    latest_omega_body_ = omega_eff;
    Eigen::Matrix3d R = orientation_.toRotationMatrix();

    // Eq 3 from paper with contact gating:
    // v_body = 1/N_contact * sum_{i in Contact} ( - [omega_eff]x ^B r_i - ^B r_dot_i )
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
        Eigen::Vector3d B_r_dot_joints(
            -params_.l_ecc * std::sin(q_str) * std::sin(q_ecc) * dq_str 
                + params_.l_ecc * std::cos(q_str) * std::cos(q_ecc) * dq_ecc,
             params_.l_ecc * std::cos(q_str) * std::sin(q_ecc) * dq_str 
                + params_.l_ecc * std::sin(q_str) * std::cos(q_ecc) * dq_ecc,
             params_.l_ecc * std::sin(q_ecc) * dq_ecc
        );

        Eigen::Vector3d B_r_dot_roll(
            -v_wheel * std::cos(q_str),
            -v_wheel * std::sin(q_str),
            0.0
        );

        Eigen::Vector3d B_r_dot_i = B_r_dot_joints + B_r_dot_roll;
        v_body_meas += (-omega_eff.cross(B_r_i) - B_r_dot_i);
        num_contact++;
    }

    if (num_contact == 0) return;  // All wheels airborne: rely purely on IMU propagation
    v_body_meas /= static_cast<double>(num_contact);

    // Predicted body-frame velocity from state
    Eigen::Vector3d v_world_pred = x_.segment<3>(3);
    Eigen::Vector3d v_body_pred = R.transpose() * v_world_pred;

    // Innovation
    Eigen::Vector3d innovation = v_body_meas - v_body_pred;

    // Observation Jacobian H (3x9):
    Eigen::Matrix<double, 3, STATE_DIM> H;
    H.setZero();
    // d(v_body)/d(v_world) = R^T
    H.block<3,3>(0, 3) = R.transpose();
    // d(v_body)/d(delta_theta) = [R^T * v_world]x
    Eigen::Vector3d v_b = R.transpose() * v_world_pred;
    Eigen::Matrix3d vb_skew;
    vb_skew <<     0.0, -v_b.z(),  v_b.y(),
               v_b.z(),     0.0, -v_b.x(),
              -v_b.y(),  v_b.x(),     0.0;
    H.block<3,3>(0, 6) = vb_skew;

    // Kalman gain
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_odom_;
    Eigen::Matrix<double, STATE_DIM, 3> K = P_ * H.transpose() * S.inverse();

    // 9-state error correction
    Eigen::Matrix<double, STATE_DIM, 1> delta_x = K * innovation;

    x_.segment<3>(0) += delta_x.segment<3>(0);
    x_.segment<3>(3) += delta_x.segment<3>(3);

    // Orientation error injection into quaternion
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
    state.angular_velocity = orientation_.toRotationMatrix() * latest_omega_body_;
    return state;
}

Eigen::Matrix<double, StateEstimator::STATE_DIM, StateEstimator::STATE_DIM>
StateEstimator::computeF(double dt) const {
    // 9x9 State transition Jacobian for error-state EKF
    // State: [pos(3), vel(3), orient_err(3)]
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> F;
    F.setIdentity();

    // dpos/dvel = I * dt
    F.block<3,3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    Eigen::Matrix3d R = orientation_.toRotationMatrix();

    // dvel/dorient_err = -R * [a_body_unbiased]x * dt
    Eigen::Vector3d a_unbiased = latest_accel_body_unbiased_;
    Eigen::Matrix3d a_skew;
    a_skew <<       0.0, -a_unbiased.z(),  a_unbiased.y(),
             a_unbiased.z(),         0.0, -a_unbiased.x(),
            -a_unbiased.y(),  a_unbiased.x(),         0.0;
    F.block<3,3>(3, 6) = -R * a_skew * dt;

    return F;
}

}  // namespace estimation
}  // namespace robot_control
