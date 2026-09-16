#pragma once

#include <Eigen/Dense>
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"

namespace robot_control {
namespace estimation {

/**
 * @brief Extended Kalman Filter (EKF) for MobED body state estimation.
 *
 * Fuses IMU measurements (accelerometer + gyroscope) with wheel odometry
 * to produce a smooth, low-latency estimate of the chassis velocity and
 * orientation. Corresponds to the State Estimation block in the paper's
 * Fig. 8 control framework.
 *
 * State vector (15-dimensional):
 *   x = [position(3), velocity(3), quaternion(4→3 error), accel_bias(3), gyro_bias(3)]
 *
 * For the current implementation, we use a simplified 9-state EKF:
 *   x = [position(3), velocity(3), orientation_error(3)]
 * Bias estimation can be added in a future iteration.
 */
class StateEstimator {
public:
    explicit StateEstimator(const RobotParams& params = RobotParams());

    /**
     * @brief Run one EKF prediction step using IMU data.
     *
     * Propagates the state forward using the IMU accelerometer and gyroscope
     * measurements as control inputs. Should be called at the IMU rate
     * (typically 100–1000 Hz).
     *
     * @param imu   Raw IMU measurement in body frame
     * @param dt    Time step since last prediction (seconds)
     */
    void predict(const ImuData& imu, double dt);

    /**
     * @brief Run one EKF correction step using wheel odometry.
     *
     * Corrects the predicted state using the velocity estimate derived from
     * wheel encoder readings and the current steering/eccentric angles
     * (forward kinematics).
     *
     * @param wheel_velocity  Chassis velocity computed from wheel odometry [vx, vy, wz] in body frame
     */
    void correctWithOdometry(const Eigen::Vector3d& wheel_velocity);

    /**
     * @brief Get the current estimated body state.
     * @return BodyState with position, orientation, linear and angular velocity.
     */
    BodyState getState() const;

    /**
     * @brief Reset the estimator to initial conditions.
     */
    void reset();

    /**
     * @brief Check if the estimator has been initialized.
     */
    bool isInitialized() const { return initialized_; }

private:
    RobotParams params_;
    bool initialized_ = false;

    // ---- EKF State ----
    // State: [px, py, pz, vx, vy, vz, droll, dpitch, dyaw]
    static constexpr int STATE_DIM = 9;
    Eigen::Matrix<double, STATE_DIM, 1> x_;        // state vector
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_; // state covariance

    // ---- Process & Measurement Noise ----
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q_; // process noise covariance
    Eigen::Matrix<double, 3, 3> R_odom_;            // odometry measurement noise

    // ---- Orientation tracking (quaternion, maintained separately) ----
    Eigen::Quaterniond orientation_;

    /**
     * @brief Build the state transition Jacobian F for the prediction step.
     * @param dt Time step
     * @return 9x9 Jacobian matrix
     */
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> computeF(double dt) const;
};

}  // namespace estimation
}  // namespace robot_control
