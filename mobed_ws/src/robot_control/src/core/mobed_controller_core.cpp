#include "robot_control/core/mobed_controller_core.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace robot_control {

MobedControllerCore::MobedControllerCore(const RobotParams& params)
    : robot_params_(params)
{
    kinematics_ = std::make_shared<kinematics::MobedKinematics>();

    controllers::DrivingControllerParams drive_params;
    controllers::BalanceControllerParams balance_params;

    driving_controller_  = std::make_unique<controllers::DrivingController>(drive_params, kinematics_);
    balance_controller_  = std::make_unique<controllers::BalanceController>(balance_params, kinematics_);
    state_estimator_     = std::make_unique<estimation::StateEstimator>(robot_params_);
    state_estimator_->startStaticCalibration(50); // fast calibration in sim
    terrain_estimator_   = std::make_unique<estimation::TerrainEstimator>(robot_params_);
    contact_detector_    = std::make_unique<estimation::ContactDetector>(robot_params_);
    climb_geometry_estimator_ = std::make_unique<estimation::ClimbGeometryEstimator>(robot_params_);
    task_controller_     = std::make_unique<tasks::TaskController>();
    task_controller_->setClimbMode(climb_mode_);
    task_controller_->setCurbHeight(0.08); // adaptive 80mm default baseline

    reset();
}

void MobedControllerCore::reset() {
    joints_initialized_ = false;
    is_homing_ = true;
    homing_timer_ = 0.0;
    strafe_outward_active_ = false;
    strafe_exit_timer_ = 0.0;
    prev_target_wheel_.setZero();
    prev_cad_ecc_.fill(0.0);

    if (state_estimator_) state_estimator_->reset();
    if (terrain_estimator_) terrain_estimator_->reset();
    if (contact_detector_) contact_detector_->reset();
    if (climb_geometry_estimator_) climb_geometry_estimator_->reset();
    if (task_controller_) task_controller_->reset();
}

void MobedControllerCore::update(const SensorInputs& in, const CommandInputs& cmd, double dt, ActuatorOutputs& out) {
    if (dt <= 0.0 || dt > 0.5) return;

    transition_smoother_.update(dt);

    // ================================================================
    // Phase 1: Unpack sensors & hardware polarities into math frame
    // ================================================================
    Eigen::Vector4d curr_steer_angles;
    Eigen::Vector4d curr_steer_velocities;
    Eigen::Vector4d curr_ecc_angles;
    Eigen::Vector4d curr_ecc_velocities;
    Eigen::Vector4d curr_ecc_efforts;
    Eigen::Vector4d curr_wheel_velocities;
    Eigen::Vector4d curr_wheel_efforts;

    for (int i = 0; i < 4; ++i) {
        // Steering
        curr_steer_angles(i)     = in.steer_pos[i] * steer_signs_[i];
        curr_steer_velocities(i) = in.steer_vel[i] * steer_signs_[i];

        // Eccentric posture
        double raw_q = (in.ecc_pos[i] - ecc_offsets_[i]) * ecc_signs_[i];
        while (raw_q > M_PI) raw_q -= 2.0 * M_PI;
        while (raw_q <= -M_PI) raw_q += 2.0 * M_PI;
        curr_ecc_angles(i)     = raw_q;
        curr_ecc_velocities(i) = in.ecc_vel[i] * ecc_signs_[i];
        curr_ecc_efforts(i)    = in.ecc_effort[i];

        // Wheels
        curr_wheel_velocities(i) = in.wheel_vel[i] * wheel_signs_[i];
        curr_wheel_efforts(i)    = in.wheel_effort[i];
    }

    if (!joints_initialized_) {
        for (int i = 0; i < 4; ++i) {
            prev_cad_ecc_[i] = in.ecc_pos[i];
        }
        joints_initialized_ = true;
    }

    // Unpack IMU
    ImuData imu;
    imu.linear_acceleration = Eigen::Vector3d(in.imu_accel[0], in.imu_accel[1], in.imu_accel[2]);
    imu.angular_velocity    = Eigen::Vector3d(in.imu_gyro[0],  in.imu_gyro[1],  in.imu_gyro[2]);
    imu.orientation         = Eigen::Quaterniond(in.imu_quat[0], in.imu_quat[1], in.imu_quat[2], in.imu_quat[3]);
    imu.has_orientation     = (in.imu_quat[0] != 0.0 || in.imu_quat[1] != 0.0 ||
                               in.imu_quat[2] != 0.0 || in.imu_quat[3] != 0.0);

    // Fast homing & preemption
    Eigen::Vector3d cmd_vel(cmd.vx, cmd.vy, cmd.wz);
    double target_height = (cmd.target_height >= 0.08 && cmd.target_height <= 0.22) ? cmd.target_height : 0.205;
    double target_roll   = std::clamp(cmd.target_roll, -0.35, 0.35);
    double target_pitch  = std::clamp(cmd.target_pitch, -0.35, 0.35);

    // Unpack user posture mode and autonomous climb mode
    user_posture_config_ = static_cast<PostureConfiguration>(cmd.posture_mode);
    climb_mode_ = static_cast<ClimbMode>(cmd.climb_mode);
    task_controller_->setClimbMode(climb_mode_);
    if (climb_mode_ == ClimbMode::DISABLED && task_controller_->isActive()) {
        task_controller_->reset();
    }

    if (is_homing_) {
        homing_timer_ += dt;
        if (!state_estimator_->isCalibrated()) {
            state_estimator_->updateStaticCalibration(imu);
        }
        bool has_teleop_input = (cmd_vel.norm() > 0.05);
        if (homing_timer_ > 0.5 || has_teleop_input) {
            is_homing_ = false;
        }
    }

    // ================================================================
    // Phase 2: State Estimation (15-State ESKF with contact gating)
    // ================================================================
    std::array<bool, NUM_LEGS> contact_valid = task_controller_->getContactValid();

    // Verify contact force: if |effort| < 1.0 Nm during motion, leg is airborne
    int supported_legs = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (contact_valid[i] && std::abs(curr_ecc_efforts(i)) >= 1.0) {
            supported_legs++;
        }
    }
    if (supported_legs >= 3) {
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (contact_valid[i] && std::abs(curr_ecc_efforts(i)) < 1.0) {
                contact_valid[i] = false;
            }
        }
    }

    state_estimator_->predict(imu, dt);
    state_estimator_->correctWithOdometry(
        curr_wheel_velocities, curr_steer_angles, curr_ecc_angles,
        curr_steer_velocities, curr_ecc_velocities, imu.angular_velocity, contact_valid);

    BodyState body_state = state_estimator_->getState();

    // ================================================================
    // Phase 2.5: Contact Points 3D & Terrain Estimation (Eq 4-6)
    // ================================================================
    std::array<Eigen::Vector3d, NUM_LEGS> foot_pos_in_base;
    std::array<Eigen::Vector3d, NUM_LEGS> contact_points_world;
    Eigen::Vector3d base_pos = body_state.position;
    Eigen::Matrix3d R_base = body_state.orientation.toRotationMatrix();

    for (int i = 0; i < NUM_LEGS; ++i) {
        foot_pos_in_base[i] = kinematics_->computeFootPositionInBase(
            i, curr_steer_angles(i), curr_ecc_angles(i));
        contact_points_world[i] = base_pos + R_base * foot_pos_in_base[i];
    }

    climb_geometry_estimator_->update(
        body_state, contact_points_world, curr_ecc_angles,
        curr_wheel_velocities, curr_wheel_efforts, prev_target_wheel_,
        imu.angular_velocity, task_controller_->getCurbHeight(), dt);

    Eigen::Vector3d active_cmd_vel = is_homing_ ? Eigen::Vector3d::Zero() : cmd_vel;
    if (task_controller_->isActive()) {
        double fsm_vx = task_controller_->getForwardVelocityOverride();
        active_cmd_vel = Eigen::Vector3d(fsm_vx, 0.0, 0.0);
    }

    bool climbing_active = task_controller_->isActive();
    if (!climbing_active) {
        bool is_stopped = (active_cmd_vel.norm() < 1e-3);
        terrain_estimator_->update(foot_pos_in_base, R_base, contact_valid, is_stopped);
    }
    TerrainState terrain_state = terrain_estimator_->getState();

    // ================================================================
    // Phase 3: Contact & Collision Detection
    // ================================================================
    Eigen::Vector3d v_body = R_base.transpose() * body_state.linear_velocity;
    double chassis_forward_vel = v_body.x();

    contact_detector_->update(
        curr_ecc_efforts, curr_wheel_efforts, curr_wheel_velocities,
        chassis_forward_vel, active_cmd_vel.x(), imu.linear_acceleration.x(),
        climb_geometry_estimator_->getGeometry(), dt);

    // ================================================================
    // Phase 4: Autonomous Obstacle Climbing State Machine (Climb FSM)
    // ================================================================
    bool e_stop = (cmd.e_stop != 0);
    if (!is_homing_ && !e_stop) {
        bool forward_intent = (cmd_vel(0) > 0.08);
        bool steer_aligned = (std::abs(curr_steer_angles(0)) < 0.60) &&
                             (std::abs(curr_steer_angles(1)) < 0.60);
        bool forward_straight_active = forward_intent && steer_aligned &&
                                       (std::abs(cmd_vel(2)) < 0.40) &&
                                       (std::abs(cmd_vel(1)) < 0.15);

        bool fl_stalled = (std::abs(curr_wheel_efforts(0)) > robot_params_.wheel_stall_torque_threshold) &&
                          (std::abs(curr_wheel_velocities(0)) < robot_params_.wheel_stall_vel_threshold);
        bool fr_stalled = (std::abs(curr_wheel_efforts(1)) > robot_params_.wheel_stall_torque_threshold) &&
                          (std::abs(curr_wheel_velocities(1)) < robot_params_.wheel_stall_vel_threshold);
        bool front_wheel_stalled = fl_stalled || fr_stalled;

        bool steady_ramp = (terrain_state.pitch_slope > robot_params_.ramp_inhibit_slope_threshold) &&
                           (std::abs(terrain_state.roll_slope) < 0.055);
        bool wheels_rolling_forward = (curr_wheel_velocities(0) > 1.0) &&
                                      (curr_wheel_velocities(1) > 1.0);
        const auto& geom = climb_geometry_estimator_->getGeometry();
        bool transient_ramp = (cmd_vel(0) > 0.05) && wheels_rolling_forward &&
                              (geom.nose_up_pitch > robot_params_.ramp_inhibit_pitch_threshold);

        bool is_on_ramp = !front_wheel_stalled && (steady_ramp || transient_ramp) && (geom.delta_height < 0.035);

        bool obstacle_climb_permitted = (climb_mode_ != ClimbMode::DISABLED) &&
                                        (forward_straight_active || geom.delta_height > 0.035) &&
                                        !is_on_ramp;

        if (!task_controller_->isActive() && cmd_vel(0) < -0.05) {
            contact_detector_->reset();
        }

        if (obstacle_climb_permitted || task_controller_->isActive()) {
            task_controller_->update(
                *contact_detector_, geom, curr_ecc_angles, curr_ecc_efforts, dt, chassis_forward_vel, cmd_vel(0));
        }
    }

    // ================================================================
    // Phase 5: Driving Controller (Swerve IK + Coherent Scaling + Bank Angle)
    // ================================================================
    auto [target_steer, target_wheel] = driving_controller_->update(
        active_cmd_vel, curr_steer_angles, curr_ecc_angles, dt,
        e_stop, is_homing_, curr_steer_velocities, curr_wheel_velocities);

    prev_target_wheel_ = target_wheel;

    // Auto-Strafe Outward Stance ("外八字") detection when pure strafing
    if (user_posture_config_ == PostureConfiguration::OUTWARD &&
        !task_controller_->isActive() && !is_homing_ && !e_stop) {

        bool strafe_intent = (std::abs(active_cmd_vel.y()) > 0.05 &&
                              std::abs(active_cmd_vel.y()) >= 0.8 * std::abs(active_cmd_vel.x()));
        bool steer_aligned_strafe = (curr_steer_angles(0) < -0.80 &&
                                     curr_steer_angles(1) >  0.80 &&
                                     curr_steer_angles(2) >  0.80 &&
                                     curr_steer_angles(3) < -0.80);

        if (strafe_intent && steer_aligned_strafe) {
            strafe_outward_active_ = true;
            strafe_exit_timer_ = 0.0;
        } else if (strafe_outward_active_) {
            bool longitudinal_intent = (std::abs(active_cmd_vel.x()) > 0.10 &&
                                        std::abs(active_cmd_vel.x()) > std::abs(active_cmd_vel.y()));
            if (longitudinal_intent) {
                strafe_outward_active_ = false;
                strafe_exit_timer_ = 0.0;
            } else if (!strafe_intent) {
                strafe_exit_timer_ += dt;
                if (strafe_exit_timer_ > 0.25) {
                    strafe_outward_active_ = false;
                }
            } else {
                strafe_exit_timer_ = 0.0;
            }
        }

        PostureConfiguration effective_config = strafe_outward_active_
            ? PostureConfiguration::STRAFE_OUTWARD
            : user_posture_config_;
        balance_controller_->setPostureConfiguration(effective_config);
    }

    // ================================================================
    // Phase 6: Balance Controller (SRBD + QP + Bank Angle overlay)
    // ================================================================
    double active_height = is_homing_ ? 0.205 : target_height;
    double active_roll   = is_homing_ ? 0.0 : target_roll;
    double active_pitch  = is_homing_ ? 0.0 : target_pitch;

    if (!is_homing_) {
        active_roll += driving_controller_->getBankAngle();
    }

    TerrainState active_terrain_state = terrain_state;
    if (is_homing_) {
        active_terrain_state.pitch_slope = 0.0;
        active_terrain_state.roll_slope  = 0.0;
        active_terrain_state.slope_angle = 0.0;
        active_terrain_state.normal      = Eigen::Vector3d::UnitZ();
        active_terrain_state.rotation    = Eigen::Matrix3d::Identity();
        active_terrain_state.is_valid    = true;
    } else if (task_controller_->isActive()) {
        active_terrain_state.pitch_slope = 0.0;
        active_terrain_state.normal = Eigen::Vector3d(0.0, -std::sin(active_terrain_state.roll_slope), std::cos(active_terrain_state.roll_slope));
        active_terrain_state.rotation = Eigen::AngleAxisd(
            active_terrain_state.roll_slope, Eigen::Vector3d::UnitX()).toRotationMatrix();
    }

    ControlOutput balance_out = balance_controller_->updateHybrid(
        active_height, active_roll, active_pitch,
        target_steer, curr_ecc_angles, curr_ecc_efforts, body_state,
        contact_points_world, contact_valid, active_terrain_state, dt, e_stop, active_cmd_vel);

    Eigen::Vector4d target_ecc = balance_out.ecc_angles;

    // ================================================================
    // Phase 7: FSM Override (Replace legs if climbing)
    // ================================================================
    Eigen::Vector4d ecc_overrides_vec = task_controller_->getEccOverrides();
    std::array<bool, NUM_LEGS> ecc_override_mask = task_controller_->getOverrideMask();
    double blend_w = task_controller_->getBlendWeight();

    if (task_controller_->isActive()) {
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (ecc_override_mask[i]) {
                target_ecc(i) = ecc_overrides_vec(i);
            }
        }
    } else if (blend_w > 0.0) {
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (ecc_override_mask[i]) {
                target_ecc(i) = blend_w * ecc_overrides_vec(i) + (1.0 - blend_w) * target_ecc(i);
            }
        }
    }

    if (transition_smoother_.in_transition) {
        double alpha = transition_smoother_.getWeight();
        for (int i = 0; i < NUM_LEGS; ++i) {
            target_ecc(i) = (1.0 - alpha) * transition_smoother_.start_ecc_angles(i) + alpha * target_ecc(i);
        }
    }

    // ================================================================
    // Phase 8: Synthesize Actuator Outputs (Shortest-Arc CAD Unwrapping)
    // ================================================================
    for (int i = 0; i < 4; ++i) {
        // Steering
        out.steer_pos[i] = target_steer(i) * steer_signs_[i];
        out.steer_effort[i] = driving_controller_->getSteerTorques()(i) * steer_signs_[i];

        // Eccentric (Shortest-arc unwrap)
        double target_cad = target_ecc(i) * ecc_signs_[i] + ecc_offsets_[i];
        double delta_cad = kinematics_->normalize_angle(target_cad - prev_cad_ecc_[i]);
        double continuous_cad = prev_cad_ecc_[i] + delta_cad;
        prev_cad_ecc_[i] = continuous_cad;
        out.ecc_pos[i] = continuous_cad;

        bool leg_is_overridden = task_controller_->isActive() && ecc_override_mask[i];
        double ff_weight = transition_smoother_.getFeedforwardWeight();
        double effort_cmd = leg_is_overridden
                          ? 0.0
                          : balance_out.ecc_torques(i) * ecc_signs_[i] * ff_weight;

        // Snap-kick downforce (-35 Nm on RL/RR)
        if (task_controller_->isActive() && task_controller_->isSnapKickActive()) {
            if (i == RL || i == RR) {
                effort_cmd = -35.0 * ecc_signs_[i];
            }
        }
        effort_cmd = std::clamp(effort_cmd, -50.0, 50.0);
        out.ecc_effort[i] = effort_cmd;

        // Wheels
        double target_wheel_vel = target_wheel(i);
        if (task_controller_->isActive()) {
            auto wheel_override_mask = task_controller_->getWheelOverrideMask();
            if (wheel_override_mask[i]) {
                target_wheel_vel = task_controller_->getWheelSpeedOverrides()(i);
            }
        }
        if (transition_smoother_.in_transition) {
            double alpha = transition_smoother_.getWeight();
            target_wheel_vel = (1.0 - alpha) * transition_smoother_.start_wheel_vels(i) + alpha * target_wheel_vel;
        }
        out.wheel_vel[i] = target_wheel_vel * wheel_signs_[i];

        // Traction torque injection
        double wheel_effort = driving_controller_->getWheelTorques()(i) * wheel_signs_[i];
        if (task_controller_->isActive()) {
            auto state = task_controller_->getState();
            if (state == tasks::ClimbState::FRONT_WALL_CLIMB || state == tasks::ClimbState::STRADDLE_LEVELING) {
                if (i == FL || i == FR) wheel_effort += 4.0 * wheel_signs_[i];
            } else if (state == tasks::ClimbState::REAR_WALL_CLIMB) {
                if (i == FL || i == FR) wheel_effort += 7.0 * wheel_signs_[i];
            } else if (state == tasks::ClimbState::REAR_VAULT_LOOP) {
                if (i == FL || i == FR) wheel_effort += 8.5 * wheel_signs_[i];
            } else if (state == tasks::ClimbState::REAR_CREEP_ADVANCE) {
                wheel_effort += 3.5 * wheel_signs_[i];
            } else if (state == tasks::ClimbState::REAR_CREST_FOLD) {
                if (task_controller_->isSnapKickActive()) {
                    if (i == FL || i == FR) wheel_effort += 7.0 * wheel_signs_[i];
                    else if (i == RL || i == RR) wheel_effort += 6.0 * wheel_signs_[i];
                } else if (task_controller_->isCoordinatedLiftActive()) {
                    wheel_effort += 5.0 * wheel_signs_[i];
                } else {
                    if (i == FL || i == FR) wheel_effort += 7.0 * wheel_signs_[i];
                }
            } else if (state == tasks::ClimbState::REAR_POST_CLIMB_RESET) {
                if (i == FL || i == FR) wheel_effort += 4.0 * wheel_signs_[i];
            }
        }
        out.wheel_effort[i] = wheel_effort;
    }

    // Telemetry & diagnostics
    out.fsm_state = static_cast<int>(task_controller_->getState());
    out.bank_angle = driving_controller_->getBankAngle();
    out.terrain_pitch = terrain_state.pitch_slope;
    out.terrain_roll = terrain_state.roll_slope;
}

} // namespace robot_control

// ----------------------------------------------------------------------------
// C-Linkage Implementations
// ----------------------------------------------------------------------------
extern "C" {

void* mobed_create() {
    return new robot_control::MobedControllerCore();
}

void mobed_destroy(void* handle) {
    if (handle) {
        delete static_cast<robot_control::MobedControllerCore*>(handle);
    }
}

void mobed_reset(void* handle) {
    if (handle) {
        static_cast<robot_control::MobedControllerCore*>(handle)->reset();
    }
}

void mobed_update(void* handle,
                  const robot_control::SensorInputs* in,
                  const robot_control::CommandInputs* cmd,
                  double dt,
                  robot_control::ActuatorOutputs* out) {
    if (handle && in && cmd && out) {
        static_cast<robot_control::MobedControllerCore*>(handle)->update(*in, *cmd, dt, *out);
    }
}

}
