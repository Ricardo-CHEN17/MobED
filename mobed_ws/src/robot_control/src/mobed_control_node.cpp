#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_interfaces/msg/mob_ed_command.hpp"

// Core
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"

// Kinematics & Controllers
#include "robot_control/controllers/driving_controller.hpp"
#include "robot_control/controllers/balance_controller.hpp"

// Estimation
#include "robot_control/estimation/state_estimator.hpp"
#include "robot_control/estimation/terrain_estimator.hpp"
#include "robot_control/estimation/contact_detector.hpp"
#include "robot_control/estimation/climb_geometry_estimator.hpp"

// Tasks
#include "robot_control/tasks/task_controller.hpp"

using namespace std::chrono_literals;

namespace robot_control {

class MobedControlNode : public rclcpp::Node {
public:
    MobedControlNode() : Node("mobed_control_node") {
        // ============================================================
        // Initialize shared parameters and kinematics
        // ============================================================
        robot_params_ = RobotParams();
        kinematics_ = std::make_shared<kinematics::MobedKinematics>();

        // ============================================================
        // Initialize all algorithm modules
        // ============================================================
        controllers::DrivingControllerParams drive_params;
        controllers::BalanceControllerParams balance_params;

        driving_controller_  = std::make_unique<controllers::DrivingController>(drive_params, kinematics_);
        balance_controller_  = std::make_unique<controllers::BalanceController>(balance_params, kinematics_);
        state_estimator_     = std::make_unique<estimation::StateEstimator>(robot_params_);
        state_estimator_->startStaticCalibration(200);
        terrain_estimator_   = std::make_unique<estimation::TerrainEstimator>(robot_params_);
        contact_detector_    = std::make_unique<estimation::ContactDetector>(robot_params_);
        climb_geometry_estimator_ = std::make_unique<estimation::ClimbGeometryEstimator>(robot_params_);
        task_controller_     = std::make_unique<tasks::TaskController>();
        task_controller_->setClimbMode(climb_mode_);

        // ============================================================
        // Initialize state variables
        // ============================================================
        curr_steer_angles_.setZero();
        curr_steer_velocities_.setZero();
        curr_ecc_angles_.setZero();
        curr_ecc_velocities_.setZero();
        curr_ecc_efforts_.setZero();
        curr_wheel_efforts_.setZero();
        curr_wheel_velocities_.setZero();
        cmd_vel_.setZero();

        // Polarity Mapping Setup (FL, FR, RL, RR)
        steer_signs_ = {1.0, 1.0, 1.0, 1.0};
        ecc_signs_   = {1.0, -1.0, 1.0, -1.0};
        wheel_signs_ = {1.0, -1.0, 1.0, -1.0};
        ecc_offsets_ = {-1.9003, -3.0526, 1.5320, -3.0473};


        // ============================================================
        // ROS 2 Subscriptions
        // ============================================================
        sub_cmd_ = this->create_subscription<robot_interfaces::msg::MobEDCommand>(
            "/mobed/command", 10,
            std::bind(&MobedControlNode::commandCallback, this, std::placeholders::_1));

        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MobedControlNode::jointStateCallback, this, std::placeholders::_1));

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&MobedControlNode::imuCallback, this, std::placeholders::_1));

        sub_e_stop_ = this->create_subscription<std_msgs::msg::Bool>(
            "/e_stop", 10,
            std::bind(&MobedControlNode::eStopCallback, this, std::placeholders::_1));

        // Dynamic Payload Mass & Posture Configuration (IEEE RA-L 2026 Eq 11, 16, 17)
        this->declare_parameter<double>("payload_mass", 0.0);
        this->declare_parameter<int>("posture_configuration", 0);

        double init_payload = this->get_parameter("payload_mass").as_double();
        int init_config = this->get_parameter("posture_configuration").as_int();

        if (init_payload > 0.0) {
            balance_controller_->setPayloadMass(init_payload);
        }
        if (init_config >= 0 && init_config <= 2) {
            balance_controller_->setPostureConfiguration(static_cast<PostureConfiguration>(init_config));
        }

        // Adaptive Obstacle Curb Height Parameter & Subscription (0.04m ~ 0.15m)
        this->declare_parameter<double>("target_curb_height", 0.075);
        double init_curb = this->get_parameter("target_curb_height").as_double();
        task_controller_->setCurbHeight(init_curb);

        sub_curb_height_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mobed/curb_height", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                double h = std::clamp(msg->data, 0.04, 0.22);
                RCLCPP_INFO(this->get_logger(), "Adaptive curb height updated: %.3f m", h);
                task_controller_->setCurbHeight(h);
            });

        sub_payload_mass_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mobed/payload_mass", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                double mass = std::max(0.0, msg->data);
                RCLCPP_INFO(this->get_logger(), "Dynamic payload mass updated: %.2f kg", mass);
                balance_controller_->setPayloadMass(mass);
            });

        sub_posture_mode_int_ = this->create_subscription<std_msgs::msg::Int32>(
            "/mobed/posture_mode", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                if (msg->data >= 0 && msg->data <= 2) {
                    auto cfg = static_cast<PostureConfiguration>(msg->data);
                    const char* name = (cfg == PostureConfiguration::OUTWARD) ? "OUTWARD" :
                                       (cfg == PostureConfiguration::INWARD)  ? "INWARD" : "SPIN";
                    RCLCPP_INFO(this->get_logger(), "Posture configuration changed to: %s (%d)", name, msg->data);
                    balance_controller_->setPostureConfiguration(cfg);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid posture configuration integer: %d (0:OUTWARD, 1:INWARD, 2:SPIN)", msg->data);
                }
            });

        sub_posture_mode_str_ = this->create_subscription<std_msgs::msg::String>(
            "/mobed/posture_mode_str", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "OUTWARD" || msg->data == "outward") {
                    RCLCPP_INFO(this->get_logger(), "Posture configuration changed to: OUTWARD");
                    balance_controller_->setPostureConfiguration(PostureConfiguration::OUTWARD);
                } else if (msg->data == "INWARD" || msg->data == "inward") {
                    RCLCPP_INFO(this->get_logger(), "Posture configuration changed to: INWARD");
                    balance_controller_->setPostureConfiguration(PostureConfiguration::INWARD);
                } else if (msg->data == "SPIN" || msg->data == "spin") {
                    RCLCPP_INFO(this->get_logger(), "Posture configuration changed to: SPIN");
                    balance_controller_->setPostureConfiguration(PostureConfiguration::SPIN);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid posture configuration string: '%s'", msg->data.c_str());
                }
            });

        // Climb Mode Subscription: 0 = DISABLED, 1 = ADAPTIVE
        sub_climb_mode_ = this->create_subscription<std_msgs::msg::Int32>(
            "/mobed/climb_mode", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                if (msg->data == 0 || msg->data == 1) {
                    auto new_mode = static_cast<ClimbMode>(msg->data);
                    if (new_mode != climb_mode_) {
                        const char* mode_name = (new_mode == ClimbMode::DISABLED) ? "DISABLED" : "ADAPTIVE";
                        RCLCPP_INFO(this->get_logger(), "Climb mode changed to: %s (%d)", mode_name, msg->data);
                        climb_mode_ = new_mode;
                        contact_detector_->reset();
                        task_controller_->reset();
                        task_controller_->setClimbMode(climb_mode_);
                        state_estimator_->reset();
                        terrain_estimator_->reset();
                        balance_controller_->syncCurrentAngles(curr_ecc_angles_);
                        target_height_ = 0.205;
                        transition_smoother_.start(curr_ecc_angles_, curr_wheel_velocities_, 1.0, 1.0, 0.25);
                    }
                } else if (msg->data == 2) {
                    // Backward-compatibility: map legacy mode 2 directly to ADAPTIVE with 200mm target height
                    if (climb_mode_ != ClimbMode::ADAPTIVE) {
                        RCLCPP_INFO(this->get_logger(), "Climb mode set to: ADAPTIVE (via legacy mode 2 request)");
                        climb_mode_ = ClimbMode::ADAPTIVE;
                        task_controller_->setClimbMode(climb_mode_);
                    }
                    task_controller_->setCurbHeight(0.20);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid climb mode: %d (0:DISABLED, 1:ADAPTIVE)", msg->data);
                }
            });

        // ============================================================
        // ROS 2 Publishers
        // ============================================================
        pub_joint_cmds_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/mobed/joint_commands", 10);

        // ============================================================
        // Control Loop Timer (100 Hz)
        // ============================================================
        timer_ = this->create_wall_timer(10ms,
            std::bind(&MobedControlNode::timerCallback, this));
        last_time_ = this->now();
        last_cmd_vel_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "MobED Control Node started. Full pipeline: "
            "EKF + Terrain + Contact + FSM + Bank Angle + Hybrid Balance.");
    }

private:
    // ================================================================
    // Callback: Teleop/Navigation Command
    // ================================================================
    void commandCallback(const robot_interfaces::msg::MobEDCommand::SharedPtr msg) {
        cmd_vel_(0) = std::clamp(msg->twist.linear.x, -1.2, 1.2);
        cmd_vel_(1) = std::clamp(msg->twist.linear.y, -1.2, 1.2);
        cmd_vel_(2) = std::clamp(msg->twist.angular.z, -2.0, 2.0);
        last_cmd_vel_time_ = this->now();

        if (msg->body_height >= 0.08) {
            target_height_ = std::clamp(msg->body_height, 0.08, 0.22);
        }
        target_roll_   = std::clamp(msg->body_roll,  -0.35, 0.35);
        target_pitch_  = std::clamp(msg->body_pitch, -0.35, 0.35);
    }

    // ================================================================
    // Callback: IMU Data → State Estimator (EKF Predict)
    // ================================================================
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        ImuData imu;
        imu.linear_acceleration = Eigen::Vector3d(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);
        imu.angular_velocity = Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
        imu.orientation = Eigen::Quaterniond(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z);
        imu.has_orientation = (msg->orientation.w != 0.0 ||
                               msg->orientation.x != 0.0 ||
                               msg->orientation.y != 0.0 ||
                               msg->orientation.z != 0.0);

        // Cache for the control loop
        latest_imu_ = imu;
        imu_received_ = true;
    }

    // ================================================================
    // Callback: Joint States → Positions + Efforts parsing
    // ================================================================
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Skip brittle single-joint reset detection since steering wraps around naturally
        if (msg->name.size() > 0) {
            last_raw_joint_0_ = msg->position[0];
        }

        bool has_effort = !msg->effort.empty();
        bool has_vel = !msg->velocity.empty();

        for (size_t i = 0; i < msg->name.size(); ++i) {
            const auto& name = msg->name[i];
            double pos = msg->position[i];
            double effort = has_effort ? msg->effort[i] : 0.0;
            double vel = has_vel ? msg->velocity[i] : 0.0;

            // ---- Steering joints: position + velocity ----
            if (name == "Steering_joint_LF") {
                curr_steer_angles_(0) = pos * steer_signs_[0];
                curr_steer_velocities_(0) = vel * steer_signs_[0];
            } else if (name == "Steering_joint_RF") {
                curr_steer_angles_(1) = pos * steer_signs_[1];
                curr_steer_velocities_(1) = vel * steer_signs_[1];
            } else if (name == "Steering_joint_LB") {
                curr_steer_angles_(2) = pos * steer_signs_[2];
                curr_steer_velocities_(2) = vel * steer_signs_[2];
            } else if (name == "Steering_joint_RB") {
                curr_steer_angles_(3) = pos * steer_signs_[3];
                curr_steer_velocities_(3) = vel * steer_signs_[3];
            }

            // ---- Eccentric joints: position + velocity + effort ----
            else if (name == "Posture_control_joint_LF") {
                double raw_q = (pos - ecc_offsets_[0]) * ecc_signs_[0];
                while (raw_q > M_PI) raw_q -= 2.0 * M_PI;
                while (raw_q <= -M_PI) raw_q += 2.0 * M_PI;
                curr_ecc_angles_(0) = raw_q;
                curr_ecc_velocities_(0) = vel * ecc_signs_[0];
                curr_ecc_efforts_(0) = effort;
            }
            else if (name == "Posture_control_joint_RF") {
                double raw_q = (pos - ecc_offsets_[1]) * ecc_signs_[1];
                while (raw_q > M_PI) raw_q -= 2.0 * M_PI;
                while (raw_q <= -M_PI) raw_q += 2.0 * M_PI;
                curr_ecc_angles_(1) = raw_q;
                curr_ecc_velocities_(1) = vel * ecc_signs_[1];
                curr_ecc_efforts_(1) = effort;
            }
            else if (name == "Posture_control_joint_LB") {
                double raw_q = (pos - ecc_offsets_[2]) * ecc_signs_[2];
                while (raw_q > M_PI) raw_q -= 2.0 * M_PI;
                while (raw_q <= -M_PI) raw_q += 2.0 * M_PI;
                curr_ecc_angles_(2) = raw_q;
                curr_ecc_velocities_(2) = vel * ecc_signs_[2];
                curr_ecc_efforts_(2) = effort;
            }
            else if (name == "Posture_control_joint_RB") {
                double raw_q = (pos - ecc_offsets_[3]) * ecc_signs_[3];
                while (raw_q > M_PI) raw_q -= 2.0 * M_PI;
                while (raw_q <= -M_PI) raw_q += 2.0 * M_PI;
                curr_ecc_angles_(3) = raw_q;
                curr_ecc_velocities_(3) = vel * ecc_signs_[3];
                curr_ecc_efforts_(3) = effort;
            }

            // ---- Wheel joints: velocity + effort ----
            else if (name == "Wheel_joint_LF") {
                curr_wheel_velocities_(0) = vel * wheel_signs_[0];
                curr_wheel_efforts_(0) = effort;
            }
            else if (name == "Wheel_joint_RF") {
                curr_wheel_velocities_(1) = vel * wheel_signs_[1];
                curr_wheel_efforts_(1) = effort;
            }
            else if (name == "Wheel_joint_LB") {
                curr_wheel_velocities_(2) = vel * wheel_signs_[2];
                curr_wheel_efforts_(2) = effort;
            }
            else if (name == "Wheel_joint_RB") {
                curr_wheel_velocities_(3) = vel * wheel_signs_[3];
                curr_wheel_efforts_(3) = effort;
            }
        }

        if (!joints_initialized_) {
            for (int i = 0; i < 4; ++i) {
                prev_cad_ecc_[i] = curr_ecc_angles_(i) * ecc_signs_[i] + ecc_offsets_[i];
            }
            joints_initialized_ = true;
        }
    }

    // ================================================================
    // Callback: Emergency Stop
    // ================================================================
    void eStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        e_stop_active_ = msg->data;
        if (e_stop_active_) {
            RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED!");
            task_controller_->abort();
            task_controller_->reset();
            contact_detector_->reset();
            climb_geometry_estimator_->reset();
            terrain_estimator_->reset();
            state_estimator_->reset();
            for (int i = 0; i < 4; ++i) {
                prev_cad_ecc_[i] = curr_ecc_angles_(i) * ecc_signs_[i] + ecc_offsets_[i];
            }
        }
    }

    // ================================================================
    // Main Control Loop Callback
    // ================================================================
    void timerCallback() {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0 || dt > 0.5) return;
        if (!joints_initialized_) return;  // Wait for first real joint state before running any control

        // Teleop watchdog: decay cmd_vel_ to zero if no command received for > 0.5s
        if ((now - last_cmd_vel_time_).seconds() > 0.5) {
            cmd_vel_.setZero();
        }

        transition_smoother_.update(dt);

        // ============================================================
        // Phase 1: Homing Initialization
        // ============================================================
        if (is_homing_) {
            homing_timer_ += dt;
            if (imu_received_ && !state_estimator_->isCalibrated()) {
                if (state_estimator_->updateStaticCalibration(latest_imu_)) {
                    auto b_a = state_estimator_->getAccelBias();
                    auto b_g = state_estimator_->getGyroBias();
                    RCLCPP_INFO(this->get_logger(),
                        "Static IMU calibration complete! Accel bias: [%.3f, %.3f, %.3f] m/s^2, Gyro bias: [%.4f, %.4f, %.4f] rad/s",
                        b_a.x(), b_a.y(), b_a.z(), b_g.x(), b_g.y(), b_g.z());
                }
            }
            if (homing_timer_ > HOMING_DURATION) {
                is_homing_ = false;
                RCLCPP_INFO(this->get_logger(), "Homing complete. Full control pipeline active.");
            }
        }

        // ============================================================
        // Phase 2: State Estimation (EKF predict + odometry correct Eq 3 with contact gating)
        // ============================================================
        // Query FSM task controller for airborne legs during climbing
        std::array<bool, NUM_LEGS> contact_valid = task_controller_->getContactValid();

        // Real-Time Contact Force Verification (Phase 2.7):
        // If a leg has near-zero torque (|tau| < 1.0 Nm, nominal is 3.5~5 Nm) while the chassis
        // is in motion, the wheel has lost ground support (suspended in air or stepping off a bump).
        // Mask it out from contact_valid so it does not pull the terrain plane into a tilted deadlock!
        int supported_legs = 0;
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (contact_valid[i] && std::abs(curr_ecc_efforts_(i)) >= 1.0) {
                supported_legs++;
            }
        }
        if (supported_legs >= 3) {
            for (int i = 0; i < NUM_LEGS; ++i) {
                if (contact_valid[i] && std::abs(curr_ecc_efforts_(i)) < 1.0) {
                    contact_valid[i] = false;
                }
            }
        }

        if (imu_received_) {
            state_estimator_->predict(latest_imu_, dt);
        }
        Eigen::Vector3d omega_body = imu_received_ ? latest_imu_.angular_velocity : Eigen::Vector3d::Zero();
        state_estimator_->correctWithOdometry(
            curr_wheel_velocities_, curr_steer_angles_, curr_ecc_angles_,
            curr_steer_velocities_, curr_ecc_velocities_, omega_body, contact_valid);

        BodyState body_state = state_estimator_->getState();

        // ============================================================
        // Phase 2.5: Terrain Estimation (Eq 4-6 in paper)
        // ============================================================
        std::array<Eigen::Vector3d, NUM_LEGS> foot_pos_in_base;
        std::array<Eigen::Vector3d, NUM_LEGS> contact_points_world;

        Eigen::Vector3d base_pos = body_state.position;
        Eigen::Matrix3d R_base = body_state.orientation.toRotationMatrix();

        for (int i = 0; i < NUM_LEGS; ++i) {
            foot_pos_in_base[i] = kinematics_->computeFootPositionInBase(
                i, curr_steer_angles_(i), curr_ecc_angles_(i));
            // p_C = p + R * ^B r_i (Eq 2) for srbd_model
            contact_points_world[i] = base_pos + R_base * foot_pos_in_base[i];
        }

        // Update climb geometry estimator (step height, standardized pitch, support margins)
        climb_geometry_estimator_->update(
            body_state, contact_points_world,
            curr_ecc_angles_,
            curr_wheel_velocities_, curr_wheel_efforts_,
            prev_target_wheel_, omega_body,
            task_controller_->getCurbHeight(), dt);

        Eigen::Vector3d active_cmd_vel = is_homing_ ? Eigen::Vector3d::Zero() : cmd_vel_;
        if (task_controller_->isActive()) {
            double fsm_vx = task_controller_->getForwardVelocityOverride();
            active_cmd_vel = Eigen::Vector3d(fsm_vx, 0.0, 0.0);
        }

        // Run plane fitting via pseudo-inverse least squares in robot heading frame
        bool climbing_active = task_controller_->isActive();
        if (!climbing_active) {
            bool is_stopped = (active_cmd_vel.norm() < 1e-3);
            terrain_estimator_->update(foot_pos_in_base, R_base, contact_valid, is_stopped);
        }
        TerrainState terrain_state = terrain_estimator_->getState();

        // ============================================================
        // Phase 3: Contact Detection (Multi-Modal: Torque + Wheel Stall + Chassis Blocked + Wall Roll)
        // ============================================================
        Eigen::Vector3d v_body = R_base.transpose() * body_state.linear_velocity;
        double chassis_forward_vel = v_body.x();

        double imu_accel_x = imu_received_ ? latest_imu_.linear_acceleration.x() : 0.0;
        contact_detector_->update(
            curr_ecc_efforts_, curr_wheel_efforts_, curr_wheel_velocities_,
            chassis_forward_vel, active_cmd_vel.x(), imu_accel_x,
            climb_geometry_estimator_->getGeometry(), dt);

        // ============================================================
        // Phase 4: FSM Task Controller (climbing state machine)
        // ============================================================
        if (!is_homing_ && !e_stop_active_) {
            // Forward Motion & Steering Inhibit Gate (Layer 2 Defense):
            // Allow intentional forward approach (vx > 0.03) with reasonable oblique angles (|steer| < 0.85 rad ≈ 48°)
            bool forward_intent = (cmd_vel_(0) > 0.03);
            bool steer_aligned = (std::abs(curr_steer_angles_(0)) < 0.85) &&
                                 (std::abs(curr_steer_angles_(1)) < 0.85);
            bool forward_straight_active = forward_intent && steer_aligned &&
                                           (std::abs(cmd_vel_(2)) < 0.80) &&
                                           (std::abs(cmd_vel_(1)) < 0.25);

            bool fl_stalled = (std::abs(curr_wheel_efforts_(0)) > robot_params_.wheel_stall_torque_threshold) &&
                              (std::abs(curr_wheel_velocities_(0)) < robot_params_.wheel_stall_vel_threshold);
            bool fr_stalled = (std::abs(curr_wheel_efforts_(1)) > robot_params_.wheel_stall_torque_threshold) &&
                              (std::abs(curr_wheel_velocities_(1)) < robot_params_.wheel_stall_vel_threshold);
            bool front_wheel_stalled = fl_stalled || fr_stalled;

            bool steady_ramp = (terrain_state.pitch_slope > robot_params_.ramp_inhibit_slope_threshold) &&
                               (std::abs(terrain_state.roll_slope) < 0.055);

            bool wheels_rolling_forward = (curr_wheel_velocities_(0) > 1.0) &&
                                          (curr_wheel_velocities_(1) > 1.0);
            const auto& geom = climb_geometry_estimator_->getGeometry();
            bool transient_ramp = (cmd_vel_(0) > 0.05) && wheels_rolling_forward &&
                                  (geom.nose_up_pitch > robot_params_.ramp_inhibit_pitch_threshold);

            bool is_on_ramp = !front_wheel_stalled && (steady_ramp || transient_ramp);

            bool obstacle_climb_permitted = (climb_mode_ != ClimbMode::DISABLED) &&
                                            forward_straight_active && !is_on_ramp;

            // Only reset contact detector when explicitly reversing or driving backwards
            if (!task_controller_->isActive() && cmd_vel_(0) < -0.05) {
                contact_detector_->reset();
            }

            if (obstacle_climb_permitted || task_controller_->isActive()) {
                task_controller_->update(
                    *contact_detector_, geom, curr_ecc_angles_, curr_ecc_efforts_, dt, chassis_forward_vel);
            }
        }

        // Log FSM state transitions with override mask info
        static std::string prev_state_name = "IDLE";
        std::string curr_state_name = task_controller_->getStateName();
        if (curr_state_name != prev_state_name) {
            auto mask = task_controller_->getOverrideMask();
            RCLCPP_INFO(this->get_logger(), "[Adaptive FSM] Transition: %s -> %s | Overrides: [FL:%d FR:%d RL:%d RR:%d]",
                        prev_state_name.c_str(), curr_state_name.c_str(),
                        mask[FL], mask[FR], mask[RL], mask[RR]);
            prev_state_name = curr_state_name;
        }

        // ============================================================
        // Phase 5: Driving Controller (with Bank Angle & Torque Loops)
        // ============================================================
        auto [target_steer, target_wheel] = driving_controller_->update(
            active_cmd_vel, curr_steer_angles_, curr_ecc_angles_, dt,
            e_stop_active_, is_homing_, curr_steer_velocities_, curr_wheel_velocities_);

        prev_target_wheel_ = target_wheel;

        // ============================================================
        // Phase 6: Balance Controller (with Bank Angle overlay)
        // ============================================================
        Eigen::Vector4d target_ecc = Eigen::Vector4d::Zero();
        ControlOutput balance_out;

        double active_height = is_homing_ ? 0.205 : target_height_;
        double active_roll   = is_homing_ ? 0.0 : target_roll_;
        double active_pitch  = is_homing_ ? 0.0 : target_pitch_;

        // Overlay bank angle from driving controller onto target roll (only during active motion, not homing)
        if (!is_homing_) {
            active_roll += driving_controller_->getBankAngle();
        }

        // Prepare terrain state for BalanceController:
        // During homing, enforce flat horizontal terrain to prevent uncalibrated IMU drift from tilting the posture.
        // When climbing an obstacle, zero out longitudinal pitch slope compensation.
        TerrainState active_terrain_state = terrain_state;
        if (is_homing_) {
            active_terrain_state.pitch_slope = 0.0;
            active_terrain_state.roll_slope = 0.0;
            active_terrain_state.slope_angle = 0.0;
            active_terrain_state.normal = Eigen::Vector3d::UnitZ();
            active_terrain_state.rotation = Eigen::Matrix3d::Identity();
            active_terrain_state.is_valid = true;
        } else if (task_controller_->isActive()) {
            active_terrain_state.pitch_slope = 0.0;
            active_terrain_state.rotation = Eigen::AngleAxisd(
                active_terrain_state.roll_slope, Eigen::Vector3d::UnitX()).toRotationMatrix();
        }

        // Call updateHybrid using contact points from Phase 2.5 and body_state from Phase 2
        balance_out = balance_controller_->updateHybrid(
            active_height, active_roll, active_pitch,
            target_steer, curr_ecc_angles_, curr_ecc_efforts_, body_state,
            contact_points_world, contact_valid, active_terrain_state, dt, e_stop_active_, active_cmd_vel);

        target_ecc = balance_out.ecc_angles;

        // ============================================================
        // Phase 7: FSM Override — replace specific legs if climbing
        // ============================================================
        // Always snapshot the FSM override angles; needed for blend-out lerp in Phase 8
        // even when isActive() is false (during the IDLE blend-out period).
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

        // Apply TransitionSmoother (Zero-jerk mode switch S-curve)
        if (transition_smoother_.in_transition) {
            double alpha = transition_smoother_.getWeight();
            for (int i = 0; i < NUM_LEGS; ++i) {
                target_ecc(i) = (1.0 - alpha) * transition_smoother_.start_ecc_angles(i) + alpha * target_ecc(i);
            }
        }

        // ============================================================
        // Phase 8: Publish joint commands
        // ============================================================
        sensor_msgs::msg::JointState cmd_msg;
        cmd_msg.header.stamp = now;

        std::vector<std::string> steer_names = {
            "Steering_joint_LF", "Steering_joint_RF",
            "Steering_joint_LB", "Steering_joint_RB"};
        std::vector<std::string> ecc_names = {
            "Posture_control_joint_LF", "Posture_control_joint_RF",
            "Posture_control_joint_LB", "Posture_control_joint_RB"};
        std::vector<std::string> wheel_names = {
            "Wheel_joint_LF", "Wheel_joint_RF",
            "Wheel_joint_LB", "Wheel_joint_RB"};

        for (int i = 0; i < 4; ++i) {
            // Steering: math → CAD
            cmd_msg.name.push_back(steer_names[i]);
            cmd_msg.position.push_back(target_steer(i) * steer_signs_[i]);
            cmd_msg.velocity.push_back(0.0);
            double steer_effort = driving_controller_->getSteerTorques()(i) * steer_signs_[i];
            cmd_msg.effort.push_back(steer_effort);

            // Eccentric: math → CAD
            cmd_msg.name.push_back(ecc_names[i]);

            // Convert math-frame target directly to CAD frame (target_ecc is already smooth and continuous)
            double raw_cad = target_ecc(i) * ecc_signs_[i] + ecc_offsets_[i];
            prev_cad_ecc_[i] = raw_cad;

            cmd_msg.position.push_back(raw_cad);
            cmd_msg.velocity.push_back(0.0);

            // Zero out feedforward GRF torque for FSM-overridden legs to prevent conflict with position trajectory
            bool leg_is_overridden = task_controller_->isActive() && ecc_override_mask[i];
            double ff_weight = transition_smoother_.getFeedforwardWeight();
            double effort_cmd = leg_is_overridden
                              ? 0.0
                              : balance_out.ecc_torques(i) * ecc_signs_[i] * ff_weight;
            effort_cmd = std::clamp(effort_cmd, -50.0, 50.0);
            cmd_msg.effort.push_back(effort_cmd);

            // Wheels: math → CAD
            cmd_msg.name.push_back(wheel_names[i]);
            cmd_msg.position.push_back(0.0);

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
            cmd_msg.velocity.push_back(target_wheel_vel * wheel_signs_[i]);
            double wheel_effort = driving_controller_->getWheelTorques()(i) * wheel_signs_[i];
            cmd_msg.effort.push_back(wheel_effort);
        }

        pub_joint_cmds_->publish(cmd_msg);
    }

    // ================================================================
    // Algorithm Modules
    // ================================================================
    std::shared_ptr<kinematics::MobedKinematics>      kinematics_;
    std::unique_ptr<controllers::DrivingController>  driving_controller_;
    std::unique_ptr<controllers::BalanceController>   balance_controller_;
    std::unique_ptr<estimation::StateEstimator>       state_estimator_;
    std::unique_ptr<estimation::TerrainEstimator>     terrain_estimator_;
    std::unique_ptr<estimation::ContactDetector>      contact_detector_;
    std::unique_ptr<tasks::TaskController>            task_controller_;
    std::unique_ptr<estimation::ClimbGeometryEstimator> climb_geometry_estimator_;

    RobotParams robot_params_;
    ClimbMode climb_mode_ = ClimbMode::ADAPTIVE;

    // Zero-Jerk Mode Transition Smoother (0.2s Cosine S-curve)
    struct TransitionSmoother {
        bool in_transition = false;
        double timer = 0.0;
        double duration = 0.20;
        Eigen::Vector4d start_ecc_angles = Eigen::Vector4d::Zero();
        Eigen::Vector4d start_wheel_vels = Eigen::Vector4d::Zero();
        double start_ff_torque_weight = 1.0;
        double target_ff_torque_weight = 1.0;

        void start(const Eigen::Vector4d& curr_ecc, const Eigen::Vector4d& curr_wheel_vels,
                   double start_ff_weight, double target_ff_weight, double trans_duration = 0.20) {
            in_transition = true;
            timer = 0.0;
            duration = trans_duration;
            start_ecc_angles = curr_ecc;
            start_wheel_vels = curr_wheel_vels;
            start_ff_torque_weight = start_ff_weight;
            target_ff_torque_weight = target_ff_weight;
        }

        void update(double dt) {
            if (!in_transition) return;
            timer += dt;
            if (timer >= duration) {
                in_transition = false;
                timer = duration;
            }
        }

        double getWeight() const {
            if (!in_transition) return 1.0;
            double s = std::clamp(timer / duration, 0.0, 1.0);
            return 0.5 * (1.0 - std::cos(M_PI * s));
        }

        double getFeedforwardWeight() const {
            if (!in_transition) return target_ff_torque_weight;
            double w = getWeight();
            return (1.0 - w) * start_ff_torque_weight + w * target_ff_torque_weight;
        }
    } transition_smoother_;

    // ================================================================
    // ROS 2 Communication
    // ================================================================
    rclcpp::Subscription<robot_interfaces::msg::MobEDCommand>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_e_stop_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_payload_mass_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_curb_height_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_posture_mode_int_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_posture_mode_str_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_climb_mode_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_cmds_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
    rclcpp::Time last_cmd_vel_time_;

    // ================================================================
    // State Variables
    // ================================================================
    Eigen::Vector3d cmd_vel_;
    double target_height_ = 0.205;
    double target_roll_   = 0.0;
    double target_pitch_  = 0.0;

    Eigen::Vector4d curr_steer_angles_;
    Eigen::Vector4d curr_steer_velocities_;
    Eigen::Vector4d curr_ecc_angles_;
    Eigen::Vector4d curr_ecc_velocities_;
    Eigen::Vector4d curr_ecc_efforts_;
    Eigen::Vector4d curr_wheel_efforts_;
    Eigen::Vector4d curr_wheel_velocities_;
    Eigen::Vector4d prev_target_wheel_ = Eigen::Vector4d::Zero();

    ImuData latest_imu_;
    bool imu_received_ = false;
    bool joints_initialized_ = false;  // true after first real joint_states received

    bool e_stop_active_ = false;
    double last_raw_joint_0_ = 0.0;
    bool is_homing_ = true;
    double homing_timer_ = 0.0;
    const double HOMING_DURATION = 3.0;

    // Polarity Mapping Arrays
    std::vector<double> steer_signs_;
    std::vector<double> ecc_signs_;
    std::vector<double> wheel_signs_;
    std::vector<double> ecc_offsets_;

    // Continuous angle unwrap state for eccentric joint CAD commands.
    // Tracks the last-commanded CAD-frame angle for each ecc joint to ensure
    // the shortest-arc delta is applied each cycle, preventing 360° runaway.
    std::array<double, 4> prev_cad_ecc_ = {0.0, 0.0, 0.0, 0.0};
};

}  // namespace robot_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_control::MobedControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
