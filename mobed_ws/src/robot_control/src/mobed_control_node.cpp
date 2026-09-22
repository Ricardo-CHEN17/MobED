#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
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
        terrain_estimator_   = std::make_unique<estimation::TerrainEstimator>(robot_params_);
        contact_detector_    = std::make_unique<estimation::ContactDetector>(robot_params_);
        task_controller_     = std::make_unique<tasks::TaskController>();

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

        RCLCPP_INFO(this->get_logger(),
            "MobED Control Node started. Full pipeline: "
            "EKF + Terrain + Contact + FSM + Bank Angle + Hybrid Balance.");
    }

private:
    // ================================================================
    // Callback: Teleop/Navigation Command
    // ================================================================
    void commandCallback(const robot_interfaces::msg::MobEDCommand::SharedPtr msg) {
        cmd_vel_(0) = msg->twist.linear.x;
        cmd_vel_(1) = msg->twist.linear.y;
        cmd_vel_(2) = msg->twist.angular.z;

        target_height_ = msg->body_height;
        target_roll_   = msg->body_roll;
        target_pitch_  = msg->body_pitch;
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
                double q = (pos - ecc_offsets_[0]) * ecc_signs_[0];
                while (q >  M_PI) q -= 2.0 * M_PI;
                while (q <= -M_PI) q += 2.0 * M_PI;
                curr_ecc_angles_(0) = q;
                curr_ecc_velocities_(0) = vel * ecc_signs_[0];
                curr_ecc_efforts_(0) = effort;
            }
            else if (name == "Posture_control_joint_RF") {
                double q = (pos - ecc_offsets_[1]) * ecc_signs_[1];
                while (q >  M_PI) q -= 2.0 * M_PI;
                while (q <= -M_PI) q += 2.0 * M_PI;
                curr_ecc_angles_(1) = q;
                curr_ecc_velocities_(1) = vel * ecc_signs_[1];
                curr_ecc_efforts_(1) = effort;
            }
            else if (name == "Posture_control_joint_LB") {
                double q = (pos - ecc_offsets_[2]) * ecc_signs_[2];
                while (q >  M_PI) q -= 2.0 * M_PI;
                while (q <= -M_PI) q += 2.0 * M_PI;
                curr_ecc_angles_(2) = q;
                curr_ecc_velocities_(2) = vel * ecc_signs_[2];
                curr_ecc_efforts_(2) = effort;
            }
            else if (name == "Posture_control_joint_RB") {
                double q = (pos - ecc_offsets_[3]) * ecc_signs_[3];
                while (q >  M_PI) q -= 2.0 * M_PI;
                while (q <= -M_PI) q += 2.0 * M_PI;
                curr_ecc_angles_(3) = q;
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
    }

    // ================================================================
    // Callback: Emergency Stop
    // ================================================================
    void eStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        e_stop_active_ = msg->data;
        if (e_stop_active_) {
            RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED!");
            task_controller_->abort();
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

        // ============================================================
        // Phase 1: Homing Initialization
        // ============================================================
        if (is_homing_) {
            homing_timer_ += dt;
            if (homing_timer_ > HOMING_DURATION) {
                is_homing_ = false;
                RCLCPP_INFO(this->get_logger(), "Homing complete. Full control pipeline active.");
            }
        }

        // ============================================================
        // Phase 2: State Estimation (EKF predict + odometry correct Eq 3)
        // ============================================================
        if (imu_received_) {
            state_estimator_->predict(latest_imu_, dt);
        }
        Eigen::Vector3d omega_body = imu_received_ ? latest_imu_.angular_velocity : Eigen::Vector3d::Zero();
        state_estimator_->correctWithOdometry(
            curr_wheel_velocities_, curr_steer_angles_, curr_ecc_angles_,
            curr_steer_velocities_, curr_ecc_velocities_, omega_body);

        BodyState body_state = state_estimator_->getState();

        // ============================================================
        // Phase 3: Contact Detection (torque monitoring)
        // ============================================================
        contact_detector_->update(curr_ecc_efforts_, curr_wheel_efforts_, dt);

        // ============================================================
        // Phase 4: FSM Task Controller (climbing state machine)
        // ============================================================
        if (!is_homing_ && !e_stop_active_) {
            task_controller_->update(*contact_detector_, curr_ecc_angles_, dt);
        }

        // Log FSM state transitions
        static std::string prev_state_name = "IDLE";
        std::string curr_state_name = task_controller_->getStateName();
        if (curr_state_name != prev_state_name) {
            RCLCPP_INFO(this->get_logger(), "FSM Transition: %s → %s",
                        prev_state_name.c_str(), curr_state_name.c_str());
            prev_state_name = curr_state_name;
        }

        // ============================================================
        // Phase 2.5: Terrain Estimation (Eq 4-6 in paper)
        // ============================================================
        std::array<Eigen::Vector3d, NUM_LEGS> contact_points_world;
        std::array<bool, NUM_LEGS> contact_valid = task_controller_->getContactValid();

        Eigen::Vector3d base_pos = body_state.position;
        Eigen::Matrix3d R_base = body_state.orientation.toRotationMatrix();

        for (int i = 0; i < NUM_LEGS; ++i) {
            Eigen::Vector3d B_r_i = kinematics_->computeFootPositionInBase(
                i, curr_steer_angles_(i), curr_ecc_angles_(i));
            // p_C = p + R * ^B r_i (Eq 2)
            contact_points_world[i] = base_pos + R_base * B_r_i;
        }

        // ============================================================
        // Phase 5: Driving Controller (with Bank Angle)
        // ============================================================
        Eigen::Vector3d active_cmd_vel = is_homing_ ? Eigen::Vector3d::Zero() : cmd_vel_;

        // If FSM is actively climbing, override forward velocity
        if (task_controller_->isActive()) {
            double fsm_vx = task_controller_->getForwardVelocityOverride();
            active_cmd_vel = Eigen::Vector3d(fsm_vx, 0.0, 0.0);
        }

        // Run plane fitting via pseudo-inverse least squares (Eq 5: a = W^+ p^z)
        // Only update continuous terrain plane when FSM is not actively negotiating discontinuous obstacles
        if (!task_controller_->isActive()) {
            bool is_stopped = (active_cmd_vel.norm() < 1e-3);
            terrain_estimator_->update(contact_points_world, contact_valid, is_stopped);
        }

        auto [target_steer, target_wheel] = driving_controller_->update(
            active_cmd_vel, curr_steer_angles_, curr_ecc_angles_, dt,
            e_stop_active_, is_homing_);

        // ============================================================
        // Phase 6: Balance Controller (with Bank Angle overlay)
        // ============================================================
        double active_height = is_homing_ ? 0.22 : target_height_;
        double active_roll   = is_homing_ ? 0.0 : target_roll_;
        double active_pitch  = is_homing_ ? 0.0 : target_pitch_;

        // Overlay bank angle from driving controller onto target roll
        active_roll += driving_controller_->getBankAngle();

        Eigen::Matrix3d R_T = terrain_estimator_->getState().rotation;

        // Call updateHybrid using contact points from Phase 2.5 and body_state from Phase 2
        ControlOutput balance_out = balance_controller_->updateHybrid(
            active_height, active_roll, active_pitch,
            target_steer, curr_ecc_angles_, body_state,
            contact_points_world, contact_valid, R_T, dt, e_stop_active_);

        Eigen::Vector4d target_ecc = balance_out.ecc_angles;

        // ============================================================
        // Phase 7: FSM Override — replace specific legs if climbing
        // ============================================================
        if (task_controller_->isActive()) {
            auto override_mask = task_controller_->getOverrideMask();
            Eigen::Vector4d ecc_overrides = task_controller_->getEccOverrides();

            for (int i = 0; i < NUM_LEGS; ++i) {
                if (override_mask[i]) {
                    target_ecc(i) = ecc_overrides(i);
                }
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
            cmd_msg.effort.push_back(0.0);

            // Eccentric: math → CAD (position + hybrid torque)
            cmd_msg.name.push_back(ecc_names[i]);
            double cad_angle = target_ecc(i) * ecc_signs_[i] + ecc_offsets_[i];
            while (cad_angle >  M_PI) cad_angle -= 2.0 * M_PI;
            while (cad_angle <= -M_PI) cad_angle += 2.0 * M_PI;
            cmd_msg.position.push_back(cad_angle);
            cmd_msg.velocity.push_back(0.0);

            // Zero out feedforward GRF torque for FSM-overridden legs to prevent conflict with position trajectory
            double effort_cmd = (task_controller_->isActive() && task_controller_->getOverrideMask()[i])
                              ? 0.0
                              : balance_out.ecc_torques(i) * ecc_signs_[i];
            cmd_msg.effort.push_back(effort_cmd);

            // Wheels: math → CAD
            cmd_msg.name.push_back(wheel_names[i]);
            cmd_msg.position.push_back(0.0);
            cmd_msg.velocity.push_back(target_wheel(i) * wheel_signs_[i]);
            cmd_msg.effort.push_back(0.0);
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

    RobotParams robot_params_;

    // ================================================================
    // ROS 2 Communication
    // ================================================================
    rclcpp::Subscription<robot_interfaces::msg::MobEDCommand>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_e_stop_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_cmds_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    // ================================================================
    // State Variables
    // ================================================================
    Eigen::Vector3d cmd_vel_;
    double target_height_ = 0.22;
    double target_roll_   = 0.0;
    double target_pitch_  = 0.0;

    Eigen::Vector4d curr_steer_angles_;
    Eigen::Vector4d curr_steer_velocities_;
    Eigen::Vector4d curr_ecc_angles_;
    Eigen::Vector4d curr_ecc_velocities_;
    Eigen::Vector4d curr_ecc_efforts_;
    Eigen::Vector4d curr_wheel_efforts_;
    Eigen::Vector4d curr_wheel_velocities_;

    ImuData latest_imu_;
    bool imu_received_ = false;

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
};

}  // namespace robot_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_control::MobedControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
