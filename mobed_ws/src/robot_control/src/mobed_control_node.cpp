#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_interfaces/msg/mob_ed_command.hpp"
#include "robot_control/controllers/driving_controller.hpp"
#include "robot_control/controllers/balance_controller.hpp"

using namespace std::chrono_literals;

namespace robot_control {

class MobedControlNode : public rclcpp::Node {
public:
    MobedControlNode() : Node("mobed_control_node") {
        // 1. Initialize mathematical kinematics and controllers
        auto kinematics = std::make_shared<kinematics::MobedKinematics>();
        
        controllers::DrivingControllerParams drive_params;
        controllers::BalanceControllerParams balance_params;
        
        driving_controller_ = std::make_unique<controllers::DrivingController>(drive_params, kinematics);
        balance_controller_ = std::make_unique<controllers::BalanceController>(balance_params, kinematics);
        
        // 2. Initialize Internal States
        curr_steer_angles_.setZero();
        curr_ecc_angles_.setZero();
        cmd_vel_.setZero();
        
        // 3. Create Subscribers
        sub_cmd_ = this->create_subscription<robot_interfaces::msg::MobEDCommand>(
            "/mobed/command", 10, std::bind(&MobedControlNode::commandCallback, this, std::placeholders::_1));
            
        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&MobedControlNode::jointStateCallback, this, std::placeholders::_1));
            
        sub_e_stop_ = this->create_subscription<std_msgs::msg::Bool>(
            "/e_stop", 10, std::bind(&MobedControlNode::eStopCallback, this, std::placeholders::_1));
            
        // 4. Create Publishers
        pub_joint_cmds_ = this->create_publisher<sensor_msgs::msg::JointState>("/mobed/joint_commands", 10);
        
        // 5. Create 100Hz Control Loop Timer
        timer_ = this->create_wall_timer(10ms, std::bind(&MobedControlNode::timerCallback, this));
        last_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "MobED Control Node started successfully.");
    }
    
private:
    void commandCallback(const robot_interfaces::msg::MobEDCommand::SharedPtr msg) {
        cmd_vel_(0) = msg->twist.linear.x;
        cmd_vel_(1) = msg->twist.linear.y;
        cmd_vel_(2) = msg->twist.angular.z;
        
        target_height_ = msg->body_height;
        target_roll_ = msg->body_roll;
        target_pitch_ = msg->body_pitch;
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const auto& name = msg->name[i];
            double pos = msg->position[i];
            
            // Map names perfectly matched with MuJoCo XML
            if (name == "Steering_joint_LF") curr_steer_angles_(0) = pos;
            else if (name == "Steering_joint_RF") curr_steer_angles_(1) = pos;
            else if (name == "Steering_joint_LB") curr_steer_angles_(2) = pos;
            else if (name == "Steering_joint_RB") curr_steer_angles_(3) = pos;
            
            else if (name == "Posture_control_joint_LF") curr_ecc_angles_(0) = pos;
            else if (name == "Posture_control_joint_RF") curr_ecc_angles_(1) = pos;
            else if (name == "Posture_control_joint_LB") curr_ecc_angles_(2) = pos;
            else if (name == "Posture_control_joint_RB") curr_ecc_angles_(3) = pos;
        }
    }
    
    void eStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        e_stop_active_ = msg->data;
        if (e_stop_active_) {
            RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED!");
        }
    }
    
    void timerCallback() {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        
        // Prevent huge dt jumps (e.g. simulation pause/resume)
        if (dt <= 0.0 || dt > 0.1) dt = 0.01;
        
        // Run Driving Controller (returns target steer and target wheel speeds)
        auto [target_steer, target_wheel] = driving_controller_->update(
            cmd_vel_, curr_steer_angles_, curr_ecc_angles_, dt, e_stop_active_);
            
        // Run Balance Controller (returns target eccentric angles)
        Eigen::Vector4d target_ecc = balance_controller_->update(
            target_height_, target_roll_, target_pitch_, curr_steer_angles_, curr_ecc_angles_, dt, e_stop_active_);
            
        // Pack into JointState and Publish
        sensor_msgs::msg::JointState cmd_msg;
        cmd_msg.header.stamp = now;
        
        std::vector<std::string> steer_names = {"Steering_joint_LF", "Steering_joint_RF", "Steering_joint_LB", "Steering_joint_RB"};
        std::vector<std::string> ecc_names = {"Posture_control_joint_LF", "Posture_control_joint_RF", "Posture_control_joint_LB", "Posture_control_joint_RB"};
        std::vector<std::string> wheel_names = {"Wheel_joint_LF", "Wheel_joint_RF", "Wheel_joint_LB", "Wheel_joint_RB"};
        
        for (int i = 0; i < 4; ++i) {
            // Steer (Position control)
            cmd_msg.name.push_back(steer_names[i]);
            cmd_msg.position.push_back(target_steer(i));
            cmd_msg.velocity.push_back(0.0);
            
            // Eccentric (Position control)
            cmd_msg.name.push_back(ecc_names[i]);
            cmd_msg.position.push_back(target_ecc(i));
            cmd_msg.velocity.push_back(0.0);
            
            // Wheel (Velocity control, sent via velocity field)
            cmd_msg.name.push_back(wheel_names[i]);
            cmd_msg.position.push_back(0.0);
            cmd_msg.velocity.push_back(target_wheel(i));
        }
        
        pub_joint_cmds_->publish(cmd_msg);
    }
    
    std::unique_ptr<controllers::DrivingController> driving_controller_;
    std::unique_ptr<controllers::BalanceController> balance_controller_;
    
    rclcpp::Subscription<robot_interfaces::msg::MobEDCommand>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_e_stop_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_cmds_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
    
    Eigen::Vector3d cmd_vel_;
    double target_height_ = 0.15; // default safe height
    double target_roll_ = 0.0;
    double target_pitch_ = 0.0;
    
    Eigen::Vector4d curr_steer_angles_;
    Eigen::Vector4d curr_ecc_angles_;
    
    bool e_stop_active_ = false;
};

} // namespace robot_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_control::MobedControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
