#pragma once

#include <array>
#include <memory>
#include "robot_control/core/mobed_types.hpp"
#include "robot_control/core/robot_params.hpp"
#include "robot_control/kinematics/mobed_kinematics.hpp"
#include "robot_control/controllers/driving_controller.hpp"
#include "robot_control/controllers/balance_controller.hpp"
#include "robot_control/estimation/state_estimator.hpp"
#include "robot_control/estimation/terrain_estimator.hpp"
#include "robot_control/estimation/contact_detector.hpp"
#include "robot_control/estimation/climb_geometry_estimator.hpp"
#include "robot_control/tasks/task_controller.hpp"

namespace robot_control {

// C-compatible sensor input structure
struct SensorInputs {
    double imu_accel[3];     // ax, ay, az (m/s^2, body frame)
    double imu_gyro[3];      // wx, wy, wz (rad/s, body frame)
    double imu_quat[4];      // qw, qx, qy, qz
    double steer_pos[4];     // LF, RF, LB, RB (CAD frame, rad)
    double steer_vel[4];     // rad/s
    double steer_effort[4];  // Nm
    double ecc_pos[4];       // LF, RF, LB, RB (CAD frame, rad)
    double ecc_vel[4];       // rad/s
    double ecc_effort[4];    // Nm
    double wheel_pos[4];     // rad
    double wheel_vel[4];     // rad/s
    double wheel_effort[4];  // Nm
};

// C-compatible command input structure
struct CommandInputs {
    double vx;               // m/s
    double vy;               // m/s
    double wz;               // rad/s
    double target_height;    // m (nominal: 0.205)
    double target_roll;      // rad (nominal: 0.0)
    double target_pitch;     // rad (nominal: 0.0)
    int    posture_mode;     // 0: OUTWARD, 1: INWARD, 2: SPIN, 3: STRAFE_OUTWARD
    int    climb_mode;       // 1: ADAPTIVE (autonomous multi-modal trigger)
    int    e_stop;           // 0: normal, 1: emergency stop
};

// C-compatible actuator output structure
struct ActuatorOutputs {
    double steer_pos[4];     // CAD frame target position (rad)
    double ecc_pos[4];       // CAD frame target position (continuous unwrapped rad)
    double wheel_vel[4];     // CAD frame target velocity (rad/s)
    double steer_effort[4];  // synthesized feedforward/PD effort (Nm)
    double ecc_effort[4];    // synthesized feedforward GRF/kick effort (Nm)
    double wheel_effort[4];  // synthesized traction effort (Nm)
    int    fsm_state;        // ClimbState ID (0: IDLE, 1: FRONT_WALL_CLIMB, etc.)
    double bank_angle;       // Desired motorcycle roll angle (rad)
    double terrain_pitch;    // Estimated terrain pitch (rad)
    double terrain_roll;     // Estimated terrain roll (rad)
};

class MobedControllerCore {
public:
    explicit MobedControllerCore(const RobotParams& params = RobotParams());
    ~MobedControllerCore() = default;

    void reset();
    void update(const SensorInputs& in, const CommandInputs& cmd, double dt, ActuatorOutputs& out);

private:
    RobotParams robot_params_;
    ClimbMode climb_mode_ = ClimbMode::ADAPTIVE;
    PostureConfiguration user_posture_config_ = PostureConfiguration::OUTWARD;

    std::shared_ptr<kinematics::MobedKinematics> kinematics_;
    std::unique_ptr<controllers::DrivingController> driving_controller_;
    std::unique_ptr<controllers::BalanceController> balance_controller_;
    std::unique_ptr<estimation::StateEstimator> state_estimator_;
    std::unique_ptr<estimation::TerrainEstimator> terrain_estimator_;
    std::unique_ptr<estimation::ContactDetector> contact_detector_;
    std::unique_ptr<estimation::ClimbGeometryEstimator> climb_geometry_estimator_;
    std::unique_ptr<tasks::TaskController> task_controller_;

    // Polarity & offset mappings
    std::array<double, 4> steer_signs_ = {1.0, 1.0, 1.0, 1.0};
    std::array<double, 4> ecc_signs_   = {1.0, -1.0, 1.0, -1.0};
    std::array<double, 4> wheel_signs_ = {1.0, -1.0, 1.0, -1.0};
    std::array<double, 4> ecc_offsets_ = {-1.9003, -3.0526, 1.5320, -3.0473};

    // Tracking states
    std::array<double, 4> prev_cad_ecc_ = {0.0, 0.0, 0.0, 0.0};
    bool joints_initialized_ = false;
    bool strafe_outward_active_ = false;
    double strafe_exit_timer_ = 0.0;
    Eigen::Vector4d prev_target_wheel_ = Eigen::Vector4d::Zero();

    // Homing & calibration tracking
    bool is_homing_ = true;
    double homing_timer_ = 0.0;

    // Zero-Jerk Mode Transition Smoother
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
};

} // namespace robot_control

// ----------------------------------------------------------------------------
// C-Linkage Exports for Python ctypes / native integration
// ----------------------------------------------------------------------------
extern "C" {
    void* mobed_create();
    void  mobed_destroy(void* handle);
    void  mobed_reset(void* handle);
    void  mobed_update(void* handle,
                       const robot_control::SensorInputs* in,
                       const robot_control::CommandInputs* cmd,
                       double dt,
                       robot_control::ActuatorOutputs* out);
}
