#include "robot_control/tasks/task_controller.hpp"
#include <cmath>
#include <algorithm>

namespace robot_control {
namespace tasks {

TaskController::TaskController(const TaskControllerParams& params)
    : params_(params)
{
    reset();
}

void TaskController::reset() {
    state_ = ClimbState::IDLE;
    ecc_overrides_ = Eigen::Vector4d::Zero();
    override_mask_.fill(false);
    wheel_speed_overrides_.setZero();
    wheel_override_mask_.fill(false);
    active_lift_mask_.fill(false);
    forward_vel_override_ = 0.0;
    state_timer_ = 0.0;
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 0.0;
    cooldown_timer_ = 0.0;
    travel_distance_ = 0.0;
    stall_jam_timer_ = 0.0;
    current_front_target_ = 0.0;
    lift_start_angles_ = Eigen::Vector4d::Zero();
    front_target_angles_ = Eigen::Vector2d::Zero();
    rear_target_angles_ = Eigen::Vector2d::Zero();
    has_climbed_ = false;
    snap_kick_active_ = false;
    coordinated_lift_active_ = false;
    landing_rear_q_ = -4.65;
}

void TaskController::abort() {
    blend_out_timer_ = 0.0;
    blend_out_weight_ = 1.0;
    cooldown_timer_ = params_.cooldown_duration;
    travel_distance_ = 0.0;
    stall_jam_timer_ = 0.0;
    forward_vel_override_ = 0.0;
    wheel_speed_overrides_.setZero();
    wheel_override_mask_.fill(false);
    active_lift_mask_.fill(false);
    snap_kick_active_ = false;
    coordinated_lift_active_ = false;
    landing_rear_q_ = -4.65;
    transitionTo(ClimbState::IDLE);
}

void TaskController::transitionTo(ClimbState new_state) {
    state_ = new_state;
    state_timer_ = 0.0;
    travel_distance_ = 0.0;
    stall_jam_timer_ = 0.0;
    if (new_state == ClimbState::IDLE) {
        snap_kick_active_ = false;
        coordinated_lift_active_ = false;
    }
}

double TaskController::computeFrontTargetAngle(double h_curb) const {
    if (h_curb > 0.15) {
        // 200mm high step: deep crouch to 1.85 rad (~106 deg)
        // Flattens chassis pitch to < 8 deg, shifts CoM forward by >45mm onto plateau,
        // dramatically increasing front wheel traction and eliminating backward tip-over.
        return 1.85;
    }
    double ratio = std::clamp((h_curb - 0.04) / (0.15 - 0.04), 0.0, 1.0);
    // 5cm:  1.20 rad (low pitch, plenty of belly clearance)
    // 10cm: 1.30 rad (pitch ~12 deg, clearance > 60mm)
    return 1.20 + ratio * (1.30 - 1.20);
}

double TaskController::computeAdaptiveLiftAngle([[maybe_unused]] double q_start) const {
    return computeFrontTargetAngle(params_.target_curb_height);
}

double TaskController::computeRearLevelingAngle([[maybe_unused]] double q_front_avg, double h_curb) const {
    double ratio = std::clamp((h_curb - 0.04) / (0.20 - 0.04), 0.0, 1.0);
    // Smooth transition from -0.55 rad (gentle 40mm) to -0.15 rad (extreme 200mm high clearance)
    return -0.55 + ratio * (-0.15 - (-0.55));
}

double TaskController::computeRearClimbTargetAngle(double h_curb) const {
    double ratio = std::clamp((h_curb - 0.04) / (0.20 - 0.04), 0.0, 1.0);
    // Height-adaptive rear climb target angle:
    // 40mm: -0.85 rad
    // 100mm: -1.25 rad (~71.6 deg, lifts axle ~40mm, directly clearing 100mm step to step on top)
    // 200mm: -2.10 rad (~120.3 deg, provides full vertical stroke to clear 200mm curb lip)
    return -0.85 - ratio * (2.10 - 0.85);
}

std::string TaskController::getStateName() const {
    switch (state_) {
        case ClimbState::IDLE:                  return "IDLE";
        case ClimbState::FRONT_WALL_CLIMB:      return "FRONT_WALL_CLIMB";
        case ClimbState::STRADDLE_LEVELING:     return "STRADDLE_LEVELING";
        case ClimbState::STRADDLE_HOLD:         return "STRADDLE_HOLD";
        case ClimbState::REAR_WALL_CLIMB:       return "REAR_WALL_CLIMB";
        case ClimbState::REAR_VAULT_LOOP:       return "REAR_VAULT_LOOP";
        case ClimbState::REAR_TOUCHDOWN_SETTLE: return "REAR_TOUCHDOWN_SETTLE";
        case ClimbState::REAR_CREEP_ADVANCE:    return "REAR_CREEP_ADVANCE";
        case ClimbState::REAR_CREST_FOLD:       return "REAR_CREST_FOLD";
        case ClimbState::REAR_POST_CLIMB_RESET: return "REAR_POST_CLIMB_RESET";
        case ClimbState::REAR_PLACED:           return "REAR_PLACED";
        default:                                return "UNKNOWN";
    }
}

void TaskController::update(
    const estimation::ContactDetector& contact_detector,
    const estimation::ClimbGeometry& geometry,
    const Eigen::Vector4d& current_ecc_angles,
    const Eigen::Vector4d& current_ecc_efforts,
    double dt,
    double body_vx,
    double cmd_vx)
{
    if (dt <= 0.0 || dt > 0.5) return;
    (void)current_ecc_efforts;

    state_timer_ += dt;

    // Clear overrides by default
    override_mask_.fill(false);
    forward_vel_override_ = 0.0;
    wheel_speed_overrides_.setZero();
    wheel_override_mask_.fill(false);

    switch (state_) {
        // ============================================================
        // 0. IDLE: Normal driving, monitoring for front wheel impact or wall-climbing
        // ============================================================
        case ClimbState::IDLE: {
            if (cooldown_timer_ > 0.0) {
                cooldown_timer_ = std::max(0.0, cooldown_timer_ - dt);
                if (cooldown_timer_ <= 0.0) {
                    has_climbed_ = false; // Auto-clear lockout after cooldown expires
                }
                break;
            }

            bool front_impact = contact_detector.frontImpactDetected();
            bool front_already_on_curb = (geometry.front_height >= 0.035) &&
                                         (geometry.delta_height >= 0.035) &&
                                         geometry.front_edge_crested && (std::abs(body_vx) > 0.02);

            if (front_impact) {
                lift_start_angles_ = current_ecc_angles;
                current_front_target_ = 0.5 * (current_ecc_angles(FL) + current_ecc_angles(FR));
                stall_jam_timer_ = 0.0;

                // Stage 1: Moderate initial climb posture (lift wheel without premature deep crouch)
                front_target_angles_(0) = 1.25;
                front_target_angles_(1) = 1.25;

                double target_r = computeRearLevelingAngle(1.25, params_.target_curb_height);
                rear_target_angles_(0) = target_r;
                rear_target_angles_(1) = target_r;

                active_lift_mask_[FL] = true;
                active_lift_mask_[FR] = true;
                active_lift_mask_[RL] = false;
                active_lift_mask_[RR] = false;
                transitionTo(ClimbState::FRONT_WALL_CLIMB);
            } else if (front_already_on_curb) {
                lift_start_angles_ = current_ecc_angles;
                current_front_target_ = 0.5 * (current_ecc_angles(FL) + current_ecc_angles(FR));
                stall_jam_timer_ = 0.0;

                double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
                double target_f = computeFrontTargetAngle(active_h);
                front_target_angles_(0) = target_f;
                front_target_angles_(1) = target_f;

                double target_r = computeRearLevelingAngle(target_f, active_h);
                rear_target_angles_(0) = target_r;
                rear_target_angles_(1) = target_r;

                active_lift_mask_[FL] = true;
                active_lift_mask_[FR] = true;
                active_lift_mask_[RL] = true;
                active_lift_mask_[RR] = true;
                transitionTo(ClimbState::STRADDLE_LEVELING);
            }
            break;
        }

        // ============================================================
        // 1. FRONT_WALL_CLIMB: Front wheels roll up wall; eccentric arms fold moderately with pitch feedback
        // ============================================================
        case ClimbState::FRONT_WALL_CLIMB: {
            // Two-Stage Front Decoupled Folding:
            // Stage 1 (Wall Climbing): Lift wheels and absorb pitch, but DO NOT deep-crouch to bottom out chassis!
            // Fold moderately up to 1.28 rad (~73 deg), ensuring belly clearance > 70mm while wheels bite the wall.
            double theta_ref = 0.080; // nominal nose-up pitch reference during climb
            double pitch_err = geometry.nose_up_pitch - theta_ref;
            double q_dot_fold = 1.2; // steady adaptive climbing rate
            if (geometry.nose_up_pitch > theta_ref) {
                q_dot_fold = 1.2 + 5.0 * pitch_err + 0.3 * geometry.nose_up_pitch_rate;
                q_dot_fold = std::clamp(q_dot_fold, 0.4, 3.0);
            }

            current_front_target_ += q_dot_fold * dt;
            current_front_target_ = std::clamp(current_front_target_,
                                               0.5 * (lift_start_angles_(FL) + lift_start_angles_(FR)),
                                               1.28); // Moderate wall-climb ceiling (NOT 1.95 rad yet!)

            // Override front legs
            override_mask_[FL] = true;
            override_mask_[FR] = true;
            ecc_overrides_(FL) = current_front_target_;
            ecc_overrides_(FR) = current_front_target_;

            // Rear legs: remain under Balance Controller control (In Tandem with Balance Controller)
            // Balance controller dynamically manages ground reaction forces and prevents backward flipping!
            override_mask_[RL] = false;
            override_mask_[RR] = false;

            // Traction drive: front wheels spin forward to grip wall, rear wheels push forward
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_(FL) = 3.5; // active wall roll speed
            wheel_speed_overrides_(FR) = 3.5;
            double rear_push_vx = 0.15;
            wheel_speed_overrides_(RL) = rear_push_vx / params_.wheel_radius;
            wheel_speed_overrides_(RR) = rear_push_vx / params_.wheel_radius;
            forward_vel_override_ = rear_push_vx;

            // Dynamic Progress Watchdog (replacing rigid 2.0s timer):
            // Check if active climb progress is occurring:
            // 1. Vertical climb progress: front_z_vel > 0.010 m/s OR delta_height increasing
            // 2. Arm folding progress: current_front_target_ < 1.25 rad
            // 3. Chassis moving or operator actively pushing stick forward (cmd_vx > 0.08)
            bool active_climb_motion = (geometry.front_z_vel > 0.010) || (geometry.delta_height > 0.040);
            bool active_arm_folding = (current_front_target_ < 1.25);
            bool operator_driving = (cmd_vx > 0.08);

            if (active_climb_motion || (active_arm_folding && operator_driving)) {
                stall_jam_timer_ = 0.0; // Active progress: pet the watchdog
            } else {
                stall_jam_timer_ += dt;
            }

            // Safety abort conditions:
            // 1. Physical hard jam: zero climb progress for > 2.50s continuously while stalled
            // 2. Dangerous pitch: chassis pitch > 28 deg (0.49 rad) backward tipping hazard
            // 3. Operator reverse: cmd_vx < -0.15
            bool physical_jam = (stall_jam_timer_ >= 2.50);
            bool pitch_hazard = (geometry.nose_up_pitch > 0.49);
            bool operator_abort = (cmd_vx < -0.15);

            if (physical_jam || pitch_hazard || operator_abort) {
                cooldown_timer_ = params_.cooldown_duration;
                transitionTo(ClimbState::IDLE);
                break;
            }

            // Transition to STRADDLE_LEVELING once front wheels have crested the top edge
            // Evaluated autonomously by the tri-feature edge observer
            if (geometry.front_edge_crested) {
                double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
                double target_f = computeFrontTargetAngle(active_h);
                front_target_angles_(0) = target_f;
                front_target_angles_(1) = target_f;

                double target_r = computeRearLevelingAngle(target_f, active_h);
                rear_target_angles_(0) = target_r;
                rear_target_angles_(1) = target_r;

                travel_distance_ = 0.0;
                transitionTo(ClimbState::STRADDLE_LEVELING);
            }
            break;
        }

        // ============================================================
        // 2. STRADDLE_LEVELING: Front wheels on plateau; deep-crouch to plant chassis & skid wheels
        // ============================================================
        case ClimbState::STRADDLE_LEVELING: {
            // Operator intent check: if operator releases throttle, pause into STRADDLE_HOLD immediately
            if (cmd_vx < 0.05) {
                transitionTo(ClimbState::STRADDLE_HOLD);
                break;
            }

            double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
            double target_f_adaptive = computeFrontTargetAngle(active_h);
            front_target_angles_(0) = target_f_adaptive;
            front_target_angles_(1) = target_f_adaptive;

            // Stage 2 (Straddle Leveling & Deep Crouch):
            // Smoothly fold front legs from moderate climb angle (~1.25) to target_f_adaptive (1.85 rad for 200mm)
            // For 200mm, flattens pitch to < 8 deg, pulls CoM deeply onto plateau and firmly plants front wheels.
            double crouch_time = (active_h > 0.15) ? 0.40 : 0.35;
            double crouch_progress = std::clamp(state_timer_ / crouch_time, 0.0, 1.0);
            double q_f_base = (1.0 - crouch_progress) * 1.25 + crouch_progress * target_f_adaptive;
            double pitch_corr = params_.leveling_kp_pitch * std::max(0.0, geometry.nose_up_pitch);
            double max_q_f = (active_h > 0.15) ? 1.90 : 1.45;
            double q_f = std::clamp(q_f_base + 0.3 * pitch_corr, 1.20, max_q_f);

            double q_r = computeRearLevelingAngle(q_f, active_h);
            rear_target_angles_(0) = q_r;
            rear_target_angles_(1) = q_r;

            // Active coordination on all legs
            override_mask_.fill(true);
            ecc_overrides_(FL) = q_f;
            ecc_overrides_(FR) = q_f;
            ecc_overrides_(RL) = q_r;
            ecc_overrides_(RR) = q_r;

            // Traction drive: front wheels pull forward strongly to drag chassis, rear wheels push forward
            wheel_override_mask_.fill(true);
            double pull_spd = 4.5; // rad/s (~0.45 m/s) strong traction pull
            double push_spd = 3.5;
            wheel_speed_overrides_(FL) = pull_spd;
            wheel_speed_overrides_(FR) = pull_spd;
            wheel_speed_overrides_(RL) = push_spd;
            wheel_speed_overrides_(RR) = push_spd;
            forward_vel_override_ = 0.35;

            // Integrate travel distance (only integrate true forward motion)
            double effective_vx = (body_vx > 0.02) ? body_vx : 0.0;
            travel_distance_ += effective_vx * dt;

            // Multi-cue arrival verification:
            // Cue 1: Dual-track contact detector (reaction torque / momentum arrest)
            // Cue 2: Odometry path arrival (>= 0.22m forward travel + rear slip)
            // Cue 3: Physical stall backup while driving forward
            bool rear_impact = contact_detector.rearImpactDetected();
            bool front_deep_crouched = (active_h > 0.15) ? (q_f >= 1.82 && state_timer_ >= 0.45)
                                                         : (state_timer_ >= 0.20);
            bool odometry_arrival = (geometry.travel_distance >= 0.22 && geometry.rear_slip_ratio > 0.65);
            bool stall_arrival = (state_timer_ >= 0.60 && std::abs(body_vx) < 0.03 && geometry.travel_distance >= 0.10);

            if (front_deep_crouched && (rear_impact || odometry_arrival || stall_arrival)) {
                active_lift_mask_[RL] = true;
                active_lift_mask_[RR] = true;
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_WALL_CLIMB);
            }
            break;
        }

        // ============================================================
        // 2b. STRADDLE_HOLD: Operator paused during straddle; lock stance & balance closed-loop
        // ============================================================
        case ClimbState::STRADDLE_HOLD: {
            double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
            double target_f = computeFrontTargetAngle(active_h);
            double pitch_corr = params_.leveling_kp_pitch * std::max(0.0, geometry.nose_up_pitch);
            double q_f = std::clamp(target_f + 0.3 * pitch_corr, 1.20, 1.45);
            double q_r = computeRearLevelingAngle(q_f, active_h);

            override_mask_.fill(true);
            ecc_overrides_(FL) = q_f;
            ecc_overrides_(FR) = q_f;
            ecc_overrides_(RL) = q_r;
            ecc_overrides_(RR) = q_r;

            // Brake wheels during pause hold (zero velocity command, holding brakes)
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setZero();
            forward_vel_override_ = 0.0;

            // ZERO TIMEOUT: Permanent hold while paused! (Never timed out to IDLE)
            // Resume forward climbing when user presses forward
            if (cmd_vx > 0.08) {
                transitionTo(ClimbState::STRADDLE_LEVELING);
            }
            // Dismount backwards only when user commands intentional reverse
            else if (cmd_vx < -0.20) {
                cooldown_timer_ = params_.cooldown_duration;
                transitionTo(ClimbState::IDLE);
            }
            // Auto-recovery: If falsely triggered on flat ground (delta_height < 35mm) and paused > 1.0s, return to IDLE
            else if (geometry.delta_height < 0.035 && state_timer_ > 1.0) {
                cooldown_timer_ = params_.cooldown_duration;
                transitionTo(ClimbState::IDLE);
            }
            break;
        }

        // ============================================================
        // 3. REAR_WALL_CLIMB: Rear wheels at wall
        //    - 80~110mm: Snappy sweep to -1.25 rad preparing for dynamic snap-kick
        //    - 200mm: Gentle soft descent (0.40s) to rest chassis belly on curb edge
        //    - Other low obstacles: standard adaptive lift
        // ============================================================
        case ClimbState::REAR_WALL_CLIMB: {
            double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
            double current_front_q = front_target_angles_(0);
            override_mask_.fill(true);
            ecc_overrides_(FL) = current_front_q;
            ecc_overrides_(FR) = current_front_q;

            // ----------------------------------------------------
            // Branch 1: 200mm High Step (active_h > 0.15m)
            // Soft Descent: front wheels braked, rear arms slowly rotate to -1.20 rad (0.40s)
            // gently and softly resting the chassis belly on the 200mm curb edge!
            // ----------------------------------------------------
            if (active_h > 0.15) {
                wheel_override_mask_.fill(true);
                wheel_speed_overrides_.setZero(); // Front wheels firmly braked: chassis stays completely steady
                forward_vel_override_ = 0.0;

                double descent_progress = std::clamp(state_timer_ / 0.40, 0.0, 1.0);
                double current_rear_q = (1.0 - descent_progress) * rear_target_angles_(0) + descent_progress * (-1.20);
                ecc_overrides_(RL) = current_rear_q;
                ecc_overrides_(RR) = current_rear_q;

                // Once belly is settled on the curb edge (0.40s), transition to fast 360-degree vault loop!
                if (descent_progress >= 1.0 || state_timer_ >= 0.40) {
                    travel_distance_ = 0.0;
                    transitionTo(ClimbState::REAR_VAULT_LOOP);
                }
            }
            // ----------------------------------------------------
            // Branch 2: 80mm ~ 110mm Medium Step (Snap-Kick Mode)
            // Snappy preparation sweep to -1.25 rad in 0.22s with strong forward pull
            // ----------------------------------------------------
            else if (active_h >= 0.075 && active_h <= 0.115) {
                wheel_override_mask_.fill(true);
                double pull_vx = 0.45;
                wheel_speed_overrides_.setConstant(pull_vx / params_.wheel_radius);
                forward_vel_override_ = pull_vx;

                double sweep_progress = std::clamp(state_timer_ / 0.22, 0.0, 1.0);
                double current_rear_q = (1.0 - sweep_progress) * rear_target_angles_(0) + sweep_progress * (-1.25);
                ecc_overrides_(RL) = current_rear_q;
                ecc_overrides_(RR) = current_rear_q;

                if (sweep_progress >= 1.0 || state_timer_ >= 0.22) {
                    travel_distance_ = 0.0;
                    snap_kick_active_ = true;
                    transitionTo(ClimbState::REAR_CREST_FOLD);
                }
            }
            // ----------------------------------------------------
            // Branch 3: Standard low obstacle (<= 75mm)
            // ----------------------------------------------------
            else {
                wheel_override_mask_.fill(true);
                double pull_vx = 0.35;
                wheel_speed_overrides_.setConstant(pull_vx / params_.wheel_radius);
                forward_vel_override_ = pull_vx;

                double q_r_target = computeRearClimbTargetAngle(active_h);
                double sweep_progress = std::clamp(state_timer_ / 0.30, 0.0, 1.0);
                double current_rear_q = (1.0 - sweep_progress) * rear_target_angles_(0) + sweep_progress * q_r_target;
                ecc_overrides_(RL) = current_rear_q;
                ecc_overrides_(RR) = current_rear_q;

                bool rear_wheels_lifted = (geometry.rear_height >= (active_h - 0.025));
                if (sweep_progress >= 1.0 && (rear_wheels_lifted || state_timer_ >= 0.35)) {
                    travel_distance_ = 0.0;
                    transitionTo(ClimbState::REAR_CREST_FOLD);
                } else if (state_timer_ >= 2.50) {
                    cooldown_timer_ = params_.cooldown_duration;
                    transitionTo(ClimbState::IDLE);
                }
            }
            break;
        }

        // ============================================================
        // 3b. REAR_VAULT_LOOP: 200mm only — High-speed 360-degree forward vault loop
        //     Chassis belly rests stably on curb edge (stationary), while rear legs
        //     rapidly whip over the top (+6mm clearance) and land in front on the plateau!
        // ============================================================
        case ClimbState::REAR_VAULT_LOOP: {
            double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
            wheel_override_mask_.fill(true);
            // Dynamic Vault Pull: Front wheels pull forward at 0.38 m/s to shift chassis forward 80~120mm
            // while rear wheels are airborne, completely bypassing the curb corner lip!
            // Airborne rear wheels are pre-spun forward at the same speed to eliminate landing brake shock.
            double vault_pull_vx = 0.38;
            wheel_speed_overrides_.setConstant(vault_pull_vx / params_.wheel_radius);
            forward_vel_override_ = vault_pull_vx;

            double current_front_q = front_target_angles_(0);
            override_mask_.fill(true);
            ecc_overrides_(FL) = current_front_q;
            ecc_overrides_(FR) = current_front_q;

            // Rapid 360-degree forward loop vault in 0.42s:
            // From -1.20 rad past -pi (-3.14) over the top, down onto plateau to -4.65 rad!
            double vault_progress = std::clamp(state_timer_ / 0.42, 0.0, 1.0);
            double q_r_vault = -1.20 - vault_progress * (4.65 - 1.20);
            ecc_overrides_(RL) = q_r_vault;
            ecc_overrides_(RR) = q_r_vault;

            // Multi-modal touchdown detection:
            // 1. Angular sector: arm has reached landing angle <= -4.55 rad or vault_progress >= 0.95
            // 2. Physical contact: rear height near plateau (>= active_h - 0.035) or rear_edge_crested or reaction torque
            bool angle_reached = (vault_progress >= 0.95) || (q_r_vault <= -4.55);
            bool physical_touch = geometry.rear_edge_crested ||
                                 (geometry.rear_height >= (active_h - 0.035)) ||
                                 (std::abs(current_ecc_efforts(RL)) > 2.0 || std::abs(current_ecc_efforts(RR)) > 2.0);
            if ((angle_reached && physical_touch && state_timer_ >= 0.32) || state_timer_ >= 0.48) {
                landing_rear_q_ = ecc_overrides_(RL);
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_TOUCHDOWN_SETTLE);
            }
            break;
        }

        // ============================================================
        // 3c. REAR_TOUCHDOWN_SETTLE: 200mm only — 0.20s Stance freeze & damping buffer
        //     Freeze rear legs at landing angle, wheels braked, dissipate all loop kinetic energy
        // ============================================================
        case ClimbState::REAR_TOUCHDOWN_SETTLE: {
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setZero(); // Wheels firmly braked: zero motion, zero bounce
            forward_vel_override_ = 0.0;

            override_mask_.fill(true);
            ecc_overrides_(FL) = front_target_angles_(0); // hold deep crouch (1.85 rad)
            ecc_overrides_(FR) = front_target_angles_(0);
            ecc_overrides_(RL) = landing_rear_q_;         // hold landing angle (-4.65 rad)
            ecc_overrides_(RR) = landing_rear_q_;

            // Damping pause duration: 0.20s
            if (state_timer_ >= 0.20) {
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_CREEP_ADVANCE);
            }
            break;
        }

        // ============================================================
        // 3d. REAR_CREEP_ADVANCE: 200mm only — Four-wheel creeping forward crawl (50~80mm)
        //     Maintain low stance, crawl slowly at 0.12 m/s away from curb cliff edge
        // ============================================================
        case ClimbState::REAR_CREEP_ADVANCE: {
            override_mask_.fill(true);
            ecc_overrides_(FL) = front_target_angles_(0); // hold deep crouch
            ecc_overrides_(FR) = front_target_angles_(0);
            ecc_overrides_(RL) = landing_rear_q_;         // hold landing angle
            ecc_overrides_(RR) = landing_rear_q_;

            wheel_override_mask_.fill(true);
            double creep_vx = 0.12; // 0.12 m/s slow creeping crawl
            wheel_speed_overrides_.setConstant(creep_vx / params_.wheel_radius);
            forward_vel_override_ = creep_vx;

            double effective_vx = (std::abs(body_vx) > 0.02) ? std::abs(body_vx) : creep_vx;
            travel_distance_ += effective_vx * dt;

            // Advance 50mm (0.05m) away from curb edge onto plateau, or 0.40s watchdog
            if (travel_distance_ >= 0.05 || state_timer_ >= 0.40) {
                travel_distance_ = 0.0;
                coordinated_lift_active_ = true;
                transitionTo(ClimbState::REAR_CREST_FOLD);
            }
            break;
        }

        // ============================================================
        // 4. REAR_CREST_FOLD:
        //    - 80~110mm: High-speed dynamic snap-kick (0.18s + 35Nm downforce)
        //    - 200mm: Coordinated 4-wheel chassis lift (0.55s) & plateau extraction
        //    - Low obstacles: standard smooth crest fold
        // ============================================================
        case ClimbState::REAR_CREST_FOLD: {
            double active_h = geometry.has_latched_height ? geometry.latched_curb_height : params_.target_curb_height;
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            double nominal_r = -0.65;
            double current_front_q = front_target_angles_(0);
            ecc_overrides_(FL) = current_front_q;
            ecc_overrides_(FR) = current_front_q;

            // ----------------------------------------------------
            // Mode A: 80~110mm Dynamic Snap-Kick (0.18s)
            // ----------------------------------------------------
            if (snap_kick_active_) {
                double progress = std::clamp(state_timer_ / 0.18, 0.0, 1.0);
                double q_r = (1.0 - progress) * (-1.25) + progress * nominal_r;
                ecc_overrides_(RL) = q_r;
                ecc_overrides_(RR) = q_r;

                double kick_vx = 0.45;
                wheel_speed_overrides_.setConstant(kick_vx / params_.wheel_radius);
                forward_vel_override_ = kick_vx;

                double effective_vx = (std::abs(body_vx) > 0.02) ? std::abs(body_vx) : kick_vx;
                travel_distance_ += effective_vx * dt;

                bool rear_touchdown = geometry.rear_edge_crested || (state_timer_ >= 0.18);
                if (rear_touchdown) {
                    travel_distance_ = 0.0;
                    snap_kick_active_ = false;
                    transitionTo(ClimbState::REAR_PLACED);
                }
            }
            // ----------------------------------------------------
            // Mode B: 200mm Coordinated 4-Wheel Chassis Lift & Extraction (0.55s)
            // Synchronously push front & rear legs down to nominal, lifting chassis 65~80mm horizontally!
            // ----------------------------------------------------
            else if (coordinated_lift_active_) {
                double lift_duration = 0.55;
                double progress = std::clamp(state_timer_ / lift_duration, 0.0, 1.0);

                // Front legs lift from deep crouch (1.85 rad) up to nominal (0.65 rad)
                double q_f = (1.0 - progress) * front_target_angles_(0) + progress * 0.65;
                ecc_overrides_(FL) = q_f;
                ecc_overrides_(FR) = q_f;

                // Rear legs push down from forward landing angle (landing_rear_q_ ~ -4.71 rad)
                // to forward nominal angle (-5.63 rad = -2*pi + 0.65 rad):
                // Swings forward-downward by only 53 degrees, generating pure upward lift without backward kick!
                double q_r = (1.0 - progress) * landing_rear_q_ + progress * (-5.63);
                ecc_overrides_(RL) = q_r;
                ecc_overrides_(RR) = q_r;

                double pull_vx = 0.35;
                wheel_speed_overrides_.setConstant(pull_vx / params_.wheel_radius);
                forward_vel_override_ = pull_vx;

                double effective_vx = (std::abs(body_vx) > 0.02) ? std::abs(body_vx) : pull_vx;
                travel_distance_ += effective_vx * dt;

                if (progress >= 1.0 || state_timer_ >= 0.60) {
                    travel_distance_ = 0.0;
                    coordinated_lift_active_ = false;
                    front_target_angles_.setConstant(0.65);
                    transitionTo(ClimbState::REAR_POST_CLIMB_RESET);
                }
            }
            // ----------------------------------------------------
            // Mode C: Standard low obstacle crest fold
            // ----------------------------------------------------
            else {
                double progress = std::clamp(state_timer_ / 0.35, 0.0, 1.0);
                double target_q_r_start = computeRearClimbTargetAngle(active_h);
                double q_r = (1.0 - progress) * target_q_r_start + progress * nominal_r;
                ecc_overrides_(RL) = q_r;
                ecc_overrides_(RR) = q_r;

                double advance_vx = 0.35;
                wheel_speed_overrides_.setConstant(advance_vx / params_.wheel_radius);
                forward_vel_override_ = advance_vx;

                double effective_vx = (std::abs(body_vx) > 0.02) ? std::abs(body_vx) : advance_vx;
                travel_distance_ += effective_vx * dt;

                if (progress >= 1.0 || state_timer_ >= 0.45) {
                    travel_distance_ = 0.0;
                    transitionTo(ClimbState::REAR_PLACED);
                }
            }
            break;
        }

        // ============================================================
        // 4b. REAR_POST_CLIMB_RESET: 200mm only — Autonomous Ground-Rolling Reset
        //     After 4-wheel lift, rear legs are in forward stance (-5.63 rad = +37 deg).
        //     Smoothly sweep rear legs through the bottom (-74.5 deg minor arc) from -5.63 to -6.93 rad (-0.65 rad).
        //     Vehicle advances at 0.25 m/s, perfectly countering the 206mm rear axle retreat
        //     so rear wheels roll with nearly ZERO relative ground sliding!
        // ============================================================
        case ClimbState::REAR_POST_CLIMB_RESET: {
            override_mask_.fill(true);
            wheel_override_mask_.fill(true);

            double reset_duration = 0.80;
            double progress = std::clamp(state_timer_ / reset_duration, 0.0, 1.0);

            // Front legs subtly compensate for the 35mm height variation when rear legs pass vertical (0 rad):
            // When rear leg is at 0 rad (progress = 0.5), front leg slightly lowers to keep body level
            double q_f = 0.65 - 0.22 * std::sin(M_PI * progress);
            ecc_overrides_(FL) = q_f;
            ecc_overrides_(FR) = q_f;

            // Rear legs sweep continuously through bottom from -5.63 rad to -6.93 rad (-2*pi - 0.65 rad)
            double q_r = (1.0 - progress) * (-5.63) + progress * (-6.93);
            ecc_overrides_(RL) = q_r;
            ecc_overrides_(RR) = q_r;

            // Drive forward during reset: front wheels pull at 0.25 m/s, rear wheels roll at 0.12 m/s
            double reset_vx = 0.25;
            forward_vel_override_ = reset_vx;
            wheel_speed_overrides_(FL) = reset_vx / params_.wheel_radius;
            wheel_speed_overrides_(FR) = reset_vx / params_.wheel_radius;
            wheel_speed_overrides_(RL) = 0.12 / params_.wheel_radius;
            wheel_speed_overrides_(RR) = 0.12 / params_.wheel_radius;

            double effective_vx = (std::abs(body_vx) > 0.02) ? std::abs(body_vx) : reset_vx;
            travel_distance_ += effective_vx * dt;

            if (progress >= 1.0 || state_timer_ >= 0.85) {
                // Instantly re-base rear overrides from -6.93 to principal nominal angle -0.65:
                // delta is exactly 2*pi, so normalize_angle in CAD delta calculation evaluates to 0.0!
                // Zero jump, zero jerk, perfectly aligned with balance controller!
                landing_rear_q_ = -0.65;
                ecc_overrides_(RL) = -0.65;
                ecc_overrides_(RR) = -0.65;
                ecc_overrides_(FL) = 0.65;
                ecc_overrides_(FR) = 0.65;
                front_target_angles_.setConstant(0.65);
                travel_distance_ = 0.0;
                transitionTo(ClimbState::REAR_PLACED);
            }
            break;
        }

        // ============================================================
        // 5. REAR_PLACED: All wheels on plateau; advance 0.25m into plateau then return to IDLE
        // ============================================================
        case ClimbState::REAR_PLACED: {
            override_mask_.fill(true);
            double q_f = 0.65;
            double q_r = -0.65; // Standard OUTWARD cruising stance for all obstacles

            ecc_overrides_(FL) = q_f;
            ecc_overrides_(FR) = q_f;
            ecc_overrides_(RL) = q_r;
            ecc_overrides_(RR) = q_r;

            forward_vel_override_ = params_.forward_drive_speed;
            wheel_override_mask_.fill(true);
            wheel_speed_overrides_.setConstant(params_.forward_drive_speed / params_.wheel_radius);

            double effective_vx = (std::abs(body_vx) > 0.02) ? std::abs(body_vx) : params_.forward_drive_speed;
            travel_distance_ += effective_vx * dt;

            // Advance by at least params_.settling_distance (0.25m) so rear wheels are deeply positioned on plateau
            bool distance_settled = travel_distance_ >= params_.settling_distance;
            bool time_settled = state_timer_ >= params_.settling_duration;

            if ((distance_settled && state_timer_ >= 0.40) || time_settled) {
                blend_out_timer_ = 0.0;
                blend_out_weight_ = 1.0;
                cooldown_timer_ = params_.cooldown_duration;
                has_climbed_ = true;
                transitionTo(ClimbState::IDLE);
            }
            break;
        }
    }

    // Blend-out fade back to balance controller
    if (state_ == ClimbState::IDLE && blend_out_weight_ > 0.0) {
        blend_out_timer_ += dt;
        double progress = blend_out_timer_ / params_.blend_out_duration;
        blend_out_weight_ = std::max(0.0, 1.0 - progress);

        if (blend_out_weight_ > 0.0) {
            override_mask_.fill(true);
        } else {
            override_mask_.fill(false);
            active_lift_mask_.fill(false);
        }
    }
}

std::array<bool, NUM_LEGS> TaskController::getContactValid() const {
    std::array<bool, NUM_LEGS> valid = {true, true, true, true};
    if (state_ == ClimbState::FRONT_WALL_CLIMB) {
        valid[FL] = false;
        valid[FR] = false;
    } else if (state_ == ClimbState::REAR_WALL_CLIMB || state_ == ClimbState::REAR_VAULT_LOOP) {
        valid[RL] = false;
        valid[RR] = false;
    } else if (state_ == ClimbState::REAR_CREST_FOLD && snap_kick_active_) {
        valid[RL] = false;
        valid[RR] = false;
    }
    return valid;
}

}  // namespace tasks
}  // namespace robot_control
