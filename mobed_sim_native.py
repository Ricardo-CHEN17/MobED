#!/usr/bin/env python3
"""
MobED Native 120 FPS Simulation Runner (Direct GLFW + MuJoCo Rendering)
========================================================================
Runs high-fidelity physics + hardware-accelerated 3D rendering directly via GLFW.
Links in-memory to libmobed_control.dylib via ctypes (0 ms latency, 0 packet loss).

Key Advantages:
- True Simultaneous Multi-Key Holding: Hardware-level polling via glfw.get_key().
  Holding [W + A] or [W + D] simultaneously is 100% supported with 0 ms latency!
- ZERO MuJoCo Hotkey Conflicts: Bypasses MuJoCo's built-in Simulate GUI hotkeys.
  [Space] is a pure smooth brake and NEVER freezes physics; [A/E/J] NEVER trigger UI badges.
- 100% Clean 3D Viewport: Pure 3D simulation with zero intrusive text boxes or debug clutter.
- Real-time mouse orbit (left drag), pan (right drag), and zoom (scroll).

Usage:
    ./.venv/bin/python3 mobed_sim_native.py [--arena rmuc|playground|flat] [--climb auto|on|off]
"""

import os
import sys
import time
import argparse
import ctypes
import numpy as np
import glfw
import mujoco

# Import RoboMaster teleoperation planner
from teleop_commander import RoboMasterTeleopCommander


# -----------------------------------------------------------------------------
# C-Types Definitions matching mobed_controller_core.hpp
# -----------------------------------------------------------------------------
class SensorInputs(ctypes.Structure):
    _fields_ = [
        ('imu_accel',    ctypes.c_double * 3),
        ('imu_gyro',     ctypes.c_double * 3),
        ('imu_quat',     ctypes.c_double * 4),
        ('steer_pos',    ctypes.c_double * 4),
        ('steer_vel',    ctypes.c_double * 4),
        ('steer_effort', ctypes.c_double * 4),
        ('ecc_pos',      ctypes.c_double * 4),
        ('ecc_vel',      ctypes.c_double * 4),
        ('ecc_effort',   ctypes.c_double * 4),
        ('wheel_pos',    ctypes.c_double * 4),
        ('wheel_vel',    ctypes.c_double * 4),
        ('wheel_effort', ctypes.c_double * 4),
    ]

class CommandInputs(ctypes.Structure):
    _fields_ = [
        ('vx',            ctypes.c_double),
        ('vy',            ctypes.c_double),
        ('wz',            ctypes.c_double),
        ('target_height', ctypes.c_double),
        ('target_roll',   ctypes.c_double),
        ('target_pitch',  ctypes.c_double),
        ('posture_mode',  ctypes.c_int),
        ('climb_mode',    ctypes.c_int),
        ('e_stop',        ctypes.c_int),
    ]

class ActuatorOutputs(ctypes.Structure):
    _fields_ = [
        ('steer_pos',     ctypes.c_double * 4),
        ('ecc_pos',       ctypes.c_double * 4),
        ('wheel_vel',     ctypes.c_double * 4),
        ('steer_effort',  ctypes.c_double * 4),
        ('ecc_effort',    ctypes.c_double * 4),
        ('wheel_effort',  ctypes.c_double * 4),
        ('fsm_state',     ctypes.c_int),
        ('bank_angle',    ctypes.c_double),
        ('terrain_pitch', ctypes.c_double),
        ('terrain_roll',  ctypes.c_double),
    ]


# -----------------------------------------------------------------------------
# Arena & Joint Constants
# -----------------------------------------------------------------------------
PLAYGROUND_XML = 'models/robot_playground.xml'
FLAT_XML       = 'models/robot.xml'
RMUC_XML       = 'models/rmuc2026/robot_rmuc2026.xml'
DYLIB_PATH     = 'build_native/libmobed_control.dylib'

JOINT_NAMES = [
    'Steering_joint_LF', 'Steering_joint_RF', 'Steering_joint_LB', 'Steering_joint_RB',
    'Posture_control_joint_LF', 'Posture_control_joint_RF', 'Posture_control_joint_LB', 'Posture_control_joint_RB',
    'Wheel_joint_LF', 'Wheel_joint_RF', 'Wheel_joint_LB', 'Wheel_joint_RB',
]

FSM_STATE_NAMES = [
    "IDLE",
    "FRONT_WALL_CLIMB",
    "STRADDLE_LEVELING",
    "STRADDLE_HOLD",
    "REAR_WALL_CLIMB",
    "REAR_VAULT_LOOP",
    "REAR_TOUCHDOWN_SETTLE",
    "REAR_CREEP_ADVANCE",
    "REAR_CREST_FOLD",
    "REAR_POST_CLIMB_RESET",
    "REAR_PLACED"
]


# -----------------------------------------------------------------------------
# Interactive Mouse Camera Controller
# -----------------------------------------------------------------------------
class InteractiveCamera:
    """Standard orbit, pan, and zoom camera controller with chassis tracking."""
    def __init__(self, model, chassis_id):
        self.model = model
        self.chassis_id = chassis_id
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()

        # Configure initial chassis tracking camera
        if chassis_id != -1:
            self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.cam.trackbodyid = chassis_id
            self.cam.distance = 2.8
            self.cam.elevation = -18.0
            self.cam.azimuth = 145.0

        self.last_x = 0.0
        self.last_y = 0.0
        self.button_left = False
        self.button_middle = False
        self.button_right = False

    def on_mouse_button(self, window, button, action, mods):
        self.button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        self.button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        self.button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)
        xpos, ypos = glfw.get_cursor_pos(window)
        self.last_x = xpos
        self.last_y = ypos

    def on_cursor_pos(self, window, xpos, ypos):
        dx = xpos - self.last_x
        dy = ypos - self.last_y
        self.last_x = xpos
        self.last_y = ypos

        if not (self.button_left or self.button_middle or self.button_right):
            return

        w, h = glfw.get_framebuffer_size(window)
        if h <= 0:
            return

        mod_shift = (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or
                     glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)

        if self.button_right:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif self.button_left:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM

        mujoco.mjv_moveCamera(self.model, action, dx / h, dy / h, self.cam)

    def on_scroll(self, window, xoffset, yoffset):
        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, self.cam)


# -----------------------------------------------------------------------------
# Hardware-Polled Simultaneous Keyboard State
# -----------------------------------------------------------------------------
def get_hardware_keyboard_state(window) -> dict:
    """Directly polls GLFW hardware key states. 100% supports multi-key simultaneous holding."""
    # Forward / Backward
    fwd = (glfw.get_key(window, glfw.KEY_W) == glfw.PRESS or
           glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS)
    bwd = (glfw.get_key(window, glfw.KEY_S) == glfw.PRESS or
           glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS)

    # Turn / Strafe Left / Right
    left = (glfw.get_key(window, glfw.KEY_A) == glfw.PRESS or
            glfw.get_key(window, glfw.KEY_LEFT) == glfw.PRESS)
    rite = (glfw.get_key(window, glfw.KEY_D) == glfw.PRESS or
            glfw.get_key(window, glfw.KEY_RIGHT) == glfw.PRESS)

    # In-place Spin
    spin_l = (glfw.get_key(window, glfw.KEY_Q) == glfw.PRESS)
    spin_r = (glfw.get_key(window, glfw.KEY_E) == glfw.PRESS)

    # Smooth Brake
    brake = (glfw.get_key(window, glfw.KEY_SPACE) == glfw.PRESS or
             glfw.get_key(window, glfw.KEY_X) == glfw.PRESS)

    return {
        'fwd': fwd, 'bwd': bwd, 'left': left, 'rite': rite,
        'spin_l': spin_l, 'spin_r': spin_r, 'brake': brake
    }


# -----------------------------------------------------------------------------
# Main Simulation Runner
# -----------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='MobED 120 FPS Direct GLFW MuJoCo Simulator')
    parser.add_argument('--arena', type=str, default='rmuc', choices=['rmuc', 'playground', 'flat'],
                        help='Arena scene: rmuc (default), playground, flat')
    parser.add_argument('--keyframe', type=str, default='home',
                        help='Spawn location keyframe (home, home_red, home_blue, test_bumps, test_slope_23, test_bunker, test_flyslope)')
    parser.add_argument('--climb', type=str, default='auto', choices=['auto', 'on', 'off'],
                        help='Obstacle climb FSM mode: auto (default: on for playground, off for rmuc/flat), on, off')
    parser.add_argument('--model', type=str, default=None, help='Custom MJCF model path')
    args = parser.parse_args()

    # 1. Select scene XML
    if args.model:
        xml_path = args.model
    elif args.arena == 'flat':
        xml_path = FLAT_XML
    elif args.arena == 'playground':
        xml_path = PLAYGROUND_XML
    elif os.path.exists(RMUC_XML):
        xml_path = RMUC_XML
    else:
        xml_path = FLAT_XML

    # Climb FSM mode: disabled for flat & rmuc to guarantee high-speed cruise stability
    if args.climb == 'on':
        climb_mode_val = 1
    elif args.climb == 'off':
        climb_mode_val = 0
    else:
        climb_mode_val = 1 if args.arena == 'playground' else 0

    print(f'==> Loading MuJoCo Model: {xml_path}')
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 2. Apply keyframe
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, args.keyframe)
    if key_id != -1:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        for i, name in enumerate(JOINT_NAMES[:8]):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.ctrl[i] = float(data.qpos[model.jnt_qposadr[jid]])
        applied = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, key_id)
        print(f"==> Keyframe '{applied}' applied successfully.")
    mujoco.mj_forward(model, data)

    # 3. Load native C++ algorithm dylib
    if not os.path.exists(DYLIB_PATH):
        raise FileNotFoundError(f"Native library {DYLIB_PATH} not found! Run scripts/build_native_dylib.sh first.")

    print(f'==> Loading MobED C++ Control Core from {DYLIB_PATH} ...')
    lib = ctypes.CDLL(DYLIB_PATH)
    lib.mobed_create.restype = ctypes.c_void_p
    lib.mobed_destroy.argtypes = [ctypes.c_void_p]
    lib.mobed_reset.argtypes = [ctypes.c_void_p]
    lib.mobed_update.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(SensorInputs),
        ctypes.POINTER(CommandInputs),
        ctypes.c_double,
        ctypes.POINTER(ActuatorOutputs)
    ]

    controller = lib.mobed_create()
    if not controller:
        raise RuntimeError("Failed to allocate MobedControllerCore instance!")

    # 4. Initialize GLFW window on Main Thread
    if not glfw.init():
        raise RuntimeError("Failed to initialize GLFW!")

    # Request MSAA 4x for smooth anti-aliased rendering
    glfw.window_hint(glfw.SAMPLES, 4)

    window_width, window_height = 1280, 800
    window = glfw.create_window(
        window_width, window_height,
        f"MobED 120 FPS Simulation — [{args.arena.upper()}] (RoboMaster Full Multi-Key)",
        None, None
    )
    if not window:
        glfw.terminate()
        raise RuntimeError("Failed to create native GLFW window!")

    glfw.make_context_current(window)
    glfw.swap_interval(1)  # Enable VSync for rock-solid 60/120 FPS Metal sync

    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
    camera_ctrl = InteractiveCamera(model, chassis_id)

    # Register mouse callbacks
    glfw.set_mouse_button_callback(window, camera_ctrl.on_mouse_button)
    glfw.set_cursor_pos_callback(window, camera_ctrl.on_cursor_pos)
    glfw.set_scroll_callback(window, camera_ctrl.on_scroll)

    # Initialize MuJoCo visualization context
    scn = mujoco.MjvScene(model, maxgeom=2000)
    con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

    # 5. Initialize Teleop Commander
    teleop_commander = RoboMasterTeleopCommander(
        max_lin_vel=1.0,     # 1.0 m/s agile cruising
        max_strafe_vel=0.8,  # 0.8 m/s lateral strafe
        max_ang_vel=1.5,     # 1.5 rad/s turning
        spin_ang_vel=1.8     # 1.8 rad/s in-place spin
    )

    # Pre-allocate C structs
    s_in = SensorInputs()
    c_in = CommandInputs()
    c_in.target_height = 0.205
    c_in.target_roll   = 0.0
    c_in.target_pitch  = 0.0
    c_in.posture_mode  = 0
    c_in.climb_mode    = climb_mode_val
    c_in.e_stop        = 0

    out = ActuatorOutputs()

    # Joint lookups
    joint_qpos_adrs = []
    joint_dof_adrs = []
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        joint_qpos_adrs.append(model.jnt_qposadr[jid])
        joint_dof_adrs.append(model.jnt_dofadr[jid])

    g_world = np.array([0.0, 0.0, -9.81])
    prev_lin_vel = np.array(data.qvel[0:3], dtype=np.float64)

    climb_str = "ENABLED (Auto)" if climb_mode_val == 1 else "DISABLED (Flat Ground Cruise Lockout)"

    print("\n" + "="*70)
    print("  MobED Direct GLFW Native Simulation Ready (100% Conflict-Free)")
    print("="*70)
    print("  Controls (Directly in 3D Window, Full Multi-Key Support):")
    print("    [W / S]       Drive Forward / Backward (1.0 m/s)")
    print("    [W + A / D]   Agile Driving Curve + Motorcycle Lean (SIMULTANEOUS!)")
    print("    [A / D]       Pure Lateral Strafe (when stopped, 391mm Wide Stance)")
    print("    [Q / E]       In-place Spin CCW / CW (1.8 rad/s)")
    print("    [Space / X]   Brake / Soft Stop (Release keys to coast to stop)")
    print("    Mouse Drag    Left: Rotate around car | Right: Pan | Scroll: Zoom")
    print(f"  Climb FSM : {climb_str}")
    print("="*70 + "\n")

    # Timing variables
    last_sim_time = time.perf_counter()
    sim_time_debt = 0.0
    last_hud_time = 0.0
    control_substep_timer = 0.0
    CONTROL_DT = 0.010  # 100 Hz control loop
    PHYSICS_DT = model.opt.timestep

    try:
        while not glfw.window_should_close(window):
            # Process window events (mouse, keyboard, window resize)
            glfw.poll_events()

            now_perf = time.perf_counter()
            dt_wall = min(now_perf - last_sim_time, 0.05)
            last_sim_time = now_perf
            sim_time_debt += dt_wall

            # Step physics in 1:1 real-time accumulator
            while sim_time_debt >= PHYSICS_DT:
                control_substep_timer += PHYSICS_DT

                # ---------------- 100 Hz Control Update ----------------
                if control_substep_timer >= CONTROL_DT:
                    control_substep_timer -= CONTROL_DT

                    # 1. Read joints
                    for i in range(4):
                        s_in.steer_pos[i]    = float(data.qpos[joint_qpos_adrs[i]])
                        s_in.steer_vel[i]    = float(data.qvel[joint_dof_adrs[i]])
                        s_in.steer_effort[i] = float(data.actuator_force[i])

                        s_in.ecc_pos[i]      = float(data.qpos[joint_qpos_adrs[4 + i]])
                        s_in.ecc_vel[i]      = float(data.qvel[joint_dof_adrs[4 + i]])
                        s_in.ecc_effort[i]   = float(data.actuator_force[4 + i])

                        s_in.wheel_pos[i]    = float(data.qpos[joint_qpos_adrs[8 + i]])
                        s_in.wheel_vel[i]    = float(data.qvel[joint_dof_adrs[8 + i]])
                        s_in.wheel_effort[i] = float(data.actuator_force[8 + i])

                    # 2. Read IMU (finite-difference linear acceleration + gyro + quaternion)
                    chassis_xmat = data.xmat[chassis_id].reshape(3, 3)
                    curr_lin_vel = np.array(data.qvel[0:3], dtype=np.float64)
                    acc_world = (curr_lin_vel - prev_lin_vel) / CONTROL_DT
                    prev_lin_vel = curr_lin_vel.copy()
                    specific_force_body = chassis_xmat.T @ (acc_world - g_world)

                    s_in.imu_quat[0] = float(data.qpos[3])
                    s_in.imu_quat[1] = float(data.qpos[4])
                    s_in.imu_quat[2] = float(data.qpos[5])
                    s_in.imu_quat[3] = float(data.qpos[6])

                    s_in.imu_gyro[0] = float(data.qvel[3])
                    s_in.imu_gyro[1] = float(data.qvel[4])
                    s_in.imu_gyro[2] = float(data.qvel[5])

                    s_in.imu_accel[0] = float(specific_force_body[0])
                    s_in.imu_accel[1] = float(specific_force_body[1])
                    s_in.imu_accel[2] = float(specific_force_body[2])

                    # 3. Poll multi-key hardware state (W + A simultaneous supported!)
                    key_state = get_hardware_keyboard_state(window)
                    cmd_vel = teleop_commander.update(key_state, CONTROL_DT)

                    c_in.vx = float(cmd_vel[0])
                    c_in.vy = float(cmd_vel[1])
                    c_in.wz = float(cmd_vel[2])

                    # 4. Invoke native C++ control core
                    lib.mobed_update(controller, ctypes.byref(s_in), ctypes.byref(c_in), CONTROL_DT, ctypes.byref(out))

                    # 5. Apply commands to MuJoCo actuators
                    for i in range(4):
                        data.ctrl[i]     = out.steer_pos[i]
                        data.ctrl[4 + i] = out.ecc_pos[i]
                        data.ctrl[8 + i] = out.wheel_vel[i]

                # Step physics
                mujoco.mj_step(model, data)
                sim_time_debt -= PHYSICS_DT

            # ---------------- Render 3D Scene directly via OpenGL/Metal ----------------
            fb_w, fb_h = glfw.get_framebuffer_size(window)
            viewport = mujoco.MjrRect(0, 0, fb_w, fb_h)

            mujoco.mjv_updateScene(
                model, data, camera_ctrl.opt, None,
                camera_ctrl.cam, mujoco.mjtCatBit.mjCAT_ALL, scn
            )
            mujoco.mjr_render(viewport, scn, con)
            glfw.swap_buffers(window)

            # ---------------- Dynamic Single-Line Terminal Telemetry (10 Hz) ----------------
            now = time.time()
            if now - last_hud_time > 0.10:
                last_hud_time = now
                fsm_idx = out.fsm_state
                fsm_str = FSM_STATE_NAMES[fsm_idx] if 0 <= fsm_idx < len(FSM_STATE_NAMES) else f"STATE_{fsm_idx}"
                is_strafing = (abs(c_in.vy) > 0.05 and abs(c_in.vx) < 0.1)
                mode_tag = "STRAFE(391mm)" if is_strafing else "ROAD(Turn+Bank)"

                sys.stdout.write(
                    f"\r\033[K[MobED] {mode_tag} | Vx={c_in.vx:+.2f} m/s | Vy={c_in.vy:+.2f} m/s | Wz={c_in.wz:+.2f} rad/s | Bank={np.degrees(out.bank_angle):+.1f}° | FSM={fsm_str}"
                )
                sys.stdout.flush()

    finally:
        print("\n\n==> Cleaning up resources...")
        lib.mobed_destroy(controller)
        con.free()
        glfw.destroy_window(window)
        glfw.terminate()
        print("==> Simulation terminated cleanly.")


if __name__ == '__main__':
    main()
