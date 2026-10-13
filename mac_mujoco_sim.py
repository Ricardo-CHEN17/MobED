"""
MuJoCo Simulation Runner (Mac Native)
======================================
Runs physics + native 3D rendering on macOS.
Communicates with Docker ROS2 via UDP (port 9000).
Integrates with standalone floating GUI dashboard (mac_teleop_gui.py) via UDP (port 9002).

Usage:
    mjpython mac_mujoco_sim.py

Features:
- Completely decouples teleop keys from the MuJoCo 3D window to prevent
  any shortcut conflicts (A, D, C, 1-9, [, ], Space).
- Receives full teleoperation packets from mac_teleop_gui.py on UDP port 9002.
- Forwards teleop commands to Docker udp_interface.py at 100 Hz.
- Sends live simulation telemetry (height, roll, pitch, linear/angular velocity)
  back to mac_teleop_gui.py on UDP port 9003 for live HUD visualization.
"""

import mujoco
import mujoco.viewer
import socket
import json
import time
import select
import math
import numpy as np
import os
import sys
import argparse

PLAYGROUND_XML = 'local_mtgvakn0_9w389c_mjcf_stl/robot_playground.xml'
FLAT_XML = 'local_mtgvakn0_9w389c_mjcf_stl/robot.xml'
RMUC_XML = 'local_mtgvakn0_9w389c_mjcf_stl/rmuc2026/robot_rmuc2026.xml'
UDP_PORT = 9000           # Docker communication port
TELEOP_UDP_PORT = 9002    # Local teleop GUI listener port
TELEMETRY_PORT = 9003     # Telemetry target port for mac_teleop_gui.py

# Canonical joint names — same order as JOINT_NAMES in udp_interface.py
JOINT_NAMES = [
    'Steering_joint_LF', 'Steering_joint_RF', 'Steering_joint_LB', 'Steering_joint_RB',
    'Posture_control_joint_LF', 'Posture_control_joint_RF', 'Posture_control_joint_LB', 'Posture_control_joint_RB',
    'Wheel_joint_LF', 'Wheel_joint_RF', 'Wheel_joint_LB', 'Wheel_joint_RB',
]


def get_joint_positions(model, data):
    """Read current qpos for all 12 joints in canonical order."""
    positions = []
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        positions.append(float(data.qpos[model.jnt_qposadr[jid]]))
    return positions


def get_joint_velocities(model, data):
    """Read current qvel for all 12 joints in canonical order."""
    velocities = []
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        velocities.append(float(data.qvel[model.jnt_dofadr[jid]]))
    return velocities


def get_joint_efforts(model, data):
    """Read current actuator output forces for all 12 actuators in canonical order."""
    return [float(data.actuator_force[i]) for i in range(model.nu)]


class SimTeleopBridge:
    """Bridges commands from the external floating teleop dashboard (mac_teleop_gui.py)."""
    def __init__(self):
        # Commanded values
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.height = 0.205        # m (nominal Mode 1 cruising height, 37 deg outward)
        self.roll = 0.0            # rad
        self.pitch = 0.0           # rad
        self.posture_mode = 0      # 0: OUTWARD, 1: INWARD, 2: SPIN
        self.curb_height = 0.075   # m (adaptive curb height in [0.04m, 0.15m])
        self.climb_mode = 1        # 1: LOW_80MM, 2: HIGH_200MM
        self.e_stop = False

        self.last_update_time = 0.0
        self.is_connected = False
        self.last_hud_render = 0.0

    def update_from_dict(self, t: dict):
        """Update teleop states from received UDP packet."""
        self.vx = float(t.get('vx', self.vx))
        self.vy = float(t.get('vy', self.vy))
        self.wz = float(t.get('wz', self.wz))
        self.height = float(t.get('height', self.height))
        self.roll = float(t.get('roll', self.roll))
        self.pitch = float(t.get('pitch', self.pitch))
        self.posture_mode = int(t.get('posture_mode', self.posture_mode))
        self.curb_height = float(t.get('curb_height', self.curb_height))
        self.climb_mode = int(t.get('climb_mode', self.climb_mode))
        self.e_stop = bool(t.get('e_stop', self.e_stop))

        self.last_update_time = time.time()
        self.is_connected = True

    def check_connection(self):
        if time.time() - self.last_update_time > 1.5:
            self.is_connected = False
        return self.is_connected

    def render_hud(self, viewer):
        """Render clean, non-intrusive status overlay in MuJoCo viewer."""
        now = time.time()
        if now - self.last_hud_render < 0.05:  # 20 Hz
            return
        self.last_hud_render = now

        connected = self.check_connection()
        posture_names = {0: "OUTWARD (432mm)", 1: "INWARD (167mm)", 2: "SPIN (0-radius)"}
        climb_names = {0: "DISABLED", 1: "ADAPTIVE", 2: "ADAPTIVE"}

        status_text = "● GUI CONNECTED" if connected else "○ WAITING FOR GUI (mac_teleop_gui.py)"

        left_col = (
            "--- MobED Sim HUD ---\n"
            "Dashboard:\n"
            "Cmd Velocity:\n"
            "Chassis Posture:\n"
            "Curb Target:\n"
            "E-Stop State:"
        )

        right_col = (
            "\n"
            f"{status_text}\n"
            f"Vx={self.vx:+.2f} m/s, Wz={self.wz:+.2f} rad/s\n"
            f"H={self.height*1000:.0f}mm | {posture_names.get(self.posture_mode, 'UNKNOWN')}\n"
            f"{self.curb_height*1000:.0f}mm | CLIMB: {climb_names.get(self.climb_mode, 'ADAPTIVE')}\n"
            f"{'LOCKED (E-STOP)' if self.e_stop else 'READY'}"
        )

        viewer.set_texts([(
            mujoco.mjtFontScale.mjFONTSCALE_100,
            mujoco.mjtGridPos.mjGRID_TOPLEFT,
            left_col,
            right_col
        )])

    def get_teleop_dict(self):
        """Pack current teleop state into dictionary for UDP transmission to Docker."""
        return {
            'vx': self.vx,
            'vy': self.vy,
            'wz': self.wz,
            'height': self.height,
            'roll': self.roll,
            'pitch': self.pitch,
            'posture_mode': self.posture_mode,
            'curb_height': self.curb_height,
            'climb_mode': self.climb_mode,
            'e_stop': self.e_stop,
        }


def main():
    parser = argparse.ArgumentParser(description='MuJoCo Simulation Runner for MobED')
    parser.add_argument('--arena', type=str, default='rmuc', choices=['rmuc', 'playground', 'flat'],
                        help='Arena environment: rmuc (RMUC 2026), playground, flat (default: rmuc)')
    parser.add_argument('--flat', action='store_true', help='Use flat ground scene (alias for --arena flat)')
    parser.add_argument('--model', type=str, default=None, help='Custom MJCF model path')
    parser.add_argument('--keyframe', type=str, default='home',
                        help='Initial spawn keyframe (home, home_red, home_blue, test_bumps, test_slope_23, test_bunker, test_flyslope)')
    args = parser.parse_args()

    teleop_bridge = SimTeleopBridge()
    print('MuJoCo Passive 3D Simulation initialized (zero key conflicts).')
    print('Launch mac_teleop_gui.py in another terminal for floating control dashboard.')

    if args.model:
        xml_path = args.model
    elif args.flat or args.arena == 'flat':
        xml_path = FLAT_XML
    elif args.arena == 'playground':
        xml_path = PLAYGROUND_XML
    elif os.path.exists(RMUC_XML):
        xml_path = RMUC_XML
    elif os.path.exists(PLAYGROUND_XML):
        xml_path = PLAYGROUND_XML
    else:
        xml_path = FLAT_XML

    # ---- Load model ----
    print(f'Loading model: {xml_path}')
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # ---- Apply spawn keyframe ----
    key_target = args.keyframe
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, key_target)
    if key_id == -1 and key_target != 'home':
        print(f"Warning: keyframe '{key_target}' not found, falling back to 'home'")
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'home')
    if key_id != -1:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        positions = get_joint_positions(model, data)
        for i in range(8):          # first 8 actuators are position servos
            data.ctrl[i] = positions[i]
        applied_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, key_id)
        print(f"Keyframe '{applied_name}' applied and actuators initialized.")

    # Synchronize kinematics & dynamics state so xpos and sensors reflect keyframe
    mujoco.mj_forward(model, data)

    # ---- UDP socket for Docker ROS2 ----
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.setblocking(False)
    print(f'Docker UDP listening on :{UDP_PORT}')

    # ---- UDP socket for local Teleop GUI ----
    teleop_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        teleop_sock.bind(('0.0.0.0', TELEOP_UDP_PORT))
        teleop_sock.setblocking(False)
        print(f'Teleop GUI UDP listening on :{TELEOP_UDP_PORT}')
    except OSError as e:
        print(f'Warning: could not bind teleop port {TELEOP_UDP_PORT}: {e}')

    docker_addr = None
    last_telemetry_time = 0.0

    # ---- Launch passive viewer WITHOUT conflicting key_callback ----
    print('Starting MuJoCo viewer …')
    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_render = time.time()

        # Focus viewer camera directly on robot chassis with auto-tracking so it is immediately visible
        chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
        if chassis_id != -1:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = chassis_id
            viewer.cam.distance = 2.8
            viewer.cam.elevation = -18
            viewer.cam.azimuth = 145

        while viewer.is_running():
            step_start = time.time()

            # ---- Receive teleop commands from mac_teleop_gui ----
            try:
                ready_t, _, _ = select.select([teleop_sock], [], [], 0.0)
                if ready_t:
                    pkt_t, _ = teleop_sock.recvfrom(4096)
                    parsed_t = json.loads(pkt_t.decode('utf-8'))
                    if 'teleop' in parsed_t:
                        teleop_bridge.update_from_dict(parsed_t['teleop'])
            except Exception:
                pass

            # ---- Update HUD overlay ----
            teleop_bridge.render_hud(viewer)

            # ---- Receive joint commands from Docker ----
            try:
                ready, _, _ = select.select([sock], [], [], 0.0)
                if ready:
                    pkt, addr = sock.recvfrom(4096)
                    docker_addr = addr
                    msg = json.loads(pkt.decode('utf-8'))
                    if 'cmd' in msg and len(msg['cmd']) == 12:
                        for i in range(12):
                            data.ctrl[i] = msg['cmd'][i]
                    # Reset applied generalized forces
                    data.qfrc_applied[:] = 0.0
                    # Note: Actuators in MuJoCo model are <position kp="300" kv="30"> position servos.
                    # Bypassing qfrc_applied prevents dual-controller fighting and eliminates jitter/limit cycles.
            except Exception:
                pass

            # ---- Animate battlefield dynamic elements (Windmill & Outposts) ----
            wm_jnt = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'windmill_joint')
            if wm_jnt != -1:
                data.qvel[model.jnt_dofadr[wm_jnt]] = 0.8
            red_op = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'red_outpost_spin')
            if red_op != -1:
                data.qvel[model.jnt_dofadr[red_op]] = 0.4
            blue_op = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'blue_outpost_spin')
            if blue_op != -1:
                data.qvel[model.jnt_dofadr[blue_op]] = -0.4

            # ---- Step physics ----
            mujoco.mj_step(model, data)

            # ---- Send state back to Docker ----
            chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
            state_pos = get_joint_positions(model, data)
            state_vel = get_joint_velocities(model, data)
            state_eff = get_joint_efforts(model, data)

            chassis_xmat = data.xmat[chassis_id].reshape(3, 3)
            acc_world = np.array(data.cacc[chassis_id][3:6])
            g_world = np.array([0.0, 0.0, -9.81])
            specific_force_body = chassis_xmat.T @ (acc_world - g_world)

            imu_data = {
                'quat': [float(data.qpos[3]), float(data.qpos[4]),
                         float(data.qpos[5]), float(data.qpos[6])],  # [qw, qx, qy, qz]
                'omega': [float(data.qvel[3]), float(data.qvel[4]), float(data.qvel[5])],
                'acc': [float(specific_force_body[0]),
                        float(specific_force_body[1]),
                        float(specific_force_body[2])]
            }

            if docker_addr is not None:
                try:
                    payload = {
                        'state':    state_pos,   # joint positions  [12]
                        'velocity': state_vel,   # joint velocities [12]
                        'effort':   state_eff,   # actuator forces  [12]
                        'imu':      imu_data,
                        'teleop':   teleop_bridge.get_teleop_dict(),
                    }
                    sock.sendto(json.dumps(payload).encode('utf-8'), docker_addr)
                except Exception:
                    pass

            # ---- Send live telemetry back to mac_teleop_gui (:9003) ----
            now = time.time()
            if now - last_telemetry_time > 0.033:  # ~30 Hz
                last_telemetry_time = now
                try:
                    qpos_z = float(data.qpos[2])
                    r31 = chassis_xmat[2, 0]
                    r32 = chassis_xmat[2, 1]
                    r33 = chassis_xmat[2, 2]
                    act_roll = math.atan2(r32, r33)
                    act_pitch = math.atan2(-r31, math.sqrt(r32**2 + r33**2))
                    act_lin_vel = float(np.linalg.norm(data.qvel[0:2]))
                    act_ang_vel = float(data.qvel[5])

                    telem_pkt = {
                        'telemetry': {
                            'height': qpos_z,
                            'roll': act_roll,
                            'pitch': act_pitch,
                            'linear_vel': act_lin_vel,
                            'angular_vel': act_ang_vel,
                        }
                    }
                    teleop_sock.sendto(json.dumps(telem_pkt).encode('utf-8'), ('127.0.0.1', TELEMETRY_PORT))
                except Exception:
                    pass

            # ---- Render at ~60 fps ----
            if now - last_render > 0.016:
                viewer.sync()
                last_render = now

            # ---- Real-time pacing ----
            elapsed = time.time() - step_start
            sleep_time = model.opt.timestep - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


if __name__ == '__main__':
    main()
