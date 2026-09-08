"""
MuJoCo Simulation Runner (Mac Native)
======================================
Runs physics + native 3D rendering on macOS.
Communicates with Docker ROS2 via UDP.

Usage:  mjpython mac_mujoco_sim.py

Protocol:
  Receives: JSON {"cmd": [12 floats]}  on UDP port 9000
  Sends:    JSON {"state": [12 floats]} back to sender

Actuator ordering in robot.xml (must match):
  [0-3]  act_steer_FL/FR/RL/RR   (position servo)
  [4-7]  act_ecc_FL/FR/RL/RR     (position servo)
  [8-11] act_wheel_FL/FR/RL/RR   (velocity servo)
"""

import mujoco
import mujoco.viewer
import socket
import json
import time
import select

XML_PATH = 'local_mtgvakn0_9w389c_mjcf_stl/robot.xml'
UDP_PORT = 9000

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


def main():
    # ---- Load model ----
    print(f'Loading model: {XML_PATH}')
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    # ---- Apply home keyframe ----
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'home')
    if key_id != -1:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        # Set actuator ctrl to match initial joint positions so PD servos
        # don't yank joints back to zero on the first timestep
        positions = get_joint_positions(model, data)
        for i in range(8):          # first 8 actuators are position servos
            data.ctrl[i] = positions[i]
        print('Home keyframe applied and actuators initialised.')

    # ---- UDP socket ----
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.setblocking(False)
    print(f'UDP listening on :{UDP_PORT}')

    docker_addr = None  # learnt dynamically from first incoming packet

    # ---- Launch viewer ----
    print('Starting MuJoCo viewer …')
    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_render = time.time()

        while viewer.is_running():
            step_start = time.time()

            # ---- Receive commands from Docker ----
            try:
                ready, _, _ = select.select([sock], [], [], 0.0)
                if ready:
                    pkt, addr = sock.recvfrom(4096)
                    docker_addr = addr
                    msg = json.loads(pkt.decode('utf-8'))
                    if 'cmd' in msg and len(msg['cmd']) == 12:
                        for i in range(12):
                            data.ctrl[i] = msg['cmd'][i]
            except Exception:
                pass

            # ---- Step physics ----
            mujoco.mj_step(model, data)

            # ---- Send state back to Docker ----
            if docker_addr is not None:
                try:
                    state = get_joint_positions(model, data)
                    sock.sendto(json.dumps({'state': state}).encode('utf-8'),
                                docker_addr)
                except Exception:
                    pass

            # ---- Render at ~60 fps ----
            now = time.time()
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
