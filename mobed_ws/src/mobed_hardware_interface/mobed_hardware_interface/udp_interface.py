"""
UDP Hardware Interface Node
===========================
Bridges ROS2 /mobed/joint_commands (from control node) to Mac MuJoCo via UDP,
and bridges Mac MuJoCo joint states back to ROS2 /joint_states.

Protocol:
  Downstream (Docker -> Mac): JSON {"cmd": [12 floats]}
  Upstream   (Mac -> Docker): JSON {"state": [12 floats]}

Joint ordering (both directions):
  [0-3]  Steering:  LF, RF, LB, RB  (position)
  [4-7]  Eccentric: LF, RF, LB, RB  (position)
  [8-11] Wheel:     LF, RF, LB, RB  (velocity)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from robot_interfaces.msg import MobEDCommand
from std_msgs.msg import Bool, Int32, Float64
import socket
import json
import time

# Canonical joint name ordering — must match Mac sim and control node
JOINT_NAMES = [
    'Steering_joint_LF', 'Steering_joint_RF', 'Steering_joint_LB', 'Steering_joint_RB',
    'Posture_control_joint_LF', 'Posture_control_joint_RF', 'Posture_control_joint_LB', 'Posture_control_joint_RB',
    'Wheel_joint_LF', 'Wheel_joint_RF', 'Wheel_joint_LB', 'Wheel_joint_RB',
]

WATCHDOG_TIMEOUT = 0.5  # seconds — if no UDP reply in this window, publish safe zeros


class UdpInterfaceNode(Node):
    def __init__(self):
        super().__init__('udp_interface')

        # ---------- UDP setup ----------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass
        self.sock.bind(('0.0.0.0', 9001))
        self.sock.setblocking(False)

        # Target: Mac host from inside Docker
        self.mac_addr = ('host.docker.internal', 9000)

        # ---------- ROS2 pub / sub ----------
        self.pub_states = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu', 10)
        self.sub_cmds = self.create_subscription(
            JointState, '/mobed/joint_commands', self._cmd_callback, 10)

        # ---------- ROS2 teleop publishers (from Mac MuJoCo HUD/Window) ----------
        self.pub_cmd = self.create_publisher(MobEDCommand, '/mobed/command', 10)
        self.pub_posture = self.create_publisher(Int32, '/mobed/posture_mode', 10)
        self.pub_curb_height = self.create_publisher(Float64, '/mobed/curb_height', 10)
        self.pub_climb_mode = self.create_publisher(Int32, '/mobed/climb_mode', 10)
        self.pub_estop = self.create_publisher(Bool, '/e_stop', 10)

        self.prev_posture = None
        self.prev_curb_height = None
        self.prev_climb_mode = None
        self.prev_estop = None

        # ---------- 100 Hz poll timer ----------
        self.poll_timer = self.create_timer(0.01, self._poll_udp)

        # ---------- Watchdog state ----------
        self.last_recv_time = time.monotonic()
        self.has_received_packet = False
        self.last_valid_positions = None

        self.get_logger().info(
            f'UDP Interface started — listening on :9001, sending to {self.mac_addr}')

    # ------------------------------------------------------------------
    # Downstream: ROS2 /mobed/joint_commands  -->  UDP JSON  -->  Mac
    # ------------------------------------------------------------------
    def _cmd_callback(self, msg: JointState):
        """Convert JointState from control node into a flat 12-element command array.

        IMPORTANT: The control node publishes joints in interleaved order
        (steer_LF, ecc_LF, wheel_LF, steer_RF, ...) but MuJoCo's data.ctrl
        expects the canonical JOINT_NAMES order (all steers, all eccs, all wheels).
        We MUST map by name, not by positional index.
        """
        if len(msg.name) != 12:
            return

        name_to_canonical_idx = {name: i for i, name in enumerate(JOINT_NAMES)}
        commands = [0.0] * 12
        torques = [0.0] * 12
        has_effort = (len(msg.effort) == len(msg.name))

        for i, name in enumerate(msg.name):
            canonical_idx = name_to_canonical_idx.get(name)
            if canonical_idx is None:
                continue
            if 'Wheel' in name:
                commands[canonical_idx] = msg.velocity[i]
            else:
                commands[canonical_idx] = msg.position[i]
            if has_effort:
                torques[canonical_idx] = msg.effort[i]

        payload = {'cmd': commands}
        if has_effort:
            payload['torque'] = torques

        packet = json.dumps(payload).encode('utf-8')
        try:
            self.sock.sendto(packet, self.mac_addr)
        except OSError as e:
            self.get_logger().warn(f'UDP send failed: {e}', throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    # Upstream: Mac  -->  UDP JSON  -->  ROS2 /joint_states
    # ------------------------------------------------------------------
    def _poll_udp(self):
        """Drain all pending UDP packets; publish the latest state, IMU, and teleop."""
        latest_state = None
        latest_velocity = None
        latest_effort = None
        latest_imu = None
        latest_teleop = None

        # Drain — always consume everything so the buffer never grows stale
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                if addr and addr[0] == '127.0.0.1':
                    self.mac_addr = addr
                else:
                    self.mac_addr = ('host.docker.internal', 9000)
                parsed = json.loads(data.decode('utf-8'))
                if 'state' in parsed and len(parsed['state']) == 12:
                    latest_state = parsed['state']
                if 'velocity' in parsed and len(parsed['velocity']) == 12:
                    latest_velocity = parsed['velocity']
                if 'effort' in parsed and len(parsed['effort']) == 12:
                    latest_effort = parsed['effort']
                if 'imu' in parsed and isinstance(parsed['imu'], dict):
                    latest_imu = parsed['imu']
                if 'teleop' in parsed and isinstance(parsed['teleop'], dict):
                    latest_teleop = parsed['teleop']
            except BlockingIOError:
                break  # nothing left in buffer
            except (json.JSONDecodeError, UnicodeDecodeError):
                # Dirty data — silently discard
                continue
            except OSError:
                break

        now = time.monotonic()

        if latest_state is not None:
            self.last_recv_time = now
            self.has_received_packet = True
            self.last_valid_positions = latest_state
            self._publish_joint_states(latest_state, latest_velocity, latest_effort)
            if latest_imu is not None:
                self._publish_imu(latest_imu)
        elif self.has_received_packet and (now - self.last_recv_time) > WATCHDOG_TIMEOUT:
            # Watchdog triggered — hold last valid positions and zero velocities so control node doesn't
            # drive blindly with stale motion, while avoiding injecting destructive 0.0 posture angles
            self.get_logger().warn(
                'No UDP data from Mac for >0.5 s — holding last joint positions and zeroing velocities',
                throttle_duration_sec=2.0)
            self._publish_joint_states(self.last_valid_positions, [0.0] * 12, [0.0] * 12)

        if latest_teleop is not None:
            self._publish_teleop(latest_teleop)

    def _publish_teleop(self, t: dict):
        """Publish teleop commands received from Mac MuJoCo viewer."""
        cmd = MobEDCommand()
        cmd.twist.linear.x = float(t.get('vx', 0.0))
        cmd.twist.linear.y = float(t.get('vy', 0.0))
        cmd.twist.angular.z = float(t.get('wz', 0.0))
        cmd.body_height = float(t.get('height', 0.205))
        cmd.body_roll = float(t.get('roll', 0.0))
        cmd.body_pitch = float(t.get('pitch', 0.0))
        self.pub_cmd.publish(cmd)

        posture = t.get('posture_mode')
        if posture is not None and posture != self.prev_posture:
            msg = Int32()
            msg.data = int(posture)
            self.pub_posture.publish(msg)
            self.prev_posture = posture
            self.get_logger().info(f'[Teleop] Forwarded Posture Mode -> {posture}')

        curb = t.get('curb_height')
        if curb is not None and (self.prev_curb_height is None or abs(curb - self.prev_curb_height) > 1e-4):
            msg = Float64()
            msg.data = float(curb)
            self.pub_curb_height.publish(msg)
            self.prev_curb_height = curb
            self.get_logger().info(f'[Teleop] Forwarded Curb Height -> {curb:.3f} m')

        climb = t.get('climb_mode')
        if climb is not None and climb != self.prev_climb_mode:
            msg = Int32()
            msg.data = int(climb)
            self.pub_climb_mode.publish(msg)
            self.prev_climb_mode = climb
            self.get_logger().info(f'[Teleop] Forwarded Climb Mode -> {climb}')

        estop = t.get('e_stop')
        if estop is not None and estop != self.prev_estop:
            msg = Bool()
            msg.data = bool(estop)
            self.pub_estop.publish(msg)
            self.prev_estop = estop
            self.get_logger().info(f'[Teleop] Forwarded E-Stop -> {estop}')

    def _publish_joint_states(self, positions: list, velocities: list = None, efforts: list = None):
        """Publish a JointState message with positions, velocities, and efforts.

        velocities and efforts fall back to all-zeros when not provided
        (watchdog path or old protocol compatibility).
        """
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINT_NAMES)
        js.position = [float(v) for v in positions]
        js.velocity = [float(v) for v in velocities] if velocities is not None else [0.0] * 12
        js.effort   = [float(v) for v in efforts]    if efforts   is not None else [0.0] * 12
        self.pub_states.publish(js)

    def _publish_imu(self, imu_dict: dict):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'

        quat = imu_dict.get('quat', [1.0, 0.0, 0.0, 0.0])  # [qw, qx, qy, qz]
        if len(quat) == 4:
            imu_msg.orientation.w = float(quat[0])
            imu_msg.orientation.x = float(quat[1])
            imu_msg.orientation.y = float(quat[2])
            imu_msg.orientation.z = float(quat[3])

        omega = imu_dict.get('omega', [0.0, 0.0, 0.0])
        if len(omega) == 3:
            imu_msg.angular_velocity.x = float(omega[0])
            imu_msg.angular_velocity.y = float(omega[1])
            imu_msg.angular_velocity.z = float(omega[2])

        acc = imu_dict.get('acc', [0.0, 0.0, 0.0])
        if len(acc) == 3:
            imu_msg.linear_acceleration.x = float(acc[0])
            imu_msg.linear_acceleration.y = float(acc[1])
            imu_msg.linear_acceleration.z = float(acc[2])

        self.pub_imu.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UdpInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
