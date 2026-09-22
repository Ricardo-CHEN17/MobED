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
        self.sock.bind(('0.0.0.0', 9001))
        self.sock.setblocking(False)

        # Target: Mac host from inside Docker
        self.mac_addr = ('host.docker.internal', 9000)

        # ---------- ROS2 pub / sub ----------
        self.pub_states = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu', 10)
        self.sub_cmds = self.create_subscription(
            JointState, '/mobed/joint_commands', self._cmd_callback, 10)

        # ---------- 100 Hz poll timer ----------
        self.poll_timer = self.create_timer(0.01, self._poll_udp)

        # ---------- Watchdog state ----------
        self.last_recv_time = time.monotonic()

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

        for i, name in enumerate(msg.name):
            canonical_idx = name_to_canonical_idx.get(name)
            if canonical_idx is None:
                continue
            if 'Wheel' in name:
                commands[canonical_idx] = msg.velocity[i]
            else:
                commands[canonical_idx] = msg.position[i]

        packet = json.dumps({'cmd': commands}).encode('utf-8')
        try:
            self.sock.sendto(packet, self.mac_addr)
        except OSError as e:
            self.get_logger().warn(f'UDP send failed: {e}', throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    # Upstream: Mac  -->  UDP JSON  -->  ROS2 /joint_states
    # ------------------------------------------------------------------
    def _poll_udp(self):
        """Drain all pending UDP packets; publish the latest state and IMU."""
        latest_state = None
        latest_imu = None

        # Drain — always consume everything so the buffer never grows stale
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                parsed = json.loads(data.decode('utf-8'))
                if 'state' in parsed and len(parsed['state']) == 12:
                    latest_state = parsed['state']
                if 'imu' in parsed and isinstance(parsed['imu'], dict):
                    latest_imu = parsed['imu']
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
            self._publish_joint_states(latest_state)
            if latest_imu is not None:
                self._publish_imu(latest_imu)
        elif (now - self.last_recv_time) > WATCHDOG_TIMEOUT:
            # Watchdog triggered — publish safe zeros so control node doesn't
            # drive blindly with stale data
            self.get_logger().warn(
                'No UDP data from Mac for >0.5 s — publishing zero state',
                throttle_duration_sec=2.0)
            self._publish_joint_states([0.0] * 12)

    def _publish_joint_states(self, values: list):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINT_NAMES)
        js.position = [float(v) for v in values]
        js.velocity = [0.0] * 12
        js.effort = [0.0] * 12
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
