"""
Keyboard Teleop Node for MobED
================================
Captures keyboard input and publishes MobEDCommand messages.

Key bindings:
  W/S  : forward / backward   (linear.x)
  A/D  : strafe left / right   (linear.y)
  Q/E  : rotate CCW / CW       (angular.z)
  U/J  : raise / lower height
  I/K  : tilt roll left / right
  O/L  : tilt pitch up / down
  SPACE: toggle E-Stop
  ESC  : quit

Safety:
  - Terminal is set to raw mode; restored in finally block even on crash.
  - Deadman switch: velocities decay to zero when keys are released.
  - Hard limits on roll/pitch prevent impossible commands.
"""

import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import MobEDCommand
from std_msgs.msg import Bool

# ---------- Tunables ----------
LINEAR_STEP = 0.1       # m/s per key press
ANGULAR_STEP = 0.1      # rad/s per key press
HEIGHT_STEP = 0.005     # m per key press
ROLL_STEP = 0.02        # rad per key press
PITCH_STEP = 0.02       # rad per key press

MAX_LINEAR = 1.0        # m/s
MAX_ANGULAR = 1.5       # rad/s
MAX_ROLL = 0.30         # rad (~17 deg)
MAX_PITCH = 0.38        # rad (~22 deg)
MIN_HEIGHT = 0.08       # m
MAX_HEIGHT = 0.25       # m
DEFAULT_HEIGHT = 0.20   # m  (0.15 caused near-horizontal arms due to 4.5cm z_offset)

PUBLISH_RATE = 50.0     # Hz
DECAY_FACTOR = 0.85     # velocity decays each tick when no key held

HELP_TEXT = """
╔══════════════════════════════════════════╗
║        MobED Keyboard Teleop            ║
╠══════════════════════════════════════════╣
║  W/S    : forward / backward            ║
║  A/D    : strafe left / right           ║
║  Q/E    : rotate CCW / CW              ║
║  U/J    : raise / lower height          ║
║  I/K    : roll left / right             ║
║  O/L    : pitch up / down               ║
║  SPACE  : toggle E-Stop                 ║
║  ESC    : quit                          ║
╚══════════════════════════════════════════╝
"""


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub_cmd = self.create_publisher(MobEDCommand, '/mobed/command', 10)
        self.pub_estop = self.create_publisher(Bool, '/e_stop', 10)

        # State
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.height = DEFAULT_HEIGHT
        self.roll = 0.0
        self.pitch = 0.0
        self.e_stop = False

        # Publish timer
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self._publish)

    def _publish(self):
        # Decay velocities (deadman switch)
        self.vx *= DECAY_FACTOR
        self.vy *= DECAY_FACTOR
        self.wz *= DECAY_FACTOR

        # Zero-snap: prevent drift from tiny residuals
        if abs(self.vx) < 0.005:
            self.vx = 0.0
        if abs(self.vy) < 0.005:
            self.vy = 0.0
        if abs(self.wz) < 0.005:
            self.wz = 0.0

        msg = MobEDCommand()
        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        msg.twist.angular.z = self.wz
        msg.body_height = self.height
        msg.body_roll = self.roll
        msg.body_pitch = self.pitch
        self.pub_cmd.publish(msg)

    def process_key(self, key: str):
        if key == 'w':
            self.vx = min(self.vx + LINEAR_STEP, MAX_LINEAR)
        elif key == 's':
            self.vx = max(self.vx - LINEAR_STEP, -MAX_LINEAR)
        elif key == 'a':
            self.vy = min(self.vy + LINEAR_STEP, MAX_LINEAR)
        elif key == 'd':
            self.vy = max(self.vy - LINEAR_STEP, -MAX_LINEAR)
        elif key == 'q':
            self.wz = min(self.wz + ANGULAR_STEP, MAX_ANGULAR)
        elif key == 'e':
            self.wz = max(self.wz - ANGULAR_STEP, -MAX_ANGULAR)
        elif key == 'u':
            self.height = min(self.height + HEIGHT_STEP, MAX_HEIGHT)
        elif key == 'j':
            self.height = max(self.height - HEIGHT_STEP, MIN_HEIGHT)
        elif key == 'i':
            self.roll = min(self.roll + ROLL_STEP, MAX_ROLL)
        elif key == 'k':
            self.roll = max(self.roll - ROLL_STEP, -MAX_ROLL)
        elif key == 'o':
            self.pitch = min(self.pitch + PITCH_STEP, MAX_PITCH)
        elif key == 'l':
            self.pitch = max(self.pitch - PITCH_STEP, -MAX_PITCH)
        elif key == ' ':
            self.e_stop = not self.e_stop
            estop_msg = Bool()
            estop_msg.data = self.e_stop
            self.pub_estop.publish(estop_msg)
            state = 'ON' if self.e_stop else 'OFF'
            self.get_logger().warn(f'E-Stop toggled: {state}')


def _get_key(settings, timeout=0.05):
    """Non-blocking key read from stdin in raw mode."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    # Save terminal settings BEFORE entering raw mode
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    print(HELP_TEXT)
    print(f'Height={node.height:.3f}  Roll={node.roll:.2f}  Pitch={node.pitch:.2f}')

    try:
        while rclpy.ok():
            key = _get_key(settings)
            if key == '\x1b':  # ESC
                print('\nExiting teleop.')
                break
            if key:
                node.process_key(key)
                # Update status line
                sys.stdout.write(
                    f'\rVx={node.vx:+.2f} Vy={node.vy:+.2f} Wz={node.wz:+.2f} '
                    f'| H={node.height:.3f} R={node.roll:+.2f} P={node.pitch:+.2f} '
                    f'| E-Stop={"ON " if node.e_stop else "OFF"}  ')
                sys.stdout.flush()

            # Spin once to process timer callbacks
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        # CRITICAL: always restore terminal to normal mode
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\nTerminal restored. Bye!')


if __name__ == '__main__':
    main()
