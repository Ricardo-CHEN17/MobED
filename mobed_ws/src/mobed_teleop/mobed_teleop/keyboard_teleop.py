
"""
Keyboard Teleop Node for MobED (RC Style)
=========================================
Captures keyboard input and acts like an RC controller.
Holding a key sets the velocity to the target speed instantly.
Releasing it stops the robot.

Key bindings:
  W/S  : forward / backward
  A/D  : strafe left / right
  Q/E  : rotate CCW / CW
  U/J  : raise / lower height
  I/K  : tilt roll left / right
  O/L  : tilt pitch up / down
  
  1/2  : decrease / increase linear speed
  3/4  : decrease / increase angular speed

  SPACE: toggle E-Stop
  ESC  : quit
"""

import sys
import tty
import termios
import select
import time
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import MobEDCommand
from std_msgs.msg import Bool

# ---------- Defaults ----------
DEFAULT_LINEAR_SPEED = 0.5      # m/s
DEFAULT_ANGULAR_SPEED = 1.0     # rad/s
HEIGHT_STEP = 0.005
ROLL_STEP = 0.02
PITCH_STEP = 0.02

MAX_HEIGHT = 0.22
MIN_HEIGHT = 0.08
MAX_ROLL = 0.30
MAX_PITCH = 0.38
DEFAULT_HEIGHT = 0.18

PUBLISH_RATE = 50.0
THROTTLE_TIMEOUT = 0.35  # seconds before throttle resets to 0
STEER_TIMEOUT = 0.25     # seconds before steering auto-centers to 0
STRAFE_TIMEOUT = 0.25    # seconds before strafe resets to 0
SPIN_TIMEOUT = 0.25      # seconds before in-place spin resets to 0

HELP_TEXT = """
╔══════════════════════════════════════════╗
║        MobED RC Keyboard Teleop          ║
╠══════════════════════════════════════════╣
║  W/S (↑/↓): forward / backward (Throttle)║
║  A/D (←/→): turn left / right (Steering) ║
║  W+A / W+D: forward while turning        ║
║  Q/E      : in-place rotate CCW / CW     ║
║  Z/C      : strafe left / right          ║
║  U/J      : raise / lower height         ║
║  I/K      : roll left / right            ║
║  O/L      : pitch up / down              ║
║  1/2      : dec/inc linear speed         ║
║  3/4      : dec/inc angular speed        ║
║  SPACE    : toggle E-Stop                ║
║  ESC      : quit                         ║
╚══════════════════════════════════════════╝
"""

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub_cmd = self.create_publisher(MobEDCommand, '/mobed/command', 10)
        self.pub_estop = self.create_publisher(Bool, '/e_stop', 10)

        # Configurable target speeds
        self.target_linear = DEFAULT_LINEAR_SPEED
        self.target_angular = DEFAULT_ANGULAR_SPEED

        # Current commands
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.height = DEFAULT_HEIGHT
        self.roll = 0.0
        self.pitch = 0.0
        self.e_stop = False

        # Throttle memory state: 0 = neutral, 1 = forward, -1 = backward
        self.throttle_dir = 0
        # Steering memory state: 0 = straight, 1 = left, -1 = right
        self.steer_dir = 0

        # Independent channel timestamps for decoupled RC control
        self.last_key_time = 0.0
        self.last_throttle_time = 0.0
        self.last_steer_time = 0.0
        self.last_strafe_time = 0.0
        self.last_spin_time = 0.0
        self.in_spin_mode = False

        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self._publish)

    def _publish(self):
        now = time.time()

        # Deadman switch: if no key pressed for > 0.3s, stop all motion
        if now - self.last_key_time > 0.3:
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
            self.throttle_dir = 0
            self.in_spin_mode = False

        msg = MobEDCommand()
        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        msg.twist.angular.z = self.wz
        msg.body_height = self.height
        msg.body_roll = self.roll
        msg.body_pitch = self.pitch
        self.pub_cmd.publish(msg)

    def process_key(self, key: str):
        now = time.time()
        self.last_key_time = now

        # Throttle Channel: W (Straight Forward) / S (Straight Backward)
        if key == 'w':
            self.throttle_dir = 1
            self.in_spin_mode = False
            self.vx = self.target_linear
            self.vy = 0.0
            self.wz = 0.0  # Straight forward

        elif key == 's':
            self.throttle_dir = -1
            self.in_spin_mode = False
            self.vx = -self.target_linear
            self.vy = 0.0
            self.wz = 0.0  # Straight backward

        # Steering Channel: A (Turn Left while driving) / D (Turn Right while driving)
        elif key == 'a':
            self.in_spin_mode = False
            self.vy = 0.0
            # If reversing, curve in reverse; otherwise curve forward!
            if self.throttle_dir == -1:
                self.vx = -self.target_linear
            else:
                self.vx = self.target_linear
                self.throttle_dir = 1
            self.wz = self.target_angular  # Forward + Left curve!

        elif key == 'd':
            self.in_spin_mode = False
            self.vy = 0.0
            if self.throttle_dir == -1:
                self.vx = -self.target_linear
            else:
                self.vx = self.target_linear
                self.throttle_dir = 1
            self.wz = -self.target_angular  # Forward + Right curve!

        # In-Place Spin Channel: Q / E (Pure rotation, zero linear velocity)
        elif key == 'q':
            self.throttle_dir = 0
            self.vx = 0.0
            self.vy = 0.0
            self.wz = self.target_angular  # Pure in-place CCW spin
            self.in_spin_mode = True

        elif key == 'e':
            self.throttle_dir = 0
            self.vx = 0.0
            self.vy = 0.0
            self.wz = -self.target_angular  # Pure in-place CW spin
            self.in_spin_mode = True

        # Strafe Channel: Z / C (Pure lateral movement)
        elif key == 'z':
            self.throttle_dir = 0
            self.vx = 0.0
            self.vy = self.target_linear  # Pure left strafe
            self.wz = 0.0
            self.in_spin_mode = False

        elif key == 'c':
            self.throttle_dir = 0
            self.vx = 0.0
            self.vy = -self.target_linear  # Pure right strafe
            self.wz = 0.0
            self.in_spin_mode = False

        # Posture (Incremental)
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

        # Adjust Speeds
        elif key == '1':
            self.target_linear = max(self.target_linear - 0.1, 0.1)
        elif key == '2':
            self.target_linear = min(self.target_linear + 0.1, 2.0)
        elif key == '3':
            self.target_angular = max(self.target_angular - 0.2, 0.2)
        elif key == '4':
            self.target_angular = min(self.target_angular + 0.2, 4.0)

        # E-Stop
        elif key == ' ':
            self.e_stop = not self.e_stop
            estop_msg = Bool()
            estop_msg.data = self.e_stop
            self.pub_estop.publish(estop_msg)

def _get_key(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            # Check if this is an escape sequence (arrow keys)
            rlist_seq, _, _ = select.select([sys.stdin], [], [], 0.01)
            if rlist_seq:
                seq = sys.stdin.read(2)
                if seq == '[A':
                    key = 'w'  # Up arrow -> forward
                elif seq == '[B':
                    key = 's'  # Down arrow -> backward
                elif seq == '[C':
                    key = 'd'  # Right arrow -> right turn
                elif seq == '[D':
                    key = 'a'  # Left arrow -> left turn
            # Otherwise, standalone ESC stays as '\x1b'
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    print(HELP_TEXT)

    try:
        while rclpy.ok():
            key = _get_key(settings)
            if key == '\x1b':  # ESC
                print('\nExiting teleop.')
                break
            if key:
                node.process_key(key)
                
            # Print status bar unconditionally to show auto-stopping
            sys.stdout.write(
                f'\r[Speed L:{node.target_linear:.1f} A:{node.target_angular:.1f}] '
                f'Cmd Vx={node.vx:+.2f} Vy={node.vy:+.2f} Wz={node.wz:+.2f} '
                f'| H={node.height:.3f} R={node.roll:+.2f} P={node.pitch:+.2f}   '
            )
            sys.stdout.flush()

            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\nTerminal restored. Bye!')

if __name__ == '__main__':
    main()
