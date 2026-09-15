import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

ecc_signs = [1.0, -1.0, 1.0, -1.0]
ecc_offsets = [-1.9003, -3.0526, 1.5320, -3.0473]

class Checker(Node):
    def __init__(self):
        super().__init__('checker')
        self.sub = self.create_subscription(JointState, '/mobed/joint_commands', self.cb, 10)
        
    def cb(self, msg):
        lf_cad = msg.position[1]
        rf_cad = msg.position[4]
        lb_cad = msg.position[7]
        rb_cad = msg.position[10]
        
        lf_math = (lf_cad - ecc_offsets[0]) / ecc_signs[0]
        rf_math = (rf_cad - ecc_offsets[1]) / ecc_signs[1]
        lb_math = (lb_cad - ecc_offsets[2]) / ecc_signs[2]
        rb_math = (rb_cad - ecc_offsets[3]) / ecc_signs[3]
        
        def norm(ang):
            while ang > math.pi: ang -= 2*math.pi
            while ang <= -math.pi: ang += 2*math.pi
            return ang
            
        print(f'LF Math: {norm(lf_math):.4f}')
        print(f'RF Math: {norm(rf_math):.4f}')
        print(f'LB Math: {norm(lb_math):.4f}')
        print(f'RB Math: {norm(rb_math):.4f}')
        
        rclpy.shutdown()

rclpy.init()
node = Checker()
rclpy.spin(node)
