import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 核心控制大脑节点 (包含所有的 Plan 1-5 算法：EKF, FSM, 动力学等)
        Node(
            package='robot_control',
            executable='mobed_control_node',
            name='mobed_control_node',
            output='screen',
            emulate_tty=True, # 保证日志颜色输出
            parameters=[
                # 这里可以添加预留的 ROS 参数，例如：
                # {"use_sim_time": False}
            ]
        ),
        
        # 2. UDP 硬件/仿真接口桥接节点 (与宿主机 Mac MuJoCo 通信)
        Node(
            package='mobed_hardware_interface',
            executable='udp_interface',
            name='udp_interface',
            output='screen',
            emulate_tty=True
        )
        
        # 键盘遥控由宿主机独立浮动控制看板 mac_teleop_gui.py 经 UDP 自动无缝转发至 ROS 2 话题。
    ])
