import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. The Core Control Brain
        Node(
            package='robot_control',
            executable='mobed_control_node',
            name='mobed_control_node',
            output='screen'
        ),
        
        # 2. The UDP Hardware Interface Bridge
        Node(
            package='mobed_hardware_interface',
            executable='udp_interface',
            name='udp_interface',
            output='screen'
        )
    ])
