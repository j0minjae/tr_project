from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rci_tr_controller',
            executable='ik_arm_control',
            name='ik_arm_control',
            output='screen'
        )
    ])
