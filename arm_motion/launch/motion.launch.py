from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_motion', executable='motion_node.py', output='screen'
        )
    ])
