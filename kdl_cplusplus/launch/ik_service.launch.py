from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='kdl_cplusplus',
            executable='ik_service_cpp',
            name='ik_service_cpp',
            parameters=[{
                'robot_description': '',
                'base_link': 'base_link',
                'tip_link': 'End-Coupler-v1',
            }]
        )
    ])


