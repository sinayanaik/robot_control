from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='kdl_cplusplus',
            executable='ik_service_cpp',
            name='ik_service_cpp'
        ),
        Node(
            package='arm_kinematics',
            executable='trajectory_service.py',
            name='trajectory_service'
        ),
        Node(
            package='arm_kinematics',
            executable='trajectory_executor.py',
            name='trajectory_executor'
        ),
        Node(
            package='arm_kinematics',
            executable='gui_ee.py',
            name='gui_ee'
        ),
        Node(
            package='arm_kinematics',
            executable='motion_logger.py',
            name='motion_logger',
            parameters=[
                {'base_link': 'base_link'},
                {'tip_link': 'End-Coupler-v1'},
                {'log_dir': 'log_data'},
                {'enabled': False}
            ]
        ),
    ])


