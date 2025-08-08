import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('arm_controller')
    controllers_yaml = os.path.join(pkg_share, 'config', 'arm_controllers.yaml')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[controllers_yaml],
        output='screen',
    )

    arm_controller_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                    parameters=[controllers_yaml],
                    output='screen',
                ),
            ],
        )
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])
