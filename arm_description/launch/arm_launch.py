import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_description')
    model_default = os.path.join(pkg_share, 'urdf', 'arm.urdf.xacro')
    rviz_default = os.path.join(pkg_share, 'rviz', 'config.rviz')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=model_default,
        description='Absolute path to robot xacro file',
    )
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=rviz_default,
        description='Absolute path to rviz config file',
    )

    # Make Gazebo able to resolve package resources
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[str(Path(pkg_share).parent.resolve())],
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model'), ' is_ignition:=False']),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-v 3 -r empty.sdf'}.items(),
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'kikobot'],
    )

    # Bridge Gazebo clock to ROS to eliminate controller_manager warnings
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Controllers
    controllers_yaml = os.path.join(
        get_package_share_directory('arm_controller'), 'config', 'arm_controllers.yaml'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[controllers_yaml],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        parameters=[controllers_yaml],
        output='screen',
    )

    # Spawn controllers only after the entity is inserted, and chain spawns
    spawn_jsb_after_entity = RegisterEventHandler(
        OnProcessExit(target_action=gz_spawn_entity, on_exit=[joint_state_broadcaster_spawner])
    )
    spawn_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[arm_controller_spawner])
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        gazebo_resource_path,
        gazebo,
        robot_state_publisher_node,
        gz_spawn_entity,
        gz_ros2_bridge,
        spawn_jsb_after_entity,
        spawn_arm_after_jsb,
        rviz_node,
    ])
