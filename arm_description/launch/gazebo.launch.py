import os
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('arm_description')
    xacro_path = os.path.join(pkg_share, 'urdf', 'Kikobot.urdf.xacro')

    robot_description = ParameterValue(Command(['xacro ', xacro_path, ' is_ignition:=False']), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )),
        launch_arguments={'gz_args': '-v 3 -r empty.sdf'}.items(),
    )

    gz_spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'kikobot'],
    )

    return LaunchDescription([
        gz_launch,
        robot_state_publisher_node,
        gz_spawn,
    ])
