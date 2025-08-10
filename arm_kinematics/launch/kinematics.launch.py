from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='arm_kinematics', executable='trajectory_service.py', output='screen'),
        Node(package='arm_kinematics', executable='ik_service.py', output='screen'),
        Node(package='arm_kinematics', executable='trajectory_executor.py', output='screen'),
        Node(package='arm_kinematics', executable='demo_plan_publish.py', output='screen'),
    ])


