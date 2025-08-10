## Kikobot: From-Scratch Guide (ROS 2 + ros2_control + Gazebo + RViz + Motion)

This is a complete setup/run manual for this workspace. It starts from a clean machine and covers: URDF/Xacro, ros2_control, Gazebo, RViz, controller setup, and starting motion commands.

### TL;DR (quick start)
```bash
cd /home/san/Public/kikobot_control
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash

# Terminal 1: launch Gazebo + RViz + controllers
ros2 launch arm_description arm_launch.py

# Terminal 2: start sinusoid motion
source /opt/ros/$ROS_DISTRO/setup.bash && source /home/san/Public/kikobot_control/install/setup.bash
ros2 run arm_motion motion_node.py
```

You should see in Terminal 1 logs similar to the user run (Gazebo GUI, controller spawns, etc.). In Terminal 2 you should see:
```
[INFO] [...] [joint_position_publisher]: arm_motion: sinusoid command node started
```

---

### 1) Prerequisites (Ubuntu, ROS 2 Jazzy)
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-desktop-full
sudo apt install \
  ros-$ROS_DISTRO-ros-gz-sim ros-$ROS_DISTRO-ros-gz-bridge \
  ros-$ROS_DISTRO-gz-ros2-control \
  ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers \
  ros-$ROS_DISTRO-position-controllers ros-$ROS_DISTRO-joint-state-broadcaster \
  ros-$ROS_DISTRO-joint-trajectory-controller \
  ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui \
  ros-$ROS_DISTRO-xacro

# Optional (for motion node CSV/plots and to silence pandas warning):
sudo apt install python3-pandas python3-matplotlib python3-numpy python3-bottleneck
```

Notes:
- If you use Conda, ensure `catkin_pkg rospkg empy` are available to the Python used by CMake/ament, or avoid Conda when building.
- Pandas may warn about Bottleneck version. Install `python3-bottleneck` as above.

---

### 2) Workspace layout
```
kikobot_control/
  src/
    arm_description/        # Robot model, URDF/Xacro, RViz config, launches
    arm_controller/         # Controllers YAML and spawner launch
    arm_motion/             # Motion node + launch
```

Build:
```bash
cd /home/san/Public/kikobot_control
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build && source install/setup.bash
```

---

### 3) Robot URDF/Xacro (modeling and interfaces)

Key file: `arm_description/urdf/arm.urdf.xacro`. It defines:
Joint/link tree:
```
root [⚓] => /Base/
    Base_Revolute-1 [⚙+Z] => /Arm-1/
        Arm-1_Revolute-2 [⚙] => /Arm-2/
            Arm-2_Revolute-3 [⚙-Z] => /Arm-3/
                Arm-3_Revolute-4 [⚙] => /Arm-4/
                    Arm-4_Revolute-5 [⚙+Y] => /Arm-5/
                        Arm-5_Revolute-6 [⚙] => /End-Coupler-v1/
```
- Root frames and fixed joints (`world` → `base_link` → `Base`).
- Visual + collision meshes using share-resolved paths so both RViz and Gazebo load them.
- The ros2_control hardware and joint interfaces.
- The Gazebo `gz_ros2_control` plugin pointing to the controller YAML.

Root and base links:
```xml
<link name="world" />
<link name="base_link" />
<joint name="world_to_base_link" type="fixed">
  <parent link="world" />
  <child link="base_link" />
  <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>
<joint name="base_link_to_Base" type="fixed">
  <parent link="base_link" />
  <child link="Base" />
  <origin xyz="0 0 0" rpy="0 0 0" />
</joint>
```

Example mesh usage (xacro-friendly):
```xml
<geometry>
  <mesh filename="file://$(find arm_description)/urdf/meshes/Arm-1.stl" scale="0.001 0.001 0.001" />
</geometry>
```

ros2_control hardware and per-joint interfaces:
```xml
<ros2_control name="KikobotSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <joint name="Base_Revolute-1">
    <command_interface name="position" />
    <state_interface name="position" />
    <state_interface name="velocity" />
    <state_interface name="effort" />
  </joint>
  <!-- repeat for all 6 actuated joints -->
</ros2_control>
```

Gazebo plugin wiring ros2_control to the controller manager with our YAML:
```xml
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find arm_controller)/config/arm_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

Tip: If the robot intersects ground, raise `world_to_base_link` origin, e.g. `xyz="0 0 0.05"`. For a fixed base, you can set the base link non-dynamic in Gazebo:
```xml
<gazebo reference="Base">
  <gravity>false</gravity>
  <kinematic>true</kinematic>
</gazebo>
```

---

### 4) ros2_control controllers (YAML)

File: `arm_controller/config/arm_controllers.yaml` (full):
```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000
    use_sim_time: true

    arm_controller:
      type: position_controllers/JointGroupPositionController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    joints: [Base_Revolute-1, Arm-1_Revolute-2, Arm-2_Revolute-3, Arm-3_Revolute-4, Arm-4_Revolute-5, Arm-5_Revolute-6]
    command_interfaces: [position]
    state_interfaces: [position, velocity, effort]
```

This exposes `/arm_controller/commands` (Float64MultiArray of 6 positions) and publishes `/joint_states` via the broadcaster.

---

### 5) Launch orchestration (Gazebo + RViz + controllers)

File: `arm_description/launch/arm_launch.py` (full):
```python
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
```

This launch sets `GZ_SIM_RESOURCE_PATH` automatically so Gazebo can resolve package assets, spawns the robot from `robot_description`, bridges the `/clock`, then spawns `joint_state_broadcaster` and `arm_controller`, and finally opens RViz.

---

### 6) Run the simulation

Terminal 1:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/san/Public/kikobot_control && source install/setup.bash
ros2 launch arm_description arm_launch.py
```

Expect logs similar to:
- Gazebo GUI loading, world `empty.sdf`.
- ros2_control hardware `KikobotSystem` initialized.
- Controllers loaded and activated: `joint_state_broadcaster`, `arm_controller`.

Verify:
```bash
ros2 control list_controllers
ros2 topic list | grep -E 'joint_states|arm_controller'
```

Publish a manual pose (6 values in order of controller YAML):
```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.3, -0.3, 0.2, 0.0, 0.0]}"
```

---

### 7) Start motion (sinusoid command node)

Terminal 2:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash && source /home/san/Public/kikobot_control/install/setup.bash
ros2 run arm_motion motion_node.py
```

What it does:
- Publishes continuous sinusoidal joint positions to `/arm_controller/commands`.
- Subscribes to `/joint_states` and visualizes the end-effector trail in RViz as a `MarkerArray`.
- On Ctrl+C, if `pandas/numpy/matplotlib` are installed, writes a CSV under `test_datas/` with timestamps, desired/actual positions, velocities, efforts, and EE positions.

Optional: there is also a tiny launch file to start the node:
```python
# File: src/arm_motion/launch/motion.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_motion', executable='motion_node.py', output='screen'
        )
    ])
```
Run it as:
```bash
ros2 launch arm_motion motion.launch.py
```

---

### 8) Common warnings and fixes

- libEGL warning: "egl: failed to create dri2 screen" — occurs on some systems; usually harmless for Gazebo GUI if it still renders. If headless, run without GUI or use virtual GL.
- Controller manager "No clock received" — the launch bridges `/clock` via `ros_gz_bridge`; ensure that node is running.
- Pandas Bottleneck warning — install `python3-bottleneck` as shown above.

---

### 9) Switching to trajectory control (for MoveIt)

Replace the controller in the YAML with `joint_trajectory_controller/JointTrajectoryController` and keep joint lists consistent. Example snippet:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 250
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    joints: [Base_Revolute-1, Arm-1_Revolute-2, Arm-2_Revolute-3, Arm-3_Revolute-4, Arm-4_Revolute-5, Arm-5_Revolute-6]
    command_interfaces: [position]
    state_interfaces: [position]
    allow_partial_joints_goal: true
```

---

### 10) Package dependencies (sanity checklist)

- `arm_description/package.xml`: `urdf`, `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `ros2launch`, `gz_ros2_control`, `controller_manager`, `ros_gz_sim`.
- `arm_controller/package.xml`: `gz_ros2_control`, `controller_manager`, `position_controllers`, `joint_state_broadcaster` (or `joint_trajectory_controller`).

---

### 11) Files of interest
- `arm_description/urdf/arm.urdf.xacro` — full robot model, `<ros2_control>`, Gazebo plugin, mesh paths.
- `arm_controller/config/arm_controllers.yaml` — controller manager and controllers.
- `arm_description/launch/arm_launch.py` — launches Gazebo, robot, controllers, RViz.
- `arm_motion/src/motion_node.py` — sinusoidal position driver with RViz markers and optional CSV output.

---

### 12) Useful ROS 2 commands
```bash
# Controllers
ros2 control list_controllers
ros2 control list_hardware_interfaces

# Topics
ros2 topic list
ros2 topic echo /joint_states

# Frames/TF (RViz handles visualization)
ros2 run tf2_tools view_frames   # if installed
```

If controllers are already active, use `ros2 control switch_controllers` or unload/load instead of re-configuring.

