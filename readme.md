## Kikobot ROS 2 Control + Gazebo (+ RViz, MoveIt) – End‑to‑End Guide

This guide documents how the workspace is wired to simulate Kikobot with ros2_control in Gazebo (ros_gz), visualize in RViz, and prepare for MoveIt integration. It is written to be reusable for other robots; adapt joint names, file paths, and masses as needed.

### Table of contents
- Concepts overview
- Install prerequisites
- Workspace layout and packages
- Modeling the robot (URDF/Xacro): links, joints, visuals, collisions, inertias
- ros2_control: hardware, interfaces, controllers
- Gazebo integration (ros_gz + gz_ros2_control)
- Launching everything (single orchestrator launch)
- Controller YAML
- Build and run
- Troubleshooting (real issues and fixes we hit)
- MoveIt integration (how to add, recommended controller)
- Adapting this template to another robot

---

### Concepts overview
- **Command interface**: values you send to hardware (e.g., position).
- **State interface**: values you read from hardware (position/velocity/effort).
- **Resource manager**: loads hardware from the `<ros2_control>` tag and exposes joint interfaces.
- **Controller manager**: loads controllers and connects them to the resource manager.
- **ros2_control**: framework for hardware + controllers.
- **ros2_controllers**: ready controllers (joint state broadcaster, position group, trajectory, etc.).
- **gz_ros2_control**: Gazebo system plugin that provides a ros2_control hardware in simulation.

---

### Install prerequisites (Ubuntu + ROS 2 Jazzy)
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
```

Optional plotting libs: `python3-pandas python3-matplotlib python3-numpy`.

Avoid Conda when building ROS workspaces or ensure its Python has `catkin_pkg rospkg empy` installed.

---

### Workspace layout and packages
```
kikobot_control/
  src/
    arm_description/        # Robot model, URDF/Xacro, RViz config, launches
    arm_controller/         # Controllers YAML and spawner launch
```

- `arm_description` is an ament_cmake package that installs `urdf/`, `rviz/`, `launch/`.
- `arm_controller` installs `config/arm_controllers.yaml` and `launch/controllers.launch.py`.

---

### Modeling the robot (URDF/Xacro)

Key file: `arm_description/urdf/Kikobot.urdf.xacro` (xacro is expanded at launch time so paths and params resolve correctly for both RViz and Gazebo).

Highlights you should adapt for a new robot:
- Stable link names and joint names. Example used here: `Base`, `Arm-1` … `Arm-5`, `End-Coupler-v1` and joints `Base_Revolute-1` … `Arm-5_Revolute-6`.
- Provide both visual and collision geometry; set non‑zero masses and reasonable inertias.
- Mesh paths that work in both RViz and Gazebo:
  - Use `file://$(find arm_description)/urdf/meshes/<file>.stl` in xacro (RViz can load these; Gazebo also resolves them after xacro expansion).

Root and base fixing (prevents toppling and removes KDL warning):
```xml
<link name="world"/>
<link name="base_link"/>
<joint name="world_to_base_link" type="fixed">
  <parent link="world"/>
  <child link="base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<joint name="base_link_to_Base" type="fixed">
  <parent link="base_link"/>
  <child link="Base"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

Tip: If the robot intersects the ground, raise it by changing `world_to_base_link` origin, e.g. `xyz="0 0 0.05"`.

Optional: For extra stability you can add Gazebo tags to the base link:
```xml
<gazebo reference="Base">
  <gravity>false</gravity>
  <kinematic>true</kinematic>
</gazebo>
```

Meshes example (xacro‑friendly):
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="file://$(find arm_description)/urdf/meshes/Arm-1.stl" scale="0.001 0.001 0.001"/>
  </geometry>
</visual>
```

---

### ros2_control block (hardware + interfaces)

Declare the simulation hardware and joint interfaces in the same xacro:
```xml
<ros2_control name="KikobotSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <joint name="Base_Revolute-1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <!-- repeat for each actuated joint -->
</ros2_control>
```

This exposes position commands and position/velocity/effort states per joint in simulation.

---

### Gazebo integration (ros_gz + gz_ros2_control)

Load the system plugin and point it to your controllers YAML:
```xml
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find arm_controller)/config/arm_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

The plugin creates the controller manager inside Gazebo and wires to the resource manager generated from `<ros2_control>`.

---

### Controller YAML (`arm_controller/config/arm_controllers.yaml`)

Used here (position group controller for quick tests):
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

For MoveIt use `joint_trajectory_controller/JointTrajectoryController` instead (see MoveIt section).

---

### Launching everything (single file)

`arm_description/launch/arm_launch.py` orchestrates Gazebo, robot state publisher, entity spawn, `/clock` bridge, controller spawners, and RViz.

Nodes started and responsibilities:
- `ros_gz_sim/gz_sim.launch.py`: brings up Gazebo (empty world).
- `robot_state_publisher`: publishes TF from `robot_description` (expanded via xacro).
- `ros_gz_sim/create`: spawns the robot entity from `robot_description`.
- `ros_gz_bridge/parameter_bridge`: bridges Gazebo `/clock` → ROS `/clock`.
- `controller_manager/spawner joint_state_broadcaster`: starts the broadcaster.
- `controller_manager/spawner arm_controller`: starts the arm controller after JSB becomes active.
- `rviz2`: opens the RViz view.

Snippet (key parts):
```python
robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model'), ' is_ignition:=False']), value_type=str)

gz_spawn_entity = Node(
  package='ros_gz_sim', executable='create', arguments=['-topic', 'robot_description', '-name', 'kikobot']
)

gz_ros2_bridge = Node(
  package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
)
```

We also set `GZ_SIM_RESOURCE_PATH` to the parent of the description share directory so Gazebo can resolve resources.

---

### Build and run
```bash
cd /home/san/Public/kikobot_control
source /opt/ros/$ROS_DISTRO/setup.bash   # system ROS
colcon build
source install/setup.bash

# One command to launch Gazebo, RViz, controllers, and spawn the robot
ros2 launch arm_description arm_launch.py

# Drive the arm (position group controller expects Float64MultiArray)
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.3, -0.3, 0.2, 0.0, 0.0]}"

# Inspect
ros2 control list_controllers
ros2 topic list | grep arm_controller
```

---

### Troubleshooting (issues we actually hit and fixes)

- "No module named catkin_pkg" during build:
  - Cause: Conda Python used by CMake/ament.
  - Fix: deactivate Conda; or `pip install catkin_pkg rospkg empy` in that env; clean `build/ install/ log/` and rebuild.

- Gazebo crashed when plugin tried to load YAML via `package://...`:
  - Fix: expand xacro and use `$(find arm_controller)/config/arm_controllers.yaml` so an absolute path reaches the plugin.

- Robot didn’t appear; frame errors like parent `Base` not found, or PoseRelativeToGraph:
  - Cause: missing/invalid root setup or zero-mass root.
  - Fix: root is `world` → `base_link` (fixed), then `base_link` → `Base` (fixed). Give `Base` non‑zero mass/inertia.

- RViz couldn’t load meshes from installed share path:
  - Fix: use `file://$(find arm_description)/...` in xacro for mesh filenames.

- Controller manager warnings "No clock received":
  - Fix: bridge Gazebo clock `/clock` using `ros_gz_bridge/parameter_bridge`.

---

### MoveIt integration (recommended setup)

1) Generate a MoveIt config via the MoveIt Setup Assistant, pointing to your main xacro (`Kikobot.urdf.xacro`).
2) In the MoveIt config package:
   - Use `joint_trajectory_controller/JointTrajectoryController` for each joint group.
   - Provide `ros2_controllers.yaml` like:
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
3) Ensure MoveIt planning groups match the controller joint lists.
4) When using MoveIt, swap the controller type in your sim YAML to trajectory controller to keep consistency.

---

### Dependencies to declare

- `arm_description/package.xml` should include:
  - `gz_ros2_control`, `ros2launch`, `robot_state_publisher`, `xacro`, `controller_manager`, `joint_state_publisher_gui`, `rviz2`, `ros_gz_sim` (exec_depends).

- `arm_controller/package.xml` should include:
  - `gz_ros2_control`, `controller_manager`, `position_controllers`, `joint_state_broadcaster` (or `joint_trajectory_controller`).

---

### Adapting this template to another robot
- Keep joint names consistent across URDF, controllers YAML, and (optionally) MoveIt.
- Provide valid masses/inertias; add damping to joints for stability.
- Use `file://$(find <pkg>)/...` for meshes in xacro so both RViz and Gazebo load them.
- Root pattern that works well: `world` → `base_link` (fixed) → `<your real base link>` (fixed).
- In simulation, use `gz_ros2_control` plugin and declare interfaces in the `<ros2_control>` tag.
- Start `ros_gz_bridge` clock bridge to remove timing warnings.

---

### Files of interest (this repo)
- `arm_description/urdf/Kikobot.urdf.xacro`: full robot model, `<ros2_control>`, Gazebo plugin, mesh paths.
- `arm_controller/config/arm_controllers.yaml`: controller manager and controllers.
- `arm_description/launch/arm_launch.py`: single entrypoint launching Gazebo, robot, controllers, RViz.

---

### Verifying the system
```bash
ros2 control list_controllers
ros2 topic list | grep -E 'joint_states|arm_controller'
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.3, -0.3, 0.2, 0.0, 0.0]}"
```

If controllers are already active, don’t reconfigure them; use `switch_controllers` or unload/load as needed.

---

### Notes
- RViz shows `/joint_states` from the `joint_state_broadcaster`.
- Gazebo entity name is `kikobot` (see launch spawn args).
- To lift the robot or offset pose, adjust `world_to_base_link` origin.

