# Robot Control Workspace - 6-DOF Robotic Arm

## Overview

This is a comprehensive 6-DOF robotic arm simulation and control stack built on **ROS 2 Jazzy**. The system provides a complete robotics pipeline including robot modeling (URDF/Xacro), physics simulation (Gazebo), ros2_control hardware interfaces, position controllers, inverse kinematics solvers, trajectory planning and execution, visualization (RViz), and an interactive GUI for end-effector control.

### 🤖 System Capabilities

- **✅ Full Physics Simulation**: Gazebo integration with ros2_control
- **✅ Real-time Control**: Position-controlled 6-DOF arm with hardware interfaces
- **✅ Inverse Kinematics**: Both Python (PyKDL) and C++ (Orocos KDL) implementations
- **✅ Trajectory Planning**: Smooth motion planning with quintic interpolation
- **✅ Interactive Control**: GUI for end-effector pose control and joint manipulation
- **✅ Data Logging**: CSV logging of joint states and end-effector positions
- **✅ Visualization**: RViz integration with real-time robot state display

### 📐 Robot Configuration

**Joint Chain (6-DOF):**
```
Base_Revolute-1 → Arm-1_Revolute-2 → Arm-2_Revolute-3 → Arm-3_Revolute-4 → Arm-4_Revolute-5 → Arm-5_Revolute-6
```

**Kinematic Chain:**
```
world → base_link → Base → Arm-1 → Arm-2 → Arm-3 → Arm-4 → Arm-5 → End-Coupler-v1
```Control Workspace (ROS 2 + ros2_control + Gazebo + RViz + IK + Trajectories)

### Overview

Kikobot is a 6‑DOF robotic arm simulation and control stack built on ROS 2. It includes a complete pipeline: robot model (URDF/Xacro), physics simulation (Gazebo via ros_gz), ros2_control hardware, controllers, joint command nodes, TF2, inverse kinematics (Python PyKDL and C++ Orocos KDL), trajectory planning/execution, RViz visualization, and a lightweight Tkinter GUI for interactive end‑effector motions. It also logs joint/EE data to CSV.

Joint order used across the stack:

```
['Base_Revolute-1', 'Arm-1_Revolute-2', 'Arm-2_Revolute-3', 'Arm-3_Revolute-4', 'Arm-4_Revolute-5', 'Arm-5_Revolute-6']
```

### Package roles and technologies

- arm_description
  - Role: Robot model, URDF/Xacro, RViz config, and launch orchestration for Gazebo + controllers + RViz.
  - Technologies: `urdf`, `xacro`, `robot_state_publisher`, `ros_gz_sim` (Gazebo), `ros_gz_bridge`, `gz_ros2_control`, `controller_manager`, `rviz2`.
  - How they’re used/implemented:
    - `urdf/xacro`: Defines links/joints, `<ros2_control>` interfaces, and Gazebo plugin.
    - `gz_ros2_control` plugin connects simulated hardware to ROS 2 controller manager; parameters point to the YAML under `arm_controller`.
    - `ros_gz_sim` launches Gazebo; `ros_gz_bridge` bridges `/clock`.
    - `robot_state_publisher` publishes TF tree from `robot_description`.
    - Launch file `arm_description/launch/arm_launch.py` sets `GZ_SIM_RESOURCE_PATH`, spawns the robot, bridges clock, spawns the controllers, and opens RViz.

- arm_controller
  - Role: ros2_control controller configuration (YAML) and spawner launch.
  - Technologies: `controller_manager`, `position_controllers/JointGroupPositionController`, `joint_state_broadcaster`.
  - Implementation:
    - `config/arm_controllers.yaml` defines controller manager params and exposes:
      - `/arm_controller/commands` as `std_msgs/Float64MultiArray` for 6 joint positions (position interface).
      - `/joint_states` via `joint_state_broadcaster`.
    - `launch/controllers.launch.py` spawns JSB then arm controller.

- arm_motion
  - Role: Example motion node publishing sinusoidal joint positions; TF2‑based EE trail markers; CSV logging.
  - Technologies: `rclpy`, `tf2_ros`, `visualization_msgs/MarkerArray`, `sensor_msgs/JointState`, `std_msgs/Float64MultiArray`, optional `pandas/numpy/matplotlib`.
  - Implementation:
    - Publishes to `/arm_controller/commands` at 20 Hz.
    - Subscribes `/joint_states`, looks up `base_link -> End-Coupler-v1` with TF2, publishes RViz markers, logs CSV to `test_datas/` on Ctrl+C.

- arm_kinematics
  - Role: IK and trajectory planning services, trajectory executor, demo publisher, and Tkinter EE GUI.
  - Technologies: `rclpy`, `python3-numpy`, `PyKDL` (`python3-pykdl`), `kdl_parser_py`, `trajectory_msgs`, optional `python3-ruckig`.
  - Implementation:
    - IK service (`/compute_ik`, Python): builds KDL chain from the URDF (via `xacro`), solves with `ChainIkSolverPos_LMA`.
    - Trajectory planner service (`/plan_joint_trajectory`): time‑parameterizes joint motion with quintic time scaling (Ruckig optional); returns `trajectory_msgs/JointTrajectory`.
    - Trajectory executor: subscribes to `planned_trajectory`, interpolates in time, publishes `/arm_controller/commands` as `Float64MultiArray`.
    - `gui_ee.py`: Tkinter GUI to call IK, plan, and publish trajectory.

- arm_pykdl
  - Role: C++ IK service alternative compatible with `arm_kinematics/srv/ComputeIK` for faster IK.
  - Technologies: `rclcpp`, `orocos_kdl`, `kdl_parser`, `ament_index_cpp`.
  - Implementation:
    - Node `ik_service_cpp` on `/compute_ik` uses Orocos KDL (`ChainIkSolverPos_LMA` with multiple seed strategies; Newton‑Raphson fallback). Accepts `base_link`/`tip_link` overrides and can read `robot_description` param or run `xacro`.

### Core capabilities and how they work

- Control all the joints
  - Topic: `/arm_controller/commands` (`std_msgs/Float64MultiArray`), 6 values in the joint order above.
  - Controller: `position_controllers/JointGroupPositionController` configured in `arm_controller/config/arm_controllers.yaml`.

- Compute inverse kinematics
  - Services: `/compute_ik` (Python PyKDL in `arm_kinematics`) and `/compute_ik` (C++ Orocos KDL in `arm_pykdl`).
  - Request fields: `joint_names[]`, `seed_positions[]`, `geometry_msgs/PoseStamped target` (frame `base_link`), optional `base_link`, `tip_link`.
  - Response: `success`, `message`, `solution_positions[]`.

- Move the end effector to a desired pose via ROS 2 topic/message
  - Compute IK to joint targets using `/compute_ik`, then publish positions directly to `/arm_controller/commands` (`Float64MultiArray`), or plan a trajectory and use the executor to stream commands smoothly.

- PyKDL for IK
  - `arm_kinematics/src/ik_service.py` uses `PyKDL` with `kdl_parser_py` to parse URDF (via `xacro`) and solve with `ChainIkSolverPos_LMA`.

- Trajectory planning and motion
  - Planner: `arm_kinematics/src/trajectory_service.py` computes quintic time‑scaling positions/velocities/accelerations over a duration (est. from vmax/amax or provided `target_time`). Optional Ruckig support if installed.
  - Execution: `arm_kinematics/src/trajectory_executor.py` time‑interpolates and publishes `/arm_controller/commands`.

- Visualization using RViz
  - `rviz2` is launched by `arm_description/launch/arm_launch.py` with `config.rviz` for the robot model and fixed frame `world`.
  - `arm_motion` publishes a `MarkerArray` EE trail in `base_link`.

- Physics simulation using Gazebo (gz_ros2_control)
  - Gazebo launched via `ros_gz_sim` with world `empty.sdf`.
  - The URDF embeds the `gz_ros2_control` plugin that loads the controller YAML; controller manager and controllers are spawned after the robot entity exists.

- TF2 for end‑effector transforms
  - `arm_motion` uses `tf2_ros` buffer/listener to lookup `base_link -> End-Coupler-v1` transforms for EE trail logging/markers.

- URDF/xacro for robot model
  - `arm_description/urdf/arm.urdf.xacro` defines links/joints, inertials, `<ros2_control>` joint interfaces, Gazebo plugin, and a fixed base option.

- Lightweight Tkinter GUI for interactive EE commands
  - `arm_kinematics/src/gui_ee.py` creates sliders for start/target EE poses, calls IK and trajectory plan services, and publishes `JointTrajectory` to `planned_trajectory` (executed by the executor node).

- Log a full stroke of joint angles and EE positions to CSV
  - `arm_motion/src/motion_node.py` records timestamps, desired/actual joint positions, velocities, efforts, and EE XYZ. On Ctrl+C, saves under `test_datas/motion_<timestamp>.csv` if `pandas/numpy/matplotlib` are present.

## How‑To Guide (exhaustive)

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

# IK and planning (Python stack)
sudo apt install python3-numpy python3-pykdl python3-kdl-parser-py

# Optional: GUI + data/plots
sudo apt install python3-tk python3-pandas python3-matplotlib python3-bottleneck

# Optional: Ruckig bindings (if available for your distro)
sudo apt install python3-ruckig || true
```

Notes:
- If using Conda, ensure `catkin_pkg rospkg empy` are available to the Python used by CMake/ament, or avoid Conda when building.
- Bottleneck warnings from pandas are silenced by installing `python3-bottleneck`.

### 2) Build the workspace

```bash
cd /home/san/Public/kikobot_control
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

### 3) Launch simulation (Gazebo + controllers + RViz)

Terminal 1:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/san/Public/kikobot_control && source install/setup.bash
ros2 launch arm_description arm_launch.py
```

Expect:
- Gazebo GUI opens with `empty.sdf`.
- `robot_state_publisher` publishes TF; ros2_control hardware initialized.
- Controllers loaded/active: `joint_state_broadcaster`, `arm_controller`.

Verify:

```bash
ros2 control list_controllers
ros2 topic list | grep -E 'joint_states|arm_controller'
```

### 4) Control joints directly (position interfaces)

Publish a 6‑value pose to `/arm_controller/commands`:

```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.3, -0.3, 0.2, 0.0, 0.0]}"
```

Inspect actual states:

```bash
ros2 topic echo /joint_states
```

### 5) Start sinusoidal motion + EE trail + CSV logging

Terminal 2:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash && source /home/san/Public/kikobot_control/install/setup.bash
ros2 run arm_motion motion_node.py
```

What it does:
- Publishes continuous sinusoids to `/arm_controller/commands`.
- Uses TF2 to get `base_link -> End-Coupler-v1`, publishes EE trail markers to `/visualization_marker_array` for RViz.
- On Ctrl+C, writes CSV to `test_datas/` with joint and EE data (if pandas/numpy/matplotlib installed).

### 6) Inverse kinematics options

- Python IK service (PyKDL):

```bash
ros2 run arm_kinematics ik_service.py
```

- C++ IK service (Orocos KDL):

```bash
ros2 run arm_pykdl ik_service_cpp
```

Call example (compute pose in `base_link` to `End-Coupler-v1`):

```bash
ros2 service call /compute_ik arm_kinematics/srv/ComputeIK "{
  joint_names: ['Base_Revolute-1','Arm-1_Revolute-2','Arm-2_Revolute-3','Arm-3_Revolute-4','Arm-4_Revolute-5','Arm-5_Revolute-6'],
  seed_positions: [0,0,0,0,0,0],
  base_link: 'base_link',
  tip_link: 'End-Coupler-v1',
  target: { header: {frame_id: 'base_link'}, pose: { position: {x: 0.20, y: 0.00, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} } }
}"
```

If `success: true`, take `solution_positions` and command the controller:

```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [<solution 6 values>]}"
```

### 7) Trajectory planning and execution

Start planner + executor:

```bash
ros2 run arm_kinematics trajectory_service.py
ros2 run arm_kinematics trajectory_executor.py
```

Plan via service, then publish the returned trajectory to `planned_trajectory` (demo node does this):

```bash
ros2 run arm_kinematics demo_plan_publish.py
```

Alternatively, call the planner directly:

```bash
ros2 service call /plan_joint_trajectory arm_kinematics/srv/PlanJointTrajectory "{
  joint_names: ['Base_Revolute-1','Arm-1_Revolute-2','Arm-2_Revolute-3','Arm-3_Revolute-4','Arm-4_Revolute-5','Arm-5_Revolute-6'],
  start_positions: [0,0,0,0,0,0],
  target_positions: [0.3,-0.4,0.5,-0.3,0.2,0.1],
  max_velocity: [1,1,1,1,1,1],
  max_acceleration: [2,2,2,2,2,2],
  max_jerk: 10.0,
  target_time: 3.0
}"
```

Notes:
- Planner uses quintic time scaling by default; if `python3-ruckig` is available the implementation can be switched to Ruckig.
- Executor streams `/arm_controller/commands` at high rate using time interpolation between trajectory points.

### 8) Tkinter EE GUI (interactive)

Launch full GUI setup (C++ IK + planner + executor + GUI):

```bash
ros2 launch arm_kinematics gui_ee.launch.py
```

Usage:
- Use sliders to set start/target EE poses (XYZ + RPY), then:
  - Compute IK: calls `/compute_ik` and shows joint solution.
  - Plan + Move: calls planner service, publishes `JointTrajectory` on `planned_trajectory` for the executor to stream to the controller.

### 9) RViz visualization

- RViz is started by `arm_description/launch/arm_launch.py` and uses `arm_description/rviz/config.rviz` (fixed frame `world`).
- Add the `MarkerArray` display on `/visualization_marker_array` to see the EE trail from `arm_motion`.

### 10) Gazebo simulation details

- `ros_gz_sim` launches Gazebo with `-v 3 -r empty.sdf`.
- The URDF includes:
  - `<ros2_control>` with per‑joint `position` command/state interfaces.
  - `<gazebo>` plugin `gz_ros2_control-system` with `<parameters>` pointing to `arm_controller/config/arm_controllers.yaml`.
- `ros_gz_bridge` bridges `/clock` to ROS to satisfy controller manager timing.

### 11) Data logging (CSV)

- On SIGINT, `arm_motion` saves `test_datas/motion_<timestamp>.csv` with columns:
  - `t`, `des_1..6`, `act_1..6`, `vel_1..6`, `eff_1..6`, `ee_x`, `ee_y`, `ee_z`.

### 12) Troubleshooting

- Controller manager says “No clock received” → ensure `ros_gz_bridge parameter_bridge` for `/clock` is running (it’s launched by `arm_launch.py`).
- Gazebo libEGL warnings on some systems are typically harmless if GUI renders; for headless use, consider running without GUI.
- If RViz shows no robot, ensure `robot_state_publisher` is running and `robot_description` is populated from `xacro`.

## Reference file map

- `arm_description/urdf/arm.urdf.xacro`: Robot model (links/joints, `<ros2_control>`, Gazebo plugin, meshes).
- `arm_description/launch/arm_launch.py`: Gazebo + robot spawn + controllers + RViz + clock bridge.
- `arm_controller/config/arm_controllers.yaml`: Controller manager and position controller configuration.
- `arm_motion/src/motion_node.py`: Sinusoidal joint driver, TF2, RViz markers, CSV logging.
- `arm_kinematics/src/ik_service.py`: Python IK service (PyKDL).
- `arm_pykdl/src/ik_service_cpp.cpp`: C++ IK service (Orocos KDL).
- `arm_kinematics/src/trajectory_service.py`: Trajectory planning service (quintic scaling; optional Ruckig).
- `arm_kinematics/src/trajectory_executor.py`: Streams joint commands from planned trajectories.
- `arm_kinematics/src/gui_ee.py`: Tkinter GUI for IK + planning + publish.
- `arm_kinematics/launch/kinematics.launch.py`: Starts planner, IK (Python), executor, and demo publisher.
- `arm_kinematics/launch/gui_ee.launch.py`: Starts IK (C++), planner, executor, GUI.

## Quick start

```bash
cd /home/san/Public/kikobot_control
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build && source install/setup.bash

# Terminal 1: Gazebo + RViz + controllers
ros2 launch arm_description arm_launch.py

# Terminal 2: sinusoid motion + EE logging
source /opt/ros/$ROS_DISTRO/setup.bash && source /home/san/Public/kikobot_control/install/setup.bash
ros2 run arm_motion motion_node.py
```


