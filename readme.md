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
```

## 📦 Package Architecture

### `arm_description`
**Role**: Robot modeling, URDF definition, and simulation orchestration  
**Key Technologies**: `urdf`, `xacro`, `robot_state_publisher`, `ros_gz_sim`, `gz_ros2_control`

**Features:**
- Complete robot URDF/Xacro with physics properties and visual meshes
- ros2_control hardware interfaces for all 6 joints (position/velocity/effort)
- Gazebo physics simulation integration with gz_ros2_control plugin
- RViz configuration and launch coordination
- Automatic controller spawning and parameter management

**Key Files:**
- `urdf/arm.urdf.xacro` - Robot model with ros2_control interfaces
- `launch/arm_launch.py` - Main simulation launch (Gazebo + RViz + Controllers)
- `launch/rviz_launch.py` - RViz-only launch with joint state publisher GUI
- `rviz/config.rviz` - RViz configuration

### `arm_controller`
**Role**: ros2_control configuration and controller management  
**Key Technologies**: `controller_manager`, `position_controllers`, `joint_state_broadcaster`

**Features:**
- Position control for all 6 joints via `/arm_controller/commands`
- Joint state broadcasting via `/joint_states`
- Proper ros2_control parameter configuration
- Controller spawning and lifecycle management

**Key Files:**
- `config/arm_controllers.yaml` - Controller configuration
- `launch/controllers.launch.py` - Controller spawning launch file

### `arm_motion`
**Role**: Motion generation, visualization, and data logging  
**Key Technologies**: `rclpy`, `tf2_ros`, `visualization_msgs`, `sensor_msgs`

**Features:**
- Sinusoidal joint motion generation for testing
- End-effector trail visualization in RViz
- Real-time TF2-based end-effector tracking
- CSV data logging for motion analysis

**Key Files:**
- `src/motion_node.py` - Main motion control node

### `arm_kinematics`
**Role**: Inverse kinematics, trajectory planning, and execution  
**Key Technologies**: `PyKDL`, `kdl_parser_py`, `trajectory_msgs`, `rclpy`

**Features:**
- Python-based inverse kinematics service using PyKDL
- Trajectory planning with quintic interpolation
- Trajectory execution with real-time interpolation
- Interactive Tkinter GUI for end-effector control
- Motion logging and data analysis

**Key Files:**
- `src/ik_service.py` - Python IK service
- `src/trajectory_service.py` - Trajectory planning service
- `src/trajectory_executor.py` - Trajectory execution node
- `src/gui_ee.py` - End-effector control GUI
- `src/motion_logger.py` - Data logging service
- `launch/gui_ee.launch.py` - Complete GUI system launch
- `launch/kinematics.launch.py` - Kinematics services launch

### `arm_pykdl`
**Role**: High-performance C++ inverse kinematics  
**Key Technologies**: `rclcpp`, `orocos_kdl`, `kdl_parser`

**Features:**
- Fast C++ IK solver using Orocos KDL
- Multiple solving strategies with fallback mechanisms
- Compatible with Python IK service interface
- Optimized for real-time applications

**Key Files:**
- `src/ik_service_cpp.cpp` - C++ IK service implementation
- `scripts/ik_test_client.py` - IK service test client

## 🚀 Core Capabilities

### 1. Joint Position Control
**Topic**: `/arm_controller/commands` (`std_msgs/Float64MultiArray`)  
**Format**: 6 floating-point values in joint order  
**Example**:
```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.3, -0.3, 0.2, 0.0, 0.0]}"
```

### 2. Inverse Kinematics Service
**Service**: `/compute_ik` (`arm_kinematics/srv/ComputeIK`)  
**Available**: Python (PyKDL) and C++ (Orocos KDL) implementations  
**Input**: Target pose (position + orientation) in `base_link` frame  
**Output**: Joint angles solution

### 3. Trajectory Planning & Execution
**Planning Service**: `/plan_joint_trajectory` (`arm_kinematics/srv/PlanJointTrajectory`)  
**Execution Topic**: `planned_trajectory` (`trajectory_msgs/msg/JointTrajectory`)  
**Features**: Smooth quintic interpolation, velocity/acceleration limits

### 4. Real-time Monitoring
**Joint States**: `/joint_states` (position, velocity, effort)  
**TF2 Transforms**: Complete kinematic chain with real-time updates  
**Visualization**: RViz integration with robot model and end-effector trail

### 5. Interactive Control
**GUI**: Tkinter-based end-effector pose control  
**Manual Control**: Joint state publisher GUI for individual joint control  
**Data Logging**: CSV export of motion data and trajectories

## 🛠️ Installation & Setup

### Prerequisites (Ubuntu 22.04 + ROS 2 Jazzy)

```bash
# ROS 2 Jazzy Desktop Full
sudo apt update
sudo apt install ros-jazzy-desktop-full

# Gazebo and ros2_control packages
sudo apt install \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-position-controllers \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro

# Kinematics and planning dependencies
sudo apt install \
  python3-numpy \
  python3-pykdl \
  python3-kdl-parser-py

# GUI and data analysis (optional)
sudo apt install \
  python3-tk \
  python3-pandas \
  python3-matplotlib \
  python3-bottleneck

# Optional: Ruckig trajectory optimization
sudo apt install python3-ruckig || echo "Ruckig not available, using fallback"
```

### Build the Workspace

```bash
cd /home/san/Public/robot_control
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## 📋 Usage Guide

### 1. 🎮 Launch Full Simulation (Gazebo + RViz + Controllers)

**Terminal 1:**
```bash
cd /home/san/Public/robot_control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch arm_description arm_launch.py
```

**What this does:**
- Starts Gazebo physics simulation
- Spawns the robot model with ros2_control integration
- Launches RViz for visualization
- Automatically loads and activates controllers:
  - `joint_state_broadcaster` (publishes `/joint_states`)
  - `arm_controller` (accepts commands via `/arm_controller/commands`)

**Verification:**
```bash
# Check controllers are active
ros2 control list_controllers

# Check available topics
ros2 topic list | grep -E "(joint_states|arm_controller)"
```

### 2. 🎯 Launch RViz-Only Mode (No Physics)

```bash
ros2 launch arm_description rviz_launch.py
```

This launches RViz with a joint state publisher GUI for manual joint control without physics simulation.

### 3. 🧠 Launch Kinematics & Planning Services

**Option A: Python-based IK with full planning stack**
```bash
ros2 launch arm_kinematics kinematics.launch.py
```

**Option B: C++ IK with GUI interface**
```bash
ros2 launch arm_kinematics gui_ee.launch.py
```

### 4. 🎪 Test Joint Control

**Direct joint position command:**
```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.5, -0.3, 0.8, -0.2, 0.4, 0.1]}"
```

**Sinusoidal motion with visualization:**
```bash
# Terminal 2 (after launching simulation)
ros2 run arm_motion motion_node.py
```

### 5. 🎲 Test Inverse Kinematics

**Start IK service:**
```bash
ros2 run arm_kinematics ik_service.py
# OR for C++ version:
ros2 run arm_pykdl ik_service_cpp
```

**Call IK service:**
```bash
ros2 service call /compute_ik arm_kinematics/srv/ComputeIK "{
  joint_names: ['Base_Revolute-1','Arm-1_Revolute-2','Arm-2_Revolute-3','Arm-3_Revolute-4','Arm-4_Revolute-5','Arm-5_Revolute-6'],
  seed_positions: [0,0,0,0,0,0],
  base_link: 'base_link',
  tip_link: 'End-Coupler-v1',
  target: {
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.20, y: 0.00, z: 0.35},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

### 6. 🎛️ Use Interactive GUI

```bash
ros2 launch arm_kinematics gui_ee.launch.py
```

**GUI Features:**
- Sliders for start/target end-effector poses (X, Y, Z, Roll, Pitch, Yaw)
- "Compute IK" button to solve inverse kinematics
- "Plan + Move" button to execute smooth trajectories
- Real-time joint angle display

### 7. 📊 Trajectory Planning & Execution

**Start planning services:**
```bash
ros2 run arm_kinematics trajectory_service.py
ros2 run arm_kinematics trajectory_executor.py
```

**Plan a trajectory:**
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

### 8. 📈 Data Logging

Motion data is automatically logged to CSV files in the `log_data/` directory when using:
- `arm_motion/motion_node.py` (logs on Ctrl+C)
- `arm_kinematics/motion_logger.py` (continuous logging)

**CSV format includes:**
- Timestamps
- Desired vs actual joint positions
- Joint velocities and efforts
- End-effector XYZ positions

## 🔧 Technical Details

### ros2_control Configuration

The system uses ros2_control with the following setup:

**Hardware Interface**: `gz_ros2_control/GazeboSimSystem`
- All 6 joints provide position, velocity, and effort state interfaces
- All 6 joints accept position command interface

**Controllers**:
- `joint_state_broadcaster`: Publishes joint states to `/joint_states`
- `arm_controller`: Position controller accepting commands via `/arm_controller/commands`

### Kinematic Solver Details

**Python Implementation (PyKDL)**:
- Uses `ChainIkSolverPos_LMA` (Levenberg-Marquardt)
- Builds KDL chain from URDF via `kdl_parser_py`
- Supports custom base/tip links

**C++ Implementation (Orocos KDL)**:
- Multiple solving strategies with fallback
- Optimized for real-time performance
- Newton-Raphson fallback for robustness

### Trajectory Planning

**Algorithm**: Quintic polynomial interpolation
- Smooth position, velocity, and acceleration profiles
- Configurable velocity/acceleration/jerk limits
- Time-optimal trajectory scaling

**Optional Ruckig Integration**: If available, provides advanced trajectory optimization

## 🐛 Troubleshooting

### Common Issues

**1. Controllers fail to load**
```
ERROR: Failed loading controller joint_state_broadcaster
```
**Solution**: Ensure Gazebo simulation is running and ros2_control plugin is loaded:
```bash
# Check if controller manager is available
ros2 service list | grep controller_manager
```

**2. No robot in RViz**
```
No transforms available
```
**Solution**: Verify robot_state_publisher is running:
```bash
ros2 node list | grep robot_state_publisher
ros2 topic echo /robot_description --once
```

**3. Gazebo libEGL warnings**
```
libEGL warning: egl: failed to create dri2 screen
```
**Solution**: Usually harmless if GUI renders. For headless operation, use Gazebo server mode.

**4. Clock synchronization issues**
```
No clock received
```
**Solution**: Ensure ros_gz_bridge is running:
```bash
ros2 topic list | grep clock
```

### Verification Commands

```bash
# Check all nodes are running
ros2 node list

# Verify controllers
ros2 control list_controllers

# Check hardware interfaces
ros2 control list_hardware_interfaces

# Monitor joint states
ros2 topic echo /joint_states

# Test basic motion
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0]}"
```

## 📁 File Structure

```
robot_control/
├── src/
│   ├── arm_description/
│   │   ├── urdf/
│   │   │   ├── arm.urdf.xacro          # Robot model with ros2_control
│   │   │   └── meshes/                 # STL mesh files
│   │   ├── launch/
│   │   │   ├── arm_launch.py           # Main simulation launch
│   │   │   ├── rviz_launch.py          # RViz-only launch
│   │   │   └── gazebo.launch.py        # Gazebo-only launch
│   │   └── rviz/
│   │       └── config.rviz             # RViz configuration
│   ├── arm_controller/
│   │   ├── config/
│   │   │   └── arm_controllers.yaml    # Controller configuration
│   │   └── launch/
│   │       └── controllers.launch.py   # Controller spawning
│   ├── arm_motion/
│   │   └── src/
│   │       └── motion_node.py          # Motion generation & logging
│   ├── arm_kinematics/
│   │   ├── src/
│   │   │   ├── ik_service.py           # Python IK service
│   │   │   ├── trajectory_service.py   # Trajectory planning
│   │   │   ├── trajectory_executor.py  # Trajectory execution
│   │   │   ├── gui_ee.py              # End-effector GUI
│   │   │   └── motion_logger.py        # Data logging
│   │   ├── srv/
│   │   │   ├── ComputeIK.srv          # IK service definition
│   │   │   └── PlanJointTrajectory.srv # Planning service
│   │   └── launch/
│   │       ├── gui_ee.launch.py        # GUI system launch
│   │       └── kinematics.launch.py    # Kinematics services
│   └── arm_pykdl/
│       ├── src/
│       │   └── ik_service_cpp.cpp      # C++ IK service
│       └── scripts/
│           └── ik_test_client.py       # IK testing script
├── build/                              # Build artifacts
├── install/                            # Installation files
├── log/                               # ROS 2 logs
└── log_data/                          # Motion data logs
```

## 🎯 Quick Start Commands

**Complete System Launch:**
```bash
# Terminal 1: Full simulation
ros2 launch arm_description arm_launch.py

# Terminal 2: Interactive GUI (wait for simulation to start)
ros2 launch arm_kinematics gui_ee.launch.py

# Terminal 3: Test motion (optional)
ros2 run arm_motion motion_node.py
```

**RViz-Only Mode:**
```bash
ros2 launch arm_description rviz_launch.py
```

**Manual Control:**
```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.1, 0.2, -0.3, 0.4, -0.1, 0.2]}"
```

This comprehensive robot control system provides a solid foundation for robotics research, education, and development of advanced manipulation algorithms.