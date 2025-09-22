# Robot Control Workspace - 6-DOF Robotic Arm

## Overview

This is a comprehensive **6-DOF robotic arm simulation and control stack** built on **ROS 2 Jazzy**. The system provides a complete robotics pipeline including URDF modeling (Xacro), Gazebo physics simulation, ros2_control hardware interfaces, trajectory planning/execution, inverse kinematics, and comprehensive data logging.

### Key Features

- **Complete Robot Model**: Detailed URDF/Xacro model with accurate inertial properties and STL meshes
- **Physics Simulation**: Full Gazebo integration with ros2_control hardware interfaces
- **Trajectory Planning**: Quintic (5th-order) time-scaling planner with optional Ruckig integration
- **Inverse Kinematics**: Both Python (PyKDL) and C++ (Orocos KDL) IK services
- **Real-time Control**: 1000Hz controller update rate with 200Hz trajectory execution
- **Data Logging**: Comprehensive motion data logging with CSV export for analysis
- **GUI Interface**: Interactive end-effector control with pose sliders and numeric input
- **Visualization**: RViz integration with real-time robot state and trajectory visualization

## Architecture & Data Flow

```
[Gazebo Physics] ←→ [ros2_control] ←→ [Joint Controllers] ←→ [Trajectory Planner/Executor]
                         ↓                    ↓                         ↓
                 [joint_states topic]  [/arm_controller/commands]  [planned_trajectory]
                         ↓                                              ↓
                [Motion Logger] ←←←←←←←←←←←←←←←← [TF2 Tree] ←←←←← [robot_state_publisher]
                         ↓
                [CSV Data Export: log_data/stroke_NNNN.csv]
```

## Package Overview

### `arm_description` - Robot Model & Simulation
- **URDF/Xacro model**: Complete 6-DOF arm definition with meshes and inertial properties
- **Gazebo integration**: Physics simulation with gz_ros2_control hardware interface
- **Launch system**: Comprehensive launch file orchestrating simulation, controllers, and visualization
- **RViz configuration**: Pre-configured visualization setup

### `arm_controller` - ros2_control Configuration
- **Controller setup**: JointGroupPositionController for 6-DOF position control
- **Hardware interface**: gz_ros2_control integration for Gazebo simulation
- **Joint state broadcasting**: Real-time joint feedback via joint_state_broadcaster
- **1000Hz update rate**: High-frequency control loop for precise motion

### `arm_kinematics` - Trajectory Planning & Execution
- **Trajectory planner**: Quintic time-scaling with optional Ruckig integration
- **Motion execution**: 200Hz interpolated trajectory following
- **IK service**: Python PyKDL inverse kinematics service
- **GUI interface**: Interactive end-effector control with pose adjustment
- **Motion logging**: Real-time data capture with CSV export

### `kdl_cplusplus` - High-Performance IK Service
- **C++ implementation**: Orocos KDL-based inverse kinematics
- **Performance optimized**: Drop-in replacement for Python IK service
- **Chain caching**: Efficient KDL chain management for multiple base/tip combinations

### `arm_motion` - Demo & Testing
- **Sinusoidal motion**: Example motion patterns for testing
- **Visualization markers**: End-effector trail visualization
- **Development sandbox**: Testing utilities separate from main kinematics stack

## Robot Specifications

### Joint Configuration
The robot uses the following joint naming convention (must be consistent across all components):
```
['Base_Revolute-1', 'Arm-1_Revolute-2', 'Arm-2_Revolute-3', 
 'Arm-3_Revolute-4', 'Arm-4_Revolute-5', 'Arm-5_Revolute-6']
```

### Physical Properties
- **Base Link**: Fixed base with heavy inertia (5.0 kg)
- **Arm Segments**: Progressive mass distribution (1.07 to 2.52 kg)
- **End Effector**: Tool mounting point (`End-Coupler-v1`)
- **Materials**: STL meshes with accurate collision geometry

### Control Interfaces
- **Command Topic**: `/arm_controller/commands` (std_msgs/Float64MultiArray)
- **State Feedback**: `/joint_states` (sensor_msgs/JointState)
- **Update Rate**: 1000Hz controller, 200Hz trajectory execution
- **Position Control**: Joint position commands with velocity/effort feedback

## Quick Start

### System Startup (Two-Terminal Method)
```bash
# Terminal 1: Launch simulation, controllers, and visualization
ros2 launch arm_description arm_launch.py

# Terminal 2: Start kinematics stack with GUI control
ros2 launch arm_kinematics gui_ee.launch.py
```

### Build & Environment
```bash
# Build from workspace root
cd /home/san/Public/robot_control
colcon build
source install/setup.bash

# For development - rebuild specific package
colcon build --packages-select arm_kinematics
source install/setup.bash
```

### Basic Operation
1. **Launch System**: Use the two-terminal startup above
2. **GUI Control**: In the kinematics GUI, adjust target pose sliders
3. **Plan Motion**: Click "Compute IK" to solve inverse kinematics
4. **Execute**: Click "Plan + Move" to execute trajectory
5. **Monitor**: View motion in RViz and check CSV logs in `log_data/`

## Trajectory Planning Details

The system uses a **quintic (5th-order) polynomial time-scaling** approach for smooth trajectory generation:

### Mathematical Foundation
- **Scaling Function**: $s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5$ where $\tau \in [0,1]$
- **Derivatives**: Continuous position, velocity, and acceleration profiles
- **Sampling Rate**: 100Hz trajectory points for smooth execution

### Planning Pipeline
1. **Joint Space Path**: Straight-line interpolation between start and goal configurations
2. **Time Parameterization**: Duration estimation from velocity/acceleration limits
3. **Profile Generation**: Quintic scaling with full derivative information
4. **Trajectory Output**: Standard trajectory_msgs/JointTrajectory format

### Optional Ruckig Integration
- **Advanced Planning**: Jerk-limited trajectories with real-time replanning
- **Installation**: `pip install ruckig` or `apt install python3-ruckig`
- **Activation**: Enable in `trajectory_service.py` by modifying the conditional branch

## Inverse Kinematics

### Dual IK Services
1. **Python Service** (`arm_kinematics/ik_service.py`):
   - PyKDL-based LMA solver
   - Runtime URDF expansion via xacro
   - Dependency: `kdl_parser_py`, `python3-pykdl`

2. **C++ Service** (`kdl_cplusplus/ik_service_cpp`):
   - Orocos KDL implementation
   - Performance-optimized for real-time use
   - Chain caching for efficiency

### Usage Example
```bash
ros2 service call /compute_ik arm_kinematics/srv/ComputeIK "{
  joint_names: ['Base_Revolute-1','Arm-1_Revolute-2','Arm-2_Revolute-3','Arm-3_Revolute-4','Arm-4_Revolute-5','Arm-5_Revolute-6'],
  seed_positions: [0,0,0,0,0,0],
  target: { 
    header: { frame_id: 'base_link' }, 
    pose: { 
      position: {x: 0.30, y: 0.10, z: 0.25}, 
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} 
    } 
  },
  base_link: 'base_link',
  tip_link: 'End-Coupler-v1'
}"
```

## Data Logging & Analysis

**Access logged data:**
```bash
# View latest CSV file
ls -la log_data/
head -n 5 log_data/stroke_0001.csv

# Check data structure
python3 -c "
import pandas as pd
df = pd.read_csv('log_data/stroke_0001.csv')
print('Columns:', df.columns.tolist())
print('Shape:', df.shape)
print('Duration:', f'{df[\"t\"].max():.2f} seconds')
print('Sample rate:', f'{len(df)/df[\"t\"].max():.1f} Hz')
"
```

**Python analysis tools:**
```bash
# Install analysis dependencies (if not already installed)
sudo apt install python3-pandas python3-matplotlib

# Create analysis script
cat > analyze_motion.py << 'EOF'
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load and analyze motion data
df = pd.read_csv('log_data/stroke_0001.csv')
joints = ['base_revolute_1', 'arm1_revolute_2', 'arm2_revolute_3', 
          'arm3_revolute_4', 'arm4_revolute_5', 'arm5_revolute_6']

# Calculate tracking errors
tracking_errors = {}
for joint in joints:
    error = df[f'act_{joint}'] - df[f'des_{joint}']
    tracking_errors[joint] = {
        'rmse': np.sqrt(np.mean(error**2)),
        'max_error': np.max(np.abs(error)),
        'std': np.std(error)
    }

print("Joint Tracking Performance:")
for joint, metrics in tracking_errors.items():
    print(f"{joint}: RMSE={metrics['rmse']:.4f} rad, Max={metrics['max_error']:.4f} rad")

# Plot motion analysis
plt.figure(figsize=(15, 12))

# Joint positions
plt.subplot(2, 2, 1)
for joint in joints[:3]:  # First 3 joints
    plt.plot(df['t'], df[f'des_{joint}'], '--', label=f'{joint} (des)')
    plt.plot(df['t'], df[f'act_{joint}'], '-', label=f'{joint} (act)')
plt.xlabel('Time (s)')
plt.ylabel('Position (rad)')
plt.title('Joint Position Tracking (Joints 1-3)')
plt.legend()
plt.grid(True)

# Joint efforts
plt.subplot(2, 2, 2)
for joint in joints:
    plt.plot(df['t'], df[f'eff_{joint}'], label=f'{joint}')
plt.xlabel('Time (s)')
plt.ylabel('Effort (N⋅m)')
plt.title('Joint Efforts')
plt.legend()
plt.grid(True)

# End-effector trajectory
plt.subplot(2, 2, 3)
plt.plot(df['ee_act_x'], df['ee_act_y'], 'b-', linewidth=2, label='XY trajectory')
plt.scatter(df['ee_act_x'].iloc[0], df['ee_act_y'].iloc[0], color='green', s=100, label='Start')
plt.scatter(df['ee_act_x'].iloc[-1], df['ee_act_y'].iloc[-1], color='red', s=100, label='End')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('End-Effector XY Trajectory')
plt.legend()
plt.grid(True)
plt.axis('equal')

# Tracking errors
plt.subplot(2, 2, 4)
for joint in joints:
    error = df[f'act_{joint}'] - df[f'des_{joint}']
    plt.plot(df['t'], error, label=f'{joint}')
plt.xlabel('Time (s)')
plt.ylabel('Position Error (rad)')
plt.title('Joint Tracking Errors')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
EOF

# Run analysis
python3 analyze_motion.py
```

The system provides comprehensive motion data logging capabilities for analysis, debugging, and research purposes. Data is automatically captured and exported to CSV files in the `log_data/` directory.

#### **Logging Sources**

**Primary Logger**: `arm_kinematics/motion_logger.py`
- **Trigger**: Activated when trajectory messages are published to `planned_trajectory` topic
- **Sampling Rate**: 100 Hz (configurable via `sample_rate` parameter)
- **Data Source**: Real-time sensor data from `/joint_states` topic and TF2 transforms
- **Storage**: Automatic CSV export with sequential stroke numbering

**Secondary Logger**: `arm_motion/motion_node.py`
- **Trigger**: Manual logging on Ctrl+C termination
- **Sampling Rate**: Node execution frequency (~10 Hz)
- **Data Source**: Joint commands and TF2 end-effector poses
- **Usage**: Testing and debugging motion patterns

#### **CSV Data Format & Units**

Each trajectory execution creates a new CSV file named `stroke_NNNN.csv` with the following structure:

| Column Type | Column Name | Units | Description |
|-------------|-------------|-------|-------------|
| **Time** | `t` | seconds | Elapsed time since trajectory start (float64) |
| **Desired Positions** | `des_{joint_name}` | radians | Target joint positions from trajectory plan |
| **Actual Positions** | `act_{joint_name}` | radians | Measured joint positions from encoders |
| **Joint Efforts** | `eff_{joint_name}` | Newton⋅meters | Applied joint torques from actuators |
| **End-Effector Position** | `ee_act_x` | meters | Actual end-effector X position (via TF2) |
| **End-Effector Position** | `ee_act_y` | meters | Actual end-effector Y position (via TF2) |
| **End-Effector Position** | `ee_act_z` | meters | Actual end-effector Z position (via TF2) |

#### **Example CSV Structure**

```csv
t,des_base_revolute_1,des_arm1_revolute_2,des_arm2_revolute_3,des_arm3_revolute_4,des_arm4_revolute_5,des_arm5_revolute_6,act_base_revolute_1,act_arm1_revolute_2,act_arm2_revolute_3,act_arm3_revolute_4,act_arm4_revolute_5,act_arm5_revolute_6,eff_base_revolute_1,eff_arm1_revolute_2,eff_arm2_revolute_3,eff_arm3_revolute_4,eff_arm4_revolute_5,eff_arm5_revolute_6,ee_act_x,ee_act_y,ee_act_z
0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.125,0.000,0.435
0.010,0.002,0.003,0.001,0.001,0.000,0.001,0.001,0.002,0.001,0.001,0.000,0.000,0.125,0.087,0.034,0.012,0.003,0.001,0.126,0.001,0.436
```

#### **Data Acquisition Pipeline**

```mermaid
graph LR
    A[Joint Controllers] -->|commands| B[/arm_controller/commands]
    C[Gazebo Physics] -->|sensor data| D[/joint_states]
    D --> E[motion_logger.py]
    F[planned_trajectory] -->|trajectory| E
    G[TF2 Tree] -->|transforms| E
    E -->|CSV export| H[log_data/stroke_NNNN.csv]
```

**Data Flow:**
1. **Command Generation**: Trajectory planner generates smooth joint position commands
2. **Physics Simulation**: Gazebo simulates robot dynamics and sensor responses
3. **State Publishing**: `joint_state_broadcaster` publishes real-time joint states
4. **Transform Broadcasting**: `robot_state_publisher` maintains TF2 kinematic tree
5. **Data Acquisition**: `motion_logger` samples all data streams at 100 Hz
6. **Export**: Automatic CSV generation with timestamp synchronization

#### **Motion Logger Configuration**

```yaml
# Parameters (ROS 2 parameter server)
motion_logger:
  log_dir: "./log_data"           # Output directory (relative to workspace)
  sample_rate: 100.0              # Sampling frequency (Hz)
  ee_frame: "End-Coupler-v1"      # End-effector frame for TF lookup
  base_frame: "base_link"         # Base frame for end-effector pose
  enabled: true                   # Enable/disable logging
```

#### **Data Analysis Utilities**

**View CSV Data:**
```bash
# Quick inspection
head -n 5 log_data/stroke_0001.csv

# Column count verification
head -n 1 log_data/stroke_0001.csv | tr ',' '\n' | wc -l
```

**Python Analysis Example:**
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load trajectory data
df = pd.read_csv('log_data/stroke_0001.csv')

# Plot joint tracking performance
fig, axes = plt.subplots(2, 3, figsize=(15, 10))
joints = ['base_revolute_1', 'arm1_revolute_2', 'arm2_revolute_3', 
          'arm3_revolute_4', 'arm4_revolute_5', 'arm5_revolute_6']

for i, joint in enumerate(joints):
    ax = axes[i//3, i%3]
    ax.plot(df['t'], df[f'des_{joint}'], label='Desired', linewidth=2)
    ax.plot(df['t'], df[f'act_{joint}'], label='Actual', linestyle='--')
    ax.fill_between(df['t'], df[f'eff_{joint}'], alpha=0.3, label='Effort')
    ax.set_title(f'{joint} Tracking')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (rad) / Effort (N⋅m)')
    ax.legend()
    ax.grid(True)

plt.tight_layout()
plt.show()

# End-effector trajectory visualization
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(df['ee_act_x'], df['ee_act_y'], df['ee_act_z'], linewidth=3)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('End-Effector Trajectory')
plt.show()
```

#### **Logging Best Practices**

1. **Trajectory Execution**: Always use `trajectory_executor.py` for consistent logging trigger
2. **Data Integrity**: Verify CSV headers match expected joint names and count
3. **Storage Management**: Monitor `log_data/` directory size for long experiments
4. **Synchronization**: All data streams are timestamp-synchronized for accurate analysis
5. **Quality Assurance**: Check for NaN values indicating sensor/communication issues

#### **Troubleshooting Data Acquisition**

**Missing Data Issues:**
```bash
# Verify joint states are published
ros2 topic echo /joint_states --once

# Check TF2 transforms
ros2 run tf2_tools view_frames.py
ros2 run tf2_ros tf2_echo base_link End-Coupler-v1

# Monitor logging node
ros2 node info /motion_logger
```

**Common Data Quality Issues:**
- **NaN Values**: Indicates missing sensor data or TF lookup failures
- **Duplicate Timestamps**: Suggests sampling rate configuration issues
- **Missing Effort Data**: Verify Gazebo provides effort feedback in joint states
- **Inconsistent Joint Names**: Check URDF joint naming vs. controller configurationollers, inverse kinematics solvers, trajectory planning and execution, visualization (RViz), and an interactive GUI for end-effector control.

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
- **Comprehensive motion logging and data analysis with 100 Hz sampling**
- **CSV export with joint efforts, positions, and end-effector tracking**
- **Synchronized data acquisition from multiple sensor streams**

**Key Files:**
- `src/ik_service.py` - Python IK service
- `src/trajectory_service.py` - Trajectory planning service
- `src/trajectory_executor.py` - Trajectory execution node
- `src/gui_ee.py` - End-effector control GUI
- `src/motion_logger.py` - Real-time data acquisition and CSV logging
- `launch/gui_ee.launch.py` - Complete GUI system launch
- `launch/kinematics.launch.py` - Kinematics services launch

### `kdl_cplusplus`
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
**Motion logging and data analysis:** The system captures comprehensive motion data during trajectory execution, including joint tracking performance, effort requirements, and end-effector positioning accuracy. This data is essential for:
- **Performance Analysis**: Compare desired vs actual joint positions
- **Control Tuning**: Analyze tracking errors and system response
- **Force Analysis**: Monitor joint efforts and torque requirements  
- **Trajectory Optimization**: Validate smooth motion profiles
- **Research Applications**: Export data for machine learning or dynamics analysis

All logged data includes proper units (radians for joints, meters for positions, Newton⋅meters for efforts) and synchronized timestamps for precise analysis.

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
ros2 run kdl_cplusplus ik_service_cpp
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
ros2 run arm_kinematics motion_logger.py  # For automatic data logging
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

**Note**: Each trajectory execution automatically triggers data logging. The motion logger creates a new CSV file (`stroke_NNNN.csv`) containing complete motion data including joint efforts and end-effector positions.

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

### 10. � Data Visualization Tools

The workspace includes a built-in plot viewer for analyzing logged motion data:

**Using the Plot Log Viewer:**
```bash
# Run the integrated plot viewer
python3 src/plot_log_viewer.py

# Or specify a particular CSV file
python3 src/plot_log_viewer.py log_data/stroke_0003.csv
```

**Plot Viewer Features:**
- **Joint Position Tracking**: Overlays desired vs actual joint positions
- **Effort Analysis**: Displays joint torque requirements over time
- **3D End-Effector Trajectory**: Visualizes workspace utilization
- **Error Analysis**: Calculates and plots tracking performance metrics
- **Multi-Stroke Comparison**: Load and compare multiple trajectory executions
- **Export Capabilities**: Save plots as PNG/PDF for documentation

**Interpreting the Data:**

**Joint Position Plots:**
- **Solid lines**: Desired joint positions from trajectory planner
- **Dashed lines**: Actual joint positions from encoders
- **Good tracking**: Minimal gap between desired and actual
- **Units**: All joint positions in radians

**Joint Effort Plots:**
- **Magnitude**: Higher values indicate more demanding motions
- **Patterns**: Smooth curves suggest good trajectory planning
- **Spikes**: May indicate rapid accelerations or mechanical constraints
- **Units**: Newton⋅meters (N⋅m)

**End-Effector Trajectory:**
- **3D Path**: Shows actual workspace coverage
- **Smoothness**: Indicates quality of kinematic control
- **Accuracy**: Comparison with intended Cartesian path
- **Units**: Meters (m) in base_link coordinate frame

**Example Data Analysis:**
```bash
# Generate test data with known trajectory
ros2 launch arm_description arm_launch.py &
sleep 5  # Wait for simulation startup
ros2 launch arm_kinematics gui_ee.launch.py &
sleep 3  # Wait for services

# Execute a test trajectory via GUI or service call
ros2 service call /plan_joint_trajectory arm_kinematics/srv/PlanJointTrajectory "{
  joint_names: ['Base_Revolute-1','Arm-1_Revolute-2','Arm-2_Revolute-3','Arm-3_Revolute-4','Arm-4_Revolute-5','Arm-5_Revolute-6'],
  start_positions: [0,0,0,0,0,0],
  target_positions: [0.5,0.3,-0.4,0.2,0.1,0.3],
  max_velocity: [0.8,0.8,0.8,0.8,0.8,0.8],
  max_acceleration: [1.5,1.5,1.5,1.5,1.5,1.5],
  max_jerk: 5.0,
  target_time: 4.0
}"

# Analyze the resulting data
python3 src/plot_log_viewer.py log_data/stroke_0001.csv
```

This will generate comprehensive motion analysis plots showing tracking performance, effort requirements, and end-effector trajectory accuracy.

## 🐛 Troubleshooting

### Common Issues
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
│   │   │   └── motion_logger.py        # Real-time data acquisition
│   │   ├── srv/
│   │   │   ├── ComputeIK.srv          # IK service definition
│   │   │   └── PlanJointTrajectory.srv # Planning service
│   │   ├── launch/
│   │   │   ├── gui_ee.launch.py        # GUI system with logging
│   │   │   └── kinematics.launch.py    # Kinematics services
│   │   └── log_data/                   # CSV motion data output
│   └── kdl_cplusplus/
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
# Terminal 1: Full simulation with logging
ros2 launch arm_description arm_launch.py

# Terminal 2: Interactive GUI with automatic logging (wait for simulation to start)
ros2 launch arm_kinematics gui_ee.launch.py

# Terminal 3: Test motion and data collection (optional)
ros2 run arm_motion motion_node.py

# Terminal 4: Real-time data analysis (after generating some data)
python3 analyze_motion.py
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