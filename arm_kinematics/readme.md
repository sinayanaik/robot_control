# arm_kinematics - Trajectory Planning, Execution & Inverse Kinematics

## Overview

The `arm_kinematics` package provides comprehensive trajectory generation, execution, and inverse kinematics capabilities for the 6-DOF Kikobot robotic arm. This package implements quintic polynomial trajectory planning with optional Ruckig integration, high-frequency trajectory execution, PyKDL-based inverse kinematics, motion logging with CSV export, and an intuitive GUI interface for end-effector control.

## Package Architecture

### Core Components
1. **Trajectory Planner** (`trajectory_service.py`) - Quintic/Ruckig trajectory generation
2. **Trajectory Executor** (`trajectory_executor.py`) - High-frequency motion execution  
3. **IK Service** (`ik_service.py`) - PyKDL inverse kinematics solver
4. **Motion Logger** (`motion_logger.py`) - Real-time data capture and CSV export
5. **GUI Interface** (`gui_ee.py`) - Interactive end-effector control
6. **Demo Client** (`demo_plan_publish.py`) - Example trajectory execution

### Data Flow Pipeline
```
[GUI/Client] → [IK Service] → [Trajectory Planner] → [Trajectory Executor] → [Controller]
                     ↓                ↓                      ↓               ↓
[End-Effector Pose] [Joint Goals] [JointTrajectory] [Position Commands] [Joint States]
                                                                              ↓
                                      [Motion Logger] ← [TF2 Transforms] ← [Robot State]
                                            ↓
                                    [CSV Data Export]
```

## Trajectory Planning

### Mathematical Foundation

The trajectory planner uses **quintic (5th-order) polynomial time-scaling** for smooth motion profiles:

#### Quintic Scaling Function
$$s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5, \quad \tau \in [0,1]$$

#### Derivatives (for velocity/acceleration profiles)
- **Velocity**: $\dot{s}(\tau) = 30\tau^2 - 60\tau^3 + 30\tau^4$
- **Acceleration**: $\ddot{s}(\tau) = 60\tau - 180\tau^2 + 120\tau^3$

#### Joint-Space Interpolation
For each joint $i$:
- **Position**: $q_i(t) = q_{i,start} + (q_{i,goal} - q_{i,start}) \cdot s(\tau)$
- **Velocity**: $\dot{q}_i(t) = (q_{i,goal} - q_{i,start}) \cdot \dot{s}(\tau) / T$
- **Acceleration**: $\ddot{q}_i(t) = (q_{i,goal} - q_{i,start}) \cdot \ddot{s}(\tau) / T^2$

Where $\tau = t/T$ and $T$ is the total trajectory duration.

### Trajectory Service Interface

#### Service: `/plan_joint_trajectory`
**Type**: `arm_kinematics/srv/PlanJointTrajectory`

**Request Fields**:
- `joint_names[]`: Joint names in controller order
- `start_positions[]`: Initial joint positions [rad]
- `target_positions[]`: Goal joint positions [rad]  
- `max_velocity[]`: Per-joint velocity limits [rad/s]
- `max_acceleration[]`: Per-joint acceleration limits [rad/s²]
- `max_jerk`: Global jerk limit [rad/s³] (for Ruckig)
- `target_time`: Desired duration [s] (0 = auto-calculate)

**Response Fields**:
- `success`: Planning success flag
- `message`: Status/error message
- `trajectory`: Complete `trajectory_msgs/JointTrajectory`

#### Duration Estimation
When `target_time ≤ 0`, duration is estimated using:
```python
# Per-joint duration estimation
T_vel = max_displacement / max_velocity
T_acc = sqrt(max_displacement / max_acceleration)
T_joint = max(T_vel, T_acc)

# Global duration (conservative)
T_total = max(T_joint for all joints) * safety_factor
```

### Ruckig Integration (Optional)

For advanced trajectory planning with jerk constraints:

#### Installation
```bash
# Ubuntu/Debian
sudo apt install python3-ruckig

# Or via pip
pip install ruckig
```

#### Activation
Edit `trajectory_service.py` and enable the Ruckig branch:
```python
# Change this line:
if ruckig is not None and False:  # Currently disabled
# To:
if ruckig is not None and True:   # Enable Ruckig
```

#### Ruckig Benefits
- **Jerk-limited motion**: Smooth acceleration changes
- **Online replanning**: Real-time trajectory modification
- **Optimal timing**: Minimum-time trajectories
- **Real-time capable**: Suitable for control loops

## Trajectory Execution

### Executor Node: `trajectory_executor.py`

#### Subscription & Publication
- **Subscribes**: `planned_trajectory` (`trajectory_msgs/JointTrajectory`)
- **Publishes**: `/arm_controller/commands` (`std_msgs/Float64MultiArray`)

#### High-Frequency Interpolation
- **Timer Rate**: 200Hz (5ms period) for smooth execution
- **Interpolation**: Linear interpolation between trajectory points
- **Synchronization**: Uses `time_from_start` for precise timing

#### Execution Algorithm
```python
def interpolate_trajectory(self, current_time):
    # Find trajectory segment
    for i, point in enumerate(self.trajectory.points):
        if current_time <= point.time_from_start:
            if i == 0:
                return point.positions
            else:
                # Linear interpolation between points
                prev_point = self.trajectory.points[i-1]
                alpha = (current_time - prev_point.time_from_start) / \
                       (point.time_from_start - prev_point.time_from_start)
                return interpolate(prev_point.positions, point.positions, alpha)
```

## Inverse Kinematics

### IK Service: `/compute_ik`
**Type**: `arm_kinematics/srv/ComputeIK`

#### Request Fields
- `joint_names[]`: Joint names (must match controller order)
- `seed_positions[]`: Initial guess for IK solver [rad]
- `target`: Target end-effector pose (`geometry_msgs/PoseStamped`)
- `base_link`: Base frame (default: `"base_link"`)
- `tip_link`: End-effector frame (default: `"End-Coupler-v1"`)

#### Response Fields
- `success`: IK solution found
- `message`: Status message
- `solution_positions[]`: Joint solution [rad]

### PyKDL Implementation

#### Solver Configuration
- **Algorithm**: Levenberg-Marquardt (LMA)
- **Backend**: PyKDL (Python bindings for KDL)
- **Chain Building**: Runtime URDF expansion via xacro

#### Usage Example
```bash
ros2 service call /compute_ik arm_kinematics/srv/ComputeIK "{
  joint_names: ['Base_Revolute-1','Arm-1_Revolute-2','Arm-2_Revolute-3','Arm-3_Revolute-4','Arm-4_Revolute-5','Arm-5_Revolute-6'],
  seed_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
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

#### Dependencies
```bash
# Required for PyKDL IK service
sudo apt install ros-jazzy-kdl-parser-py python3-pykdl
```

## Motion Logging & Data Export

### Motion Logger: `motion_logger.py`

#### Real-Time Data Capture
- **Sampling Rate**: 100Hz for comprehensive data collection
- **Data Sources**: Joint states, TF transforms, trajectory commands
- **Export Format**: CSV with timestamped data

#### Logged Data Schema
| Column Prefix | Description | Units |
|---------------|-------------|-------|
| `t` | Trajectory time | seconds |
| `des_{joint}` | Planned joint positions | radians |
| `act_{joint}` | Actual joint positions | radians |
| `vel_{joint}` | Joint velocities | rad/s |
| `eff_{joint}` | Joint efforts/torques | N⋅m |
| `ee_act_x`, `ee_act_y`, `ee_act_z` | End-effector position | meters |
| `ee_act_qx`, `ee_act_qy`, `ee_act_qz`, `ee_act_qw` | End-effector orientation | quaternion |

#### Automatic File Management
- **File Pattern**: `log_data/stroke_NNNN.csv`
- **Auto-increment**: Sequential numbering for each trajectory
- **Directory Creation**: Automatic `log_data/` directory creation

### Data Analysis

#### Quick Analysis Commands
```bash
# View latest log
ls -la log_data/
head -5 log_data/stroke_0001.csv

# Basic data inspection
python3 -c "
import pandas as pd
df = pd.read_csv('log_data/stroke_0001.csv')
print('Duration:', f'{df[\"t\"].max():.2f}s')
print('Sample rate:', f'{len(df)/df[\"t\"].max():.1f}Hz')
print('Joint tracking RMSE:')
for joint in ['Base_Revolute-1', 'Arm-1_Revolute-2']:
    if f'des_{joint}' in df.columns:
        error = df[f'act_{joint}'] - df[f'des_{joint}']
        print(f'  {joint}: {(error**2).mean()**0.5:.4f} rad')
"
```

#### Advanced Analysis Tools
See the project root `plot_log_viewer.py` for comprehensive analysis:
- Joint tracking performance plots
- End-effector trajectory visualization  
- Effort/torque analysis
- Velocity profile examination

## GUI Interface

### Interactive Control: `gui_ee.py`

#### Features
- **Pose Sliders**: Intuitive end-effector position/orientation control
- **Numeric Input**: Precise pose entry
- **IK Preview**: Real-time joint solution display
- **Motion Execution**: Direct trajectory planning and execution
- **Status Display**: Connection status and operation feedback

#### GUI Layout
```
┌─────────────────────────────────────┐
│ Target End-Effector Pose            │
├─────────────────────────────────────┤
│ Position: X [ 0.30] Y [ 0.10] Z [ 0.25] │
│ Orient:   X [ 0.00] Y [ 0.00] Z [ 0.00] │
│           W [ 1.00]                 │
├─────────────────────────────────────┤
│ [Compute IK] [Plan + Move]          │
├─────────────────────────────────────┤
│ Joint Solutions:                    │
│ Joint 1: [ 0.524] rad              │
│ Joint 2: [-0.785] rad              │
│ ...                                 │
└─────────────────────────────────────┘
```

#### Operation Workflow
1. **Adjust Target**: Use sliders or numeric inputs for desired pose
2. **Compute IK**: Click to solve inverse kinematics
3. **Preview**: View joint solutions before motion
4. **Execute**: Click "Plan + Move" to execute trajectory

## Launch System

### Complete System Launch
```bash
# Terminal 1: Start simulation and controllers
ros2 launch arm_description arm_launch.py

# Terminal 2: Start kinematics stack with GUI
ros2 launch arm_kinematics gui_ee.launch.py
```

### Launch File: `gui_ee.launch.py`
Coordinates startup of:
- `arm_pykdl/ik_service_cpp` - High-performance C++ IK service
- `trajectory_service.py` - Quintic trajectory planner  
- `trajectory_executor.py` - Motion execution node
- `motion_logger.py` - Data logging service
- `gui_ee.py` - Interactive GUI interface

### Individual Component Launch
```bash
# Kinematics services only
ros2 launch arm_kinematics kinematics.launch.py

# Manual node startup
ros2 run arm_kinematics trajectory_service.py
ros2 run arm_kinematics trajectory_executor.py
ros2 run arm_kinematics motion_logger.py
```

## Programming Interface

### Python Client Example
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arm_kinematics.srv import PlanJointTrajectory, ComputeIK
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray

class MotionClient(Node):
    def __init__(self):
        super().__init__('motion_client')
        
        # Service clients
        self.ik_client = self.create_client(ComputeIK, '/compute_ik')
        self.plan_client = self.create_client(PlanJointTrajectory, '/plan_joint_trajectory')
        
        # Publishers
        self.traj_pub = self.create_publisher(JointTrajectory, 'planned_trajectory', 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        
    async def move_to_pose(self, target_pose):
        # Step 1: Compute IK
        ik_request = ComputeIK.Request()
        ik_request.target = target_pose
        ik_request.joint_names = ['Base_Revolute-1', 'Arm-1_Revolute-2', ...]
        ik_request.seed_positions = [0.0] * 6
        
        ik_response = await self.ik_client.call_async(ik_request)
        if not ik_response.success:
            self.get_logger().error(f"IK failed: {ik_response.message}")
            return False
            
        # Step 2: Plan trajectory
        plan_request = PlanJointTrajectory.Request()
        plan_request.joint_names = ik_request.joint_names
        plan_request.start_positions = [0.0] * 6  # Current position
        plan_request.target_positions = ik_response.solution_positions
        plan_request.max_velocity = [1.0] * 6
        plan_request.max_acceleration = [2.0] * 6
        
        plan_response = await self.plan_client.call_async(plan_request)
        if not plan_response.success:
            self.get_logger().error(f"Planning failed: {plan_response.message}")
            return False
            
        # Step 3: Execute trajectory
        self.traj_pub.publish(plan_response.trajectory)
        self.get_logger().info("Trajectory published for execution")
        return True
```

### Direct Command Interface
```python
# Direct joint position command
cmd_msg = Float64MultiArray()
cmd_msg.data = [0.1, -0.2, 0.3, -0.1, 0.2, 0.0]  # 6 joint positions
cmd_publisher.publish(cmd_msg)
```

## Configuration & Tuning

### Joint Naming Convention
**Critical**: All components must use this exact joint order:
```python
JOINT_ORDER = [
    'Base_Revolute-1', 'Arm-1_Revolute-2', 'Arm-2_Revolute-3',
    'Arm-3_Revolute-4', 'Arm-4_Revolute-5', 'Arm-5_Revolute-6'
]
```

### Trajectory Parameters
```python
# Typical motion limits
MAX_VELOCITY = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]      # rad/s
MAX_ACCELERATION = [4.0, 4.0, 4.0, 4.0, 4.0, 4.0]  # rad/s²
MAX_JERK = 10.0                                      # rad/s³

# Trajectory sampling
TRAJECTORY_DT = 0.01  # 100Hz trajectory points
EXECUTION_DT = 0.005  # 200Hz execution rate
```

### IK Solver Tuning
```python
# PyKDL solver parameters
MAX_ITERATIONS = 500
EPSILON = 1e-6
DAMPING_LAMBDA = 0.01
```

## Troubleshooting

### Common Issues

#### Trajectory Planning Failures
```bash
# Check service availability
ros2 service list | grep plan_joint_trajectory

# Test basic planning
ros2 service call /plan_joint_trajectory arm_kinematics/srv/PlanJointTrajectory "{
  joint_names: ['Base_Revolute-1','Arm-1_Revolute-2','Arm-2_Revolute-3','Arm-3_Revolute-4','Arm-4_Revolute-5','Arm-5_Revolute-6'],
  start_positions: [0,0,0,0,0,0],
  target_positions: [0.1,0.1,0.1,0.1,0.1,0.1],
  max_velocity: [1,1,1,1,1,1],
  max_acceleration: [2,2,2,2,2,2],
  target_time: 2.0
}"
```

#### IK Service Issues
```bash
# Check IK service
ros2 service list | grep compute_ik

# Verify dependencies
python3 -c "import kdl_parser_py, PyKDL; print('PyKDL OK')"

# Test simple IK
ros2 service call /compute_ik arm_kinematics/srv/ComputeIK "{
  joint_names: ['Base_Revolute-1','Arm-1_Revolute-2','Arm-2_Revolute-3','Arm-3_Revolute-4','Arm-4_Revolute-5','Arm-5_Revolute-6'],
  seed_positions: [0,0,0,0,0,0],
  target: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}
}"
```

#### Execution Problems
```bash
# Check trajectory subscription
ros2 topic info planned_trajectory

# Monitor command publication
ros2 topic echo /arm_controller/commands

# Verify controller connection
ros2 control list_controllers
```

#### Motion Logging Issues
```bash
# Check log directory permissions
ls -la log_data/

# Verify TF availability for end-effector poses
ros2 run tf2_ros tf2_echo base_link End-Coupler-v1

# Monitor logging rate
ros2 topic hz /joint_states
```

### Debugging Commands

```bash
# Node status
ros2 node list | grep arm_kinematics

# Topic monitoring
ros2 topic list | grep -E "(planned_trajectory|commands|joint_states)"

# Service testing
ros2 service type /plan_joint_trajectory
ros2 service type /compute_ik

# Parameter inspection
ros2 param list /trajectory_service
ros2 param list /motion_logger
```

## Performance Optimization

### CPU Usage Optimization
- **Service Caching**: IK and planning services cache URDF chains
- **Efficient Interpolation**: Optimized trajectory execution loops
- **Selective Logging**: Configurable data capture rates

### Memory Management
- **Trajectory Buffering**: Circular buffers for trajectory storage
- **Log Rotation**: Automatic cleanup of old CSV files
- **Chain Caching**: Persistent KDL chain storage

### Real-Time Considerations
- **Deterministic Execution**: Fixed-rate timers for trajectory execution
- **Priority Scheduling**: Real-time scheduling for critical components
- **Lock-Free Design**: Minimal synchronization overhead

---

*This package provides the core kinematics and motion planning capabilities for the Kikobot 6-DOF robotic arm, integrating seamlessly with `arm_description`, `arm_controller`, and `arm_pykdl` packages.*inematics — Trajectory Generation, Execution, and IK (ROS 2)

Overview
- This package provides joint-space trajectory generation, simple execution to ros2_control, and an optional inverse kinematics (IK) service for the Kikobot arm.
- It is designed to work with the simulation stack in `arm_description/arm_launch.py` (Gazebo + ros2_control) and the controller configuration in `arm_controller`.

Provided nodes
1) Trajectory planner: `trajectory_service.py`
   - Service name: `plan_joint_trajectory` (type: `arm_kinematics/srv/PlanJointTrajectory`).
   - Input (request):
     - `joint_names`: ordered joint names. Must match the controller YAML order: `['Base_Revolute-1','Arm-1_Revolute-2','Arm-2_Revolute-3','Arm-3_Revolute-4','Arm-4_Revolute-5','Arm-5_Revolute-6']`.
     - `start_positions`: joint start positions [rad].
     - `target_positions`: joint goal positions [rad].
     - `max_velocity` (per-joint), `max_acceleration` (per-joint), `max_jerk` (scalar): limits used by the time-parameterization backend.
     - `target_time`: optional fixed duration [s]; if <= 0 the planner estimates a feasible time from limits.
   - Output (response):
     - `success`, `message`.
     - `trajectory`: `trajectory_msgs/JointTrajectory` with sampled `positions`, `velocities`, `accelerations`, and `time_from_start`.
   - Backend details:
     - Default: smooth 5th‑order (quintic) time-scaling of a straight line in joint space.
       - s(τ) = 10τ^3 − 15τ^4 + 6τ^5, τ ∈ [0,1]; derivatives ds/dt and d²s/dt² are used to populate velocity and acceleration.
       - The duration T is either the provided `target_time` or estimated from limits (heuristic using peak ds and dds bounds), then sampled at dt = 0.01 s (100 Hz).
     - Optional (when installed): Ruckig for jerk-limited trajectories (online/offline). The code is structured to switch to Ruckig if `python3-ruckig` is available. Enable by installing the dependency and flipping the guarded branch in `trajectory_service.py`.

2) Trajectory executor: `trajectory_executor.py`
   - Subscribes: `planned_trajectory` (`trajectory_msgs/JointTrajectory`).
   - Publishes: `/arm_controller/commands` (`std_msgs/Float64MultiArray`) for `position_controllers/JointGroupPositionController`.
   - Behavior:
     - On receiving a `JointTrajectory`, it interprets `time_from_start` as a schedule and linearly interpolates between points at 200 Hz (5 ms timer) to produce position commands.
     - The published vector order must match `arm_controller/config/arm_controllers.yaml`.

3) (Optional) IK service: `ik_service.py`
   - Service name: `compute_ik` (type: `arm_kinematics/srv/ComputeIK`).
   - Input: `joint_names`, `seed_positions`, target `geometry_msgs/PoseStamped`, `base_link`, `tip_link`.
   - Output: `success`, `message`, `solution_positions` (joint vector).
   - Backend: PyKDL (LMA solver). At runtime, the node xacro-expands `arm_description/urdf/arm.urdf.xacro`, builds a KDL tree, extracts the chain base→tip, and runs `ChainIkSolverPos_LMA` from the seed to solve pose IK.
   - Dependencies: `kdl_parser_py` and `python3-pykdl`. If unavailable, the node exits gracefully and the rest of the package still functions.

Topics and services
- Subscribed topics:
  - `planned_trajectory` (`trajectory_msgs/JointTrajectory`) — consumed by the executor.
- Published topics:
  - `/arm_controller/commands` (`std_msgs/Float64MultiArray`) — joint position commands for ros2_control.
- Services:
  - `/plan_joint_trajectory` (`arm_kinematics/srv/PlanJointTrajectory`)
  - `/compute_ik` (`arm_kinematics/srv/ComputeIK`) [optional]

Launch files
- Simulation + controllers (Gazebo, RViz, robot_state_publisher, controller spawners):
  - `ros2 launch arm_description arm_launch.py`
- GUI + kinematics stack (IK service, planner, executor, GUI controls):
  - `ros2 launch arm_kinematics gui_ee.launch.py`
  - This starts:
    - `arm_pykdl/ik_service_cpp` — C++ Orocos KDL IK service on `/compute_ik`.
    - `trajectory_service.py` — planner service `/plan_joint_trajectory` (quintic by default).
    - `trajectory_executor.py` — subscribes `planned_trajectory` and publishes `/arm_controller/commands`.
    - `gui_ee.py` — interactive end‑effector GUI (sliders + numeric entries for start/target pose) to compute IK and command motion.

End‑to‑end flow
1) Bring up the robot and controllers via `arm_launch.py`. The `controller_manager` loads and activates:
   - `joint_state_broadcaster`
   - `arm_controller` (`JointGroupPositionController`), configured for 6 joints.
2) Generate a joint trajectory either by:
   - Calling the planner service: a joint‑space quintic profile (or Ruckig, if enabled) is produced as `JointTrajectory`.
   - Manually publishing your own `JointTrajectory` to `planned_trajectory` (useful for custom paths).
3) The executor interpolates the trajectory timing and publishes dense position setpoints to `/arm_controller/commands`, which ros2_control uses to drive the simulated joints.

How the planner computes trajectories
- Given `q0` (start) and `q1` (goal), a straight line in joint space is time‑parameterized.
- If `target_time` ≤ 0, a duration T is heuristically estimated from per‑joint velocity/acceleration limits so that the motion respects constraints for each joint; then the max across joints is used.
- Quintic time-scaling creates continuous position/velocity/acceleration profiles; samples are emitted at 100 Hz into a `JointTrajectory` (with velocities and accelerations filled per sample).

Ruckig integration (optional)
- Ruckig can compute jerk-limited (`C^2`) trajectories respecting velocity, acceleration, and jerk limits in real-time.
- To enable:
  1) Install `python3-ruckig` (apt or pip, depending on OS/distro).
  2) In `trajectory_service.py`, switch the guarded branch to actually call Ruckig (currently the code uses quintic by default). The structure is already present; only the conditional needs to be enabled.
- Ruckig is most useful when tight dynamic limits or real-time reparameterization are required.

Inverse Kinematics details (optional)
- The IK service runs only if `kdl_parser_py` and `python3-pykdl` are installed.
- The node expands `arm.urdf.xacro` at runtime to obtain the current URDF and then constructs a KDL chain between `base_link` and `End-Coupler-v1` (overridable in the request).
- LMA IK solver uses the provided seed; success depends on reachability and initial guess. Output is a joint vector suitable for use as `target_positions` in the planner.

Using the package
1) Bring up simulation:
   - `ros2 launch arm_description arm_launch.py`
2) Launch the GUI and kinematics stack:
   - `ros2 launch arm_kinematics gui_ee.launch.py`
   - In the GUI, adjust target sliders/entries, click “Compute IK” to preview joints, then “Plan + Move” to execute. Start pose fields are optional; if empty, current `/joint_states` are used.
3) Publish your own trajectory (example):
    - Run:

```bash
ros2 topic pub --once /planned_trajectory trajectory_msgs/msg/JointTrajectory "joint_names:
- Base_Revolute-1
- Arm-1_Revolute-2
- Arm-2_Revolute-3
- Arm-3_Revolute-4
- Arm-4_Revolute-5
- Arm-5_Revolute-6
points:
- positions: [0, 0, 0, 0, 0, 0]
  time_from_start: {sec: 0}
- positions: [0.3, -0.4, 0.5, -0.3, 0.2, 0.1]
  time_from_start: {sec: 3}"
```
4) Or request a planned trajectory and execute automatically (demo node does this on startup).

Data logging (optional)
- A lightweight logger node `motion_logger.py` can be launched alongside the GUI via `gui_ee.launch.py`.
- It subscribes to `planned_trajectory`, `/joint_states`, and TF to log per‑stroke CSVs under `log_data/`.
- Each CSV contains:
  - `t` (s), `des_*` joint columns, `act_*` joint columns, `ee_act_x/y/z` (from TF), `ee_des_x/y/z` (from FK on desired joints).
- Logging is controlled from the GUI via the “Log stroke to CSV” checkbox.

Important assumptions and limits
- The executor and planner assume joint ordering matches `arm_controller/config/arm_controllers.yaml`. Mismatched order will produce incorrect motion.
- No collision checking or environment awareness is provided in this package (pure kinematics/dynamics time parameterization). For collision-aware planning use higher-level planners (e.g., Tesseract + TrajOpt, OMPL) and feed their joint paths to this package for time-parameterization and execution.
- The active controller is position-only; it ignores incoming velocities/accelerations in `JointTrajectory` and uses only the streamed `position` commands.
- Timing is based on ROS time; the executor starts relative timing at the moment the trajectory is received.

Troubleshooting
- Controller spawner errors:
  - Ensure `arm_launch.py` has fully started and both `joint_state_broadcaster` and `arm_controller` are activated before launching this package.
- IK node exits immediately:
  - Install `ros-$ROS_DISTRO-kdl-parser-py` and `python3-pykdl`. If not installed, planning/execution still works; only IK is disabled.
- No motion when publishing `JointTrajectory`:
  - Confirm topic name is `planned_trajectory` and that `joint_names` order matches the controller config. Ensure each `time_from_start` is strictly increasing.

File map
- `srv/PlanJointTrajectory.srv` — service definition for joint-space planning.
- `srv/ComputeIK.srv` — service definition for IK.
- `src/trajectory_service.py` — planner node.
- `src/trajectory_executor.py` — executor node.
- `src/demo_plan_publish.py` — example client that demonstrates plan→execute.
- `src/ik_service.py` — Python IK sample (optional; superseded by C++ `arm_pykdl/ik_service_cpp`).
- `launch/kinematics.launch.py` — launches planner, executor, demo, and IK.
- `launch/gui_ee.launch.py` — launches IK (C++), planner, executor, and the GUI.

Dependencies and technologies
- ROS 2 (rclcpp/rclpy, ros2launch): Nodes, services, and launch orchestration.
- geometry_msgs, trajectory_msgs, sensor_msgs, builtin_interfaces: Message types for pose, trajectories, joint states, and time.
- ros2_control (via `arm_controller`): Consumes `/arm_controller/commands` to move joints.
- Orocos KDL + kdl_parser (C++): URDF parsing to KDL tree/chain, IK via `ChainIkSolverPos_LMA` and NR fallback.
- ament_index_cpp: Locates package share directories (URDF/xacro path fallback).
- xacro (fallback path only): If `robot_description` param is not set, xacro expands `arm_description/urdf/arm.urdf.xacro`.
- numpy (Python planner): Quintic time-scaling sampling for trajectories.
- Optional Ruckig: Jerk‑limited trajectories if `python3-ruckig` is installed and enabled in `trajectory_service.py`.

Runtime order
1) Launch simulation/controllers: `ros2 launch arm_description arm_launch.py`
2) Launch GUI + IK + planner + executor: `ros2 launch arm_kinematics gui_ee.launch.py`
3) Use the GUI or call services/topics directly as needed.


