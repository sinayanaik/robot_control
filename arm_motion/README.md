# arm_motion - Demo Motion Patterns & Testing Utilities

## Overview

The `arm_motion` package provides example motion patterns and testing utilities for the 6-DOF Kikobot robotic arm. This package serves as a development sandbox separate from the main `arm_kinematics` stack, offering pre-programmed motion demonstrations, visualization capabilities, and testing tools for validating system functionality.

## Package Components

### Motion Node: `motion_node.py`

The main demonstration node that provides:
- **Sinusoidal joint motion patterns** for smooth, continuous movement
- **Real-time joint state monitoring** via subscription to `/joint_states`
- **End-effector trail visualization** using RViz markers
- **Configurable motion parameters** for different demonstration modes

### Motion Patterns

#### Sinusoidal Joint Motion
The node generates sinusoidal motion patterns for each joint:

```python
# Motion equation for each joint
joint_position[i] = amplitude[i] * sin(frequency[i] * time + phase[i]) + offset[i]
```

**Default Parameters**:
- **Amplitudes**: `[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]` rad
- **Frequencies**: `[0.2, 0.3, 0.4, 0.3, 0.2, 0.1]` Hz
- **Phase Offsets**: `[0, π/4, π/2, 3π/4, π, 5π/4]` rad
- **Center Positions**: `[0, 0, 0, 0, 0, 0]` rad

#### Coordinated Motion
The different frequencies and phase offsets create complex, visually appealing motion patterns that exercise the full workspace while avoiding singularities and joint limits.

### Visualization Features

#### End-Effector Trail
- **Marker Type**: LINE_STRIP visualization markers
- **Trail Length**: Configurable history buffer
- **Color Coding**: Time-based color gradients
- **Frame Reference**: Displayed in `base_link` frame

#### Joint State Monitoring
- **Real-time feedback**: Continuous monitoring of actual joint positions
- **Error detection**: Comparison between commanded and actual positions
- **Performance metrics**: Motion smoothness and tracking accuracy

## Launch System

### Quick Start
```bash
# Terminal 1: Start simulation and controllers
ros2 launch arm_description arm_launch.py

# Terminal 2: Start motion demonstration
ros2 launch arm_motion motion.launch.py
```

### Launch Configuration: `motion.launch.py`
The launch file starts:
- `motion_node.py` - Main motion generation node
- Configurable parameters for motion patterns
- Optional RViz markers for visualization

## Usage Examples

### Basic Motion Demo
```bash
# Start standard sinusoidal motion
ros2 launch arm_motion motion.launch.py

# Monitor motion in terminal
ros2 topic echo /arm_controller/commands

# View end-effector trail in RViz
# Add MarkerArray display for topic: /visualization_marker_array
```

### Custom Motion Parameters
```bash
# Launch with custom parameters
ros2 launch arm_motion motion.launch.py amplitude:=0.3 frequency:=0.5

# Or set parameters directly
ros2 param set /motion_node amplitude "[0.3, 0.3, 0.3, 0.3, 0.3, 0.3]"
ros2 param set /motion_node frequency "[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]"
```

### Individual Node Execution
```bash
# Run motion node directly
ros2 run arm_motion motion_node.py

# With parameter overrides
ros2 run arm_motion motion_node.py --ros-args -p amplitude:="[0.2, 0.2, 0.2, 0.2, 0.2, 0.2]"
```

---

*This is a demo package; prefer `arm_kinematics` planner/executor for real trajectories.*
*Ensure controllers are running (e.g., via `ros2 launch arm_description arm_launch.py`).*


