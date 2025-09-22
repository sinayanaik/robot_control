# arm_description - Robot Model & Simulation

## Overview

The `arm_description` package provides the complete robot model definition and simulation environment for the 6-DOF Kikobot robotic arm. This package contains the URDF/Xacro robot description, STL meshes, Gazebo simulation setup, RViz configuration, and comprehensive launch files that orchestrate the entire simulation stack.

## Package Contents

### Robot Model (`urdf/`)
- **`arm.urdf.xacro`**: Main robot description file with complete kinematic and dynamic model
- **`meshes/`**: STL mesh files for all robot links
  - `Base.stl` - Robot base structure
  - `Arm-1.stl` through `Arm-5.stl` - Arm segment meshes
  - `End-Coupler-v1.stl` - End-effector mounting interface

### Launch Files (`launch/`)
- **`arm_launch.py`**: Master launch file for complete system startup
- **`gazebo.launch.py`**: Gazebo-specific simulation launch
- **`rviz_launch.py`**: RViz visualization launch

### Visualization (`rviz/`)
- **`config.rviz`**: Pre-configured RViz setup with robot model, joint states, and TF frames

### Information (`info/`)
- **`README.md`**: Detailed robot specifications and link properties
- **`robot_info.py`**: Python utilities for robot parameter extraction

## Robot Specifications

### Kinematic Structure
```
world → base_link → Base → Arm-1 → Arm-2 → Arm-3 → Arm-4 → Arm-5 → End-Coupler-v1
```

### Joint Configuration
| Joint Name | Type | Parent Link | Child Link | Axis | Limits |
|------------|------|-------------|------------|------|---------|
| `Base_Revolute-1` | revolute | Base | Arm-1 | Z | ±2π |
| `Arm-1_Revolute-2` | revolute | Arm-1 | Arm-2 | Z | ±2π |
| `Arm-2_Revolute-3` | revolute | Arm-2 | Arm-3 | Z | ±2π |
| `Arm-3_Revolute-4` | revolute | Arm-3 | Arm-4 | Z | ±2π |
| `Arm-4_Revolute-5` | revolute | Arm-4 | Arm-5 | Z | ±2π |
| `Arm-5_Revolute-6` | revolute | Arm-5 | End-Coupler-v1 | Z | ±2π |

### Physical Properties
| Link | Mass (kg) | Inertia (Ixx, Iyy, Izz) | Description |
|------|-----------|------------------------|-------------|
| Base | 5.0 | (0.05, 0.05, 0.05) | Fixed base structure |
| Arm-1 | 1.069 | (0.00067, 0.00063, 0.00048) | First arm segment |
| Arm-2 | 2.517 | Dynamic calculation | Second arm segment |
| Arm-3 | 2.152 | Dynamic calculation | Third arm segment |
| Arm-4 | 0.843 | Dynamic calculation | Fourth arm segment |
| Arm-5 | 0.900 | Dynamic calculation | Fifth arm segment |

### Frame Definitions
- **Base Frame**: `base_link` (world reference for control)
- **End-Effector**: `End-Coupler-v1` (tool mounting point)
- **World Frame**: `world` (global reference)

## Gazebo Integration

### Hardware Interface
The robot integrates with Gazebo using `gz_ros2_control` plugin:

```xml
<plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimSystem">
  <parameters>$(find arm_controller)/config/arm_controllers.yaml</parameters>
</plugin>
```

### Joint Properties
- **Update Rate**: 1000Hz for high-fidelity simulation
- **Interface Types**: Position command, position/velocity/effort state
- **Effort Limits**: Configured per joint for realistic dynamics
- **Velocity Limits**: Set for safe operation bounds

### Simulation Features
- **Physics Engine**: Gazebo with accurate mass/inertia properties
- **Collision Detection**: Full collision geometry from STL meshes
- **Visual Rendering**: High-quality mesh visualization with materials
- **Clock Bridging**: Synchronized simulation time with ROS 2

## Launch System

### Master Launch: `arm_launch.py`

Comprehensive system startup that coordinates:

1. **Environment Setup**
   - Gazebo resource path configuration
   - URDF parameter loading via xacro

2. **Core Nodes**
   - `robot_state_publisher`: TF tree publication
   - `gz_spawn_entity`: Robot insertion into Gazebo
   - `gz_ros2_bridge`: Clock synchronization

3. **Controller Stack**
   - `joint_state_broadcaster`: Joint state publication
   - `arm_controller`: Position controller for 6-DOF control
   - Sequential spawning with proper dependencies

4. **Visualization**
   - RViz with pre-configured robot display
   - Real-time joint state visualization

#### Launch Arguments
```bash
ros2 launch arm_description arm_launch.py model:=/path/to/custom.xacro rviz_config:=/path/to/custom.rviz
```

### Quick Start Commands

```bash
# Standard system launch
ros2 launch arm_description arm_launch.py

# Launch with custom URDF
ros2 launch arm_description arm_launch.py model:=/path/to/modified_arm.urdf.xacro

# Launch without RViz (headless)
ros2 launch arm_description arm_launch.py rviz_config:=""
```

## Development & Customization

### URDF Modifications

The URDF uses Xacro for parameterization. Key customization points:

1. **Joint Limits**: Modify in joint definitions
```xml
<limit effort="100" lower="-6.28" upper="6.28" velocity="2.0" />
```

2. **Inertial Properties**: Update mass and inertia tensors
```xml
<inertial>
  <mass value="1.069" />
  <inertia ixx="0.00067" iyy="0.00063" izz="0.00048" ... />
</inertial>
```

3. **Mesh Scaling**: Adjust mesh scale factors
```xml
<mesh filename="file://$(find arm_description)/urdf/meshes/Arm-1.stl" scale="0.001 0.001 0.001" />
```

### Adding New Components

To extend the robot:

1. **New Links**: Add link definitions with appropriate meshes
2. **Additional Joints**: Define kinematic connections
3. **Controller Updates**: Modify `arm_controller` configuration
4. **Mesh Integration**: Place STL files in `urdf/meshes/`

### Simulation Tuning

Gazebo parameters can be tuned for different simulation requirements:

- **Physics Step Size**: Modify for accuracy vs. performance
- **Solver Iterations**: Adjust for stability
- **Contact Parameters**: Tune for realistic contact dynamics

## Visualization Configuration

### RViz Setup
The included RViz configuration provides:

- **Robot Model**: 3D visualization with proper materials
- **TF Frames**: Joint coordinate frames display
- **Joint State Slider**: Manual joint position control
- **Grid Reference**: Spatial orientation aid

### Customizing Visualization
```bash
# Launch RViz with custom config
ros2 launch arm_description rviz_launch.py rviz_config:=/path/to/custom.rviz

# Save current RViz configuration
# File → Save Config As... → save to package rviz/ directory
```

## Troubleshooting

### Common Issues

1. **Missing Meshes**
   - Ensure STL files are in `urdf/meshes/`
   - Check file permissions and paths
   - Verify mesh scale factors

2. **URDF Parse Errors**
   - Validate XML syntax
   - Check xacro parameter substitutions
   - Use `check_urdf` for validation

3. **Gazebo Loading Issues**
   - Verify gz_ros2_control installation
   - Check resource path environment variables
   - Ensure controller configuration matches URDF

4. **Controller Spawn Failures**
   - Verify joint names match exactly
   - Check controller configuration YAML
   - Ensure proper spawn sequencing

### Debugging Commands

```bash
# Validate URDF syntax
check_urdf /path/to/generated.urdf

# Expand xacro to URDF
xacro arm.urdf.xacro > expanded.urdf

# View TF tree
ros2 run tf2_tools view_frames.py

# Check joint states
ros2 topic echo /joint_states

# Monitor controller status
ros2 control list_controllers
```

## Dependencies

### Required Packages
- `urdf` - URDF parsing and utilities
- `xacro` - Xacro macro processing
- `robot_state_publisher` - TF tree publication
- `rviz2` - 3D visualization
- `ros_gz_sim` - Gazebo integration
- `gz_ros2_control` - Hardware interface
- `controller_manager` - Controller management

### Optional Dependencies
- `joint_state_publisher_gui` - Manual joint control
- `ros_gz_bridge` - Additional Gazebo bridging

## Integration Notes

### Controller Compatibility
This package is designed to work with:
- `arm_controller` - Position controller configuration
- `arm_kinematics` - Trajectory planning and execution
- `kdl_cplusplus` - Inverse kinematics services

### Coordinate Frame Conventions
- **Right-handed coordinate system**
- **Z-up orientation for world frame**
- **Standard ROS TF conventions**
- **Base link at origin for control reference**

---

*For detailed robot specifications and link properties, see `info/README.md`*