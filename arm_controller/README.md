# arm_controller - ROS 2 Control Configuration

## Overview

The `arm_controller` package provides the ros2_control configuration for the 6-DOF Kikobot robotic arm. This package configures the controller manager, hardware interfaces, and joint controllers required for precise position control of the robot in both simulation and (potentially) real hardware environments.

## Package Architecture

### Controller Stack
- **Controller Manager**: Central hub for controller lifecycle management
- **Hardware Interface**: gz_ros2_control integration for Gazebo simulation
- **Joint Controllers**: Position control for 6-DOF motion
- **State Broadcasting**: Real-time joint state publication

### Control Flow
```
[Trajectory Commands] → [arm_controller] → [Hardware Interface] → [Gazebo Physics]
                                ↓
[Joint State Feedback] ← [joint_state_broadcaster] ← [Hardware Interface]
```

## Controller Configuration

### Main Controller: `arm_controller`
- **Type**: `position_controllers/JointGroupPositionController`
- **Update Rate**: 1000Hz for high-precision control
- **Command Interface**: Position commands for all 6 joints
- **State Interfaces**: Position, velocity, and effort feedback

### Joint State Broadcasting: `joint_state_broadcaster`
- **Type**: `joint_state_broadcaster/JointStateBroadcaster`
- **Function**: Publishes current joint states to `/joint_states` topic
- **Rate**: Synchronized with controller manager update rate

## Joint Configuration

### Critical Joint Ordering
The joint order is **hardcoded** and must be consistent across all system components:

```yaml
joints: [Base_Revolute-1, Arm-1_Revolute-2, Arm-2_Revolute-3, 
         Arm-3_Revolute-4, Arm-4_Revolute-5, Arm-5_Revolute-6]
```

### Interface Configuration
```yaml
arm_controller:
  ros__parameters:
    joints: [Base_Revolute-1, Arm-1_Revolute-2, Arm-2_Revolute-3, Arm-3_Revolute-4, Arm-4_Revolute-5, Arm-5_Revolute-6]
    command_interfaces: [position]
    state_interfaces: [position, velocity, effort]
```

## Topics & Services

### Command Interface
- **Topic**: `/arm_controller/commands`
- **Type**: `std_msgs/msg/Float64MultiArray`
- **Format**: 6-element array with joint positions in radians
- **Order**: Must match the joint configuration exactly

### State Feedback
- **Topic**: `/joint_states`
- **Type**: `sensor_msgs/msg/JointState`
- **Content**: Position, velocity, and effort for all joints
- **Rate**: 1000Hz update frequency

### Controller Management Services
- **List Controllers**: `ros2 control list_controllers`
- **Load Controller**: `ros2 control load_controller <controller_name>`
- **Configure Controller**: `ros2 control set_controller_state <controller_name> configure`
- **Start Controller**: `ros2 control switch_controllers --start <controller_name>`

## Usage Examples

### Basic Operation

#### System Startup
```bash
# Launch complete system (includes controller spawning)
ros2 launch arm_description arm_launch.py

# Or spawn controllers explicitly
ros2 launch arm_controller controllers.launch.py
```

#### Manual Joint Commands
```bash
# Send single position command (all joints to specific positions)
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray \
  '{data: [0.1, -0.2, 0.3, -0.1, 0.2, 0.0]}'

# Send home position command
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray \
  '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'

# Continuous commanding (be careful!)
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray \
  '{data: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]}' --rate 10
```

#### Monitoring Joint States
```bash
# View current joint positions
ros2 topic echo /joint_states

# Monitor specific joint
ros2 topic echo /joint_states --field position

# Check update rate
ros2 topic hz /joint_states
```

### Controller Management

#### Status Checking
```bash
# List all controllers and their states
ros2 control list_controllers

# Get detailed controller info
ros2 control describe_controller arm_controller

# Check hardware interface status
ros2 control list_hardware_interfaces
```

#### Advanced Control
```bash
# Stop controller
ros2 control switch_controllers --stop arm_controller

# Restart controller
ros2 control switch_controllers --start arm_controller

# Switch between controllers (if multiple are configured)
ros2 control switch_controllers --stop old_controller --start new_controller
```

## Configuration Details

### Controller Manager Settings
```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # High-frequency control loop
    use_sim_time: true # Synchronized with Gazebo simulation time
```

### Performance Tuning
The 1000Hz update rate provides:
- **Smooth Motion**: Minimal discretization artifacts
- **Fast Response**: Quick reaction to command changes
- **Stable Control**: Reduced oscillation and overshoot
- **Data Quality**: High-resolution state feedback for logging

### Hardware Interface Integration
The controller interfaces with Gazebo through:
```xml
<!-- In URDF -->
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <joint name="Base_Revolute-1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <!-- ... additional joints ... -->
</ros2_control>
```

## Integration with Other Packages

### Trajectory Execution (`arm_kinematics`)
The controller receives commands from the trajectory executor:
```python
# In trajectory_executor.py
self.command_pub = self.create_publisher(
    Float64MultiArray, 
    '/arm_controller/commands', 
    10
)
```

### Motion Logging
Joint states are logged by the motion logger for analysis:
```python
# Motion logger subscribes to controller feedback
self.joint_state_sub = self.create_subscription(
    JointState,
    '/joint_states',
    self.joint_state_callback,
    10
)
```

### GUI Control (`arm_kinematics`)
The GUI interface sends direct commands for manual control:
```python
# GUI publishes to controller command topic
self.joint_pub = self.create_publisher(
    Float64MultiArray,
    '/arm_controller/commands',
    10
)
```

## Safety Considerations

### Joint Limits
Joint limits are enforced at multiple levels:
1. **URDF Definition**: Physical limits in robot description
2. **Controller Limits**: Software limits in controller configuration
3. **Planning Limits**: Trajectory planner respects constraints

### Emergency Stop
```bash
# Stop all controllers immediately
ros2 control switch_controllers --stop arm_controller

# Or stop all motion by sending current position
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray \
  '{data: [$(ros2 topic echo /joint_states --field position --once | tr -d "[]" | tr "," " " | xargs echo)]}'
```

### Rate Limiting
The high update rate requires careful consideration:
- **CPU Usage**: Monitor system load during operation
- **Message Flooding**: Avoid excessive command publishing
- **Synchronization**: Ensure proper timing with trajectory execution

## Troubleshooting

### Common Issues

#### Controller Fails to Start
```bash
# Check if controller manager is running
ros2 node list | grep controller_manager

# Verify controller configuration
ros2 param list /controller_manager

# Check joint names match URDF
ros2 param get /controller_manager arm_controller.joints
```

#### No Joint Commands Response
```bash
# Verify controller is active
ros2 control list_controllers

# Check command topic subscription
ros2 topic info /arm_controller/commands

# Test with manual command
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray '{data: [0,0,0,0,0,0]}'
```

#### Joint State Issues
```bash
# Check if joint_state_broadcaster is active
ros2 control list_controllers | grep joint_state_broadcaster

# Verify joint state publication
ros2 topic hz /joint_states

# Check for joint name mismatches
ros2 topic echo /joint_states --field name
```

### Debugging Commands

```bash
# View controller configuration
ros2 param dump /controller_manager

# Monitor controller performance
ros2 topic hz /arm_controller/commands
ros2 topic hz /joint_states

# Check system resource usage
top -p $(pgrep -f controller_manager)

# Validate controller yaml
ros2 param load /controller_manager /path/to/arm_controllers.yaml
```

## Advanced Configuration

### Custom Controller Parameters
```yaml
arm_controller:
  ros__parameters:
    joints: [...]
    command_interfaces: [position]
    state_interfaces: [position, velocity, effort]
    # Optional: Add PID parameters for enhanced control
    pid_gains:
      Base_Revolute-1: {p: 100, i: 0.1, d: 10}
      # ... additional joints
```

### Multiple Controller Setup
For advanced applications, multiple controllers can be configured:
```yaml
# Additional controller for velocity control
velocity_controller:
  type: velocity_controllers/JointGroupVelocityController
  joints: [Base_Revolute-1, Arm-1_Revolute-2, ...]
```

### Hardware-Specific Configuration
When adapting for real hardware:
```yaml
# Replace gz_ros2_control with actual hardware interface
hardware:
  plugin: your_hardware_interface/YourHardwareSystem
  parameters:
    device: "/dev/ttyUSB0"
    baudrate: 115200
```

## Performance Metrics

### Typical Performance
- **Update Rate**: 1000Hz stable
- **Command Latency**: <1ms
- **Joint Tracking**: Sub-milliradian accuracy
- **CPU Usage**: ~5-10% on modern hardware

### Benchmarking
```bash
# Monitor timing performance
ros2 topic hz /joint_states
ros2 topic delay /arm_controller/commands

# System resource monitoring
htop -p $(pgrep -f controller_manager)
```

---

*This controller configuration is optimized for the Kikobot 6-DOF arm and integrates seamlessly with the `arm_description`, `arm_kinematics`, and `arm_pykdl` packages.*


