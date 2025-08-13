# arm_controller — ros2_control setup for Kikobot

Overview
- This package provides the ros2_control configuration and launch for the Kikobot 6‑DOF arm.
- It configures a JointGroupPositionController named `arm_controller` and a `joint_state_broadcaster`.

Files
- `config/arm_controllers.yaml`: Controller and hardware interface configuration.
- `launch/controllers.launch.py`: Launches controller manager and spawns required controllers.

Controller interface
- Controller name: `arm_controller` (type: `position_controllers/JointGroupPositionController`).
- Command topic: `/arm_controller/commands` (`std_msgs/Float64MultiArray`), ordering MUST match the joint list below.
- Joints (order):
  - `Base_Revolute-1`
  - `Arm-1_Revolute-2`
  - `Arm-2_Revolute-3`
  - `Arm-3_Revolute-4`
  - `Arm-4_Revolute-5`
  - `Arm-5_Revolute-6`

Usage
```bash
# Start controllers along with the simulation via arm_description launch
ros2 launch arm_description arm_launch.py

# Or run controller spawners explicitly
ros2 launch arm_controller controllers.launch.py

# Send a one‑shot position command
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.1, -0.2, 0.3, -0.1, 0.2, 0.0]}'
```

Notes
- The joint ordering must be consistent across planner/executor and this controller.
- `joint_state_broadcaster` publishes `/joint_states` used by visualization and logging.


