# arm_motion — Example motion utilities for Kikobot

Overview
- Contains example nodes for visualizing and publishing joint motion. Intended as a sandbox separate from `arm_kinematics`.

Node
- `src/motion_node.py`
  - Publishes sinusoidal joint commands to `/arm_controller/commands`.
  - Subscribes to `/joint_states` for feedback.
  - Publishes `/visualization_marker_array` with an end‑effector trail when TF is available.

Usage
```bash
ros2 launch arm_motion motion.launch.py
```

Notes
- This is a demo node; prefer `arm_kinematics` planner/executor for real trajectories.
- Ensure controllers are running (e.g., via `ros2 launch arm_description arm_launch.py`).


