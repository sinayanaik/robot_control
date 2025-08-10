arm_kinematics — Trajectory Generation, Execution, and IK (ROS 2)

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
- Kinematics stack (planner, executor, demo publisher, optional IK):
  - `ros2 launch arm_kinematics kinematics.launch.py`
  - This starts:
    - `trajectory_service.py` (planner)
    - `trajectory_executor.py` (executor)
    - `demo_plan_publish.py` (example client that calls the service and publishes to `planned_trajectory`)
    - `ik_service.py` (only succeeds if KDL is installed)

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
2) Start kinematics stack:
   - `ros2 launch arm_kinematics kinematics.launch.py`
3) Publish your own trajectory (example):
   - `ros2 topic pub /planned_trajectory trajectory_msgs/JointTrajectory "..."`
4) Or request a planned trajectory and execute automatically (demo node does this on startup).

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
- `src/ik_service.py` — IK node (optional, KDL-based).
- `launch/kinematics.launch.py` — launches planner, executor, demo, and IK.


