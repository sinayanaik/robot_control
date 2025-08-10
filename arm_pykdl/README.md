arm_pykdl — C++ IK Service (ROS 2)

Overview
- Provides `/compute_ik` compatible with `arm_kinematics/srv/ComputeIK`.
- Drop-in replacement for the Python `ik_service.py` using Orocos KDL.
- Parameters: `robot_description` (URDF XML string), `base_link` (default `base_link`), `tip_link` (default `End-Coupler-v1`). Request fields override params when provided.

Build & Run
```bash
cd ~/Public/ws  # or your workspace
colcon build --packages-select arm_pykdl
source install/setup.bash
ros2 run arm_pykdl ik_service_cpp
# Or with launch file (parameters optional):
ros2 launch arm_pykdl ik_service.launch.py
```

URDF loading
- If `robot_description` parameter is provided, it is used directly.
- Otherwise, the node falls back to executing `xacro` on `$(ament_index_cpp get arm_description)/urdf/arm.urdf.xacro` and parses the resulting URDF text with `kdl_parser`.
- KDL chains are cached per `(base_link, tip_link)` pair.

Service test example
```bash
ros2 service call /compute_ik arm_kinematics/srv/ComputeIK "{
  joint_names: ['Base_Revolute-1','Arm-1_Revolute-2','Arm-2_Revolute-3','Arm-3_Revolute-4','Arm-4_Revolute-5','Arm-5_Revolute-6'],
  seed_positions: [0,0,0,0,0,0],
  target: { header: { frame_id: 'base_link' }, pose: { position: {x: 0.30, y: 0.10, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} } },
  base_link: 'base_link',
  tip_link: 'End-Coupler-v1'
}"
```

Client script
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arm_kinematics.srv import ComputeIK

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_test_client')
        self.cli = self.create_client(ComputeIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')

    def call(self):
        req = ComputeIK.Request()
        req.joint_names = ['Base_Revolute-1','Arm-1_Revolute-2','Arm-2_Revolute-3','Arm-3_Revolute-4','Arm-4_Revolute-5','Arm-5_Revolute-6']
        req.seed_positions = [0.0]*6
        req.target.header.frame_id = 'base_link'
        req.target.pose.position.x = 0.30
        req.target.pose.position.y = 0.10
        req.target.pose.position.z = 0.25
        req.target.pose.orientation.w = 1.0
        req.base_link = 'base_link'
        req.tip_link = 'End-Coupler-v1'
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    client = IKClient()
    res = client.call()
    print('success:', res.success)
    print('message:', res.message)
    print('solution_positions:', list(res.solution_positions))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Notes
- Validates `joint_names.size() == seed_positions.size() == chain joints`. On mismatch: `joint arrays mismatch`.
- On IK failure: `IK failed`. On chain/URDF failure: `failed to build KDL chain`.
- Keep existing topics: planner/executor remain unchanged.


