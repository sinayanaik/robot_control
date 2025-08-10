#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory

from arm_kinematics.srv import PlanJointTrajectory


class DemoPlanPublish(Node):
    def __init__(self) -> None:
        super().__init__('demo_plan_publish')
        self.cli = self.create_client(PlanJointTrajectory, 'plan_joint_trajectory')
        self.pub = self.create_publisher(JointTrajectory, 'planned_trajectory', 10)
        self.timer = self.create_timer(1.0, self._tick)
        self.sent = False

    def _tick(self) -> None:
        if self.sent:
            return
        if not self.cli.wait_for_service(timeout_sec=0.1):
            return
        req = PlanJointTrajectory.Request()
        req.joint_names = [
            'Base_Revolute-1',
            'Arm-1_Revolute-2',
            'Arm-2_Revolute-3',
            'Arm-3_Revolute-4',
            'Arm-4_Revolute-5',
            'Arm-5_Revolute-6',
        ]
        req.start_positions = [0.0] * 6
        req.target_positions = [0.3, -0.4, 0.5, -0.3, 0.2, 0.1]
        req.max_velocity = [1.0] * 6
        req.max_acceleration = [2.0] * 6
        req.max_jerk = 10.0
        req.target_time = 3.0

        future = self.cli.call_async(req)
        future.add_done_callback(self._on_done)
        self.sent = True

    def _on_done(self, fut) -> None:
        try:
            resp = fut.result()
        except Exception as e:
            self.get_logger().error(f'service error: {e}')
            return
        if not resp.success:
            self.get_logger().error(f'planning failed: {resp.message}')
            return
        traj: JointTrajectory = resp.trajectory
        traj.header = Header()
        self.pub.publish(traj)
        self.get_logger().info('published planned trajectory')


def main() -> None:
    rclpy.init()
    node = DemoPlanPublish()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


