#!/usr/bin/env python3
import bisect
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryExecutor(Node):
    def __init__(self) -> None:
        super().__init__('trajectory_executor')
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.sub = self.create_subscription(JointTrajectory, 'planned_trajectory', self._on_traj, 10)
        self.timer = self.create_timer(0.005, self._on_timer)
        self.joint_names: List[str] = []
        self.times: List[float] = []
        self.points: List[List[float]] = []
        self.start_time: Time | None = None

    def _on_traj(self, msg: JointTrajectory) -> None:
        if len(msg.points) == 0:
            return
        self.joint_names = list(msg.joint_names)
        self.times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in msg.points]
        self.points = [list(p.positions) for p in msg.points]
        self.start_time = self.get_clock().now()

    def _on_timer(self) -> None:
        if self.start_time is None or not self.points:
            return
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if t >= self.times[-1]:
            pos = self.points[-1]
            self.start_time = None
        else:
            i = max(1, bisect.bisect_left(self.times, t))
            t0, t1 = self.times[i - 1], self.times[i]
            p0, p1 = self.points[i - 1], self.points[i]
            s = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            pos = [p0[j] + (p1[j] - p0[j]) * s for j in range(len(p0))]
        msg = Float64MultiArray()
        msg.data = pos
        self.cmd_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TrajectoryExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


