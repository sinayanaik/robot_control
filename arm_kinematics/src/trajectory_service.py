#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    import numpy as np
except Exception:  # pragma: no cover
    np = None

try:
    # Optional; not required at runtime
    import ruckig  # type: ignore
except Exception:
    ruckig = None

from arm_kinematics.srv import PlanJointTrajectory


def _quintic_time_scaling(duration: float, num_samples: int) -> List[float]:
    t = np.linspace(0.0, duration, num_samples, dtype=float)
    tau = t / duration
    s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    return s.tolist()


def _quintic_derivatives(duration: float, num_samples: int) -> tuple[List[float], List[float]]:
    t = np.linspace(0.0, duration, num_samples, dtype=float)
    tau = t / duration
    ds = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration
    dds = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / (duration**2)
    return ds.tolist(), dds.tolist()


def _estimate_duration(dq: float, vmax: float, amax: float) -> float:
    vmax = max(1e-6, abs(vmax))
    amax = max(1e-6, abs(amax))
    adq = abs(dq)
    max_ds = 1.875
    max_dds = 5.769
    tv = adq * max_ds / vmax
    ta = math.sqrt(adq * max_dds / amax)
    return max(tv, ta, 0.2)


class RuckigTrajectoryPlanner(Node):
    def __init__(self) -> None:
        super().__init__('ruckig_trajectory_planner')
        self.srv = self.create_service(PlanJointTrajectory, 'plan_joint_trajectory', self.handle_request)

    def handle_request(self, req: PlanJointTrajectory.Request, res: PlanJointTrajectory.Response):
        if np is None:
            res.success = False
            res.message = 'numpy not available'
            return res

        nj = len(req.joint_names)
        if nj == 0 or len(req.start_positions) != nj or len(req.target_positions) != nj:
            res.success = False
            res.message = 'invalid joint arrays'
            return res

        vmax = (req.max_velocity if len(req.max_velocity) == nj else [1.0] * nj)
        amax = (req.max_acceleration if len(req.max_acceleration) == nj else [1.0] * nj)
        dq = [req.target_positions[i] - req.start_positions[i] for i in range(nj)]

        if req.target_time > 0.0:
            T = max(req.target_time, 0.2)
        else:
            T = max(_estimate_duration(dq[i], vmax[i], amax[i]) for i in range(nj))

        dt = 0.01
        n = int(max(2, math.ceil(T / dt)) + 1)

        if ruckig is not None and False:
            pass
        else:
            s = _quintic_time_scaling(T, n)
            ds, dds = _quintic_derivatives(T, n)
            pos = [
                [req.start_positions[j] + dq[j] * s[i] for j in range(nj)]
                for i in range(n)
            ]
            vel = [
                [dq[j] * ds[i] for j in range(nj)]
                for i in range(n)
            ]
            acc = [
                [dq[j] * dds[i] for j in range(nj)]
                for i in range(n)
            ]

        traj = JointTrajectory()
        traj.joint_names = list(req.joint_names)
        traj_points = []
        for i in range(n):
            p = JointTrajectoryPoint()
            p.positions = pos[i]
            p.velocities = vel[i]
            p.accelerations = acc[i]
            p.time_from_start = DurationMsg(sec=int(i * dt), nanosec=int((i * dt % 1.0) * 1e9))
            traj_points.append(p)
        traj.points = traj_points

        res.success = True
        res.message = ''
        res.trajectory = traj
        return res


def main() -> None:
    rclpy.init()
    node = RuckigTrajectoryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


