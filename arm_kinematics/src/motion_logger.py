#!/usr/bin/env python3
import bisect
import csv
import math
import os
import subprocess
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory

import tf2_ros
import numpy as np


class MotionLogger(Node):
    def __init__(self) -> None:
        super().__init__('motion_logger')

        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link', 'End-Coupler-v1')
        self.declare_parameter('log_dir', 'log_data')
        self.declare_parameter('enabled', False)
        # Actual EE pose is obtained from TF tree, which in sim reflects physics via joint_states

        self.base_link: str = self.get_parameter('base_link').get_parameter_value().string_value
        self.tip_link: str = self.get_parameter('tip_link').get_parameter_value().string_value
        cfg_log_dir: str = self.get_parameter('log_dir').get_parameter_value().string_value
        self.log_dir: str = str(self._resolve_log_dir(cfg_log_dir))
        

        Path(self.log_dir).mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'log directory: {self.log_dir}')

        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self._on_joint_state, 50)
        self.traj_sub = self.create_subscription(JointTrajectory, 'planned_trajectory', self._on_trajectory, 10)
        self.enable_sub = self.create_subscription(Bool, '/motion_logger/enable', self._on_enable, 10)
        # No direct Gazebo pose subscriptions (topic names may be invalid in ROS). Use TF instead.

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sample_timer = self.create_timer(0.01, self._on_sample)

        self.latest_joint_state: Optional[JointState] = None
        self.active_joint_names: List[str] = []
        self.traj_times: List[float] = []
        self.traj_points: List[List[float]] = []
        self.traj_start_time: Optional[Time] = None
        self.rows: List[List[float]] = []
        self.stroke_index: int = 0
        self.enabled: bool = self.get_parameter('enabled').get_parameter_value().bool_value
        

    def _on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def _on_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        state = 'ENABLED' if self.enabled else 'DISABLED'
        self.get_logger().info(f'logging {state}')

    

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        if len(msg.points) == 0:
            return
        if self.traj_start_time is not None and self.enabled:
            self._finalize_and_save()
        if not self.enabled:
            self.get_logger().info('logging disabled; ignoring incoming trajectory')
            self._reset()
            return
        self.active_joint_names = list(msg.joint_names)
        self.traj_times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in msg.points]
        self.traj_points = [list(p.positions) for p in msg.points]
        self.traj_start_time = self.get_clock().now()
        self.rows = []
        self.stroke_index += 1
        self.get_logger().info(f'logging started for stroke #{self.stroke_index}')

    def _on_sample(self) -> None:
        if self.traj_start_time is None or not self.traj_points:
            return
        t = (self.get_clock().now() - self.traj_start_time).nanoseconds * 1e-9

        if t >= self.traj_times[-1]:
            des = self.traj_points[-1]
            act = self._current_actual_positions(self.active_joint_names)
            eff = self._current_actual_efforts(self.active_joint_names)
            ee_act = self._lookup_ee()
            self._append_row(self.traj_times[-1], des, act, eff, ee_act)
            self._finalize_and_save()
            return

        i = max(1, bisect.bisect_left(self.traj_times, t))
        t0, t1 = self.traj_times[i - 1], self.traj_times[i]
        p0, p1 = self.traj_points[i - 1], self.traj_points[i]
        s = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        des = [p0[j] + (p1[j] - p0[j]) * s for j in range(len(p0))]

        act = self._current_actual_positions(self.active_joint_names)
        eff = self._current_actual_efforts(self.active_joint_names)
        ee_act = self._lookup_ee()
        self._append_row(t, des, act, eff, ee_act)

    def _current_actual_positions(self, names: List[str]) -> List[float]:
        if self.latest_joint_state is None:
            return [math.nan] * len(names)
        actual = []
        for n in names:
            try:
                idx = self.latest_joint_state.name.index(n)
                actual.append(self.latest_joint_state.position[idx] if idx < len(self.latest_joint_state.position) else math.nan)
            except ValueError:
                actual.append(math.nan)
        return actual

    def _current_actual_efforts(self, names: List[str]) -> List[float]:
        if self.latest_joint_state is None:
            return [math.nan] * len(names)
        efforts = []
        for n in names:
            try:
                idx = self.latest_joint_state.name.index(n)
                efforts.append(self.latest_joint_state.effort[idx] if idx < len(self.latest_joint_state.effort) else math.nan)
            except ValueError:
                efforts.append(math.nan)
        return efforts

    def _lookup_ee(self) -> Optional[List[float]]:
        # Resolve via TF tree (reflects simulated state via joint_states)
        try:
            tf = self.tf_buffer.lookup_transform(self.base_link, self.tip_link, Time(), timeout=Duration(seconds=0.05))
            t = tf.transform.translation
            return [t.x, t.y, t.z]
        except Exception:
            return None

    def _resolve_log_dir(self, configured: str) -> Path:
        conf_path = Path(configured)
        if conf_path.is_absolute():
            return conf_path
        src_based = self._find_workspace_root() / conf_path
        return src_based

    def _append_row(self, t: float, des: List[float], act: List[float], eff: List[float], ee_act: Optional[List[float]]) -> None:
        row: List[float] = [t]
        row.extend(des)
        row.extend(act)
        row.extend(eff)
        if ee_act is not None:
            row.extend(ee_act)
        else:
            row.extend([math.nan, math.nan, math.nan])
        self.rows.append(row)

    def _finalize_and_save(self) -> None:
        if not self.rows or not self.active_joint_names:
            self._reset()
            return
        n = len(self.active_joint_names)
        headers = ['t']
        headers += [f'des_{self.active_joint_names[i]}' for i in range(n)]
        headers += [f'act_{self.active_joint_names[i]}' for i in range(n)]
        headers += [f'eff_{self.active_joint_names[i]}' for i in range(n)]
        headers += ['ee_act_x', 'ee_act_y', 'ee_act_z']

        fname = f'stroke_{self.stroke_index:04d}.csv'
        path = os.path.join(self.log_dir, fname)
        try:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(headers)
                for r in self.rows:
                    writer.writerow(r)
            self.get_logger().info(f'logging stopped; saved {path}')
        except Exception as e:
            self.get_logger().error(f'failed to save log: {e}')
        self._reset()

    def _reset(self) -> None:
        self.active_joint_names = []
        self.traj_times = []
        self.traj_points = []
        self.traj_start_time = None
        self.rows = []

    

    def _resolve_log_dir(self, configured: str) -> Path:
        conf_path = Path(configured)
        if conf_path.is_absolute():
            return conf_path
        src_based = self._find_workspace_root() / conf_path
        return src_based

    def _find_workspace_root(self) -> Path:
        p = Path(__file__).resolve()
        for parent in p.parents:
            if (parent / 'src').is_dir():
                return parent
        # Fallback to current working directory
        return Path.cwd()


def main() -> None:
    rclpy.init()
    node = MotionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()


