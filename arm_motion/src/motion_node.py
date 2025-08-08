#!/usr/bin/env python3
import math
import time
from pathlib import Path
import signal

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from geometry_msgs.msg import Point

try:
    import pandas as pd
    import numpy as np
    import matplotlib.pyplot as plt
except Exception:
    pd = None
    np = None
    plt = None


class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')

        self.position_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # Names must match your controller YAML order
        self.joint_names = [
            'Base_Revolute-1',
            'Arm-1_Revolute-2',
            'Arm-2_Revolute-3',
            'Arm-3_Revolute-4',
            'Arm-4_Revolute-5',
            'Arm-5_Revolute-6',
        ]

        self.data = {
            'timestamp': [],
            'desired_positions': [],
            'actual_positions': [],
            'velocities': [],
            'efforts': [],
            'end_effector_x': [],
            'end_effector_y': [],
            'end_effector_z': [],
        }

        self.latest_joint_state = None
        self.ee_trail = []
        self.trail_duration_sec = 5.0
        self.marker_size = 0.015
        self.marker_array = MarkerArray()

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.command_timer = self.create_timer(0.05, self._command_timer_cb)  # 20 Hz
        self.marker_timer = self.create_timer(0.02, self._marker_timer_cb)   # 50 Hz

        self.start_time = time.time()
        self.get_logger().info('arm_motion: sinusoid command node started')

        signal.signal(signal.SIGINT, self._sigint)

    def _joint_state_cb(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def _get_ee_point(self) -> Point | None:
        try:
            tf = self.tf_buffer.lookup_transform('base_link', 'End-Coupler-v1', Time(), Duration(seconds=0.05))
            p = Point(x=tf.transform.translation.x, y=tf.transform.translation.y, z=tf.transform.translation.z)
            return p
        except Exception:
            return None

    def _command_timer_cb(self) -> None:
        t = time.time() - self.start_time
        # Simple 6-DOF sinusoids within limits; modify as desired
        desired = [
            0.3 * math.sin(0.3 * t),
            0.5 * math.sin(0.5 * t),
            -0.5 * math.sin(0.4 * t),
            0.2 * math.sin(0.7 * t),
            0.2 * math.sin(0.9 * t),
            0.3 * math.sin(0.6 * t),
        ]
        msg = Float64MultiArray()
        msg.data = desired
        self.position_pub.publish(msg)

        ee = self._get_ee_point()
        if self.latest_joint_state is not None:
            self.data['timestamp'].append(t)
            self.data['desired_positions'].append(desired)
            if ee is not None:
                self.data['end_effector_x'].append(ee.x)
                self.data['end_effector_y'].append(ee.y)
                self.data['end_effector_z'].append(ee.z)

            actual, vel, eff = [], [], []
            for name in self.joint_names:
                try:
                    i = self.latest_joint_state.name.index(name)
                    actual.append(self.latest_joint_state.position[i] if i < len(self.latest_joint_state.position) else 0.0)
                    vel.append(self.latest_joint_state.velocity[i] if i < len(self.latest_joint_state.velocity) else 0.0)
                    eff.append(self.latest_joint_state.effort[i] if i < len(self.latest_joint_state.effort) else 0.0)
                except ValueError:
                    actual.append(0.0); vel.append(0.0); eff.append(0.0)
            self.data['actual_positions'].append(actual)
            self.data['velocities'].append(vel)
            self.data['efforts'].append(eff)

    def _marker_timer_cb(self) -> None:
        now = self.get_clock().now()
        ee = self._get_ee_point()
        if ee is None:
            return
        self.ee_trail.append((ee, now.nanoseconds))
        cutoff = now.nanoseconds - int(self.trail_duration_sec * 1e9)
        self.ee_trail = [(p, ts) for p, ts in self.ee_trail if ts >= cutoff]

        ma = MarkerArray()
        for i, (p, ts) in enumerate(self.ee_trail):
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = now.to_msg()
            m.ns = 'ee_trail'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = p
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.marker_size
            age = (now.nanoseconds - ts) / 1e9
            alpha = max(0.0, 0.8 * (1.0 - age / self.trail_duration_sec))
            m.color.r, m.color.g, m.color.b, m.color.a = (0.0, 0.7, 1.0, alpha)
            if i == len(self.ee_trail) - 1:
                m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.0, 0.0, 1.0)
            m.lifetime = Duration(seconds=self.trail_duration_sec).to_msg()
            ma.markers.append(m)
        self.marker_pub.publish(ma)
        self.marker_array = ma

    def _sigint(self, *_):
        self.get_logger().info('SIGINT received; saving data (if libs present)')
        if pd is None or np is None or plt is None or len(self.data['timestamp']) == 0:
            rclpy.shutdown()
            return
        root = Path(__file__).resolve().parents[4]
        out_dir = root / 'test_datas'
        out_dir.mkdir(exist_ok=True)
        ts = int(time.time())
        # Flatten arrays to columns for CSV
        rows = []
        for i, t in enumerate(self.data['timestamp']):
            row = {
                't': t,
                'ee_x': self.data['end_effector_x'][i] if i < len(self.data['end_effector_x']) else float('nan'),
                'ee_y': self.data['end_effector_y'][i] if i < len(self.data['end_effector_y']) else float('nan'),
                'ee_z': self.data['end_effector_z'][i] if i < len(self.data['end_effector_z']) else float('nan'),
            }
            des = self.data['desired_positions'][i]
            act = self.data['actual_positions'][i]
            vel = self.data['velocities'][i]
            eff = self.data['efforts'][i]
            for j in range(6):
                row[f'des_{j+1}'] = des[j] if j < len(des) else float('nan')
                row[f'act_{j+1}'] = act[j] if j < len(act) else float('nan')
                row[f'vel_{j+1}'] = vel[j] if j < len(vel) else float('nan')
                row[f'eff_{j+1}'] = eff[j] if j < len(eff) else float('nan')
            rows.append(row)
        df = pd.DataFrame(rows)
        csv = out_dir / f'motion_{ts}.csv'
        df.to_csv(csv, index=False)
        self.get_logger().info(f'Saved: {csv}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = JointPositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
