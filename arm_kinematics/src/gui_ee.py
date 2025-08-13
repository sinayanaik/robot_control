#!/usr/bin/env python3
import threading
import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Bool
from arm_kinematics.srv import ComputeIK, PlanJointTrajectory


JOINT_ORDER = [
    'Base_Revolute-1',
    'Arm-1_Revolute-2',
    'Arm-2_Revolute-3',
    'Arm-3_Revolute-4',
    'Arm-4_Revolute-5',
    'Arm-5_Revolute-6',
]


class EEGui(Node):
    def __init__(self):
        super().__init__('ee_gui')
        self.ik_cli = self.create_client(ComputeIK, '/compute_ik')
        self.plan_cli = self.create_client(PlanJointTrajectory, '/plan_joint_trajectory')
        self.traj_pub = self.create_publisher(JointTrajectory, 'planned_trajectory', 10)
        self.log_pub = self.create_publisher(Bool, '/motion_logger/enable', 1)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.name_to_pos = {}

    def _js_cb(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.name_to_pos[n] = p

    def get_current_positions(self):
        return [self.name_to_pos.get(n, 0.0) for n in JOINT_ORDER]

    def call_ik(self, base_link: str, tip_link: str, pose: PoseStamped, seed):
        if not self.ik_cli.wait_for_service(timeout_sec=2.0):
            return False, 'IK service unavailable', []
        req = ComputeIK.Request()
        req.joint_names = JOINT_ORDER
        req.seed_positions = seed
        req.target = pose
        req.base_link = base_link
        req.tip_link = tip_link
        future = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            return False, 'IK call failed', []
        return res.success, res.message, list(res.solution_positions)

    def call_plan(self, start, target, max_vel, max_acc, max_jerk, target_time):
        if not self.plan_cli.wait_for_service(timeout_sec=2.0):
            return False, 'Planner service unavailable', None
        req = PlanJointTrajectory.Request()
        req.joint_names = JOINT_ORDER
        req.start_positions = start
        req.target_positions = target
        req.max_velocity = max_vel
        req.max_acceleration = max_acc
        req.max_jerk = max_jerk
        req.target_time = target_time
        future = self.plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            return False, 'Plan call failed', None
        if not res.success:
            return False, res.message, None
        return True, '', res.trajectory


def rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class App:
    def __init__(self):
        rclpy.init()
        self.node = EEGui()
        self.target_solution = None

        self.root = tk.Tk()
        self.root.title('EE GUI')
        frm = ttk.Frame(self.root, padding=10)
        frm.grid()

        self.base_var = tk.StringVar(value='base_link')
        self.tip_var = tk.StringVar(value='End-Coupler-v1')

        ttk.Label(frm, text='base_link').grid(column=0, row=0, sticky='e')
        ttk.Combobox(frm, textvariable=self.base_var, values=['base_link'], width=18, state='readonly').grid(column=1, row=0)
        ttk.Label(frm, text='tip_link').grid(column=2, row=0, sticky='e')
        ttk.Combobox(frm, textvariable=self.tip_var, values=['End-Coupler-v1'], width=18, state='readonly').grid(column=3, row=0)

        ttk.Label(frm, text='Start').grid(column=0, row=1, sticky='w')
        self.sx = tk.DoubleVar(value=0.10)
        self.sy = tk.DoubleVar(value=0.0)
        self.sz = tk.DoubleVar(value=0.30)
        self.sr = tk.DoubleVar(value=0.0)
        self.sp = tk.DoubleVar(value=0.0)
        self.syaw = tk.DoubleVar(value=0.0)

        headers = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for i, h in enumerate(headers):
            ttk.Label(frm, text=h).grid(column=2*i + 1, row=1, padx=4)

        self._make_slider_pair(frm, self.sx, 0, 2, 0.0, 0.5)
        self._make_slider_pair(frm, self.sy, 2, 2, -0.3, 0.3)
        self._make_slider_pair(frm, self.sz, 4, 2, 0.0, 0.5)
        self._make_slider_pair(frm, self.sr, 6, 2, -3.14, 3.14)
        self._make_slider_pair(frm, self.sp, 8, 2, -3.14, 3.14)
        self._make_slider_pair(frm, self.syaw, 10, 2, -3.14, 3.14)

        ttk.Label(frm, text='Target').grid(column=0, row=3, sticky='w')
        self.tx = tk.DoubleVar(value=0.10)
        self.ty = tk.DoubleVar(value=0.0)
        self.tz = tk.DoubleVar(value=0.40)
        self.tr = tk.DoubleVar(value=0.0)
        self.tp = tk.DoubleVar(value=0.0)
        self.tyaw = tk.DoubleVar(value=0.0)

        self._make_slider_pair(frm, self.tx, 0, 4, 0.0, 0.5)
        self._make_slider_pair(frm, self.ty, 2, 4, -0.3, 0.3)
        self._make_slider_pair(frm, self.tz, 4, 4, 0.0, 0.5)
        self._make_slider_pair(frm, self.tr, 6, 4, -3.14, 3.14)
        self._make_slider_pair(frm, self.tp, 8, 4, -3.14, 3.14)
        self._make_slider_pair(frm, self.tyaw, 10, 4, -3.14, 3.14)

        self.msg_var = tk.StringVar()
        ttk.Label(frm, textvariable=self.msg_var, foreground='blue').grid(column=0, row=5, columnspan=6, sticky='w', pady=(6,0))

        # Duration control (target_time for planner / Ruckig)
        self.duration = tk.DoubleVar(value=3.0)
        ttk.Label(frm, text='duration (s)').grid(column=3, row=6, sticky='e', padx=(12, 4))
        self._make_slider_pair(frm, self.duration, 4, 6, 0.2, 10.0)

        # Logging enable checkbox
        self.log_enabled = tk.BooleanVar(value=False)
        def on_log_toggle():
            msg = Bool()
            msg.data = self.log_enabled.get()
            self.node.log_pub.publish(msg)
        ttk.Checkbutton(frm, text='Log stroke to CSV', variable=self.log_enabled, command=on_log_toggle).grid(column=0, row=7, sticky='w', pady=(4,0))

        ttk.Button(frm, text='Compute IK', command=self.on_compute_target).grid(column=0, row=6, padx=4, pady=6)
        ttk.Button(frm, text='Plan + Move', command=self.on_move).grid(column=1, row=6, padx=4, pady=6)

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()

    def _make_pose(self, x, y, z, qx, qy, qz, qw):
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def _make_slider_pair(self, parent, var: tk.DoubleVar, col_start: int, row: int, vmin: float, vmax: float):
        scale = ttk.Scale(parent, from_=vmin, to=vmax, variable=var, orient='horizontal', length=140)
        scale.grid(column=col_start, row=row, padx=4, sticky='we')
        entry = ttk.Entry(parent, width=8)
        entry.grid(column=col_start + 1, row=row, padx=2)

        def on_var_changed(*_):
            entry.delete(0, tk.END)
            entry.insert(0, f"{var.get():.3f}")

        var.trace_add('write', on_var_changed)
        on_var_changed()

        def on_entry_commit(_event=None):
            try:
                val = float(entry.get())
                if val < vmin:
                    val = vmin
                if val > vmax:
                    val = vmax
                var.set(val)
            except ValueError:
                on_var_changed()

        entry.bind('<Return>', on_entry_commit)
        entry.bind('<FocusOut>', on_entry_commit)

    def on_compute_target(self):
        x, y, z = self.tx.get(), self.ty.get(), self.tz.get()
        qx, qy, qz, qw = rpy_to_quat(self.tr.get(), self.tp.get(), self.tyaw.get())
        pose = self._make_pose(x, y, z, qx, qy, qz, qw)
        success, msg, sol = self.node.call_ik(self.base_var.get(), self.tip_var.get(), pose, self.node.get_current_positions())
        if success:
            self.target_solution = sol
            self.msg_var.set('IK ok: ' + ', '.join(f'{v:.3f}' for v in sol))
        else:
            self.target_solution = None
            self.msg_var.set('IK failed: ' + msg)

    def on_move(self):
        start_positions = self.node.get_current_positions()
        sx, sy, sz = self.sx.get(), self.sy.get(), self.sz.get()
        if any(abs(v) > 1e-9 for v in [sx, sy, sz]):
            sqx, sqy, sqz, sqw = rpy_to_quat(self.sr.get(), self.sp.get(), self.syaw.get())
            s_pose = self._make_pose(sx, sy, sz, sqx, sqy, sqz, sqw)
            ok, msg, s_sol = self.node.call_ik(self.base_var.get(), self.tip_var.get(), s_pose, start_positions)
            if not ok:
                self.msg_var.set('Start IK failed: ' + msg)
                return
            start_positions = s_sol

        if self.target_solution is None:
            qx, qy, qz, qw = rpy_to_quat(self.tr.get(), self.tp.get(), self.tyaw.get())
            t_pose = self._make_pose(self.tx.get(), self.ty.get(), self.tz.get(), qx, qy, qz, qw)
            ok, msg, t_sol = self.node.call_ik(self.base_var.get(), self.tip_var.get(), t_pose, start_positions)
            if not ok:
                self.msg_var.set('Target IK failed: ' + msg)
                return
            target_positions = t_sol
        else:
            target_positions = self.target_solution

        max_vel = [1.0] * 6
        max_acc = [2.0] * 6
        ok, msg, traj = self.node.call_plan(start_positions, target_positions, max_vel, max_acc, 5.0, self.duration.get())
        if not ok:
            self.msg_var.set('Plan failed: ' + msg)
            return
        self.node.traj_pub.publish(traj)
        self.msg_var.set('Published trajectory')

    def run(self):
        self.root.mainloop()
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    App().run()


if __name__ == '__main__':
    main()


