#!/usr/bin/env python3
import sys
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import pandas as pd
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# Import 3D plotting capability
HAS_3D = True
try:
    from mpl_toolkits.mplot3d import Axes3D
    # Test if 3d projection is actually available
    fig_test = plt.figure()
    try:
        fig_test.add_subplot(111, projection='3d')
    except (ValueError, KeyError):
        HAS_3D = False
    plt.close(fig_test)
except ImportError:
    HAS_3D = False

if not HAS_3D:
    print("Note: 3D plotting not available. Using 2D fallback for trajectory visualization.")

# Set modern matplotlib style
try:
    plt.style.use('seaborn-v0_8-darkgrid')
except (OSError, ValueError):
    try:
        plt.style.use('seaborn-darkgrid')
    except (OSError, ValueError):
        plt.style.use('default')

plt.rcParams.update({
    'font.size': 9,
    'axes.labelsize': 10,
    'axes.titlesize': 11,
    'legend.fontsize': 8,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'figure.titlesize': 12,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'lines.linewidth': 1.5,
    'axes.prop_cycle': plt.cycler('color', ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b'])
})


class PlotWindow:
    def __init__(self, parent: tk.Tk, title: str, geometry: str = "800x600") -> None:
        self.win = tk.Toplevel(parent)
        self.win.title(title)
        self.win.geometry(geometry)
        
        # Create main figure with tight layout
        self.fig = Figure(figsize=(10, 7), constrained_layout=True)
        self.fig.patch.set_facecolor('white')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.win)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Create toolbar
        toolbar_frame = ttk.Frame(self.win)
        toolbar_frame.pack(side=tk.BOTTOM, fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()
        
    def clear(self):
        self.fig.clear()
        
    def draw(self):
        self.canvas.draw_idle()

    def plot_joint_positions(self, df: pd.DataFrame, joint_names: list[str], 
                           show_desired: bool, show_actual: bool) -> None:
        """Plot joint positions with adaptive scaling and trend analysis."""
        self.clear()
        
        t = df['t']
        n_joints = len(joint_names)
        
        # Determine optimal subplot arrangement
        if n_joints <= 3:
            rows, cols = 1, n_joints
        elif n_joints <= 6:
            rows, cols = 2, 3
        else:
            rows, cols = 3, (n_joints + 2) // 3
            
        colors_des = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        colors_act = ['#4CAF50', '#FF9800', '#2196F3', '#F44336', '#9C27B0', '#795548']
        
        for i, joint in enumerate(joint_names):
            ax = self.fig.add_subplot(rows, cols, i + 1)
            
            # Plot desired and actual positions
            if show_desired and f'des_{joint}' in df.columns:
                des_data = df[f'des_{joint}']
                ax.plot(t, np.degrees(des_data), color=colors_des[i % len(colors_des)], 
                       linewidth=2.5, label='Desired', alpha=0.8)
                       
            if show_actual and f'act_{joint}' in df.columns:
                act_data = df[f'act_{joint}']
                ax.plot(t, np.degrees(act_data), color=colors_act[i % len(colors_act)], 
                       linewidth=2, label='Actual', linestyle='--', alpha=0.9)
                       
            # Calculate and display statistics
            if show_desired and show_actual and f'des_{joint}' in df.columns and f'act_{joint}' in df.columns:
                error = df[f'act_{joint}'] - df[f'des_{joint}']
                rmse = np.sqrt(np.mean(error**2))
                max_error = np.max(np.abs(error))
                
                # Add error statistics text box
                stats_text = f'RMSE: {np.degrees(rmse):.2f}°\nMax Error: {np.degrees(max_error):.2f}°'
                ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='wheat', alpha=0.7),
                       verticalalignment='top', fontsize=8)
            
            # Customize plot
            ax.set_title(f'{joint.replace("_", " ").title()}', fontweight='bold', fontsize=10)
            ax.set_xlabel('Time (s)', fontsize=9)
            ax.set_ylabel('Angle (degrees)', fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)
            
            # Adaptive y-axis scaling
            if show_actual and f'act_{joint}' in df.columns:
                data_range = np.ptp(np.degrees(df[f'act_{joint}']))
                if data_range > 0:
                    margin = data_range * 0.1
                    ax.set_ylim(np.degrees(df[f'act_{joint}'].min()) - margin,
                               np.degrees(df[f'act_{joint}'].max()) + margin)
        
        self.fig.suptitle('Joint Position Tracking Analysis', fontsize=14, fontweight='bold')
        self.draw()

    def plot_joint_efforts(self, df: pd.DataFrame, joint_names: list[str]) -> None:
        """Plot joint efforts with torque analysis and statistics."""
        self.clear()
        
        t = df['t']
        n_joints = len(joint_names)
        
        # Check if effort data exists
        effort_cols = [f'eff_{joint}' for joint in joint_names if f'eff_{joint}' in df.columns]
        if not effort_cols:
            ax = self.fig.add_subplot(1, 1, 1)
            ax.text(0.5, 0.5, 'No effort data available', 
                   transform=ax.transAxes, ha='center', va='center', fontsize=16)
            self.draw()
            return
        
        # Determine layout
        if n_joints <= 3:
            rows, cols = 1, n_joints
        elif n_joints <= 6:
            rows, cols = 2, 3
        else:
            rows, cols = 3, (n_joints + 2) // 3
            
        colors = ['#E91E63', '#3F51B5', '#4CAF50', '#FF9800', '#9C27B0', '#795548']
        
        # Calculate overall effort statistics
        all_efforts = []
        for joint in joint_names:
            if f'eff_{joint}' in df.columns:
                efforts = df[f'eff_{joint}'].dropna()
                all_efforts.extend(efforts.tolist())
        
        if all_efforts:
            total_rms_effort = np.sqrt(np.mean(np.array(all_efforts)**2))
            max_effort = np.max(np.abs(all_efforts))
        
        for i, joint in enumerate(joint_names):
            if f'eff_{joint}' not in df.columns:
                continue
                
            ax = self.fig.add_subplot(rows, cols, i + 1)
            effort_data = df[f'eff_{joint}']
            
            # Plot effort with gradient coloring for intensity
            scatter = ax.scatter(t, effort_data, c=np.abs(effort_data), 
                               cmap='plasma', alpha=0.7, s=8)
            ax.plot(t, effort_data, color=colors[i % len(colors)], 
                   linewidth=1.5, alpha=0.8)
            
            # Add zero line
            ax.axhline(y=0, color='black', linestyle='-', alpha=0.3, linewidth=0.5)
            
            # Calculate statistics
            rms_effort = np.sqrt(np.mean(effort_data**2))
            max_effort_joint = np.max(np.abs(effort_data))
            mean_effort = np.mean(np.abs(effort_data))
            
            # Add statistics text
            stats_text = f'RMS: {rms_effort:.2f} N⋅m\nMax: {max_effort_joint:.2f} N⋅m\nMean: {mean_effort:.2f} N⋅m'
            ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='lightblue', alpha=0.7),
                   verticalalignment='top', fontsize=8)
            
            # Customize plot
            ax.set_title(f'{joint.replace("_", " ").title()} Effort', fontweight='bold', fontsize=10)
            ax.set_xlabel('Time (s)', fontsize=9)
            ax.set_ylabel('Torque (N⋅m)', fontsize=9)
            ax.grid(True, alpha=0.3)
            
            # Adaptive scaling with effort-based margins
            effort_range = np.ptp(effort_data)
            if effort_range > 0:
                margin = effort_range * 0.15
                ax.set_ylim(effort_data.min() - margin, effort_data.max() + margin)
        
        # Add overall statistics in title
        if all_efforts:
            title = f'Joint Effort Analysis - Total RMS: {total_rms_effort:.1f} N⋅m, Peak: {max_effort:.1f} N⋅m'
        else:
            title = 'Joint Effort Analysis'
        self.fig.suptitle(title, fontsize=14, fontweight='bold')
        self.draw()

    def plot_end_effector_analysis(self, df: pd.DataFrame) -> None:
        """Plot comprehensive end-effector analysis with trajectory and velocity."""
        self.clear()
        
        # Check if end-effector data exists
        ee_cols = ['ee_act_x', 'ee_act_y', 'ee_act_z']
        if not all(col in df.columns for col in ee_cols):
            ax = self.fig.add_subplot(1, 1, 1)
            ax.text(0.5, 0.5, 'No end-effector position data available', 
                   transform=ax.transAxes, ha='center', va='center', fontsize=16)
            self.draw()
            return
        
        t = df['t']
        x, y, z = df['ee_act_x'] * 1000, df['ee_act_y'] * 1000, df['ee_act_z'] * 1000  # Convert to mm
        
        # Create 2x2 layout
        ax1 = self.fig.add_subplot(2, 2, 1)  # XY trajectory
        ax2 = self.fig.add_subplot(2, 2, 2)  # 3D trajectory (or XZ if 3D not available)
        ax3 = self.fig.add_subplot(2, 2, 3)  # Position vs time
        ax4 = self.fig.add_subplot(2, 2, 4)  # Velocity analysis
        
        # XY Trajectory with gradient coloring
        colors = plt.cm.viridis(np.linspace(0, 1, len(t)))
        ax1.scatter(x, y, c=colors, s=15, alpha=0.7)
        ax1.plot(x, y, 'b-', alpha=0.5, linewidth=1)
        ax1.scatter(x.iloc[0], y.iloc[0], color='green', s=100, marker='o', label='Start', zorder=5)
        ax1.scatter(x.iloc[-1], y.iloc[-1], color='red', s=100, marker='s', label='End', zorder=5)
        ax1.set_xlabel('X (mm)')
        ax1.set_ylabel('Y (mm)')
        ax1.set_title('XY Trajectory')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 3D trajectory or XZ fallback
        if HAS_3D:
            try:
                ax2 = self.fig.add_subplot(2, 2, 2, projection='3d')
                ax2.plot(x, y, z, 'b-', alpha=0.7, linewidth=2)
                ax2.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color='green', s=100, label='Start')
                ax2.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], color='red', s=100, label='End')
                ax2.set_xlabel('X (mm)')
                ax2.set_ylabel('Y (mm)')
                ax2.set_zlabel('Z (mm)')
                ax2.set_title('3D Trajectory')
                ax2.legend()
            except:
                # Fallback to XZ plot
                ax2.scatter(x, z, c=colors, s=15, alpha=0.7)
                ax2.plot(x, z, 'b-', alpha=0.5, linewidth=1)
                ax2.scatter(x.iloc[0], z.iloc[0], color='green', s=100, marker='o', label='Start')
                ax2.scatter(x.iloc[-1], z.iloc[-1], color='red', s=100, marker='s', label='End')
                ax2.set_xlabel('X (mm)')
                ax2.set_ylabel('Z (mm)')
                ax2.set_title('XZ Trajectory')
                ax2.legend()
                ax2.grid(True, alpha=0.3)
        else:
            # XZ fallback
            ax2.scatter(x, z, c=colors, s=15, alpha=0.7)
            ax2.plot(x, z, 'b-', alpha=0.5, linewidth=1)
            ax2.scatter(x.iloc[0], z.iloc[0], color='green', s=100, marker='o', label='Start')
            ax2.scatter(x.iloc[-1], z.iloc[-1], color='red', s=100, marker='s', label='End')
            ax2.set_xlabel('X (mm)')
            ax2.set_ylabel('Z (mm)')
            ax2.set_title('XZ Trajectory')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
        
        # Position components vs time
        ax3.plot(t, x, 'r-', label='X', linewidth=2)
        ax3.plot(t, y, 'g-', label='Y', linewidth=2)
        ax3.plot(t, z, 'b-', label='Z', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Position (mm)')
        ax3.set_title('Position Components')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Velocity analysis
        if len(t) > 1:
            dt = np.diff(t)
            vx = np.diff(x) / dt
            vy = np.diff(y) / dt
            vz = np.diff(z) / dt
            v_mag = np.sqrt(vx**2 + vy**2 + vz**2)
            t_vel = t[:-1]
            
            ax4.plot(t_vel, v_mag, 'purple', linewidth=2, label='Speed')
            ax4.fill_between(t_vel, 0, v_mag, alpha=0.3, color='purple')
            
            # Add statistics
            max_speed = np.max(v_mag)
            avg_speed = np.mean(v_mag)
            stats_text = f'Max: {max_speed:.1f} mm/s\nAvg: {avg_speed:.1f} mm/s'
            ax4.text(0.02, 0.98, stats_text, transform=ax4.transAxes,
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen', alpha=0.7),
                    verticalalignment='top', fontsize=9)
        
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Speed (mm/s)')
        ax4.set_title('End-Effector Speed')
        ax4.grid(True, alpha=0.3)
        
        # Calculate workspace statistics
        workspace_x = x.max() - x.min()
        workspace_y = y.max() - y.min()
        workspace_z = z.max() - z.min()
        
        self.fig.suptitle(f'End-Effector Analysis - Workspace: {workspace_x:.1f}×{workspace_y:.1f}×{workspace_z:.1f} mm³', 
                         fontsize=14, fontweight='bold')
        self.draw()

    def plot_tracking_performance(self, df: pd.DataFrame, joint_names: list[str]) -> None:
        """Plot detailed tracking performance analysis."""
        self.clear()
        
        t = df['t']
        
        # Calculate tracking errors for available joints
        tracking_data = {}
        for joint in joint_names:
            if f'des_{joint}' in df.columns and f'act_{joint}' in df.columns:
                error = df[f'act_{joint}'] - df[f'des_{joint}']
                tracking_data[joint] = {
                    'error': error,
                    'rmse': np.sqrt(np.mean(error**2)),
                    'max_error': np.max(np.abs(error)),
                    'std': np.std(error)
                }
        
        if not tracking_data:
            ax = self.fig.add_subplot(1, 1, 1)
            ax.text(0.5, 0.5, 'No tracking data available\n(Need both desired and actual positions)', 
                   transform=ax.transAxes, ha='center', va='center', fontsize=16)
            self.draw()
            return
        
        # Create layout: errors plot + performance summary
        ax1 = self.fig.add_subplot(2, 1, 1)
        ax2 = self.fig.add_subplot(2, 1, 2)
        
        # Plot tracking errors
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        for i, (joint, data) in enumerate(tracking_data.items()):
            error_deg = np.degrees(data['error'])
            ax1.plot(t, error_deg, color=colors[i % len(colors)], 
                    linewidth=1.5, label=joint.replace('_', ' '), alpha=0.8)
        
        ax1.axhline(y=0, color='black', linestyle='-', alpha=0.3, linewidth=1)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Position Error (degrees)')
        ax1.set_title('Joint Tracking Errors', fontweight='bold')
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax1.grid(True, alpha=0.3)
        
        # Performance summary bar chart
        joint_labels = [j.replace('_', ' ') for j in tracking_data.keys()]
        rmse_values = [np.degrees(data['rmse']) for data in tracking_data.values()]
        max_values = [np.degrees(data['max_error']) for data in tracking_data.values()]
        
        x_pos = np.arange(len(joint_labels))
        width = 0.35
        
        bars1 = ax2.bar(x_pos - width/2, rmse_values, width, label='RMSE', 
                       color='skyblue', alpha=0.8)
        bars2 = ax2.bar(x_pos + width/2, max_values, width, label='Max Error', 
                       color='lightcoral', alpha=0.8)
        
        # Add value labels on bars
        for bar in bars1:
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                    f'{height:.2f}°', ha='center', va='bottom', fontsize=8)
        
        for bar in bars2:
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                    f'{height:.2f}°', ha='center', va='bottom', fontsize=8)
        
        ax2.set_xlabel('Joints')
        ax2.set_ylabel('Error (degrees)')
        ax2.set_title('Tracking Performance Summary', fontweight='bold')
        ax2.set_xticks(x_pos)
        ax2.set_xticklabels(joint_labels, rotation=45, ha='right')
        ax2.legend()
        ax2.grid(True, alpha=0.3, axis='y')
        
        # Calculate overall performance
        overall_rmse = np.sqrt(np.mean([data['rmse']**2 for data in tracking_data.values()]))
        
        self.fig.suptitle(f'Tracking Performance Analysis - Overall RMSE: {np.degrees(overall_rmse):.2f}°', 
                         fontsize=14, fontweight='bold')
        self.draw()

    def render_comprehensive_analysis(self, df: pd.DataFrame, joint_names: list[str], 
                                    show_des: bool, show_act: bool, show_eff: bool, show_ee: bool) -> None:
        self.fig.clear()
        
        # Create a 2x3 grid layout for comprehensive view (avoiding 3D issues)
        gs = self.fig.add_gridspec(2, 3, height_ratios=[1, 1], width_ratios=[1, 1, 1])
        
        t = df['t']
        n_joints = len(joint_names)
        
        # Define colors for consistency
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        
        # Top row: Joint Positions (Left), Joint Efforts (Middle), End-Effector (Right)
        
        # Joint Positions
        if show_des or show_act:
            ax_pos = self.fig.add_subplot(gs[0, 0])
            for i, joint in enumerate(joint_names):
                color = colors[i % len(colors)]
                if show_des and f'des_{joint}' in df.columns:
                    ax_pos.plot(t, np.degrees(df[f'des_{joint}']), '--', 
                              color=color, alpha=0.8, linewidth=1.5, label=f'{joint} (target)')
                if show_act and f'act_{joint}' in df.columns:
                    ax_pos.plot(t, np.degrees(df[f'act_{joint}']), '-', 
                              color=color, linewidth=2, label=f'{joint} (actual)')
            
            ax_pos.set_title('Joint Positions', fontweight='bold')
            ax_pos.set_xlabel('Time [s]')
            ax_pos.set_ylabel('Angle [°]')
            ax_pos.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=7)
            ax_pos.grid(True, alpha=0.3)
        
        # Joint Efforts  
        if show_eff:
            ax_eff = self.fig.add_subplot(gs[0, 1])
            for i, joint in enumerate(joint_names):
                if f'eff_{joint}' in df.columns:
                    color = colors[i % len(colors)]
                    ax_eff.plot(t, df[f'eff_{joint}'], '-', color=color, 
                              linewidth=2, label=f'{joint}')
            
            ax_eff.set_title('Joint Efforts', fontweight='bold')
            ax_eff.set_xlabel('Time [s]')
            ax_eff.set_ylabel('Torque [N⋅m]')
            ax_eff.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=7)
            ax_eff.grid(True, alpha=0.3)
            ax_eff.axhline(y=0, color='black', linestyle='-', alpha=0.3, linewidth=0.8)
        
        # End-Effector Position Components
        if show_ee and all(col in df.columns for col in ['ee_act_x', 'ee_act_y', 'ee_act_z']):
            ax_ee = self.fig.add_subplot(gs[0, 2])
            ax_ee.plot(t, df['ee_act_x'] * 1000, 'b-', linewidth=2, label='X')
            ax_ee.plot(t, df['ee_act_y'] * 1000, 'r-', linewidth=2, label='Y')
            ax_ee.plot(t, df['ee_act_z'] * 1000, 'g-', linewidth=2, label='Z')
            ax_ee.set_title('End-Effector Position', fontweight='bold')
            ax_ee.set_xlabel('Time [s]')
            ax_ee.set_ylabel('Position [mm]')
            ax_ee.legend()
            ax_ee.grid(True, alpha=0.3)
        
        # Bottom row: Tracking Errors (Left), Performance Metrics (Middle), XY Trajectory (Right)
        
        # Tracking Errors
        if show_des and show_act:
            ax_err = self.fig.add_subplot(gs[1, 0])
            for i, joint in enumerate(joint_names):
                if f'des_{joint}' in df.columns and f'act_{joint}' in df.columns:
                    error = np.degrees(df[f'act_{joint}'] - df[f'des_{joint}'])
                    color = colors[i % len(colors)]
                    ax_err.plot(t, error, '-', color=color, alpha=0.7, linewidth=1.5, label=f'{joint}')
            
            ax_err.set_title('Tracking Errors', fontweight='bold')
            ax_err.set_xlabel('Time [s]')
            ax_err.set_ylabel('Error [°]')
            ax_err.axhline(y=0, color='black', linestyle='-', alpha=0.5, linewidth=0.8)
            ax_err.grid(True, alpha=0.3)
            if len(joint_names) <= 4:
                ax_err.legend(fontsize=7)
        
        # Performance Summary
        if show_des and show_act:
            ax_summary = self.fig.add_subplot(gs[1, 1])
            ax_summary.axis('off')
            
            # Calculate performance metrics
            metrics_text = "Performance Summary\n" + "="*20 + "\n"
            total_rmse = 0
            for joint in joint_names[:4]:  # Show first 4 joints to avoid clutter
                if f'des_{joint}' in df.columns and f'act_{joint}' in df.columns:
                    error = df[f'act_{joint}'] - df[f'des_{joint}']
                    rmse = np.sqrt(np.mean(error**2))
                    max_err = np.max(np.abs(error))
                    total_rmse += rmse
                    
                    short_name = joint.replace('_Revolute-', '').replace('Base_', 'B').replace('Arm-', 'A')
                    metrics_text += f"{short_name}:\n"
                    metrics_text += f"  RMSE: {np.degrees(rmse):.2f}°\n"
                    metrics_text += f"  Max: {np.degrees(max_err):.2f}°\n\n"
            
            metrics_text += f"Overall RMSE: {np.degrees(total_rmse):.1f}°\n"
            
            # Add effort statistics
            if show_eff:
                metrics_text += "\nEffort Statistics:\n"
                total_effort = 0
                for joint in joint_names:
                    if f'eff_{joint}' in df.columns:
                        effort_rms = np.sqrt(np.mean(df[f'eff_{joint}']**2))
                        total_effort += effort_rms
                metrics_text += f"Total RMS: {total_effort:.1f} N⋅m\n"
            
            metrics_text += f"Duration: {t.iloc[-1]:.1f} s\n"
            metrics_text += f"Samples: {len(df)}"
            
            ax_summary.text(0.05, 0.95, metrics_text, transform=ax_summary.transAxes,
                          verticalalignment='top', fontfamily='monospace', fontsize=8,
                          bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.8))
        
        # XY Trajectory 
        if show_ee and all(col in df.columns for col in ['ee_act_x', 'ee_act_y', 'ee_act_z']):
            ax_traj = self.fig.add_subplot(gs[1, 2])
            x, y, z = df['ee_act_x'] * 1000, df['ee_act_y'] * 1000, df['ee_act_z'] * 1000
            
            # Plot trajectory with Z as color
            if len(set(z)) > 1:  # Only if Z varies
                scatter = ax_traj.scatter(x, y, c=z, cmap='viridis', s=12, alpha=0.7)
                try:
                    cbar = self.fig.colorbar(scatter, ax=ax_traj, shrink=0.8)
                    cbar.set_label('Z [mm]', fontsize=8)
                except:
                    pass  # Skip colorbar if it fails
            else:
                ax_traj.plot(x, y, 'b-', linewidth=2, alpha=0.8)
            
            # Mark start and end points
            ax_traj.scatter(x.iloc[0], y.iloc[0], color='green', s=80, marker='o', label='Start', zorder=5, edgecolor='white')
            ax_traj.scatter(x.iloc[-1], y.iloc[-1], color='red', s=80, marker='s', label='End', zorder=5, edgecolor='white')
            
            ax_traj.set_title('XY Trajectory', fontweight='bold')
            ax_traj.set_xlabel('X [mm]')
            ax_traj.set_ylabel('Y [mm]')
            ax_traj.legend(fontsize=7)
            ax_traj.grid(True, alpha=0.3)
            ax_traj.axis('equal')
        
        # Add overall title
        self.fig.suptitle(f'Robot Motion Analysis - {len(df)} samples, {t.iloc[-1]:.1f}s duration', 
                         fontsize=14, fontweight='bold', y=0.95)
        
        self.canvas.draw_idle()


class LogViewer:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title('Robot Motion Log Viewer')
        self.root.geometry("400x600")

        self.df: pd.DataFrame | None = None
        self.file_paths: list[Path] = []
        self.joint_names: list[str] = []

        # Display options
        self.show_desired = tk.BooleanVar(value=True)
        self.show_actual = tk.BooleanVar(value=True)
        self.show_efforts = tk.BooleanVar(value=True)
        self.show_ee = tk.BooleanVar(value=True)
        self.max_points_var = tk.IntVar(value=2000)

        self._setup_ui()
        
        # Main plotting window (created lazily)
        self.plot_windows: dict[str, PlotWindow] = {}

    def _setup_ui(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        # File selection section
        file_frame = ttk.LabelFrame(main, text='Data Files', padding=8)
        file_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        ttk.Button(file_frame, text='📁 Open CSV Files...', command=self.on_open_csv).pack(fill=tk.X, pady=(0, 5))
        
        self.files_list = tk.Listbox(file_frame, height=8, font=('Consolas', 9))
        self.files_list.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
        self.files_list.bind('<<ListboxSelect>>', self.on_select_file)
        
        # File info label
        self.file_info_label = ttk.Label(file_frame, text="No file selected", foreground="gray")
        self.file_info_label.pack(fill=tk.X)

        # Display options section
        opts_frame = ttk.LabelFrame(main, text='Display Options', padding=8)
        opts_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Create two columns for checkboxes
        opts_left = ttk.Frame(opts_frame)
        opts_left.pack(side=tk.LEFT, fill=tk.X, expand=True)
        opts_right = ttk.Frame(opts_frame)
        opts_right.pack(side=tk.RIGHT, fill=tk.X, expand=True)
        
        ttk.Checkbutton(opts_left, text='🎯 Desired Positions', variable=self.show_desired).pack(anchor='w')
        ttk.Checkbutton(opts_left, text='📍 Actual Positions', variable=self.show_actual).pack(anchor='w')
        ttk.Checkbutton(opts_right, text='⚡ Joint Efforts', variable=self.show_efforts).pack(anchor='w')
        ttk.Checkbutton(opts_right, text='🔧 End-Effector', variable=self.show_ee).pack(anchor='w')

        # Performance options
        perf_frame = ttk.Frame(opts_frame)
        perf_frame.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(perf_frame, text='Max points:').pack(side=tk.LEFT)
        ttk.Entry(perf_frame, textvariable=self.max_points_var, width=8).pack(side=tk.LEFT, padx=(5, 10))
        ttk.Button(perf_frame, text='🔄 Refresh Plot', command=self.render).pack(side=tk.LEFT)

        # Action buttons
        action_frame = ttk.Frame(main)
        action_frame.pack(fill=tk.X)
        
        ttk.Button(action_frame, text='📊 Show Analysis', command=self.render, 
                  style='Accent.TButton').pack(fill=tk.X, pady=(0, 5))
        ttk.Button(action_frame, text='📋 Export Summary', command=self.export_summary).pack(fill=tk.X)

    def on_open_csv(self) -> None:
        paths = filedialog.askopenfilenames(
            title='Select CSV Motion Log Files', 
            filetypes=[('CSV files', '*.csv'), ('All files', '*.*')],
            initialdir='./log_data'
        )
        if not paths:
            return
        
        self.file_paths = [Path(p) for p in sorted(paths)]
        self.files_list.delete(0, tk.END)
        
        for p in self.file_paths:
            # Display filename with size info
            size_kb = p.stat().st_size / 1024
            self.files_list.insert(tk.END, f"{p.name} ({size_kb:.1f} KB)")
        
        if self.file_paths:
            self.files_list.selection_set(0)
            self.on_select_file()

    def on_select_file(self, _evt=None) -> None:
        sel = self.files_list.curselection()
        if not sel:
            return
        
        idx = sel[0]
        try:
            self.load_csv(self.file_paths[idx])
            self.update_file_info()
            if self.plot_window and self._window_alive(self.plot_window.win):
                self.render()
        except Exception as e:
            messagebox.showerror('Load Error', f'Failed to load file:\n{e}')

    def load_csv(self, path: Path) -> None:
        """Load CSV file and extract joint information."""
        df = pd.read_csv(path)
        
        if 't' not in df.columns:
            raise ValueError('CSV file must contain time column "t"')
        
        self.df = df
        
        # Extract joint names from column headers
        # Look for joint names that have both desired and actual columns
        joint_candidates = set()
        for col in df.columns:
            if col.startswith('des_'):
                joint_name = col[4:]  # Remove 'des_' prefix
                if f'act_{joint_name}' in df.columns:
                    joint_candidates.add(joint_name)
        
        self.joint_names = sorted(list(joint_candidates))
        
        if not self.joint_names:
            raise ValueError('No valid joint data found (need both des_X and act_X columns)')

    def update_file_info(self) -> None:
        """Update the file information display."""
        if self.df is None:
            self.file_info_label.config(text="No file selected", foreground="gray")
            return
        
        duration = self.df['t'].iloc[-1] - self.df['t'].iloc[0]
        sample_rate = len(self.df) / duration if duration > 0 else 0
        
        info_text = (f"📊 {len(self.df)} samples | "
                    f"⏱️ {duration:.1f}s | "
                    f"📈 {sample_rate:.0f} Hz | "
                    f"🔧 {len(self.joint_names)} joints")
        
        self.file_info_label.config(text=info_text, foreground="darkblue")

    def render(self) -> None:
        """Render the comprehensive motion analysis plot."""
        if self.df is None or not self.joint_names:
            messagebox.showwarning('No Data', 'Please load a CSV file first.')
            return
        
        # Create or update plot window
        if self.plot_window is None or not self._window_alive(self.plot_window.win):
            self.plot_window = MainPlotWindow(self.root, 'Robot Motion Analysis')
        
        # Prepare data (downsample if needed for performance)
        df = self.df.copy()
        max_points = max(100, int(self.max_points_var.get() or 2000))
        
        if len(df) > max_points:
            # Smart downsampling: keep more points at the beginning and end
            indices = np.unique(np.concatenate([
                np.linspace(0, len(df) - 1, max_points * 0.8).astype(int),
                np.arange(min(50, len(df))),  # First 50 points
                np.arange(max(0, len(df) - 50), len(df))  # Last 50 points
            ]))
            df = df.iloc[indices].reset_index(drop=True)
        
        # Render the comprehensive analysis
        self.plot_window.render_comprehensive_analysis(
            df=df,
            joint_names=self.joint_names,
            show_des=self.show_desired.get(),
            show_act=self.show_actual.get(),
            show_eff=self.show_efforts.get(),
            show_ee=self.show_ee.get()
        )
        
        # Bring plot window to front
        self.plot_window.win.lift()
        self.plot_window.win.focus_force()

    def export_summary(self) -> None:
        """Export motion analysis summary to text file."""
        if self.df is None:
            messagebox.showwarning('No Data', 'Please load a CSV file first.')
            return
        
        try:
            output_path = filedialog.asksaveasfilename(
                title='Export Summary',
                defaultextension='.txt',
                filetypes=[('Text files', '*.txt'), ('All files', '*.*')]
            )
            
            if not output_path:
                return
            
            # Generate comprehensive summary
            summary_lines = []
            summary_lines.append("ROBOT MOTION ANALYSIS SUMMARY")
            summary_lines.append("=" * 40)
            summary_lines.append(f"File: {self.file_paths[self.files_list.curselection()[0]].name}")
            summary_lines.append(f"Duration: {self.df['t'].iloc[-1]:.2f} seconds")
            summary_lines.append(f"Samples: {len(self.df)}")
            summary_lines.append(f"Sample Rate: {len(self.df) / self.df['t'].iloc[-1]:.1f} Hz")
            summary_lines.append("")
            
            # Joint analysis
            summary_lines.append("JOINT TRACKING PERFORMANCE:")
            summary_lines.append("-" * 30)
            
            for joint in self.joint_names:
                if f'des_{joint}' in self.df.columns and f'act_{joint}' in self.df.columns:
                    error = self.df[f'act_{joint}'] - self.df[f'des_{joint}']
                    rmse = np.sqrt(np.mean(error**2))
                    max_error = np.max(np.abs(error))
                    
                    summary_lines.append(f"{joint}:")
                    summary_lines.append(f"  RMSE: {np.degrees(rmse):.3f} degrees")
                    summary_lines.append(f"  Max Error: {np.degrees(max_error):.3f} degrees")
                    
                    if f'eff_{joint}' in self.df.columns:
                        effort_rms = np.sqrt(np.mean(self.df[f'eff_{joint}']**2))
                        effort_max = np.max(np.abs(self.df[f'eff_{joint}']))
                        summary_lines.append(f"  RMS Effort: {effort_rms:.2f} N⋅m")
                        summary_lines.append(f"  Max Effort: {effort_max:.2f} N⋅m")
                    summary_lines.append("")
            
            # End-effector analysis
            if all(col in self.df.columns for col in ['ee_act_x', 'ee_act_y', 'ee_act_z']):
                summary_lines.append("END-EFFECTOR TRAJECTORY:")
                summary_lines.append("-" * 25)
                
                x_range = (self.df['ee_act_x'].max() - self.df['ee_act_x'].min()) * 1000
                y_range = (self.df['ee_act_y'].max() - self.df['ee_act_y'].min()) * 1000  
                z_range = (self.df['ee_act_z'].max() - self.df['ee_act_z'].min()) * 1000
                
                summary_lines.append(f"Workspace Usage:")
                summary_lines.append(f"  X range: {x_range:.1f} mm")
                summary_lines.append(f"  Y range: {y_range:.1f} mm")
                summary_lines.append(f"  Z range: {z_range:.1f} mm")
                summary_lines.append(f"  Total distance: {np.sqrt(x_range**2 + y_range**2 + z_range**2):.1f} mm")
            
            # Write summary file
            with open(output_path, 'w') as f:
                f.write('\n'.join(summary_lines))
            
            messagebox.showinfo('Export Complete', f'Summary exported to:\n{output_path}')
            
        except Exception as e:
            messagebox.showerror('Export Error', f'Failed to export summary:\n{e}')

    @staticmethod
    def _window_alive(win: tk.Toplevel) -> bool:
        try:
            return bool(win.winfo_exists())
        except Exception:
            return False


def main() -> None:
    root = tk.Tk()
    app = LogViewer(root)
    if len(sys.argv) > 1:
        try:
            paths = []
            for arg in sys.argv[1:]:
                p = Path(arg)
                if p.is_dir():
                    paths += sorted(p.glob('*.csv'))
                elif p.is_file():
                    paths.append(p)
            if paths:
                app.file_paths = [Path(p) for p in paths]
                app.files_list.delete(0, tk.END)
                for p in app.file_paths:
                    app.files_list.insert(tk.END, str(p))
                app.files_list.selection_set(0)
                app.on_select_file()
        except Exception:
            pass
    root.mainloop()


if __name__ == '__main__':
    main()


