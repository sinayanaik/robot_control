#!/usr/bin/env python3

import sys
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

# Import 3D plotting capability with robust fallback
HAS_3D = False
try:
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        from mpl_toolkits.mplot3d import Axes3D
        # Test if 3d projection is actually available
        fig_test = plt.figure()
        ax_test = fig_test.add_subplot(111, projection='3d')
        plt.close(fig_test)
        HAS_3D = True
except (ImportError, AttributeError, ValueError, KeyError):
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
                stats_text = f'RMSE: {np.degrees(rmse):.2f}°\\nMax Error: {np.degrees(max_error):.2f}°'
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
            stats_text = f'RMS: {rms_effort:.2f} N⋅m\\nMax: {max_effort_joint:.2f} N⋅m\\nMean: {mean_effort:.2f} N⋅m'
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
            stats_text = f'Max: {max_speed:.1f} mm/s\\nAvg: {avg_speed:.1f} mm/s'
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
            ax.text(0.5, 0.5, 'No tracking data available\\n(Need both desired and actual positions)', 
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


class LogViewer:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Robot Motion Data Analyzer")
        self.root.geometry("600x500")
        
        self.df: pd.DataFrame | None = None
        self.current_file: Path | None = None
        self.joint_names: list[str] = []
        self.plot_windows: dict[str, PlotWindow] = {}
        
        self._setup_ui()

    def _setup_ui(self) -> None:
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # File selection frame
        file_frame = ttk.LabelFrame(main_frame, text="Data File", padding="10")
        file_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Button(file_frame, text="Open CSV File", command=self.on_open_csv).grid(row=0, column=0, padx=(0, 10))
        
        self.file_var = tk.StringVar(value="No file selected")
        file_label = ttk.Label(file_frame, textvariable=self.file_var, font=('TkDefaultFont', 9))
        file_label.grid(row=0, column=1, sticky=tk.W)
        
        # File info frame
        info_frame = ttk.LabelFrame(main_frame, text="File Information", padding="10")
        info_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.info_text = tk.Text(info_frame, height=6, width=70, wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(info_frame, orient=tk.VERTICAL, command=self.info_text.yview)
        self.info_text.configure(yscrollcommand=scrollbar.set)
        self.info_text.grid(row=0, column=0, sticky=(tk.W, tk.E))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # Analysis options frame
        options_frame = ttk.LabelFrame(main_frame, text="Analysis Options", padding="10")
        options_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Plot type buttons in a grid
        plot_buttons = [
            ("Joint Positions", self.plot_joint_positions, 0, 0),
            ("Joint Efforts", self.plot_joint_efforts, 0, 1),
            ("End-Effector Analysis", self.plot_end_effector, 1, 0),
            ("Tracking Performance", self.plot_tracking_performance, 1, 1),
            ("Overview Dashboard", self.plot_overview, 2, 0),
            ("Export Summary", self.export_summary, 2, 1)
        ]
        
        for text, command, row, col in plot_buttons:
            btn = ttk.Button(options_frame, text=text, command=command, width=20)
            btn.grid(row=row, column=col, padx=5, pady=5, sticky=tk.W)
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        info_frame.columnconfigure(0, weight=1)

    def on_open_csv(self) -> None:
        filename = filedialog.askopenfilename(
            title="Select CSV Log File",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialdir="./log_data"
        )
        if filename:
            self.load_csv(Path(filename))

    def load_csv(self, path: Path) -> None:
        try:
            self.df = pd.read_csv(path)
            self.current_file = path
            self.file_var.set(f"Loaded: {path.name}")
            
            # Extract joint names from column headers
            self.joint_names = []
            for col in self.df.columns:
                if col.startswith('des_') or col.startswith('act_'):
                    joint_name = col[4:]  # Remove 'des_' or 'act_' prefix
                    if joint_name not in self.joint_names:
                        self.joint_names.append(joint_name)
            
            self.update_file_info()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load CSV file:\\n{str(e)}")

    def update_file_info(self) -> None:
        if self.df is None:
            return
        
        # Calculate comprehensive statistics
        duration = self.df['t'].max() - self.df['t'].min()
        samples = len(self.df)
        sample_rate = samples / duration if duration > 0 else 0
        
        # Joint analysis
        joint_info = []
        for joint in self.joint_names:
            if f'act_{joint}' in self.df.columns:
                pos_range = np.degrees(self.df[f'act_{joint}'].max() - self.df[f'act_{joint}'].min())
                joint_info.append(f"  {joint}: {pos_range:.1f}° range")
        
        # Effort analysis
        effort_info = ""
        effort_cols = [col for col in self.df.columns if col.startswith('eff_')]
        if effort_cols:
            total_efforts = []
            for col in effort_cols:
                efforts = self.df[col].dropna()
                total_efforts.extend(efforts.tolist())
            if total_efforts:
                max_effort = np.max(np.abs(total_efforts))
                rms_effort = np.sqrt(np.mean(np.array(total_efforts)**2))
                effort_info = f"\\nEffort Analysis:\\n  Max effort: {max_effort:.2f} N⋅m\\n  RMS effort: {rms_effort:.2f} N⋅m"
        
        # End-effector analysis
        ee_info = ""
        ee_cols = ['ee_act_x', 'ee_act_y', 'ee_act_z']
        if all(col in self.df.columns for col in ee_cols):
            workspace_x = (self.df['ee_act_x'].max() - self.df['ee_act_x'].min()) * 1000
            workspace_y = (self.df['ee_act_y'].max() - self.df['ee_act_y'].min()) * 1000
            workspace_z = (self.df['ee_act_z'].max() - self.df['ee_act_z'].min()) * 1000
            ee_info = f"\\nEnd-Effector Workspace: {workspace_x:.1f} × {workspace_y:.1f} × {workspace_z:.1f} mm³"
        
        info_text = f"""Data Summary:
Duration: {duration:.2f} seconds
Samples: {samples} ({sample_rate:.1f} Hz)
Joints: {len(self.joint_names)} ({', '.join(self.joint_names)})

Joint Ranges:
{chr(10).join(joint_info)}{effort_info}{ee_info}

Columns: {len(self.df.columns)} total
Available Data: {', '.join([col.split('_')[0] for col in self.df.columns if '_' in col])}"""
        
        self.info_text.delete(1.0, tk.END)
        self.info_text.insert(1.0, info_text)

    def plot_joint_positions(self) -> None:
        if self.df is None or not self.joint_names:
            messagebox.showwarning("Warning", "No joint data available")
            return
        
        window_key = "positions"
        if window_key in self.plot_windows:
            self.plot_windows[window_key].win.lift()
        else:
            self.plot_windows[window_key] = PlotWindow(self.root, "Joint Position Analysis", "1200x800")
            self.plot_windows[window_key].plot_joint_positions(self.df, self.joint_names, True, True)

    def plot_joint_efforts(self) -> None:
        if self.df is None or not self.joint_names:
            messagebox.showwarning("Warning", "No data available")
            return
        
        window_key = "efforts"
        if window_key in self.plot_windows:
            self.plot_windows[window_key].win.lift()
        else:
            self.plot_windows[window_key] = PlotWindow(self.root, "Joint Effort Analysis", "1200x800")
            self.plot_windows[window_key].plot_joint_efforts(self.df, self.joint_names)

    def plot_end_effector(self) -> None:
        if self.df is None:
            messagebox.showwarning("Warning", "No data available")
            return
        
        window_key = "end_effector"
        if window_key in self.plot_windows:
            self.plot_windows[window_key].win.lift()
        else:
            self.plot_windows[window_key] = PlotWindow(self.root, "End-Effector Analysis", "1000x800")
            self.plot_windows[window_key].plot_end_effector_analysis(self.df)

    def plot_tracking_performance(self) -> None:
        if self.df is None or not self.joint_names:
            messagebox.showwarning("Warning", "No joint data available")
            return
        
        window_key = "tracking"
        if window_key in self.plot_windows:
            self.plot_windows[window_key].win.lift()
        else:
            self.plot_windows[window_key] = PlotWindow(self.root, "Tracking Performance Analysis", "1000x700")
            self.plot_windows[window_key].plot_tracking_performance(self.df, self.joint_names)

    def plot_overview(self) -> None:
        """Create a comprehensive overview with key metrics in one window."""
        if self.df is None:
            messagebox.showwarning("Warning", "No data available")
            return
        
        window_key = "overview"
        if window_key in self.plot_windows:
            self.plot_windows[window_key].win.lift()
            return
        
        # Create overview window
        self.plot_windows[window_key] = PlotWindow(self.root, "Motion Analysis Overview", "1400x900")
        plot_win = self.plot_windows[window_key]
        plot_win.clear()
        
        # Create 2x3 layout for overview
        fig = plot_win.fig
        
        # 1. Joint position summary (top-left)
        ax1 = fig.add_subplot(2, 3, 1)
        t = self.df['t']
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        for i, joint in enumerate(self.joint_names[:6]):  # Limit to 6 joints for readability
            if f'act_{joint}' in self.df.columns:
                ax1.plot(t, np.degrees(self.df[f'act_{joint}']), 
                        color=colors[i % len(colors)], label=joint[:8], linewidth=1.5)
        ax1.set_title('Joint Positions', fontweight='bold')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Angle (°)')
        ax1.legend(fontsize=8)
        ax1.grid(True, alpha=0.3)
        
        # 2. Effort summary (top-center)
        ax2 = fig.add_subplot(2, 3, 2)
        effort_cols = [f'eff_{joint}' for joint in self.joint_names if f'eff_{joint}' in self.df.columns]
        if effort_cols:
            for i, col in enumerate(effort_cols[:6]):
                joint = col[4:]  # Remove 'eff_' prefix
                ax2.plot(t, self.df[col], color=colors[i % len(colors)], 
                        label=joint[:8], linewidth=1.5, alpha=0.8)
            ax2.axhline(y=0, color='black', linestyle='-', alpha=0.3)
        ax2.set_title('Joint Efforts', fontweight='bold')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Torque (N⋅m)')
        ax2.legend(fontsize=8)
        ax2.grid(True, alpha=0.3)
        
        # 3. End-effector trajectory (top-right)
        ax3 = fig.add_subplot(2, 3, 3)
        if all(col in self.df.columns for col in ['ee_act_x', 'ee_act_y', 'ee_act_z']):
            x, y = self.df['ee_act_x'] * 1000, self.df['ee_act_y'] * 1000
            colors_traj = plt.cm.viridis(np.linspace(0, 1, len(t)))
            ax3.scatter(x, y, c=colors_traj, s=8, alpha=0.7)
            ax3.plot(x, y, 'b-', alpha=0.3, linewidth=1)
            ax3.scatter(x.iloc[0], y.iloc[0], color='green', s=50, marker='o')
            ax3.scatter(x.iloc[-1], y.iloc[-1], color='red', s=50, marker='s')
        ax3.set_title('EE Trajectory (XY)', fontweight='bold')
        ax3.set_xlabel('X (mm)')
        ax3.set_ylabel('Y (mm)')
        ax3.grid(True, alpha=0.3)
        ax3.axis('equal')
        
        # 4. Tracking errors (bottom-left)
        ax4 = fig.add_subplot(2, 3, 4)
        for i, joint in enumerate(self.joint_names[:4]):  # Limit for readability
            if f'des_{joint}' in self.df.columns and f'act_{joint}' in self.df.columns:
                error = np.degrees(self.df[f'act_{joint}'] - self.df[f'des_{joint}'])
                ax4.plot(t, error, color=colors[i % len(colors)], 
                        label=joint[:8], linewidth=1.5, alpha=0.8)
        ax4.axhline(y=0, color='black', linestyle='-', alpha=0.3)
        ax4.set_title('Tracking Errors', fontweight='bold')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Error (°)')
        ax4.legend(fontsize=8)
        ax4.grid(True, alpha=0.3)
        
        # 5. Performance metrics (bottom-center)
        ax5 = fig.add_subplot(2, 3, 5)
        performance_data = []
        joint_labels = []
        for joint in self.joint_names:
            if f'des_{joint}' in self.df.columns and f'act_{joint}' in self.df.columns:
                error = self.df[f'act_{joint}'] - self.df[f'des_{joint}']
                rmse = np.degrees(np.sqrt(np.mean(error**2)))
                performance_data.append(rmse)
                joint_labels.append(joint[:8])
        
        if performance_data:
            bars = ax5.bar(range(len(performance_data)), performance_data, 
                          color=['lightblue', 'lightcoral', 'lightgreen', 'wheat', 'plum', 'lightsalmon'][:len(performance_data)])
            for i, (bar, value) in enumerate(zip(bars, performance_data)):
                ax5.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001,
                        f'{value:.2f}°', ha='center', va='bottom', fontsize=8)
            ax5.set_xticks(range(len(joint_labels)))
            ax5.set_xticklabels(joint_labels, rotation=45, ha='right')
        ax5.set_title('Tracking RMSE', fontweight='bold')
        ax5.set_ylabel('RMSE (°)')
        ax5.grid(True, alpha=0.3, axis='y')
        
        # 6. System summary (bottom-right)
        ax6 = fig.add_subplot(2, 3, 6)
        ax6.axis('off')
        
        # Calculate summary statistics
        duration = self.df['t'].max() - self.df['t'].min()
        samples = len(self.df)
        sample_rate = samples / duration if duration > 0 else 0
        
        if effort_cols:
            all_efforts = []
            for col in effort_cols:
                efforts = self.df[col].dropna()
                all_efforts.extend(efforts.tolist())
            max_effort = np.max(np.abs(all_efforts)) if all_efforts else 0
            rms_effort = np.sqrt(np.mean(np.array(all_efforts)**2)) if all_efforts else 0
        else:
            max_effort = rms_effort = 0
        
        if performance_data:
            overall_rmse = np.sqrt(np.mean(np.array(performance_data)**2))
        else:
            overall_rmse = 0
        
        summary_text = f"""Motion Summary

Duration: {duration:.2f} s
Samples: {samples}
Rate: {sample_rate:.1f} Hz

Performance:
RMSE: {overall_rmse:.2f}°
Max Effort: {max_effort:.1f} N⋅m
RMS Effort: {rms_effort:.1f} N⋅m

Joints: {len(self.joint_names)}
File: {self.current_file.name if self.current_file else 'Unknown'}"""
        
        ax6.text(0.1, 0.9, summary_text, transform=ax6.transAxes,
                fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgray', alpha=0.8))
        
        fig.suptitle(f'Robot Motion Analysis Overview - {self.current_file.name if self.current_file else "Data"}', 
                    fontsize=16, fontweight='bold')
        plot_win.draw()

    def export_summary(self) -> None:
        if self.df is None:
            messagebox.showwarning("Warning", "No data available")
            return
        
        # Export comprehensive analysis to text file
        output_path = filedialog.asksaveasfilename(
            title="Save Analysis Summary",
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
        
        if not output_path:
            return
        
        try:
            with open(output_path, 'w') as f:
                f.write(f"Robot Motion Analysis Summary\\n")
                f.write(f"{'='*50}\\n")
                f.write(f"File: {self.current_file.name if self.current_file else 'Unknown'}\\n")
                f.write(f"Generated: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}\\n\\n")
                
                # Basic statistics
                duration = self.df['t'].max() - self.df['t'].min()
                samples = len(self.df)
                sample_rate = samples / duration if duration > 0 else 0
                
                f.write(f"Data Overview:\\n")
                f.write(f"  Duration: {duration:.2f} seconds\\n")
                f.write(f"  Samples: {samples}\\n")
                f.write(f"  Sample Rate: {sample_rate:.1f} Hz\\n")
                f.write(f"  Joints: {len(self.joint_names)}\\n\\n")
                
                # Joint analysis
                f.write(f"Joint Analysis:\\n")
                for joint in self.joint_names:
                    if f'act_{joint}' in self.df.columns:
                        pos_range = np.degrees(self.df[f'act_{joint}'].max() - self.df[f'act_{joint}'].min())
                        pos_mean = np.degrees(self.df[f'act_{joint}'].mean())
                        f.write(f"  {joint}:\\n")
                        f.write(f"    Range: {pos_range:.2f}°\\n")
                        f.write(f"    Mean: {pos_mean:.2f}°\\n")
                        
                        if f'des_{joint}' in self.df.columns:
                            error = self.df[f'act_{joint}'] - self.df[f'des_{joint}']
                            rmse = np.degrees(np.sqrt(np.mean(error**2)))
                            max_error = np.degrees(np.max(np.abs(error)))
                            f.write(f"    Tracking RMSE: {rmse:.3f}°\\n")
                            f.write(f"    Max Error: {max_error:.3f}°\\n")
                        
                        if f'eff_{joint}' in self.df.columns:
                            effort_data = self.df[f'eff_{joint}']
                            max_effort = np.max(np.abs(effort_data))
                            rms_effort = np.sqrt(np.mean(effort_data**2))
                            f.write(f"    Max Effort: {max_effort:.3f} N⋅m\\n")
                            f.write(f"    RMS Effort: {rms_effort:.3f} N⋅m\\n")
                        f.write(f"\\n")
                
                # End-effector analysis
                ee_cols = ['ee_act_x', 'ee_act_y', 'ee_act_z']
                if all(col in self.df.columns for col in ee_cols):
                    f.write(f"End-Effector Analysis:\\n")
                    workspace_x = (self.df['ee_act_x'].max() - self.df['ee_act_x'].min()) * 1000
                    workspace_y = (self.df['ee_act_y'].max() - self.df['ee_act_y'].min()) * 1000
                    workspace_z = (self.df['ee_act_z'].max() - self.df['ee_act_z'].min()) * 1000
                    f.write(f"  Workspace: {workspace_x:.1f} × {workspace_y:.1f} × {workspace_z:.1f} mm³\\n")
                    
                    # Calculate path length
                    if len(self.df) > 1:
                        dx = np.diff(self.df['ee_act_x'] * 1000)
                        dy = np.diff(self.df['ee_act_y'] * 1000)
                        dz = np.diff(self.df['ee_act_z'] * 1000)
                        path_length = np.sum(np.sqrt(dx**2 + dy**2 + dz**2))
                        f.write(f"  Path Length: {path_length:.1f} mm\\n")
                        
                        # Average speed
                        dt = np.diff(self.df['t'])
                        distances = np.sqrt(dx**2 + dy**2 + dz**2)
                        speeds = distances / dt
                        avg_speed = np.mean(speeds)
                        max_speed = np.max(speeds)
                        f.write(f"  Average Speed: {avg_speed:.1f} mm/s\\n")
                        f.write(f"  Max Speed: {max_speed:.1f} mm/s\\n")
                
            messagebox.showinfo("Success", f"Analysis summary exported to {output_path}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to export summary:\\n{str(e)}")


def main() -> None:
    root = tk.Tk()
    app = LogViewer(root)
    if len(sys.argv) > 1:
        csv_path = Path(sys.argv[1])
        if csv_path.exists():
            app.load_csv(csv_path)
        else:
            messagebox.showerror("Error", f"File not found: {csv_path}")
    root.mainloop()


if __name__ == '__main__':
    main()