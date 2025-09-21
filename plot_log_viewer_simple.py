#!/usr/bin/env python3
"""
Simplified Robot Motion Log Viewer
Compatible with various matplotlib versions
"""
import sys
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend explicitly
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

# Set simple style
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
})

class RobotLogViewer:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title('Robot Motion Log Viewer')
        self.root.geometry("500x700")

        self.df: pd.DataFrame | None = None
        self.file_paths: list[Path] = []
        self.joint_names: list[str] = []

        # Display options
        self.show_desired = tk.BooleanVar(value=True)
        self.show_actual = tk.BooleanVar(value=True)
        self.show_efforts = tk.BooleanVar(value=True)
        self.show_ee = tk.BooleanVar(value=True)
        self.max_points_var = tk.IntVar(value=1000)

        self._setup_ui()
        
        # Plotting window (created when needed)
        self.plot_window: tk.Toplevel | None = None
        self.fig: Figure | None = None
        self.canvas = None

    def _setup_ui(self) -> None:
        main = ttk.Frame(self.root, padding=15)
        main.pack(fill=tk.BOTH, expand=True)

        # Title
        title_label = ttk.Label(main, text="🤖 Robot Motion Log Viewer", font=('Arial', 14, 'bold'))
        title_label.pack(pady=(0, 15))

        # File selection section
        file_frame = ttk.LabelFrame(main, text='📁 Data Files', padding=10)
        file_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        ttk.Button(file_frame, text='Open CSV Files...', command=self.on_open_csv).pack(fill=tk.X, pady=(0, 8))
        
        self.files_list = tk.Listbox(file_frame, height=6, font=('Consolas', 9))
        self.files_list.pack(fill=tk.BOTH, expand=True, pady=(0, 8))
        self.files_list.bind('<<ListboxSelect>>', self.on_select_file)
        
        # File info
        self.file_info_label = ttk.Label(file_frame, text="No file selected", foreground="gray")
        self.file_info_label.pack(fill=tk.X)

        # Display options
        opts_frame = ttk.LabelFrame(main, text='📊 Display Options', padding=10)
        opts_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Checkboxes in a grid
        cb_frame = ttk.Frame(opts_frame)
        cb_frame.pack(fill=tk.X)
        
        ttk.Checkbutton(cb_frame, text='🎯 Desired', variable=self.show_desired).grid(row=0, column=0, sticky='w', padx=5)
        ttk.Checkbutton(cb_frame, text='📍 Actual', variable=self.show_actual).grid(row=0, column=1, sticky='w', padx=5)
        ttk.Checkbutton(cb_frame, text='⚡ Efforts', variable=self.show_efforts).grid(row=1, column=0, sticky='w', padx=5)
        ttk.Checkbutton(cb_frame, text='🔧 End-Eff', variable=self.show_ee).grid(row=1, column=1, sticky='w', padx=5)

        # Max points setting
        perf_frame = ttk.Frame(opts_frame)
        perf_frame.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(perf_frame, text='Max points:').pack(side=tk.LEFT)
        ttk.Entry(perf_frame, textvariable=self.max_points_var, width=8).pack(side=tk.LEFT, padx=(5, 0))

        # Action buttons
        action_frame = ttk.Frame(main)
        action_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(action_frame, text='📊 Show Analysis', command=self.show_plots).pack(fill=tk.X, pady=(0, 5))
        ttk.Button(action_frame, text='📋 Export Summary', command=self.export_summary).pack(fill=tk.X)

        # Status
        self.status_label = ttk.Label(main, text="Ready", foreground="blue")
        self.status_label.pack(fill=tk.X)

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
            self.status_label.config(text="Loading...", foreground="orange")
            self.root.update()
            
            self.load_csv(self.file_paths[idx])
            self.update_file_info()
            
            self.status_label.config(text="File loaded successfully", foreground="green")
        except Exception as e:
            self.status_label.config(text=f"Error: {str(e)[:50]}...", foreground="red")
            messagebox.showerror('Load Error', f'Failed to load file:\n{e}')

    def load_csv(self, path: Path) -> None:
        """Load CSV file and extract joint information."""
        df = pd.read_csv(path)
        
        if 't' not in df.columns:
            raise ValueError('CSV file must contain time column "t"')
        
        self.df = df
        
        # Extract joint names - look for joints with both desired and actual columns
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

    def show_plots(self) -> None:
        """Show the comprehensive motion analysis plots."""
        if self.df is None or not self.joint_names:
            messagebox.showwarning('No Data', 'Please load a CSV file first.')
            return

        # Create or reuse plot window
        if self.plot_window is None or not self._window_alive():
            self._create_plot_window()
        
        self.status_label.config(text="Generating plots...", foreground="orange")
        self.root.update()
        
        try:
            self._render_plots()
            self.plot_window.lift()
            self.plot_window.focus_force()
            self.status_label.config(text="Plots generated successfully", foreground="green")
        except Exception as e:
            self.status_label.config(text=f"Plot error: {str(e)[:50]}...", foreground="red")
            messagebox.showerror('Plot Error', f'Failed to generate plots:\n{e}')

    def _create_plot_window(self) -> None:
        """Create the plotting window."""
        self.plot_window = tk.Toplevel(self.root)
        self.plot_window.title('Robot Motion Analysis')
        self.plot_window.geometry("1400x900")
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(14, 9), tight_layout=True)
        self.fig.patch.set_facecolor('white')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_window)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Toolbar
        toolbar_frame = ttk.Frame(self.plot_window)
        toolbar_frame.pack(side=tk.BOTTOM, fill=tk.X)
        toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        toolbar.update()

    def _render_plots(self) -> None:
        """Render all the analysis plots."""
        self.fig.clear()
        
        # Downsample data if needed
        df = self.df.copy()
        max_points = max(100, int(self.max_points_var.get() or 1000))
        
        if len(df) > max_points:
            indices = np.linspace(0, len(df) - 1, max_points).astype(int)
            df = df.iloc[indices].reset_index(drop=True)
        
        t = df['t']
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        
        # Create 2x3 subplot layout
        ax1 = self.fig.add_subplot(2, 3, 1)  # Joint positions
        ax2 = self.fig.add_subplot(2, 3, 2)  # Joint efforts
        ax3 = self.fig.add_subplot(2, 3, 3)  # End-effector
        ax4 = self.fig.add_subplot(2, 3, 4)  # Tracking errors
        ax5 = self.fig.add_subplot(2, 3, 5)  # XY trajectory
        ax6 = self.fig.add_subplot(2, 3, 6)  # Performance summary
        
        # 1. Joint Positions
        if self.show_desired.get() or self.show_actual.get():
            for i, joint in enumerate(self.joint_names):
                color = colors[i % len(colors)]
                if self.show_desired.get() and f'des_{joint}' in df.columns:
                    ax1.plot(t, np.degrees(df[f'des_{joint}']), '--', 
                           color=color, alpha=0.7, label=f'{joint} (des)')
                if self.show_actual.get() and f'act_{joint}' in df.columns:
                    ax1.plot(t, np.degrees(df[f'act_{joint}']), '-', 
                           color=color, linewidth=2, label=f'{joint} (act)')
            
            ax1.set_title('Joint Positions', fontweight='bold')
            ax1.set_xlabel('Time [s]')
            ax1.set_ylabel('Angle [°]')
            ax1.grid(True, alpha=0.3)
            if len(self.joint_names) <= 3:
                ax1.legend(fontsize=7)

        # 2. Joint Efforts
        if self.show_efforts.get():
            for i, joint in enumerate(self.joint_names):
                if f'eff_{joint}' in df.columns:
                    color = colors[i % len(colors)]
                    ax2.plot(t, df[f'eff_{joint}'], '-', color=color, 
                           linewidth=2, label=f'{joint}')
            
            ax2.set_title('Joint Efforts', fontweight='bold')
            ax2.set_xlabel('Time [s]')
            ax2.set_ylabel('Torque [N⋅m]')
            ax2.grid(True, alpha=0.3)
            ax2.axhline(y=0, color='black', linestyle='-', alpha=0.3)
            if len(self.joint_names) <= 3:
                ax2.legend(fontsize=7)

        # 3. End-Effector Position
        if self.show_ee.get() and all(col in df.columns for col in ['ee_act_x', 'ee_act_y', 'ee_act_z']):
            ax3.plot(t, df['ee_act_x'] * 1000, 'b-', linewidth=2, label='X')
            ax3.plot(t, df['ee_act_y'] * 1000, 'r-', linewidth=2, label='Y')
            ax3.plot(t, df['ee_act_z'] * 1000, 'g-', linewidth=2, label='Z')
            ax3.set_title('End-Effector Position', fontweight='bold')
            ax3.set_xlabel('Time [s]')
            ax3.set_ylabel('Position [mm]')
            ax3.legend()
            ax3.grid(True, alpha=0.3)

        # 4. Tracking Errors
        if self.show_desired.get() and self.show_actual.get():
            for i, joint in enumerate(self.joint_names):
                if f'des_{joint}' in df.columns and f'act_{joint}' in df.columns:
                    error = np.degrees(df[f'act_{joint}'] - df[f'des_{joint}'])
                    color = colors[i % len(colors)]
                    ax4.plot(t, error, '-', color=color, alpha=0.8, 
                           linewidth=1.5, label=f'{joint}')
            
            ax4.set_title('Tracking Errors', fontweight='bold')
            ax4.set_xlabel('Time [s]')
            ax4.set_ylabel('Error [°]')
            ax4.axhline(y=0, color='black', linestyle='-', alpha=0.5)
            ax4.grid(True, alpha=0.3)
            if len(self.joint_names) <= 3:
                ax4.legend(fontsize=7)

        # 5. XY Trajectory
        if self.show_ee.get() and all(col in df.columns for col in ['ee_act_x', 'ee_act_y', 'ee_act_z']):
            x, y, z = df['ee_act_x'] * 1000, df['ee_act_y'] * 1000, df['ee_act_z'] * 1000
            
            # Plot with varying Z as color if Z changes
            if len(set(z)) > 1:
                scatter = ax5.scatter(x, y, c=z, cmap='viridis', s=15, alpha=0.7)
                try:
                    cbar = self.fig.colorbar(scatter, ax=ax5, shrink=0.6)
                    cbar.set_label('Z [mm]', fontsize=8)
                except:
                    pass
            else:
                ax5.plot(x, y, 'b-', linewidth=2, alpha=0.8)
            
            # Mark start and end
            ax5.scatter(x.iloc[0], y.iloc[0], color='green', s=100, marker='o', 
                       label='Start', zorder=5, edgecolor='white', linewidth=2)
            ax5.scatter(x.iloc[-1], y.iloc[-1], color='red', s=100, marker='s', 
                       label='End', zorder=5, edgecolor='white', linewidth=2)
            
            ax5.set_title('XY Trajectory', fontweight='bold')
            ax5.set_xlabel('X [mm]')
            ax5.set_ylabel('Y [mm]')
            ax5.legend(fontsize=8)
            ax5.grid(True, alpha=0.3)
            ax5.axis('equal')

        # 6. Performance Summary (Text)
        ax6.axis('off')
        if self.show_desired.get() and self.show_actual.get():
            summary_text = "Performance Summary\n" + "="*25 + "\n\n"
            
            total_rmse = 0
            for joint in self.joint_names[:4]:  # First 4 joints
                if f'des_{joint}' in df.columns and f'act_{joint}' in df.columns:
                    error = df[f'act_{joint}'] - df[f'des_{joint}']
                    rmse = np.sqrt(np.mean(error**2))
                    max_err = np.max(np.abs(error))
                    total_rmse += rmse
                    
                    short_name = joint.replace('_Revolute-', '').replace('Base_', 'B').replace('Arm-', 'A')
                    summary_text += f"{short_name}: RMSE={np.degrees(rmse):.1f}°, Max={np.degrees(max_err):.1f}°\n"
            
            summary_text += f"\nOverall RMSE: {np.degrees(total_rmse):.1f}°\n"
            
            if self.show_efforts.get():
                total_effort = sum(np.sqrt(np.mean(df[f'eff_{joint}']**2)) 
                                 for joint in self.joint_names 
                                 if f'eff_{joint}' in df.columns)
                summary_text += f"Total RMS Effort: {total_effort:.1f} N⋅m\n"
            
            summary_text += f"\nDuration: {t.iloc[-1]:.1f} s\n"
            summary_text += f"Samples: {len(df)}\n"
            summary_text += f"Sample Rate: {len(df)/t.iloc[-1]:.0f} Hz"
            
            ax6.text(0.05, 0.95, summary_text, transform=ax6.transAxes,
                    verticalalignment='top', fontfamily='monospace', fontsize=9,
                    bbox=dict(boxstyle="round,pad=0.5", facecolor="lightblue", alpha=0.8))

        # Overall title
        self.fig.suptitle(f'Robot Motion Analysis - {self.file_paths[self.files_list.curselection()[0]].name}', 
                         fontsize=16, fontweight='bold')
        
        self.canvas.draw()

    def export_summary(self) -> None:
        """Export motion analysis summary."""
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
            
            summary_lines = [
                "ROBOT MOTION ANALYSIS SUMMARY",
                "=" * 40,
                f"File: {self.file_paths[self.files_list.curselection()[0]].name}",
                f"Duration: {self.df['t'].iloc[-1]:.2f} seconds",
                f"Samples: {len(self.df)}",
                f"Sample Rate: {len(self.df) / self.df['t'].iloc[-1]:.1f} Hz",
                "",
                "JOINT PERFORMANCE:",
                "-" * 20
            ]
            
            for joint in self.joint_names:
                if f'des_{joint}' in self.df.columns and f'act_{joint}' in self.df.columns:
                    error = self.df[f'act_{joint}'] - self.df[f'des_{joint}']
                    rmse = np.sqrt(np.mean(error**2))
                    max_error = np.max(np.abs(error))
                    
                    summary_lines.extend([
                        f"{joint}:",
                        f"  RMSE: {np.degrees(rmse):.3f} degrees",
                        f"  Max Error: {np.degrees(max_error):.3f} degrees",
                    ])
                    
                    if f'eff_{joint}' in self.df.columns:
                        effort_rms = np.sqrt(np.mean(self.df[f'eff_{joint}']**2))
                        effort_max = np.max(np.abs(self.df[f'eff_{joint}']))
                        summary_lines.extend([
                            f"  RMS Effort: {effort_rms:.2f} N⋅m",
                            f"  Max Effort: {effort_max:.2f} N⋅m",
                        ])
                    summary_lines.append("")
            
            with open(output_path, 'w') as f:
                f.write('\n'.join(summary_lines))
            
            messagebox.showinfo('Export Complete', f'Summary exported to:\n{output_path}')
            self.status_label.config(text="Summary exported", foreground="green")
            
        except Exception as e:
            messagebox.showerror('Export Error', f'Failed to export summary:\n{e}')
            self.status_label.config(text="Export failed", foreground="red")

    def _window_alive(self) -> bool:
        """Check if plot window is still alive."""
        if self.plot_window is None:
            return False
        try:
            return bool(self.plot_window.winfo_exists())
        except Exception:
            return False


def main() -> None:
    root = tk.Tk()
    app = RobotLogViewer(root)
    
    # Load command line arguments
    if len(sys.argv) > 1:
        try:
            paths = []
            for arg in sys.argv[1:]:
                p = Path(arg)
                if p.is_dir():
                    paths.extend(sorted(p.glob('*.csv')))
                elif p.is_file() and p.suffix.lower() == '.csv':
                    paths.append(p)
            
            if paths:
                app.file_paths = paths
                app.files_list.delete(0, tk.END)
                for p in app.file_paths:
                    size_kb = p.stat().st_size / 1024
                    app.files_list.insert(tk.END, f"{p.name} ({size_kb:.1f} KB)")
                app.files_list.selection_set(0)
                app.on_select_file()
        except Exception as e:
            print(f"Warning: Could not load command line files: {e}")
    
    root.mainloop()


if __name__ == '__main__':
    main()