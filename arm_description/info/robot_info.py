#!/usr/bin/env python3

import os
import subprocess
import tempfile
import numpy as np
import time
import threading
from roboticstoolbox import Robot
from spatialmath import SE3
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import ttk
import queue

# Try to import pybullet for 3D visualization
try:
    import pybullet as p
    import pybullet_data
    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False
    print("PyBullet not available. Install with: pip install pybullet")

class RobotVisualizer:
    def __init__(self, urdf_path):
        """Initialize robot visualizer with both Robotics Toolbox and PyBullet"""
        self.urdf_path = urdf_path
        self.robot = Robot.URDF(urdf_path)
        self.pybullet_available = PYBULLET_AVAILABLE
        self.end_effector_positions = []
        self.target_positions = []
        self.trajectory_points = []
        self.is_running = False
        
        if self.pybullet_available:
            self.setup_pybullet()
    
    def setup_pybullet(self):
        """Setup PyBullet environment"""
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load ground plane
        self.planeId = p.loadURDF("plane.urdf")
        
        # Load URDF into PyBullet
        self.robot_id = p.loadURDF(self.urdf_path, [0, 0, 0])
        print("PyBullet environment initialized successfully!")
        
        # Set camera position
        p.resetDebugVisualizerCamera(
            cameraDistance=2.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )
        
        # Get end effector link index
        self.end_effector_link = self.robot.n - 1
    
    def get_end_effector_position(self):
        """Get current end effector position from PyBullet"""
        if not self.pybullet_available:
            return None
        
        try:
            # Get the end effector link state
            link_state = p.getLinkState(self.robot_id, self.end_effector_link)
            if link_state:
                # Return the world position of the end effector
                return np.array(link_state[0])  # link_state[0] is the world position
            return None
        except Exception as e:
            print(f"Error getting end effector position: {e}")
            return None
    
    def get_end_effector_pose(self):
        """Get current end effector pose (position and orientation) from PyBullet"""
        if not self.pybullet_available:
            return None, None
        
        try:
            # Get the end effector link state
            link_state = p.getLinkState(self.robot_id, self.end_effector_link)
            if link_state:
                position = np.array(link_state[0])  # World position
                orientation = np.array(link_state[1])  # World orientation (quaternion)
                return position, orientation
            return None, None
        except Exception as e:
            print(f"Error getting end effector pose: {e}")
            return None, None
    
    def get_joint_states(self):
        """Get current joint states from PyBullet"""
        if not self.pybullet_available:
            return None
        
        joint_states = []
        for i in range(self.robot.n):
            joint_info = p.getJointState(self.robot_id, i)
            joint_states.append(joint_info[0])
        return np.array(joint_states)
    
    def set_joint_states(self, joint_states):
        """Set joint states in PyBullet"""
        if not self.pybullet_available:
            return
        
        for i, joint_state in enumerate(joint_states):
            p.resetJointState(self.robot_id, i, joint_state)
    
    def inverse_kinematics(self, target_position, target_orientation=None, q0=None):
        """Calculate inverse kinematics using Robotics Toolbox with optional initial guess"""
        try:
            # Get current joint states as initial guess if not provided
            if q0 is None:
                current_q = self.get_joint_states()
                if current_q is None:
                    current_q = np.zeros(self.robot.n)
            else:
                current_q = q0
            
            # Create target transform
            if target_orientation is None:
                # Use current orientation to maintain it
                current_T = self.robot.fkine(current_q)
                target_orientation = current_T.R
            
            target_T = SE3(target_position) * SE3.Rx(0) * SE3.Ry(0) * SE3.Rz(0)
            
            # Try multiple initial guesses for better IK solution
            initial_guesses = [
                current_q,
                np.zeros(self.robot.n),
                np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
                np.array([-0.1, -0.1, -0.1, -0.1, -0.1, -0.1])
            ]
            
            for guess in initial_guesses:
                try:
                    q_solution = self.robot.ikine_LM(target_T, q0=guess, ilimit=100, slimit=100)
                    if q_solution.success:
                        # Check if solution is within joint limits
                        if self.check_joint_limits(q_solution.q):
                            return q_solution.q
                except:
                    continue
            
            return None
            
        except Exception as e:
            return None
    
    def check_joint_limits(self, q):
        """Check if joint configuration is within limits"""
        for i in range(self.robot.n):
            link_idx = i + 2  # Skip world (0) and base (1)
            if link_idx < len(self.robot.links):
                link = self.robot.links[link_idx]
                if hasattr(link, 'qlim') and link.qlim is not None:
                    if q[i] < link.qlim[0] or q[i] > link.qlim[1]:
                        return False
        return True
    
    def get_workspace_bounds(self):
        """Get approximate workspace bounds"""
        print("Calculating workspace bounds...")
        workspace_points = []
        
        # Use a more systematic approach
        for i in range(100):
            q = np.zeros(self.robot.n)
            for j in range(self.robot.n):
                link_idx = j + 2
                if link_idx < len(self.robot.links):
                    link = self.robot.links[link_idx]
                    if hasattr(link, 'qlim') and link.qlim is not None:
                        # Use smaller ranges for better sampling
                        qlim = link.qlim
                        q[j] = np.random.uniform(qlim[0] * 0.5, qlim[1] * 0.5)
            
            try:
                T = self.robot.fkine(q)
                workspace_points.append(T.t)
            except:
                continue
        
        if len(workspace_points) > 0:
            workspace_points = np.array(workspace_points)
            bounds = {
                'x_min': np.min(workspace_points[:, 0]),
                'x_max': np.max(workspace_points[:, 0]),
                'y_min': np.min(workspace_points[:, 1]),
                'y_max': np.max(workspace_points[:, 1]),
                'z_min': np.min(workspace_points[:, 2]),
                'z_max': np.max(workspace_points[:, 2])
            }
            print(f"Workspace bounds: {bounds}")
            return bounds
        return None
    
    def generate_trajectory(self, start_pos, end_pos, num_points=30):
        """Generate trajectory between two points using joint space interpolation for better accuracy"""
        trajectory = []
        target_positions = []
        
        # Check if positions are reachable
        start_q = self.inverse_kinematics(start_pos)
        end_q = self.inverse_kinematics(end_pos)
        
        if start_q is None or end_q is None:
            print("Cannot reach start or end position")
            return None
        
        print(f"Start joint config: {start_q}")
        print(f"End joint config: {end_q}")
        
        # Generate target positions as a true linear path
        for i in range(num_points):
            t = i / (num_points - 1)
            # Linear interpolation in Cartesian space for target positions
            target_pos = start_pos + t * (end_pos - start_pos)
            target_positions.append(target_pos)
        
        # Use joint space interpolation for actual trajectory
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # Interpolate in joint space (more reliable)
            q_interp = start_q + t * (end_q - start_q)
            
            # Check if interpolated configuration is valid
            if self.check_joint_limits(q_interp):
                trajectory.append(q_interp)
            else:
                # If joint limits exceeded, use start configuration
                trajectory.append(start_q)
        
        print(f"Generated trajectory with {len(trajectory)} points")
        
        # Store target positions for plotting
        self.target_positions = target_positions
        
        return np.array(trajectory) if len(trajectory) > 0 else None
    
    def generate_linear_trajectory(self, start_pos, end_pos, num_points=50):
        """Generate trajectory that better follows a linear path"""
        trajectory = []
        target_positions = []
        
        # Check if positions are reachable
        start_q = self.inverse_kinematics(start_pos)
        end_q = self.inverse_kinematics(end_pos)
        
        if start_q is None or end_q is None:
            print("Cannot reach start or end position")
            return None
        
        print(f"Generating linear trajectory with {num_points} points...")
        
        # Generate target positions as a true linear path
        for i in range(num_points):
            t = i / (num_points - 1)
            target_pos = start_pos + t * (end_pos - start_pos)
            target_positions.append(target_pos)
        
        # Generate joint configurations for each target position
        for i, target_pos in enumerate(target_positions):
            # Try to find IK solution for this target position
            q = self.inverse_kinematics(target_pos)
            
            if q is not None:
                trajectory.append(q)
            else:
                # If IK fails, use joint space interpolation as fallback
                t = i / (num_points - 1)
                q_interp = start_q + t * (end_q - start_q)
                if self.check_joint_limits(q_interp):
                    trajectory.append(q_interp)
                else:
                    # Last resort: use start configuration
                    trajectory.append(start_q)
        
        print(f"Generated linear trajectory with {len(trajectory)} points")
        
        # Store target positions for plotting
        self.target_positions = target_positions
        
        return np.array(trajectory) if len(trajectory) > 0 else None
    
    def generate_hybrid_trajectory(self, start_pos, end_pos, num_points=40):
        """Generate trajectory using hybrid approach for better accuracy"""
        trajectory = []
        target_positions = []
        
        # Check if positions are reachable
        start_q = self.inverse_kinematics(start_pos)
        end_q = self.inverse_kinematics(end_pos)
        
        if start_q is None or end_q is None:
            print("Cannot reach start or end position")
            return None
        
        print(f"Generating hybrid trajectory with {num_points} points...")
        
        # Generate target positions as a true linear path
        for i in range(num_points):
            t = i / (num_points - 1)
            target_pos = start_pos + t * (end_pos - start_pos)
            target_positions.append(target_pos)
        
        # Use a hybrid approach: combine joint space and Cartesian space
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # Joint space interpolation
            q_joint = start_q + t * (end_q - start_q)
            
            # Cartesian space target
            target_pos = target_positions[i]
            
            # Try IK for the target position
            q_cartesian = self.inverse_kinematics(target_pos, q0=q_joint)
            
            if q_cartesian is not None and self.check_joint_limits(q_cartesian):
                trajectory.append(q_cartesian)
            elif self.check_joint_limits(q_joint):
                trajectory.append(q_joint)
            else:
                # Last resort: use start configuration
                trajectory.append(start_q)
        
        print(f"Generated hybrid trajectory with {len(trajectory)} points")
        
        # Store target positions for plotting
        self.target_positions = target_positions
        
        return np.array(trajectory) if len(trajectory) > 0 else None
    
    def visualize_trajectory(self, trajectory, duration=3):
        """Visualize trajectory in PyBullet with accurate position tracking"""
        if not self.pybullet_available or trajectory is None or len(trajectory) == 0:
            print("Cannot visualize trajectory")
            return
        
        print(f"Visualizing trajectory with {len(trajectory)} points...")
        
        # Clear previous trajectory data
        self.end_effector_positions = []
        
        # Calculate time per step for smooth motion
        time_per_step = duration / len(trajectory)
        
        for i, q in enumerate(trajectory):
            # Set joint states
            self.set_joint_states(q)
            
            # Step simulation to update physics
            p.stepSimulation()
            
            # Wait a bit for physics to settle
            time.sleep(0.01)
            
            # Record end effector position after physics update
            pos = self.get_end_effector_position()
            if pos is not None:
                self.end_effector_positions.append(pos)
            
            # Wait for the specified time per step
            time.sleep(time_per_step - 0.01)
        
        print(f"Trajectory visualization completed. Recorded {len(self.end_effector_positions)} positions")
        
        # Validate the recorded positions match the target positions
        if len(self.end_effector_positions) > 0 and len(self.target_positions) > 0:
            actual_positions = np.array(self.end_effector_positions)
            target_positions = np.array(self.target_positions[:len(actual_positions)])
            
            if len(actual_positions) == len(target_positions):
                errors = np.linalg.norm(actual_positions - target_positions, axis=1)
                mean_error = np.mean(errors)
                max_error = np.max(errors)
                print(f"Trajectory accuracy - Mean error: {mean_error:.4f}m, Max error: {max_error:.4f}m")
    
    def plot_trajectory(self):
        """Plot actual vs desired trajectories"""
        if len(self.end_effector_positions) == 0:
            print("No trajectory data to plot")
            return
        
        if len(self.target_positions) == 0:
            print("No target positions to plot")
            return
        
        # Convert to numpy arrays
        actual_positions = np.array(self.end_effector_positions)
        target_positions = np.array(self.target_positions)
        
        # Create figure with 3D subplot
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot actual trajectory
        ax.plot(actual_positions[:, 0], actual_positions[:, 1], actual_positions[:, 2], 
                'b-', linewidth=2, label='Actual Trajectory', marker='o', markersize=4)
        
        # Plot target trajectory
        ax.plot(target_positions[:, 0], target_positions[:, 1], target_positions[:, 2], 
                'r--', linewidth=2, label='Target Trajectory', marker='s', markersize=4)
        
        # Plot start and end points
        ax.scatter(actual_positions[0, 0], actual_positions[0, 1], actual_positions[0, 2], 
                   c='green', s=100, label='Start Point', marker='^')
        ax.scatter(actual_positions[-1, 0], actual_positions[-1, 1], actual_positions[-1, 2], 
                   c='red', s=100, label='End Point', marker='v')
        
        # Set labels and title
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Actual vs Target Trajectory Comparison')
        ax.legend()
        
        # Set equal aspect ratio
        ax.set_box_aspect([1, 1, 1])
        
        # Add grid
        ax.grid(True)
        
        # Calculate and display error metrics
        if len(actual_positions) == len(target_positions):
            errors = np.linalg.norm(actual_positions - target_positions, axis=1)
            mean_error = np.mean(errors)
            max_error = np.max(errors)
            
            # Add text box with error metrics
            error_text = f'Mean Error: {mean_error:.4f} m\nMax Error: {max_error:.4f} m'
            ax.text2D(0.02, 0.98, error_text, transform=ax.transAxes, 
                     bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                     verticalalignment='top')
        
        plt.tight_layout()
        plt.show()
        
        print(f"Plotted trajectory with {len(actual_positions)} actual points and {len(target_positions)} target points")

    def validate_trajectory(self, trajectory, target_positions):
        """Validate trajectory accuracy by checking end effector positions"""
        if len(trajectory) == 0 or len(target_positions) == 0:
            return False
        
        print("Validating trajectory accuracy...")
        actual_positions = []
        
        # Simulate trajectory to get actual positions
        for q in trajectory:
            self.set_joint_states(q)
            p.stepSimulation()
            pos = self.get_end_effector_position()
            if pos is not None:
                actual_positions.append(pos)
        
        if len(actual_positions) == 0:
            print("Could not validate trajectory - no positions recorded")
            return False
        
        # Calculate errors
        actual_positions = np.array(actual_positions)
        target_positions = np.array(target_positions[:len(actual_positions)])
        
        errors = np.linalg.norm(actual_positions - target_positions, axis=1)
        mean_error = np.mean(errors)
        max_error = np.max(errors)
        
        print(f"Trajectory validation:")
        print(f"  Mean error: {mean_error:.4f} m")
        print(f"  Max error: {max_error:.4f} m")
        print(f"  Points validated: {len(actual_positions)}")
        
        return mean_error < 0.01  # Accept if mean error < 1cm

    def optimize_trajectory(self, trajectory, target_positions, max_iterations=10):
        """Optimize trajectory to reduce deviation from target positions"""
        if len(trajectory) == 0 or len(target_positions) == 0:
            return trajectory
        
        print("Optimizing trajectory for better accuracy...")
        optimized_trajectory = trajectory.copy()
        
        for iteration in range(max_iterations):
            total_error = 0
            improvements = 0
            
            for i in range(len(optimized_trajectory)):
                # Get current end effector position for this joint configuration
                self.set_joint_states(optimized_trajectory[i])
                p.stepSimulation()
                current_pos = self.get_end_effector_position()
                
                if current_pos is not None:
                    target_pos = target_positions[i]
                    current_error = np.linalg.norm(current_pos - target_pos)
                    total_error += current_error
                    
                    # If error is significant, try to improve this configuration
                    if current_error > 0.005:  # 5mm threshold
                        # Try IK with target position
                        improved_q = self.inverse_kinematics(target_pos, q0=optimized_trajectory[i])
                        if improved_q is not None:
                            # Check if the improved configuration actually reduces error
                            self.set_joint_states(improved_q)
                            p.stepSimulation()
                            improved_pos = self.get_end_effector_position()
                            
                            if improved_pos is not None:
                                improved_error = np.linalg.norm(improved_pos - target_pos)
                                if improved_error < current_error:
                                    optimized_trajectory[i] = improved_q
                                    improvements += 1
            
            mean_error = total_error / len(optimized_trajectory)
            print(f"Iteration {iteration + 1}: Mean error = {mean_error:.4f}m, Improvements = {improvements}")
            
            # If no improvements or error is small enough, stop
            if improvements == 0 or mean_error < 0.003:  # 3mm threshold
                break
        
        return optimized_trajectory

class TrajectoryPlannerGUI:
    def __init__(self, robot_visualizer):
        self.robot_visualizer = robot_visualizer
        self.root = tk.Tk()
        self.root.title("Robot Trajectory Planner")
        self.root.geometry("500x400")
        
        # Get workspace bounds
        self.workspace_bounds = self.robot_visualizer.get_workspace_bounds()
        
        self.setup_gui()
    
    def setup_gui(self):
        """Setup the GUI elements"""
        # Title
        title_label = tk.Label(self.root, text="Robot Trajectory Planner", font=("Arial", 14, "bold"))
        title_label.pack(pady=5)
        
        # Workspace bounds display (compact)
        if self.workspace_bounds:
            bounds_text = f"Workspace: X[{self.workspace_bounds['x_min']:.2f},{self.workspace_bounds['x_max']:.2f}] "
            bounds_text += f"Y[{self.workspace_bounds['y_min']:.2f},{self.workspace_bounds['y_max']:.2f}] "
            bounds_text += f"Z[{self.workspace_bounds['z_min']:.2f},{self.workspace_bounds['z_max']:.2f}]"
            bounds_label = tk.Label(self.root, text=bounds_text, font=("Arial", 8))
            bounds_label.pack(pady=2)
        
        # Current position display (compact)
        current_frame = tk.LabelFrame(self.root, text="Current Position", padx=5, pady=2)
        current_frame.pack(fill="x", padx=5, pady=2)
        
        self.current_pos_label = tk.Label(current_frame, text="Position: [0.000, 0.000, 0.000]")
        self.current_pos_label.pack()
        
        # Position controls frame
        pos_frame = tk.Frame(self.root)
        pos_frame.pack(fill="x", padx=5, pady=2)
        
        # Start position frame (left side)
        start_frame = tk.LabelFrame(pos_frame, text="Start Position", padx=5, pady=2)
        start_frame.pack(side="left", fill="both", expand=True, padx=2)
        
        # Start position sliders with workspace bounds
        if self.workspace_bounds:
            x_range = (self.workspace_bounds['x_min'], self.workspace_bounds['x_max'])
            y_range = (self.workspace_bounds['y_min'], self.workspace_bounds['y_max'])
            z_range = (self.workspace_bounds['z_min'], self.workspace_bounds['z_max'])
        else:
            x_range = (-0.3, 0.3)
            y_range = (-0.3, 0.3)
            z_range = (0.2, 0.6)
        
        # Use conservative default values
        self.start_x = tk.DoubleVar(value=0.0)
        self.start_y = tk.DoubleVar(value=0.0)
        self.start_z = tk.DoubleVar(value=0.4)
        
        tk.Label(start_frame, text="X:").grid(row=0, column=0, sticky="w")
        start_x_slider = tk.Scale(start_frame, from_=x_range[0], to=x_range[1], resolution=0.01, 
                                 variable=self.start_x, orient="horizontal", length=150)
        start_x_slider.grid(row=0, column=1, padx=2)
        self.start_x_label = tk.Label(start_frame, text="0.000", width=6)
        self.start_x_label.grid(row=0, column=2, padx=2)
        
        tk.Label(start_frame, text="Y:").grid(row=1, column=0, sticky="w")
        start_y_slider = tk.Scale(start_frame, from_=y_range[0], to=y_range[1], resolution=0.01, 
                                 variable=self.start_y, orient="horizontal", length=150)
        start_y_slider.grid(row=1, column=1, padx=2)
        self.start_y_label = tk.Label(start_frame, text="0.000", width=6)
        self.start_y_label.grid(row=1, column=2, padx=2)
        
        tk.Label(start_frame, text="Z:").grid(row=2, column=0, sticky="w")
        start_z_slider = tk.Scale(start_frame, from_=z_range[0], to=z_range[1], resolution=0.01, 
                                 variable=self.start_z, orient="horizontal", length=150)
        start_z_slider.grid(row=2, column=1, padx=2)
        self.start_z_label = tk.Label(start_frame, text="0.400", width=6)
        self.start_z_label.grid(row=2, column=2, padx=2)
        
        # End position frame (right side)
        end_frame = tk.LabelFrame(pos_frame, text="End Position", padx=5, pady=2)
        end_frame.pack(side="right", fill="both", expand=True, padx=2)
        
        # End position sliders with conservative values
        self.end_x = tk.DoubleVar(value=0.1)
        self.end_y = tk.DoubleVar(value=0.1)
        self.end_z = tk.DoubleVar(value=0.4)
        
        tk.Label(end_frame, text="X:").grid(row=0, column=0, sticky="w")
        end_x_slider = tk.Scale(end_frame, from_=x_range[0], to=x_range[1], resolution=0.01, 
                               variable=self.end_x, orient="horizontal", length=150)
        end_x_slider.grid(row=0, column=1, padx=2)
        self.end_x_label = tk.Label(end_frame, text="0.100", width=6)
        self.end_x_label.grid(row=0, column=2, padx=2)
        
        tk.Label(end_frame, text="Y:").grid(row=1, column=0, sticky="w")
        end_y_slider = tk.Scale(end_frame, from_=y_range[0], to=y_range[1], resolution=0.01, 
                               variable=self.end_y, orient="horizontal", length=150)
        end_y_slider.grid(row=1, column=1, padx=2)
        self.end_y_label = tk.Label(end_frame, text="0.100", width=6)
        self.end_y_label.grid(row=1, column=2, padx=2)
        
        tk.Label(end_frame, text="Z:").grid(row=2, column=0, sticky="w")
        end_z_slider = tk.Scale(end_frame, from_=z_range[0], to=z_range[1], resolution=0.01, 
                               variable=self.end_z, orient="horizontal", length=150)
        end_z_slider.grid(row=2, column=1, padx=2)
        self.end_z_label = tk.Label(end_frame, text="0.400", width=6)
        self.end_z_label.grid(row=2, column=2, padx=2)
        
        # Bind slider updates
        self.start_x.trace("w", self.update_labels)
        self.start_y.trace("w", self.update_labels)
        self.start_z.trace("w", self.update_labels)
        self.end_x.trace("w", self.update_labels)
        self.end_y.trace("w", self.update_labels)
        self.end_z.trace("w", self.update_labels)
        
        # Control buttons (compact layout)
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=5)
        
        # First row of buttons
        row1_frame = tk.Frame(button_frame)
        row1_frame.pack()
        tk.Button(row1_frame, text="Move to Start", command=self.move_to_start, width=12).pack(side="left", padx=2)
        tk.Button(row1_frame, text="Move to End", command=self.move_to_end, width=12).pack(side="left", padx=2)
        tk.Button(row1_frame, text="Reset Robot", command=self.reset_robot, width=12).pack(side="left", padx=2)
        
        # Second row of buttons
        row2_frame = tk.Frame(button_frame)
        row2_frame.pack()
        tk.Button(row2_frame, text="Randomize Start", command=self.randomize_start_position, width=12).pack(side="left", padx=2)
        tk.Button(row2_frame, text="Randomize End", command=self.randomize_end_position, width=12).pack(side="left", padx=2)
        tk.Button(row2_frame, text="Start Motion", command=self.start_motion, width=12).pack(side="left", padx=2)
        
        # Third row of buttons
        row3_frame = tk.Frame(button_frame)
        row3_frame.pack()
        tk.Button(row3_frame, text="Plot Trajectory", command=self.plot_trajectory, width=12).pack(side="left", padx=2)
        
        # Status label
        self.status_label = tk.Label(self.root, text="Ready", fg="green", font=("Arial", 9))
        self.status_label.pack(pady=2)
        
        # Start position update thread
        self.is_running = True
        self.update_thread = threading.Thread(target=self.update_position)
        self.update_thread.daemon = True
        self.update_thread.start()
    
    def reset_robot(self):
        """Reset robot to home position"""
        home_q = np.zeros(self.robot_visualizer.robot.n)
        self.robot_visualizer.set_joint_states(home_q)
        self.status_label.config(text="Robot reset to home position", fg="green")
    
    def update_labels(self, *args):
        """Update position labels"""
        self.start_x_label.config(text=f"{self.start_x.get():.3f}")
        self.start_y_label.config(text=f"{self.start_y.get():.3f}")
        self.start_z_label.config(text=f"{self.start_z.get():.3f}")
        self.end_x_label.config(text=f"{self.end_x.get():.3f}")
        self.end_y_label.config(text=f"{self.end_y.get():.3f}")
        self.end_z_label.config(text=f"{self.end_z.get():.3f}")
    
    def update_position(self):
        """Update current position display"""
        while self.is_running:
            if self.robot_visualizer.pybullet_available:
                pos = self.robot_visualizer.get_end_effector_position()
                if pos is not None:
                    self.current_pos_label.config(text=f"Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            time.sleep(0.1)
    
    def move_to_start(self):
        """Move robot to start position"""
        start_pos = np.array([self.start_x.get(), self.start_y.get(), self.start_z.get()])
        self.status_label.config(text="Calculating IK for start position...", fg="orange")
        self.root.update()
        
        q = self.robot_visualizer.inverse_kinematics(start_pos)
        if q is not None:
            self.robot_visualizer.set_joint_states(q)
            self.status_label.config(text="Moved to start position", fg="green")
        else:
            self.status_label.config(text="Failed to reach start position - outside workspace", fg="red")
    
    def move_to_end(self):
        """Move robot to end position"""
        end_pos = np.array([self.end_x.get(), self.end_y.get(), self.end_z.get()])
        self.status_label.config(text="Calculating IK for end position...", fg="orange")
        self.root.update()
        
        q = self.robot_visualizer.inverse_kinematics(end_pos)
        if q is not None:
            self.robot_visualizer.set_joint_states(q)
            self.status_label.config(text="Moved to end position", fg="green")
        else:
            self.status_label.config(text="Failed to reach end position - outside workspace", fg="red")
    
    def generate_trajectory(self):
        """Generate and execute trajectory"""
        start_pos = np.array([self.start_x.get(), self.start_y.get(), self.start_z.get()])
        end_pos = np.array([self.end_x.get(), self.end_y.get(), self.end_z.get()])
        
        self.status_label.config(text="Generating trajectory...", fg="orange")
        self.root.update()
        
        # Generate trajectory
        trajectory = self.robot_visualizer.generate_trajectory(start_pos, end_pos)
        
        if trajectory is not None and len(trajectory) > 0:
            # Clear previous trajectory data
            self.robot_visualizer.end_effector_positions = []
            self.robot_visualizer.target_positions = []
            
            # Generate target positions for plotting
            num_points = len(trajectory)
            for i in range(num_points):
                t = i / (num_points - 1)
                target_pos = start_pos + t * (end_pos - start_pos)
                self.robot_visualizer.target_positions.append(target_pos)
            
            # Visualize trajectory
            self.robot_visualizer.visualize_trajectory(trajectory)
            self.status_label.config(text=f"Trajectory completed with {len(trajectory)} points", fg="green")
        else:
            self.status_label.config(text="Failed to generate trajectory - check workspace bounds", fg="red")
    
    def plot_trajectory(self):
        """Plot trajectory"""
        self.robot_visualizer.plot_trajectory()
    
    def get_random_workspace_point(self):
        """Get a random point within the robot's workspace"""
        if self.workspace_bounds:
            x = np.random.uniform(self.workspace_bounds['x_min'], self.workspace_bounds['x_max'])
            y = np.random.uniform(self.workspace_bounds['y_min'], self.workspace_bounds['y_max'])
            z = np.random.uniform(self.workspace_bounds['z_min'], self.workspace_bounds['z_max'])
            return np.array([x, y, z])
        else:
            # Fallback to conservative bounds
            x = np.random.uniform(-0.2, 0.2)
            y = np.random.uniform(-0.2, 0.2)
            z = np.random.uniform(0.3, 0.5)
            return np.array([x, y, z])
    
    def find_reachable_point(self, max_attempts=50):
        """Find a reachable point in the workspace"""
        for _ in range(max_attempts):
            point = self.get_random_workspace_point()
            q = self.robot_visualizer.inverse_kinematics(point)
            if q is not None:
                return point, q
        return None, None
    
    def randomize_start_position(self):
        """Randomize start position to a reachable point"""
        point, q = self.find_reachable_point()
        if point is not None:
            self.start_x.set(point[0])
            self.start_y.set(point[1])
            self.start_z.set(point[2])
            self.robot_visualizer.set_joint_states(q)
            self.status_label.config(text=f"Randomized start position: [{point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}]", fg="green")
        else:
            self.status_label.config(text="Could not find reachable start position", fg="red")
    
    def randomize_end_position(self):
        """Randomize end position to a reachable point"""
        point, q = self.find_reachable_point()
        if point is not None:
            self.end_x.set(point[0])
            self.end_y.set(point[1])
            self.end_z.set(point[2])
            self.status_label.config(text=f"Randomized end position: [{point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}]", fg="green")
        else:
            self.status_label.config(text="Could not find reachable end position", fg="red")
    
    def start_motion(self):
        """Start the motion sequence"""
        start_pos = np.array([self.start_x.get(), self.start_y.get(), self.start_z.get()])
        end_pos = np.array([self.end_x.get(), self.end_y.get(), self.end_z.get()])
        
        self.status_label.config(text="Starting motion sequence...", fg="orange")
        self.root.update()
        
        # First move to start position
        start_q = self.robot_visualizer.inverse_kinematics(start_pos)
        if start_q is not None:
            self.robot_visualizer.set_joint_states(start_q)
            self.status_label.config(text="Moved to start position", fg="green")
            self.root.update()
            time.sleep(1)  # Pause to show start position
            
            # Generate hybrid trajectory for better accuracy
            trajectory = self.robot_visualizer.generate_hybrid_trajectory(start_pos, end_pos)
            if trajectory is not None and len(trajectory) > 0:
                # Optimize trajectory for better accuracy
                optimized_trajectory = self.robot_visualizer.optimize_trajectory(
                    trajectory, self.robot_visualizer.target_positions
                )
                
                # Validate optimized trajectory accuracy
                is_accurate = self.robot_visualizer.validate_trajectory(optimized_trajectory, self.robot_visualizer.target_positions)
                
                # Clear previous trajectory data
                self.robot_visualizer.end_effector_positions = []
                
                # Visualize optimized trajectory
                self.robot_visualizer.visualize_trajectory(optimized_trajectory, duration=3)
                
                if is_accurate:
                    self.status_label.config(text="Motion completed! Trajectory is accurate. Click 'Plot Trajectory' to see comparison", fg="green")
                else:
                    self.status_label.config(text="Motion completed! Trajectory has some deviation. Click 'Plot Trajectory' to see comparison", fg="orange")
            else:
                self.status_label.config(text="Failed to generate trajectory", fg="red")
        else:
            self.status_label.config(text="Cannot reach start position", fg="red")
    
    def run(self):
        """Run the GUI"""
        self.root.mainloop()
        self.is_running = False

def generate_urdf_from_xacro(xacro_file_path):
    try:
        with tempfile.NamedTemporaryFile(prefix="kikobot_", suffix=".urdf", delete=False) as tmp:
            urdf_text = subprocess.check_output(["xacro", xacro_file_path, "is_ignition:=False"]).decode("utf-8")
            tmp.write(urdf_text.encode("utf-8"))
            return tmp.name
    except Exception:
        return None


def main():
    # Initialize robot visualizer
    xacro_path = os.path.join(os.path.dirname(__file__), "../urdf/arm.urdf.xacro")
    urdf_path = generate_urdf_from_xacro(os.path.realpath(xacro_path))
    if urdf_path is None:
        print("Failed to generate URDF from xacro. Ensure xacro is installed.")
        return
    visualizer = RobotVisualizer(urdf_path)
    
    if not PYBULLET_AVAILABLE:
        print("PyBullet not available. Install with: pip install pybullet")
        return
    
    print("\n" + "="*50)
    print("Robot Trajectory Planner")
    print("="*50)
    print("PyBullet visualization is now active!")
    print("Use the Tkinter window to plan trajectories.")
    print("="*50)
    
    # Create and run GUI
    gui = TrajectoryPlannerGUI(visualizer)
    gui.run()

if __name__ == "__main__":
    main()