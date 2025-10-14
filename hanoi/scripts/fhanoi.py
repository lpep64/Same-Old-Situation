#!/usr/bin/env python3
"""
Khanoi - Enhanced Panda Hanoi Controller
Advanced control implementation with comprehensive control functions
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import sys
from enum import Enum
from typing import List, Tuple, Optional

# Add parent directory to path so we can import utils
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class ControlMode(Enum):
    """Available control modes for the robot"""
    POSITION = "position"
    VELOCITY = "velocity" 
    TORQUE = "torque"
    MIXED = "mixed"


class RobotState(Enum):
    """Robot operation states for state machine"""
    IDLE = "idle"
    MOVING_TO_TARGET = "moving_to_target"
    APPROACHING_DISK = "approaching_disk"
    GRASPING = "grasping"
    LIFTING = "lifting"
    MOVING_WITH_DISK = "moving_with_disk"
    PLACING = "placing"
    RELEASING = "releasing"
    RETURNING_HOME = "returning_home"


class PandaAdvancedController:
    """Advanced controller for the Panda robot with comprehensive control functions."""
    
    def __init__(self, model, data):
        """Initialize the advanced Panda robot controller."""
        self.model = model
        self.data = data
        
        # Get joint and actuator indices
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        try:
            self.joint_indices = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) 
                                 for name in self.joint_names}
            
            # Gripper actuator index
            self.gripper_actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "actuator8")
            
            # Actuator indices for arm joints
            self.actuator_indices = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"actuator{i+1}") 
                                    for i, name in enumerate(self.joint_names)}
        except Exception as e:
            raise RuntimeError(f"Failed to find robot joints/actuators: {e}")
        
        # Control parameters
        self.time = 0.0
        self.control_mode = ControlMode.POSITION
        self.robot_state = RobotState.IDLE
        
        # Home position and joint limits
        self.home_qpos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
        self.joint_limits = {
            'joint1': (-2.8973, 2.8973),
            'joint2': (-1.7628, 1.7628), 
            'joint3': (-2.8973, 2.8973),
            'joint4': (-3.0718, -0.0698),
            'joint5': (-2.8973, 2.8973),
            'joint6': (-0.0175, 3.7525),
            'joint7': (-2.8973, 2.8973)
        }
        
        # Control gains for PD control (reduced for stability)
        self.kp = np.array([25.0, 25.0, 25.0, 25.0, 20.0, 20.0, 15.0])  # Position gains
        self.kd = np.array([5.0, 5.0, 5.0, 5.0, 4.0, 4.0, 3.0])        # Velocity gains
        
        # Control stability parameters
        self.position_tolerance = 0.01  # radians - stop control when close enough
        self.velocity_deadband = 0.05   # rad/s - ignore small velocities
        self.control_limit = 20.0       # Maximum control effort
        
        # Trajectory tracking
        self.target_positions = self.home_qpos.copy()
        self.trajectory_start_time = 0.0
        self.trajectory_duration = 2.0
        self.trajectory_start_pos = self.home_qpos.copy()
        self.trajectory_target_pos = self.home_qpos.copy()
        
        # Hanoi-specific parameters
        self.tower_positions = {
            'A': np.array([0.3, 0.3, 0.0]),   # Tower A position
            'B': np.array([0.0, 0.3, 0.0]),   # Tower B position  
            'C': np.array([-0.3, 0.3, 0.0])   # Tower C position
        }
        self.disk_height = 0.05
        self.approach_height = 0.15
        self.gripper_open = 255
        self.gripper_closed = 0
        
        # Move sequence for automated Hanoi solving
        self.move_sequence = []
        self.current_move_index = 0
        self.executing_sequence = False
        
        # Reset to home position
        self.reset_to_home()
        print("Starting advanced panda controller control mode now")

    # ================================
    # CORE CONTROL FUNCTIONS
    # ================================
    
    def update_control(self, dt):
        """Main control update function called each simulation step."""
        self.time += dt
        
        # Hold position when idle to prevent drift
        if self.robot_state == RobotState.IDLE:
            self.hold_position()
        
        if self.control_mode == ControlMode.POSITION:
            self._position_control()
        elif self.control_mode == ControlMode.VELOCITY:
            self._velocity_control()
        elif self.control_mode == ControlMode.TORQUE:
            self._torque_control()
        elif self.control_mode == ControlMode.MIXED:
            self._mixed_control()
        
        # Execute automated sequences if active
        if self.executing_sequence:
            self._execute_move_sequence()
    
    def control_callback(self, model, data):
        """Control callback function for use with mj.set_mjcb_control() - with stability improvements."""
        # This would be called automatically by MuJoCo during mj_step
        # All intermediate physics results are available here
        current_time = data.time
        
        # Stable PD control with deadband
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            actuator_id = self.actuator_indices[joint_name]
            
            # PD control with deadband
            pos_error = self.target_positions[i] - data.qpos[joint_id]
            current_velocity = data.qvel[joint_id]
            
            # Skip control if no position change needed
            if abs(pos_error) < 1e-6:
                continue
            
            # Apply deadband to prevent oscillations
            if abs(pos_error) < self.position_tolerance and abs(current_velocity) < self.velocity_deadband:
                control_signal = -0.1 * current_velocity  # Light damping only
            else:
                vel_error = 0.0 - current_velocity
                control_signal = self.kp[i] * pos_error + self.kd[i] * vel_error
            
            # Limit control effort
            control_signal = np.clip(control_signal, -self.control_limit, self.control_limit)
            data.ctrl[actuator_id] = control_signal

    # ================================
    # CONTROL MODE IMPLEMENTATIONS
    # ================================
    
    def _position_control(self):
        """Position control implementation using PD control with deadband."""
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            actuator_id = self.actuator_indices[joint_name]
            
            # PD control with deadband
            pos_error = self.target_positions[i] - self.data.qpos[joint_id]
            current_velocity = self.data.qvel[joint_id]
            
            # Skip control if no position change needed
            if abs(pos_error) < 1e-6:  # No position change needed
                continue
            
            # Apply deadband to prevent small oscillations
            if abs(pos_error) < self.position_tolerance and abs(current_velocity) < self.velocity_deadband:
                # Position is close enough and velocity is low - minimal control
                control_signal = -0.1 * current_velocity  # Just damping
            else:
                # Normal PD control
                vel_error = 0.0 - current_velocity
                control_signal = self.kp[i] * pos_error + self.kd[i] * vel_error
            
            # Limit control effort for stability
            control_signal = np.clip(control_signal, -self.control_limit, self.control_limit)
            
            self.data.ctrl[actuator_id] = control_signal
    
    def _velocity_control(self):
        """Velocity control implementation with deadband."""
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            actuator_id = self.actuator_indices[joint_name]
            
            # Target velocity is 0 for holding position
            target_velocity = 0.0
            current_velocity = self.data.qvel[joint_id]
            
            # Skip control if target velocity is 0.0
            if target_velocity == 0.0:
                continue
            
            # Apply deadband
            if abs(current_velocity) < self.velocity_deadband:
                control_signal = 0.0  # No control needed
            else:
                vel_error = target_velocity - current_velocity
                control_signal = self.kd[i] * vel_error
                control_signal = np.clip(control_signal, -self.control_limit, self.control_limit)
            
            self.data.ctrl[actuator_id] = control_signal
    
    def _torque_control(self):
        """Direct torque control implementation - DISABLED to prevent shakiness."""
        # REMOVED: Sine wave torques that cause unwanted movement
        # Apply minimal damping instead to maintain stability
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            actuator_id = self.actuator_indices[joint_name]
            
            # Apply only damping to current velocity
            current_velocity = self.data.qvel[joint_id]
            damping_torque = -2.0 * current_velocity  # Light damping
            
            self.data.ctrl[actuator_id] = damping_torque
    
    def _mixed_control(self):
        """Mixed control mode - different control for different joints with stability."""
        # Position control for first 4 joints, velocity for others
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            actuator_id = self.actuator_indices[joint_name]
            
            if i < 4:  # Position control with deadband
                pos_error = self.target_positions[i] - self.data.qpos[joint_id]
                current_velocity = self.data.qvel[joint_id]
                
                # Skip control if no position change needed
                if abs(pos_error) < 1e-6:
                    continue
                
                if abs(pos_error) < self.position_tolerance and abs(current_velocity) < self.velocity_deadband:
                    control_signal = -0.1 * current_velocity  # Light damping
                else:
                    vel_error = 0.0 - current_velocity
                    control_signal = self.kp[i] * pos_error + self.kd[i] * vel_error
                    
            else:  # Velocity control with deadband
                target_velocity = 0.0
                current_velocity = self.data.qvel[joint_id]
                
                # Skip control if target velocity is 0.0
                if target_velocity == 0.0:
                    continue
                
                if abs(current_velocity) < self.velocity_deadband:
                    control_signal = 0.0
                else:
                    vel_error = target_velocity - current_velocity
                    control_signal = self.kd[i] * vel_error
            
            # Apply control limits
            control_signal = np.clip(control_signal, -self.control_limit, self.control_limit)
            self.data.ctrl[actuator_id] = control_signal

    # ================================
    # BASIC MOVEMENT FUNCTIONS
    # ================================
    
    def move_joints(self, j1=None, j2=None, j3=None, j4=None, j5=None, j6=None, j7=None, gripper=None):
        """Set individual joint positions directly."""
        joint_values = [j1, j2, j3, j4, j5, j6, j7]
        
        for i, (joint_name, value) in enumerate(zip(self.joint_names, joint_values)):
            if value is not None:
                # Check joint limits
                min_limit, max_limit = self.joint_limits[joint_name]
                value = np.clip(value, min_limit, max_limit)
                self.target_positions[i] = value
        
        if gripper is not None:
            gripper = np.clip(gripper, 0, 255)
            self.data.ctrl[self.gripper_actuator_id] = gripper
    
    def move_to_position(self, target_joints: np.ndarray, duration: float = 2.0):
        """Smooth movement to target joint configuration over specified duration."""
        if len(target_joints) != 7:
            raise ValueError("target_joints must be array of 7 joint angles")
        
        # Check all joint limits
        for i, (joint_name, target) in enumerate(zip(self.joint_names, target_joints)):
            min_limit, max_limit = self.joint_limits[joint_name]
            target_joints[i] = np.clip(target, min_limit, max_limit)
        
        # Set up trajectory
        self.trajectory_start_time = self.time
        self.trajectory_duration = duration
        self.trajectory_start_pos = self.get_current_joint_positions()
        self.trajectory_target_pos = target_joints.copy()
        self.robot_state = RobotState.MOVING_TO_TARGET
        
        print(f"Starting trajectory control mode now")
    
    def get_current_joint_positions(self) -> np.ndarray:
        """Get current joint positions."""
        positions = np.zeros(7)
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            positions[i] = self.data.qpos[joint_id]
        return positions
    
    def get_current_joint_velocities(self) -> np.ndarray:
        """Get current joint velocities."""
        velocities = np.zeros(7)
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            velocities[i] = self.data.qvel[joint_id]
        return velocities

    # ================================
    # TRAJECTORY PLANNING
    # ================================
    
    def _update_trajectory(self):
        """Update trajectory tracking during movement."""
        if self.robot_state != RobotState.MOVING_TO_TARGET:
            return
        
        elapsed = self.time - self.trajectory_start_time
        progress = min(elapsed / self.trajectory_duration, 1.0)
        
        # Smooth interpolation using cubic polynomial
        t = progress
        t3 = t * t * t
        t2 = t * t
        
        # Cubic trajectory: smooth start and stop
        s = 3 * t2 - 2 * t3
        
        # Interpolate positions
        self.target_positions = self.trajectory_start_pos + s * (self.trajectory_target_pos - self.trajectory_start_pos)
        
        # Check if trajectory is complete
        if progress >= 1.0:
            self.robot_state = RobotState.IDLE
            print("Starting idle control mode now")

    # ================================
    # HANOI-SPECIFIC FUNCTIONS
    # ================================
    
    def pick_disk(self, tower: str, disk_level: int = 0):
        """Move to disk location and pick it up."""
        print(f"Starting pick disk control mode now")
        
        if tower not in self.tower_positions:
            raise ValueError(f"Invalid tower: {tower}")
        
        # Calculate disk position
        tower_pos = self.tower_positions[tower]
        disk_position = tower_pos + np.array([0, 0, disk_level * self.disk_height])
        
        # Convert to joint angles (simplified inverse kinematics)
        target_joints = self._simple_ik(disk_position + np.array([0, 0, self.approach_height]))
        
        # Execute pick sequence
        self.robot_state = RobotState.APPROACHING_DISK
        self.move_to_position(target_joints, duration=2.0)
        
        # TODO: Add state machine to complete pick sequence
        # 1. Move above disk
        # 2. Lower to disk
        # 3. Close gripper
        # 4. Lift disk
    
    def place_disk(self, tower: str, disk_level: int = 0):
        """Move to target location and place disk."""
        print(f"Starting place disk control mode now")
        
        if tower not in self.tower_positions:
            raise ValueError(f"Invalid tower: {tower}")
        
        # Calculate target position
        tower_pos = self.tower_positions[tower]
        target_position = tower_pos + np.array([0, 0, disk_level * self.disk_height])
        
        # Convert to joint angles
        target_joints = self._simple_ik(target_position + np.array([0, 0, self.approach_height]))
        
        # Execute place sequence
        self.robot_state = RobotState.MOVING_WITH_DISK
        self.move_to_position(target_joints, duration=2.0)
        
        # TODO: Add state machine to complete place sequence
        # 1. Move above target
        # 2. Lower to position
        # 3. Open gripper
        # 4. Lift away
    
    def execute_hanoi_move(self, from_tower: str, to_tower: str, disk_level_from: int = 0, disk_level_to: int = 0):
        """Execute a complete Hanoi move from one tower to another."""
        print(f"Starting hanoi move control mode now")
        
        # Add to move sequence
        move = {
            'from_tower': from_tower,
            'to_tower': to_tower,
            'disk_level_from': disk_level_from,
            'disk_level_to': disk_level_to,
            'step': 'pick'  # Current step in the move
        }
        
        self.move_sequence.append(move)
        if not self.executing_sequence:
            self.executing_sequence = True
            self.current_move_index = 0
    
    def solve_hanoi(self, n_disks: int = 3, from_tower: str = 'A', to_tower: str = 'C', aux_tower: str = 'B'):
        """Generate sequence to solve Hanoi towers puzzle."""
        print(f"Starting hanoi solver control mode now")
        
        def hanoi_recursive(n, source, destination, auxiliary):
            if n == 1:
                self.execute_hanoi_move(source, destination)
            else:
                hanoi_recursive(n-1, source, auxiliary, destination)
                self.execute_hanoi_move(source, destination)
                hanoi_recursive(n-1, auxiliary, destination, source)
        
        # Clear existing sequence
        self.move_sequence = []
        hanoi_recursive(n_disks, from_tower, to_tower, aux_tower)
    
    def _execute_move_sequence(self):
        """Execute the current move sequence step by step."""
        if not self.move_sequence or self.current_move_index >= len(self.move_sequence):
            self.executing_sequence = False
            return
        
        # Simple implementation - just print moves for now
        current_move = self.move_sequence[self.current_move_index]
        
        if self.robot_state == RobotState.IDLE:
            print(f"Executing move {self.current_move_index + 1}/{len(self.move_sequence)}: "
                  f"{current_move['from_tower']} -> {current_move['to_tower']}")
            
            # For demonstration, just wait a bit then move to next
            time.sleep(0.1)  # Brief pause
            self.current_move_index += 1

    # ================================
    # UTILITY FUNCTIONS
    # ================================
    
    def reset_to_home(self):
        """Reset robot to home position."""
        print("Starting home position control mode now")
        
        # Set joint positions
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            self.data.qpos[joint_id] = self.home_qpos[i]
        
        # Reset targets
        self.target_positions = self.home_qpos.copy()
        self.robot_state = RobotState.IDLE
        
        # Open gripper
        self.data.ctrl[self.gripper_actuator_id] = self.gripper_open
        
        mujoco.mj_forward(self.model, self.data)
    
    def set_control_mode(self, mode: ControlMode):
        """Set the control mode."""
        self.control_mode = mode
        print(f"Starting {mode.value} control mode now")
    
    def apply_external_force(self, body_name: str, force: np.ndarray, torque: np.ndarray = None):
        """Apply external forces/torques to a body."""
        try:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            
            # Apply force
            if force is not None and len(force) == 3:
                self.data.xfrc_applied[body_id][:3] = force
            
            # Apply torque
            if torque is not None and len(torque) == 3:
                self.data.xfrc_applied[body_id][3:] = torque
                
            print(f"Applied external force to {body_name}: force={force}, torque={torque}")
        except Exception as e:
            print(f"Failed to apply external force to {body_name}: {e}")
    
    def get_end_effector_position(self) -> np.ndarray:
        """Get current end-effector position."""
        # This is a simplified version - you'd typically use forward kinematics
        # or get from a sensor/site in the model
        return np.array([0.5, 0.0, 0.5])  # Placeholder
    
    def _simple_ik(self, target_position: np.ndarray) -> np.ndarray:
        """Simplified inverse kinematics - returns approximate joint angles."""
        # This is a very basic placeholder implementation
        # In practice, you'd use proper IK solver or analytical solution
        
        # For now, return a reasonable configuration
        x, y, z = target_position
        
        # Simple geometric approach for demonstration
        joint_angles = np.array([
            np.arctan2(y, x),           # Base rotation
            -0.5,                       # Shoulder
            0.0,                        # Elbow
            -1.0,                       # Wrist1  
            0.0,                        # Wrist2
            1.0,                        # Wrist3
            0.0                         # Wrist rotate
        ])
        
        # Ensure within limits
        for i, joint_name in enumerate(self.joint_names):
            min_limit, max_limit = self.joint_limits[joint_name]
            joint_angles[i] = np.clip(joint_angles[i], min_limit, max_limit)
        
        return joint_angles
    
    def get_system_info(self) -> dict:
        """Get comprehensive system information."""
        current_joints = self.get_current_joint_positions()
        current_velocities = self.get_current_joint_velocities()
        
        return {
            'time': self.time,
            'control_mode': self.control_mode.value,
            'robot_state': self.robot_state.value,
            'joint_positions': current_joints.tolist(),
            'joint_velocities': current_velocities.tolist(),
            'target_positions': self.target_positions.tolist(),
            'executing_sequence': self.executing_sequence,
            'moves_remaining': len(self.move_sequence) - self.current_move_index if self.executing_sequence else 0
        }
    
    def emergency_stop(self):
        """Emergency stop - immediately halt all motion and hold position."""
        print("Starting emergency stop control mode now")
        
        # Set current position as target to hold position
        self.target_positions = self.get_current_joint_positions()
        
        # Stop any ongoing sequences
        self.executing_sequence = False
        self.robot_state = RobotState.IDLE
        
        # Apply strong damping to stop movement quickly
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            actuator_id = self.actuator_indices[joint_name]
            
            # Apply damping control to stop motion
            current_velocity = self.data.qvel[joint_id]
            damping_control = -5.0 * current_velocity  # Strong damping
            self.data.ctrl[actuator_id] = damping_control
    
    def hold_position(self):
        """Hold current position with minimal control to prevent drift."""
        # Set current joint positions as targets
        current_positions = self.get_current_joint_positions()
        
        # Only update targets if they're significantly different (avoid tiny adjustments)
        for i in range(len(self.target_positions)):
            pos_diff = abs(current_positions[i] - self.target_positions[i])
            if pos_diff > 0.05:  # Only update if difference is significant
                self.target_positions[i] = current_positions[i]


def load_scene_xml(scene_xml_path: str) -> str:
    """Load a pre-built scene XML file."""
    if not os.path.exists(scene_xml_path):
        raise FileNotFoundError(f"Scene XML file not found at: {scene_xml_path}")
    
    with open(scene_xml_path, 'r', encoding='utf-8') as f:
        xml_content = f.read()
    
    return xml_content


def control_callback_global(model, data):
    """Global control callback function for use with mj.set_mjcb_control()."""
    # This would be set globally and called by MuJoCo
    # You would need to store a reference to your controller instance
    pass


def main():
    """Main function to run the enhanced Hanoi simulation."""
    try:
        # Define paths
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        scene_xml_path = os.path.join(project_root, "xmls", "generated", "panda_hanoi_scene.xml")
        
        # Normalize path
        scene_xml_path = os.path.normpath(scene_xml_path)
        
        print(f"Loading MuJoCo scene from: {scene_xml_path}")
        
        # Check if scene XML exists
        if not os.path.exists(scene_xml_path):
            print(f"\n❌ Scene XML file not found at: {scene_xml_path}")
            print("🔧 Please run 'python build_scene.py' first to generate the scene XML.")
            return 1
        
        # Load the pre-built XML content
        xml_content = load_scene_xml(scene_xml_path)
        
        # Create MuJoCo model from the pre-built XML
        print("Creating MuJoCo model from XML...")
        model = mujoco.MjModel.from_xml_string(xml_content)
        data = mujoco.MjData(model)
        
        # Initialize advanced robot controller
        controller = PandaAdvancedController(model, data)
        
        # Optional: Set up global control callback
        # mujoco.set_mjcb_control(controller.control_callback)
        
        # Launch viewer for the simulation
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set initial camera position
            viewer.cam.azimuth = 45
            viewer.cam.elevation = -20
            viewer.cam.distance = 3.0
            viewer.cam.lookat[:] = [0, 0, 0.5]
            print("MuJoCo Viewer launched!")
            
            print("\n" + "="*50)
            print("KHANOI - Enhanced Control Functions Available:")
            print("="*50)
            print("• Position, Velocity, Torque, and Mixed control modes")
            print("• Smooth trajectory planning and execution") 
            print("• Hanoi-specific functions: pick_disk(), place_disk()")
            print("• Automated Hanoi solving: solve_hanoi()")
            print("• State machine for complex sequences")
            print("• Emergency stop and safety functions")
            print("• Real-time system monitoring")
            print("="*50)
            
            # Demonstration sequence
            print("\nStarting demonstration control mode now...")
            
            # Wait a bit, then demonstrate some functions
            demo_start_time = time.perf_counter()
            
            # Main simulation loop
            while viewer.is_running():
                step_start = time.perf_counter()
                
                # Update trajectory if moving
                controller._update_trajectory()
                
                # Update robot control
                controller.update_control(model.opt.timestep)
                
                # Demonstration sequence
                elapsed_demo_time = time.perf_counter() - demo_start_time
                
                if 2.0 < elapsed_demo_time < 2.1:
                    controller.set_control_mode(ControlMode.POSITION)
                    
                elif 4.0 < elapsed_demo_time < 4.1:
                    test_position = np.array([0.2, -0.3, 0.1, -1.2, 0.0, 1.0, -0.5])
                    controller.move_to_position(test_position, duration=3.0)
                    
                elif 8.0 < elapsed_demo_time < 8.1:
                    controller.move_to_position(controller.home_qpos, duration=2.0)
                    
                elif 12.0 < elapsed_demo_time < 12.1:
                    controller.pick_disk('A', disk_level=0)
                    
                elif 16.0 < elapsed_demo_time < 16.1:
                    controller.solve_hanoi(n_disks=3)
                
                # Physics step
                mujoco.mj_step(model, data)
                
                # Real-time sync
                time_until_next_step = model.opt.timestep - (time.perf_counter() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                
                # Update viewer
                viewer.sync()
                
                # Print system info periodically
                if int(elapsed_demo_time) % 5 == 0 and elapsed_demo_time % 5 < 0.1:
                    info = controller.get_system_info()
                    print(f"System Info - Mode: {info['control_mode']}, State: {info['robot_state']}, Time: {info['time']:.1f}s")
        
        print("✅ Enhanced simulation completed successfully!")
        
    except Exception as e:
        print(f"\n❌ Error occurred: {str(e)}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())