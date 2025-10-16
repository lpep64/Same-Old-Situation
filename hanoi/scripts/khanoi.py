#!/usr/bin/env python3
"""
Panda Robot Controller - Optimized for ML/Optimization

SIMPLIFIED FOR MACHINE LEARNING:
- Position control only (most stable and professional)
- 7 DOF action space: joint angles in radians
- Clear joint limits and ranges documented
- Easy integration with RL frameworks (Gym, Stable-Baselines3, etc.)

KEY FUNCTIONS FOR ML:
- set_joint_targets(action_array): Vectorized control  
- move_to_position(target_joints, duration): Smooth trajectory with None support
- get_current_joint_positions(): State observation
- Joint limits available in self.joint_limits

JOINT RANGES (Action Space):
joint1: -2.90 to +2.90 rad (base rotation)
joint2: -1.76 to +1.76 rad (shoulder) 
joint3: -2.90 to +2.90 rad (elbow)
joint4: -3.07 to -0.07 rad (wrist1)
joint5: -2.90 to +2.90 rad (wrist2)
joint6: -0.02 to +3.75 rad (wrist3) 
joint7: -2.90 to +2.90 rad (wrist4)
gripper: 0 to 255 (0=closed, 255=open)
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os
from enum import Enum

class RobotState(Enum):
    """Robot operation states - simplified"""
    IDLE = "idle"
    IN_MOTION = "in_motion"


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
        self.robot_state = RobotState.IDLE
        
        # ========================================
        # JOINT LIMITS AND CONTROL PARAMETERS
        # ========================================
        # Home position and joint limits (in radians)
        self.joint_limits = {
            'joint1': (-2.8973, 2.8973),    # Base rotation
            'joint2': (-1.7628, 1.7628),    # Shoulder
            'joint3': (-2.8973, 2.8973),    # Elbow
            'joint4': (-3.0718, -0.0698),   # Wrist1
            'joint5': (-2.8973, 2.8973),    # Wrist2
            'joint6': (-0.0175, 3.7525),    # Wrist3
            'joint7': (-2.8973, 2.8973)     # Wrist rotate
        }
        
        # ========================================
        # CONTROL GAINS - OPTIMIZED FOR PRECISION
        # ========================================
        # PD control gains (tuned for stable, precise movements)
        self.kp = np.array([25.0, 25.0, 25.0, 25.0, 20.0, 20.0, 15.0])  # Position gains
        self.kd = np.array([5.0, 5.0, 5.0, 5.0, 4.0, 4.0, 3.0])        # Velocity gains
        
        # Control stability parameters
        self.control_limit = 100.0       # Maximum control effort
        
        # Trajectory tracking
        self.target_positions = np.array([0, 0, 0, 0, 0, 0, 0])
        self.trajectory_start_time = 0.0
        self.trajectory_duration = 2.0
        self.trajectory_start_pos = np.array([0, 0, 0, 0, 0, 0, 0])
        self.trajectory_target_pos = np.array([0, 0, 0, 0, 0, 0, 0])
        
        # Gripper control
        self.gripper_open = 255
        self.gripper_closed = 0
        
        # Reset to home position
        self.reset_to_home()
        print("Starting advanced panda controller control mode now")

    # ========================================
    # CORE CONTROL FUNCTIONS - POSITION ONLY
    # ========================================
    
    def update_control(self, dt):
        """
        Main control update function - POSITION CONTROL ONLY
        
        For ML/Optimization:
        - Modify self.target_positions[i] to control joint i
        - Range: Use self.joint_limits[joint_name] for valid ranges
        - Example: self.target_positions[0] = 1.5  # Move base joint
        """
        self.time += dt
        
        # Only apply control when robot is in motion
        # When IDLE, push None array to joints (doesn't trigger idle switch)
        if self.robot_state == RobotState.IDLE:
            # Push None values to all joint controllers
            for i, joint_name in enumerate(self.joint_names):
                actuator_id = self.actuator_indices[joint_name]
                self.data.ctrl[actuator_id] = None
            return
            
        # Apply position control only when moving
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            actuator_id = self.actuator_indices[joint_name]
            
            # Simple PD control
            pos_error = self.target_positions[i] - self.data.qpos[joint_id]
            current_velocity = self.data.qvel[joint_id]
            
            vel_error = 0.0 - current_velocity
            control_signal = self.kp[i] * pos_error + self.kd[i] * vel_error
            
            # Limit control effort for stability
            control_signal = np.clip(control_signal, -self.control_limit, self.control_limit)
            self.data.ctrl[actuator_id] = control_signal
    
    def set_joint_targets(self, joint_angles: np.ndarray):
        """
        Set target joint positions for ML/Optimization control
        
        Args:
            joint_angles: Array of 7 joint angles in radians
        """
        if len(joint_angles) != 7:
            raise ValueError("joint_angles must be array of 7 values")
        
        # First thing: exit IDLE state when control is requested
        if self.robot_state == RobotState.IDLE:
            self.robot_state = RobotState.IN_MOTION
        
        # Apply joint limits for safety
        for i, joint_name in enumerate(self.joint_names):
            min_limit, max_limit = self.joint_limits[joint_name]
            self.target_positions[i] = np.clip(joint_angles[i], min_limit, max_limit)

    # ========================================
    # BASIC MOVEMENT FUNCTIONS
    # ========================================
    
    def move_to_position(self, target_joints, duration: float = 2.0):
        """
        Smooth movement to target joint configuration over specified duration.
        
        Args:
            target_joints: Can be:
                - np.ndarray of 7 joint angles
                - List/tuple of 7 values (None values keep current position)
            duration: Movement duration in seconds
            
        Example with None values:
            # Only move joints 1, 3, and 5, keep others at current position
            controller.move_to_position([0.5, None, 0.2, None, -0.3, None, None])
        """
        self.robot_state = RobotState.IN_MOTION

        #if j1 is not None:
        #    self.data.ctrl[self.actuator_indices['joint1']] = j1
        #if j2 is not None:
        #    self.data.ctrl[self.actuator_indices['joint2']] = j2
        #if j3 is not None:
        #    self.data.ctrl[self.actuator_indices['joint3']] = j3
        #if j4 is not None:
        #    self.data.ctrl[self.actuator_indices['joint4']] = j4
        #if j5 is not None:
        #    self.data.ctrl[self.actuator_indices['joint5']] = j5
        #if j6 is not None:
        #    self.data.ctrl[self.actuator_indices['joint6']] = j6
        #if j7 is not None:
        #    self.data.ctrl[self.actuator_indices['joint7']] = j7
        #if gripper is not None:
        #    self.data.ctrl[self.gripper_actuator_id] = gripper
        #    
        # Handle different input types
        if isinstance(target_joints, (list, tuple)):
            # Convert to array, handling None values
            current_positions = self.get_current_joint_positions()
            target_array = np.zeros(7)
            for i, value in enumerate(target_joints):
                if i >= 7:
                    break
                target_array[i] = current_positions[i] if value is None else value
            target_joints = target_array
        elif isinstance(target_joints, np.ndarray):
            target_joints = target_joints.copy()
        else:
            raise ValueError("target_joints must be array, list, or tuple")
        
        # Check all joint limits
        for i, (joint_name, target) in enumerate(zip(self.joint_names, target_joints)):
            min_limit, max_limit = self.joint_limits[joint_name]
            target_joints[i] = np.clip(target, min_limit, max_limit)
        
        # Set up trajectory
        self.trajectory_start_time = self.time
        self.trajectory_duration = duration
        self.trajectory_start_pos = self.get_current_joint_positions()
        self.trajectory_target_pos = target_joints.copy()
        self.robot_state = RobotState.IN_MOTION
            
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
        if self.robot_state != RobotState.IN_MOTION:
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
    # UTILITY FUNCTIONS
    # ================================
    
    def reset_to_home(self):
        """Reset robot to home position."""
        print("Starting home position control mode now")
        self.move_to_position([0,0,0,0,0,0,0], duration=2.0)

    """
    This is where you can implement your own custom movement sequences!
    
    Available methods you can use:
    - self.set_joint_targets(joint_array)                     # Set all joints at once
    - self.move_to_position(target_joints, duration)          # Smooth trajectory (supports None values)
    - self.get_current_joint_positions()                      # Get current state
    - self.get_current_joint_velocities()                     # Get current velocities
    
    Joint limits: See file header for detailed ranges
    Gripper: Control via self.data.ctrl[self.gripper_actuator_id] = value (0-255)
    """

def load_scene_xml(scene_xml_path: str) -> str:
    """Load a pre-built scene XML file."""
    if not os.path.exists(scene_xml_path):
        raise FileNotFoundError(f"Scene XML file not found at: {scene_xml_path}")
    
    with open(scene_xml_path, 'r', encoding='utf-8') as f:
        xml_content = f.read()
    
    return xml_content


def main():
    """Main function to run the Panda robot simulation."""
    # Define paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    scene_xml_path = os.path.join(project_root, "xmls", "generated", "panda_hanoi_scene.xml")
    
    # Normalize path
    scene_xml_path = os.path.normpath(scene_xml_path)
    
    print(f"Loading MuJoCo scene from: {scene_xml_path}")
    
    # Load the pre-built XML content
    xml_content = load_scene_xml(scene_xml_path)
    
    # Create MuJoCo model from the pre-built XML
    print("Creating MuJoCo model from XML...")
    model = mujoco.MjModel.from_xml_string(xml_content)
    data = mujoco.MjData(model)
    
    # Initialize advanced robot controller
    controller = PandaAdvancedController(model, data)
    
    # Launch viewer for the simulation
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set initial camera position
        viewer.cam.azimuth = 45
        viewer.cam.elevation = -20
        viewer.cam.distance = 3.0
        viewer.cam.lookat[:] = [0, 0, 0.5]
                    
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
                print("Demo: Moving to home position")
                controller.reset_to_home()
                
            elif 4.0 < elapsed_demo_time < 4.1:
                print("Demo: Moving to test position")
                test_position = np.array([0.2, -0.3, 0.1, -1.2, 0.0, 1.0, -0.5])
                controller.move_to_position(test_position, duration=3.0)
                
            elif 12.0 < elapsed_demo_time < 12.1:
                print("Demo: Testing set_joint_targets")
                test_joints = np.array([0.5, -0.2, 0.3, -1.0, 0.1, 1.2, -0.3])
                controller.set_joint_targets(test_joints)
                
            elif 14.0 < elapsed_demo_time < 14.1:
                print("Demo: Getting joint velocities")
                velocities = controller.get_current_joint_velocities()
                print(f"Current joint velocities: {velocities}")
            
            else: # After 16 seconds, reset demo
                if elapsed_demo_time > 16.0:
                    demo_start_time = time.perf_counter()
                    print("Demo sequence restarting...")
            
            # Physics step
            mujoco.mj_step(model, data)
            
            # Real-time sync
            time_until_next_step = model.opt.timestep - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            
            # Update viewer
            viewer.sync()

    return 0


if __name__ == "__main__":
    exit(main())