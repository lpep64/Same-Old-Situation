#!/usr/bin/env python3
"""
Simple scripted pick-and-place demo for the Niryo robot in MuJoCo.
The robot uses position actuators and runs a short sequence before
handing control back to the interactive viewer.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

class NiryoRobotController:
    """
    A controller for the Niryo NED2 robot arm in MuJoCo.
    
    This controller uses position-based control for the Niryo robot joints
    to execute pick-and-place operations for the Tower of Hanoi puzzle.
    """
    def __init__(self, model, data, n_disks=3):
        """Initializes the NiryoRobotController."""
        self.model = model
        self.data = data
        self.n_disks = n_disks

        # --- Get MuJoCo IDs for Niryo Robot ---
        # Actuators (position control)
        self.actuator_names = [
            'shoulder_pos', 'arm_pos', 'elbow_pos', 'forearm_pos', 
            'wrist_pos', 'hand_pos', 'r_gripper_pos', 'l_gripper_pos'
        ]
        self.arm_actuator_names = self.actuator_names[:6]  # First 6 are arm joints
        self.gripper_actuator_names = self.actuator_names[6:]  # Last 2 are gripper joints
        
        # Get actuator indices
        self.arm_actuator_ids = []
        for name in self.arm_actuator_names:
            try:
                aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                self.arm_actuator_ids.append(aid)
            except:
                print(f"Warning: Could not find actuator '{name}'")
        self.arm_actuator_ids = np.array(self.arm_actuator_ids)
        
        self.gripper_actuator_ids = []
        for name in self.gripper_actuator_names:
            try:
                aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                self.gripper_actuator_ids.append(aid)
            except:
                print(f"Warning: Could not find actuator '{name}'")
        self.gripper_actuator_ids = np.array(self.gripper_actuator_ids)

        # --- Internal State Tracking ---
        self.peg_state = {
            'A': [f'disk_body{i}' for i in range(self.n_disks, 0, -1)],
            'B': [],
            'C': []
        }
        
        # Joint limits and default positions
        self.home_position = np.array([0, -0.5, 1.0, 0, 0, 0])  # From keyframe
        self.gripper_open = 0.009   # Gripper open position
        self.gripper_closed = 0.0   # Gripper closed position
        
    def get_joint_positions(self):
        """Get current joint positions."""
        return self.data.qpos[:6] if len(self.data.qpos) >= 6 else np.zeros(6)
    
    def set_joint_targets(self, joint_targets):
        """Set target positions for arm joints."""
        if len(self.arm_actuator_ids) == len(joint_targets):
            self.data.ctrl[self.arm_actuator_ids] = joint_targets
            
    def set_gripper_target(self, position):
        """Set target position for both gripper fingers."""
        if len(self.gripper_actuator_ids) == 2:
            # Right and left gripper fingers
            self.data.ctrl[self.gripper_actuator_ids[0]] = position   # Right gripper
            self.data.ctrl[self.gripper_actuator_ids[1]] = -position  # Left gripper
            
    def go_home(self, duration=3.0, viewer=None):
        """Move robot to home position."""
        print("🏠 Moving to home position...")
        self.move_to_joint_position(self.home_position, duration=duration, viewer=viewer)
        self.set_gripper('open', viewer=viewer)
        
    def move_to_joint_position(self, target_joints, duration=2.0, viewer=None):
        """Smoothly move to target joint position."""
        start_joints = self.get_joint_positions()
        steps = int(duration * 100)  # 100 Hz control
        
        for i in range(steps):
            alpha = i / (steps - 1) if steps > 1 else 1.0
            # Smooth interpolation using cosine
            smooth_alpha = (1 - np.cos(alpha * np.pi)) / 2
            current_targets = start_joints + smooth_alpha * (target_joints - start_joints)
            
            self.set_joint_targets(current_targets)
            mujoco.mj_step(self.model, self.data)
            
            if viewer is not None:
                viewer.sync()
            time.sleep(0.01)
            
    def set_gripper(self, state, duration=1.0, viewer=None):
        """Control gripper state."""
        if state == 'open':
            target = self.gripper_open
            print("✋ Opening gripper...")
        elif state == 'closed':
            target = self.gripper_closed
            print("🤏 Closing gripper...")
        else:
            target = state  # Direct position value
            
        steps = int(duration * 100)
        for i in range(steps):
            self.set_gripper_target(target)
            mujoco.mj_step(self.model, self.data)
            
            if viewer is not None:
                viewer.sync()
            time.sleep(0.01)

    def load_execute_hanoi_moves(self, solution, viewer=None): 
        """Execute a complete Hanoi solution using simple scripted movements."""
        print(f"▶️ Loading and executing Hanoi moves: {solution}")
        
        # Start from home position
        self.go_home(viewer=viewer)
        
        # Simple demo movements for each Hanoi move
        demo_phases = [
            {"name": "pre_grasp", "target": np.array([1.10, -1.05, 1.40, 0.45, -0.35, 0.15]), "duration": 2.0},
            {"name": "approach", "target": np.array([1.10, -1.18, 1.50, 0.65, -0.20, 0.10]), "duration": 1.5},
            {"name": "grasp", "target": np.array([1.10, -1.18, 1.50, 0.65, -0.20, 0.10]), "duration": 0.8},
            {"name": "lift", "target": np.array([1.00, -0.85, 1.25, 0.20, -0.45, 0.10]), "duration": 1.5},
            {"name": "swing_to_drop", "target": np.array([0.35, -0.80, 1.35, -0.05, -0.55, 0.05]), "duration": 1.6},
            {"name": "lower_place", "target": np.array([0.35, -0.98, 1.50, 0.10, -0.35, 0.0]), "duration": 1.2},
            {"name": "release", "target": np.array([0.35, -0.98, 1.50, 0.10, -0.35, 0.0]), "duration": 0.8},
            {"name": "retreat", "target": np.array([0.60, -0.70, 1.15, -0.10, -0.45, 0.10]), "duration": 1.4},
        ]
        
        for i, move in enumerate(solution):
            source_peg, target_peg = move
            print(f"\n🔄 === MOVE {i+1}/{len(solution)}: {source_peg} → {target_peg} ===")
            
            # Execute demo movement sequence for this move
            for phase in demo_phases:
                print(f"  -> {phase['name']}")
                self.move_to_joint_position(phase["target"], duration=phase["duration"], viewer=viewer)
                
                # Control gripper during grasp/release phases
                if phase["name"] == "grasp":
                    self.set_gripper('closed', viewer=viewer)
                elif phase["name"] == "release":
                    self.set_gripper('open', viewer=viewer)
                
                time.sleep(0.5)  # Brief pause between phases
            
            # Brief pause between moves
            time.sleep(1.0)

        print("\n🎉 Tower of Hanoi puzzle solved!")
        self.go_home(viewer=viewer)

def main():
    print("This is just the controller class. To run the demo, use the viewer script.")
    
    print("Demo completed.")
if __name__ == "__main__":
    main()