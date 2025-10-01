#!/usr/bin/env python3
"""
MuJoCo Panda Robot Spinning Simulation

This script creates a simulation with a Franka Emika Panda robot that:
- Spins joint1 and joint7 continuously
- Opens and closes the grippers in a rhythmic pattern
- Provides visual feedback through the MuJoCo viewer

Controls:
- Press SPACE to pause/resume the simulation
- Press ESC to exit
- Press R to reset to home position
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import math


class PandaSpinningController:
    def __init__(self, model_path="models/robots/franka_emika_panda/panda.xml"):
        """Initialize the Panda robot controller."""
        # Load the MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Get joint and actuator indices
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        self.joint_indices = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) 
                             for name in self.joint_names}
        
        # Gripper actuator index (actuator8 controls the gripper)
        self.gripper_actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "actuator8")
        
        # Actuator indices for arm joints
        self.actuator_indices = {f'joint{i+1}': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"actuator{i+1}") 
                                for i in range(7)}
        
        # Control parameters
        self.time = 0.0
        self.spinning_joints = ['joint1', 'joint7']  # Joints that will spin
        self.spin_speed = 2.0  # radians per second
        self.gripper_frequency = 0.5  # Hz (cycles per second)
        
        # Home position
        self.home_qpos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
        
        # Reset to home position
        self.reset_to_home()
        
        print("Panda Spinning Controller initialized!")
        print("Spinning joints:", self.spinning_joints)
        print("Press SPACE to pause/resume, R to reset, ESC to exit")

    def reset_to_home(self):
        """Reset the robot to home position."""
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            self.data.qpos[joint_id] = self.home_qpos[i]
        
        # Set gripper to closed position
        self.data.ctrl[self.gripper_actuator_id] = 255  # Closed
        
        # Set arm joint controls to home position
        for i, joint_name in enumerate(self.joint_names):
            actuator_id = self.actuator_indices[joint_name]
            self.data.ctrl[actuator_id] = self.home_qpos[i]
        
        mujoco.mj_forward(self.model, self.data)

    def update_control(self, dt):
        """Update robot control commands."""
        self.time += dt
        
        # Control spinning joints
        for joint_name in self.spinning_joints:
            actuator_id = self.actuator_indices[joint_name]
            
            if joint_name == 'joint1':
                # Spin joint1 continuously
                target_angle = self.spin_speed * self.time
                # Keep within joint limits
                target_angle = np.clip(target_angle % (2 * np.pi), -2.8973, 2.8973)
                self.data.ctrl[actuator_id] = target_angle
                
            elif joint_name == 'joint7':
                # Spin joint7 in opposite direction
                target_angle = -self.spin_speed * self.time
                # Keep within joint limits
                target_angle = np.clip(target_angle % (2 * np.pi), -2.8973, 2.8973)
                self.data.ctrl[actuator_id] = target_angle
        
        # Control gripper - oscillate between open and closed
        gripper_phase = 2 * np.pi * self.gripper_frequency * self.time
        gripper_control = 127.5 + 127.5 * np.sin(gripper_phase)  # Oscillate between 0 and 255
        self.data.ctrl[self.gripper_actuator_id] = gripper_control
        
        # Keep other joints at their home positions (for stability)
        stable_joints = ['joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for i, joint_name in enumerate(stable_joints):
            actuator_id = self.actuator_indices[joint_name]
            home_index = self.joint_names.index(joint_name)
            self.data.ctrl[actuator_id] = self.home_qpos[home_index]

    def print_status(self):
        """Print current robot status."""
        print(f"\nTime: {self.time:.2f}s")
        print("Joint Positions:")
        for joint_name in self.joint_names:
            joint_id = self.joint_indices[joint_name]
            angle_deg = np.degrees(self.data.qpos[joint_id])
            print(f"  {joint_name}: {angle_deg:.1f}°")
        
        gripper_pos = self.data.ctrl[self.gripper_actuator_id]
        gripper_percent = (gripper_pos / 255.0) * 100
        print(f"  Gripper: {gripper_percent:.1f}% closed")

    def run_simulation(self):
        """Run the interactive simulation."""
        paused = False
        last_status_time = 0
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # Set camera position for better view
            viewer.cam.distance = 2.5
            viewer.cam.elevation = -30
            viewer.cam.azimuth = 45
            viewer.cam.lookat[0] = 0.0
            viewer.cam.lookat[1] = 0.0
            viewer.cam.lookat[2] = 0.5
            
            print("MuJoCo Viewer launched!")
            print("The robot will start spinning joint1 and joint7, and opening/closing grippers")
            print("Close the viewer window to exit the simulation")
            
            while viewer.is_running():
                step_start = time.time()
                
                if not paused:
                    # Update control
                    self.update_control(self.model.opt.timestep)
                    
                    # Step the simulation
                    mujoco.mj_step(self.model, self.data)
                
                # Print status every 3 seconds
                if self.time - last_status_time >= 3.0:
                    self.print_status()
                    last_status_time = self.time
                
                # Sync with real-time
                viewer.sync()
                
                # Sleep to maintain real-time execution
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)


def main():
    """Main function to run the simulation."""
    try:
        controller = PandaSpinningController()
        controller.run_simulation()
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure the Panda robot model exists at 'models/robots/franka_emika_panda/panda.xml'")


if __name__ == "__main__":
    main()