#!/usr/bin/env python3
"""
Panda Hanoi
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import sys

# Add parent directory to path so we can import utils
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class PandaManualController:
    """Manual controller for the Panda robot with direct joint position control."""
    
    def __init__(self, model, data):
        """Initialize the Panda robot controller with provided model and data."""
        self.model = model
        self.data = data
        
        # Get joint and actuator indices
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        try:
            self.joint_indices = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) 
                                 for name in self.joint_names}
            
            # Gripper actuator index (actuator8 controls the gripper)
            self.gripper_actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "actuator8")
            
            # Actuator indices for arm joints
            self.actuator_indices = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"actuator{i+1}") 
                                    for i, name in enumerate(self.joint_names)}
        except Exception as e:
            raise RuntimeError(f"Failed to find robot joints/actuators: {e}")
        
        # Control parameters
        self.time = 0.0
        
        # Home position
        self.home_qpos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
        
        # Reset to home position
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            self.data.qpos[joint_id] = self.home_qpos[i]
        
        mujoco.mj_forward(self.model, self.data)
        
        print("Panda Manual Controller initialized!")
    
    def move(self, j1=None, j2=None, j3=None, j4=None, j5=None, j6=None, j7=None, gripper=None):
        """Set joint positions directly.
        
        Args:
            j1-j7: Target joint angles in radians (None = don't move that joint)
            gripper: Target gripper position 0-255 (None = don't move gripper)
        """
        # Simple direct control - no deadband, no smooth movement
        if j1 is not None:
            self.data.ctrl[self.actuator_indices['joint1']] = j1
        if j2 is not None:
            self.data.ctrl[self.actuator_indices['joint2']] = j2
        if j3 is not None:
            self.data.ctrl[self.actuator_indices['joint3']] = j3
        if j4 is not None:
            self.data.ctrl[self.actuator_indices['joint4']] = j4
        if j5 is not None:
            self.data.ctrl[self.actuator_indices['joint5']] = j5
        if j6 is not None:
            self.data.ctrl[self.actuator_indices['joint6']] = j6
        if j7 is not None:
            self.data.ctrl[self.actuator_indices['joint7']] = j7
        if gripper is not None:
            self.data.ctrl[self.gripper_actuator_id] = gripper

    def update_control(self, dt):
        """Simple manual control - set each joint position directly."""
        self.time += dt
        
        # Manual joint controls - adjust these values to move the robot
        # self.data.ctrl[self.actuator_indices['joint1']] = joint1_target
        # self.data.ctrl[self.actuator_indices['joint2']] = joint2_target    # Range: -1.7628 to +1.7628 radians
        # self.data.ctrl[self.actuator_indices['joint3']] = joint3_target    # Range: -2.8973 to +2.8973 radians
        # self.data.ctrl[self.actuator_indices['joint4']] = joint4_target    # Range: -3.0718 to -0.0698 radians
        # self.data.ctrl[self.actuator_indices['joint5']] = joint5_target    # Range: -2.8973 to +2.8973 radians
        # self.data.ctrl[self.actuator_indices['joint6']] = joint6_target    # Range: -0.0175 to +3.7525 radians
        # self.data.ctrl[self.actuator_indices['joint7']] = joint7_target    # Range: -2.8973 to +2.8973 radians
        # self.data.ctrl[self.gripper_actuator_id] = gripper_target          # Range: 0 (closed) to 255 (open)
        # Apply the controls
        if self.time > 2.0 and self.time <= 4.0:
            self.move(j2=0.5, j4=-0.6, j6=0.3, gripper=255)

        elif self.time > 4.0 and self.time <= 6.0:
            self.move(j2=0.5, j4=-0.6, j6=0.3, gripper=255)



def load_scene_xml(scene_xml_path: str) -> str:
    """Load a pre-built scene XML file.
    
    Args:
        scene_xml_path: Path to the pre-built scene XML file
        
    Returns:
        XML content as string
    """
    if not os.path.exists(scene_xml_path):
        raise FileNotFoundError(f"Scene XML file not found at: {scene_xml_path}")
    
    with open(scene_xml_path, 'r', encoding='utf-8') as f:
        xml_content = f.read()
    
    return xml_content


def main():
    """Main function to run the unified simulation using a pre-built XML file."""
    try:
        # Define paths
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        scene_xml_path = os.path.join(project_root, "xmls", "generated", "panda_hanoi_scene.xml")
        
        # Normalize path
        scene_xml_path = os.path.normpath(scene_xml_path)
        
        print(f"Loading pre-built MuJoCo scene from: {scene_xml_path}")
        
        # Check if scene XML exists, if not, suggest building it
        if not os.path.exists(scene_xml_path):
            print(f"\n❌ Scene XML file not found at: {scene_xml_path}")
            print("🔧 Please run 'python build_scene.py' first to generate the scene XML.")
            return 1
        
        # Load the pre-built XML content
        xml_content = load_scene_xml(scene_xml_path)
        
        # Create MuJoCo model from the pre-built XML
        print("Creating MuJoCo model from pre-built XML...")
        model = mujoco.MjModel.from_xml_string(xml_content)
        data = mujoco.MjData(model)
        
        # Initialize robot controller
        controller = PandaManualController(model, data)
        
        # Launch viewer for the simulation
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set initial camera position
            viewer.cam.azimuth = 45
            viewer.cam.elevation = -20
            viewer.cam.distance = 3.0
            viewer.cam.lookat[:] = [0, 0, 0.5]
            print("MuJoCo Viewer launched!")
            
            # Main simulation loop
            while viewer.is_running():
                step_start = time.perf_counter()
                
                # Update robot control
                controller.update_control(model.opt.timestep)
                
                # Physics step
                mujoco.mj_step(model, data)
                
                # Real-time sync
                time_until_next_step = model.opt.timestep - (time.perf_counter() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                
                # Update viewer
                viewer.sync()
        
        print("✅ Simulation completed successfully!")
        
    except Exception as e:
        print(f"\n❌ Error occurred: {str(e)}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())