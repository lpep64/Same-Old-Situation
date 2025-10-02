#!/usr/bin/env python3
"""
Panda Spinning Robot with Sustainable XML Builder

This file demonstrates how to use the MuJoCoXMLBuilder for a clean,
maintainable approach to building MuJoCo simulations.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import sys

# Add parent directory to path so we can import utils
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import our new XML builder
from utils.mujoco_xml_builder import create_scene_with_robot, MuJoCoXMLBuilder


class PandaSpinningController:
    """Controller for the Panda robot with spinning joints and gripper control."""
    
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
        self.spinning_joints = ['joint1', 'joint7']  # Joints that will spin
        self.spin_speed = 2.0  # radians per second
        self.gripper_frequency = 0.5  # Hz (cycles per second)
        
        # Home position
        self.home_qpos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
        
        # Reset to home position
        self.reset_to_home()
        
        print("Panda Spinning Controller initialized!")
        print("Spinning joints:", self.spinning_joints)
    
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
                self.data.ctrl[actuator_id] = self.spin_speed * self.time
                
            elif joint_name == 'joint7':
                # Spin joint7 in opposite direction
                self.data.ctrl[actuator_id] = -self.spin_speed * self.time
        
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


def create_custom_scene_builder(robot_path: str) -> MuJoCoXMLBuilder:
    """Create a custom scene using the XML builder system.
    
    Args:
        robot_path: Path to the robot XML file
        
    Returns:
        Configured MuJoCoXMLBuilder
    """
    # Create the builder with custom settings
    builder = MuJoCoXMLBuilder("panda_spinning_scene")
    
    # Configure physics
    builder.set_physics_options(
        timestep=0.005,
        integrator="RK4"
    )
    
    # Configure visuals
    builder.set_visual_options(
        headlight={"ambient": "0.3 0.3 0.3", "diffuse": "0.7 0.7 0.7"},
        global_={"offwidth": "1920", "offheight": "1080"},
        quality={"shadowsize": "2048", "offsamples": "8"}
    )
    
    # Add custom textures
    builder.add_texture("checker_floor", "2d",
                       builtin="checker",
                       rgb1="0.2 0.3 0.4",  # Blueish
                       rgb2="0.8 0.9 1.0",  # Light blue
                       width="512",
                       height="512")
    
    builder.add_texture("gradient_sky", "skybox",
                       builtin="gradient",
                       rgb1="0.4 0.6 0.8",  # Sky blue
                       rgb2="0.1 0.2 0.4",  # Darker blue
                       width="1024",
                       height="1024",
                       mark="random",
                       markrgb="1 1 1")
    
    # Add materials
    builder.add_material("floor_material", 
                        texture="checker_floor", 
                        texrepeat="8 8", 
                        reflectance="0.1")
    
    # Import the robot
    mesh_dir = "../../../models/robots/franka_emika_panda/assets"
    builder.import_robot_from_xml(robot_path, mesh_dir)
    
    # Add scene elements
    builder.add_floor(pos=(0, 0, -0.05), 
                     size=(6, 6, 0.05), 
                     material="floor_material")
    
    # Add multiple lights for better illumination
    builder.add_light("main_light",
                     pos=(0, 0, 3),
                     directional=False,
                     ambient="0.3 0.3 0.3",
                     diffuse="0.9 0.9 0.9",
                     specular="0.5 0.5 0.5")
    
    builder.add_light("side_light",
                     pos=(3, 3, 2),
                     directional=True,
                     dir="-1 -1 -0.5",
                     ambient="0.1 0.1 0.2",
                     diffuse="0.6 0.7 0.8",
                     specular="0.2 0.2 0.3")
    
    return builder


def main():
    """Main function to run the unified simulation using the XML builder."""
    try:
        # Define robot path
        robot_path = "../../../models/robots/franka_emika_panda/panda.xml"
        
        if not os.path.exists(robot_path):
            raise FileNotFoundError(f"Robot model not found at: {robot_path}")
        
        print("Building MuJoCo scene using XML Builder...")
        
        # Method 1: Use the simple scene creator
        # builder = create_scene_with_robot(robot_path)
        
        # Method 2: Use custom scene builder for more control
        builder = create_custom_scene_builder(robot_path)
        
        # Build the XML
        xml_content = builder.build_xml()
        
        # Optionally save the generated XML for debugging (relative to project root)
        debug_xml_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "xmls", "generated", "debug_generated_scene.xml")
        builder.save_xml(debug_xml_path)
        
        # Create MuJoCo model from the generated XML
        print("Creating MuJoCo model from generated XML...")
        model = mujoco.MjModel.from_xml_string(xml_content)
        data = mujoco.MjData(model)
        
        # Initialize robot controller
        controller = PandaSpinningController(model, data)
        
        # Launch viewer for the simulation
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set initial camera position
            viewer.cam.azimuth = 45
            viewer.cam.elevation = -20
            viewer.cam.distance = 3.0
            viewer.cam.lookat[:] = [0, 0, 0.5]
            
            print("🚀 MuJoCo Viewer launched!")
            print("📝 Scene built using sustainable XML Builder system")
            print("🤖 Robot spinning joints 1 & 7, gripper oscillating")
            print("🎥 Close viewer window to exit")
            
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