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

# Import our new XML builder
from utils.mujoco_xml_builder import create_scene_with_robot, MuJoCoXMLBuilder


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
        self.reset_to_home()
        
        print("Panda Manual Controller initialized!")
    
    def reset_to_home(self):
        """Reset the robot to home position."""
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            self.data.qpos[joint_id] = self.home_qpos[i]
        
        mujoco.mj_forward(self.model, self.data)

    def update_control(self, dt):
        """Simple manual control - set each joint position directly."""
        self.time += dt
        
        # Manual joint controls - adjust these values to move the robot
        joint1_target = 0.0    # Range: -2.8973 to +2.8973 radians
        joint2_target = 0.0    # Range: -1.7628 to +1.7628 radians  
        joint3_target = 0.0    # Range: -2.8973 to +2.8973 radians
        joint4_target = 0.0   # Range: -3.0718 to -0.0698 radians
        joint5_target = 0.0    # Range: -2.8973 to +2.8973 radians
        joint6_target = 0.0    # Range: -0.0175 to +3.7525 radians
        joint7_target = 0.0    # Range: -2.8973 to +2.8973 radians
        gripper_target = 0   # Range: 0 (open) to 255 (closed)
        
        # Apply the controls
        

        if self.time > 3.0:
            # After 3 seconds, move joint2 down
            self.data.ctrl[self.actuator_indices['joint1']] = 2

            if self.time > 4.0:
                self.data.ctrl[self.actuator_indices['joint2']] = 1.7
                
                if self.time > 5.0:
                    self.data.ctrl[self.actuator_indices['joint1']] = -2.8
        else:
            self.data.ctrl[self.actuator_indices['joint1']] = joint1_target
            self.data.ctrl[self.actuator_indices['joint2']] = joint2_target
            self.data.ctrl[self.actuator_indices['joint3']] = joint3_target
            self.data.ctrl[self.actuator_indices['joint4']] = joint4_target
            self.data.ctrl[self.actuator_indices['joint5']] = joint5_target
            self.data.ctrl[self.actuator_indices['joint6']] = joint6_target
            self.data.ctrl[self.actuator_indices['joint7']] = joint7_target
            self.data.ctrl[self.gripper_actuator_id] = gripper_target
            



def create_custom_scene_builder(robot_path: str) -> MuJoCoXMLBuilder:
    """Create a custom scene using the XML builder system.
    
    Args:
        robot_path: Path to the robot XML file
        
    Returns:
        Configured MuJoCoXMLBuilder
    """
    # Create the builder with custom settings
    builder = MuJoCoXMLBuilder("panda_swing_scene")
    
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
                       width="128",
                       height="128")
    
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
                     size=(2, 2, 0.05), 
                     material="floor_material")
    
    # Add dynamic disk blocks that are affected by gravity
    
    # Create dynamic bodies with freejoint (allows 6DOF movement)
    
    # Disk block 1 (largest, red)
    builder.add_worldbody_element("body_xml", content='''
    <body name="disk_body1" pos="0.75 0 0.025">
        <freejoint/>
        <inertial pos="0 0 0" mass="0.1" diaginertia="0.005 0.005 0.005"/>
        <geom name="disk_block1" type="cylinder" size="0.06 0.035" rgba="0.8 0.2 0.2 1.0"/>
    </body>''')
    
    # Disk block 2 (medium, green)
    builder.add_worldbody_element("body_xml", content='''
    <body name="disk_body2" pos="0.75 0 0.075">
        <freejoint/>
        <inertial pos="0 0 0" mass="0.08" diaginertia="0.005 0.005 0.005"/>
        <geom name="disk_block2" type="cylinder" size="0.05 0.035" rgba="0.2 0.8 0.2 1.0"/>
    </body>''')
    
    # Disk block 3 (smallest, blue)
    builder.add_worldbody_element("body_xml", content='''
    <body name="disk_body3" pos="0.75 0 0.105">
        <freejoint/>
        <inertial pos="0 0 0" mass="0.06" diaginertia="0.005 0.005 0.005"/>
        <geom name="disk_block3" type="cylinder" size="0.04 0.035" rgba="0.2 0.2 0.8 1.0"/>
    </body>''')    
    
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