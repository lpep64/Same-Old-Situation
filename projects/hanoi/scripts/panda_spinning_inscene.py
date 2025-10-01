#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import time
import random
import os

def create_interactive_scene():
    """Create a simple ground scene."""
    xml = """
    <mujoco model="ground">
      <compiler angle="radian"/>
      
      <option timestep="0.005" integrator="RK4">
        <flag warmstart="enable"/>
      </option>
      
      <visual>
        <headlight ambient="0.3 0.3 0.3" diffuse="0.7 0.7 0.7"/>
        <global offwidth="1920" offheight="1080"/>
        <quality shadowsize="2048" offsamples="8"/>
      </visual>
      
      <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1="0.3 0.3 0.3" 
                 rgb2="0.1 0.1 0.1" width="300" height="300"/>
        <material name="grid" texture="grid" texrepeat="5 5" reflectance="0."/>
        
        <!-- Sky texture for skybox -->
        <texture name="skybox" type="skybox" builtin="gradient" 
                 rgb1="0.3 0.5 0.7" rgb2="0.0 0.1 0.2" width="800" height="800" 
                 mark="random" markrgb="1 1 1"/>
      </asset>
      
      <worldbody>
        <!-- Enhanced ground with texture -->
        <geom name="floor" pos="0 0 -0.1" size="4 4 0.05" type="box" material="grid"/>
        
        <!-- Lighting -->
        <light directional="true" pos="0 0 6" dir="0 0 -1" 
               ambient="0.2 0.2 0.2" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3"/>
      </worldbody>
    </mujoco>
    """
    return xml

def combine_models(scene_xml, robot_model_path):
    """Combine scene XML with robot model by modifying the robot's XML."""
    try:
        # Read the robot XML file
        with open(robot_model_path, 'r') as f:
            robot_xml = f.read()
        
        # Create the enhanced robot XML with scene elements
        enhanced_xml = f"""<mujoco model="panda_with_scene">
  <compiler angle="radian" meshdir="../../models/robots/franka_emika_panda/assets" autolimits="true"/>

  <option timestep="0.005" integrator="RK4">
    <flag warmstart="enable"/>
  </option>

  <visual>
    <headlight ambient="0.3 0.3 0.3" diffuse="0.7 0.7 0.7"/>
    <global offwidth="1920" offheight="1080"/>
    <quality shadowsize="2048" offsamples="8"/>
  </visual>

  <default>
    <default class="panda">
      <material specular="0.5" shininess="0.25"/>
      <joint armature="0.1" damping="1" axis="0 0 1" range="-2.8973 2.8973"/>
      <general dyntype="none" biastype="affine" ctrlrange="-2.8973 2.8973" forcerange="-87 87"/>
      <default class="finger">
        <joint axis="0 1 0" type="slide" range="0 0.04"/>
      </default>

      <default class="visual">
        <geom type="mesh" contype="0" conaffinity="0" group="2"/>
      </default>
      <default class="collision">
        <geom type="mesh" group="3"/>
        <default class="fingertip_pad_collision_1">
          <geom type="box" size="0.0085 0.004 0.0085" pos="0 0.0055 0.0445"/>
        </default>
        <default class="fingertip_pad_collision_2">
          <geom type="box" size="0.003 0.002 0.003" pos="0.0055 0.002 0.05"/>
        </default>
        <default class="fingertip_pad_collision_3">
          <geom type="box" size="0.003 0.002 0.003" pos="-0.0055 0.002 0.05"/>
        </default>
        <default class="fingertip_pad_collision_4">
          <geom type="box" size="0.003 0.002 0.0035" pos="0.0055 0.002 0.0395"/>
        </default>
        <default class="fingertip_pad_collision_5">
          <geom type="box" size="0.003 0.002 0.0035" pos="-0.0055 0.002 0.0395"/>
        </default>
      </default>
    </default>
  </default>

  <asset>
    <!-- Scene textures and materials -->
    <texture name="grid" type="2d" builtin="checker" rgb1="0.3 0.3 0.3" 
             rgb2="0.1 0.1 0.1" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="5 5" reflectance="0."/>
    
    <!-- Sky texture for skybox -->
    <texture name="skybox" type="skybox" builtin="gradient" 
             rgb1="0.3 0.5 0.7" rgb2="0.0 0.1 0.2" width="800" height="800" 
             mark="random" markrgb="1 1 1"/>
    
    <!-- Robot materials -->
    <material class="panda" name="white" rgba="1 1 1 1"/>
    <material class="panda" name="off_white" rgba="0.901961 0.921569 0.929412 1"/>
    <material class="panda" name="black" rgba="0.25 0.25 0.25 1"/>
    <material class="panda" name="green" rgba="0 1 0 1"/>
    <material class="panda" name="light_blue" rgba="0.039216 0.541176 0.780392 1"/>

    <!-- Robot meshes -->
    <mesh name="link0_c" file="link0.stl"/>
    <mesh name="link1_c" file="link1.stl"/>
    <mesh name="link2_c" file="link2.stl"/>
    <mesh name="link3_c" file="link3.stl"/>
    <mesh name="link4_c" file="link4.stl"/>
    <mesh name="link5_c0" file="link5_collision_0.obj"/>
    <mesh name="link5_c1" file="link5_collision_1.obj"/>
    <mesh name="link5_c2" file="link5_collision_2.obj"/>
    <mesh name="link6_c" file="link6.stl"/>
    <mesh name="link7_c" file="link7.stl"/>
    <mesh name="hand_c" file="hand.stl"/>

    <!-- Visual meshes -->
    <mesh file="link0_0.obj"/>
    <mesh file="link0_1.obj"/>
    <mesh file="link0_2.obj"/>
    <mesh file="link0_3.obj"/>
    <mesh file="link0_4.obj"/>
    <mesh file="link0_5.obj"/>
    <mesh file="link0_7.obj"/>
    <mesh file="link0_8.obj"/>
    <mesh file="link0_9.obj"/>
    <mesh file="link0_10.obj"/>
    <mesh file="link0_11.obj"/>
    <mesh file="link1.obj"/>
    <mesh file="link2.obj"/>
    <mesh file="link3_0.obj"/>
    <mesh file="link3_1.obj"/>
    <mesh file="link3_2.obj"/>
    <mesh file="link3_3.obj"/>
    <mesh file="link4_0.obj"/>
    <mesh file="link4_1.obj"/>
    <mesh file="link4_2.obj"/>
    <mesh file="link4_3.obj"/>
    <mesh file="link5_0.obj"/>
    <mesh file="link5_1.obj"/>
    <mesh file="link5_2.obj"/>
    <mesh file="link6_0.obj"/>
    <mesh file="link6_1.obj"/>
    <mesh file="link6_2.obj"/>
    <mesh file="link6_3.obj"/>
    <mesh file="link6_4.obj"/>
    <mesh file="link6_5.obj"/>
    <mesh file="link6_6.obj"/>
    <mesh file="link6_7.obj"/>
    <mesh file="link6_8.obj"/>
    <mesh file="link6_9.obj"/>
    <mesh file="link6_10.obj"/>
    <mesh file="link6_11.obj"/>
    <mesh file="link6_12.obj"/>
    <mesh file="link6_13.obj"/>
    <mesh file="link6_14.obj"/>
    <mesh file="link6_15.obj"/>
    <mesh file="link6_16.obj"/>
    <mesh file="link7_0.obj"/>
    <mesh file="link7_1.obj"/>
    <mesh file="link7_2.obj"/>
    <mesh file="link7_3.obj"/>
    <mesh file="link7_4.obj"/>
    <mesh file="link7_5.obj"/>
    <mesh file="link7_6.obj"/>
    <mesh file="link7_7.obj"/>
    <mesh file="hand_0.obj"/>
    <mesh file="hand_1.obj"/>
    <mesh file="hand_2.obj"/>
    <mesh file="hand_3.obj"/>
    <mesh file="hand_4.obj"/>
    <mesh file="finger_0.obj"/>
    <mesh file="finger_1.obj"/>
  </asset>

  <worldbody>
    <!-- Enhanced ground with texture -->
    <geom name="floor" pos="0 0 -0.1" size="4 4 0.05" type="box" material="grid"/>
    
    <!-- Enhanced lighting -->
    <light name="top" pos="0 0 2" mode="trackcom"/>
    <light directional="true" pos="2 2 6" dir="-0.3 -0.3 -1" 
           ambient="0.2 0.2 0.2" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3"/>
    
    <!-- Robot positioned on the floor -->"""
        
        # Extract just the robot body content from the original XML
        # Find the robot body definition starting from <body name="link0"
        import re
        
        # Find where the robot body starts
        body_start = robot_xml.find('<body name="link0"')
        if body_start == -1:
            raise ValueError("Could not find robot body in XML")
        
        # Find the corresponding closing tag - need to count nested bodies
        body_count = 0
        current_pos = body_start
        body_end = -1
        
        while current_pos < len(robot_xml):
            # Look for body tags
            next_open = robot_xml.find('<body', current_pos)
            next_close = robot_xml.find('</body>', current_pos)
            
            if next_open != -1 and (next_close == -1 or next_open < next_close):
                body_count += 1
                current_pos = next_open + 5
            elif next_close != -1:
                body_count -= 1
                current_pos = next_close + 7
                if body_count == 0:
                    body_end = current_pos
                    break
            else:
                break
        
        if body_end == -1:
            raise ValueError("Could not find end of robot body in XML")
        
        robot_body = robot_xml[body_start:body_end]
        
        # Extract tendon, equality, actuator, and keyframe sections
        sections_to_extract = ['tendon', 'equality', 'actuator', 'keyframe']
        additional_content = ""
        
        for section in sections_to_extract:
            pattern = f'<{section}>.*?</{section}>'
            match = re.search(pattern, robot_xml, re.DOTALL)
            if match:
                additional_content += "\n  " + match.group(0) + "\n"
        
        # Complete the XML
        enhanced_xml += f"""
    {robot_body}
  </worldbody>
{additional_content}
</mujoco>"""
        
        # Create model from the enhanced XML
        return mujoco.MjModel.from_xml_string(enhanced_xml)
        
    except Exception as e:
        print(f"Error combining models: {e}")
        print("Falling back to robot model only...")
        # Fallback to just the robot model
        return mujoco.MjModel.from_xml_path(robot_model_path)

class PandaSpinningController:
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
        self.spin_speed = 1.0  # radians per second
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


def main():
    """Main function to run the unified simulation."""
    try:
        # Create unified model with scene and robot
        scene_xml = create_interactive_scene()
        robot_model_path = "../../models/robots/franka_emika_panda/panda.xml"
        
        # For now, use the robot model (which already has a good environment)
        # In future iterations, we can work on proper model combination
        model = combine_models(scene_xml, robot_model_path)
        data = mujoco.MjData(model)
        
        # Initialize robot controller with the unified model
        controller = PandaSpinningController(model, data)
        
        # Launch viewer for the unified simulation
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set initial camera position
            viewer.cam.azimuth = 45
            viewer.cam.elevation = -15
            viewer.cam.distance = 2.5
            viewer.cam.lookat[:] = [0, 0, 0.5]
            
            print("MuJoCo Viewer launched!")
            print("Single unified simulation with robot spinning controls")
            print("The robot will start spinning joint1 and joint7, and opening/closing grippers")
            print("Close the viewer window to exit the simulation")
            
            # Performance tracking
            start_time = time.time()
            step_count = 0
            last_fps_time = start_time
            fps_counter = 0
                        
            # Main simulation loop
            while viewer.is_running():
                step_start = time.perf_counter()
                
                # Update robot control
                controller.update_control(model.opt.timestep)
                
                # Physics step
                mujoco.mj_step(model, data)
                step_count += 1
                fps_counter += 1
                
                # Real-time sync (comment out for maximum speed)
                time_until_next_step = model.opt.timestep - (time.perf_counter() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                
                # Update viewer
                viewer.sync()
        
    except Exception as e:
        print(f"\n❌ Error occurred: {str(e)}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())