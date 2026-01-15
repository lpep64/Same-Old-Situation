#!/usr/bin/env python3
"""
Franka FR3 Robot Arm Demo with Basic Joint Controller

This demo combines:
1. Lighting and surface from examples/gui/simple_gui
2. Franka FR3 robot arm from models/robots
3. Basic controller that moves joints 2 and 3
"""

import mujoco
import mujoco.viewer
import time
import numpy as np

# Combined scene XML with simplified world, Franka FR3 robot, and controller
XML = """
<mujoco model="fr3_demo">
  <compiler angle="radian" meshdir="models/robots/franka_fr3/assets"/>
  
  <option timestep="0.01" integrator="implicitfast"/>
  
  <visual>
    <global offwidth="1280" offheight="720" azimuth="120" elevation="-20"/>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
  </visual>

  <statistic center="0.2 0 0.4" extent=".8"/>

  <default>
    <default class="fr3">
      <joint armature="0.1" damping="1"/>
      <position inheritrange="1"/>
      <default class="visual">
        <geom type="mesh" group="2" contype="0" conaffinity="0"/>
      </default>
      <default class="collision">
        <geom type="mesh" group="3" mass="0" density="0"/>
      </default>
      <site size="0.001" rgba="0.5 0.5 0.5 0.3" group="4"/>
    </default>
  </default>

  <asset>
    <!-- Skybox and ground textures -->
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    
    <!-- Franka FR3 materials -->
    <material name="black" rgba=".2 .2 .2 1"/>
    <material name="white" rgba="1 1 1 1"/>
    <material name="red" rgba="1 0.072272 0.039546 1"/>
    <material name="gray" rgba="0.863156 0.863156 0.863157 1"/>
    <material name="button_green" rgba="0.102241 0.571125 0.102242 1"/>
    <material name="button_red" rgba="0.520996 0.008023 0.013702 1"/>
    <material name="button_blue" rgba="0.024157 0.445201 0.737911 1"/>

    <!-- Franka FR3 meshes -->
    <mesh file="link0_0.obj"/>
    <mesh file="link0_1.obj"/>
    <mesh file="link0_2.obj"/>
    <mesh file="link0_3.obj"/>
    <mesh file="link0_4.obj"/>
    <mesh file="link0_5.obj"/>
    <mesh file="link0_6.obj"/>
    <mesh file="link1.obj"/>
    <mesh file="link2.obj"/>
    <mesh file="link3_0.obj"/>
    <mesh file="link3_1.obj"/>
    <mesh file="link4_0.obj"/>
    <mesh file="link4_1.obj"/>
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
    <mesh file="link7_0.obj"/>
    <mesh file="link7_1.obj"/>
    <mesh file="link7_2.obj"/>
    <mesh file="link7_3.obj"/>

    <mesh name="link0_coll" file="link0.stl"/>
    <mesh name="link1_coll" file="link1.stl"/>
    <mesh name="link2_coll" file="link2.stl"/>
    <mesh name="link3_coll" file="link3.stl"/>
    <mesh name="link4_coll" file="link4.stl"/>
    <mesh name="link5_coll" file="link5.stl"/>
    <mesh name="link6_coll" file="link6.stl"/>
    <mesh name="link7_coll" file="link7.stl"/>
  </asset>

  <worldbody>
    <!-- Lighting (from simple_gui style) -->
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    
    <!-- Ground plane (from simple_gui style but with FR3 scene material) -->
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    
    <!-- Franka FR3 Robot Arm -->
    <body name="base" childclass="fr3">
      <body name="fr3_link0">
        <geom mesh="link0_0" material="black" class="visual"/>
        <geom mesh="link0_1" material="white" class="visual"/>
        <geom mesh="link0_2" material="white" class="visual"/>
        <geom mesh="link0_3" material="white" class="visual"/>
        <geom mesh="link0_4" material="white" class="visual"/>
        <geom mesh="link0_5" material="red" class="visual"/>
        <geom mesh="link0_6" material="black" class="visual"/>
        <geom name="fr3_link0_collision" mesh="link0_coll" class="collision"/>
        
        <body name="fr3_link1" pos="0 0 0.333">
          <inertial pos="4.128e-07 -0.0181251 -0.0386036" quat="0.998098 -0.0605364 0.00380499 0.0110109" mass="2.92747"
            diaginertia="0.0239286 0.0227246 0.00610634"/>
          <joint name="fr3_joint1" axis="0 0 1" range="-2.7437 2.7437" actuatorfrcrange="-87 87"/>
          <geom name="fr3_link1_collision" class="collision" mesh="link1_coll"/>
          <geom material="white" mesh="link1" class="visual"/>
          
          <body name="fr3_link2" quat="1 -1 0 0">
            <inertial pos="0.00318289 -0.0743222 0.00881461" quat="0.502599 0.584437 -0.465998 0.434366" mass="2.93554"
              diaginertia="0.0629567 0.0411924 0.0246371"/>
            <joint name="fr3_joint2" axis="0 0 1" range="-1.7837 1.7837" actuatorfrcrange="-87 87"/>
            <geom material="white" mesh="link2" class="visual"/>
            <geom name="fr3_link2_collision" class="collision" mesh="link2_coll"/>
            
            <body name="fr3_link3" pos="0 -0.316 0" quat="1 1 0 0">
              <inertial pos="0.0407016 -0.00482006 -0.0289731" quat="0.921025 -0.244161 0.155272 0.260745" mass="2.2449"
                diaginertia="0.0267409 0.0189869 0.0171587"/>
              <joint name="fr3_joint3" axis="0 0 1" range="-2.9007 2.9007" actuatorfrcrange="-87 87"/>
              <geom mesh="link3_0" material="white" class="visual"/>
              <geom mesh="link3_1" material="black" class="visual"/>
              <geom name="fr3_link3_collision" class="collision" mesh="link3_coll"/>
              
              <body name="fr3_link4" pos="0.0825 0 0" quat="1 1 0 0">
                <inertial pos="-0.0459101 0.0630493 -0.00851879" quat="0.438018 0.803311 0.00937812 0.403414"
                  mass="2.6156" diaginertia="0.05139 0.0372717 0.0160047"/>
                <joint name="fr3_joint4" axis="0 0 1" range="-3.0421 -0.1518" actuatorfrcrange="-87 87"/>
                <geom mesh="link4_0" material="white" class="visual"/>
                <geom mesh="link4_1" material="black" class="visual"/>
                <geom name="fr3_link4_collision" class="collision" mesh="link4_coll"/>
                
                <body name="fr3_link5" pos="-0.0825 0.384 0" quat="1 -1 0 0">
                  <inertial pos="-0.00160396 0.0292536 -0.0972966" quat="0.919031 0.125604 0.0751531 -0.366003"
                    mass="2.32712" diaginertia="0.0579335 0.0449144 0.0130634"/>
                  <joint name="fr3_joint5" axis="0 0 1" range="-2.8065 2.8065" actuatorfrcrange="-12 12"/>
                  <geom mesh="link5_0" material="white" class="visual"/>
                  <geom mesh="link5_1" material="white" class="visual"/>
                  <geom mesh="link5_2" material="black" class="visual"/>
                  <geom name="fr3_link5_collision" class="collision" mesh="link5_coll"/>
                  
                  <body name="fr3_link6" quat="1 1 0 0">
                    <inertial pos="0.0597131 -0.0410295 -0.0101693" quat="0.621301 0.552665 0.510011 0.220081"
                      mass="1.81704" diaginertia="0.0175039 0.0161123 0.00193529"/>
                    <joint name="fr3_joint6" axis="0 0 1" range="0.5445 4.5169" actuatorfrcrange="-12 12"/>
                    <geom mesh="link6_0" material="button_green" class="visual"/>
                    <geom mesh="link6_1" material="white" class="visual"/>
                    <geom mesh="link6_2" material="white" class="visual"/>
                    <geom mesh="link6_3" material="gray" class="visual"/>
                    <geom mesh="link6_4" material="button_red" class="visual"/>
                    <geom mesh="link6_5" material="white" class="visual"/>
                    <geom mesh="link6_6" material="black" class="visual"/>
                    <geom mesh="link6_7" material="button_blue" class="visual"/>
                    <geom name="fr3_link6_collision" class="collision" mesh="link6_coll"/>
                    
                    <body name="fr3_link7" pos="0.088 0 0" quat="1 1 0 0">
                      <inertial pos="0.00452258 0.00862619 -0.0161633" quat="0.727579 0.0978688 -0.24906 0.63168"
                        mass="0.627143" diaginertia="0.000223836 0.000223642 5.64132e-07"/>
                      <joint name="fr3_joint7" axis="0 0 1" range="-3.0159 3.0159" actuatorfrcrange="-12 12"/>
                      <geom mesh="link7_0" material="black" class="visual"/>
                      <geom mesh="link7_1" material="white" class="visual"/>
                      <geom mesh="link7_2" material="white" class="visual"/>
                      <geom mesh="link7_3" material="black" class="visual"/>
                      <geom name="fr3_link7_collision" class="collision" mesh="link7_coll"/>
                      <site name="attachment_site" pos="0 0 0.107"/>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- Position actuators for joint control -->
    <position class="fr3" name="fr3_joint1" joint="fr3_joint1" kp="4500" kv="450"/>
    <position class="fr3" name="fr3_joint2" joint="fr3_joint2" kp="4500" kv="450"/>
    <position class="fr3" name="fr3_joint3" joint="fr3_joint3" kp="3500" kv="350"/>
    <position class="fr3" name="fr3_joint4" joint="fr3_joint4" kp="3500" kv="350"/>
    <position class="fr3" name="fr3_joint5" joint="fr3_joint5" kp="2000" kv="200"/>
    <position class="fr3" name="fr3_joint6" joint="fr3_joint6" kp="2000" kv="200"/>
    <position class="fr3" name="fr3_joint7" joint="fr3_joint7" kp="2000" kv="200"/>
  </actuator>

  <keyframe>
    <key name="home" qpos="0 0 0 -1.57079 0 1.57079 -0.7853" ctrl="0 0 0 -1.57079 0 1.57079 -0.7853"/>
  </keyframe>
</mujoco>
"""

def main():
    print("Starting Franka FR3 Robot Arm Demo...")
    print("Features:")
    print("  1. Simple world lighting and ground from examples/gui/simple_gui")
    print("  2. Franka FR3 robot arm from models/robots")
    print("  3. Basic controller moving joints 2 and 3")
    print()
    
    # Load the model
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    
    print("Model loaded successfully!")
    print(f"  Bodies: {model.nbody}")
    print(f"  Joints: {model.njnt}")
    print(f"  Actuators: {model.nu}")
    print()
    
    # Initialize joint positions (home pose from keyframe)
    data.qpos[:] = [0, 0, 0, -1.57079, 0, 1.57079, -0.7853]
    
    # Open the viewer
    print("Opening GUI viewer...")
    print("Close the viewer window to exit.")
    print()
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        
        print("Starting robot arm movement demo...")
        print("Joint 2 and 3 will move in oscillating patterns")
        print()
        
        step_count = 0
        while viewer.is_running():
            current_time = time.time() - start_time
            
            # Basic controller for joints 2 and 3
            # Joint 2: Slow oscillation
            joint2_target = 0.5 * np.sin(0.5 * current_time)
            
            # Joint 3: Faster oscillation
            joint3_target = 0.8 * np.sin(1.2 * current_time)
            
            # Set control targets (using home pose as base)
            data.ctrl[0] = 0.0                    # joint 1
            data.ctrl[1] = joint2_target          # joint 2 (oscillating)
            data.ctrl[2] = joint3_target          # joint 3 (oscillating)
            data.ctrl[3] = -1.57079               # joint 4 (bent elbow, from home pose)
            data.ctrl[4] = 0.0                    # joint 5
            data.ctrl[5] = 1.57079                # joint 6 (from home pose)
            data.ctrl[6] = -0.7853                # joint 7 (from home pose)
            
            # Step the simulation
            mujoco.mj_step(model, data)
            
            # Update viewer
            viewer.sync()
            
            # Print status every 2 seconds
            if step_count % 200 == 0:
                print(f"Time: {current_time:.1f}s | Joint2: {joint2_target:.3f} | Joint3: {joint3_target:.3f}")
            
            step_count += 1
            time.sleep(0.01)  # 100 Hz control loop
    
    print("Demo finished!")

if __name__ == "__main__":
    main()