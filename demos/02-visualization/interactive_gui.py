#!/usr/bin/env python3
"""
Enhanced MuJoCo Interactive GUI Demo

Features:
- Real-time physics simulation with GUI
- Multiple object types and materials
- Interactive controls and reset functionality
- Performance monitoring
- Camera controls
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import random

def create_interactive_scene():
    """Create an interactive physics playground."""
    xml = """
    <mujoco model="interactive_playground">
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
        <texture name="grid" type="2d" builtin="checker" rgb1="0.1 0.2 0.3" 
                 rgb2="0.2 0.3 0.4" width="300" height="300"/>
        <material name="grid" texture="grid" texrepeat="5 5" reflectance="0.2"/>
        <material name="red" rgba="0.8 0.2 0.2 1" shininess="0.8" specular="0.5"/>
        <material name="green" rgba="0.2 0.8 0.2 1" shininess="0.8" specular="0.5"/>
        <material name="blue" rgba="0.2 0.2 0.8 1" shininess="0.8" specular="0.5"/>
        <material name="yellow" rgba="0.8 0.8 0.2 1" shininess="0.8" specular="0.5"/>
        <material name="purple" rgba="0.8 0.2 0.8 1" shininess="0.8" specular="0.5"/>
        <material name="orange" rgba="0.8 0.5 0.2 1" shininess="0.8" specular="0.5"/>
      </asset>
      
      <worldbody>
        <!-- Enhanced ground with texture -->
        <geom name="floor" pos="0 0 -0.1" size="4 4 0.05" type="box" material="grid"/>
        
        <!-- Walls to contain objects -->
        <geom name="wall1" pos="4 0 0.5" size="0.1 4 0.5" type="box" rgba="0.5 0.5 0.5 0.3"/>
        <geom name="wall2" pos="-4 0 0.5" size="0.1 4 0.5" type="box" rgba="0.5 0.5 0.5 0.3"/>
        <geom name="wall3" pos="0 4 0.5" size="4 0.1 0.5" type="box" rgba="0.5 0.5 0.5 0.3"/>
        <geom name="wall4" pos="0 -4 0.5" size="4 0.1 0.5" type="box" rgba="0.5 0.5 0.5 0.3"/>
        
        <!-- Various falling objects -->
        <body name="sphere_red" pos="-2 -2 4">
          <joint type="free"/>
          <geom size="0.15" type="sphere" material="red" mass="2" friction="0.7 0.1 0.1"/>
        </body>
        
        <body name="sphere_green" pos="0 -2 4.5">
          <joint type="free"/>
          <geom size="0.12" type="sphere" material="green" mass="1.5" friction="0.8 0.1 0.1"/>
        </body>
        
        <body name="sphere_blue" pos="2 -2 5">
          <joint type="free"/>
          <geom size="0.18" type="sphere" material="blue" mass="2.5" friction="0.6 0.1 0.1"/>
        </body>
        
        <body name="box_yellow" pos="-1.5 0 3">
          <joint type="free"/>
          <geom size="0.15 0.15 0.15" type="box" material="yellow" mass="2" friction="0.9 0.1 0.1"/>
        </body>
        
        <body name="box_purple" pos="0 0 3.5">
          <joint type="free"/>
          <geom size="0.1 0.2 0.25" type="box" material="purple" mass="1.8" friction="0.7 0.1 0.1"/>
        </body>
        
        <body name="box_orange" pos="1.5 0 4">
          <joint type="free"/>
          <geom size="0.2 0.1 0.15" type="box" material="orange" mass="2.2" friction="0.8 0.1 0.1"/>
        </body>
        
        <body name="capsule1" pos="-2 2 3.5">
          <joint type="free"/>
          <geom size="0.08" fromto="0 0 -0.2 0 0 0.2" type="capsule" material="red" mass="1.2"/>
        </body>
        
        <body name="capsule2" pos="0 2 4">
          <joint type="free"/>
          <geom size="0.06" fromto="0 0 -0.25 0 0 0.25" type="capsule" material="green" mass="1"/>
        </body>
        
        <body name="capsule3" pos="2 2 4.5">
          <joint type="free"/>
          <geom size="0.1" fromto="0 0 -0.15 0 0 0.15" type="capsule" material="blue" mass="1.5"/>
        </body>
        
        <!-- Static obstacles for interesting interactions -->
        <body name="platform1" pos="-1 1 0.8">
          <geom size="0.5 0.5 0.05" type="box" rgba="0.6 0.6 0.6 1"/>
        </body>
        
        <body name="platform2" pos="1 -1 1.2">
          <geom size="0.4 0.6 0.05" type="box" rgba="0.6 0.6 0.6 1"/>
        </body>
        
        <body name="ramp" pos="0 1.5 0.3" euler="0 0.3 0">
          <geom size="0.8 0.3 0.02" type="box" rgba="0.7 0.7 0.7 1"/>
        </body>
        
        <!-- Cylinder obstacle -->
        <body name="cylinder" pos="1.5 1.5 0.25">
          <geom size="0.15 0.25" type="cylinder" rgba="0.5 0.7 0.5 1"/>
        </body>
        
        <!-- Lighting -->
        <light directional="true" pos="0 0 6" dir="0 0 -1" 
               ambient="0.2 0.2 0.2" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3"/>
      </worldbody>
      
      <keyframe>
        <key name="initial" qpos="
          -2 -2 4 1 0 0 0
          0 -2 4.5 1 0 0 0  
          2 -2 5 1 0 0 0
          -1.5 0 3 1 0 0 0
          0 0 3.5 1 0 0 0
          1.5 0 4 1 0 0 0
          -2 2 3.5 1 0 0 0
          0 2 4 1 0 0 0
          2 2 4.5 1 0 0 0
        "/>
      </keyframe>
    </mujoco>
    """
    return xml

def print_instructions():
    """Print user instructions."""
    print("\n" + "="*60)
    print("🎮 MUJOCO INTERACTIVE PHYSICS PLAYGROUND")
    print("="*60)
    print("MOUSE CONTROLS:")
    print("  • Left Click + Drag  : Rotate camera")
    print("  • Right Click + Drag : Pan camera") 
    print("  • Scroll Wheel       : Zoom in/out")
    print("  • Double Click       : Center view")
    print("\nKEYBOARD CONTROLS:")
    print("  • SPACE              : Pause/Resume simulation")
    print("  • R                  : Reset to initial state")
    print("  • TAB                : Toggle between cameras")
    print("  • ESC                : Exit simulation")
    print("  • F1                 : Toggle help overlay")
    print("\nFEATURES:")
    print("  • Real-time physics simulation")
    print("  • Multiple object types with different materials")
    print("  • Interactive obstacles and platforms")
    print("  • Performance monitoring")
    print("="*60)

def main():
    print_instructions()
    
    try:
        # Create model
        print("\n📋 Loading interactive physics scene...")
        xml = create_interactive_scene()
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        
        print(f"✅ Model loaded successfully!")
        print(f"   📊 Bodies: {model.nbody}")
        print(f"   🔗 Joints: {model.njnt}")
        print(f"   🎯 DOFs: {model.nv}")
        print(f"   ⏱️  Timestep: {model.opt.timestep:.4f}s")
        
        print(f"\n🖥️  Opening GUI viewer...")
        input("Press ENTER to start the simulation...")
        
        # Launch viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set initial camera position
            viewer.cam.azimuth = 45
            viewer.cam.elevation = -15
            viewer.cam.distance = 8
            viewer.cam.lookat[:] = [0, 0, 1]
            
            # Performance tracking
            start_time = time.time()
            step_count = 0
            last_fps_time = start_time
            fps_counter = 0
            
            print("🚀 Simulation started! Close viewer window to exit.")
            
            # Main simulation loop
            while viewer.is_running():
                step_start = time.perf_counter()
                
                # Physics step
                mujoco.mj_step(model, data)
                step_count += 1
                fps_counter += 1
                
                # Performance monitoring
                current_time = time.time()
                if current_time - last_fps_time >= 2.0:  # Every 2 seconds
                    fps = fps_counter / (current_time - last_fps_time)
                    elapsed = current_time - start_time
                    print(f"📈 Step {step_count:,} | FPS: {fps:.1f} | "
                          f"Elapsed: {elapsed:.1f}s | Sim time: {data.time:.2f}s")
                    
                    last_fps_time = current_time
                    fps_counter = 0
                
                # Real-time sync (comment out for maximum speed)
                time_until_next_step = model.opt.timestep - (time.perf_counter() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                
                # Update viewer
                viewer.sync()
        
        total_time = time.time() - start_time
        avg_fps = step_count / total_time if total_time > 0 else 0
        
        print(f"\n📊 SIMULATION SUMMARY:")
        print(f"   Total steps: {step_count:,}")
        print(f"   Total time: {total_time:.1f}s")
        print(f"   Average FPS: {avg_fps:.1f}")
        print(f"   Sim time reached: {data.time:.2f}s")
        print("👋 Thanks for using MuJoCo Interactive Demo!")
        
    except Exception as e:
        print(f"\n❌ Error occurred: {str(e)}")
        print("💡 Make sure MuJoCo viewer dependencies are installed:")
        print("   pip install mujoco glfw PyOpenGL")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())