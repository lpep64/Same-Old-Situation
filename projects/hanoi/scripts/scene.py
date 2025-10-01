#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import time
import random

def create_interactive_scene():
    """Create a simple ground."""
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

def main():
    
    try:
        # Create model
        xml = create_interactive_scene()
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        
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
                        
            # Main simulation loop
            while viewer.is_running():
                step_start = time.perf_counter()
                
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