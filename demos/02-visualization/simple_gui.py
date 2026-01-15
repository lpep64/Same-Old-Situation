#!/usr/bin/env python3
"""
Simple MuJoCo GUI Viewer

A minimal example showing how to open MuJoCo with a GUI viewer.
This is guaranteed to work with basic MuJoCo installation.
"""

import mujoco
import mujoco.viewer
import time
import numpy as np

# Simple scene XML
XML = """
<mujoco model="simple_scene">
  <option timestep="0.01"/>
  
  <visual>
    <global offwidth="1280" offheight="720"/>
  </visual>
  
  <worldbody>
    <!-- Ground -->
    <geom name="floor" pos="0 0 -0.5" size="2 2 0.1" type="box" rgba="0.8 0.8 0.8 1"/>
    
    <!-- Falling ball -->
    <body name="ball" pos="0 0 2">
      <joint type="free"/>
      <geom size="0.15" rgba="1 0.2 0.2 1" mass="1"/>
    </body>
    
    <!-- Another ball -->
    <body name="ball2" pos="0.5 0.5 3">
      <joint type="free"/>
      <geom size="0.12" rgba="0.2 1 0.2 1" mass="0.8"/>
    </body>
    
    <!-- Box -->
    <body name="box" pos="-0.5 -0.5 2.5">
      <joint type="free"/>
      <geom size="0.1 0.1 0.1" type="box" rgba="0.2 0.2 1 1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""

def main():
    print("Starting Simple MuJoCo GUI Demo...")
    
    # Load model
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    
    print("Model loaded. Opening viewer...")
    print("Close the viewer window to exit.")
    
    # Launch viewer - this will open a GUI window
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set camera position
        viewer.cam.distance = 3
        viewer.cam.elevation = -20
        
        step = 0
        # Run simulation loop
        while viewer.is_running():
            # Step physics
            mujoco.mj_step(model, data)
            
            # Print status occasionally
            if step % 500 == 0:
                print(f"Step {step}, Time: {data.time:.2f}s")
            
            step += 1
            
            # Slow down to real-time
            time.sleep(0.01)
            
            # Update display
            viewer.sync()
    
    print("Demo finished!")

if __name__ == "__main__":
    main()