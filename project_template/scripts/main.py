#!/usr/bin/env python3
"""
Template main script for MuJoCo projects.

Modify this script for your specific project needs.
"""

import mujoco
import mujoco.viewer
import numpy as np
import sys
import os

# Add parent directory to path to import controllers
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from controllers.base_controller import PositionController


def main():
    """Main execution function."""
    print("Starting MuJoCo Project Template...")
    
    # Load your model (update path as needed)
    model_path = "../../models/basic/humanoid/humanoid.xml"
    
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Error loading model: {e}")
        print("Update model_path in this script to point to your desired model.")
        return
    
    print(f"Model loaded successfully!")
    print(f"Number of joints: {model.njnt}")
    print(f"Number of actuators: {model.nu}")
    
    # Initialize controller
    controller = PositionController(model, data, kp=100.0)
    
    # Set initial target (modify as needed)
    target_positions = np.zeros(model.nu)
    
    # Set initial state
    data.ctrl[:] = target_positions
    mujoco.mj_forward(model, data)
    
    print("Starting MuJoCo viewer...")
    print("Close the window to exit.")
    
    with mujoco.viewer.launch(model, data) as viewer:
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.distance = 3.0
        
        while viewer.is_running():
            # Update control (implement your control logic here)
            data.ctrl[:] = controller.compute_control(target_positions)
            
            # Step simulation
            mujoco.mj_step(model, data)
            viewer.sync()
    
    print("Simulation completed.")


if __name__ == "__main__":
    main()