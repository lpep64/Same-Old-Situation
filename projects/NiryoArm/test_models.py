#!/usr/bin/env python3
"""
Simple MuJoCo model viewer script for testing robot arm models.
"""

import mujoco
import mujoco.viewer
import sys
import os

def load_and_view_model(model_path):
    """Load a MuJoCo model and open it in the viewer."""
    
    print(f"Loading model: {model_path}")
    
    # Check if file exists
    if not os.path.exists(model_path):
        print(f"Error: Model file not found: {model_path}")
        return False
    
    try:
        # Load the model
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        
        print(f"Model loaded successfully!")
        print(f"Number of DOFs: {model.nv}")
        print(f"Number of bodies: {model.nbody}")
        print(f"Number of joints: {model.njnt}")
        
        # Open the viewer
        print("Opening MuJoCo viewer... (Press ESC to exit)")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Run simulation
            while viewer.is_running():
                step_start = data.time
                
                # Step the simulation
                mujoco.mj_step(model, data)
                
                # Sync with real time
                viewer.sync()
        
        print("Viewer closed.")
        return True
        
    except Exception as e:
        print(f"Error loading or viewing model: {e}")
        return False

def main():
    """Main function to test different robot arm models."""
    
    # Base path to the models
    base_path = r"c:\Users\lukep\Documents\mujoco\projects\NiryoArm\models"
    
    # Available models to test
    models = {
        "2": ("Niryo with meshes", os.path.join(base_path, "niryo_arm_with_meshes.xml")),
        "3": ("Franka Panda", os.path.join(base_path, "franka_panda", "panda.xml")),
        "4": ("Franka Panda Scene", os.path.join(base_path, "franka_panda", "scene.xml")),
        "5": ("UFactory Lite6", os.path.join(base_path, "ufactory_lite6", "lite6.xml")),
        "6": ("UFactory Lite6 Scene", os.path.join(base_path, "ufactory_lite6", "scene.xml")),
    }
    
    print("Available robot arm models:")
    for key, (name, path) in models.items():
        print(f"  {key}: {name}")
    
    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        choice = input("\nEnter your choice (1-6): ").strip()
    
    if choice in models:
        name, path = models[choice]
        print(f"\nTesting {name}...")
        success = load_and_view_model(path)
        if success:
            print(f"✓ {name} loaded and displayed successfully!")
        else:
            print(f"✗ Failed to load {name}")
    else:
        print("Invalid choice. Please enter a number from 1-6.")

if __name__ == "__main__":
    main()