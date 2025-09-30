#!/usr/bin/env python3
"""
GUI Test Script for Niryo Arm Models
Opens both the geometric and mesh-based models in MuJoCo's interactive viewer.
"""

import mujoco
import mujoco.viewer
import numpy as np
import os
import sys
import time
import threading

def create_demo_controller(model, data):
    """Create a simple demo controller that moves the arm through various poses."""
    def demo_sequence():
        """Demonstration sequence showing arm movement."""
        # Home position
        target_positions = [
            [0, 0, 0, 0, 0, 0],                    # Home
            [0.5, -0.3, 0.8, 0, 0.5, 0],         # Reach pose
            [-0.5, -0.3, 0.8, 0, 0.5, 0],        # Left reach
            [0, -0.8, 1.2, 0, 0, 0],             # High pose
            [0, 0.3, -0.5, 1.0, -0.5, 1.0],      # Complex pose
            [0, 0, 0, 0, 0, 0],                    # Return home
        ]
        
        current_target = 0
        steps_per_pose = 500
        step_count = 0
        
        while True:
            if step_count >= steps_per_pose:
                current_target = (current_target + 1) % len(target_positions)
                step_count = 0
            
            # Get target position
            target = target_positions[current_target]
            
            # Simple proportional control
            for i in range(min(len(target), model.nu)):
                if i < len(data.qpos):
                    error = target[i] - data.qpos[i]
                    data.ctrl[i] = 5.0 * error  # P-gain
            
            step_count += 1
            time.sleep(0.01)  # 100 Hz control rate
    
    return demo_sequence

def run_viewer(model_path, window_title):
    """Run MuJoCo viewer for a specific model."""
    try:
        print(f"🚀 Loading {window_title}...")
        
        # Load model
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        
        print(f"✅ Model loaded: {model.nbody} bodies, {model.nmesh} meshes")
        
        # Create and start demo controller
        demo_controller = create_demo_controller(model, data)
        controller_thread = threading.Thread(target=demo_controller, daemon=True)
        controller_thread.start()
        
        # Launch viewer
        print(f"🎮 Starting viewer: {window_title}")
        print("   - Press 'Space' to pause/unpause")
        print("   - Press 'R' to reset")
        print("   - Mouse: Rotate view, Scroll: Zoom")
        print("   - Close window to exit")
        
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set initial camera position for better view
            viewer.cam.distance = 2.0
            viewer.cam.elevation = -20
            viewer.cam.azimuth = 45
            
            # Run simulation
            while viewer.is_running():
                # Step physics
                mujoco.mj_step(model, data)
                
                # Update viewer
                viewer.sync()
                
                # Sleep to maintain real-time
                time.sleep(0.002)
        
        print(f"🔚 Viewer closed: {window_title}")
        
    except Exception as e:
        print(f"❌ Error in {window_title}: {str(e)}")
        return False
    
    return True

def main():
    """Main function to run GUI tests."""
    print("🎮 Niryo Arm GUI Test")
    print("=" * 50)
    
    # Get project root
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    # Model configurations
    models = [
        {
            "name": "Basic Geometric Model",
            "path": os.path.join(project_root, "models", "niryo_arm.xml"),
            "description": "Fast geometric shapes (cylinders, boxes)"
        },
        {
            "name": "Detailed STL Mesh Model", 
            "path": os.path.join(project_root, "models", "niryo_arm_with_meshes.xml"),
            "description": "Realistic Niryo STL meshes"
        }
    ]
    
    print("Available models:")
    for i, model in enumerate(models, 1):
        print(f"  {i}. {model['name']}")
        print(f"     {model['description']}")
        print(f"     Path: {model['path']}")
        print()
    
    # Get user choice
    while True:
        try:
            choice = input("Select model to view (1-2, or 'both' for sequential viewing): ").strip().lower()
            
            if choice == 'both':
                # Run both models sequentially
                for model in models:
                    if not os.path.exists(model['path']):
                        print(f"❌ Model file not found: {model['path']}")
                        continue
                    
                    print(f"\n🎯 Running {model['name']}")
                    print("-" * 40)
                    success = run_viewer(model['path'], model['name'])
                    
                    if not success:
                        print(f"Failed to run {model['name']}")
                    
                    if model != models[-1]:  # Not the last model
                        input("\nPress Enter to continue to next model...")
                break
                
            elif choice in ['1', '2']:
                model_idx = int(choice) - 1
                model = models[model_idx]
                
                if not os.path.exists(model['path']):
                    print(f"❌ Model file not found: {model['path']}")
                    continue
                
                print(f"\n🎯 Running {model['name']}")
                print("-" * 40)
                run_viewer(model['path'], model['name'])
                break
                
            else:
                print("❌ Invalid choice. Please enter 1, 2, or 'both'")
                
        except KeyboardInterrupt:
            print("\n👋 Exiting...")
            return 0
        except Exception as e:
            print(f"❌ Error: {str(e)}")
            return 1
    
    print("\n✅ GUI test completed!")
    return 0

if __name__ == "__main__":
    sys.exit(main())