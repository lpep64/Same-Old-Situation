#!/usr/bin/env python3
"""
Test script to verify the Niryo mesh models load correctly in MuJoCo.
This script tests both the basic geometric model and the detailed STL mesh model.
"""

import mujoco
import numpy as np
import time
import os
import sys

def test_model_loading(model_path):
    """Test if a MuJoCo model loads successfully."""
    try:
        print(f"📂 Testing model: {model_path}")
        
        # Check if file exists
        if not os.path.exists(model_path):
            print(f"❌ Model file not found: {model_path}")
            return False
        
        # Load the model
        print("⏳ Loading model...")
        start_time = time.time()
        model = mujoco.MjModel.from_xml_path(model_path)
        load_time = time.time() - start_time
        
        print(f"✅ Model loaded successfully in {load_time:.3f}s")
        
        # Create data structure
        data = mujoco.MjData(model)
        
        # Print model information
        print(f"📊 Model Statistics:")
        print(f"   - Bodies: {model.nbody}")
        print(f"   - Joints: {model.njnt}")
        print(f"   - Actuators: {model.nu}")
        print(f"   - Meshes: {model.nmesh}")
        print(f"   - Geoms: {model.ngeom}")
        
        # Test a few simulation steps
        print("🔄 Testing simulation...")
        for i in range(10):
            mujoco.mj_step(model, data)
        
        print("✅ Simulation test passed")
        
        # Test joint limits
        print("🔧 Joint Information:")
        joint_names = []
        for i in range(model.njnt):
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name:
                joint_names.append(joint_name)
                joint_range = model.jnt_range[i]
                print(f"   - {joint_name}: [{joint_range[0]:.2f}, {joint_range[1]:.2f}]")
        
        return True
        
    except Exception as e:
        print(f"❌ Error loading model: {str(e)}")
        return False

def main():
    """Main test function."""
    print("🚀 Niryo Arm MuJoCo Model Test")
    print("=" * 50)
    
    # Get the project root directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    # Define model paths
    models = {
        "Basic Geometric Model": os.path.join(project_root, "models", "niryo_arm.xml"),
        "Detailed STL Mesh Model": os.path.join(project_root, "models", "niryo_arm_with_meshes.xml")
    }
    
    results = {}
    
    # Test each model
    for model_name, model_path in models.items():
        print(f"\n🧪 Testing {model_name}")
        print("-" * 40)
        results[model_name] = test_model_loading(model_path)
    
    # Summary
    print("\n📋 Test Summary")
    print("=" * 50)
    
    all_passed = True
    for model_name, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{model_name}: {status}")
        if not passed:
            all_passed = False
    
    if all_passed:
        print("\n🎉 All tests passed! The Niryo models are ready to use.")
        print("\n💡 Next steps:")
        print("   - Use niryo_arm.xml for fast training and prototyping")
        print("   - Use niryo_arm_with_meshes.xml for realistic visualization")
        print("   - Check assets/meshes/niryo/README.md for mesh documentation")
    else:
        print("\n⚠️  Some tests failed. Please check the model files and dependencies.")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())