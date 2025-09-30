#!/usr/bin/env python3
"""
MuJoCo Non-GUI Basic Tests

This module provides basic non-GUI testing functionality for MuJoCo.
Perfect for automated testing, batch processing, and headless environments.
"""

import mujoco
import numpy as np
import time
import os

def test_basic_physics():
    """Test basic physics simulation without GUI."""
    print("🔬 Running Basic Physics Test...")
    
    xml = """
    <mujoco model="basic_test">
      <option timestep="0.01"/>
      <worldbody>
        <body name="ball" pos="0 0 2">
          <joint type="free"/>
          <geom size="0.1" rgba="1 0 0 1" mass="1"/>
        </body>
        <geom name="floor" pos="0 0 0" size="2 2 0.05" type="box" rgba="0.8 0.8 0.8 1"/>
      </worldbody>
    </mujoco>
    """
    
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    
    initial_height = data.qpos[2]  # z position
    print(f"   Initial ball height: {initial_height:.3f}m")
    
    # Run simulation
    steps = 1000
    for i in range(steps):
        mujoco.mj_step(model, data)
        if i % 200 == 0:
            height = data.qpos[2]
            print(f"   Step {i:4d}: Ball height = {height:.3f}m")
    
    final_height = data.qpos[2]
    fall_distance = initial_height - final_height
    
    print(f"   Final height: {final_height:.3f}m")
    print(f"   Total fall: {fall_distance:.3f}m")
    
    # Verify physics worked
    if fall_distance > 0.5:
        print("   ✅ Basic physics test PASSED")
        return True
    else:
        print("   ❌ Basic physics test FAILED")
        return False

def test_multiple_objects():
    """Test simulation with multiple interacting objects."""
    print("\n🔬 Running Multiple Objects Test...")
    
    xml = """
    <mujoco model="multi_objects">
      <option timestep="0.005"/>
      <worldbody>
        <geom name="floor" pos="0 0 -0.1" size="3 3 0.05" type="box" rgba="0.8 0.8 0.8 1"/>
        
        <body name="sphere1" pos="-0.5 0 2">
          <joint type="free"/>
          <geom size="0.1" type="sphere" rgba="1 0 0 1" mass="1"/>
        </body>
        
        <body name="sphere2" pos="0.5 0 2.5">
          <joint type="free"/>
          <geom size="0.12" type="sphere" rgba="0 1 0 1" mass="1.2"/>
        </body>
        
        <body name="box1" pos="0 -0.5 1.5">
          <joint type="free"/>
          <geom size="0.1 0.1 0.1" type="box" rgba="0 0 1 1" mass="0.8"/>
        </body>
        
        <body name="capsule1" pos="0 0.5 3">
          <joint type="free"/>
          <geom size="0.05" fromto="0 0 -0.15 0 0 0.15" type="capsule" rgba="1 1 0 1" mass="0.6"/>
        </body>
      </worldbody>
    </mujoco>
    """
    
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    
    # Initialize the simulation data properly
    mujoco.mj_forward(model, data)
    
    print(f"   Model loaded: {model.nbody} bodies, {model.nv} DOFs")
    
    # Track center of mass
    initial_com = np.mean([data.xpos[body_id] for body_id in range(1, model.nbody)], axis=0)
    
    # Run simulation
    steps = 2000
    for step in range(steps):
        mujoco.mj_step(model, data)
        if step % 400 == 0:
            current_com = np.mean([data.xpos[body_id] for body_id in range(1, model.nbody)], axis=0)
            print(f"   Step {step:4d}: Center of mass height = {current_com[2]:.3f}m")
    
    final_com = np.mean([data.xpos[body_id] for body_id in range(1, model.nbody)], axis=0)
    com_fall = initial_com[2] - final_com[2]
    
    print(f"   COM fell {com_fall:.3f}m")
    
    if com_fall > 0.5:  # More realistic expectation for objects settling
        print("   ✅ Multiple objects test PASSED")
        return True
    else:
        print("   ❌ Multiple objects test FAILED")
        return False

def test_model_loading():
    """Test loading various model files."""
    print("\n🔬 Running Model Loading Test...")
    
    # Test built-in models
    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    model_paths = [
        os.path.join(workspace_root, "model", "humanoid", "humanoid.xml"),
        os.path.join(workspace_root, "model", "cube", "cube.xml"), 
        os.path.join(workspace_root, "model", "car", "car.xml")
    ]
    
    passed = 0
    total = len(model_paths)
    
    for model_path in model_paths:
        try:
            if os.path.exists(model_path):
                model = mujoco.MjModel.from_xml_path(model_path)
                data = mujoco.MjData(model)
                
                # Quick simulation test
                for _ in range(10):
                    mujoco.mj_step(model, data)
                
                model_name = os.path.basename(model_path)
                print(f"   ✅ {model_name}: {model.nbody} bodies, {model.nv} DOFs")
                passed += 1
            else:
                model_name = os.path.basename(model_path)
                print(f"   ⚠️  {model_name}: File not found")
        except Exception as e:
            model_name = os.path.basename(model_path)
            print(f"   ❌ {model_name}: Error - {str(e)[:40]}...")
    
    print(f"   Loaded {passed}/{total} models successfully")
    return passed > 0

def test_performance_benchmark():
    """Benchmark simulation performance."""
    print("\n🔬 Running Performance Benchmark...")
    
    xml = """
    <mujoco model="performance_test">
      <option timestep="0.002"/>
      <worldbody>
        <geom name="floor" pos="0 0 -0.1" size="4 4 0.05" type="box"/>
        
        <!-- Create multiple objects for stress test -->
        <body name="obj1" pos="-1 -1 1"><joint type="free"/><geom size="0.05"/></body>
        <body name="obj2" pos="0 -1 1"><joint type="free"/><geom size="0.05"/></body>
        <body name="obj3" pos="1 -1 1"><joint type="free"/><geom size="0.05"/></body>
        <body name="obj4" pos="-1 0 1"><joint type="free"/><geom size="0.05"/></body>
        <body name="obj5" pos="0 0 1"><joint type="free"/><geom size="0.05"/></body>
        <body name="obj6" pos="1 0 1"><joint type="free"/><geom size="0.05"/></body>
        <body name="obj7" pos="-1 1 1"><joint type="free"/><geom size="0.05"/></body>
        <body name="obj8" pos="0 1 1"><joint type="free"/><geom size="0.05"/></body>
        <body name="obj9" pos="1 1 1"><joint type="free"/><geom size="0.05"/></body>
      </worldbody>
    </mujoco>
    """
    
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    
    steps = 10000
    print(f"   Running {steps:,} simulation steps...")
    
    start_time = time.perf_counter()
    
    for i in range(steps):
        mujoco.mj_step(model, data)
    
    end_time = time.perf_counter()
    elapsed = end_time - start_time
    steps_per_sec = steps / elapsed
    
    print(f"   Elapsed time: {elapsed:.3f}s")
    print(f"   Performance: {steps_per_sec:,.0f} steps/sec")
    print(f"   Real-time factor: {(steps * model.opt.timestep / elapsed):.1f}x")
    
    if steps_per_sec > 1000:
        print("   ✅ Performance test PASSED (>1000 steps/sec)")
        return True
    else:
        print("   ⚠️  Performance test SLOW (<1000 steps/sec)")
        return False

def run_all_tests():
    """Run all non-GUI tests."""
    print("=" * 60)
    print("🧪 MUJOCO NON-GUI TEST SUITE")
    print("=" * 60)
    
    results = []
    
    # Run tests
    results.append(("Basic Physics", test_basic_physics()))
    results.append(("Multiple Objects", test_multiple_objects()))
    results.append(("Model Loading", test_model_loading()))
    results.append(("Performance Benchmark", test_performance_benchmark()))
    
    # Summary
    print("\n" + "=" * 60)
    print("📊 TEST RESULTS SUMMARY")
    print("=" * 60)
    
    passed = 0
    for test_name, result in results:
        status = "✅ PASSED" if result else "❌ FAILED"
        print(f"   {test_name:<20} {status}")
        if result:
            passed += 1
    
    print(f"\n🎯 Overall: {passed}/{len(results)} tests passed")
    
    if passed == len(results):
        print("🎉 All tests PASSED! MuJoCo is working perfectly.")
    else:
        print("⚠️  Some tests failed. Check installation and dependencies.")
    
    return passed == len(results)

if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)