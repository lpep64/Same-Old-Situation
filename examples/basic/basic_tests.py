#!/usr/bin/env python3#!/usr/bin/env python3

""""""

MuJoCo Non-GUI Basic TestsMuJoCo Non-GUI Basic Tests



This module provides basic non-GUI testing functionality for MuJoCo.This module provides basic non-GUI testing functionality for MuJoCo.

Perfect for automated testing, batch processing, and headless environments.Perfect for automated testing, batch processing, and headless environments.

""""""

    print("="* 60)

import mujoco    print("MUJOCO NON-GUI TEST SUITE")

import numpy as np    print("="* 60)port mujo    # Summary

import time    print("\n" + "="* 60)

import os    print("TEST RESULTS SUMMARY")

    print("="* 60)

def test_basic_physics():    

    """Test basic physics simulation without GUI."""    passed = 0

    print("[TEST] Running Basic Physics Test...")    for test_name, result in results:

            status = "[PASS] PASSED" if result else "[FAIL] FAILED"

    xml = """        print(f"   {test_name:<20} {status}")t numpy as np

    <mujoco model="basic_test">import time

      <option timestep="0.01"/>import os

      <worldbody>

        <body name="ball" pos="0 0 2">def test_basic_physics():

          <joint type="free"/>    """Test basic physics simulation without GUI."""

          <geom size="0.1" rgba="1 0 0 1" mass="1"/>    print("[TEST] Running Basic Physics Test...")

        </body>    

        <geom name="floor" pos="0 0 0" size="2 2 0.05" type="box" rgba="0.8 0.8 0.8 1"/>    xml = """

      </worldbody>    <mujoco model="basic_test">

    </mujoco>      <option timestep="0.01"/>

    """      <worldbody>

            <body name="ball" pos="0 0 2">

    model = mujoco.MjModel.from_xml_string(xml)          <joint type="free"/>

    data = mujoco.MjData(model)          <geom size="0.1" rgba="1 0 0 1" mass="1"/>

            </body>

    initial_height = data.qpos[2]  # z position        <geom name="floor" pos="0 0 0" size="2 2 0.05" type="box" rgba="0.8 0.8 0.8 1"/>

    print(f"   Initial ball height: {initial_height:.3f}m")      </worldbody>

        </mujoco>

    # Run simulation    """

    steps = 1000    

    for i in range(steps):    model = mujoco.MjModel.from_xml_string(xml)

        mujoco.mj_step(model, data)    data = mujoco.MjData(model)

        if i % 200 == 0:    

            height = data.qpos[2]    initial_height = data.qpos[2]  # z position

            print(f"   Step {i:4d}: Ball height = {height:.3f}m")    print(f"   Initial ball height: {initial_height:.3f}m")

        

    final_height = data.qpos[2]    # Run simulation

    fall_distance = initial_height - final_height    steps = 1000

        for i in range(steps):

    print(f"   Final height: {final_height:.3f}m")        mujoco.mj_step(model, data)

    print(f"   Total fall: {fall_distance:.3f}m")        if i % 200 == 0:

                height = data.qpos[2]

    # Verify physics worked            print(f"   Step {i:4d}: Ball height = {height:.3f}m")

    if fall_distance > 0.5:    

        print("   [PASS] Basic physics test PASSED")    final_height = data.qpos[2]

        return True    fall_distance = initial_height - final_height

    else:    

        print("   [FAIL] Basic physics test FAILED")    print(f"   Final height: {final_height:.3f}m")

        return False    print(f"   Total fall: {fall_distance:.3f}m")

    

def test_multiple_objects():    # Verify physics worked

    """Test simulation with multiple interacting objects."""    if fall_distance > 0.5:

    print("\n[TEST] Running Multiple Objects Test...")        print("   [PASS] Basic physics test PASSED")

            return True

    xml = """    else:

    <mujoco model="multi_objects">        print("   [FAIL] Basic physics test FAILED")

      <option timestep="0.005"/>        return False

      <worldbody>

        <geom name="floor" pos="0 0 -0.1" size="3 3 0.05" type="box" rgba="0.8 0.8 0.8 1"/>def test_multiple_objects():

            """Test simulation with multiple interacting objects."""

        <body name="sphere1" pos="-0.5 0 2">    print("\n[TEST] Running Multiple Objects Test...")

          <joint type="free"/>    

          <geom size="0.1" type="sphere" rgba="1 0 0 1" mass="1"/>    xml = """

        </body>    <mujoco model="multi_objects">

              <option timestep="0.005"/>

        <body name="sphere2" pos="0.5 0 2.5">      <worldbody>

          <joint type="free"/>        <geom name="floor" pos="0 0 -0.1" size="3 3 0.05" type="box" rgba="0.8 0.8 0.8 1"/>

          <geom size="0.12" type="sphere" rgba="0 1 0 1" mass="1.2"/>        

        </body>        <body name="sphere1" pos="-0.5 0 2">

                  <joint type="free"/>

        <body name="box1" pos="0 -0.5 1.5">          <geom size="0.1" type="sphere" rgba="1 0 0 1" mass="1"/>

          <joint type="free"/>        </body>

          <geom size="0.1 0.1 0.1" type="box" rgba="0 0 1 1" mass="0.8"/>        

        </body>        <body name="sphere2" pos="0.5 0 2.5">

                  <joint type="free"/>

        <body name="capsule1" pos="0 0.5 3">          <geom size="0.12" type="sphere" rgba="0 1 0 1" mass="1.2"/>

          <joint type="free"/>        </body>

          <geom size="0.05" fromto="0 0 -0.15 0 0 0.15" type="capsule" rgba="1 1 0 1" mass="0.6"/>        

        </body>        <body name="box1" pos="0 -0.5 1.5">

      </worldbody>          <joint type="free"/>

    </mujoco>          <geom size="0.1 0.1 0.1" type="box" rgba="0 0 1 1" mass="0.8"/>

    """        </body>

            

    model = mujoco.MjModel.from_xml_string(xml)        <body name="capsule1" pos="0 0.5 3">

    data = mujoco.MjData(model)          <joint type="free"/>

              <geom size="0.05" fromto="0 0 -0.15 0 0 0.15" type="capsule" rgba="1 1 0 1" mass="0.6"/>

    # Initialize the simulation data properly        </body>

    mujoco.mj_forward(model, data)      </worldbody>

        </mujoco>

    print(f"   Model loaded: {model.nbody} bodies, {model.nv} DOFs")    """

        

    # Track center of mass    model = mujoco.MjModel.from_xml_string(xml)

    initial_com = np.mean([data.xpos[body_id] for body_id in range(1, model.nbody)], axis=0)    data = mujoco.MjData(model)

    print(f"   Initial COM: ({initial_com[0]:.2f}, {initial_com[1]:.2f}, {initial_com[2]:.2f})")    

        # Initialize the simulation data properly

    # Run simulation    mujoco.mj_forward(model, data)

    steps = 2000    

    for i in range(steps):    print(f"   Model loaded: {model.nbody} bodies, {model.nv} DOFs")

        mujoco.mj_step(model, data)    

        if i % 400 == 0:    # Track center of mass

            com = np.mean([data.xpos[body_id] for body_id in range(1, model.nbody)], axis=0)    initial_com = np.mean([data.xpos[body_id] for body_id in range(1, model.nbody)], axis=0)

            print(f"   Step {i:4d}: COM Z = {com[2]:.3f}m")    

        # Run simulation

    final_com = np.mean([data.xpos[body_id] for body_id in range(1, model.nbody)], axis=0)    steps = 2000

    fall_distance = initial_com[2] - final_com[2]    for step in range(steps):

            mujoco.mj_step(model, data)

    print(f"   Final COM height: {final_com[2]:.3f}m")        if step % 400 == 0:

    print(f"   Total fall: {fall_distance:.3f}m")            current_com = np.mean([data.xpos[body_id] for body_id in range(1, model.nbody)], axis=0)

                print(f"   Step {step:4d}: Center of mass height = {current_com[2]:.3f}m")

    if fall_distance > 0.8:    

        print("   [PASS] Multiple objects test PASSED")    final_com = np.mean([data.xpos[body_id] for body_id in range(1, model.nbody)], axis=0)

        return True    com_fall = initial_com[2] - final_com[2]

    else:    

        print("   [FAIL] Multiple objects test FAILED")    print(f"   COM fell {com_fall:.3f}m")

        return False    

    if com_fall > 0.5:  # More realistic expectation for objects settling

def test_model_loading():        print("   ✅ Multiple objects test PASSED")

    """Test loading various model files."""        return True

    print("\n[TEST] Running Model Loading Test...")    else:

            print("   ❌ Multiple objects test FAILED")

    # Test models to load        return False

    test_models = [

        "models/basic/ball.xml",def test_model_loading():

        "models/basic/cube.xml",     """Test loading various model files."""

        "models/basic/pendulum.xml",    print("\n🔬 Running Model Loading Test...")

        "models/robots/franka_fr3.xml",    

        "projects/NiryoArm/niryo_arm.xml"    # Test built-in models

    ]    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

        model_paths = [

    passed = 0        os.path.join(workspace_root, "model", "humanoid", "humanoid.xml"),

    total = len(test_models)        os.path.join(workspace_root, "model", "cube", "cube.xml"), 

            os.path.join(workspace_root, "model", "car", "car.xml")

    for model_path in test_models:    ]

        try:    

            if os.path.exists(model_path):    passed = 0

                model = mujoco.MjModel.from_xml_path(model_path)    total = len(model_paths)

                model_name = os.path.basename(model_path)    

                print(f"   [OK] {model_name}: {model.nbody} bodies, {model.nv} DOFs")    for model_path in model_paths:

                passed += 1        try:

            else:            if os.path.exists(model_path):

                model_name = os.path.basename(model_path)                model = mujoco.MjModel.from_xml_path(model_path)

                print(f"   [WARN] {model_name}: File not found")                data = mujoco.MjData(model)

        except Exception as e:                

            model_name = os.path.basename(model_path)                # Quick simulation test

            print(f"   [ERROR] {model_name}: Error - {str(e)[:40]}...")                for _ in range(10):

                        mujoco.mj_step(model, data)

    print(f"   Loaded {passed}/{total} models successfully")                

    return passed > 0                model_name = os.path.basename(model_path)

                print(f"   ✅ {model_name}: {model.nbody} bodies, {model.nv} DOFs")

def test_performance_benchmark():                passed += 1

    """Benchmark simulation performance."""            else:

    print("\n[TEST] Running Performance Benchmark...")                model_name = os.path.basename(model_path)

                    print(f"   [WARN] {model_name}: File not found")

    xml = """        except Exception as e:

    <mujoco model="performance_test">            model_name = os.path.basename(model_path)

      <option timestep="0.002"/>            print(f"   [ERROR] {model_name}: Error - {str(e)[:40]}...")

      <worldbody>    

        <geom name="floor" pos="0 0 -0.1" size="4 4 0.05" type="box"/>    print(f"   Loaded {passed}/{total} models successfully")

            return passed > 0

        <!-- Create multiple objects for stress test -->

        <body name="obj1" pos="-1 -1 1"><joint type="free"/><geom size="0.05"/></body>def test_performance_benchmark():

        <body name="obj2" pos="0 -1 1"><joint type="free"/><geom size="0.05"/></body>    """Benchmark simulation performance."""

        <body name="obj3" pos="1 -1 1"><joint type="free"/><geom size="0.05"/></body>    print("\n[TEST] Running Performance Benchmark...")

        <body name="obj4" pos="-1 0 1"><joint type="free"/><geom size="0.05"/></body>    

        <body name="obj5" pos="0 0 1"><joint type="free"/><geom size="0.05"/></body>    xml = """

        <body name="obj6" pos="1 0 1"><joint type="free"/><geom size="0.05"/></body>    <mujoco model="performance_test">

        <body name="obj7" pos="-1 1 1"><joint type="free"/><geom size="0.05"/></body>      <option timestep="0.002"/>

        <body name="obj8" pos="0 1 1"><joint type="free"/><geom size="0.05"/></body>      <worldbody>

        <body name="obj9" pos="1 1 1"><joint type="free"/><geom size="0.05"/></body>        <geom name="floor" pos="0 0 -0.1" size="4 4 0.05" type="box"/>

      </worldbody>        

    </mujoco>        <!-- Create multiple objects for stress test -->

    """        <body name="obj1" pos="-1 -1 1"><joint type="free"/><geom size="0.05"/></body>

            <body name="obj2" pos="0 -1 1"><joint type="free"/><geom size="0.05"/></body>

    model = mujoco.MjModel.from_xml_string(xml)        <body name="obj3" pos="1 -1 1"><joint type="free"/><geom size="0.05"/></body>

    data = mujoco.MjData(model)        <body name="obj4" pos="-1 0 1"><joint type="free"/><geom size="0.05"/></body>

            <body name="obj5" pos="0 0 1"><joint type="free"/><geom size="0.05"/></body>

    steps = 10000        <body name="obj6" pos="1 0 1"><joint type="free"/><geom size="0.05"/></body>

    print(f"   Running {steps:,} simulation steps...")        <body name="obj7" pos="-1 1 1"><joint type="free"/><geom size="0.05"/></body>

            <body name="obj8" pos="0 1 1"><joint type="free"/><geom size="0.05"/></body>

    start_time = time.perf_counter()        <body name="obj9" pos="1 1 1"><joint type="free"/><geom size="0.05"/></body>

          </worldbody>

    for i in range(steps):    </mujoco>

        mujoco.mj_step(model, data)    """

        

    end_time = time.perf_counter()    model = mujoco.MjModel.from_xml_string(xml)

    elapsed = end_time - start_time    data = mujoco.MjData(model)

    steps_per_sec = steps / elapsed    

        steps = 10000

    print(f"   Elapsed time: {elapsed:.3f}s")    print(f"   Running {steps:,} simulation steps...")

    print(f"   Performance: {steps_per_sec:,.0f} steps/sec")    

    print(f"   Real-time factor: {(steps * model.opt.timestep / elapsed):.1f}x")    start_time = time.perf_counter()

        

    if steps_per_sec > 1000:    for i in range(steps):

        print("   [PASS] Performance test PASSED (>1000 steps/sec)")        mujoco.mj_step(model, data)

        return True    

    else:    end_time = time.perf_counter()

        print("   [WARN] Performance test SLOW (<1000 steps/sec)")    elapsed = end_time - start_time

        return False    steps_per_sec = steps / elapsed

    

def run_all_tests():    print(f"   Elapsed time: {elapsed:.3f}s")

    """Run all non-GUI tests."""    print(f"   Performance: {steps_per_sec:,.0f} steps/sec")

    print("=" * 60)    print(f"   Real-time factor: {(steps * model.opt.timestep / elapsed):.1f}x")

    print("MUJOCO NON-GUI TEST SUITE")    

    print("=" * 60)    if steps_per_sec > 1000:

            print("   [PASS] Performance test PASSED (>1000 steps/sec)")

    results = []        return True

        else:

    # Run tests        print("   [WARN] Performance test SLOW (<1000 steps/sec)")

    results.append(("Basic Physics", test_basic_physics()))        return False

    results.append(("Multiple Objects", test_multiple_objects()))

    results.append(("Model Loading", test_model_loading()))def run_all_tests():

    results.append(("Performance Benchmark", test_performance_benchmark()))    """Run all non-GUI tests."""

        print("=" * 60)

    # Summary    print("🧪 MUJOCO NON-GUI TEST SUITE")

    print("\n" + "=" * 60)    print("=" * 60)

    print("TEST RESULTS SUMMARY")    

    print("=" * 60)    results = []

        

    passed = 0    # Run tests

    for test_name, result in results:    results.append(("Basic Physics", test_basic_physics()))

        status = "[PASS] PASSED" if result else "[FAIL] FAILED"    results.append(("Multiple Objects", test_multiple_objects()))

        print(f"   {test_name:<20} {status}")    results.append(("Model Loading", test_model_loading()))

        if result:    results.append(("Performance Benchmark", test_performance_benchmark()))

            passed += 1    

        # Summary

    print(f"\nOverall: {passed}/{len(results)} tests passed")    print("\n" + "=" * 60)

        print("📊 TEST RESULTS SUMMARY")

    if passed == len(results):    print("=" * 60)

        print("All tests PASSED! MuJoCo is working perfectly.")    

    else:    passed = 0

        print("[WARN] Some tests failed. Check installation and dependencies.")    for test_name, result in results:

            status = "✅ PASSED" if result else "❌ FAILED"

    return passed == len(results)        print(f"   {test_name:<20} {status}")

        if result:

if __name__ == "__main__":            passed += 1

    success = run_all_tests()    

    exit(0 if success else 1)    print(f"\nOverall: {passed}/{len(results)} tests passed")
    
    if passed == len(results):
        print("All tests PASSED! MuJoCo is working perfectly.")
    else:
        print("[WARN] Some tests failed. Check installation and dependencies.")
    
    return passed == len(results)

if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)