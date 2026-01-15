#!/usr/bin/env python3
"""
MuJoCo Non-GUI Basic Tests

This module provides basic non-GUI testing functionality for MuJoCo.
Perfect for automated testing, batch processing, and headless environments.
"""

import mujoco
import numpy as np
import time


def test_basic_physics():
    """Test basic physics simulation without GUI."""
    print("[TEST] Running Basic Physics Test...")

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
    steps = 100  # Reduced for quick testing
    for i in range(steps):
        mujoco.mj_step(model, data)
        if i % 25 == 0:
            height = data.qpos[2]
            print(f"   Step {i:4d}: Ball height = {height:.3f}m")

    final_height = data.qpos[2]
    fall_distance = initial_height - final_height

    print(f"   Final height: {final_height:.3f}m")
    print(f"   Total fall: {fall_distance:.3f}m")

    # Basic assertion
    assert fall_distance > 0, "Ball should have fallen"
    print("[PASS] Basic physics test passed!")
    return True


def test_multiple_bodies():
    """Test simulation with multiple bodies."""
    print("[TEST] Running Multiple Bodies Test...")

    xml = """
    <mujoco model="multi_body">
      <option timestep="0.01"/>
      <worldbody>
        <body name="body1" pos="0 0 1">
          <joint type="free"/>
          <geom size="0.1" rgba="1 0 0 1" mass="1"/>
        </body>
        <body name="body2" pos="1 0 1">
          <joint type="free"/>
          <geom size="0.1" rgba="0 1 0 1" mass="1"/>
        </body>
        <geom name="floor" pos="0 0 0" size="3 3 0.05" type="box" rgba="0.8 0.8 0.8 1"/>
      </worldbody>
    </mujoco>
    """

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    # Run simulation
    for i in range(50):
        mujoco.mj_step(model, data)

    print("[PASS] Multiple bodies test passed!")
    return True


def run_all_tests():
    """Run all available tests."""
    print("=" * 60)
    print("MUJOCO NON-GUI TEST SUITE")
    print("=" * 60)

    results = []
    tests = [
        ("Basic Physics", test_basic_physics),
        ("Multiple Bodies", test_multiple_bodies),
    ]

    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
            print()
        except Exception as e:
            print(f"[FAIL] {test_name} failed: {e}")
            results.append((test_name, False))
            print()

    # Summary
    print("=" * 60)
    print("TEST RESULTS SUMMARY")
    print("=" * 60)

    passed = sum(1 for _, result in results if result)
    total = len(results)

    for test_name, result in results:
        status = "[PASS]" if result else "[FAIL]"
        print(f"   {test_name:<30} {status}")

    print("=" * 60)
    print(f"Results: {passed}/{total} tests passed")
    print("=" * 60)

    return passed == total


if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)
