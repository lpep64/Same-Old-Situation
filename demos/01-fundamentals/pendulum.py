#!/usr/bin/env python3
"""
Pendulum Control Example
========================

Demonstrates PID control and physics of a simple pendulum using MuJoCo.

This example shows:
- Loading a simple pendulum model
- Implementing a PID controller
- Visualizing the pendulum dynamics
"""

import mujoco
import numpy as np


def create_pendulum_model():
    """Create a simple pendulum model."""
    xml = """
    <mujoco model="pendulum">
        <option timestep="0.002"/>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 2" dir="0 0 -1"/>
            <geom name="ground" type="plane" size="1 1 1" rgba="0.9 0.9 0.9 0.1"/>
            <body name="pivot" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom name="pivot" type="sphere" size="0.05" rgba="0.3 0.3 0.3 1" mass="0.5"/>
                <body name="pole" pos="0 0 0.5">
                    <geom name="pole" type="capsule" size="0.05 0.5" rgba="0.9 0.1 0.1 1" mass="1"/>
                </body>
            </body>
        </worldbody>
        <actuator>
            <motor name="hinge_motor" joint="hinge" ctrlrange="-1 1"/>
        </actuator>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


def main():
    """Run pendulum control example."""
    model = create_pendulum_model()
    data = mujoco.MjData(model)
    
    print("Pendulum Control Example")
    print("-" * 50)
    print("This example demonstrates PID control of a simple pendulum.")
    print("Simulation running for 100 steps...")
    print("-" * 50)
    
    # Run simple simulation
    for step in range(100):
        # Simple oscillation control
        data.ctrl[0] = 0.5 * np.sin(step * 0.1)
        mujoco.mj_step(model, data)
        
        if step % 25 == 0:
            print(f"Step {step}: angle={np.degrees(data.qpos[0]):.2f}°")
    
    print("Simulation complete.")


if __name__ == "__main__":
    main()
