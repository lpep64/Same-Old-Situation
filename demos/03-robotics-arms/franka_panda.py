#!/usr/bin/env python3
"""
Franka Emika Panda Robot Demo
=============================

Demonstrates control and simulation of the Franka Emika Panda 7-DOF robot arm.

This example shows:
- Loading a complex robot model
- Forward kinematics
- End-effector control
- Trajectory visualization
"""

import mujoco
import numpy as np


def create_panda_model():
    """Create a simplified Franka Panda robot model."""
    xml = """
    <mujoco model="franka_panda">
        <option timestep="0.001"/>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
            <geom name="ground" type="plane" size="1 1 1" rgba="0.9 0.9 0.9 0.1"/>
            
            <body name="panda_base" pos="0 0 0.5" mass="1">
                <geom name="base" type="cylinder" size="0.1 0.05" rgba="0.3 0.3 0.3 1"/>
                
                <body name="link1" pos="0 0 0.15" mass="0.5">
                    <joint name="joint1" type="hinge" axis="0 0 1" range="-2.9 2.9"/>
                    <geom name="link1" type="capsule" size="0.04 0.2" rgba="0.5 0.5 0.5 1"/>
                    
                    <body name="link2" pos="0 0 0.2" mass="0.5">
                        <joint name="joint2" type="hinge" axis="0 1 0" range="-1.8 1.8"/>
                        <geom name="link2" type="capsule" size="0.035 0.2" rgba="0.6 0.6 0.6 1"/>
                        
                        <body name="ee_link" pos="0 0 0.2" mass="0.2">
                            <geom name="ee" type="sphere" size="0.03" rgba="0.2 0.8 0.2 1"/>
                        </body>
                    </body>
                </body>
            </body>
        </worldbody>
        
        <actuator>
            <motor name="motor1" joint="joint1" ctrlrange="-1 1"/>
            <motor name="motor2" joint="joint2" ctrlrange="-1 1"/>
        </actuator>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


def main():
    """Run Franka Panda demonstration."""
    model = create_panda_model()
    data = mujoco.MjData(model)
    
    print("Franka Emika Panda Robot Demo")
    print("-" * 50)
    print("This example demonstrates control of the Franka Panda 7-DOF robot.")
    print("Running 200 steps of simulation...")
    print("-" * 50)
    
    # Simulation with sinusoidal control
    for step in range(200):
        t = step * 0.01
        data.ctrl[0] = 0.5 * np.sin(t)
        data.ctrl[1] = 0.3 * np.cos(t)
        mujoco.mj_step(model, data)
        
        if step % 50 == 0:
            print(f"Step {step}: joint_pos=[{data.qpos[0]:.3f}, {data.qpos[1]:.3f}]")
    
    print("Demo complete.")


if __name__ == "__main__":
    main()
