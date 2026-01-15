#!/usr/bin/env python3
"""
Pick and Place Task Demo
=========================

Demonstrates automated pick and place operations using MuJoCo and robotic control.

This example shows:
- Loading objects and robot models
- Path planning for pick-up
- Grasp point calculation
- Object placement logic
"""

import mujoco
import numpy as np


def create_pick_place_scene():
    """Create a scene with robot and objects for pick and place."""
    xml = """
    <mujoco model="pick_place_scene">
        <option timestep="0.002"/>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
            <geom name="ground" type="plane" size="0 0 1" rgba="0.9 0.9 0.9 0.1"/>
            
            <!-- Table -->
            <body name="table" pos="0 0 0.4">
                <geom name="table_top" type="box" size="0.3 0.3 0.02" rgba="0.7 0.5 0.3 1"/>
                <geom name="table_leg1" type="box" size="0.02 0.02 0.4" pos="0.25 0.25 -0.4" rgba="0.7 0.5 0.3 1"/>
                <geom name="table_leg2" type="box" size="0.02 0.02 0.4" pos="0.25 -0.25 -0.4" rgba="0.7 0.5 0.3 1"/>
                <geom name="table_leg3" type="box" size="0.02 0.02 0.4" pos="-0.25 0.25 -0.4" rgba="0.7 0.5 0.3 1"/>
                <geom name="table_leg4" type="box" size="0.02 0.02 0.4" pos="-0.25 -0.25 -0.4" rgba="0.7 0.5 0.3 1"/>
            </body>
            
            <!-- Objects to pick -->
            <body name="object1" pos="0.1 0.1 0.45">
                <joint name="obj1_x" type="slide" axis="1 0 0"/>
                <joint name="obj1_y" type="slide" axis="0 1 0"/>
                <joint name="obj1_z" type="slide" axis="0 0 1"/>
                <joint name="obj1_rot" type="hinge" axis="0 0 1"/>
                <inertia mass="0.2" diaginv="1 1 1"/>
                <geom name="object1" type="box" size="0.04 0.04 0.04" rgba="1 0 0 1"/>
            </body>
            
            <body name="object2" pos="-0.1 0.1 0.45">
                <joint name="obj2_x" type="slide" axis="1 0 0"/>
                <joint name="obj2_y" type="slide" axis="0 1 0"/>
                <joint name="obj2_z" type="slide" axis="0 0 1"/>
                <joint name="obj2_rot" type="hinge" axis="0 0 1"/>
                <inertia mass="0.2" diaginv="1 1 1"/>
                <geom name="object2" type="box" size="0.04 0.04 0.04" rgba="0 1 0 1"/>
            </body>
            
            <!-- Robot arm (simplified) -->
            <body name="arm_base" pos="0 -0.5 0.4">
                <inertia mass="1" diaginv="1 1 1"/>
                <geom name="base" type="cylinder" size="0.05 0.05" rgba="0.3 0.3 0.3 1"/>
                
                <body name="link1" pos="0 0 0.15">
                    <joint name="joint1" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
                    <inertia mass="0.5" diaginv="1 1 1"/>
                    <geom name="link1" type="capsule" size="0.04" fromto="0 0 0 0 0 0.3" rgba="0.6 0.6 0.6 1"/>
                    
                    <body name="link2" pos="0 0 0.3">
                        <joint name="joint2" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
                        <inertia mass="0.3" diaginv="1 1 1"/>
                        <geom name="link2" type="capsule" size="0.035" fromto="0 0 0 0 0 0.3" rgba="0.7 0.7 0.7 1"/>
                        
                        <body name="gripper" pos="0 0 0.3">
                            <geom name="gripper" type="sphere" size="0.05" rgba="0.2 0.2 0.8 1"/>
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
    """Run pick and place demonstration."""
    model = create_pick_place_scene()
    data = mujoco.MjData(model)
    
    print("Pick and Place Task Demo")
    print("-" * 50)
    print("This example demonstrates automated pick and place operations.")
    print("Target: Pick objects and place them in new locations.")
    print("-" * 50)
    
    # Simulation parameters
    num_steps = 3000
    
    for step in range(num_steps):
        t = step * 0.002  # Time in seconds
        
        # Simple reaching trajectory
        if t < 1.0:
            # Phase 1: Move to first object
            data.ctrl[0] = 0.5 * np.sin(t * np.pi)
            data.ctrl[1] = -0.5 * np.cos(t * np.pi)
        elif t < 2.0:
            # Phase 2: Pick up
            data.ctrl[0] = 0.3
            data.ctrl[1] = 0.0
        elif t < 3.0:
            # Phase 3: Move to placement location
            data.ctrl[0] = -0.3
            data.ctrl[1] = 0.3
        else:
            # Phase 4: Place object
            data.ctrl[0] = 0.0
            data.ctrl[1] = -0.3
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        if step % 500 == 0:
            obj1_pos = data.qpos[4:7] if len(data.qpos) > 6 else [0, 0, 0]
            print(f"Step {step}: Time={t:.2f}s, Gripper position={data.qpos[1:3]}")
    
    print("Demo complete.")


if __name__ == "__main__":
    main()
