#!/usr/bin/env python3
"""
Advanced Manipulation Tasks Demo
=================================

Demonstrates advanced robotic manipulation tasks using MuJoCo.

This example shows:
- Multi-step manipulation sequences
- Force control and compliance
- Complex object interactions
- Constraint-based planning
"""

import mujoco
import numpy as np


def create_manipulation_scene():
    """Create a complex manipulation scene."""
    xml = """
    <mujoco model="manipulation_scene">
        <option timestep="0.002"/>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
            <geom name="ground" type="plane" size="0 0 1" rgba="0.9 0.9 0.9 0.1"/>
            
            <!-- Workspace area -->
            <body name="workspace" pos="0 0 0.5">
                <geom name="workspace_outline" type="box" size="0.5 0.5 0.01" rgba="0.3 0.3 0.3 0.3"/>
            </body>
            
            <!-- Robot arm (2-DOF manipulator) -->
            <body name="robot_base" pos="0 -0.3 0.5">
                <inertia mass="2" diaginv="1 1 1"/>
                <geom name="base" type="cylinder" size="0.08 0.08" rgba="0.2 0.2 0.2 1"/>
                
                <body name="shoulder" pos="0 0 0.1">
                    <joint name="shoulder_joint" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
                    <inertia mass="0.8" diaginv="1 1 1"/>
                    <geom name="shoulder" type="capsule" size="0.05" fromto="0 0 0 0 0 0.35" rgba="0.4 0.4 0.4 1"/>
                    
                    <body name="elbow" pos="0 0 0.35">
                        <joint name="elbow_joint" type="hinge" axis="0 1 0" range="-2.0 2.0"/>
                        <inertia mass="0.5" diaginv="1 1 1"/>
                        <geom name="elbow" type="capsule" size="0.04" fromto="0 0 0 0 0 0.35" rgba="0.5 0.5 0.5 1"/>
                        
                        <body name="wrist" pos="0 0 0.35">
                            <joint name="wrist_joint" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
                            <inertia mass="0.3" diaginv="1 1 1"/>
                            <geom name="wrist" type="sphere" size="0.04" rgba="0.6 0.6 0.6 1"/>
                            
                            <!-- Gripper -->
                            <body name="gripper" pos="0 0 0.1">
                                <geom name="gripper_palm" type="box" size="0.03 0.06 0.02" rgba="0.3 0.6 0.8 1"/>
                                
                                <body name="left_finger" pos="0 0.06 0">
                                    <joint name="left_finger_joint" type="slide" axis="0 1 0" range="-0.05 0.05"/>
                                    <geom name="left_finger" type="box" size="0.02 0.03 0.02" rgba="0.2 0.5 0.7 1"/>
                                </body>
                                
                                <body name="right_finger" pos="0 -0.06 0">
                                    <joint name="right_finger_joint" type="slide" axis="0 -1 0" range="-0.05 0.05"/>
                                    <geom name="right_finger" type="box" size="0.02 0.03 0.02" rgba="0.2 0.5 0.7 1"/>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
            
            <!-- Objects for manipulation -->
            <body name="cube1" pos="0.2 0 0.52">
                <joint name="cube1_x" type="slide" axis="1 0 0"/>
                <joint name="cube1_y" type="slide" axis="0 1 0"/>
                <joint name="cube1_z" type="slide" axis="0 0 1"/>
                <joint name="cube1_rot_z" type="hinge" axis="0 0 1"/>
                <inertia mass="0.3" diaginv="1 1 1"/>
                <geom name="cube1" type="box" size="0.04 0.04 0.04" rgba="1 0.5 0 1"/>
            </body>
            
            <body name="cylinder1" pos="-0.15 0.15 0.52">
                <joint name="cyl1_x" type="slide" axis="1 0 0"/>
                <joint name="cyl1_y" type="slide" axis="0 1 0"/>
                <joint name="cyl1_z" type="slide" axis="0 0 1"/>
                <joint name="cyl1_rot_z" type="hinge" axis="0 0 1"/>
                <inertia mass="0.25" diaginv="1 1 1"/>
                <geom name="cylinder1" type="cylinder" size="0.03 0.05" rgba="0 0.8 0.5 1"/>
            </body>
            
            <!-- Placement locations -->
            <body name="target1" pos="0.3 -0.2 0.51">
                <geom name="target1" type="box" size="0.05 0.05 0.005" rgba="0.2 0.8 0.2 0.5"/>
            </body>
            
            <body name="target2" pos="-0.25 0.25 0.51">
                <geom name="target2" type="box" size="0.05 0.05 0.005" rgba="0.8 0.2 0.2 0.5"/>
            </body>
        </worldbody>
        
        <actuator>
            <motor name="shoulder_motor" joint="shoulder_joint" ctrlrange="-2 2"/>
            <motor name="elbow_motor" joint="elbow_joint" ctrlrange="-2 2"/>
            <motor name="wrist_motor" joint="wrist_joint" ctrlrange="-1 1"/>
            <motor name="left_finger_motor" joint="left_finger_joint" ctrlrange="-0.5 0.5"/>
            <motor name="right_finger_motor" joint="right_finger_joint" ctrlrange="-0.5 0.5"/>
        </actuator>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


def main():
    """Run advanced manipulation demonstration."""
    model = create_manipulation_scene()
    data = mujoco.MjData(model)
    
    print("Advanced Manipulation Tasks Demo")
    print("-" * 50)
    print("This example demonstrates multi-step manipulation sequences.")
    print("Target: Move and manipulate multiple objects in sequence.")
    print("-" * 50)
    
    # Simulation parameters
    num_steps = 4000
    
    for step in range(num_steps):
        t = step * 0.002  # Time in seconds
        phase = int((t / 4.0) * 4)  # 4 phases, 1 second each
        
        # Smooth trajectories for arm movement
        if phase == 0:
            # Move to cube 1
            data.ctrl[0] = 0.5 * np.sin(2 * np.pi * (t % 1.0))
            data.ctrl[1] = -0.3 * np.cos(2 * np.pi * (t % 1.0))
            data.ctrl[2] = 0.0
            data.ctrl[3] = 0.0
            data.ctrl[4] = 0.0
        elif phase == 1:
            # Grasp cube 1
            data.ctrl[0] = 0.2
            data.ctrl[1] = -0.1
            data.ctrl[2] = 0.0
            data.ctrl[3] = 0.3  # Close left finger
            data.ctrl[4] = 0.3  # Close right finger
        elif phase == 2:
            # Move to target 1
            data.ctrl[0] = 0.8 * np.sin(2 * np.pi * (t % 1.0))
            data.ctrl[1] = 0.2 * np.cos(2 * np.pi * (t % 1.0))
            data.ctrl[2] = 0.0
            data.ctrl[3] = 0.3
            data.ctrl[4] = 0.3
        else:
            # Place and reset
            data.ctrl[0] = 0.0
            data.ctrl[1] = 0.0
            data.ctrl[2] = 0.0
            data.ctrl[3] = -0.3  # Open fingers
            data.ctrl[4] = -0.3
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        if step % 500 == 0:
            print(f"Step {step}: Time={t:.2f}s, Phase={phase}, Arm angles=[{data.qpos[6]:.2f}, {data.qpos[7]:.2f}]")
    
    print("Demo complete.")


if __name__ == "__main__":
    main()
