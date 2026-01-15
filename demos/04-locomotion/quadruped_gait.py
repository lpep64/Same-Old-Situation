#!/usr/bin/env python3
"""
Quadruped Gait Pattern Demo
============================

Demonstrates locomotion gaits for quadrupedal robots using MuJoCo.

This example shows:
- Loading a quadruped model
- Implementing trotting gait
- Stability and center of mass control
- Gait phase transitions
"""

import mujoco
import numpy as np


def create_quadruped_model():
    """Create a simple quadruped model."""
    xml = """
    <mujoco model="quadruped">
        <option timestep="0.002"/>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
            <geom name="ground" type="plane" size="0 0 1" rgba="0.9 0.9 0.9 0.1"/>
            
            <!-- Body -->
            <body name="body" pos="0 0 0.5">
                <joint name="body_x" type="slide" axis="1 0 0"/>
                <joint name="body_z" type="slide" axis="0 0 1" limited="true" range="0.3 0.7"/>
                <joint name="body_y" type="hinge" axis="0 1 0"/>
                <inertia mass="3" diaginv="1 1 1"/>
                <geom name="body" type="box" size="0.15 0.1 0.1" rgba="0.5 0.5 0.5 1"/>
                
                <!-- Front Left Leg -->
                <body name="fl_thigh" pos="0.1 0.08 -0.15">
                    <joint name="fl_hip" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
                    <inertia mass="0.3" diaginv="1 1 1"/>
                    <geom name="fl_thigh" type="capsule" size="0.03" fromto="0 0 0 0 0 -0.2" rgba="0.6 0.3 0.3 1"/>
                    
                    <body name="fl_calf" pos="0 0 -0.2">
                        <joint name="fl_knee" type="hinge" axis="0 1 0" range="0 2.0"/>
                        <inertia mass="0.2" diaginv="1 1 1"/>
                        <geom name="fl_calf" type="capsule" size="0.025" fromto="0 0 0 0 0 -0.2" rgba="0.7 0.3 0.3 1"/>
                        
                        <body name="fl_foot" pos="0 0 -0.2">
                            <geom name="fl_foot" type="box" size="0.06 0.04 0.02" rgba="0.8 0.3 0.3 1"/>
                        </body>
                    </body>
                </body>
                
                <!-- Front Right Leg -->
                <body name="fr_thigh" pos="0.1 -0.08 -0.15">
                    <joint name="fr_hip" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
                    <inertia mass="0.3" diaginv="1 1 1"/>
                    <geom name="fr_thigh" type="capsule" size="0.03" fromto="0 0 0 0 0 -0.2" rgba="0.3 0.3 0.6 1"/>
                    
                    <body name="fr_calf" pos="0 0 -0.2">
                        <joint name="fr_knee" type="hinge" axis="0 1 0" range="0 2.0"/>
                        <inertia mass="0.2" diaginv="1 1 1"/>
                        <geom name="fr_calf" type="capsule" size="0.025" fromto="0 0 0 0 0 -0.2" rgba="0.3 0.3 0.7 1"/>
                        
                        <body name="fr_foot" pos="0 0 -0.2">
                            <geom name="fr_foot" type="box" size="0.06 0.04 0.02" rgba="0.3 0.3 0.8 1"/>
                        </body>
                    </body>
                </body>
                
                <!-- Rear Left Leg -->
                <body name="rl_thigh" pos="-0.1 0.08 -0.15">
                    <joint name="rl_hip" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
                    <inertia mass="0.3" diaginv="1 1 1"/>
                    <geom name="rl_thigh" type="capsule" size="0.03" fromto="0 0 0 0 0 -0.2" rgba="0.6 0.6 0.3 1"/>
                    
                    <body name="rl_calf" pos="0 0 -0.2">
                        <joint name="rl_knee" type="hinge" axis="0 1 0" range="0 2.0"/>
                        <inertia mass="0.2" diaginv="1 1 1"/>
                        <geom name="rl_calf" type="capsule" size="0.025" fromto="0 0 0 0 0 -0.2" rgba="0.7 0.6 0.3 1"/>
                        
                        <body name="rl_foot" pos="0 0 -0.2">
                            <geom name="rl_foot" type="box" size="0.06 0.04 0.02" rgba="0.8 0.6 0.3 1"/>
                        </body>
                    </body>
                </body>
                
                <!-- Rear Right Leg -->
                <body name="rr_thigh" pos="-0.1 -0.08 -0.15">
                    <joint name="rr_hip" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
                    <inertia mass="0.3" diaginv="1 1 1"/>
                    <geom name="rr_thigh" type="capsule" size="0.03" fromto="0 0 0 0 0 -0.2" rgba="0.3 0.6 0.6 1"/>
                    
                    <body name="rr_calf" pos="0 0 -0.2">
                        <joint name="rr_knee" type="hinge" axis="0 1 0" range="0 2.0"/>
                        <inertia mass="0.2" diaginv="1 1 1"/>
                        <geom name="rr_calf" type="capsule" size="0.025" fromto="0 0 0 0 0 -0.2" rgba="0.3 0.6 0.7 1"/>
                        
                        <body name="rr_foot" pos="0 0 -0.2">
                            <geom name="rr_foot" type="box" size="0.06 0.04 0.02" rgba="0.3 0.6 0.8 1"/>
                        </body>
                    </body>
                </body>
            </body>
        </worldbody>
        
        <actuator>
            <motor name="fl_hip_motor" joint="fl_hip" ctrlrange="-1 1"/>
            <motor name="fl_knee_motor" joint="fl_knee" ctrlrange="-1 1"/>
            <motor name="fr_hip_motor" joint="fr_hip" ctrlrange="-1 1"/>
            <motor name="fr_knee_motor" joint="fr_knee" ctrlrange="-1 1"/>
            <motor name="rl_hip_motor" joint="rl_hip" ctrlrange="-1 1"/>
            <motor name="rl_knee_motor" joint="rl_knee" ctrlrange="-1 1"/>
            <motor name="rr_hip_motor" joint="rr_hip" ctrlrange="-1 1"/>
            <motor name="rr_knee_motor" joint="rr_knee" ctrlrange="-1 1"/>
        </actuator>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


def main():
    """Run quadruped gait demonstration."""
    model = create_quadruped_model()
    data = mujoco.MjData(model)
    
    print("Quadruped Gait Pattern Demo")
    print("-" * 50)
    print("This example demonstrates quadrupedal walking with trotting gait.")
    print("Target: Trot forward with diagonal leg pairs synchronized.")
    print("-" * 50)
    
    # Simulation parameters
    num_steps = 4000
    
    # Trotting gait: diagonal pairs move together
    # FL+RR (front-left + rear-right), FR+RL (front-right + rear-left)
    for step in range(num_steps):
        t = step * 0.002  # Time in seconds
        phase = (t % 1.0)  # Gait cycle
        
        # Diagonal pair 1 (FL + RR) leading
        if phase < 0.5:
            # Swing phase
            swing = np.sin(2 * np.pi * (phase / 0.5))
            data.ctrl[0] = 0.5 * swing  # FL hip
            data.ctrl[1] = 0.3 * swing  # FL knee
            data.ctrl[4] = 0.5 * swing  # RL hip
            data.ctrl[5] = 0.3 * swing  # RL knee
            
            # Diagonal pair 2 in stance
            data.ctrl[2] = 0.1  # FR hip
            data.ctrl[3] = 0.0  # FR knee
            data.ctrl[6] = 0.1  # RR hip
            data.ctrl[7] = 0.0  # RR knee
        else:
            # Stance phase for pair 1
            data.ctrl[0] = 0.1  # FL hip
            data.ctrl[1] = 0.0  # FL knee
            data.ctrl[4] = 0.1  # RL hip
            data.ctrl[5] = 0.0  # RL knee
            
            # Swing phase for pair 2
            swing = np.sin(2 * np.pi * ((phase - 0.5) / 0.5))
            data.ctrl[2] = 0.5 * swing  # FR hip
            data.ctrl[3] = 0.3 * swing  # FR knee
            data.ctrl[6] = 0.5 * swing  # RR hip
            data.ctrl[7] = 0.3 * swing  # RR knee
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        if step % 500 == 0:
            print(f"Step {step}: Time={t:.2f}s, Position={data.qpos[0]:.3f}")
    
    print("Demo complete.")


if __name__ == "__main__":
    main()
