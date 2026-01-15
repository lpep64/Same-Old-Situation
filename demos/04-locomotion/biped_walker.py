#!/usr/bin/env python3
"""Biped Walker Gait Pattern Demo"""
import mujoco
import numpy as np

def create_biped_model():
    """Create a simple biped model."""
    xml = """
    <mujoco model="biped_walker">
        <option timestep="0.002"/>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
            <geom name="ground" type="plane" size="0 0 1" material="grid" rgba="0.9 0.9 0.9 0.1"/>
            
            <body name="torso" pos="0 0 1" mass="2">
                <joint name="torso_x" type="slide" axis="1 0 0"/>
                <joint name="torso_z" type="slide" axis="0 0 1" limited="true" range="0.5 1.5"/>
                <joint name="torso_y" type="hinge" axis="0 1 0"/>
                <geom name="torso" type="box" size="0.08 0.15 0.15" rgba="0.5 0.5 0.5 1"/>
                
                <!-- Left leg -->
                <body name="left_thigh" pos="0 0.1 -0.2">
                    <joint name="left_hip" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
                    <inertia mass="0.5" diaginv="1 1 1"/>
                    <geom name="left_thigh" type="capsule" size="0.04" fromto="0 0 0 0 0 -0.25" rgba="0.6 0.3 0.3 1"/>
                    
                    <body name="left_calf" pos="0 0 -0.25">
                        <joint name="left_knee" type="hinge" axis="0 1 0" range="0 2.0"/>
                        <inertia mass="0.3" diaginv="1 1 1"/>
                        <geom name="left_calf" type="capsule" size="0.035" fromto="0 0 0 0 0 -0.25" rgba="0.7 0.3 0.3 1"/>
                        
                        <body name="left_foot" pos="0 0 -0.25">
                            <geom name="left_foot" type="box" size="0.08 0.05 0.02" rgba="0.8 0.3 0.3 1"/>
                        </body>
                    </body>
                </body>
                
                <!-- Right leg -->
                <body name="right_thigh" pos="0 -0.1 -0.2">
                    <joint name="right_hip" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
                    <inertia mass="0.5" diaginv="1 1 1"/>
                    <geom name="right_thigh" type="capsule" size="0.04" fromto="0 0 0 0 0 -0.25" rgba="0.3 0.3 0.6 1"/>
                    
                    <body name="right_calf" pos="0 0 -0.25">
                        <joint name="right_knee" type="hinge" axis="0 1 0" range="0 2.0"/>
                        <inertia mass="0.3" diaginv="1 1 1"/>
                        <geom name="right_calf" type="capsule" size="0.035" fromto="0 0 0 0 0 -0.25" rgba="0.3 0.3 0.7 1"/>
                        
                        <body name="right_foot" pos="0 0 -0.25">
                            <geom name="right_foot" type="box" size="0.08 0.05 0.02" rgba="0.3 0.3 0.8 1"/>
                        </body>
                    </body>
                </body>
            </body>
        </worldbody>
        
        <actuator>
            <motor name="left_hip_motor" joint="left_hip" ctrlrange="-1 1"/>
            <motor name="left_knee_motor" joint="left_knee" ctrlrange="-1 1"/>
            <motor name="right_hip_motor" joint="right_hip" ctrlrange="-1 1"/>
            <motor name="right_knee_motor" joint="right_knee" ctrlrange="-1 1"/>
        </actuator>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


def main():
    """Run biped walking demonstration."""
    model = create_biped_model()
    data = mujoco.MjData(model)
    
    print("Biped Walker Gait Pattern Demo")
    print("-" * 50)
    print("This example demonstrates bipedal walking locomotion.")
    print("Target: Walk forward with alternating leg movements.")
    print("-" * 50)
    
    # Simulation parameters
    num_steps = 3000
    
    # Implement a basic walking gait
    for step in range(num_steps):
        t = step * 0.002  # Time in seconds
        phase = (t % 1.0)  # Gait cycle (period = 1 second)
        
        # Left leg (0-0.5s swing, 0.5-1s stance)
        if phase < 0.5:
            data.ctrl[0] = 0.5 * np.sin(2 * np.pi * phase)  # Left hip swing
            data.ctrl[1] = 0.3 * np.sin(2 * np.pi * phase)  # Left knee
        else:
            data.ctrl[0] = 0.2 * np.sin(2 * np.pi * phase)  # Left hip stance
            data.ctrl[1] = 0.1
        
        # Right leg (0.5-1s swing, 0-0.5s stance) - opposite of left
        if phase < 0.5:
            data.ctrl[2] = 0.2 * np.sin(2 * np.pi * phase)  # Right hip stance
            data.ctrl[3] = 0.1
        else:
            data.ctrl[2] = 0.5 * np.sin(2 * np.pi * (phase - 0.5))  # Right hip swing
            data.ctrl[3] = 0.3 * np.sin(2 * np.pi * (phase - 0.5))  # Right knee
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        if step % 500 == 0:
            print(f"Step {step}: Time={t:.2f}s, Position={data.qpos[0]:.3f}")
    
    print("Demo complete.")


if __name__ == "__main__":
    main()
