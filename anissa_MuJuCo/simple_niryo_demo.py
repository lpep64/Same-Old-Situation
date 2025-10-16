#!/usr/bin/env python3
"""
Simple scripted pick-and-place demo for the Niryo robot in MuJoCo.
The robot uses position actuators and runs a short sequence before
handing control back to the interactive viewer.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    """
    Main function to run the Niryo robot demo with position control.
    """
    print("Loading Simple Niryo Robot Demo...")
    
    # Load the MuJoCo model
    model_path = "niryo_demo_simple.xml"
    
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    
    print("Model loaded successfully!")
    print(f"Number of joints: {model.njnt}")
    print(f"Number of actuators: {model.nu}")
    
    # Scripted joint targets (including gripper sliders) for each phase
    phases = [
        {"name": "pre_grasp", "target": np.array([1.10, -1.05, 1.40, 0.45, -0.35, 0.15, 0.009, -0.009]), "duration": 2.0},
        {"name": "approach", "target": np.array([1.10, -1.18, 1.50, 0.65, -0.20, 0.10, 0.009, -0.009]), "duration": 1.5},
        {"name": "grasp", "target": np.array([1.10, -1.18, 1.50, 0.65, -0.20, 0.10, 0.0, 0.0]), "duration": 0.8},
        {"name": "lift", "target": np.array([1.00, -0.85, 1.25, 0.20, -0.45, 0.10, 0.0, 0.0]), "duration": 1.5},
        {"name": "swing_to_drop", "target": np.array([0.35, -0.80, 1.35, -0.05, -0.55, 0.05, 0.0, 0.0]), "duration": 1.6},
        {"name": "lower_place", "target": np.array([0.35, -0.98, 1.50, 0.10, -0.35, 0.0, 0.0, 0.0]), "duration": 1.2},
        {"name": "release", "target": np.array([0.35, -0.98, 1.50, 0.10, -0.35, 0.0, 0.009, -0.009]), "duration": 0.8},
        {"name": "retreat", "target": np.array([0.60, -0.70, 1.15, -0.10, -0.45, 0.10, 0.009, -0.009]), "duration": 1.4},
    ]

    initial_positions = phases[0]["target"]
    
    # Set initial positions
    for i, pos in enumerate(initial_positions):
        if i < len(data.qpos):
            data.qpos[i] = pos
    
    # Place the free block above the conveyor belt if present
    try:
        block_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "red_block_free")
        block_qpos_addr = model.jnt_qposadr[block_joint_id]
        # Absolute world position to start on the conveyor belt
        data.qpos[block_qpos_addr:block_qpos_addr + 3] = np.array([-0.20, 0.35, 0.60])
        data.qpos[block_qpos_addr + 3:block_qpos_addr + 7] = np.array([1.0, 0.0, 0.0, 0.0])
    except mujoco.FatalError:
        print("Warning: could not initialize red block joint; continuing without placement.")
    
    # Set control targets to initial positions
    data.ctrl[:] = initial_positions[:model.nu]
    
    # Forward kinematics to update the model state
    mujoco.mj_forward(model, data)
    
    print("\nStarting MuJoCo viewer...")
    print("Watch the scripted pick-and-place sequence, then take over with the control sliders.")
    print("Press ESC or close the window to exit.")
    
    def smoothstep(x: float) -> float:
        """Cubic smoothing for nicer joint motion."""
        x = np.clip(x, 0.0, 1.0)
        return x * x * (3.0 - 2.0 * x)
    
    with mujoco.viewer.launch(model, data) as viewer:
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.distance = 2.6
        viewer.cam.lookat[:] = [-0.05, 0.30, 0.55]
        
        phase_index = 0
        phase_step = 0
        phase_steps = 1
        phase_start_ctrl = data.ctrl.copy()
        dt = model.opt.timestep
        auto_demo = True
        
        print("\nRunning scripted sequence...")
        
        while viewer.is_running():
            if auto_demo and phase_index < len(phases):
                phase = phases[phase_index]
                if phase_step == 0:
                    phase_start_ctrl = data.ctrl[:model.nu].copy()
                    phase_steps = max(1, int(round(phase["duration"] / dt)))
                    print(f"  -> {phase['name']}")
                blend = smoothstep((phase_step + 1) / phase_steps)
                data.ctrl[:] = phase_start_ctrl + (phase["target"] - phase_start_ctrl) * blend
                phase_step += 1
                if phase_step >= phase_steps:
                    phase_index += 1
                    phase_step = 0
                    if phase_index >= len(phases):
                        auto_demo = False
                        print("Sequence finished. You can now use the MuJoCo control sliders.")
            
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(dt if auto_demo else 0.006)
    
    print("Demo completed.")

if __name__ == "__main__":
    main()
