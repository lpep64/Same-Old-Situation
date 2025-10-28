#!/usr/bin/env python3
"""
Panda Hanoi - Baseline Solver

This script orchestrates the solution to the Tower of Hanoi problem:
1.  Generates the optimal abstract move sequence using `hanoi_logic`.
2.  Loads the MuJoCo simulation environment.
3.  Initializes the professional `NiryoRobotController`.
4.  Commands the controller to execute each move in the sequence.
"""

import mujoco
import mujoco.viewer
import os
import sys

# Add parent directory to path to allow imports from sibling directories
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import project modules
from hanoi_logic import solve_hanoi
from niryo_robot_controller import NiryoRobotController

def main():
    """Main function to run the baseline solver."""
    try:
        # --- 1. Generate the Abstract Plan ---
        n_disks = 3
        solution = solve_hanoi(n_disks, 'A', 'C', 'B')
        print(f"Generated solution for {n_disks} disks: {solution}\n")

        # --- 2. Load the MuJoCo Environment ---
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        # Prefer the checked-in scene XML under project xmls/ directory.
        scene_xml_path = os.path.join(project_root, "xmls", "niryo_hanoi_cube_scene.xml")
        scene_xml_path = os.path.normpath(scene_xml_path)

        if not os.path.exists(scene_xml_path):
            print(f"❌ Scene XML not found at: {scene_xml_path}\nPlease ensure 'xmls/niryo_hanoi_cube_scene.xml' exists in the project.")
            return 1
        
        model = mujoco.MjModel.from_xml_path(scene_xml_path)
        data = mujoco.MjData(model)
        
        # --- 3. Initialize Controller and Execute Plan ---
        controller = NiryoRobotController(model, data, n_disks=n_disks)
        
        with mujoco.viewer.launch_passive(model, data) as viewer:

            #controller.load_execute_hanoi_moves(solution)

            while viewer.is_running() or data.time < 15.0:
                mujoco.mj_step(model, data)
                viewer.sync()

        return 0
        
    except Exception as e:
        print(f"\n❌ An error occurred: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit(main())