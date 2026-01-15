import numpy as np
import os

# Add parent directory to path for imports
import sys
sys.path.append(os.path.dirname(__file__))

from mujoco_parser import MuJoCoParserClass
from tgg import get_q_from_ik

def main():
    # MuJoCo Niryo robot simulation
    # Get the directory of the current file
    current_dir = os.path.dirname(__file__)
    xml_path = os.path.join(current_dir, 'niryo_conveyor.xml')
    env = MuJoCoParserClass(name='Niryo', rel_xml_path=xml_path, VERBOSE=False)
    # env.print_info()

    env.forward()

    # Initialize MuJoCo viewer
    env.init_viewer(viewer_title="JPNP test", viewer_width=1600, viewer_height=900,
                    viewer_hide_menus=False)
    env.update_viewer()  # Remove cam_id since no cameras are defined
    env.reset()
    
    # NOTE: Niryo uses POSITION actuators (not torque like Panda)
    # Position actuators have built-in PID, so we don't need an external PID controller
    # We can directly set desired joint positions

    # Get IK solution
    pre_grasp_q, rotate_eef_q_lst = get_q_from_ik(env)

    # Define env max_tick
    max_tick    = 10000000
    
    # Define task
    task_sequnce_idx = 0
    task_sequnce = ["home",
                    "shoulder_pos_r",      # 1
                    "arm_elbow_pos_f",     # 2
                    "elbow_wrist_pos_u",   # 3
                    "wrist_rotate_pos_c",  # 4
                    "hand_down_pos",       # 5
                    "gripper_pos_close",   # 6
                    "gripper_pos_open",    # 7
                    "wrist_rotate_pos_cc", # 8
                    "elbow_wrist_pos_d",   # 9
                    "arm_elbow_pos_b",     # 10
                    "shoulder_pos_l",      # 11
                    "back_to_home"]
    previous_task = None  # Track previous task for printing
    current_task = "home"  # Initialize current task

    # Control loop
    while env.tick < max_tick:
        # Act every 10000 steps (20 seconds at 0.002s timestep)
        if env.tick % 10000 == 0:
            # Repeat task sequence
            if task_sequnce_idx >= len(task_sequnce):
                task_sequnce_idx = 0  # Loop back to start

            current_task = task_sequnce[task_sequnce_idx]
            task_sequnce_idx = task_sequnce_idx + 1
            
            # Print only when task changes
            if current_task != previous_task:
                print(f"[{env.tick}] current_task : {current_task}\t task_sequnce_idx : {task_sequnce_idx}")
                previous_task = current_task

        if current_task == "home":
            desired_q = pre_grasp_q

        elif current_task == "shoulder_pos_r":
            desired_q = rotate_eef_q_lst[0]

        elif current_task == "arm_elbow_pos_f":
            desired_q = rotate_eef_q_lst[1]

        elif current_task == "elbow_wrist_pos_u":
            desired_q = rotate_eef_q_lst[2]

        elif current_task == "wrist_rotate_pos_c":
            desired_q = rotate_eef_q_lst[3]

        elif current_task == "hand_down_pos":
            desired_q = rotate_eef_q_lst[4]

        elif current_task == "gripper_pos_close":
            desired_q = rotate_eef_q_lst[5]

        elif current_task == "gripper_pos_open":
            desired_q = rotate_eef_q_lst[6]

        elif current_task == "wrist_rotate_pos_cc":
            desired_q = rotate_eef_q_lst[7]

        elif current_task == "elbow_wrist_pos_d":
            desired_q = rotate_eef_q_lst[8]

        elif current_task == "arm_elbow_pos_b":
            desired_q = rotate_eef_q_lst[9]

        elif current_task == "shoulder_pos_l":
            desired_q = rotate_eef_q_lst[10]

        elif current_task == "back_to_home":
            desired_q = pre_grasp_q

        # With position actuators, directly set the desired joint positions
        # No PID needed - the actuators have built-in position control
        env.step(ctrl=desired_q)

        # Render
        if (env.tick % 3) == 0:
            env.render()

    env.close_viewer()
    print("Done")


if __name__ == "__main__":
    main()