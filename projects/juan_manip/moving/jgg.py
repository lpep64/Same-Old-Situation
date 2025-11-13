import numpy as np
import mujoco
import os
import sys
from scipy.spatial.transform import Rotation as R

# Add current directory to path for imports
sys.path.append(os.path.dirname(__file__))

from ik_module import solve_IK


def get_q_from_ik(env):
    body_name = 'hand_link'  # Changed from 'panda_eef' for Niryo robot

    """
    Pick and Place task sequence:
        0. home - Safe starting position
        1. above_block - Move above the block (approach)
        2. at_block - Move down to block (grasp position)
        3. close_gripper - Close gripper to grab block
        4. lift - Lift block up
        5. move_sideways - Move block to new location
        6. lower_place - Lower to place position
        7. open_gripper - Open gripper to release block
        8. lift_after_place - Lift after placing
        9. move_down_again - Move down to block again (for return)
        10. close_gripper_2 - Close gripper again
        11. lift_2 - Lift block again
        12. move_sideways_back - Move back to original position
        13. lower_original - Lower to original position
        14. open_gripper_2 - Open gripper
        15. lift_final - Lift after releasing
    """

    # Block position on conveyor: approximately [-0.25, 0.35, 0.595]
    # Robot base at: [-0.3, 0.0, 0.45]
    block_pos = np.array([-0.25, 0.35, 0.595])
    
    # SIMPLIFIED PATH - Raised heights to avoid extreme arm joint angles
    # The arm joint struggles with angles below -1.5 rad, so we keep movements higher
    
    # Home position - robot in neutral, reachable pose
    pre_grasp_p = np.array([0.02, 0.0, 0.96])  # Near home keyframe end-effector position
    pre_grasp_R = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ])

    # SIMPLIFIED Pick and place positions - FEWER waypoints, HIGHER positions
    # Position 1: Above block (approach position, 20cm above block instead of 15cm)
    above_block_p = block_pos + np.array([0.0, 0.0, 0.20])
    
    # Position 2: At block (grasp position, 5cm above block surface - higher than before)
    at_block_p = block_pos + np.array([0.0, 0.0, 0.05])
    
    # Position 3: Lift position (15cm above grasp - higher lift)
    lift_p = at_block_p + np.array([0.0, 0.0, 0.15])
    
    # Position 4: Move sideways (20cm in -Y direction, keep same height)
    move_sideways_p = lift_p + np.array([0.0, -0.20, 0.0])
    
    # Position 5: Place position (same height as lift - NO lowering to avoid extreme angles)
    # Instead of lowering, we place at a higher position
    place_p = move_sideways_p  # Keep at same height as lift
    
    # Position 5: Place position (same height as lift - NO lowering to avoid extreme angles)
    # Instead of lowering, we place at a higher position
    place_p = move_sideways_p  # Keep at same height as lift
    
    # SIMPLIFIED SEQUENCE - Reduced from 15 to 11 waypoints
    # Removed redundant "wait" and "move down again" positions that caused issues
    test_positions_p = [
        above_block_p,           # 1. Above block (approach)
        at_block_p,              # 2. At block (grasp position - raised)
        at_block_p,              # 3. Close gripper (same position)
        lift_p,                  # 4. Lift with block
        move_sideways_p,         # 5. Move sideways (transport)
        place_p,                 # 6. Place position (HIGH - no lowering)
        place_p,                 # 7. Open gripper (release)
        lift_p,                  # 8. Lift after release
        at_block_p,              # 9. Return to grasp position
        at_block_p,              # 10. Close gripper again
        lift_p,                  # 11. Lift with block again
    ]
    
    # All positions use same downward-facing orientation
    test_rotations_R = [pre_grasp_R.copy() for _ in range(len(test_positions_p))]
    
    rotate_eef_p_lst = test_positions_p
    rotate_eef_R_lst = test_rotations_R

    # IMPROVED IK - Use IK solver with better degeneracy avoidance
    # This will try multiple initial poses and prefer solutions that use more joints
    
    print("Using improved IK solver with degeneracy avoidance...")
    
    # Get home position from environment's current state after reset
    # Use the home keyframe which is a good starting configuration
    env.reset()
    # Set to home keyframe position - this is a known good configuration
    home_q = np.array([0.0, -0.5, 1.0, 0.0, 0.0, 0.0])
    env.data.qpos[env.rev_joint_qpos_idxs] = home_q
    env.forward()
    print(f"Initial robot joint positions (home keyframe): {home_q}")
    print(f"Joint names: {env.rev_joint_names}")
    print(f"End-effector position: {env.data.body('hand_link').xpos}")
    
    # Solve IK for home position first
    pre_grasp_q = solve_IK(env, 5000, pre_grasp_p, pre_grasp_R, body_name, 
                          curr_q=home_q, VERBOSE=False, prefer_large_changes=False)
    
    print(f"Home position IK: {pre_grasp_q}")
    
    # Solve IK for each test position
    rotate_eef_q_lst = []
    for i, (p, R) in enumerate(zip(rotate_eef_p_lst, rotate_eef_R_lst)):
        # Use previous solution as starting point for better continuity
        start_q = rotate_eef_q_lst[-1] if len(rotate_eef_q_lst) > 0 else pre_grasp_q
        
        # For positions that should show more joint movement, use prefer_large_changes
        # This helps avoid the wrist-only solutions we were getting before
        prefer_large = (i in [0, 1, 2, 8, 9, 10])  # Positions that should move the whole arm
        
        q = solve_IK(env, 5000, p, R, body_name, 
                    curr_q=start_q, VERBOSE=False, prefer_large_changes=prefer_large)
        
        rotate_eef_q_lst.append(q)
        print(f"Test position {i+1}: {q}")
        
        # Show joint changes from previous
        if len(rotate_eef_q_lst) > 1:
            joint_diff = q - rotate_eef_q_lst[-2]
            print(f"  Joint changes: {joint_diff}")
            print(f"  Total change magnitude: {np.linalg.norm(joint_diff):.4f} rad")
    
    # Add gripper joint (Niryo uses slide joints, not rotation)
    # Right gripper: positive=open, negative=close
    # Left gripper: negative=open, positive=close (opposite direction)
    pre_grasp_q = np.concatenate([pre_grasp_q, [0.01, 0.01]])  # Open gripper (both positive)

    # SIMPLIFIED gripper states for 11-position sequence
    gripper_states = [
        [0.01, 0.01],    # 1. Above block - Open
        [0.01, 0.01],    # 2. At block - Open
        [-0.01, -0.01],  # 3. Close gripper to grab
        [-0.01, -0.01],  # 4. Lift - Keep closed
        [-0.01, -0.01],  # 5. Move sideways - Keep closed
        [-0.01, -0.01],  # 6. Place - Keep closed (HIGH position)
        [0.01, 0.01],    # 7. Open gripper to release
        [0.01, 0.01],    # 8. Lift after release - Open
        [0.01, 0.01],    # 9. Return to grasp - Open
        [-0.01, -0.01],  # 10. Close gripper to grab again
        [-0.01, -0.01],  # 11. Lift with block - Keep closed
    ]

    for i, tmp in enumerate(rotate_eef_q_lst):
        tmp = np.concatenate([tmp, gripper_states[i]])
        rotate_eef_q_lst[i] = tmp

    env.reset()

    return pre_grasp_q, rotate_eef_q_lst