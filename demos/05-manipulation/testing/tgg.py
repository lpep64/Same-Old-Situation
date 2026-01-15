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
    Define simple test poses to check joint control

    Task sequence for testing:
        1. shoulder_pos_r - move right to test shoulder joint
        2. arm_elbow_pos_f - move forward to test arm/elbow joint
        3. elbow_wrist_pos_u - move up to test elbow/wrist joint
        4. wrist_rotate_pos_c - rotate wrist clockwise
        5. hand_down_pos - move hand down
        6. gripper_pos_close - close gripper
        7. gripper_pos_open - open gripper
        8. wrist_rotate_pos_cc - rotate wrist counter-clockwise
        9. elbow_wrist_pos_d - move elbow/wrist down
        10. arm_elbow_pos_b - move arm/elbow back
        11. shoulder_pos_l - move left to test shoulder joint
    """

    # Home position - robot in neutral, reachable pose
    # Robot base is at [-0.3, 0.0, 0.45], end-effector default at [-0.033, 0.0, 0.879]
    # Conveyor block is at [-0.25, 0.35, 0.595]
    # Use home keyframe position as pre-grasp
    pre_grasp_p = np.array([0.02, 0.0, 0.96])  # Near home keyframe end-effector position
    pre_grasp_R = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ])

    # Rotation matrices for wrist rotation tests
    from scipy.spatial.transform import Rotation as Rot
    R_wrist_cw = Rot.from_euler('z', 45, degrees=True).as_matrix()  # 45° clockwise
    R_wrist_ccw = Rot.from_euler('z', -45, degrees=True).as_matrix()  # 45° counter-clockwise

    # Test positions - each tests different joints with movements within robot workspace
    # Conveyor belt center is at approximately x=-0.1, y=0.35, z=0.595 (with block surface)
    # Robot can reach roughly x:[-0.3 to 0.1], y:[-0.3 to 0.3], z:[0.6 to 1.0]
    test_positions_p = [
        np.array([0.02, 0.1, 0.96]),      # 1. Move right (shoulder rotation)
        np.array([0.05, 0.1, 0.92]),      # 2. Move forward and down slightly
        np.array([0.05, 0.1, 0.95]),      # 3. Move up (elbow/wrist)
        np.array([0.05, 0.1, 0.95]),      # 4. Same position, rotate wrist clockwise
        np.array([0.05, 0.2, 0.75]),      # 5. Move toward conveyor (hand down)
        np.array([0.05, 0.2, 0.75]),      # 6. Same position, close gripper  
        np.array([0.05, 0.2, 0.75]),      # 7. Same position, open gripper
        np.array([0.05, 0.2, 0.75]),      # 8. Same position, rotate wrist counter-clockwise
        np.array([0.02, 0.1, 0.85]),      # 9. Move back and down
        np.array([0.02, 0.0, 0.90]),      # 10. Return toward center
        np.array([0.02, -0.1, 0.96]),     # 11. Move left (shoulder other direction)
    ]
    
    # Corresponding rotations for each position
    test_rotations_R = [
        pre_grasp_R.copy(),           # 1. Normal orientation
        pre_grasp_R.copy(),           # 2. Normal orientation
        pre_grasp_R.copy(),           # 3. Normal orientation
        pre_grasp_R @ R_wrist_cw,     # 4. Wrist rotated clockwise
        pre_grasp_R @ R_wrist_cw,     # 5. Keep wrist rotation
        pre_grasp_R @ R_wrist_cw,     # 6. Keep wrist rotation
        pre_grasp_R @ R_wrist_cw,     # 7. Keep wrist rotation
        pre_grasp_R @ R_wrist_ccw,    # 8. Wrist rotated counter-clockwise
        pre_grasp_R @ R_wrist_ccw,    # 9. Keep wrist rotation
        pre_grasp_R.copy(),           # 10. Back to normal orientation
        pre_grasp_R.copy(),           # 11. Normal orientation
    ]
    
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

    # Set gripper state for each test position
    gripper_states = [
        [0.01, 0.01],    # 1. Open (right +, left +)
        [0.01, 0.01],    # 2. Open
        [0.01, 0.01],    # 3. Open
        [0.01, 0.01],    # 4. Open
        [0.01, 0.01],    # 5. Open
        [-0.01, -0.01],  # 6. Close gripper (right -, left -)
        [0.01, 0.01],    # 7. Open gripper (right +, left +)
        [0.01, 0.01],    # 8. Open
        [0.01, 0.01],    # 9. Open
        [0.01, 0.01],    # 10. Open
        [0.01, 0.01],    # 11. Open
    ]

    for i, tmp in enumerate(rotate_eef_q_lst):
        tmp = np.concatenate([tmp, gripper_states[i]])
        rotate_eef_q_lst[i] = tmp

    env.reset()

    return pre_grasp_q, rotate_eef_q_lst