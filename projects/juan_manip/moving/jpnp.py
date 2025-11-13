import numpy as np
import os

# Add parent directory to path for imports
import sys
sys.path.append(os.path.dirname(__file__))

from mujoco_parser import MuJoCoParserClass
from jgg import get_q_from_ik
import time

def wait_for_convergence(env, target_q, max_ticks=15000, tolerance=0.01, render_interval=3, verbose=True):
    """
    Wait for robot to converge to target position.
    
    Args:
        env: MuJoCo environment
        target_q: Target joint positions (6 arm joints + 2 gripper joints)
        max_ticks: Maximum ticks to wait
        tolerance: Max difference in radians to consider converged
        render_interval: Render every N ticks
        verbose: Print progress updates
    
    Returns:
        converged: True if converged within max_ticks
    """
    start_tick = env.tick
    converged = False
    
    while (env.tick - start_tick) < max_ticks:
        # Get current joint positions - handle both arm joints (6) and gripper joints (2)
        num_arm_joints = len(env.rev_joint_qpos_idxs)
        if len(target_q) > num_arm_joints:
            # Includes gripper joints
            current_q = np.concatenate([
                env.data.qpos[env.rev_joint_qpos_idxs],
                env.data.qpos[env.rev_joint_qpos_idxs[-1]+1:env.rev_joint_qpos_idxs[-1]+1+len(target_q)-num_arm_joints]
            ])
        else:
            # Only arm joints
            current_q = env.data.qpos[env.rev_joint_qpos_idxs][:len(target_q)]
        
        # Calculate error
        error = np.abs(current_q - target_q)
        max_error = np.max(error)
        
        # Check convergence
        if max_error < tolerance:
            converged = True
            if verbose:
                print(f"  [OK] Converged! Error: {max_error:.6f} rad (took {env.tick - start_tick} ticks, {(env.tick - start_tick)*0.002:.2f}s)")
            break
        
        # Step simulation
        env.step(ctrl=target_q)
        
        # Render
        if (env.tick % render_interval) == 0:
            env.render()
        
        # Print progress every 2000 ticks
        if verbose and ((env.tick - start_tick) % 2000 == 0) and (env.tick - start_tick) > 0:
            print(f"  Progress: {env.tick - start_tick} ticks, max error: {max_error:.6f} rad")
    
    if not converged and verbose:
        num_arm_joints = len(env.rev_joint_qpos_idxs)
        if len(target_q) > num_arm_joints:
            current_q = np.concatenate([
                env.data.qpos[env.rev_joint_qpos_idxs],
                env.data.qpos[env.rev_joint_qpos_idxs[-1]+1:env.rev_joint_qpos_idxs[-1]+1+len(target_q)-num_arm_joints]
            ])
        else:
            current_q = env.data.qpos[env.rev_joint_qpos_idxs][:len(target_q)]
        max_error = np.max(np.abs(current_q - target_q))
        print(f"  [WARN] Did not fully converge after {max_ticks} ticks. Final error: {max_error:.6f} rad")
    
    return converged


def move_to_position(env, target_q, task_name, max_wait=15000, tolerance=0.01):
    """Move robot to target position and wait for convergence."""
    print(f"\n{'='*80}")
    print(f"[Tick {env.tick}] TASK: {task_name}")
    print(f"{'='*80}")
    
    # Get current joint positions - need to handle both arm joints (6) and gripper joints (2)
    # env.rev_joint_qpos_idxs gives arm joints, gripper joints come after
    num_arm_joints = len(env.rev_joint_qpos_idxs)
    if len(target_q) > num_arm_joints:
        # Includes gripper joints
        current_q = np.concatenate([
            env.data.qpos[env.rev_joint_qpos_idxs],
            env.data.qpos[env.rev_joint_qpos_idxs[-1]+1:env.rev_joint_qpos_idxs[-1]+1+len(target_q)-num_arm_joints]
        ])
    else:
        # Only arm joints
        current_q = env.data.qpos[env.rev_joint_qpos_idxs][:len(target_q)]
    
    print(f"  Current q: {current_q}")
    print(f"  Target q:  {target_q}")
    diff = np.abs(current_q - target_q)
    print(f"  Max diff:  {np.max(diff):.6f} rad")
    
    # Wait for convergence
    converged = wait_for_convergence(env, target_q, max_wait, tolerance, verbose=True)
    
    return converged

def main():
    print("\n" + "="*80)
    print("NIRYO PICK AND PLACE - BLOCK MANIPULATION")
    print("="*80 + "\n")
    
    # MuJoCo Niryo robot simulation
    # Get the XML path (file is in the same directory as this script)
    xml_path = os.path.join(os.path.dirname(__file__), 'niryo_conveyor.xml')
    env = MuJoCoParserClass(name='Niryo', rel_xml_path=xml_path, VERBOSE=False)
    # env.print_info()

    env.forward()

    # Initialize MuJoCo viewer
    env.init_viewer(viewer_title="Pick and Place", viewer_width=1600, viewer_height=900,
                    viewer_hide_menus=False)
    env.update_viewer()  # Remove cam_id since no cameras are defined
    env.reset()
    
    # NOTE: Niryo uses POSITION actuators (not torque like Panda)
    # Position actuators have built-in PID, so we don't need an external PID controller
    # We can directly set desired joint positions

    print("Calculating IK solutions for pick-and-place sequence...")
    
    # Get IK solution
    pre_grasp_q, rotate_eef_q_lst = get_q_from_ik(env)
    
    # Use home keyframe directly for starting position (no IK needed, known good position)
    # Home keyframe: [0, -0.5, 1.0, 0, 0, 0, 0, 0] (shoulder, arm, elbow, forearm, wrist, hand, r_gripper, l_gripper)
    home_q = np.array([0.0, -0.5, 1.0, 0.0, 0.0, 0.0, 0.01, 0.01])  # Use keyframe with open gripper
    
    # Print initial joint configurations
    print("\n" + "="*80)
    print("INITIAL JOINT CONFIGURATIONS")
    print("="*80)
    print(f"home_q (from keyframe): {home_q}")
    print(f"pre_grasp_q (from IK):  {pre_grasp_q}")
    print(f"\nNumber of positions in sequence: {len(rotate_eef_q_lst)}")
    for i, q in enumerate(rotate_eef_q_lst):
        print(f"Position[{i}]: {q}")
    print("="*80 + "\n")

    # Define SIMPLIFIED pick-and-place task sequence (11 positions)
    # START FROM ABOVE BLOCK directly (skip home to save time)
    task_sequence = [
        # Simplified sequence with raised positions to avoid extreme arm angles
        ('Move ABOVE BLOCK', rotate_eef_q_lst[0], 15000, 0.02),  # Approach
        ('Move to GRASP position', rotate_eef_q_lst[1], 15000, 0.02),  # Grasp height (raised)
        ('CLOSE GRIPPER', rotate_eef_q_lst[2], 8000, 0.01),  # Grab block
        ('LIFT with block', rotate_eef_q_lst[3], 15000, 0.02),  # Lift up
        ('TRANSPORT sideways', rotate_eef_q_lst[4], 15000, 0.02),  # Move to new location
        ('PLACE (high)', rotate_eef_q_lst[5], 15000, 0.02),  # Place (stays high - no lowering)
        ('OPEN GRIPPER', rotate_eef_q_lst[6], 8000, 0.01),  # Release
        ('LIFT after release', rotate_eef_q_lst[7], 15000, 0.02),  # Move up
        ('RETURN to block', rotate_eef_q_lst[8], 15000, 0.02),  # Return to grasp position
        ('CLOSE GRIPPER again', rotate_eef_q_lst[9], 8000, 0.01),  # Grab again
        ('LIFT with block', rotate_eef_q_lst[10], 15000, 0.02),  # Final lift
        ('Return to START', rotate_eef_q_lst[0], 15000, 0.02),  # Return to above block
    ]
    
    # Run pick and place loop
    max_cycles = 3  # Number of complete pick-place cycles
    cycle = 0
    
    print(f"\nStarting pick-and-place cycle (will run {max_cycles} times)...\n")
    
    while cycle < max_cycles:
        print(f"\n{'#'*80}")
        print(f"### CYCLE {cycle + 1} of {max_cycles}")
        print(f"{'#'*80}\n")
        
        # Execute sequence
        for task_name, target_q, max_wait, tolerance in task_sequence:
            converged = move_to_position(env, target_q, task_name, max_wait, tolerance)
            
            # Short pause between tasks for visibility
            for _ in range(50):
                env.step(ctrl=target_q)
                if env.tick % 3 == 0:
                    env.render()
        
        cycle += 1
        
        if cycle < max_cycles:
            print(f"\n{'='*80}")
            print(f"Cycle {cycle} complete! Starting next cycle...")
            print(f"{'='*80}\n")
    
    print(f"\n{'='*80}")
    print(f"ALL {max_cycles} CYCLES COMPLETE!")
    print(f"{'='*80}\n")
    
    # Keep viewer open for a moment
    print("Keeping viewer open for 5 seconds...")
    for _ in range(2500):  # 5 seconds at 0.002s timestep
        env.step(ctrl=target_q)
        if env.tick % 3 == 0:
            env.render()

    env.close_viewer()
    print("Done")


if __name__ == "__main__":
    main()