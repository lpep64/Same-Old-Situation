'''
Code fully based on https://github.com/sjchoi86/yet-another-mujoco-tutorial-v2
'''

import numpy as np
import mujoco

def r2w(R):
    r"""
        R to \omega
    """
    el = np.array([
            [R[2,1] - R[1,2]],
            [R[0,2] - R[2,0]], 
            [R[1,0] - R[0,1]]
        ])
    norm_el = np.linalg.norm(el)
    if norm_el > 1e-10:
        w = np.arctan2(norm_el, np.trace(R)-1) / norm_el * el
    elif R[0,0] > 0 and R[1,1] > 0 and R[2,2] > 0:
        w = np.array([[0, 0, 0]]).T
    else:
        w = np.pi/2 * np.array([[R[0,0]+1], [R[1,1]+1], [R[2,2]+1]])
    return w.flatten()


def trim_scale(x,th):
    """
        Trim scale
    """
    x         = np.copy(x)
    x_abs_max = np.abs(x).max()
    if x_abs_max > th:
        x = x*th/x_abs_max
    return x


def get_J_body(model,data,body_name,rev_joint_idxs=None):
    J_p = np.zeros((3,model.nv)) # nv: nDoF
    J_R = np.zeros((3,model.nv))
    mujoco.mj_jacBody(model,data,J_p,J_R,data.body(body_name).id)
    if rev_joint_idxs is not None:
        J_p = J_p[:,rev_joint_idxs]
        J_R = J_R[:,rev_joint_idxs]
    J_full = np.array(np.vstack([J_p,J_R]))
    return J_p,J_R,J_full


def solve_IK(env, max_tick, p_trgt, R_trgt, body_name,
             curr_q=None, is_render=False, VERBOSE=False, 
             prefer_large_changes=False, avoid_joint_limits=True):
    """
    Solve IK with joint limit avoidance and mid-range preference.
    
    Args:
        avoid_joint_limits: If True, adds penalties to keep joints away from limits
                          and prefer mid-range joint angles (especially for arm joint)
    """
    # IK in MJ with improved degenerate solution avoidance
    # Note: Use rev_joint_qpos_idxs for qpos addressing, rev_joint_idxs for Jacobian columns
    q_init = env.data.qpos[env.rev_joint_qpos_idxs] if curr_q is None else curr_q
    p_trgt = p_trgt
    R_trgt = R_trgt
    
    # Get joint limits from environment
    q_min = env.rev_joint_mins
    q_max = env.rev_joint_maxs
    
    # Calculate mid-range for each joint
    q_mid = (q_min + q_max) / 2.0
    
    # Special handling for arm joint (index 1) - keep it away from extreme negative values
    # Arm joint range: -1.8326 to 0.610167, problematic below -1.5
    arm_joint_idx = 1
    arm_safe_min = -1.4  # Keep arm joint above -1.4 rad to avoid singularities
    
    err_eps = 2e-2  # Slightly relaxed tolerance for faster convergence
    is_render = False
    
    # Try multiple initial configurations to avoid local minima
    best_q = None
    best_err = float('inf')
    best_joint_change = 0
    
    # Initial poses to try (prevents getting stuck in degenerate solutions)
    initial_poses = [q_init]
    if prefer_large_changes:
        # Add perturbed initial poses to encourage exploration
        for i in range(len(q_init)):
            q_perturb = q_init.copy()
            q_perturb[i] += 0.3  # 17 degrees perturbation
            # Ensure perturbation stays within limits
            q_perturb = np.clip(q_perturb, q_min, q_max)
            initial_poses.append(q_perturb)
    
    for attempt, q_start in enumerate(initial_poses):
        q = q_start.copy()
        env.data.qpos[env.rev_joint_qpos_idxs] = q
        env.forward()
        
        start_tick = env.tick
        iterations = 0
        max_iterations = max_tick - start_tick
        
        while iterations < max_iterations:
            J_p, J_R, J_full = get_J_body(env.model, env.data, body_name, rev_joint_idxs=env.rev_joint_dof_idxs)

            # Numerical IK
            p_curr = env.data.body(body_name).xpos
            R_curr = env.data.body(body_name).xmat.reshape([3, 3])
            p_err = (p_trgt - p_curr)
            R_err = np.linalg.solve(R_curr, R_trgt)
            w_err = R_curr @ r2w(R_err)

            # Compute dq with damped least squares
            J = J_full
            err = np.concatenate((p_err, w_err))
            eps = 1e-1
            
            # Build penalty terms for better joint configuration
            penalty = np.zeros_like(q)
            
            # Penalty 1: Prefer solutions with larger joint changes (avoid degeneracy)
            if prefer_large_changes:
                penalty += 0.01 * (q - q_init)
            
            # Penalty 2: Avoid joint limits and prefer mid-range positions
            if avoid_joint_limits:
                # Push joints toward mid-range (stronger near limits)
                for i in range(len(q)):
                    range_size = q_max[i] - q_min[i]
                    # Normalized distance from center (-1 to 1)
                    normalized_pos = 2 * (q[i] - q_mid[i]) / range_size
                    # Quadratic penalty increases near limits
                    penalty[i] += 0.05 * normalized_pos * abs(normalized_pos)
                
                # Penalty 3: Strong penalty for arm joint going too negative
                if q[arm_joint_idx] < arm_safe_min:
                    # Exponentially increasing penalty below safe threshold
                    penalty[arm_joint_idx] += 0.5 * (arm_safe_min - q[arm_joint_idx])
            
            # Solve with penalties
            dq = np.linalg.solve(a=(J.T@J) + eps*np.eye(J.shape[1]), 
                                b=J.T@err + penalty)
            
            dq = trim_scale(x=dq, th=5.0*np.pi/180.0)

            # Update
            q = q + dq
            
            # Clamp joints to their limits to prevent invalid solutions
            # Use stricter limit for arm joint to avoid extreme configurations
            q_min_safe = q_min.copy()
            q_min_safe[arm_joint_idx] = max(q_min[arm_joint_idx], arm_safe_min)
            q = np.clip(q, q_min_safe, q_max)
            
            env.data.qpos[env.rev_joint_qpos_idxs] = q

            # FK
            env.forward()
            
            if is_render:
                if (iterations % 5) == 0:
                    env.render()

            err_norm = np.linalg.norm(err)
            if VERBOSE: print(f"Attempt {attempt}, Iteration {iterations}, err_norm: {err_norm}")

            if err_norm < err_eps:
                # Calculate joint change magnitude
                joint_change = np.linalg.norm(q - q_init)
                
                # Store if this is the best solution
                if err_norm < best_err or (err_norm < err_eps and joint_change > best_joint_change):
                    best_q = q.copy()
                    best_err = err_norm
                    best_joint_change = joint_change
                    if VERBOSE:
                        print(f"IK solved - Attempt {attempt}, err: {err_norm:.4f}, joint change: {joint_change:.4f}")
                break
            
            iterations += 1
        
        # If we found a good solution on first try and not preferring large changes, stop
        if best_err < err_eps and not prefer_large_changes:
            break
    
    if best_q is not None:
        env.data.qpos[env.rev_joint_qpos_idxs] = best_q
        env.forward()
        if VERBOSE:
            print(f"Best IK solution: err={best_err:.4f}, joint_change={best_joint_change:.4f}")
            print(f"Joint angles: {best_q}")
    else:
        print(f"Warning: IK did not converge. Using last attempt.")
        best_q = q
            
    env.reset()
    return best_q
