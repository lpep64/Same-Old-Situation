#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import time
from scipy.optimize import minimize

# ============================================================================
# GLOBAL CONFIGURATION
# ============================================================================
MAGNET_EE_BODY = "r_gripper_finger"
MAGNET_SITE = "end_effector"
MAGNET_OFFSET_LOCAL = np.array([0.0, -0.016, -0.01])
MAGNET_ROTATION_REL = None  # Initialized later

# World axis references (initialized in main)
WORLD_X = None
WORLD_Y = None
WORLD_Z = None
X_TARGET = None
Z_TARGET = None


# ============================================================================
# MATHEMATICAL UTILITIES - QUATERNIONS
# ============================================================================
def quat_from_axis_angle(axis, angle_rad):
    """Creates a quaternion from axis and angle."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / (np.linalg.norm(axis) + 1e-9)
    s = np.sin(angle_rad / 2.0)
    return np.array([np.cos(angle_rad / 2.0), axis[0]*s, axis[1]*s, axis[2]*s])


def quat_mul(q1, q2):
    """Multiplies two quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def quat_conj(q):
    """Quaternion conjugate."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def rotate_vec_by_quat(v, q):
    """Rotates a vector by a quaternion: v' = q * (0,v) * q*"""
    qv = np.array([0.0, v[0], v[1], v[2]])
    return quat_mul(quat_mul(q, qv), quat_conj(q))[1:]


# ============================================================================
# MAGNET/SUCTION SYSTEM (MAGNET STATE)
# ============================================================================
class MagnetState:
    """State of the gripper magnet/suction cup."""
    def __init__(self):
        self.on = False
        self.rel_offset_local = MAGNET_OFFSET_LOCAL.copy()


MAGNET = MagnetState()


def magnet_attach():
    """Activates the magnet."""
    MAGNET.on = True


def magnet_detach():
    """Deactivates the magnet."""
    MAGNET.on = False


def get_magnet_pose(model, data):
    """Gets pose (pos, quat) of the magnet reference body."""
    b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, MAGNET_EE_BODY)
    if b == -1:
        raise RuntimeError(f"Body '{MAGNET_EE_BODY}' does not exist in XML.")
    return data.xpos[b].copy(), data.xquat[b].copy()


def get_block_free_addrs(model):
    """Gets qpos/qvel addresses of the block's free joint."""
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "blue_block_free")
    if jid == -1:
        raise RuntimeError("Joint 'blue_block_free' does not exist in XML.")
    return model.jnt_qposadr[jid], model.jnt_dofadr[jid]


def magnet_follow_step(model, data, strength_pos=0.2, strength_quat=0.2):
    """
    Keeps the block attached to the right gripper finger via smooth interpolation.
    - Target position = gripper pose + local offset (rotated by gripper)
    - Target orientation = gripper_orientation * MAGNET_ROTATION_REL
    
    IMPORTANT: Only acts if magnet is activated (MAGNET.on == True)
    """
    if not MAGNET.on:
        return  # Do nothing if magnet is off

    qpos_addr, qvel_addr = get_block_free_addrs(model)

    # Reference pose (right gripper finger)
    pos_ref, quat_ref = get_magnet_pose(model, data)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, MAGNET_EE_BODY)

    # Local offset -> world using gripper orientation
    offset_world = rotate_vec_by_quat(MAGNET.rel_offset_local, quat_ref)
    target_pos = pos_ref + offset_world

    # Desired rotation = gripper * relative_rotation
    target_quat = quat_mul(quat_ref, MAGNET_ROTATION_REL)

    # Current block pose
    block_pos = data.qpos[qpos_addr:qpos_addr+3]
    block_quat = data.qpos[qpos_addr+3:qpos_addr+7]

    # Smooth interpolation (pos and quat)
    new_pos = block_pos + strength_pos * (target_pos - block_pos)
    new_quat = block_quat + strength_quat * (target_quat - block_quat)
    new_quat /= (np.linalg.norm(new_quat) + 1e-9)

    # Apply
    data.qpos[qpos_addr:qpos_addr+3] = new_pos
    data.qpos[qpos_addr+3:qpos_addr+7] = new_quat

    # CRITICAL: Match block velocity to gripper velocity (prevents bounce)
    # Gripper has fewer DOF than block (free joint = 6 DOF)
    # Copy linear and angular velocities from gripper to block
    gripper_vel_start = model.body_dofadr[body_id]
    if gripper_vel_start >= 0:
        # Determine how many DOF the gripper has
        gripper_dof_count = model.body_dofnum[body_id]
        if gripper_dof_count >= 6:
            gripper_vel = data.qvel[gripper_vel_start:gripper_vel_start+6]
        else:
            # If gripper has fewer DOF, pad with zeros
            gripper_vel = np.zeros(6)
            gripper_vel[:gripper_dof_count] = data.qvel[gripper_vel_start:gripper_vel_start+gripper_dof_count]
        data.qvel[qvel_addr:qvel_addr+6] = gripper_vel
    else:
        data.qvel[qvel_addr:qvel_addr+6] = 0.0


# ============================================================================
# GRIPPER ORIENTATION CONTROL
# ============================================================================
def orient_gripper_release(model, data):
    """
    Sets end effector orientation so that:
    - Z_local points downward (-Z world)
    - Y_local (gripper opening direction) aligns with +X world
    => Horizontal grippers, perpendicular to Z axis and aligned with conveyor
    """
    hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand_link")
    if hand_id == -1:
        return

    # Current end effector rotation matrix
    R = data.xmat[hand_id].reshape(3, 3)

    # Current local axes of end effector
    x_local = R[:, 0]
    y_local = R[:, 1]
    z_local = R[:, 2]

    # Targets
    z_target = np.array([0, 0, -1])  # points downward
    y_target = np.array([1, 0, 0])   # aligned with global X axis

    # Angular error (cross product)
    err_z = np.cross(z_local, z_target)
    err_y = np.cross(y_local, y_target)

    # Combined error (smooth)
    rot_error = 0.5 * (err_z + err_y)

    # Apply small correction (simulates torque)
    gain = 0.5
    qvel_addr = model.body_dofadr[hand_id]
    if qvel_addr >= 0:
        data.qvel[qvel_addr:qvel_addr+3] += gain * rot_error


# ============================================================================
# POSITION GETTERS
# ============================================================================
def get_end_effector_position(model, data):
    """Gets the position of the end_effector site."""
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    return data.site_xpos[site_id].copy()


def get_block_position(model, data):
    """Gets the current block position."""
    try:
        block_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "blue_block_free")
        block_qpos_addr = model.jnt_qposadr[block_joint_id]
        return data.qpos[block_qpos_addr:block_qpos_addr + 3].copy()
    except:
        return np.array([0, 0, 0])


def get_robot_joint_indices(model):
    """Gets qpos indices of robot joints."""
    joint_names = ["shoulder_joint", "arm_joint", "elbow_joint", 
                   "forearm_joint", "wrist_joint", "hand_joint"]
    indices = []
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos_addr = model.jnt_qposadr[joint_id]
        indices.append(qpos_addr)
    return np.array(indices)


# ============================================================================
# INVERSE KINEMATICS (IK)
# ============================================================================
def compute_grasp_target(model, data, block_pos, clearance=0.003):
    """
    Calculates a Cartesian target for IK that places the right gripper finger tip
    just above the block, with 'clearance' (m) gap.
    """
    # Right gripper finger body
    b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "r_gripper_finger")
    if b == -1:
        raise RuntimeError("'r_gripper_finger' does not exist in XML.")

    # Site currently used by IK (end_effector)
    s = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    if s == -1:
        raise RuntimeError("Site 'end_effector' does not exist in XML.")

    finger_pos = data.xpos[b].copy()
    ee_pos = data.site_xpos[s].copy()

    # Vector from current site to gripper tip (in world)
    ee_to_finger = finger_pos - ee_pos

    # Z axis of gripper finger body in world
    R = data.xmat[b].reshape(3, 3)
    z_axis_finger = R[:, 2]

    # Desired target: bring site to tip + clearance along -Z of finger
    target = block_pos - ee_to_finger + (-clearance) * z_axis_finger
    return target


def inverse_kinematics_optimization(
    model, data, target_pos, robot_joint_indices,
    initial_guess=None, use_orientation=False,
    y_target=None, z_target=None,
    w_pos=1.0, w_orient=0.3
):
    """
    IK with optimization and optional orientation constraints.
    Aligns the Y axis of the right finger with y_target and Z axis with z_target.
    """
    joint_names = ["shoulder_joint", "arm_joint", "elbow_joint",
                   "forearm_joint", "wrist_joint", "hand_joint"]
    
    # Get joint limits
    bounds = []
    for name in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        lo, hi = model.jnt_range[jid]
        bounds.append((lo + 0.05, hi - 0.05))

    # Save original state
    orig_qpos = data.qpos.copy()
    x0 = data.qpos[robot_joint_indices].copy() if initial_guess is None else initial_guess.copy()

    body_ref = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "r_gripper_finger")

    def objective(q):
        """Objective function for optimizer."""
        data.qpos[robot_joint_indices] = q
        mujoco.mj_forward(model, data)
        ee_pos = get_end_effector_position(model, data)
        e_pos = np.linalg.norm(ee_pos - target_pos)

        if not use_orientation:
            return e_pos * w_pos

        # Rotation matrix of right finger
        R = data.xmat[body_ref].reshape(3, 3)
        y_cur = R[:, 1]  # local Y axis of finger
        z_cur = R[:, 2]  # local Z axis of finger

        e_y = 1.0 - np.dot(y_cur, y_target)
        e_z = 1.0 - np.dot(z_cur, z_target)
        return w_pos * e_pos + w_orient * (e_y**2 + e_z**2)

    # Optimization
    res = minimize(objective, x0, method="SLSQP", bounds=bounds,
                   options={"maxiter": 300, "ftol": 1e-9})
    
    # Restore state and validate solution
    data.qpos[:] = orig_qpos
    data.qpos[robot_joint_indices] = res.x
    mujoco.mj_forward(model, data)
    pos_final = get_end_effector_position(model, data)
    pos_err = np.linalg.norm(pos_final - target_pos)
    data.qpos[:] = orig_qpos
    
    return (pos_err < 0.000015), pos_err, res.x


# ============================================================================
# MODEL INITIALIZATION
# ============================================================================
def initialize_model(model_path):
    """Loads the model and configures global references."""
    global WORLD_X, WORLD_Y, WORLD_Z, X_TARGET, Z_TARGET, MAGNET_ROTATION_REL
    
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Initialize world axes
    WORLD_X = np.array([1.0, 0.0, 0.0])
    WORLD_Y = np.array([0.0, 1.0, 0.0])
    WORLD_Z = np.array([0.0, 0.0, 1.0])
    X_TARGET = WORLD_X
    Z_TARGET = -WORLD_Z
    
    # Initialize magnet rotation
    MAGNET_ROTATION_REL = quat_from_axis_angle([0, 0, 1], np.deg2rad(90))
    
    return model, data


def setup_initial_configuration(model, data):
    """Configures initial position of robot and block."""
    # Initial joint positions
    initial_positions = np.array([0.0, -0.5, 1.0, 0.0, 0.0, 0.0, 0.010, 0.010])
    joint_order = ["shoulder_joint", "arm_joint", "elbow_joint", "forearm_joint", 
                   "wrist_joint", "hand_joint", "r_gripper_joint", "l_gripper_joint"]
    
    for idx, joint_name in enumerate(joint_order):
        try:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            qaddr = model.jnt_qposadr[joint_id]
            data.qpos[qaddr] = initial_positions[idx]
        except:
            pass
    
    # Position block on conveyor (without gravcomp, more stable)
    try:
        block_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "blue_block_free")
        block_qpos_addr = model.jnt_qposadr[block_joint_id]
        data.qpos[block_qpos_addr:block_qpos_addr + 3] = np.array([-0.25, 0.35, 0.595])
        data.qpos[block_qpos_addr + 3:block_qpos_addr + 7] = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Reset block velocities (important for initial stability)
        block_dofadr = model.jnt_dofadr[block_joint_id]
        data.qvel[block_dofadr:block_dofadr+6] = 0.0
        
        print("Blue block placed on conveyor belt")
    except:
        pass
    
    data.ctrl[:] = initial_positions[:model.nu]
    mujoco.mj_forward(model, data)


# ============================================================================
# TRAJECTORY DEFINITION
# ============================================================================
def create_trajectory(block_pos):
    """Creates waypoint sequence for pick and place."""
    offset = np.array([0, 0, 0])
    gripper_close = 0.006
    gripper_open = 0.010
    
    trajectory = [
        {"name": "home", 
         "position": np.array([-0.10, 0.20, 0.65]), 
         "gripper": gripper_open, 
         "duration": 3.0},
        
        {"name": "above_block", 
         "position": block_pos + np.array([0, 0, 0.08]), 
         "gripper": gripper_open, 
         "duration": 4.0},
        
        {"name": "descend_to_block", 
         "position": block_pos + offset + np.array([0, 0, 0.01]), 
         "gripper": gripper_open, 
         "duration": 3.0},
        
        {"name": "grasp", 
         "position": block_pos + offset + np.array([0, 0, 0.01]), 
         "gripper": gripper_close, 
         "duration": 2.0},
        
        {"name": "lift", 
         "position": block_pos + offset + np.array([0, 0, 0.12]), 
         "gripper": gripper_close, 
         "duration": 2.0},
        
        {"name": "move_to_place", 
         "position": np.array([0.00, 0.35, 0.70]), 
         "gripper": gripper_close, 
         "duration": 4.0},
        
        {"name": "lower_to_place", 
         "position": np.array([0.00, 0.35, 0.65]), 
         "gripper": gripper_close, 
         "duration": 3.0},
        
        {"name": "release", 
         "position": np.array([0.00, 0.35, 0.605]), 
         "gripper": gripper_close, 
         "duration": 2.0},
        
        {"name": "return_home", 
         "position": np.array([-0.10, 0.20, 0.605]), 
         "gripper": gripper_open, 
         "duration": 3.0},
    ]
    
    return trajectory


# ============================================================================
# WAYPOINT EXECUTION
# ============================================================================
def execute_waypoint(model, data, viewer, waypoint, idx, total, robot_joint_indices):
    """Executes a trajectory waypoint."""
    print(f"Phase {idx + 1}/{total}: {waypoint['name']}")
    print(f"  Target position: [{waypoint['position'][0]:.3f}, {waypoint['position'][1]:.3f}, {waypoint['position'][2]:.3f}]")
    print(f"  Computing IK... (this may take a moment)")
    
    # IK configuration
    current_joints = data.qpos[robot_joint_indices].copy()
    use_orient = waypoint['name'] in ("descend_to_block", "grasp", "lower_to_place", "release")
    
    success, error, target_joints = inverse_kinematics_optimization(
        model, data, waypoint['position'], robot_joint_indices, 
        initial_guess=current_joints, 
        use_orientation=use_orient,
        y_target=X_TARGET,
        z_target=Z_TARGET,
        w_pos=1.0,
        w_orient=0.3
    )
    
    if success:
        print(f"  ✓ IK converged! Error: {error*1000:.2f}mm")
    else:
        print(f"  ⚠ IK partial convergence. Error: {error*1000:.2f}mm")
    
    print(f"  Target joint angles: {np.round(target_joints, 4)}")
    
    # Execute interpolated movement
    dt = model.opt.timestep
    steps = int(waypoint['duration'] / dt)
    
    # Activate magnet BEFORE movement (if grasp)
    if waypoint['name'] == "grasp":
        # CRITICAL: Prepare block BEFORE activating magnet
        try:
            qpos_addr, qvel_addr = get_block_free_addrs(model)
            gripper_pos, gripper_quat = get_magnet_pose(model, data)
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, MAGNET_EE_BODY)
            
            # Position block near gripper
            offset_world = rotate_vec_by_quat(MAGNET_OFFSET_LOCAL, gripper_quat)
            target_block_pos = gripper_pos + offset_world
            data.qpos[qpos_addr:qpos_addr+3] = target_block_pos
            
            # Match velocities (PREVENTS BOUNCE)
            gripper_vel_start = model.body_dofadr[body_id]
            if gripper_vel_start >= 0:
                gripper_dof_count = model.body_dofnum[body_id]
                if gripper_dof_count >= 6:
                    data.qvel[qvel_addr:qvel_addr+6] = data.qvel[gripper_vel_start:gripper_vel_start+6]
                else:
                    # If gripper has fewer DOF, copy only available ones
                    gripper_vel = np.zeros(6)
                    gripper_vel[:gripper_dof_count] = data.qvel[gripper_vel_start:gripper_vel_start+gripper_dof_count]
                    data.qvel[qvel_addr:qvel_addr+6] = gripper_vel
            else:
                data.qvel[qvel_addr:qvel_addr+6] = 0.0
            
            # Forward kinematics to update
            mujoco.mj_forward(model, data)
            
            print("  Block pre-positioned for grasp")
        except Exception as e:
            print(f"  Warning: Could not pre-position block: {e}")
        
        magnet_attach()
        print("  ✓ Magnet ACTIVATED")
    
    # Smooth interpolation towards target
    for step in range(steps):
        alpha = (step + 1) / steps
        alpha_smooth = alpha * alpha * (3.0 - 2.0 * alpha)
        
        interpolated_joints = current_joints + alpha_smooth * (target_joints - current_joints)
        data.ctrl[0:6] = interpolated_joints
        data.ctrl[6] = waypoint['gripper']
        data.ctrl[7] = waypoint['gripper']
        
        # ONLY follow block if magnet is activated
        magnet_follow_step(model, data)
        
        # Only orient gripper during release
        if waypoint['name'] == "release":
            orient_gripper_release(model, data)
        
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(dt)
    
    # Settling time at target position
    data.ctrl[0:6] = target_joints
    settling_steps = int(1.5 / dt)
    for _ in range(settling_steps):
        # ONLY follow block if magnet is activated
        magnet_follow_step(model, data)
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(dt)
    
    # Deactivate magnet AFTER settling (if release)
    if waypoint['name'] == "release":
        print(f"  Current magnet state: {MAGNET.on}")
        magnet_detach()  # Deactivate magnet
        print("  ✓ Magnet DEACTIVATED")
        
        try:
            qpos_addr, qvel_addr = get_block_free_addrs(model)
            # Give small initial downward velocity
            data.qvel[qvel_addr:qvel_addr+3] = np.array([0.0, 0.0, -0.3])
            data.qvel[qvel_addr+3:qvel_addr+6] = np.array([0.0, 0.0, 0.0])
            print("  Block released with downward velocity")
            
            # Some extra steps for physics to act
            for _ in range(50):
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(dt)
                
        except Exception as e:
            print(f"  Warning: Could not set release velocity: {e}")
    
    # Accuracy report
    actual_ee_pos = get_end_effector_position(model, data)
    print(f"  Actual EE position: [{actual_ee_pos[0]:.3f}, {actual_ee_pos[1]:.3f}, {actual_ee_pos[2]:.3f}]")
    position_error = np.linalg.norm(actual_ee_pos - waypoint['position'])
    print(f"  Position error: {position_error*1000:.2f}mm")
    
    if waypoint['name'] in ["grasp", "lift", "release"]:
        current_block_pos = get_block_position(model, data)
        print(f"  Block position: [{current_block_pos[0]:.3f}, {current_block_pos[1]:.3f}, {current_block_pos[2]:.3f}]")
    
    print()


# ============================================================================
# MAIN FUNCTION
# ============================================================================
def main():
    print("Loading Niryo IK Demo (Optimization-based)...")
    model_path = "niryo_conveyor.xml"
    
    try:
        model, data = initialize_model(model_path)
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    
    print("Model loaded successfully!")
    
    # Initial configuration
    setup_initial_configuration(model, data)
    
    # Get robot and block information
    block_pos = get_block_position(model, data)
    print(f"Block position: [{block_pos[0]:.3f}, {block_pos[1]:.3f}, {block_pos[2]:.3f}]")
    
    robot_joint_indices = get_robot_joint_indices(model)
    print(f"Robot joint indices in qpos: {robot_joint_indices}")
    print(f"Initial robot joint angles: {data.qpos[robot_joint_indices]}")
    
    # Create trajectory
    trajectory = create_trajectory(block_pos)
    
    print("\nStarting IK-based pick and place demo (Optimization method)...")
    print("Working in Cartesian coordinates (x, y, z)\n")
    
    # Execute demo with viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Configure camera
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.distance = 2.6
        viewer.cam.lookat[:] = [-0.05, 0.30, 0.55]
        
        dt = model.opt.timestep
        
        # Execute each waypoint
        for idx, waypoint in enumerate(trajectory):
            execute_waypoint(model, data, viewer, waypoint, idx, len(trajectory), robot_joint_indices)
        
        # Final report
        final_block_pos = get_block_position(model, data)
        print(f"Pick and place complete!")
        print(f"Final block position: [{final_block_pos[0]:.3f}, {final_block_pos[1]:.3f}, {final_block_pos[2]:.3f}]")
        print("\nViewer will remain open. Press Ctrl+C or close window to exit.")
        
        # Keep viewer open
        try:
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\nDemo interrupted by user.")
    
    print("Demo completed.")


if __name__ == "__main__":
    main()