import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mujoco
from mujoco import MjModel, MjData, viewer
import time
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

XML_MODEL_PATH = "models/franka_fr3/scene.xml"

class CustomRobotArmEnv(gym.Env):
    metadata = {'render_modes': ['human']}

    def __init__(self):
        super().__init__()
        self.model = MjModel.from_xml_path(XML_MODEL_PATH)
        self.data = MjData(self.model)

        # Control 7 joints (base to gripper)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(7,), dtype=np.float32)
        # Observation: positions of gripper, circle block, x block, and triangle block.
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)

        self.max_steps = 500
        self.current_step = 0
        self.circle_picked = False
        self.x_picked = False
        self.triangle_picked = False
        self.conveyor_speed = 0.1  # Speed of conveyor belt
        self.block_reset_pos = -0.8  # Position to reset blocks
        self.block_recycle_pos = 0.8  # Position to recycle blocks

        # Initialize joint positions
        self.home_position = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
        self.joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", 
            "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
        ]
        self.joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.joint_names]

        # Initialize the arm to home position
        for i in range(7):
            self.data.ctrl[i] = self.home_position[i]

    def _get_obs(self):
        gripper_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        circle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "circle_block")
        x_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "x_block")
        triangle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "triangle_block")

        gripper_pos = self.data.xpos[gripper_id].copy()
        circle_pos = self.data.xpos[circle_id].copy()
        x_pos = self.data.xpos[x_id].copy()
        triangle_pos = self.data.xpos[triangle_id].copy()

        return np.concatenate([gripper_pos, circle_pos, x_pos, triangle_pos]).astype(np.float32)

    def _reset_block_position(self, block_id):
        # Reset block to starting position
        self.data.qpos[block_id] = self.block_reset_pos
        self.data.qvel[block_id] = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        
        # Reset conveyor and blocks
        conveyor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "conveyor_joint")
        circle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "circle_joint")
        x_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "x_joint")
        triangle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "triangle_joint")
        
        # Set initial positions
        self._reset_block_position(circle_id)
        self._reset_block_position(x_id)
        self._reset_block_position(triangle_id)
        
        # Set conveyor velocity
        self.data.ctrl[conveyor_id] = self.conveyor_speed
        
        mujoco.mj_forward(self.model, self.data)
        return self._get_obs(), {}

    def step(self, action):
        # Apply robot action
        for i in range(len(action)):
            self.data.ctrl[i] = action[i] * 0.05

        # Check and recycle blocks that have gone too far
        circle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "circle_joint")
        x_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "x_joint")
        triangle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "triangle_joint")

        # Check each block position and reset if needed
        for block_id in [circle_id, x_id, triangle_id]:
            if self.data.qpos[block_id] > self.block_recycle_pos:
                self._reset_block_position(block_id)

        mujoco.mj_step(self.model, self.data)
        self.current_step += 1

        reward = 0.0
        threshold = 0.1

        gripper_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        gripper_pos = self.data.xpos[gripper_id].copy()

        # Circle block: +1
        circle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "circle_block")
        circle_pos = self.data.xpos[circle_id].copy()
        if not self.circle_picked and np.linalg.norm(gripper_pos - circle_pos) < threshold:
            reward += 1.0
            self.circle_picked = True
            print("Circle block picked up! +1 reward")

        # X block: -1
        x_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "x_block")
        x_pos = self.data.xpos[x_id].copy()
        if not self.x_picked and np.linalg.norm(gripper_pos - x_pos) < threshold:
            reward -= 1.0
            self.x_picked = True
            print("X block picked up! -1 penalty")

        # Triangle block: 0
        triangle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "triangle_block")
        triangle_pos = self.data.xpos[triangle_id].copy()
        if not self.triangle_picked and np.linalg.norm(gripper_pos - triangle_pos) < threshold:
            reward += 0.0
            self.triangle_picked = True
            print("Triangle block picked up! 0 reward")

        terminated = self.current_step >= self.max_steps
        truncated = False
        return self._get_obs(), reward, terminated, truncated, {}

    def render(self, mode="human"):
        pass

    def close(self):
        pass

    def move_to_target(self, target_pos, steps=100):
        """Move the gripper towards a target position"""
        gripper_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "fr3_link7")
        
        for _ in range(steps):
            current_pos = self.data.xpos[gripper_id].copy()
            # Simple proportional control
            direction = target_pos - current_pos
            
            # Convert direction to joint velocities (simplified)
            for i in range(7):
                self.data.ctrl[i] += direction[i] * 0.01 if i < 3 else 0
            
            mujoco.mj_step(self.model, self.data)
            yield

    def grip(self, close=True):
        """Control gripper"""
        gripper_value = 0.04 if close else 0.0
        self.data.ctrl[7] = gripper_value  # Assuming joint 7 is the gripper

    def pick_and_place(self, pickup_pos, place_pos):
        """Execute a pick and place sequence"""
        # Move above pickup
        yield from self.move_to_target(pickup_pos + np.array([0, 0, 0.1]))
        # Move down
        yield from self.move_to_target(pickup_pos)
        # Close gripper
        self.grip(close=True)
        # Move up
        yield from self.move_to_target(pickup_pos + np.array([0, 0, 0.1]))
        # Move to place position
        yield from self.move_to_target(place_pos + np.array([0, 0, 0.1]))
        # Move down
        yield from self.move_to_target(place_pos)
        # Open gripper
        self.grip(close=False)

    def set_joint_angles(self, angles):
        """Set joint angles directly"""
        for i, joint_id in enumerate(self.joint_ids):
            self.data.ctrl[i] = angles[i]

def main():
    # Initialize environment
    env = CustomRobotArmEnv()
    
    # Launch viewer first
    vis = viewer.launch(env.model, env.data)
    time.sleep(1)  # Give the viewer time to initialize
    
    # Define some joint angle configurations
    joint_positions = [
        # Home position
        np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853]),
        # Wave motion
        np.array([0.5, 0.5, 0, -1.57079, 0, 1.57079, -0.7853]),
        np.array([-0.5, -0.5, 0, -1.57079, 0, 1.57079, -0.7853]),
        # Reach forward
        np.array([0, -0.5, 0, -2.0, 0, 1.57079, -0.7853]),
        # Reach up
        np.array([0, 0.5, 0, -1.0, 0, 1.57079, -0.7853]),
    ]
    
    try:
        while vis.is_running():
            # Cycle through joint positions
            for target_joints in joint_positions:
                print(f"Moving to joint configuration: {target_joints}")
                
                # Interpolate to target joint positions
                current_joints = env.data.qpos[env.joint_ids].copy()
                steps = 100
                
                for step in range(steps):
                    # Linearly interpolate between current and target positions
                    alpha = step / steps
                    intermediate_joints = (1 - alpha) * current_joints + alpha * target_joints
                    
                    # Apply joint positions
                    env.set_joint_angles(intermediate_joints)
                    
                    # Step the simulation
                    mujoco.mj_step(env.model, env.data)
                    
                    # Render and wait
                    vis.render()
                    time.sleep(0.01)
                
                # Pause at each position
                print("Holding position")
                for _ in range(50):
                    # Keep stepping the simulation during pause
                    mujoco.mj_step(env.model, env.data)
                    vis.render()
                    time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
    finally:
        vis.close()

if __name__ == "__main__":
    main()
