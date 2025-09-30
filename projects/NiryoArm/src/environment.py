"""
MuJoCo Environment for Niryo Arm Simulation

This module provides a Gymnasium-compatible environment for the Niryo robotic arm,
designed for reinforcement learning and robotics research.
"""

import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
from typing import Dict, Any, Tuple, Optional
from pathlib import Path


class NiryoArmEnv(gym.Env):
    """
    Niryo Arm MuJoCo Environment
    
    A Gymnasium environment for simulating the Niryo 6-DOF robotic arm
    with various manipulation tasks.
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}
    
    def __init__(
        self, 
        model_path: str = None,
        render_mode: Optional[str] = None,
        **kwargs
    ):
        """
        Initialize the Niryo Arm environment.
        
        Args:
            model_path: Path to the MuJoCo XML model file (auto-detected if None)
            render_mode: Rendering mode ("human" or "rgb_array")
            **kwargs: Additional environment parameters
        """
        super().__init__()
        
        # Auto-detect model path if not provided
        if model_path is None:
            current_dir = Path(__file__).parent.parent
            model_path = current_dir / "models" / "niryo_arm.xml"
            
        if not Path(model_path).exists():
            raise FileNotFoundError(f"Model file not found: {model_path}")
        
        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        
        # Environment parameters
        self.render_mode = render_mode
        self.n_joints = 6  # Niryo arm has 6 DOF
        
        # Define action and observation spaces
        # Actions: joint velocities or positions
        self.action_space = spaces.Box(
            low=-1.0, 
            high=1.0, 
            shape=(self.n_joints,), 
            dtype=np.float32
        )
        
        # Observations: joint positions, velocities, end-effector pose
        obs_dim = self.n_joints * 2 + 7  # positions + velocities + ee_pose
        self.observation_space = spaces.Box(
            low=-np.inf, 
            high=np.inf, 
            shape=(obs_dim,), 
            dtype=np.float32
        )
        
        # Initialize simulation state
        self.reset()
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        Execute one environment step.
        
        Args:
            action: Joint actions (velocities or positions)
            
        Returns:
            observation: Current state observation
            reward: Reward for this step
            terminated: Whether episode ended due to success/failure
            truncated: Whether episode ended due to time limit
            info: Additional information dictionary
        """
        # Apply action to simulation (placeholder)
        # self.data.ctrl[:] = action
        # mujoco.mj_step(self.model, self.data)
        
        # Get observation
        observation = self._get_observation()
        
        # Calculate reward
        reward = self._calculate_reward(action)
        
        # Check if episode is done
        terminated = self._is_terminated()
        truncated = self._is_truncated()
        
        # Additional info
        info = self._get_info()
        
        return observation, reward, terminated, truncated, info
    
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        """
        Reset the environment to initial state.
        
        Args:
            seed: Random seed for reproducibility
            options: Additional reset options
            
        Returns:
            observation: Initial observation
            info: Additional information dictionary
        """
        super().reset(seed=seed)
        
        # Reset simulation to initial state
        # mujoco.mj_resetData(self.model, self.data)
        
        # Set initial joint positions (placeholder)
        self.joint_positions = np.zeros(self.n_joints)
        self.joint_velocities = np.zeros(self.n_joints)
        
        observation = self._get_observation()
        info = self._get_info()
        
        return observation, info
    
    def render(self):
        """Render the environment."""
        if self.render_mode == "human":
            # Would open viewer window
            pass
        elif self.render_mode == "rgb_array":
            # Would return RGB image array
            return np.zeros((480, 640, 3), dtype=np.uint8)
    
    def close(self):
        """Clean up environment resources."""
        pass
    
    def _get_observation(self) -> np.ndarray:
        """Get current state observation."""
        # Placeholder observation
        obs = np.concatenate([
            self.joint_positions,      # Joint positions
            self.joint_velocities,     # Joint velocities  
            np.zeros(7)                # End-effector pose (pos + quat)
        ])
        return obs.astype(np.float32)
    
    def _calculate_reward(self, action: np.ndarray) -> float:
        """Calculate reward for current step."""
        # Placeholder reward function
        # In practice, would be task-specific (reaching, grasping, etc.)
        return 0.0
    
    def _is_terminated(self) -> bool:
        """Check if episode terminated (success/failure)."""
        return False
    
    def _is_truncated(self) -> bool:
        """Check if episode truncated (time limit)."""
        return False
    
    def _get_info(self) -> Dict[str, Any]:
        """Get additional information dictionary."""
        return {
            "joint_positions": self.joint_positions.copy(),
            "joint_velocities": self.joint_velocities.copy(),
        }