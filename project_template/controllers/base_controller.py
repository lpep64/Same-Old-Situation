"""
Base controller interface for MuJoCo projects.

This module provides a standardized interface for implementing
robot controllers in MuJoCo simulations.
"""

import numpy as np
from abc import ABC, abstractmethod
import mujoco


class BaseController(ABC):
    """Abstract base class for all controllers."""
    
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        """
        Initialize controller with MuJoCo model and data.
        
        Args:
            model: MuJoCo model
            data: MuJoCo data
        """
        self.model = model
        self.data = data
        self.control_dim = model.nu
        
    @abstractmethod
    def compute_control(self, target_state: np.ndarray) -> np.ndarray:
        """
        Compute control action for given target state.
        
        Args:
            target_state: Desired state (implementation-specific format)
            
        Returns:
            Control action vector of shape (control_dim,)
        """
        pass
    
    @abstractmethod
    def reset(self):
        """Reset controller internal state."""
        pass


class PositionController(BaseController):
    """Simple position controller example."""
    
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, kp: float = 100.0):
        """
        Initialize position controller.
        
        Args:
            model: MuJoCo model
            data: MuJoCo data
            kp: Proportional gain
        """
        super().__init__(model, data)
        self.kp = kp
        
    def compute_control(self, target_positions: np.ndarray) -> np.ndarray:
        """
        Compute position control.
        
        Args:
            target_positions: Target joint positions
            
        Returns:
            Control torques
        """
        current_positions = self.data.qpos[:len(target_positions)]
        position_error = target_positions - current_positions
        return self.kp * position_error
    
    def reset(self):
        """Reset controller (no internal state for this simple controller)."""
        pass