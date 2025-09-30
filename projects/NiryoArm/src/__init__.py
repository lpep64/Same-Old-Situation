"""
NiryoArm MuJoCo Project

A comprehensive robotics simulation project demonstrating proper project structure
for MuJoCo-based robotics applications.
"""

__version__ = "0.1.0"
__author__ = "Your Name"
__email__ = "your.email@example.com"

from .environment import NiryoArmEnv
from .arm_controller import ArmController

__all__ = ["NiryoArmEnv", "ArmController"]