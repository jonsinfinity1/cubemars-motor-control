"""
Motor Control Library

Simple motor control for robotics projects.
"""

__version__ = '0.1.0'

# Use relative imports (. means current package)
from .motor_controller import MotorController
from .cubemars_controller import CubeMarsController
from .fixed_range_oscillation import FixedRangeOscillation

__all__ = ['MotorController', 'CubeMarsController', 'FixedRangeOscillation']