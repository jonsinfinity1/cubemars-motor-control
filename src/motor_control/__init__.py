"""
Motor Control Package

High-level control logic for robotic joints and behaviors.
This layer is hardware-agnostic - it works with any MotorDriver implementation.
"""

__version__ = '0.2.0'

from .joint import Joint
from .oscillation import FixedRangeOscillation

__all__ = ['Joint', 'FixedRangeOscillation']
