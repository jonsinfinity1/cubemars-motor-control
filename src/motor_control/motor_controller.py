#!/usr/bin/env python3
"""
Abstract Motor Controller Interface

This module defines the abstract base class for all motor controllers.
Different motor types (CubeMars, T-Motor, etc.) implement this interface.
"""

from abc import ABC, abstractmethod


class MotorController(ABC):
    """
    Abstract base class for motor controllers.
    
    This is similar to a Java interface - it defines the contract that all
    motor controllers must follow. Any class that extends this MUST implement
    all the abstract methods.
    
    This allows you to create different motor controller implementations
    (CubeMars, T-Motor, etc.) that all share the same interface, making it
    easy to swap motors without changing your high-level code.
    """
    
    @abstractmethod
    def send_command(self, position, velocity, kp, kd, torque):
        """
        Send a control command to the motor.
        
        Args:
            position: Desired position (radians)
            velocity: Desired velocity (rad/s)
            kp: Position gain (stiffness)
            kd: Damping gain
            torque: Feed-forward torque (Nm)
        """
        pass
    
    @abstractmethod
    def read_feedback(self, timeout):
        """
        Read feedback from the motor.
        
        Args:
            timeout: Maximum time to wait for feedback (seconds)
            
        Returns:
            dict with keys: 'position', 'velocity', 'torque', 'id'
            or None if no feedback received
        """
        pass
    
    @abstractmethod
    def enter_motor_mode(self):
        """Enter motor control mode"""
        pass
    
    @abstractmethod
    def exit_motor_mode(self):
        """Exit motor control mode"""
        pass
    
    @abstractmethod
    def flush_can_buffer(self):
        """Clear any pending CAN messages"""
        pass
    
    @abstractmethod
    def close(self):
        """Clean shutdown of the motor controller"""
        pass
