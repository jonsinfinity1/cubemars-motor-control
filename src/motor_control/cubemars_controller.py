#!/usr/bin/env python3
"""
CubeMars Motor Controller Adapter

Adapter class that makes CubeMarsMotor compatible with the MotorController interface.
This is the Adapter design pattern - it wraps an existing class to make it conform
to a new interface.
"""

from .motor_controller import MotorController
from motors.cubemars_motor import CubeMarsMotor


class CubeMarsController(MotorController):
    """
    Adapter that wraps CubeMarsMotor to implement MotorController interface.
    
    This allows CubeMarsMotor to be used interchangeably with other motor
    controllers (like T-Motor) through the common MotorController interface.
    
    When you add T-Motor support, you'd create a TMotorController class
    that implements the same interface.
    """
    
    def __init__(self, motor_id=1, can_channel='can0', bitrate=1000000):
        """
        Initialize CubeMars motor controller.
        
        Args:
            motor_id: Motor CAN ID (1-32)
            can_channel: CAN interface name (default: 'can0')
            bitrate: CAN bus bitrate (default: 1000000)
        """
        self.motor = CubeMarsMotor(motor_id, can_channel, bitrate)
        self.motor_id = motor_id
    
    def send_command(self, position, velocity, kp, kd, torque):
        """Send control command to the motor"""
        self.motor.send_command(position, velocity, kp, kd, torque)
    
    def read_feedback(self, timeout):
        """Read motor feedback from CAN bus"""
        return self.motor.read_feedback(timeout)
    
    def enter_motor_mode(self):
        """Enter motor control mode"""
        self.motor.enter_motor_mode()
    
    def exit_motor_mode(self):
        """Exit motor control mode"""
        self.motor.exit_motor_mode()
    
    def flush_can_buffer(self):
        """Clear any pending CAN messages"""
        self.motor.flush_can_buffer()
    
    def close(self):
        """Clean shutdown"""
        self.motor.close()
