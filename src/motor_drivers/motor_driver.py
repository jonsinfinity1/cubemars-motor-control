#!/usr/bin/env python3
"""
Abstract Motor Driver Interface

This module defines the abstract base class for all motor drivers.
Different motor types (CubeMars, T-Motor, Maxon, etc.) implement this interface.

Robotics Context:
-----------------
This is the "driver" layer - it handles electrical communication protocols
(CAN, UART, SPI, etc.) and motor-specific encoding/decoding.

Think of this as the device driver - it knows how to talk to the hardware
but doesn't understand high-level concepts like "joint range" or "trajectories".
"""

from abc import ABC, abstractmethod


class MotorDriver(ABC):
    """
    Abstract base class for low-level motor drivers.
    
    This defines the hardware communication interface. In Java terms, this
    is like an interface - it defines the contract that all motor drivers
    must implement.
    
    Each motor type (CubeMars AK-series, T-Motor, Maxon EPOS, etc.) will
    have its own driver class that implements these methods according to
    that motor's specific communication protocol.
    
    Example:
        class MyMotorDriver(MotorDriver):
            def send_command(self, position, velocity, kp, kd, torque):
                # Implement specific to my motor's protocol
                pass
            
            def read_feedback(self, timeout):
                # Implement specific to my motor's protocol
                pass
            
            # ... implement other abstract methods
    """
    
    @abstractmethod
    def send_command(self, position, velocity, kp, kd, torque):
        """
        Send a control command to the motor hardware.
        
        This uses impedance control - you specify both the desired state
        (position, velocity) AND how stiffly to achieve it (kp, kd).
        
        Robotics Note:
        - High kp = stiff/precise control (robot resists external forces)
        - Low kp = compliant/soft control (robot yields to external forces)
        
        Args:
            position: Desired position (radians)
            velocity: Desired velocity (rad/s)
            kp: Position gain / stiffness (motor-specific range)
            kd: Damping gain (motor-specific range)
            torque: Feed-forward torque (Nm)
        """
        pass
    
    @abstractmethod
    def read_feedback(self, timeout):
        """
        Read feedback from the motor hardware.
        
        This should return the motor's current state as measured by its
        internal sensors (typically encoder for position, estimated velocity
        and torque from current sensing).
        
        Args:
            timeout: Maximum time to wait for feedback (seconds)
            
        Returns:
            dict: {'position': float (rad),
                   'velocity': float (rad/s),
                   'torque': float (Nm),
                   'id': int}
            None: If timeout expires or read fails
        """
        pass
    
    @abstractmethod
    def enter_motor_mode(self):
        """
        Enable motor control mode.
        
        Most motors have a "safe" state where they're passive (zero torque)
        and an "active" state where they respond to control commands.
        This method transitions to the active state.
        
        Safety Note: Always call this before sending commands!
        """
        pass
    
    @abstractmethod
    def exit_motor_mode(self):
        """
        Disable motor control mode.
        
        Transitions motor back to passive/safe state with zero torque.
        The motor will go limp and can be moved freely.
        
        Safety Note: Always call this during shutdown!
        """
        pass
    
    @abstractmethod
    def flush_buffer(self):
        """
        Clear any pending messages from the communication buffer.
        
        Useful when starting up or recovering from errors to ensure you're
        reading fresh data, not stale messages from the buffer.
        """
        pass
    
    @abstractmethod
    def close(self):
        """
        Clean shutdown of the motor driver.
        
        Should:
        1. Exit motor mode (disable control)
        2. Close communication interfaces
        3. Release any resources
        
        This is like a destructor in C++ or Java's finalize().
        Always call this before program exit.
        """
        pass
