#!/usr/bin/env python3
"""
High-Level Motor Control with State Management

This module provides the Motor class which wraps low-level motor controllers
and provides high-level operations like smooth moves, range discovery, and
state tracking.
"""

import time
import math


class Motor:
    """
    High-level motor control with state management and common operations.
    
    This wraps a low-level MotorController and provides higher-level
    functionality like smooth moves, range discovery, and state tracking.
    
    Think of this as a service layer - it coordinates multiple operations
    and maintains state, while delegating actual hardware communication
    to the underlying motor controller.
    
    Example usage:
        controller = CubeMarsController(motor_id=1)
        motor = Motor(controller, name="left_knee", kp=20.0, kd=1.0)
        motor.initialize()
        motor.discover_range()
        motor.move_to(45.0, duration=2.0)
    """
    
    def __init__(self, motor_controller, motor_id=1, name=None, kp=20.0, kd=1.0):
        """
        Initialize the motor with a controller.
        
        Args:
            motor_controller: Instance implementing MotorController interface
            motor_id: Motor CAN ID
            name: Human-readable name (e.g., "left_knee", "right_elbow")
            kp: Default position gain (stiffness)
            kd: Default damping gain
            
        This is dependency injection - we pass in the controller rather than
        creating it here. Makes testing easier and supports different motor types.
        """
        self.controller = motor_controller
        self.motor_id = motor_id
        self.name = name or f"Motor_{motor_id}"
        self.default_kp = kp
        self.default_kd = kd
        
        # State tracking
        self.current_position = 0.0  # radians
        self.current_velocity = 0.0  # rad/s
        self.current_torque = 0.0    # Nm
        self.is_initialized = False
        
        # Discovered range (None until discovery is run)
        self.min_position = None  # radians
        self.max_position = None  # radians
        self.range_discovered = False
        
        # Safety limits
        self.safety_margin = math.radians(5)  # 5 degrees from hard stops
    
    def initialize(self):
        """
        Initialize the motor for operation.
        Call this before any other operations.
        """
        print(f"[{self.name}] Initializing...")
        self.controller.flush_can_buffer()
        self.controller.enter_motor_mode()
        time.sleep(0.5)
        
        # Read initial position
        self._update_state()
        self.is_initialized = True
        print(f"[{self.name}] Initialized at {math.degrees(self.current_position):.2f}°")
    
    def _update_state(self):
        """
        Update internal state from motor feedback.
        
        Leading underscore indicates this is "private" - meant for internal
        use only. It's a Python convention, not enforced like Java's private.
        """
        self.controller.send_command(
            position=self.current_position,
            velocity=0.0,
            kp=self.default_kp,
            kd=self.default_kd,
            torque=0.0
        )
        time.sleep(0.05)
        
        feedback = self.controller.read_feedback(timeout=0.1)
        if feedback:
            self.current_position = feedback['position']
            self.current_velocity = feedback.get('velocity', 0.0)
            self.current_torque = feedback.get('torque', 0.0)
            return True
        return False
    
    def get_position_degrees(self):
        """Get current position in degrees"""
        return math.degrees(self.current_position)
    
    def get_position_radians(self):
        """Get current position in radians"""
        return self.current_position
    
    def move_to(self, target_deg, duration=2.0, kp=None, kd=None, verbose=True):
        """
        Move smoothly to a target position using S-curve trajectory.
        
        The S-curve provides smooth acceleration and deceleration, which is
        important for preventing jerky motion and protecting mechanical components.
        
        Args:
            target_deg: Target position in degrees
            duration: Time to complete move (seconds)
            kp: Position gain (uses default if None)
            kd: Damping gain (uses default if None)
            verbose: Print progress updates
            
        Returns:
            bool: True if move completed successfully
        """
        if not self.is_initialized:
            raise RuntimeError(f"[{self.name}] Motor not initialized. Call initialize() first.")
        
        kp = kp or self.default_kp
        kd = kd or self.default_kd
        target_rad = math.radians(target_deg)
        
        if verbose:
            print(f"[{self.name}] Moving from {self.get_position_degrees():.2f}° "
                  f"to {target_deg:.2f}° over {duration:.1f}s")
        
        start_position = self.current_position
        start_time = time.time()
        
        while True:
            elapsed = time.time() - start_time
            
            if elapsed >= duration:
                position = target_rad
                done = True
            else:
                # S-curve trajectory: smooth acceleration and deceleration
                # The cosine function creates an S-shaped velocity profile
                progress = 0.5 - 0.5 * math.cos((elapsed / duration) * math.pi)
                position = start_position + (target_rad - start_position) * progress
                done = False
            
            self.controller.send_command(
                position=position,
                velocity=0.0,
                kp=kp,
                kd=kd,
                torque=0.0
            )
            
            feedback = self.controller.read_feedback(timeout=0.01)
            if feedback:
                self.current_position = feedback['position']
                self.current_velocity = feedback.get('velocity', 0.0)
                self.current_torque = feedback.get('torque', 0.0)
                
                if verbose and int(elapsed * 2) % 1 == 0:  # Print every 0.5s
                    print(f"  [{self.name}] Position: {self.get_position_degrees():6.2f}°")
            
            if done:
                break
            
            time.sleep(0.01)  # 100Hz control loop
        
        if verbose:
            print(f"[{self.name}] Reached {self.get_position_degrees():.2f}°")
        
        return True
    
    def discover_range(self, torque_threshold=0.5, creep_velocity=0.15, 
                       max_exploration_time=15.0, verbose=True):
        """
        Discover the full range of motion by exploring until hitting stops.
        
        This method slowly moves the joint in each direction until it detects
        a physical stop (via torque feedback), then backs off slightly and
        records the safe limits.
        
        Args:
            torque_threshold: Torque (Nm) indicating a hard stop
            creep_velocity: Exploration speed (rad/s)
            max_exploration_time: Safety timeout per direction (seconds)
            verbose: Print progress updates
            
        Returns:
            tuple: (min_position_deg, max_position_deg)
            
        This method updates the motor's internal range state.
        """
        if not self.is_initialized:
            raise RuntimeError(f"[{self.name}] Motor not initialized. Call initialize() first.")
        
        if verbose:
            print(f"\n[{self.name}] Starting range discovery...")
            print(f"  Torque threshold: {torque_threshold} Nm")
            print(f"  Exploration speed: {creep_velocity} rad/s")
        
        # Move to center first
        if verbose:
            print(f"[{self.name}] Moving to center (0°)...")
        self.move_to(0.0, duration=2.0, verbose=False)
        time.sleep(0.5)
        
        # Explore positive direction (max)
        if verbose:
            print(f"\n[{self.name}] Exploring positive direction...")
        max_pos = self._explore_direction(1, torque_threshold, creep_velocity, 
                                          max_exploration_time, verbose)
        time.sleep(1.0)
        
        # Return to center
        if verbose:
            print(f"\n[{self.name}] Returning to center...")
        self.move_to(0.0, duration=3.0, verbose=False)
        time.sleep(1.0)
        
        # Explore negative direction (min)
        if verbose:
            print(f"\n[{self.name}] Exploring negative direction...")
        min_pos = self._explore_direction(-1, torque_threshold, creep_velocity,
                                          max_exploration_time, verbose)
        time.sleep(1.0)
        
        # Store discovered range
        self.min_position = min_pos
        self.max_position = max_pos
        self.range_discovered = True
        
        min_deg = math.degrees(min_pos)
        max_deg = math.degrees(max_pos)
        
        if verbose:
            print(f"\n[{self.name}] Range discovery complete:")
            print(f"  Min: {min_deg:7.2f}°")
            print(f"  Max: {max_deg:7.2f}°")
            print(f"  Total range: {max_deg - min_deg:7.2f}°")
        
        return (min_deg, max_deg)
    
    def _explore_direction(self, direction, torque_threshold, creep_velocity,
                          max_exploration_time, verbose):
        """
        Explore in one direction until hitting a stop.
        
        Internal helper method for discover_range(). Uses very compliant
        control (low kp/kd) so the motor can "feel" the stop without
        fighting it.
        
        Args:
            direction: 1 for positive, -1 for negative
            torque_threshold: Torque indicating a stop
            creep_velocity: Slow exploration speed
            max_exploration_time: Safety timeout
            verbose: Print updates
            
        Returns:
            float: Position where stop was detected (radians)
        """
        start_time = time.time()
        stop_detected = False
        stop_position = None
        
        # Very compliant control for exploration - we want to feel the stop
        kp = 5.0   # Low stiffness
        kd = 0.5   # Light damping
        
        while not stop_detected and (time.time() - start_time) < max_exploration_time:
            # Command a slow velocity in the exploration direction
            self.controller.send_command(
                position=self.current_position + (direction * 0.1),
                velocity=direction * creep_velocity,
                kp=kp,
                kd=kd,
                torque=0.0
            )
            
            feedback = self.controller.read_feedback(timeout=0.02)
            
            if feedback:
                self.current_position = feedback['position']
                self.current_torque = feedback.get('torque', 0.0)
                
                torque_magnitude = abs(self.current_torque)
                
                # Check if we've hit a stop
                if torque_magnitude >= torque_threshold:
                    stop_detected = True
                    stop_position = self.current_position
                    if verbose:
                        print(f"  Stop detected at {self.get_position_degrees():.2f}° "
                              f"(Torque: {self.current_torque:.3f} Nm)")
                elif verbose and int((time.time() - start_time) * 4) % 1 == 0:
                    print(f"  Position: {self.get_position_degrees():6.2f}°  "
                          f"Torque: {self.current_torque:5.3f} Nm")
            
            time.sleep(0.05)  # 20Hz exploration (slow and gentle)
        
        if not stop_detected:
            if verbose:
                print(f"  Warning: No stop detected after {max_exploration_time}s")
            stop_position = self.current_position
        
        # Back off slightly from the hard stop for safety
        backoff = math.radians(2)  # 2 degrees
        safe_position = stop_position - (direction * backoff)
        
        if verbose:
            print(f"  Backing off 2° from stop...")
        
        # Use direct movement for quick backoff
        start_pos = self.current_position
        for i in range(20):
            progress = i / 20.0
            pos = start_pos + (safe_position - start_pos) * progress
            self.controller.send_command(pos, 0.0, 10.0, 0.8, 0.0)
            self.controller.read_feedback(timeout=0.01)
            time.sleep(0.05)
        
        self.current_position = safe_position
        return safe_position
    
    def get_safe_range(self):
        """
        Get the safe operating range (with safety margins from hard stops).
        
        Returns:
            tuple: (safe_min_deg, safe_max_deg, center_deg, range_deg)
            
        Raises:
            RuntimeError: If range hasn't been discovered yet
        """
        if not self.range_discovered:
            raise RuntimeError(
                f"[{self.name}] Range not discovered. Call discover_range() first."
            )
        
        safe_min_rad = self.min_position + self.safety_margin
        safe_max_rad = self.max_position - self.safety_margin
        
        safe_min_deg = math.degrees(safe_min_rad)
        safe_max_deg = math.degrees(safe_max_rad)
        center_deg = (safe_min_deg + safe_max_deg) / 2
        range_deg = safe_max_deg - safe_min_deg
        
        return (safe_min_deg, safe_max_deg, center_deg, range_deg)
    
    def hold_position(self, duration=None, kp=None, kd=None, print_interval=1.0):
        """
        Hold current position with impedance control.
        
        Args:
            duration: How long to hold (None = indefinite, until Ctrl+C)
            kp: Position gain (uses default if None)
            kd: Damping gain (uses default if None)
            print_interval: How often to print status (seconds)
        """
        if not self.is_initialized:
            raise RuntimeError(
                f"[{self.name}] Motor not initialized. Call initialize() first."
            )
        
        kp = kp or self.default_kp
        kd = kd or self.default_kd
        
        hold_position = self.current_position
        
        print(f"[{self.name}] Holding position at {self.get_position_degrees():.2f}°")
        if duration:
            print(f"  Duration: {duration:.1f}s")
        else:
            print(f"  Press Ctrl+C to stop")
        
        start_time = time.time()
        last_print = start_time
        
        try:
            while True:
                current_time = time.time()
                
                # Check if duration has elapsed
                if duration and (current_time - start_time) >= duration:
                    break
                
                self.controller.send_command(
                    position=hold_position,
                    velocity=0.0,
                    kp=kp,
                    kd=kd,
                    torque=0.0
                )
                
                feedback = self.controller.read_feedback(timeout=0.01)
                if feedback:
                    self.current_position = feedback['position']
                    self.current_torque = feedback.get('torque', 0.0)
                    
                    if (current_time - last_print) >= print_interval:
                        error_deg = math.degrees(hold_position - self.current_position)
                        print(f"  Position: {self.get_position_degrees():6.2f}°  "
                              f"Error: {error_deg:+5.2f}°  "
                              f"Torque: {self.current_torque:5.3f} Nm")
                        last_print = current_time
                
                time.sleep(0.01)  # 100Hz control loop
                
        except KeyboardInterrupt:
            print(f"\n[{self.name}] Hold interrupted")
    
    def shutdown(self):
        """Clean shutdown of the motor"""
        print(f"[{self.name}] Shutting down...")
        self.controller.close()
        self.is_initialized = False
