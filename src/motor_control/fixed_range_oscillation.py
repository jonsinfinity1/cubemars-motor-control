#!/usr/bin/env python3
"""
Fixed Range Oscillation

Provides smooth, controlled oscillation within specified limits using
sinusoidal motion profiles with S-curve blending.
"""

import time
import math


class FixedRangeOscillation:
    """
    Manages smooth sinusoidal oscillation within a defined range.
    
    This class encapsulates all the oscillation logic, making it reusable
    across different joints in your humanoid robot. You can create multiple
    instances for different joints, each with their own oscillation parameters.
    
    Example usage:
        oscillator = FixedRangeOscillation(motor_controller, kp=20.0, kd=1.0)
        oscillator.oscillate(
            center_position_deg=0,
            amplitude_deg=30,
            frequency=0.6,
            duration=10
        )
    """
    
    def __init__(self, motor_controller, kp=20.0, kd=1.0):
        """
        Initialize the oscillation controller.
        
        Args:
            motor_controller: Motor controller instance (CubeMarsMotor or similar)
            kp: Position gain for impedance control (stiffness)
            kd: Damping gain for impedance control
            
        Note: This stores a reference to the motor controller (composition).
        The oscillator "has-a" motor rather than "is-a" motor.
        """
        self.motor = motor_controller
        self.kp = kp
        self.kd = kd
        
        # Current state tracking
        self.current_position = 0.0
        self.is_oscillating = False
    
    def smooth_move_to_position(self, target_position, duration=2.0, verbose=True):
        """
        Move smoothly to a target position using S-curve trajectory.
        
        This is a public helper method that can be used standalone for
        smooth position changes. In Java you might mark this private,
        but in Python it's useful to keep it public.
        
        Args:
            target_position: Target position in radians
            duration: Time to complete the move (seconds)
            verbose: Whether to print progress updates
        """
        # Get current actual position from motor
        self.motor.send_command(
            position=self.current_position, 
            velocity=0.0,
            kp=self.kp, 
            kd=self.kd, 
            torque=0.0
        )
        time.sleep(0.05)
        
        feedback = self.motor.read_feedback(timeout=0.1)
        if feedback:
            start_position = feedback['position']
        else:
            start_position = self.current_position
            if verbose:
                print("Warning: Could not read position, using last known position")
        
        if verbose:
            print(f"Moving from {math.degrees(start_position):.2f}° to "
                  f"{math.degrees(target_position):.2f}° over {duration:.1f}s...")
        
        start_time = time.time()
        
        while True:
            elapsed = time.time() - start_time
            
            if elapsed >= duration:
                position = target_position
                done = True
            else:
                # S-curve blending for smooth motion
                progress = 0.5 - 0.5 * math.cos((elapsed / duration) * math.pi)
                position = start_position + (target_position - start_position) * progress
                done = False
            
            self.motor.send_command(
                position=position,
                velocity=0.0,
                kp=self.kp,
                kd=self.kd,
                torque=0.0
            )
            
            feedback = self.motor.read_feedback(timeout=0.01)
            if feedback:
                self.current_position = feedback['position']
                
                if verbose and int(elapsed * 10) % 5 == 0:  # Print every 0.5s
                    current_deg = math.degrees(feedback['position'])
                    target_deg = math.degrees(position)
                    print(f"  Position: {current_deg:6.2f}° (target: {target_deg:6.2f}°)")
            
            if done:
                break
            
            time.sleep(0.01)  # 100Hz control loop
        
        if verbose:
            print(f"Reached target position: {math.degrees(target_position):.2f}°\n")
    
    def oscillate(self, 
                  center_position_deg, 
                  amplitude_deg,
                  frequency=0.6,
                  duration=None,
                  move_to_start_duration=2.0,
                  print_interval=0.25,
                  torque_reverse_enabled=False,
                  torque_threshold=0.4,
                  reverse_cooldown=0.5):
        """
        Perform smooth oscillation around a center position.
        
        This creates a sinusoidal motion pattern with smooth S-curve blending
        for transitions. The motion is natural and prevents jerky movements
        that could damage mechanical components.
        
        Args:
            center_position_deg: Center of oscillation in degrees
            amplitude_deg: Total range of motion in degrees (±amplitude/2 from center)
            frequency: Oscillations per second (Hz)
            duration: How long to oscillate (seconds). None = run until interrupted
            move_to_start_duration: Time to move to starting position (seconds)
            print_interval: How often to print status (seconds)
            torque_reverse_enabled: If True, reverse direction on high torque
            torque_threshold: Torque (Nm) that triggers reversal
            reverse_cooldown: Minimum time between reversals (seconds)
            
        Returns:
            None (runs until duration expires or KeyboardInterrupt)
            
        Example usage:
            # Oscillate ±15° around 45° at 0.5 Hz for 10 seconds
            oscillator.oscillate(
                center_position_deg=45, 
                amplitude_deg=30,  # ±15°
                frequency=0.5,
                duration=10
            )
        """
        # Convert degrees to radians for internal calculations
        center_position = math.radians(center_position_deg)
        amplitude = math.radians(amplitude_deg)
        
        # Calculate the starting position (center of oscillation)
        start_position = center_position
        
        print("="*60)
        print("FIXED RANGE OSCILLATION")
        print("="*60)
        print(f"Center position:  {center_position_deg:6.2f}°")
        print(f"Amplitude:        ±{amplitude_deg/2:5.2f}° (total range: {amplitude_deg:.2f}°)")
        print(f"Frequency:        {frequency:.2f} Hz ({1/frequency:.2f}s per cycle)")
        print(f"Range:            {center_position_deg - amplitude_deg/2:.2f}° to "
              f"{center_position_deg + amplitude_deg/2:.2f}°")
        if torque_reverse_enabled:
            print(f"Torque reversal:  ENABLED (threshold: {torque_threshold:.2f} Nm)")
        print("="*60 + "\n")
        
        # Move to starting position
        print("Moving to starting position...")
        self.smooth_move_to_position(start_position, duration=move_to_start_duration)
        time.sleep(0.5)
        
        # Update current position after move
        self.motor.send_command(
            position=start_position,
            velocity=0.0,
            kp=self.kp,
            kd=self.kd,
            torque=0.0
        )
        time.sleep(0.05)
        feedback = self.motor.read_feedback(timeout=0.1)
        if feedback:
            initial_position = feedback['position']
        else:
            initial_position = start_position
        
        print("Beginning oscillation...")
        if duration:
            print(f"Will run for {duration:.1f} seconds")
        else:
            print("Press Ctrl+C to stop")
        print()
        
        # Oscillation state variables
        direction = 1  # 1 for forward, -1 for reverse
        last_direction_change_time = 0
        direction_transition_start = None
        old_direction = 1
        new_direction = 1
        
        # Blending from initial position into oscillation
        blend_duration = 2.0  # Seconds to blend into oscillation
        
        # For smooth direction transitions
        direction_transition_duration = 0.5  # 0.5s to reverse direction
        
        start_time = time.time()
        last_print_time = start_time
        loop_count = 0
        
        self.is_oscillating = True
        
        try:
            while self.is_oscillating:
                current_time = time.time()
                t = current_time - start_time
                
                # Check if duration has elapsed
                if duration is not None and t >= duration:
                    print(f"\nOscillation duration ({duration:.1f}s) complete.")
                    break
                
                loop_count += 1
                
                # Calculate smooth direction multiplier during transitions
                if direction_transition_start is not None:
                    transition_time = t - direction_transition_start
                    if transition_time < direction_transition_duration:
                        # S-curve transition between directions
                        progress = transition_time / direction_transition_duration
                        smooth_progress = 0.5 - 0.5 * math.cos(progress * math.pi)
                        direction_multiplier = (old_direction * (1 - smooth_progress) + 
                                              new_direction * smooth_progress)
                    else:
                        # Transition complete
                        direction_multiplier = new_direction
                        direction_transition_start = None
                else:
                    direction_multiplier = direction
                
                # Calculate oscillation (centered at 0)
                oscillation = (direction_multiplier * amplitude / 2 * 
                             math.sin(2 * math.pi * frequency * t))
                
                # Blend from initial position to oscillation pattern
                if t < blend_duration:
                    blend_factor = 0.5 - 0.5 * math.cos(t / blend_duration * math.pi)
                    target_position = (initial_position * (1 - blend_factor) + 
                                     (center_position + oscillation) * blend_factor)
                else:
                    target_position = center_position + oscillation
                
                # Send motor command
                self.motor.send_command(
                    position=target_position,
                    velocity=0.0,
                    kp=self.kp,
                    kd=self.kd,
                    torque=0.0
                )
                
                # Read feedback
                feedback = self.motor.read_feedback(timeout=0.01)
                
                if feedback:
                    current_deg = math.degrees(feedback['position'])
                    target_deg = math.degrees(target_position)
                    torque_nm = feedback['torque']
                    
                    self.current_position = feedback['position']
                    
                    # Check for torque-based direction reversal
                    if torque_reverse_enabled:
                        time_since_last_change = t - last_direction_change_time
                        if (abs(torque_nm) >= torque_threshold and 
                            time_since_last_change >= reverse_cooldown):
                            # Start smooth direction transition
                            old_direction = direction
                            new_direction = -direction
                            direction = new_direction
                            direction_transition_start = t
                            last_direction_change_time = t
                            print(f"\n*** Direction reversing! Torque: {torque_nm:.3f} Nm ***\n")
                    
                    # Print status at specified interval
                    if (current_time - last_print_time) >= print_interval:
                        error_deg = target_deg - current_deg
                        print(f"[{t:6.2f}s] Target: {target_deg:6.2f}°  "
                              f"Current: {current_deg:6.2f}°  "
                              f"Error: {error_deg:+5.2f}°  "
                              f"Torque: {torque_nm:6.3f} Nm")
                        last_print_time = current_time
                
                time.sleep(0.01)  # 100Hz control loop
                
        except KeyboardInterrupt:
            print("\n\nOscillation interrupted by user.")
            self.is_oscillating = False
        
        print(f"\nOscillation stopped after {time.time() - start_time:.1f} seconds")
        print(f"Total loop iterations: {loop_count}")
    
    def stop(self):
        """
        Stop the oscillation.
        
        This can be called from another thread or as a cleanup method.
        Useful if you're running oscillation in a separate thread and want
        to stop it programmatically.
        """
        self.is_oscillating = False
