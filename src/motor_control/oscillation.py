#!/usr/bin/env python3
"""
Fixed Range Oscillation

Provides smooth, controlled oscillation within specified limits using
sinusoidal motion profiles with S-curve blending.

Useful for:
- Testing joint range of motion
- Creating rhythmic behaviors (walking gaits, breathing motions)
- Demonstrating impedance control
- Interactive demonstrations (torque-based direction reversal)
"""

import time
import math


class FixedRangeOscillation:
    """
    Manages smooth sinusoidal oscillation within a defined range.
    
    This class encapsulates oscillation logic, making it reusable across
    different joints in your humanoid robot. Each joint can have its own
    oscillator instance with different parameters.
    
    Robotics Context:
    ----------------
    Oscillating motion is fundamental in robotics:
    - Walking gaits are oscillations of leg joints
    - Breathing motions for humanoid realism
    - Swimming motions for underwater robots
    - Testing to verify mechanical range and smoothness
    
    This implementation uses:
    - Sinusoidal motion for smooth, natural oscillation
    - S-curve blending for smooth start/stop
    - Optional torque-based interaction (robot responds to external forces)
    
    Design Pattern:
    --------------
    This uses composition - it "has-a" motor driver but doesn't "own" it.
    In Java terms, this is dependency injection via the constructor.
    
    Example Usage:
    -------------
        from motor_drivers import CubeMarsDriver
        from motor_control import FixedRangeOscillation
        
        driver = CubeMarsDriver(motor_id=1)
        driver.enter_motor_mode()
        
        oscillator = FixedRangeOscillation(driver, kp=20.0, kd=1.0)
        oscillator.oscillate(
            center_position_deg=0,
            amplitude_deg=30,      # ±15° from center
            frequency=0.6,         # 0.6 Hz (1.67 second period)
            duration=10
        )
        
        driver.close()
    """
    
    def __init__(self, motor_driver, kp=20.0, kd=1.0):
        """
        Initialize the oscillation controller.
        
        Args:
            motor_driver: MotorDriver instance (e.g., CubeMarsDriver)
            kp: Position gain for impedance control (stiffness)
            kd: Damping gain for impedance control
        
        Python Note:
        -----------
        Storing the motor_driver reference is like dependency injection in Java.
        The class doesn't create its own driver - it uses what's provided.
        This makes testing easier (you can inject a mock driver).
        """
        self.driver = motor_driver
        self.kp = kp
        self.kd = kd
        
        # Current state tracking
        self.current_position = 0.0
        self.is_oscillating = False
    
    def smooth_move_to_position(self, target_position, duration=2.0, verbose=True):
        """
        Move smoothly to a target position using S-curve trajectory.
        
        This is a helper method for positioning before oscillation starts.
        It's public rather than private because it's useful standalone too.
        
        Args:
            target_position: Target position in radians
            duration: Time to complete the move (seconds)
            verbose: Whether to print progress updates
        """
        # Get current actual position from motor WITHOUT commanding movement
        # We use very low gains (compliant) to avoid sudden jumps
        self.driver.send_command(
            position=self.current_position,
            velocity=0.0,
            kp=0.1,  # Very low stiffness - just querying position
            kd=0.1,  # Very low damping
            torque=0.0
        )
        time.sleep(0.05)
        
        feedback = self.driver.read_feedback(timeout=0.1)
        if feedback:
            start_position = feedback['position']
            self.current_position = start_position  # Update our tracked position
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
            
            self.driver.send_command(
                position=position,
                velocity=0.0,
                kp=self.kp,
                kd=self.kd,
                torque=0.0
            )
            
            feedback = self.driver.read_feedback(timeout=0.01)
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
        
        Robotics: Sinusoidal Motion
        ---------------------------
        This creates motion following: position = center + (amplitude/2) * sin(2πft)
        
        Where:
        - f is frequency in Hz (oscillations per second)
        - t is time in seconds
        - amplitude is total range (±amplitude/2 from center)
        
        The sine wave provides:
        - Smooth acceleration at the ends of travel
        - Smooth deceleration at the ends of travel
        - Natural, continuous motion
        
        S-curve blending is used at start/stop to avoid sudden jerks.
        
        Torque Reversal Feature:
        ------------------------
        When enabled, the robot monitors torque feedback and reverses
        direction when it feels resistance. This creates interactive
        behavior - push the joint and it pushes back by reversing.
        
        Args:
            center_position_deg: Center of oscillation in degrees
            amplitude_deg: Total range of motion in degrees (±amplitude/2 from center)
            frequency: Oscillations per second (Hz)
                      0.5 Hz = 2 second period (slow)
                      1.0 Hz = 1 second period (medium)
                      2.0 Hz = 0.5 second period (fast)
            duration: How long to oscillate (seconds). None = run until interrupted
            move_to_start_duration: Time to move to starting position (seconds)
            print_interval: How often to print status (seconds)
            torque_reverse_enabled: If True, reverse direction on high torque
            torque_threshold: Torque (Nm) that triggers reversal
            reverse_cooldown: Minimum time between reversals (seconds)
        
        Returns:
            None (runs until duration expires or KeyboardInterrupt)
        
        Example Usage:
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
        
        print("=" * 60)
        print("FIXED RANGE OSCILLATION")
        print("=" * 60)
        print(f"Center position:  {center_position_deg:6.2f}°")
        print(f"Amplitude:        ±{amplitude_deg / 2:5.2f}° (total range: {amplitude_deg:.2f}°)")
        print(f"Frequency:        {frequency:.2f} Hz ({1 / frequency:.2f}s per cycle)")
        print(f"Range:            {center_position_deg - amplitude_deg / 2:.2f}° to "
              f"{center_position_deg + amplitude_deg / 2:.2f}°")
        if torque_reverse_enabled:
            print(f"Torque reversal:  ENABLED (threshold: {torque_threshold:.2f} Nm)")
        print("=" * 60 + "\n")
        
        # Move to starting position
        print("Moving to starting position...")
        self.smooth_move_to_position(start_position, duration=move_to_start_duration)
        time.sleep(0.5)
        
        # Update current position after move
        self.driver.send_command(
            position=start_position,
            velocity=0.0,
            kp=self.kp,
            kd=self.kd,
            torque=0.0
        )
        time.sleep(0.05)
        feedback = self.driver.read_feedback(timeout=0.1)
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
                self.driver.send_command(
                    position=target_position,
                    velocity=0.0,
                    kp=self.kp,
                    kd=self.kd,
                    torque=0.0
                )
                
                # Read feedback
                feedback = self.driver.read_feedback(timeout=0.01)
                
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
