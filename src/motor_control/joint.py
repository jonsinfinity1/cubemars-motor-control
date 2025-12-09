#!/usr/bin/env python3
"""
High-Level Joint Control with State Management

This module provides the Joint class which wraps low-level motor drivers
and provides high-level operations like smooth moves, range discovery, and
state tracking.

Robotics Context:
----------------
In robotics, we think in terms of joints (movable connections) rather than
motors (actuators). A joint has:
- Physical range of motion (limits)
- Current state (position, velocity, torque)
- Behaviors (move to position, hold position, etc.)

The motor/actuator is just the hardware that makes the joint move.
This class represents the joint-level abstraction.
"""

import time
import math


class Joint:
    """
    High-level robotic joint control with state management.
    
    This wraps a low-level MotorDriver and provides:
    - State tracking (position, velocity, torque)
    - Range discovery (finding physical limits via force sensing)
    - Smooth trajectory generation (S-curves for natural motion)
    - Safety management (limits, timeouts, error handling)
    
    Design Pattern:
    --------------
    This uses the Strategy pattern via composition - the Joint "has-a"
    driver, not "is-a" driver. This means you can swap different motor
    types (CubeMars, T-Motor, Maxon, etc.) without changing this class.
    
    Java Analogy:
    ------------
    In Java you might write:
        public class Joint {
            private MotorDriver driver;  // Dependency injection
            public Joint(MotorDriver driver) {
                this.driver = driver;
            }
        }
    
    Python does the same thing, just with less ceremony.
    
    Example Usage:
    -------------
        from motor_drivers import CubeMarsDriver
        from motor_control import Joint
        
        driver = CubeMarsDriver(motor_id=1)
        knee = Joint(driver=driver, name="left_knee", kp=20.0, kd=1.0)
        
        knee.initialize()
        knee.discover_range()
        knee.move_to(45.0, duration=2.0)
        knee.shutdown()
    """
    
    def __init__(self, driver, name=None, kp=20.0, kd=1.0):
        """
        Initialize joint controller.
        
        Args:
            driver: MotorDriver implementation (e.g., CubeMarsDriver)
            name: Human-readable name (e.g., "left_knee", "right_elbow")
            kp: Default position gain / stiffness (0-500 typical)
            kd: Default damping gain (0-5 typical)
        
        Robotics Note on Gains:
        ----------------------
        - kp (position gain): How stiffly the joint resists position error
          * High (50-100): Precise, fights external forces, can be jerky
          * Medium (20-50): Balanced for humanoid motion
          * Low (5-20): Compliant, yields to external forces
        
        - kd (damping gain): How much the joint resists velocity changes
          * High (2-5): Heavily damped, slow, smooth
          * Medium (0.5-2): Natural motion
          * Low (0.1-0.5): Fast, underdamped, can oscillate
        """
        self.driver = driver
        self.name = name or "Joint"
        self.default_kp = kp
        self.default_kd = kd
        
        # Current state tracking
        # IMPORTANT: None means "unknown" - we must read before using!
        self.current_position = None  # radians (None until first read)
        self.current_velocity = 0.0   # rad/s
        self.current_torque = 0.0     # Nm
        self.is_initialized = False
        
        # Discovered range (None until discovery is run)
        self.min_position = None  # radians
        self.max_position = None  # radians
        self.range_discovered = False
        
        # Safety parameters
        self.safety_margin = math.radians(5)  # 5 degrees from hard stops
    
    def initialize(self):
        """
        Initialize the joint for operation.
        
        CRITICAL SAFETY: This uses a careful sequence to prevent motor jumps:
        1. Clear stale CAN messages
        2. Enable motor mode
        3. IMMEDIATELY send zero-gain command to read position (no movement)
        4. Validate we got a valid position before proceeding
        
        The key insight: MIT mode commands always include a position target.
        If we send ANY command with non-zero kp before knowing the actual
        position, the motor will jump to whatever position we specified.
        
        Must be called before any other operations.
        """
        print(f"[{self.name}] Initializing...")
        self.driver.flush_buffer()
        self.driver.enter_motor_mode()
        
        # CRITICAL: Immediately read position with ZERO gains
        # This prevents any movement while we synchronize state
        time.sleep(0.1)  # Brief settle time for motor mode transition
        
        # Try multiple times to get a valid position reading
        position_read_success = False
        for attempt in range(5):
            # Send zero-gain command - motor won't move, just responds with state
            self.driver.send_command(
                position=0.0,    # Ignored when kp=0
                velocity=0.0,
                kp=0.0,          # ZERO - no position control
                kd=0.0,          # ZERO - no damping
                torque=0.0       # ZERO - no torque
            )
            time.sleep(0.05)
            
            feedback = self.driver.read_feedback(timeout=0.1)
            if feedback and 'position' in feedback:
                self.current_position = feedback['position']
                self.current_velocity = feedback.get('velocity', 0.0)
                self.current_torque = feedback.get('torque', 0.0)
                position_read_success = True
                break
            else:
                print(f"[{self.name}] Position read attempt {attempt + 1}/5 failed, retrying...")
                time.sleep(0.1)
        
        if not position_read_success:
            # SAFETY: Cannot proceed without knowing current position
            self.driver.exit_motor_mode()
            raise RuntimeError(
                f"[{self.name}] SAFETY ABORT: Failed to read motor position after 5 attempts. "
                f"Cannot safely initialize without knowing current position."
            )
        
        self.is_initialized = True
        print(f"[{self.name}] Initialized at {math.degrees(self.current_position):.2f}°")
    
    def _update_state(self):
        """
        Update internal state from motor feedback.
        
        Uses ZERO gains to avoid any movement - we're just
        querying the current position.
        
        Returns:
            bool: True if state was successfully updated, False otherwise
        
        IMPORTANT: Callers should check the return value! If this returns
        False, self.current_position may be stale or None.
        
        Python Note: Leading underscore indicates this is "private" - meant
        for internal use only. It's a convention, not enforced like Java's private.
        """
        # CRITICAL: Flush any stale messages before reading
        # This is essential when multiple motors share the same CAN bus,
        # as each driver's buffer receives ALL messages on the bus
        self.driver.flush_buffer()
        
        # CRITICAL: Use kp=0, kd=0, torque=0 to query position WITHOUT moving!
        # The position parameter is ignored when gains are zero.
        self.driver.send_command(
            position=0.0,    # Ignored when kp=0
            velocity=0.0,
            kp=0.0,          # ZERO - don't try to move!
            kd=0.0,          # ZERO - don't try to move!
            torque=0.0
        )
        time.sleep(0.05)
        
        feedback = self.driver.read_feedback(timeout=0.1)
        if feedback and 'position' in feedback:
            self.current_position = feedback['position']
            self.current_velocity = feedback.get('velocity', 0.0)
            self.current_torque = feedback.get('torque', 0.0)
            return True
        return False
    
    def get_position_degrees(self):
        """Get current position in degrees. Returns None if position unknown."""
        if self.current_position is None:
            return None
        return math.degrees(self.current_position)
    
    def get_position_radians(self):
        """Get current position in radians. Returns None if position unknown."""
        return self.current_position
    
    # Control loop timing constants
    CONTROL_RATE_HZ = 100       # Target loop frequency
    CONTROL_PERIOD = 1.0 / CONTROL_RATE_HZ  # 0.01 seconds per iteration
    
    def move_to(self, target_deg, duration=2.0, kp=None, kd=None, verbose=True):
        """
        Move smoothly to a target position using S-curve trajectory.
        
        Robotics: S-Curve Trajectories
        ------------------------------
        An S-curve provides smooth acceleration and deceleration:
        - Starts with zero velocity (smooth start)
        - Accelerates smoothly
        - Cruises at max velocity
        - Decelerates smoothly
        - Ends with zero velocity (smooth stop)
        
        Why this matters:
        - Prevents jerky motion that can damage mechanical components
        - Reduces wear on gearboxes and joints
        - Makes motion look more natural for humanoid robots
        - Minimizes oscillations and overshoot
        
        We use a cosine-based S-curve: position = 0.5 - 0.5*cos(π*t)
        This gives the classic S-shaped velocity profile.
        
        Real-Time Control:
        -----------------
        This method uses deadline-based timing rather than sleep-based timing.
        Instead of "do work, then sleep for 10ms", we calculate "when should
        the next iteration start" and sleep until that exact time.
        
        This ensures consistent loop timing regardless of how long the
        processing takes, which is critical for smooth motion.
        
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
            raise RuntimeError(f"[{self.name}] Not initialized. Call initialize() first.")
        
        kp = kp or self.default_kp
        kd = kd or self.default_kd
        target_rad = math.radians(target_deg)
        
        # CRITICAL: Read actual position before starting move
        # This ensures we start from where the motor actually is,
        # not from stale state that might be wrong
        if not self._update_state():
            raise RuntimeError(
                f"[{self.name}] SAFETY ABORT: Failed to read current position. "
                f"Cannot safely move without knowing starting position."
            )
        
        # Double-check we have a valid position (not None)
        if self.current_position is None:
            raise RuntimeError(
                f"[{self.name}] SAFETY ABORT: Current position is unknown. "
                f"This should not happen after successful _update_state()."
            )
        
        if verbose:
            print(f"[{self.name}] Moving from {self.get_position_degrees():.2f}° "
                  f"to {target_deg:.2f}° over {duration:.1f}s")
        
        start_position = self.current_position
        
        # Timing: use perf_counter for high-precision timing
        # perf_counter is monotonic and has higher resolution than time.time()
        loop_start = time.perf_counter()
        next_deadline = loop_start
        
        # Timing statistics (for diagnostics)
        overrun_count = 0
        max_overrun = 0.0
        iteration_count = 0
        last_print_time = loop_start
        
        while True:
            iteration_count += 1
            now = time.perf_counter()
            elapsed = now - loop_start
            
            # Calculate trajectory position
            if elapsed >= duration:
                position = target_rad
                done = True
            else:
                # S-curve trajectory: smooth acceleration and deceleration
                # The cosine function creates the S-shaped velocity profile
                progress = 0.5 - 0.5 * math.cos((elapsed / duration) * math.pi)
                position = start_position + (target_rad - start_position) * progress
                done = False
            
            # Send command to motor
            self.driver.send_command(
                position=position,
                velocity=0.0,
                kp=kp,
                kd=kd,
                torque=0.0
            )
            
            # Read feedback (with short timeout to not block too long)
            feedback = self.driver.read_feedback(timeout=0.005)
            if feedback:
                self.current_position = feedback['position']
                self.current_velocity = feedback.get('velocity', 0.0)
                self.current_torque = feedback.get('torque', 0.0)
            
            # Verbose output (throttled to ~2Hz to not spam)
            if verbose and (now - last_print_time) >= 0.5:
                print(f"  [{self.name}] Position: {self.get_position_degrees():6.2f}°")
                last_print_time = now
            
            if done:
                break
            
            # Deadline-based timing:
            # Calculate when the next iteration SHOULD start
            next_deadline += self.CONTROL_PERIOD
            sleep_time = next_deadline - time.perf_counter()
            
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Overrun: loop took longer than budget
                overrun_count += 1
                overrun_amount = -sleep_time
                max_overrun = max(max_overrun, overrun_amount)
                # Reset deadline to avoid cascading overruns
                next_deadline = time.perf_counter()
        
        # Final position command (hold at target)
        self.driver.send_command(
            position=target_rad,
            velocity=0.0,
            kp=kp,
            kd=kd,
            torque=0.0
        )
        
        if verbose:
            print(f"[{self.name}] Reached {self.get_position_degrees():.2f}°")
            if overrun_count > 0:
                print(f"  Timing: {overrun_count}/{iteration_count} overruns "
                      f"(max: {max_overrun*1000:.1f}ms)")
        
        return True
    
    def discover_range(self, torque_threshold=0.5, creep_velocity=0.15,
                       max_exploration_time=15.0, verbose=True):
        """
        Discover the full range of motion by exploring until hitting stops.
        
        Robotics: Range Discovery
        -------------------------
        This method uses force sensing to find physical limits. It slowly
        moves the joint in each direction until it detects a hard stop
        (via torque feedback), then backs off slightly and records the
        safe operating limits.
        
        Why this is better than manual calibration:
        - Automatically adapts to different mechanical designs
        - Finds the actual limits, not theoretical limits
        - Can re-run if mechanical stops change
        - Safer than sending commands beyond limits
        
        The process:
        1. Move to center (0°)
        2. Explore positive direction until hitting stop
        3. Back off and record max position
        4. Return to center
        5. Explore negative direction until hitting stop
        6. Back off and record min position
        
        Args:
            torque_threshold: Torque (Nm) indicating a hard stop
            creep_velocity: Slow exploration speed (rad/s)
            max_exploration_time: Safety timeout per direction (seconds)
            verbose: Print progress updates
            
        Returns:
            tuple: (min_position_deg, max_position_deg)
            
        This method updates the joint's internal range state.
        """
        if not self.is_initialized:
            raise RuntimeError(f"[{self.name}] Not initialized. Call initialize() first.")
        
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
        center_deg = (min_deg + max_deg) / 2
        
        if verbose:
            print(f"\n[{self.name}] Range discovery complete:")
            print(f"  Min: {min_deg:7.2f}°")
            print(f"  Max: {max_deg:7.2f}°")
            print(f"  Total range: {max_deg - min_deg:7.2f}°")
        
        # Return to center position for safety
        if verbose:
            print(f"\n[{self.name}] Returning to center ({center_deg:.2f}°)...")
        self.move_to(center_deg, duration=3.0, verbose=False)
        time.sleep(0.5)
        
        return (min_deg, max_deg)
    
    # Exploration timing (slower than control loop for gentle movement)
    EXPLORE_RATE_HZ = 20
    EXPLORE_PERIOD = 1.0 / EXPLORE_RATE_HZ  # 0.05 seconds per iteration
    
    def _explore_direction(self, direction, torque_threshold, creep_velocity,
                          max_exploration_time, verbose):
        """
        Explore in one direction until hitting a stop.
        
        Internal helper method for discover_range(). Uses very compliant
        control (low kp/kd) so the motor can "feel" the stop without
        fighting it aggressively.
        
        Uses deadline-based timing at 20Hz (slower than normal control)
        for gentle, consistent exploration speed.
        
        Args:
            direction: 1 for positive, -1 for negative
            torque_threshold: Torque indicating a stop
            creep_velocity: Slow exploration speed
            max_exploration_time: Safety timeout
            verbose: Print updates
            
        Returns:
            float: Position where stop was detected (radians)
        """
        stop_detected = False
        stop_position = None
        
        # Very compliant control for exploration
        # We want to feel the stop, not fight it
        kp = 5.0   # Low stiffness
        kd = 0.5   # Light damping
        
        # Timing setup
        loop_start = time.perf_counter()
        next_deadline = loop_start
        last_print_time = loop_start
        
        while not stop_detected:
            now = time.perf_counter()
            elapsed = now - loop_start
            
            # Safety timeout
            if elapsed >= max_exploration_time:
                break
            
            # Command a slow velocity in the exploration direction
            self.driver.send_command(
                position=self.current_position + (direction * 0.1),
                velocity=direction * creep_velocity,
                kp=kp,
                kd=kd,
                torque=0.0
            )
            
            feedback = self.driver.read_feedback(timeout=0.02)
            
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
                elif verbose and (now - last_print_time) >= 0.25:
                    print(f"  Position: {self.get_position_degrees():6.2f}°  "
                          f"Torque: {self.current_torque:5.3f} Nm")
                    last_print_time = now
            
            # Deadline-based timing at 20Hz
            next_deadline += self.EXPLORE_PERIOD
            sleep_time = next_deadline - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_deadline = time.perf_counter()  # Reset on overrun
        
        if not stop_detected:
            if verbose:
                print(f"  Warning: No stop detected after {max_exploration_time}s")
            stop_position = self.current_position
        
        # Back off slightly from the hard stop for safety
        backoff = math.radians(2)  # 2 degrees
        safe_position = stop_position - (direction * backoff)
        
        if verbose:
            print(f"  Backing off 2° from stop...")
        
        # Quick backoff movement with deadline-based timing
        start_pos = self.current_position
        backoff_start = time.perf_counter()
        next_deadline = backoff_start
        
        for i in range(20):
            progress = (i + 1) / 20.0
            pos = start_pos + (safe_position - start_pos) * progress
            self.driver.send_command(pos, 0.0, 10.0, 0.8, 0.0)
            self.driver.read_feedback(timeout=0.01)
            
            next_deadline += self.EXPLORE_PERIOD
            sleep_time = next_deadline - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        self.current_position = safe_position
        return safe_position
    
    def get_safe_range(self):
        """
        Get the safe operating range (with safety margins from hard stops).
        
        Returns a tuple with all the useful range information:
        - Safe minimum (with margin from hard stop)
        - Safe maximum (with margin from hard stop)
        - Center position
        - Total safe range
        
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
        
        This actively maintains the current position using impedance control.
        The joint will resist external forces (how much depends on kp/kd).
        
        Uses deadline-based timing at 100Hz for consistent control.
        
        Useful for:
        - Testing impedance control tuning
        - Maintaining a specific posture
        - Demonstrating force interaction
        
        Args:
            duration: How long to hold (None = indefinite, until Ctrl+C)
            kp: Position gain (uses default if None)
            kd: Damping gain (uses default if None)
            print_interval: How often to print status (seconds)
        """
        if not self.is_initialized:
            raise RuntimeError(
                f"[{self.name}] Not initialized. Call initialize() first."
            )
        
        kp = kp or self.default_kp
        kd = kd or self.default_kd
        
        # Read actual position before holding
        if not self._update_state():
            raise RuntimeError(
                f"[{self.name}] SAFETY ABORT: Failed to read current position. "
                f"Cannot safely hold without knowing current position."
            )
        
        if self.current_position is None:
            raise RuntimeError(
                f"[{self.name}] SAFETY ABORT: Current position is unknown."
            )
        
        hold_position = self.current_position
        
        print(f"[{self.name}] Holding position at {self.get_position_degrees():.2f}°")
        if duration:
            print(f"  Duration: {duration:.1f}s")
        else:
            print(f"  Press Ctrl+C to stop")
        
        # Timing setup
        loop_start = time.perf_counter()
        next_deadline = loop_start
        last_print_time = loop_start
        overrun_count = 0
        iteration_count = 0
        
        try:
            while True:
                iteration_count += 1
                now = time.perf_counter()
                elapsed = now - loop_start
                
                # Check if duration has elapsed
                if duration and elapsed >= duration:
                    break
                
                self.driver.send_command(
                    position=hold_position,
                    velocity=0.0,
                    kp=kp,
                    kd=kd,
                    torque=0.0
                )
                
                feedback = self.driver.read_feedback(timeout=0.005)
                if feedback:
                    self.current_position = feedback['position']
                    self.current_torque = feedback.get('torque', 0.0)
                
                # Print status at specified interval
                if (now - last_print_time) >= print_interval:
                    error_deg = math.degrees(hold_position - self.current_position)
                    print(f"  Position: {self.get_position_degrees():6.2f}°  "
                          f"Error: {error_deg:+5.2f}°  "
                          f"Torque: {self.current_torque:5.3f} Nm")
                    if overrun_count > 0:
                        print(f"  Timing: {overrun_count}/{iteration_count} overruns")
                    last_print_time = now
                    # Reset counters each print cycle
                    overrun_count = 0
                    iteration_count = 0
                
                # Deadline-based timing
                next_deadline += self.CONTROL_PERIOD
                sleep_time = next_deadline - time.perf_counter()
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    overrun_count += 1
                    next_deadline = time.perf_counter()  # Reset on overrun
                
        except KeyboardInterrupt:
            print(f"\n[{self.name}] Hold interrupted")
    
    def shutdown(self):
        """
        Clean shutdown of the joint.
        
        1. Exits motor mode (motor goes limp)
        2. Closes driver
        3. Marks as not initialized
        
        Safe to call multiple times - subsequent calls are no-ops.
        Always call this before program exit!
        """
        if not self.is_initialized:
            # Already shut down, nothing to do
            return
        
        print(f"[{self.name}] Shutting down...")
        try:
            self.driver.close()
        except Exception as e:
            # Driver may already be closed - that's fine
            pass
        self.is_initialized = False
