#!/usr/bin/env python3
"""
Basic Motor Control Demonstration

Simple example showing basic position control with sinusoidal motion.
This is the simplest possible use case - just moving a motor back and forth.
"""

import time
import math
from motor_drivers import CubeMarsDriver


def main():
    """
    Basic sinusoidal motion demo.
    
    This demonstrates:
    - Initializing a motor driver
    - Entering motor mode
    - Sending position commands in a loop
    - Reading feedback
    - Clean shutdown
    """
    # Create motor driver (motor ID 1)
    driver = CubeMarsDriver(motor_id=1)
    
    try:
        # Enter motor mode (enable control)
        driver.enter_motor_mode()
        time.sleep(0.5)
        
        # Optional: Set current position as zero reference
        # Uncomment this if you want to zero the motor at startup
        # driver.set_zero_position()
        # time.sleep(0.5)
        
        print("\nStarting motor control demo...")
        print("Press Ctrl+C to stop\n")
        
        # Control loop
        start_time = time.time()
        while True:
            # Generate sinusoidal position command
            t = time.time() - start_time
            target_position = 2.0 * math.sin(2 * math.pi * 0.2 * t)  # 0.2 Hz sine wave
            
            # Send command with position control
            # kp=50, kd=1 are moderate gains for smooth motion
            driver.send_command(
                position=target_position,
                velocity=0.0,
                kp=50.0,
                kd=1.0,
                torque=0.0
            )
            
            # Read feedback and display
            feedback = driver.read_feedback(timeout=0.05)
            if feedback:
                print(f"Pos: {feedback['position']:6.3f} rad  "
                      f"Vel: {feedback['velocity']:6.3f} rad/s  "
                      f"Torque: {feedback['torque']:6.3f} Nm", end='\r')
            
            time.sleep(0.01)  # 100 Hz control loop
    
    except KeyboardInterrupt:
        print("\n\nStopping motor...")
    
    finally:
        # Always exit motor mode and close driver safely
        driver.close()
        print("Motor stopped")


if __name__ == "__main__":
    main()
