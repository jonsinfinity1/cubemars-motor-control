#!/usr/bin/env python3
"""
Basic demonstration of CubeMars AK40-10 motor control
Simple sinusoidal motion example
"""

import time
import math
from motors.cubemars_motor import CubeMarsMotor


def main():
    # Create motor controller (motor ID 1)
    motor = CubeMarsMotor(motor_id=1)

    try:
        # Enter motor mode
        motor.enter_motor_mode()
        time.sleep(0.5)

        # Optional: Set current position as zero
        # motor.set_zero_position()
        # time.sleep(0.5)

        print("\nStarting motor control demo...")
        print("Press Ctrl+C to stop\n")

        # Control loop
        start_time = time.time()
        while True:
            # Example: Sinusoidal position command
            t = time.time() - start_time
            target_position = 2.0 * math.sin(2 * math.pi * 0.2 * t)  # 0.2 Hz sine wave

            # Send command with position control
            # kp=50, kd=1 are moderate gains for smooth motion
            motor.send_command(
                position=target_position,
                velocity=0.0,
                kp=50.0,
                kd=1.0,
                torque=0.0
            )

            # Read feedback
            feedback = motor.read_feedback(timeout=0.05)
            if feedback:
                print(f"Pos: {feedback['position']:6.3f} rad  "
                      f"Vel: {feedback['velocity']:6.3f} rad/s  "
                      f"Torque: {feedback['torque']:6.3f} Nm", end='\r')

            time.sleep(0.01)  # 100 Hz control loop

    except KeyboardInterrupt:
        print("\n\nStopping motor...")

    finally:
        # Always exit motor mode safely
        motor.close()
        print("Motor stopped")


if __name__ == "__main__":
    main()