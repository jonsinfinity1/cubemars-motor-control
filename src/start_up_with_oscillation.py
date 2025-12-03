#!/usr/bin/env python3
"""
Combined Range Discovery and Oscillation Demo
Discovers the range, then oscillates within safe limits
"""

import time
from motors.cubemars_motor import CubeMarsMotor
from fixed_range_oscillation import FixedRangeOscillation


def main():
    motor = CubeMarsMotor(motor_id=1)

    try:
        # Initialize
        print("Initializing motor...")
        motor.flush_can_buffer()
        motor.enter_motor_mode()
        time.sleep(0.5)

        # For this demo, let's assume you've already discovered your range
        # In a real application, you'd run the range discovery first
        # Let's say your range is -60° to +60° (adjust to your actual range)

        discovered_min = -60  # degrees
        discovered_max = 60  # degrees

        # Calculate safe oscillation limits (stay 5° away from hard stops)
        safety_margin = 5  # degrees
        safe_min = discovered_min + safety_margin
        safe_max = discovered_max - safety_margin

        # Calculate center and amplitude
        center = (safe_max + safe_min) / 2
        total_range = safe_max - safe_min

        print(f"\nDiscovered range: {discovered_min}° to {discovered_max}°")
        print(f"Safe range: {safe_min}° to {safe_max}°")
        print(f"Center: {center}°, Total safe range: {total_range}°\n")

        # Create oscillator
        oscillator = FixedRangeOscillation(motor=motor, kp=20.0, kd=1.0)

        # Oscillate through the full safe range
        print("Oscillating through full safe range...")
        oscillator.oscillate(
            center_position_deg=center,
            amplitude_deg=total_range,
            frequency=0.3,  # Slow for full range
            duration=20,  # 20 seconds
            print_interval=0.5  # Print twice per second
        )

        time.sleep(2)

        # Now do a smaller oscillation at the center
        print("\nSmaller oscillation at center...")
        oscillator.oscillate(
            center_position_deg=center,
            amplitude_deg=30,  # Just ±15°
            frequency=0.6,
            duration=15
        )

    except KeyboardInterrupt:
        print("\n\nProgram stopped by user.")

    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()

    finally:
        try:
            motor.close()
        except:
            pass
        print("\nMotor stopped.")


if __name__ == "__main__":
    main()