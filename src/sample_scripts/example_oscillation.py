#!/usr/bin/env python3
"""
Oscillation Example

Demonstrates how to use the FixedRangeOscillation class to create
smooth oscillating motion patterns.
"""

import time
from motor_drivers import CubeMarsDriver
from motor_control import FixedRangeOscillation


def main():
    """
    Demonstration of various oscillation patterns.
    
    Shows:
    - Small, fast oscillations
    - Large, slow oscillations
    - Offset oscillations (not centered at zero)
    - Interactive torque reversal
    """
    driver = CubeMarsDriver.from_config(motor_id=1)
    
    try:
        # Initialize motor
        driver.flush_buffer()
        driver.enter_motor_mode()
        time.sleep(0.5)
        
        # Create oscillator with moderate impedance control
        oscillator = FixedRangeOscillation(
            motor_driver=driver,
            kp=20.0,  # Moderate stiffness
            kd=1.0    # Light damping
        )
        
        print("\n" + "=" * 60)
        print("OSCILLATION DEMONSTRATION")
        print("=" * 60 + "\n")
        
        # Example 1: Small, moderate-speed oscillation at center
        print("Example 1: Small amplitude, moderate frequency")
        oscillator.oscillate(
            center_position_deg=0,
            amplitude_deg=20,      # ±10° from center
            frequency=0.6,
            duration=8             # 8 seconds
        )
        time.sleep(1)
        
        # Example 2: Larger, slower oscillation offset from center
        print("\nExample 2: Larger amplitude, slower frequency, offset center")
        oscillator.oscillate(
            center_position_deg=30,   # Offset 30° from zero
            amplitude_deg=40,         # ±20° from center
            frequency=0.4,            # Slower
            duration=10
        )
        time.sleep(1)
        
        # Example 3: Fast, small oscillation
        print("\nExample 3: Fast, small oscillation")
        oscillator.oscillate(
            center_position_deg=0,
            amplitude_deg=15,      # ±7.5°
            frequency=1.0,         # 1 Hz (faster)
            duration=8
        )
        time.sleep(1)
        
        # Example 4: With torque reversal enabled (interactive)
        print("\nExample 4: Interactive mode - try gently pushing the joint!")
        print("The motor will reverse direction when it feels resistance.\n")
        oscillator.oscillate(
            center_position_deg=0,
            amplitude_deg=30,
            frequency=0.5,
            duration=15,
            torque_reverse_enabled=True,
            torque_threshold=0.4,
            reverse_cooldown=0.5
        )
        
        print("\n" + "=" * 60)
        print("Demonstration complete!")
        print("=" * 60)
    
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user.")
    
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        try:
            driver.close()
        except:
            pass
        print("\nMotor stopped and CAN bus closed.")


if __name__ == "__main__":
    main()
