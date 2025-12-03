#!/usr/bin/env python3
"""
Example: Motor Range Discovery

This script demonstrates how to use the Motor class to discover
the range of motion for a joint, then move to various positions.
"""

import sys
from pathlib import Path

# Add parent directory to path so we can import motor_control
sys.path.insert(0, str(Path(__file__).parent.parent))

import time
from motor_control.cubemars_controller import CubeMarsController
from motors.motor import Motor


def main():
    """
    Discover joint range and demonstrate position control.
    """
    
    # Create the low-level controller
    controller = CubeMarsController(motor_id=1)
    
    # Create the high-level Motor interface
    motor = Motor(
        motor_controller=controller,
        motor_id=1,
        name="test_joint",
        kp=20.0,
        kd=1.0
    )
    
    try:
        # Initialize the motor
        motor.initialize()
        time.sleep(0.5)
        
        # Discover the full range of motion
        print("\n" + "="*60)
        min_deg, max_deg = motor.discover_range(
            torque_threshold=0.5,
            creep_velocity=0.15
        )
        print("="*60)
        
        time.sleep(1.0)
        
        # Get the safe operating range
        safe_min, safe_max, center, total_range = motor.get_safe_range()
        
        print(f"\nSafe operating range:")
        print(f"  Min:    {safe_min:7.2f}°")
        print(f"  Max:    {safe_max:7.2f}°")
        print(f"  Center: {center:7.2f}°")
        print(f"  Range:  {total_range:7.2f}°")
        
        # Demonstrate moving to various positions
        print("\n" + "="*60)
        print("POSITION CONTROL DEMONSTRATION")
        print("="*60)
        
        # Move to center
        print("\n1. Moving to center position...")
        motor.move_to(center, duration=3.0)
        time.sleep(1.0)
        
        # Move to maximum
        print("\n2. Moving to maximum safe position...")
        motor.move_to(safe_max, duration=3.0)
        time.sleep(1.0)
        
        # Move to minimum
        print("\n3. Moving to minimum safe position...")
        motor.move_to(safe_min, duration=4.0)
        time.sleep(1.0)
        
        # Return to center
        print("\n4. Returning to center...")
        motor.move_to(center, duration=3.0)
        time.sleep(1.0)
        
        # Hold position
        print("\n5. Holding at center for 5 seconds...")
        motor.hold_position(duration=5.0, print_interval=0.5)
        
        print("\n" + "="*60)
        print("Demonstration complete!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        motor.shutdown()
        print("\nMotor shut down successfully")


if __name__ == "__main__":
    main()
