#!/usr/bin/env python3
"""
Range Discovery Example

Demonstrates how to use the Joint class to discover the range of motion
for a joint, then move to various positions within that range.
"""

import time
from motor_drivers import CubeMarsDriver
from motor_control import Joint


def main():
    """
    Discover joint range and demonstrate position control.
    
    This shows the high-level Joint interface which provides:
    - Automatic range discovery
    - Smooth trajectory generation
    - State management
    - Safety margins
    """
    
    # Create the motor driver (with offset from config)
    driver = CubeMarsDriver.from_config(motor_id=1)
    
    # Create the high-level Joint interface
    # This wraps the driver and provides convenient high-level operations
    joint = Joint(
        driver=driver,
        name="test_joint",
        kp=20.0,
        kd=1.0
    )
    
    try:
        # Initialize the joint
        joint.initialize()
        time.sleep(0.5)
        
        # Discover the full range of motion
        print("\n" + "=" * 60)
        min_deg, max_deg = joint.discover_range(
            torque_threshold=0.5,   # Detect stop at 0.5 Nm torque
            creep_velocity=0.15     # Explore at 0.15 rad/s
        )
        print("=" * 60)
        
        time.sleep(1.0)
        
        # Get the safe operating range (with safety margins)
        safe_min, safe_max, center, total_range = joint.get_safe_range()
        
        print(f"\nSafe operating range:")
        print(f"  Min:    {safe_min:7.2f}°")
        print(f"  Max:    {safe_max:7.2f}°")
        print(f"  Center: {center:7.2f}°")
        print(f"  Range:  {total_range:7.2f}°")
        
        # Demonstrate moving to various positions
        print("\n" + "=" * 60)
        print("POSITION CONTROL DEMONSTRATION")
        print("=" * 60)
        
        # Move to center
        print("\n1. Moving to center position...")
        joint.move_to(center, duration=3.0)
        time.sleep(1.0)
        
        # Move to maximum
        print("\n2. Moving to maximum safe position...")
        joint.move_to(safe_max, duration=3.0)
        time.sleep(1.0)
        
        # Move to minimum
        print("\n3. Moving to minimum safe position...")
        joint.move_to(safe_min, duration=4.0)
        time.sleep(1.0)
        
        # Return to center
        print("\n4. Returning to center...")
        joint.move_to(center, duration=3.0)
        time.sleep(1.0)
        
        # Hold position
        print("\n5. Holding at center for 5 seconds...")
        print("   (Try gently pushing the joint to feel the impedance control)")
        joint.hold_position(duration=5.0, print_interval=0.5)
        
        print("\n" + "=" * 60)
        print("Demonstration complete!")
        print("=" * 60)
    
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
    
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        joint.shutdown()
        print("\nJoint shut down successfully")


if __name__ == "__main__":
    main()
