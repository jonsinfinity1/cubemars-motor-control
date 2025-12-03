#!/usr/bin/env python3
"""
Combined Range Discovery and Oscillation Example

Complete workflow demonstrating:
1. Discover the joint's range of motion
2. Calculate safe oscillation parameters
3. Oscillate within the safe range
"""

import time
from motor_drivers import CubeMarsDriver
from motor_control import Joint, FixedRangeOscillation


def main():
    """
    Complete workflow: Discover range, then oscillate within it.
    
    This demonstrates how the Joint and FixedRangeOscillation classes
    work together to provide safe, intelligent motion control.
    """
    
    # Create the motor driver
    driver = CubeMarsDriver(motor_id=1)
    
    # Create the high-level Joint interface
    joint = Joint(
        driver=driver,
        name="demo_joint",
        kp=20.0,
        kd=1.0
    )
    
    try:
        # STEP 1: Initialize and discover range
        print("=" * 60)
        print("COMBINED RANGE DISCOVERY AND OSCILLATION")
        print("=" * 60)
        
        joint.initialize()
        time.sleep(0.5)
        
        print("\n" + "=" * 60)
        print("STEP 1: RANGE DISCOVERY")
        print("=" * 60)
        
        min_deg, max_deg = joint.discover_range(
            torque_threshold=0.5,
            creep_velocity=0.15
        )
        
        # STEP 2: Calculate safe oscillation parameters
        print("\n" + "=" * 60)
        print("STEP 2: CALCULATING SAFE OSCILLATION PARAMETERS")
        print("=" * 60)
        
        safe_min, safe_max, center, total_range = joint.get_safe_range()
        
        print(f"\nDiscovered range: {min_deg:.2f}° to {max_deg:.2f}°")
        print(f"Safe range:       {safe_min:.2f}° to {safe_max:.2f}°")
        print(f"Center:           {center:.2f}°")
        print(f"Total safe range: {total_range:.2f}°")
        
        # STEP 3: Create oscillator and run patterns
        print("\n" + "=" * 60)
        print("STEP 3: OSCILLATION WITHIN SAFE RANGE")
        print("=" * 60 + "\n")
        
        # Create oscillator using the same driver
        oscillator = FixedRangeOscillation(
            motor_driver=driver,
            kp=20.0,
            kd=1.0
        )
        
        # Pattern 1: Full range, slow oscillation
        print("Pattern 1: Full safe range oscillation (slow)")
        oscillator.oscillate(
            center_position_deg=center,
            amplitude_deg=total_range * 0.9,  # Use 90% of safe range
            frequency=0.3,                     # Slow for full range
            duration=15,
            print_interval=0.5
        )
        
        time.sleep(2)
        
        # Pattern 2: Smaller oscillation at center
        print("\nPattern 2: Smaller oscillation at center (medium speed)")
        oscillator.oscillate(
            center_position_deg=center,
            amplitude_deg=30,      # Just ±15°
            frequency=0.6,
            duration=10,
            print_interval=0.5
        )
        
        time.sleep(2)
        
        # Pattern 3: Oscillation offset to one side
        offset_center = safe_min + (total_range * 0.25)  # 25% from min
        print(f"\nPattern 3: Offset oscillation (centered at {offset_center:.2f}°)")
        oscillator.oscillate(
            center_position_deg=offset_center,
            amplitude_deg=40,
            frequency=0.5,
            duration=10,
            print_interval=0.5
        )
        
        # STEP 4: Return to center and hold
        print("\n" + "=" * 60)
        print("STEP 4: RETURNING TO CENTER")
        print("=" * 60 + "\n")
        
        joint.move_to(center, duration=3.0)
        time.sleep(1.0)
        
        print("Holding at center for 5 seconds...")
        joint.hold_position(duration=5.0, print_interval=1.0)
        
        print("\n" + "=" * 60)
        print("DEMONSTRATION COMPLETE!")
        print("=" * 60)
        print(f"\nJoint range:  {min_deg:.2f}° to {max_deg:.2f}°")
        print(f"Safe range:   {safe_min:.2f}° to {safe_max:.2f}°")
        print(f"Final position: {joint.get_position_degrees():.2f}°")
    
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
    
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        joint.shutdown()
        print("\nProgram complete")


if __name__ == "__main__":
    main()
