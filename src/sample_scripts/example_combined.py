#!/usr/bin/env python3
"""
Example: Combined Range Discovery and Oscillation

This script demonstrates the complete workflow:
1. Discover the joint's range of motion
2. Calculate safe oscillation parameters
3. Oscillate within the safe range
"""

import sys
from pathlib import Path

# Add parent directory to path so we can import motor_control
sys.path.insert(0, str(Path(__file__).parent.parent))

import time
from motor_control.cubemars_controller import CubeMarsController
from motors.motor import Motor
from motor_control.fixed_range_oscillation import FixedRangeOscillation


def main():
    """
    Complete workflow: Discover range, then oscillate within it.
    
    This demonstrates how the Motor and FixedRangeOscillation classes
    work together to provide safe, intelligent motion control.
    """
    
    # Create the motor controller
    controller = CubeMarsController(motor_id=1)
    
    # Create the high-level Motor interface
    motor_interface = Motor(
        motor_controller=controller,
        motor_id=1,
        name="demo_joint",
        kp=20.0,
        kd=1.0
    )
    
    try:
        # STEP 1: Initialize and discover range
        print("="*60)
        print("COMBINED RANGE DISCOVERY AND OSCILLATION")
        print("="*60)
        
        motor_interface.initialize()
        time.sleep(0.5)
        
        print("\n" + "="*60)
        print("STEP 1: RANGE DISCOVERY")
        print("="*60)
        
        min_deg, max_deg = motor_interface.discover_range(
            torque_threshold=0.5,
            creep_velocity=0.15
        )
        
        # STEP 2: Calculate safe oscillation parameters
        print("\n" + "="*60)
        print("STEP 2: CALCULATING SAFE OSCILLATION PARAMETERS")
        print("="*60)
        
        safe_min, safe_max, center, total_range = motor_interface.get_safe_range()
        
        print(f"\nDiscovered range: {min_deg:.2f}° to {max_deg:.2f}°")
        print(f"Safe range:       {safe_min:.2f}° to {safe_max:.2f}°")
        print(f"Center:           {center:.2f}°")
        print(f"Total safe range: {total_range:.2f}°")
        
        # STEP 3: Create oscillator and run patterns
        print("\n" + "="*60)
        print("STEP 3: OSCILLATION WITHIN SAFE RANGE")
        print("="*60 + "\n")
        
        # Note: We need to get the raw motor controller for the oscillator
        # In a future refactor, you might make Motor expose the controller,
        # or have FixedRangeOscillation work with Motor directly
        
        # For now, create a new oscillator with the raw controller
        oscillator = FixedRangeOscillation(
            motor_controller=controller,
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
        print("\n" + "="*60)
        print("STEP 4: RETURNING TO CENTER")
        print("="*60 + "\n")
        
        motor_interface.move_to(center, duration=3.0)
        time.sleep(1.0)
        
        print("Holding at center for 5 seconds...")
        motor_interface.hold_position(duration=5.0, print_interval=1.0)
        
        print("\n" + "="*60)
        print("DEMONSTRATION COMPLETE!")
        print("="*60)
        print(f"\nJoint range:  {min_deg:.2f}° to {max_deg:.2f}°")
        print(f"Safe range:   {safe_min:.2f}° to {safe_max:.2f}°")
        print(f"Final position: {motor_interface.get_position_degrees():.2f}°")
        
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        motor_interface.shutdown()
        print("\nProgram complete")


if __name__ == "__main__":
    main()
