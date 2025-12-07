#!/usr/bin/env python3
"""
Read Motor Position (No Movement)

This script reads motor position without sending any movement commands.
Safe to use - will not cause motor to move or kick.

Uses pure query mode to read encoder position.
"""

import sys
import time
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from motor_drivers import CubeMarsDriver


def read_position_safely(motor_id):
    """
    Read motor position without causing any movement.
    
    This uses a special technique:
    - Enter motor mode
    - Read feedback WITHOUT sending position commands
    - Just listen to the motor's state broadcasts
    """
    print(f"\n{'=' * 70}")
    print(f"READING MOTOR {motor_id} POSITION (NO MOVEMENT)")
    print('=' * 70)
    
    try:
        driver = CubeMarsDriver(motor_id=motor_id)
        driver.flush_buffer()
        
        # Enter motor mode
        print("\nEntering motor mode...")
        driver.enter_motor_mode()
        time.sleep(0.5)
        
        # The motor may send status updates on its own
        # Try to read without sending any command first
        print("Listening for motor status...")
        time.sleep(0.2)
        
        # Read any existing messages
        feedback = driver.read_feedback(timeout=0.3)
        
        if feedback:
            raw_pos = math.degrees(feedback['position'])
            print(f"\nMotor {motor_id} raw position: {raw_pos:.2f}°")
            print(f"Torque: {feedback['torque']:.3f} Nm")
            print(f"Velocity: {feedback['velocity']:.3f} rad/s")
        else:
            # If no automatic status, send a VERY gentle query
            # Use torque=0 and very low gains so it won't try to move
            print("\nNo automatic status, sending gentle query...")
            
            # CRITICAL: Use current position as target (no movement)
            # Since we don't know current position yet, use kp=0 (no position control)
            driver.send_command(
                position=0,      # Doesn't matter, kp=0 means ignored
                velocity=0,
                kp=0.0,          # ZERO position gain = no position control!
                kd=0.05,         # Tiny damping only
                torque=0.0       # No feedforward torque
            )
            time.sleep(0.1)
            
            feedback = driver.read_feedback(timeout=0.2)
            
            if feedback:
                raw_pos = math.degrees(feedback['position'])
                print(f"\nMotor {motor_id} raw position: {raw_pos:.2f}°")
                print(f"Torque: {feedback['torque']:.3f} Nm")
                print(f"Velocity: {feedback['velocity']:.3f} rad/s")
            else:
                print("\n✗ Could not read position")
        
        # Exit motor mode
        driver.exit_motor_mode()
        driver.close()
        
        return feedback
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return None


def read_with_offset(motor_id):
    """
    Read position with offset applied (logical position).
    """
    print(f"\n{'=' * 70}")
    print(f"READING MOTOR {motor_id} WITH OFFSET")
    print('=' * 70)
    
    try:
        from offset_motor_driver import OffsetCubeMarsDriver
        
        driver = OffsetCubeMarsDriver.from_config(motor_id=motor_id)
        driver.flush_buffer()
        driver.enter_motor_mode()
        time.sleep(0.5)
        
        # Use kp=0 to avoid movement
        driver.send_command(0, 0, 0.0, 0.05, 0)
        time.sleep(0.1)
        
        feedback = driver.read_feedback(timeout=0.2)
        
        if feedback:
            logical_pos = math.degrees(feedback['position'])
            print(f"\nMotor {motor_id} logical position: {logical_pos:.2f}°")
            print(f"(This is the offset-corrected position)")
        else:
            print("\n✗ Could not read position")
        
        driver.close()
        return feedback
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        return None


def main():
    print("=" * 70)
    print("SAFE POSITION READER")
    print("=" * 70)
    print("\nThis script reads motor position without causing movement.")
    print("Safe to use even when motor position doesn't match software state.")
    
    motor_id = int(input("\nEnter motor ID: "))
    
    # Read raw position (no offset)
    raw_feedback = read_position_safely(motor_id)
    
    if raw_feedback:
        raw_pos = math.degrees(raw_feedback['position'])
        
        # Try to read with offset
        print("\n" + "-" * 70)
        offset_feedback = read_with_offset(motor_id)
        
        if offset_feedback:
            logical_pos = math.degrees(offset_feedback['position'])
            calculated_offset = raw_pos - logical_pos
            
            print("\n" + "=" * 70)
            print("POSITION SUMMARY")
            print("=" * 70)
            print(f"\nRaw motor position:     {raw_pos:7.2f}°")
            print(f"Logical position:       {logical_pos:7.2f}°")
            print(f"Calculated offset:      {calculated_offset:7.2f}°")
            
            # Load config to show configured offset
            import json
            config_path = Path(__file__).parent.parent / 'configs' / 'motor_config.json'
            if config_path.exists():
                with open(config_path, 'r') as f:
                    config = json.load(f)
                for motor in config['motors']:
                    if motor['id'] == motor_id:
                        configured_offset = motor.get('position_offset_deg', 0.0)
                        print(f"Configured offset:      {configured_offset:7.2f}°")
                        print(f"Offset difference:      {abs(calculated_offset - configured_offset):7.2f}°")
                        
                        if abs(calculated_offset - configured_offset) < 1.0:
                            print("\n✓ Offset configuration is correct!")
                        else:
                            print("\n⚠  Offset mismatch detected!")
                            print("  Motor may have moved since offset was configured")
                        break


if __name__ == "__main__":
    main()
