#!/usr/bin/env python3
"""
Validate Position Offsets

This script validates that position offsets are configured correctly by:
1. Reading raw motor positions
2. Reading offset-corrected positions
3. Comparing the two

This helps verify that offsets are working as expected.
"""

import json
import sys
import time
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from motor_drivers import CubeMarsDriver
from offset_motor_driver import OffsetCubeMarsDriver


def validate_motor(motor_id, motor_name, offset_deg):
    """
    Validate offset configuration for a single motor.
    """
    print("\n" + "=" * 70)
    print(f"VALIDATING: {motor_name} (ID: {motor_id})")
    print("=" * 70)
    print(f"Configured offset: {offset_deg:.2f}°")
    
    try:
        # Create both drivers - one raw, one with offset
        print("\nCreating drivers...")
        raw_driver = CubeMarsDriver(motor_id=motor_id)
        offset_driver = OffsetCubeMarsDriver(motor_id=motor_id, position_offset_deg=offset_deg)
        
        # Initialize both
        raw_driver.flush_buffer()
        raw_driver.enter_motor_mode()
        time.sleep(0.3)
        
        # Read raw position
        print("\nReading raw motor position...")
        raw_driver.send_command(0, 0, 0.1, 0.1, 0)
        time.sleep(0.1)
        raw_feedback = raw_driver.read_feedback(timeout=0.2)
        
        if not raw_feedback:
            print("✗ Could not read raw position")
            raw_driver.close()
            offset_driver.close()
            return False
        
        raw_pos_deg = math.degrees(raw_feedback['position'])
        print(f"  Raw motor position: {raw_pos_deg:.2f}°")
        
        # Read offset-corrected position
        print("\nReading offset-corrected position...")
        offset_driver.send_command(0, 0, 0.1, 0.1, 0)
        time.sleep(0.1)
        offset_feedback = offset_driver.read_feedback(timeout=0.2)
        
        if not offset_feedback:
            print("✗ Could not read offset position")
            raw_driver.close()
            offset_driver.close()
            return False
        
        offset_pos_deg = math.degrees(offset_feedback['position'])
        print(f"  Offset-corrected position: {offset_pos_deg:.2f}°")
        
        # Calculate what offset was actually applied
        actual_offset = raw_pos_deg - offset_pos_deg
        print(f"\nOffset calculation:")
        print(f"  Raw position:    {raw_pos_deg:7.2f}°")
        print(f"  Logical position: {offset_pos_deg:7.2f}°")
        print(f"  Actual offset:   {actual_offset:7.2f}°")
        print(f"  Expected offset: {offset_deg:7.2f}°")
        print(f"  Difference:      {abs(actual_offset - offset_deg):7.2f}°")
        
        # Validate
        offset_error = abs(actual_offset - offset_deg)
        
        if offset_error < 0.5:
            print(f"\n✓ PASS: Offset is working correctly!")
            print(f"  Error: {offset_error:.3f}° (< 0.5° threshold)")
            success = True
        else:
            print(f"\n✗ FAIL: Offset mismatch!")
            print(f"  Error: {offset_error:.3f}° (> 0.5° threshold)")
            print(f"\n  This usually means:")
            print(f"  1. Motor moved between configuring offset and now")
            print(f"  2. Offset wasn't saved correctly to config")
            print(f"  3. Wrong offset value in config")
            success = False
        
        # Cleanup
        raw_driver.close()
        offset_driver.close()
        
        return success
        
    except Exception as e:
        print(f"\n✗ Error during validation: {e}")
        import traceback
        traceback.print_exc()
        return False


def validate_at_logical_zero(motor_id, motor_name):
    """
    Check if motor is currently at logical zero position.
    """
    print("\n" + "=" * 70)
    print(f"LOGICAL ZERO CHECK: {motor_name} (ID: {motor_id})")
    print("=" * 70)
    
    try:
        driver = OffsetCubeMarsDriver.from_config(motor_id=motor_id)
        driver.flush_buffer()
        driver.enter_motor_mode()
        time.sleep(0.3)
        
        driver.send_command(0, 0, 0.1, 0.1, 0)
        time.sleep(0.1)
        feedback = driver.read_feedback(timeout=0.2)
        
        driver.close()
        
        if not feedback:
            print("✗ Could not read position")
            return False
        
        logical_pos_deg = math.degrees(feedback['position'])
        print(f"\nCurrent logical position: {logical_pos_deg:.2f}°")
        
        if abs(logical_pos_deg) < 2.0:
            print("✓ Motor is at (or very close to) logical zero!")
        else:
            print(f"⚠  Motor is {logical_pos_deg:.2f}° away from logical zero")
            print("  This is OK if you moved it after configuring offset")
        
        return True
        
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def main():
    """
    Main validation workflow.
    """
    config_path = Path(__file__).parent.parent / 'configs' / 'motor_config.json'
    
    if not config_path.exists():
        print(f"✗ Config file not found: {config_path}")
        sys.exit(1)
    
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    print("=" * 70)
    print("POSITION OFFSET VALIDATION")
    print("=" * 70)
    print("\nThis script validates that position offsets are working correctly.")
    print("It compares raw motor positions with offset-corrected positions.")
    
    # Check which motors have offsets configured
    motors_with_offsets = []
    motors_without_offsets = []
    
    for motor in sorted(config['motors'], key=lambda m: m['id']):
        offset = motor.get('position_offset_deg')
        if offset is not None and offset != 0.0:
            motors_with_offsets.append(motor)
        else:
            motors_without_offsets.append(motor)
    
    if not motors_with_offsets:
        print("\n✗ No motors have offsets configured!")
        print("Run configure_offsets.py first.")
        sys.exit(1)
    
    print(f"\nMotors with offsets configured: {len(motors_with_offsets)}")
    for motor in motors_with_offsets:
        print(f"  - Motor {motor['id']}: {motor['name']} (offset: {motor['position_offset_deg']:.2f}°)")
    
    if motors_without_offsets:
        print(f"\nMotors without offsets: {len(motors_without_offsets)}")
        for motor in motors_without_offsets:
            print(f"  - Motor {motor['id']}: {motor['name']}")
    
    # Validate each motor with offset
    print("\n" + "=" * 70)
    print("RUNNING VALIDATION TESTS")
    print("=" * 70)
    
    results = {}
    
    for motor in motors_with_offsets:
        motor_id = motor['id']
        motor_name = motor['name']
        offset = motor['position_offset_deg']
        
        # Test 1: Validate offset math
        success = validate_motor(motor_id, motor_name, offset)
        results[motor_id] = success
        
        time.sleep(0.5)
        
        # Test 2: Check if at logical zero
        validate_at_logical_zero(motor_id, motor_name)
        
        time.sleep(0.5)
    
    # Summary
    print("\n" + "=" * 70)
    print("VALIDATION SUMMARY")
    print("=" * 70)
    
    all_passed = all(results.values())
    
    print(f"\nMotors tested: {len(results)}")
    print(f"Passed: {sum(results.values())}")
    print(f"Failed: {len(results) - sum(results.values())}")
    
    if all_passed:
        print("\n✓✓✓ ALL VALIDATIONS PASSED ✓✓✓")
        print("\nYour offsets are configured correctly!")
        print("\nNext steps:")
        print("  1. Re-run calibration: python3 initialize_hip.py")
        print("  2. Normal operation should work now")
    else:
        print("\n✗✗✗ SOME VALIDATIONS FAILED ✗✗✗")
        print("\nTroubleshooting:")
        print("  1. Check if motor moved since configuring offset")
        print("  2. Verify offset values in motor_config.json")
        print("  3. Try reconfiguring: python3 configure_offsets.py")
    
    print("\n" + "=" * 70)


if __name__ == "__main__":
    main()
