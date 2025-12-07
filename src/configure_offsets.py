#!/usr/bin/env python3
"""
Configure Motor Offsets

Instead of trying to set the motor's internal zero position (which may not
work reliably on all CubeMars motors), this script helps you configure
SOFTWARE offsets that translate between motor positions and logical positions.

This is actually MORE FLEXIBLE than hardware zero because:
- You can change it anytime without touching the motor
- It's stored in your config file (version controlled)
- You can have different offsets for different configurations
- No risk of corrupting motor memory

How it works:
1. Position motor where you want logical "zero" to be
2. Read the motor's actual position (e.g., 41.10°)
3. Store this as an offset in config
4. Software subtracts offset: logical_pos = motor_pos - offset
"""

import sys
import time
import math
import json
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from motor_drivers import CubeMarsDriver


def configure_offset_for_motor(motor_id, motor_name, config):
    """
    Configure software offset for a motor.
    """
    print("\n" + "=" * 70)
    print(f"CONFIGURING OFFSET FOR: {motor_name} (ID: {motor_id})")
    print("=" * 70)
    
    print("\nSTEP 1: Position the motor")
    print("-" * 70)
    print("Manually move the motor to where you want LOGICAL ZERO to be.")
    print()
    print("For hip flexion: Leg vertical (straight down)")
    print("For hip abduction: Leg straight down (not angled to side)")
    print()
    
    response = input("Is motor at desired logical zero position? (yes/no): ")
    if response.lower() not in ['yes', 'y']:
        print("Cancelled")
        return None
    
    print("\nSTEP 2: Read motor's actual position")
    print("-" * 70)
    
    try:
        driver = CubeMarsDriver(motor_id=motor_id)
        driver.flush_buffer()
        driver.enter_motor_mode()
        time.sleep(0.3)
        
        # FIRST READ: Get current position with zero torque command
        # This doesn't try to move anywhere - just queries current state
        driver.send_command(
            position=0,      # Ignored when torque=0
            velocity=0, 
            kp=0,            # No position control
            kd=0,            # No damping
            torque=0         # Zero torque = don't move
        )
        time.sleep(0.1)
        feedback = driver.read_feedback(timeout=0.2)
        
        if not feedback:
            print("✗ Could not read motor position")
            driver.close()
            return None
        
        current_position = feedback['position']
        motor_position_deg = math.degrees(current_position)
        print(f"\nMotor's current raw position: {motor_position_deg:.2f}°")
        
        # SECOND COMMAND: Hold current position with low gains
        # This keeps the motor stable at current position
        print("Holding current position for verification...")
        driver.send_command(
            position=current_position,  # Stay where you are
            velocity=0,
            kp=5.0,                     # Low gain for gentle hold
            kd=0.5,
            torque=0
        )
        time.sleep(0.5)
        
        # Verify position hasn't drifted
        feedback = driver.read_feedback(timeout=0.2)
        if feedback:
            verified_position_deg = math.degrees(feedback['position'])
            drift = abs(verified_position_deg - motor_position_deg)
            if drift > 1.0:
                print(f"⚠ Warning: Position drifted {drift:.2f}° during verification")
                print(f"  Original: {motor_position_deg:.2f}°, Now: {verified_position_deg:.2f}°")
                # Use the more recent reading
                motor_position_deg = verified_position_deg
        
        driver.close()
        
    except Exception as e:
        print(f"✗ Error reading motor: {e}")
        return None
    
    print("\nSTEP 3: Configure offset")
    print("-" * 70)
    print(f"\nOffset will be: {motor_position_deg:.2f}°")
    print(f"\nThis means:")
    print(f"  Motor reads:  {motor_position_deg:.2f}° → Software sees: 0.00°")
    print(f"  Motor reads:  {motor_position_deg + 10:.2f}° → Software sees: 10.00°")
    print(f"  Motor reads:  {motor_position_deg - 10:.2f}° → Software sees: -10.00°")
    print()
    
    response = input("Apply this offset? (yes/no): ")
    if response.lower() not in ['yes', 'y']:
        print("Cancelled")
        return None
    
    return motor_position_deg


def main():
    """
    Main workflow: configure offsets for all motors
    """
    config_path = Path(__file__).parent.parent / 'configs' / 'motor_config.json'
    
    if not config_path.exists():
        print(f"✗ Config file not found: {config_path}")
        sys.exit(1)
    
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    print("=" * 70)
    print("SOFTWARE OFFSET CONFIGURATION")
    print("=" * 70)
    print("\nThis script will help you configure position offsets for your motors.")
    print("Offsets allow the software to translate between motor positions")
    print("and logical positions without changing the motor's internal zero.")
    
    print("\n" + "=" * 70)
    print("AVAILABLE MOTORS")
    print("=" * 70)
    
    motors = sorted(config['motors'], key=lambda m: m['id'])
    
    for motor in motors:
        current_offset = motor.get('position_offset_deg', 0.0)
        print(f"\nMotor ID {motor['id']}: {motor['name']}")
        print(f"  Current offset: {current_offset:.2f}°")
    
    print("\n" + "=" * 70)
    
    # Configure offsets for each motor
    offsets_updated = {}
    
    for motor in motors:
        motor_id = motor['id']
        motor_name = motor['name']
        
        response = input(f"\nConfigure offset for {motor_name}? (yes/no/skip): ")
        
        if response.lower() in ['yes', 'y']:
            offset = configure_offset_for_motor(motor_id, motor_name, config)
            if offset is not None:
                offsets_updated[motor_id] = offset
        elif response.lower() == 'skip':
            print(f"Skipping {motor_name}")
            continue
        else:
            print(f"Skipping {motor_name}")
    
    if not offsets_updated:
        print("\nNo offsets configured. Exiting.")
        return
    
    # Update config file
    print("\n" + "=" * 70)
    print("UPDATING CONFIGURATION")
    print("=" * 70)
    
    for motor in config['motors']:
        if motor['id'] in offsets_updated:
            motor['position_offset_deg'] = round(offsets_updated[motor['id']], 2)
            print(f"\nMotor ID {motor['id']}: {motor['name']}")
            print(f"  Offset set to: {motor['position_offset_deg']:.2f}°")
    
    # Save config
    print(f"\nSaving configuration to: {config_path}")
    
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)
    
    print("\n✓ Configuration updated successfully!")
    
    print("\n" + "=" * 70)
    print("NEXT STEPS")
    print("=" * 70)
    print("\n1. Test with query_positions.py")
    print("   → Should now show ~0° when motor is at logical zero")
    print()
    print("2. Update your motor driver to apply offsets:")
    print("   → In CubeMarsDriver.read_feedback(), subtract offset")
    print("   → In CubeMarsDriver.send_command(), add offset")
    print()
    print("3. Re-run calibration (initialize_hip.py)")
    print("   → Discovers ranges in logical (offset-corrected) coordinates")
    
    print("\n" + "=" * 70)
    print("EXAMPLE CODE TO APPLY OFFSET")
    print("=" * 70)
    print("""
# In your Joint or control code:

import json
with open('configs/motor_config.json', 'r') as f:
    config = json.load(f)

# Get offset for motor ID 2
motor_2_offset = None
for motor in config['motors']:
    if motor['id'] == 2:
        motor_2_offset = math.radians(motor.get('position_offset_deg', 0.0))
        break

# When reading position:
raw_position = feedback['position']
logical_position = raw_position - motor_2_offset

# When commanding position:
logical_target = math.radians(-10.0)  # Want to go to -10°
motor_target = logical_target + motor_2_offset  # Add offset for motor
driver.send_command(motor_target, ...)
""")


if __name__ == "__main__":
    main()
