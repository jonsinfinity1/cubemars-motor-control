#!/usr/bin/env python3
"""
Query Current Positions

Quick utility to read and display current positions of all configured motors.
Useful for finding good starting positions by manually positioning the robot
and then reading where the motors ended up.

Usage:
1. Position robot manually (motors can be in idle mode)
2. Run this script
3. Note the positions
4. Update motor_config.json with these values
"""

import json
import sys
import time
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from motor_drivers import CubeMarsDriver


def query_motor_position(motor_id, motor_name):
    """
    Query a single motor's current position.
    
    Uses very low gains so it doesn't try to move the motor,
    just queries where it currently is.
    """
    try:
        driver = CubeMarsDriver(motor_id=motor_id)
        driver.flush_buffer()
        driver.enter_motor_mode()
        time.sleep(0.2)
        
        # Query with very low gains (basically just reading)
        driver.send_command(
            position=0.0,
            velocity=0.0,
            kp=0.1,  # Very low - won't try to move
            kd=0.1,
            torque=0.0
        )
        time.sleep(0.1)
        
        feedback = driver.read_feedback(timeout=0.2)
        
        driver.close()
        
        if feedback:
            pos_deg = math.degrees(feedback['position'])
            return pos_deg
        else:
            return None
            
    except Exception as e:
        print(f"  Error reading motor {motor_id}: {e}")
        return None


def main():
    """Query all configured motors"""
    
    # Load config
    config_path = Path(__file__).parent.parent / 'configs' / 'motor_config.json'
    
    if not config_path.exists():
        print(f"Config file not found: {config_path}")
        sys.exit(1)
    
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    print("=" * 70)
    print("QUERY CURRENT MOTOR POSITIONS")
    print("=" * 70)
    print("\nThis script reads the current position of each configured motor.")
    print("You can use these values to set starting positions in motor_config.json\n")
    
    motors = sorted(config['motors'], key=lambda m: m['id'])
    
    print("Querying motors...\n")
    
    results = {}
    
    for motor_config in motors:
        motor_id = motor_config['id']
        name = motor_config['name']
        
        print(f"Motor {motor_id} ({name}):", end=" ")
        
        position = query_motor_position(motor_id, name)
        
        if position is not None:
            results[motor_id] = {
                'name': name,
                'position': position
            }
            print(f"{position:7.2f}°")
        else:
            print("Failed to read")
        
        time.sleep(0.3)  # Brief pause between motors
    
    # Display results
    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)
    
    if not results:
        print("\nNo motors successfully queried")
        return
    
    print("\nCurrent positions:")
    for motor_id, data in sorted(results.items()):
        print(f"  [{motor_id}] {data['name']:25s}: {data['position']:7.2f}°")
    
    # Generate config snippet
    print("\n" + "=" * 70)
    print("CONFIG SNIPPET")
    print("=" * 70)
    print("\nYou can copy these values into motor_config.json:\n")
    
    for motor_id, data in sorted(results.items()):
        print(f'  "starting_position_deg": {data["position"]:.1f},  // {data["name"]}')
    
    # Offer to update config
    print("\n" + "=" * 70)
    response = input("\nUpdate motor_config.json with these positions? (yes/no): ")
    
    if response.lower() in ['yes', 'y']:
        # Update config
        for motor_config in config['motors']:
            motor_id = motor_config['id']
            if motor_id in results:
                motor_config['starting_position_deg'] = round(results[motor_id]['position'], 1)
        
        # Save
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)
        
        print(f"✓ Updated {config_path}")
        print("  Run 'python3 move_to_start.py' to test the new positions")
    else:
        print("Config not updated")


if __name__ == "__main__":
    main()
