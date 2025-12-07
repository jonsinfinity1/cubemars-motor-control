#!/usr/bin/env python3
"""
Move to Starting Position and HOLD

This version moves to starting positions and then HOLDS them with active
impedance control instead of shutting down. This prevents "kick out" from
gravity when motors go limp.

Use this instead of move_to_start.py when you want the motors to stay active.
Press Ctrl+C when you want to shut down.
"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from motor_drivers import CubeMarsDriver
from motor_control import Joint


def load_config():
    """Load motor configuration"""
    config_path = Path(__file__).parent.parent / 'configs' / 'motor_config.json'
    
    if not config_path.exists():
        print(f"✗ Config file not found: {config_path}")
        sys.exit(1)
    
    with open(config_path, 'r') as f:
        return json.load(f)


def move_and_hold(config):
    """
    Move to starting positions and hold indefinitely.
    
    This keeps motors ACTIVE so they don't fall due to gravity.
    """
    print("=" * 70)
    print("MOVE TO START AND HOLD")
    print("=" * 70)
    
    joints = {}
    starting_positions = {}  # Store target positions
    
    try:
        # Initialize all motors
        for motor_config in sorted(config['motors'], key=lambda m: m['id']):
            motor_id = motor_config['id']
            name = motor_config['name']
            starting_pos = motor_config.get('starting_position_deg')
            
            if starting_pos is None:
                print(f"\n{name}: No starting position configured, skipping")
                continue
            
            print(f"\n{name} (ID: {motor_id})")
            print(f"  Target: {starting_pos:.1f}°")
            
            # Create driver and joint
            driver = CubeMarsDriver(motor_id=motor_id)
            joint = Joint(
                driver=driver,
                name=name,
                kp=motor_config['control_params']['default_kp'],
                kd=motor_config['control_params']['default_kd']
            )
            
            # Initialize
            joint.initialize()
            joints[motor_id] = joint
            starting_positions[motor_id] = starting_pos
        
        time.sleep(0.5)
        
        # Move all to starting positions
        print("\n" + "=" * 70)
        print("Moving to starting positions...")
        print("=" * 70 + "\n")
        
        for motor_id, target_pos in starting_positions.items():
            joints[motor_id].move_to(target_pos, duration=3.0, verbose=False)
            print(f"{joints[motor_id].name}: {target_pos:.1f}°")
        
        print(f"\n✓ All motors at starting positions")
        
        # CRITICAL: Read actual positions after movement
        print("\nReading final positions...")
        import math
        actual_positions = {}
        
        for motor_id, joint in joints.items():
            # Query position with low gains (don't command movement)
            joint.driver.send_command(
                position=joint.current_position,
                velocity=0.0,
                kp=0.1,
                kd=0.1,
                torque=0.0
            )
            time.sleep(0.05)
            
            feedback = joint.driver.read_feedback(timeout=0.1)
            if feedback:
                actual_pos = math.degrees(feedback['position'])
                actual_positions[motor_id] = feedback['position']  # Store in radians
                target_pos = starting_positions[motor_id]
                error = actual_pos - target_pos
                
                print(f"  {joint.name}: {actual_pos:.2f}° (error: {error:+.2f}°)")
        
        # Active holding loop - uses ACTUAL positions, not target
        print("\n" + "=" * 70)
        print("HOLDING POSITIONS")
        print("=" * 70)
        print("Motors will actively hold their current positions.")
        print("Press Ctrl+C to shut down.\n")
        
        last_print = time.time()
        
        while True:
            current_time = time.time()
            
            for motor_id, joint in joints.items():
                # Hold at the ACTUAL position where motor ended up
                hold_position = actual_positions.get(motor_id, joint.current_position)
                
                joint.driver.send_command(
                    position=hold_position,  # Use actual position, not target
                    velocity=0.0,
                    kp=20.0,  # Moderate stiffness
                    kd=1.0,
                    torque=0.0
                )
                
                feedback = joint.driver.read_feedback(timeout=0.01)
                if feedback:
                    joint.current_position = feedback['position']
            
            # Print status every 2 seconds
            if current_time - last_print >= 2.0:
                print(f"Holding... (Press Ctrl+C to stop)")
                for motor_id, joint in joints.items():
                    import math
                    pos_deg = math.degrees(joint.current_position)
                    torque = joint.current_torque if hasattr(joint, 'current_torque') else 0
                    print(f"  {joint.name}: {pos_deg:.2f}° (torque: {torque:.3f} Nm)")
                print()
                last_print = current_time
            
            time.sleep(0.01)  # 100Hz control loop
        
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    
    finally:
        print("\nDisabling motors...")
        for joint in joints.values():
            try:
                joint.shutdown()
            except:
                pass
        print("✓ Complete")


def main():
    config = load_config()
    
    print("\n⚠ WARNING: Motors will STAY ACTIVE and hold position")
    print("They will resist movement and draw power continuously.")
    print("Press Ctrl+C when you want to shut down.\n")
    
    response = input("Continue? (yes/no): ")
    if response.lower() not in ['yes', 'y']:
        print("Cancelled")
        return
    
    move_and_hold(config)


if __name__ == "__main__":
    main()
