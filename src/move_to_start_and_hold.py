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
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from motor_drivers import CubeMarsDriver
from motor_control import Joint

# Control loop timing constants
CONTROL_RATE_HZ = 100
CONTROL_PERIOD = 1.0 / CONTROL_RATE_HZ  # 0.01 seconds


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
        
        # Move all to starting positions SIMULTANEOUSLY
        print("\n" + "=" * 70)
        print("Moving to starting positions...")
        print("=" * 70 + "\n")
        
        duration = 3.0
        
        # Read all current positions
        start_positions = {}
        for motor_id, joint in joints.items():
            joint._update_state()
            start_positions[motor_id] = joint.current_position
            target_deg = starting_positions[motor_id]
            current_deg = math.degrees(start_positions[motor_id])
            print(f"{joint.name}:")
            print(f"  Current: {current_deg:.2f}°")
            print(f"  Target:  {target_deg:.2f}°")
            print(f"  Distance: {abs(current_deg - target_deg):.2f}°")
        
        # Convert target positions to radians
        target_positions_rad = {motor_id: math.radians(deg) 
                                for motor_id, deg in starting_positions.items()}
        
        print("\nMoving all motors simultaneously...")
        
        # Timing setup for deadline-based loop
        loop_start = time.perf_counter()
        next_deadline = loop_start
        overrun_count = 0
        max_overrun = 0.0
        iteration_count = 0
        
        # Simultaneous S-curve movement with deadline-based timing
        while True:
            iteration_count += 1
            now = time.perf_counter()
            elapsed = now - loop_start
            
            if elapsed >= duration:
                # Final positions
                for motor_id, joint in joints.items():
                    target_rad = target_positions_rad[motor_id]
                    joint.driver.send_command(
                        position=target_rad,
                        velocity=0.0,
                        kp=joint.default_kp,
                        kd=joint.default_kd,
                        torque=0.0
                    )
                    feedback = joint.driver.read_feedback(timeout=0.005)
                    if feedback:
                        joint.current_position = feedback['position']
                break
            else:
                # S-curve trajectory for all motors
                progress = 0.5 - 0.5 * math.cos((elapsed / duration) * math.pi)
                
                for motor_id, joint in joints.items():
                    start_pos = start_positions[motor_id]
                    target_rad = target_positions_rad[motor_id]
                    position = start_pos + (target_rad - start_pos) * progress
                    
                    joint.driver.send_command(
                        position=position,
                        velocity=0.0,
                        kp=joint.default_kp,
                        kd=joint.default_kd,
                        torque=0.0
                    )
                    
                    feedback = joint.driver.read_feedback(timeout=0.005)
                    if feedback:
                        joint.current_position = feedback['position']
            
            # Deadline-based timing
            next_deadline += CONTROL_PERIOD
            sleep_time = next_deadline - time.perf_counter()
            
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Overrun detected
                overrun_count += 1
                max_overrun = max(max_overrun, -sleep_time)
                next_deadline = time.perf_counter()  # Reset to avoid cascading
        
        print(f"\n✓ All motors at starting positions")
        if overrun_count > 0:
            print(f"  Movement timing: {overrun_count}/{iteration_count} overruns "
                  f"(max: {max_overrun*1000:.1f}ms)")
        
        # CRITICAL: Read actual positions after movement
        print("\nReading final positions...")
        actual_positions = {}
        
        for motor_id, joint in joints.items():
            # Query position with ZERO gains (don't move!)
            joint.driver.send_command(
                position=0,       # Ignored when kp=0
                velocity=0.0,
                kp=0,             # ZERO - just reading
                kd=0,             # ZERO - just reading
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
        
        # Timing setup for holding loop
        next_deadline = time.perf_counter()
        last_print_time = next_deadline
        hold_overrun_count = 0
        hold_iteration_count = 0
        
        while True:
            hold_iteration_count += 1
            now = time.perf_counter()
            
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
                
                feedback = joint.driver.read_feedback(timeout=0.005)
                if feedback:
                    joint.current_position = feedback['position']
                    joint.current_torque = feedback.get('torque', 0.0)
            
            # Print status every 2 seconds
            if (now - last_print_time) >= 2.0:
                print(f"Holding... (Press Ctrl+C to stop)")
                for motor_id, joint in joints.items():
                    pos_deg = math.degrees(joint.current_position)
                    torque = getattr(joint, 'current_torque', 0.0)
                    print(f"  {joint.name}: {pos_deg:.2f}° (torque: {torque:.3f} Nm)")
                if hold_overrun_count > 0:
                    print(f"  Timing: {hold_overrun_count}/{hold_iteration_count} overruns")
                print()
                last_print_time = now
                # Reset overrun counter each print cycle
                hold_overrun_count = 0
                hold_iteration_count = 0
            
            # Deadline-based timing
            next_deadline += CONTROL_PERIOD
            sleep_time = next_deadline - time.perf_counter()
            
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                hold_overrun_count += 1
                next_deadline = time.perf_counter()
        
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
