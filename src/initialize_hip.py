#!/usr/bin/env python3
"""
Hip Assembly Initialization Script

This script performs initial calibration of the hip assembly by:
1. Reading motor configuration from a JSON file
2. Initializing each motor sequentially (lowest ID to highest)
3. Discovering the range of motion for each joint
4. Saving calibration results for future use

Robotics Context:
----------------
This is a calibration routine - critical for any multi-joint robotic system.
You need to know each joint's physical limits before attempting coordinated
motion. Sequential discovery (one motor at a time) is safer than simultaneous
discovery, as it prevents unexpected interactions between joints.

Python Context:
--------------
This demonstrates:
- JSON configuration file parsing
- Error handling for hardware operations
- Sequential processing with error recovery
- File I/O for saving calibration data
- Logging for debugging
"""

import json
import time
import sys
import os
from datetime import datetime
from pathlib import Path

# Add src directory to path for imports
# Python note: This is a common pattern for project organization
# Java equivalent would be package imports
sys.path.insert(0, str(Path(__file__).parent))

from motor_drivers import CubeMarsDriver
from motor_control import Joint


class HipInitializer:
    """
    Manages initialization and calibration of multi-motor hip assembly.
    
    This class orchestrates the entire calibration process:
    - Reads configuration
    - Manages multiple joints
    - Handles partial failures gracefully
    - Saves calibration results
    
    Design Pattern:
    --------------
    This is a Facade pattern - it provides a simple interface to the
    complex process of initializing multiple motors. In Java you'd
    organize this similarly with a high-level coordinator class.
    """
    
    def __init__(self, config_path):
        """
        Initialize the hip calibrator.
        
        Args:
            config_path: Path to motor_config.json file
        """
        self.config_path = Path(config_path)
        self.config = None
        self.joints = {}  # Dictionary: motor_id -> Joint object
        self.calibration_results = {}  # Dictionary: motor_id -> range data
        self.failures = []  # List of (motor_id, error_message) tuples
        
        # Load configuration
        self._load_config()
    
    def _load_config(self):
        """
        Load and parse the motor configuration file.
        
        Python Note:
        -----------
        The 'with' statement is Python's try-with-resources equivalent.
        It ensures the file is properly closed even if an exception occurs.
        
        Java equivalent:
            try (FileReader reader = new FileReader(path)) {
                // use reader
            } // auto-closed
        """
        try:
            with open(self.config_path, 'r') as f:
                self.config = json.load(f)
            print(f"✓ Loaded configuration from {self.config_path}")
            print(f"  Found {len(self.config['motors'])} motors")
        except FileNotFoundError:
            print(f"✗ Error: Config file not found: {self.config_path}")
            sys.exit(1)
        except json.JSONDecodeError as e:
            print(f"✗ Error: Invalid JSON in config file: {e}")
            sys.exit(1)
    
    def initialize_all_joints(self):
        """
        Initialize all motors in configuration.
        
        Creates Joint objects for each motor but doesn't enter motor mode yet.
        This separates object creation from hardware initialization.
        
        Robotics Safety:
        ---------------
        We create all the software objects first, then enable hardware
        sequentially. This allows us to validate the configuration before
        touching any hardware.
        """
        print("\n" + "=" * 70)
        print("INITIALIZING JOINT OBJECTS")
        print("=" * 70)
        
        # Sort motors by ID (lowest to highest)
        motors = sorted(self.config['motors'], key=lambda m: m['id'])
        
        for motor_config in motors:
            motor_id = motor_config['id']
            name = motor_config['name']
            
            try:
                print(f"\nCreating joint: {name} (ID: {motor_id})")
                
                # Create motor driver with offset support
                driver = CubeMarsDriver.from_config(motor_id=motor_id)
                
                # Create high-level joint controller
                joint = Joint(
                    driver=driver,
                    name=name,
                    kp=motor_config['control_params']['default_kp'],
                    kd=motor_config['control_params']['default_kd']
                )
                
                # Store for later use
                self.joints[motor_id] = joint
                print(f"  ✓ Joint object created successfully")
                
            except Exception as e:
                error_msg = f"Failed to create joint {name} (ID: {motor_id}): {e}"
                print(f"  ✗ {error_msg}")
                self.failures.append((motor_id, error_msg))
                
                # Check if we've exceeded max failures
                max_failures = self.config['safety']['max_sequential_failures']
                if len(self.failures) >= max_failures:
                    print(f"\n✗ Exceeded maximum failures ({max_failures}). Aborting.")
                    raise RuntimeError("Too many initialization failures")
        
        if self.joints:
            print(f"\n✓ Successfully created {len(self.joints)} joint objects")
        else:
            print("\n✗ No joints were successfully created")
            sys.exit(1)
    
    def discover_all_ranges(self):
        """
        Discover range of motion for each joint sequentially.
        
        Robotics Context:
        ----------------
        Sequential discovery is important because:
        1. One joint at a time reduces mechanical interference
        2. You can monitor each joint's behavior individually
        3. If one joint fails, others can still be calibrated
        4. Total power draw is lower (only one motor moving at a time)
        
        For a hip assembly with 2 DOF per side, this means we discover:
        1. Hip abduction range (motor moves side-to-side)
        2. Hip flexion range (motor moves forward-back)
        
        The order matters - we do abduction first because it typically
        has fewer mechanical constraints than flexion.
        """
        print("\n" + "=" * 70)
        print("RANGE OF MOTION DISCOVERY")
        print("=" * 70)
        print("\nDiscovering ranges sequentially from lowest ID to highest...")
        print("This will take several minutes. Do not interrupt!\n")
        
        # Get motor configs sorted by ID
        motors = sorted(self.config['motors'], key=lambda m: m['id'])
        
        for i, motor_config in enumerate(motors, 1):
            motor_id = motor_config['id']
            
            # Skip if joint wasn't created successfully
            if motor_id not in self.joints:
                print(f"\nSkipping motor ID {motor_id} (failed to initialize)")
                continue
            
            joint = self.joints[motor_id]
            name = motor_config['name']
            
            print(f"\n{'─' * 70}")
            print(f"Motor {i}/{len(motors)}: {name} (ID: {motor_id})")
            print(f"{'─' * 70}")
            
            try:
                # Initialize this specific joint (enter motor mode)
                print(f"Enabling motor control...")
                joint.initialize()
                time.sleep(0.5)
                
                # Get discovery parameters from config
                params = motor_config['discovery_params']
                
                # Discover range of motion
                print(f"\nStarting range discovery...")
                print(f"  Torque threshold: {params['torque_threshold']} Nm")
                print(f"  Exploration speed: {params['creep_velocity']} rad/s")
                
                min_deg, max_deg = joint.discover_range(
                    torque_threshold=params['torque_threshold'],
                    creep_velocity=params['creep_velocity'],
                    max_exploration_time=params['max_exploration_time'],
                    verbose=True
                )
                
                # Get safe range (with margins)
                safe_min, safe_max, center, total_range = joint.get_safe_range()
                
                # Store results
                self.calibration_results[motor_id] = {
                    'name': name,
                    'joint_type': motor_config['joint_type'],
                    'dof': motor_config['dof'],
                    'side': motor_config.get('side', 'unknown'),
                    'min_deg': min_deg,
                    'max_deg': max_deg,
                    'safe_min_deg': safe_min,
                    'safe_max_deg': safe_max,
                    'center_deg': center,
                    'total_range_deg': total_range,
                    'timestamp': datetime.now().isoformat()
                }
                
                print(f"\n✓ Range discovery complete for {name}")
                print(f"  Full range: {min_deg:.2f}° to {max_deg:.2f}°")
                print(f"  Safe range: {safe_min:.2f}° to {safe_max:.2f}°")
                print(f"  Center: {center:.2f}°")
                
                # Brief pause before next motor
                # This gives you time to see the results and lets the motor settle
                if i < len(motors):
                    print(f"\nPausing before next motor...")
                    time.sleep(2.0)
                
            except KeyboardInterrupt:
                print(f"\n\n⚠ Range discovery interrupted by user")
                print(f"Motor {name} (ID: {motor_id}) was not fully calibrated")
                raise
                
            except Exception as e:
                error_msg = f"Range discovery failed for {name} (ID: {motor_id}): {e}"
                print(f"\n✗ {error_msg}")
                self.failures.append((motor_id, error_msg))
                
                # Decide whether to continue or abort
                max_failures = self.config['safety']['max_sequential_failures']
                if len(self.failures) >= max_failures:
                    print(f"\n✗ Exceeded maximum failures ({max_failures}). Aborting.")
                    raise RuntimeError("Too many discovery failures")
                else:
                    print(f"Continuing with remaining motors...")
            
            finally:
                # Always shutdown this joint before moving to next
                # This ensures the motor is in a safe state
                try:
                    joint.shutdown()
                except:
                    pass
        
        # Summary
        print("\n" + "=" * 70)
        print("RANGE DISCOVERY SUMMARY")
        print("=" * 70)
        print(f"Successful: {len(self.calibration_results)}/{len(motors)} motors")
        print(f"Failed: {len(self.failures)}/{len(motors)} motors")
        
        if self.calibration_results:
            print("\nCalibrated motors:")
            for motor_id, result in sorted(self.calibration_results.items()):
                print(f"  [{motor_id}] {result['name']}: "
                      f"{result['safe_min_deg']:.1f}° to {result['safe_max_deg']:.1f}° "
                      f"(center: {result['center_deg']:.1f}°)")
        
        if self.failures:
            print("\nFailed motors:")
            for motor_id, error in self.failures:
                print(f"  [{motor_id}] {error}")
    
    def save_calibration_results(self, output_path=None):
        """
        Save calibration results to a JSON file.
        
        This creates a calibration file that can be loaded by your main
        control software. This way, you only need to calibrate once
        (unless you change the mechanical setup).
        
        Args:
            output_path: Path to save results. If None, uses default location.
        """
        if not self.calibration_results:
            print("\n⚠ No calibration results to save")
            return
        
        # Default output path
        if output_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = Path(__file__).parent.parent / 'configs' / f'calibration_{timestamp}.json'
        else:
            output_path = Path(output_path)
        
        # Create output structure
        output_data = {
            'calibration_date': datetime.now().isoformat(),
            'config_file': str(self.config_path),
            'motors': self.calibration_results,
            'failures': [
                {'motor_id': motor_id, 'error': error}
                for motor_id, error in self.failures
            ]
        }
        
        try:
            # Ensure directory exists
            output_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Write JSON with pretty formatting
            # Python note: indent=2 makes the JSON human-readable
            with open(output_path, 'w') as f:
                json.dump(output_data, f, indent=2)
            
            print(f"\n✓ Calibration results saved to: {output_path}")
            
        except Exception as e:
            print(f"\n✗ Failed to save calibration results: {e}")
    
    def move_to_starting_positions(self):
        """
        Move all motors to their configured starting positions.
        
        This is called after calibration to leave the robot in a known,
        comfortable pose rather than at the geometric midpoint.
        
        Note: Creates fresh driver instances because the original drivers
        were shut down after range discovery.
        
        Robotics Note:
        -------------
        Starting positions are often different from range midpoints because:
        - Midpoint might be mechanically unstable
        - Midpoint might not be a natural/comfortable pose
        - You want a specific "ready" or "home" position
        
        For example, a standing humanoid might have:
        - Hip abduction at 0° (leg straight down)
        - Hip flexion at -10° (leg slightly back for stability)
        """
        print("\n" + "=" * 70)
        print("MOVING TO STARTING POSITIONS")
        print("=" * 70)
        
        motors = sorted(self.config['motors'], key=lambda m: m['id'])
        active_joints = {}  # Fresh joints for this phase
        
        for motor_config in motors:
            motor_id = motor_config['id']
            
            # Skip if not calibrated
            if motor_id not in self.calibration_results:
                continue
            
            name = motor_config['name']
            starting_pos = motor_config.get('starting_position_deg')
            
            if starting_pos is None:
                print(f"\n{name}: No starting position configured, skipping")
                continue
            
            print(f"\n{name}:")
            print(f"  Starting position: {starting_pos:.1f}°")
            
            # Get safe range from calibration
            result = self.calibration_results[motor_id]
            safe_min = result['safe_min_deg']
            safe_max = result['safe_max_deg']
            
            # Validate starting position is within safe range
            if starting_pos < safe_min or starting_pos > safe_max:
                print(f"  ⚠ WARNING: Starting position {starting_pos:.1f}° is outside "
                      f"safe range [{safe_min:.1f}°, {safe_max:.1f}°]")
                print(f"  Using center position {result['center_deg']:.1f}° instead")
                starting_pos = result['center_deg']
            
            try:
                # Create FRESH driver (old one was closed after discovery)
                driver = CubeMarsDriver.from_config(motor_id=motor_id)
                
                # Create fresh joint with the new driver
                joint = Joint(
                    driver=driver,
                    name=name,
                    kp=motor_config['control_params']['default_kp'],
                    kd=motor_config['control_params']['default_kd']
                )
                
                # Initialize this joint
                joint.initialize()
                time.sleep(0.3)
                
                # Set discovered range so joint enforces limits
                import math
                joint.min_position = math.radians(result['min_deg'])
                joint.max_position = math.radians(result['max_deg'])
                joint.range_discovered = True
                
                # Store for holding phase
                active_joints[motor_id] = joint
                
                # Move to starting position
                joint.move_to(starting_pos, duration=3.0, verbose=False)
                print(f"  ✓ Moved to {starting_pos:.1f}°")
                
            except Exception as e:
                print(f"  ✗ Failed to move to starting position: {e}")
                import traceback
                traceback.print_exc()
        
        if not active_joints:
            print("\n⚠ No motors successfully moved to starting positions")
            return
        
        # Hold briefly then shut down
        print("\nHolding positions for 2 seconds...")
        
        # Store actual positions (not targets) for holding
        import math
        hold_positions = {}
        for motor_id, joint in active_joints.items():
            hold_positions[motor_id] = joint.current_position  # Use actual position
        
        start_time = time.time()
        while time.time() - start_time < 2.0:
            for motor_id, joint in active_joints.items():
                if motor_id in hold_positions:
                    try:
                        joint.driver.send_command(
                            position=hold_positions[motor_id],
                            velocity=0.0,
                            kp=20.0,
                            kd=1.0,
                            torque=0.0
                        )
                        joint.driver.read_feedback(timeout=0.01)
                    except:
                        pass
            time.sleep(0.01)
        
        # Shutdown fresh joints
        print("\nShutting down motors...")
        for motor_id, joint in active_joints.items():
            try:
                joint.shutdown()
            except Exception as e:
                print(f"  Warning: Failed to shutdown motor {motor_id}: {e}")
        
        print("\n✓ All motors at starting positions")
    
    def run_full_initialization(self):
        """
        Run the complete initialization sequence.
        
        This is the main entry point that coordinates everything:
        1. Initialize joint objects
        2. Discover ranges
        3. Save results
        4. Move to starting positions
        
        Returns:
            dict: Calibration results
        """
        print("\n" + "=" * 70)
        print("HIP ASSEMBLY INITIALIZATION")
        print("=" * 70)
        print(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        start_time = time.time()
        
        try:
            # Step 1: Create joint objects
            self.initialize_all_joints()
            
            # Step 2: Discover ranges
            self.discover_all_ranges()
            
            # Step 3: Save results
            self.save_calibration_results()
            
            # Step 4: Move to starting positions
            if self.calibration_results:
                self.move_to_starting_positions()
            
            elapsed = time.time() - start_time
            print(f"\n✓ Initialization complete in {elapsed:.1f} seconds")
            
            return self.calibration_results
            
        except KeyboardInterrupt:
            print("\n\n⚠ Initialization interrupted by user")
            print("Saving partial results...")
            self.save_calibration_results()
            return self.calibration_results
            
        except Exception as e:
            print(f"\n✗ Initialization failed: {e}")
            import traceback
            traceback.print_exc()
            
            # Try to save whatever we got
            if self.calibration_results:
                print("\nAttempting to save partial results...")
                self.save_calibration_results()
            
            raise
        
        finally:
            # Ensure all motors are safely shut down
            print("\nShutting down all motors...")
            for motor_id, joint in self.joints.items():
                try:
                    joint.shutdown()
                except Exception as e:
                    print(f"  Warning: Failed to shutdown motor {motor_id}: {e}")


def main():
    """
    Main entry point for the initialization script.
    
    Python Note:
    -----------
    This is the standard pattern for executable Python scripts.
    The if __name__ == "__main__" check ensures this only runs
    when the script is executed directly, not when imported as a module.
    
    In Java, this would be your public static void main(String[] args) method.
    """
    # Determine config file path
    # This script assumes it's in the src directory
    script_dir = Path(__file__).parent
    config_path = script_dir.parent / 'configs' / 'motor_config.json'
    
    print("=" * 70)
    print("HIP ASSEMBLY INITIALIZATION SCRIPT")
    print("=" * 70)
    print(f"\nConfiguration file: {config_path}")
    
    if not config_path.exists():
        print(f"\n✗ Error: Configuration file not found!")
        print(f"Expected location: {config_path}")
        print(f"\nPlease create motor_config.json in the configs directory")
        sys.exit(1)
    
    # Confirm before starting
    print("\nThis script will:")
    print("  1. Read motor configuration")
    print("  2. Initialize each motor sequentially")
    print("  3. Discover range of motion (will move motors!)")
    print("  4. Save calibration results")
    print("\n⚠ WARNING: Motors will move during range discovery!")
    print("Ensure the hip assembly is:")
    print("  - Securely mounted")
    print("  - Free to move through full range")
    print("  - Clear of obstructions")
    print("  - Properly powered")
    
    response = input("\nProceed with initialization? (yes/no): ")
    if response.lower() not in ['yes', 'y']:
        print("Initialization cancelled.")
        sys.exit(0)
    
    # Run initialization
    initializer = HipInitializer(config_path)
    results = initializer.run_full_initialization()
    
    print("\n" + "=" * 70)
    print("INITIALIZATION COMPLETE")
    print("=" * 70)
    print(f"\nCalibration data has been saved and is ready to use.")
    print(f"You can now run your main control software with these calibrated ranges.")


if __name__ == "__main__":
    main()
