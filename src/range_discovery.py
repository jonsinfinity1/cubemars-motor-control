#!/usr/bin/env python3
"""
Range of Motion Discovery and Settling
Slowly explores joint limits using torque feedback, then settles at max position
"""

import time
import math
from motors.cubemars_motor import CubeMarsMotor


class RangeDiscovery:
    """
    Discovers the range of motion for a joint by slowly moving until
    physical stops are detected via torque feedback.
    """

    def __init__(self, motor, torque_threshold=0.5, creep_velocity=0.1):
        """
        Args:
            motor: CubeMarsMotor instance
            torque_threshold: Torque (Nm) that indicates hitting a stop
            creep_velocity: Slow exploration speed (rad/s)
        """
        self.motor = motor
        self.torque_threshold = torque_threshold
        self.creep_velocity = creep_velocity

        # Discovery results
        self.min_position = None
        self.max_position = None
        self.current_position = 0.0

    def smooth_move_to_position(self, target_position, duration=2.0, kp=20.0, kd=1.0):
        """
        Move smoothly to a target position using S-curve trajectory.

        Args:
            target_position: Target position in radians
            duration: Time to complete the move (seconds)
            kp: Position gain for impedance control
            kd: Damping gain for impedance control

        This demonstrates trajectory generation - a key robotics concept.
        The S-curve (using cosine) provides smooth acceleration/deceleration.
        """
        # Get current position from motor feedback
        self.motor.send_command(position=self.current_position, velocity=0.0,
                                kp=kp, kd=kd, torque=0.0)
        time.sleep(0.05)

        feedback = self.motor.read_feedback(timeout=0.1)
        if feedback:
            start_position = feedback['position']
        else:
            start_position = self.current_position

        start_time = time.time()

        while True:
            elapsed = time.time() - start_time

            if elapsed >= duration:
                # Ensure we end exactly at target
                position = target_position
                done = True
            else:
                # S-curve blending: 0.5 - 0.5*cos gives smooth 0->1 transition
                # This is a common trajectory planning technique
                progress = 0.5 - 0.5 * math.cos((elapsed / duration) * math.pi)
                position = start_position + (target_position - start_position) * progress
                done = False

            # Use impedance control (low stiffness) for compliance
            self.motor.send_command(position=position, velocity=0.0,
                                    kp=kp, kd=kd, torque=0.0)

            feedback = self.motor.read_feedback(timeout=0.01)
            if feedback:
                current_deg = math.degrees(feedback['position'])
                target_deg = math.degrees(position)
                print(f"Moving: Target={target_deg:6.2f}°  Current={current_deg:6.2f}°  "
                      f"Torque={feedback['torque']:5.3f} Nm")
                self.current_position = feedback['position']

            if done:
                break

            time.sleep(0.01)  # 100Hz control loop

    def explore_direction(self, direction, max_exploration_time=10.0):
        """
        Slowly move in one direction until hitting a physical stop.

        Args:
            direction: 1 for positive, -1 for negative
            max_exploration_time: Safety timeout (seconds)

        Returns:
            Position where stop was detected (radians)

        This demonstrates force-sensitive exploration - the robot uses
        torque feedback to detect when it's hit something.
        """
        print(f"\nExploring {'positive' if direction > 0 else 'negative'} direction...")
        print(f"Will stop when torque exceeds {self.torque_threshold} Nm")

        start_time = time.time()
        stop_detected = False
        stop_position = None

        # Very compliant control - we want to feel the stop, not fight it
        kp = 5.0  # Low stiffness
        kd = 0.5  # Light damping

        while not stop_detected and (time.time() - start_time) < max_exploration_time:
            # Command a slow velocity in the exploration direction
            # Note: We're using velocity control with low impedance
            self.motor.send_command(
                position=self.current_position + (direction * 0.1),  # Slight position bias
                velocity=direction * self.creep_velocity,
                kp=kp,
                kd=kd,
                torque=0.0
            )

            feedback = self.motor.read_feedback(timeout=0.02)

            if feedback:
                torque_magnitude = abs(feedback['torque'])
                current_deg = math.degrees(feedback['position'])

                # Check if we've hit a stop
                if torque_magnitude >= self.torque_threshold:
                    stop_detected = True
                    stop_position = feedback['position']
                    print(f"\n*** Stop detected at {current_deg:.2f}° "
                          f"(Torque: {feedback['torque']:.3f} Nm) ***")
                else:
                    # Print progress every 0.25 seconds
                    if int((time.time() - start_time) * 4) % 1 == 0:
                        print(f"Position: {current_deg:6.2f}°  "
                              f"Torque: {feedback['torque']:5.3f} Nm  "
                              f"Exploring...")

                self.current_position = feedback['position']

            time.sleep(0.05)  # 20Hz exploration (slow and gentle)

        if not stop_detected:
            print(f"Warning: No stop detected after {max_exploration_time}s")
            stop_position = self.current_position

        # Back off slightly from the hard stop
        # This demonstrates safety margins in robotics
        backoff_distance = math.radians(2)  # 2 degrees
        safe_position = stop_position - (direction * backoff_distance)

        print(f"Backing off {math.degrees(backoff_distance):.1f}° from stop...")
        self.smooth_move_to_position(safe_position, duration=1.0, kp=10.0, kd=0.8)

        return safe_position


def main():
    """
    Main execution: discover range and settle at maximum position.

    This demonstrates a complete robotic calibration routine:
    1. Initialize hardware
    2. Discover physical limits
    3. Move to a known good position
    """

    motor = CubeMarsMotor(motor_id=1)

    try:
        # Initialize motor
        print("=" * 60)
        print("Range of Motion Discovery")
        print("=" * 60)

        motor.flush_can_buffer()
        motor.enter_motor_mode()
        time.sleep(0.5)

        # Get initial position
        motor.send_command(position=0.0, velocity=0.0, kp=20.0, kd=1.0, torque=0.0)
        time.sleep(0.1)

        feedback = motor.read_feedback(timeout=0.1)
        if feedback:
            initial_position = feedback['position']
            print(f"\nInitial position: {math.degrees(initial_position):.2f}°")
        else:
            initial_position = 0.0
            print("\nWarning: Could not read initial position, assuming 0°")

        # Create discovery object
        # Python note: Classes help organize related functionality
        # Java developers will find this familiar
        discoverer = RangeDiscovery(
            motor=motor,
            torque_threshold=0.5,  # Nm - adjust based on your setup
            creep_velocity=0.15  # rad/s - slow exploration speed
        )
        discoverer.current_position = initial_position

        # First, move to approximate center to explore both directions equally
        print("\nMoving to approximate center position...")
        discoverer.smooth_move_to_position(0.0, duration=2.0)
        time.sleep(0.5)

        # Explore positive direction (max)
        max_position = discoverer.explore_direction(direction=1, max_exploration_time=15.0)
        time.sleep(1.0)

        # Return toward center before exploring negative direction
        print("\nReturning toward center...")
        discoverer.smooth_move_to_position(0.0, duration=3.0)
        time.sleep(1.0)

        # Explore negative direction (min)
        min_position = discoverer.explore_direction(direction=-1, max_exploration_time=15.0)
        time.sleep(1.0)

        # Display discovered range
        print("\n" + "=" * 60)
        print("DISCOVERED RANGE OF MOTION")
        print("=" * 60)
        print(f"Minimum position: {math.degrees(min_position):7.2f}° ({min_position:+.4f} rad)")
        print(f"Maximum position: {math.degrees(max_position):7.2f}° ({max_position:+.4f} rad)")
        print(f"Total range:      {math.degrees(max_position - min_position):7.2f}°")
        print("=" * 60)

        # Settle at maximum position
        print(f"\nSettling at maximum position ({math.degrees(max_position):.2f}°)...")
        discoverer.smooth_move_to_position(max_position, duration=3.0)

        # Hold position with moderate stiffness
        print("\nHolding at max position. Press Ctrl+C to exit.")
        print("Try gently pushing the joint - you'll feel the impedance control.")

        hold_kp = 30.0  # Moderate stiffness for holding
        hold_kd = 1.5  # Good damping

        loop_count = 0
        while True:
            motor.send_command(
                position=max_position,
                velocity=0.0,
                kp=hold_kp,
                kd=hold_kd,
                torque=0.0
            )

            feedback = motor.read_feedback(timeout=0.01)
            if feedback and loop_count % 50 == 0:  # Print every 0.5 seconds
                current_deg = math.degrees(feedback['position'])
                error_deg = math.degrees(max_position - feedback['position'])
                print(f"Holding: {current_deg:6.2f}° (error: {error_deg:+5.2f}°)  "
                      f"Torque: {feedback['torque']:5.3f} Nm")

            loop_count += 1
            time.sleep(0.01)  # 100Hz control loop

    except KeyboardInterrupt:
        print("\n\nStopping gracefully...")
        time.sleep(0.1)

    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()

    finally:
        try:
            motor.close()
        except:
            pass
        print("\nMotor stopped and CAN bus closed.")
        print("Range discovery complete!")


if __name__ == "__main__":
    main()