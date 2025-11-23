#!/usr/bin/env python3
"""
CubeMars AK40-10 Motor Controller using CAN bus
MIT Mini Cheetah Protocol Implementation
"""

import can
import time
import math


class CubeMarsMotor:
    def __init__(self, motor_id=1, can_channel='can0', bitrate=1000000):
        """
        Initialize CubeMars motor controller

        Args:
            motor_id: Motor CAN ID (1-32)
            can_channel: CAN interface name (default: 'can0')
            bitrate: CAN bus bitrate (default: 1000000)
        """
        self.motor_id = motor_id
        self.can_channel = can_channel

        # Motor limits for AK40-10
        self.P_MIN = -12.5  # radians
        self.P_MAX = 12.5  # radians
        self.V_MIN = -45.0  # rad/s
        self.V_MAX = 45.0  # rad/s
        self.T_MIN = -18.0  # Nm
        self.T_MAX = 18.0  # Nm
        self.KP_MIN = 0.0
        self.KP_MAX = 500.0
        self.KD_MIN = 0.0
        self.KD_MAX = 5.0

        # Initialize CAN bus
        try:
            self.bus = can.interface.Bus(channel=can_channel, bustype='socketcan')
            print(f"Connected to {can_channel}")
        except Exception as e:
            print(f"Error connecting to CAN bus: {e}")
            raise

        # Motor state
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.temperature = 0.0
        self.error = 0

    def _float_to_uint(self, x, x_min, x_max, bits):
        """Convert float to unsigned int with given range and bit depth"""
        span = x_max - x_min
        offset = x_min
        return int((x - offset) * ((1 << bits) - 1) / span)

    def _uint_to_float(self, x_int, x_min, x_max, bits):
        """Convert unsigned int to float with given range and bit depth"""
        span = x_max - x_min
        offset = x_min
        return float(x_int) * span / ((1 << bits) - 1) + offset

    def enter_motor_mode(self):
        """Enter motor control mode"""
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
            is_extended_id=False
        )
        self.bus.send(msg)
        print(f"Motor {self.motor_id}: Entering motor mode")
        time.sleep(0.1)

    def exit_motor_mode(self):
        """Exit motor control mode"""
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
            is_extended_id=False
        )
        self.bus.send(msg)
        print(f"Motor {self.motor_id}: Exiting motor mode")
        time.sleep(0.1)

    def set_zero_position(self):
        """Set current position as zero"""
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE],
            is_extended_id=False
        )
        self.bus.send(msg)
        print(f"Motor {self.motor_id}: Setting zero position")
        time.sleep(0.1)

    def send_command(self, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0):
        """
        Send motor control command using MIT Mini Cheetah protocol

        Args:
            position: Desired position (radians)
            velocity: Desired velocity (rad/s)
            kp: Position gain (0-500)
            kd: Velocity gain (0-5)
            torque: Feed-forward torque (Nm)
        """
        # Limit values to safe ranges
        position = max(self.P_MIN, min(self.P_MAX, position))
        velocity = max(self.V_MIN, min(self.V_MAX, velocity))
        kp = max(self.KP_MIN, min(self.KP_MAX, kp))
        kd = max(self.KD_MIN, min(self.KD_MAX, kd))
        torque = max(self.T_MIN, min(self.T_MAX, torque))

    # Convert to unsigned ints
        p_int = self._float_to_uint(position, self.P_MIN, self.P_MAX, 16)
        v_int = self._float_to_uint(velocity, self.V_MIN, self.V_MAX, 12)
        kp_int = self._float_to_uint(kp, self.KP_MIN, self.KP_MAX, 12)
        kd_int = self._float_to_uint(kd, self.KD_MIN, self.KD_MAX, 12)
        t_int = self._float_to_uint(torque, self.T_MIN, self.T_MAX, 12)

        # Pack into CAN message
        data = bytes([
            p_int >> 8,
            p_int & 0xFF,
            (v_int >> 4) & 0xFF,
            ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF),
            kp_int & 0xFF,
            (kd_int >> 4) & 0xFF,
            ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF),
            t_int & 0xFF
        ])

        msg = can.Message(
            arbitration_id=self.motor_id,
            data=data,
            is_extended_id=False
        )
        self.bus.send(msg)

    def read_feedback(self, timeout=0.1):
        """
        Read motor feedback from CAN bus

        Returns:
            dict with position, velocity, torque, temperature, error
        """
        msg = self.bus.recv(timeout=timeout)
        if msg is None:
            return None

        if len(msg.data) >= 6:
            # Decode motor response
            p_int = (msg.data[1] << 8) | msg.data[2]
            v_int = (msg.data[3] << 4) | (msg.data[4] >> 4)
            t_int = ((msg.data[4] & 0xF) << 8) | msg.data[5]

            self.position = self._uint_to_float(p_int, self.P_MIN, self.P_MAX, 16)
            self.velocity = self._uint_to_float(v_int, self.V_MIN, self.V_MAX, 12)
            self.torque = self._uint_to_float(t_int, self.T_MIN, self.T_MAX, 12)

            return {
                'position': self.position,
                'velocity': self.velocity,
                'torque': self.torque,
                'id': msg.arbitration_id
            }
        return None

    def flush_can_buffer(self):
        """Clear any pending CAN messages"""
        while self.bus.recv(timeout=0) is not None:
            pass

    def close(self):
        """Close CAN bus connection"""
        self.exit_motor_mode()
        self.bus.shutdown()
        print("CAN bus closed")


# Example usage
if __name__ == "__main__":
    # Create motor controller (motor ID 3)
    motor = CubeMarsMotor(motor_id=1)

    try:
        # Enter motor mode
        motor.enter_motor_mode()
        time.sleep(0.5)

        # Optional: Set current position as zero
        # motor.set_zero_position()
        # time.sleep(0.5)

        print("\nStarting motor control demo...")
        print("Press Ctrl+C to stop\n")

        # Control loop
        start_time = time.time()
        while True:
            # Example: Sinusoidal position command
            t = time.time() - start_time
            target_position = 2.0 * math.sin(2 * math.pi * 0.2 * t)  # 0.2 Hz sine wave

            # Send command with position control
            # kp=50, kd=1 are moderate gains for smooth motion
            motor.send_command(
                position=target_position,
                velocity=0.0,
                kp=50.0,
                kd=1.0,
                torque=0.0
            )

            # Read feedback
            feedback = motor.read_feedback(timeout=0.05)
            if feedback:
                print(f"Pos: {feedback['position']:6.3f} rad  "
                      f"Vel: {feedback['velocity']:6.3f} rad/s  "
                      f"Torque: {feedback['torque']:6.3f} Nm", end='\r')

            time.sleep(0.01)  # 100 Hz control loop

    except KeyboardInterrupt:
        print("\n\nStopping motor...")

    finally:
        # Always exit motor mode safely
        motor.close()
        print("Motor stopped")