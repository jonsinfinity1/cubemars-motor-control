#!/usr/bin/env python3
"""
CubeMars AK-Series Motor Driver

Low-level driver for CubeMars AK-series actuators using CAN bus.
Implements the MIT Mini Cheetah protocol for position/velocity/torque
control with impedance control parameters.

Tested with: AK40-10
Compatible with: AK60-6, AK70-10, AK80-6, AK80-9, AK10-9

Hardware Requirements:
- CAN interface (e.g., Waveshare RS485 CAN HAT on Raspberry Pi)
- 24V power supply for AK40-10
- Proper 120Ω termination on CAN bus

Protocol Reference:
MIT Mini Cheetah protocol uses 8-byte CAN frames with:
- Position: 16-bit signed int (-12.5 to 12.5 rad)
- Velocity: 12-bit signed int (-45 to 45 rad/s)
- Kp: 12-bit unsigned int (0 to 500)
- Kd: 12-bit unsigned int (0 to 5)
- Torque: 12-bit signed int (-18 to 18 Nm)
"""

import can
import time
from .motor_driver import MotorDriver


class CubeMarsDriver(MotorDriver):
    """
    Driver for CubeMars AK-series motors via CAN bus.
    
    This class handles all the low-level details of communicating with
    CubeMars motors: encoding commands into CAN frames, decoding responses,
    and managing the motor's control modes.
    
    Robotics Context:
    ----------------
    The MIT Mini Cheetah protocol provides impedance control - you can
    command both a target state AND how stiffly to achieve it. This is
    essential for compliant robotics (humanoids, legged robots) where
    you need to react to unexpected forces.
    
    Python Context:
    --------------
    This class directly implements the MotorDriver interface (no adapter
    needed). The python-can library handles the Linux SocketCAN interface,
    which talks to your CAN hardware.
    
    Example:
        driver = CubeMarsDriver(motor_id=1, can_channel='can0')
        driver.enter_motor_mode()
        driver.send_command(position=0.5, velocity=0, kp=20, kd=1, torque=0)
        feedback = driver.read_feedback(timeout=0.1)
        driver.close()
    """
    
    def __init__(self, motor_id=1, can_channel='can0', bitrate=1000000, position_offset_deg=None):
        """
        Initialize CubeMars motor driver.
        
        Args:
            motor_id: Motor CAN ID (1-32)
            can_channel: CAN interface name (default: 'can0')
            bitrate: CAN bus bitrate in bits/sec (default: 1000000 = 1 Mbps)
            position_offset_deg: Position offset in degrees
                - None (default): Auto-load from motor_config.json
                - 0.0: Explicitly no offset (raw motor coordinates)
                - <value>: Use this specific offset
        
        Note: The bitrate must match what you configured using:
              sudo ip link set can0 type can bitrate 1000000
        """
        import math
        import json
        from pathlib import Path
        
        self.motor_id = motor_id
        self.can_channel = can_channel
        
        # Auto-load offset from config if not specified
        if position_offset_deg is None:
            try:
                config_path = Path(__file__).parent.parent.parent / 'configs' / 'motor_config.json'
                with open(config_path, 'r') as f:
                    config = json.load(f)
                
                # Find motor in config
                for m in config['motors']:
                    if m['id'] == motor_id:
                        position_offset_deg = m.get('position_offset_deg', 0.0)
                        break
                else:
                    # Motor not in config, use 0.0
                    position_offset_deg = 0.0
            except:
                # Config file not found or other error, use 0.0
                position_offset_deg = 0.0
        
        # SAFETY: Validate offset is within reasonable range
        # AK40-10 motors have ~±12.5 rad (±716°) absolute range
        # Offsets should typically be within ±360° (one full rotation)
        # Anything beyond ±400° is almost certainly an error
        if abs(position_offset_deg) > 400.0:
            print(f"⚠️  WARNING: Motor {motor_id} offset {position_offset_deg:.2f}° is dangerously large!")
            print(f"    This is likely a configuration error. Using 0.0° instead.")
            print(f"    Please re-run configure_offsets.py to set correct offset.")
            position_offset_deg = 0.0
        
        self.position_offset_rad = math.radians(position_offset_deg)
        
        # Motor physical limits for AK40-10
        # These define the motor's safe operating envelope
        self.P_MIN = -12.5   # rad (minimum position)
        self.P_MAX = 12.5    # rad (maximum position)
        self.V_MIN = -45.0   # rad/s (minimum velocity)
        self.V_MAX = 45.0    # rad/s (maximum velocity)
        self.T_MIN = -18.0   # Nm (minimum torque)
        self.T_MAX = 18.0    # Nm (maximum torque)
        self.KP_MIN = 0.0    # minimum position gain
        self.KP_MAX = 500.0  # maximum position gain
        self.KD_MIN = 0.0    # minimum damping gain
        self.KD_MAX = 5.0    # maximum damping gain
        
        # Initialize CAN interface
        # Python note: This uses the python-can library which provides
        # a nice abstraction over Linux SocketCAN
        try:
            self.bus = can.interface.Bus(channel=can_channel, bustype='socketcan')
            offset_msg = f" (offset: {position_offset_deg:.2f}°)" if position_offset_deg != 0.0 else ""
            print(f"[CubeMars-{motor_id}] Connected to {can_channel}{offset_msg}")
        except Exception as e:
            print(f"[CubeMars-{motor_id}] Error connecting to CAN: {e}")
            raise
        
        # State tracking (optional, for debugging)
        self.last_position = 0.0
        self.last_velocity = 0.0
        self.last_torque = 0.0
    
    def send_command(self, position, velocity, kp, kd, torque):
        """
        Send MIT Mini Cheetah protocol command.
        
        This packs all control parameters into an 8-byte CAN frame and
        sends it to the motor. The motor will respond with its current
        state (position, velocity, torque).
        
        Robotics Insight:
        ----------------
        Impedance control lets you tune the motor's "personality":
        - High kp (e.g., 100): Stiff, precise, fights external forces
        - Low kp (e.g., 5): Compliant, yields to external forces
        - High kd: Heavily damped, resists velocity changes
        - Low kd: Lightly damped, allows faster motion
        
        For humanoid robots, you typically want:
        - Moderate kp (20-50) for compliant but controlled motion
        - Low kd (0.5-2) for smooth, natural movement
        
        Args:
            position: Target position in radians (logical coordinates)
            velocity: Target velocity in rad/s
            kp: Position gain (0-500)
            kd: Damping gain (0-5)
            torque: Feed-forward torque in Nm
        """
        # Apply offset to convert logical position to motor position
        motor_position = position + self.position_offset_rad
        
        # Clamp all values to motor's safe limits
        # This is a safety feature - prevents sending dangerous commands
        position = max(self.P_MIN, min(self.P_MAX, motor_position))
        velocity = max(self.V_MIN, min(self.V_MAX, velocity))
        kp = max(self.KP_MIN, min(self.KP_MAX, kp))
        kd = max(self.KD_MIN, min(self.KD_MAX, kd))
        torque = max(self.T_MIN, min(self.T_MAX, torque))
        
        # Convert float values to unsigned integers for CAN transmission
        # Each parameter gets packed into a specific number of bits
        p_int = self._float_to_uint(position, self.P_MIN, self.P_MAX, 16)
        v_int = self._float_to_uint(velocity, self.V_MIN, self.V_MAX, 12)
        kp_int = self._float_to_uint(kp, self.KP_MIN, self.KP_MAX, 12)
        kd_int = self._float_to_uint(kd, self.KD_MIN, self.KD_MAX, 12)
        t_int = self._float_to_uint(torque, self.T_MIN, self.T_MAX, 12)
        
        # Pack into 8-byte CAN frame according to MIT Mini Cheetah protocol
        # Byte layout:
        #   0-1: Position (16 bits)
        #   2-3: Velocity (12 bits) + Kp high nibble (4 bits)
        #   4-5: Kp low byte (8 bits) + Kd high byte (8 bits)
        #   6-7: Kd low nibble (4 bits) + Torque (12 bits)
        data = bytes([
            p_int >> 8,                                      # Position high byte
            p_int & 0xFF,                                    # Position low byte
            (v_int >> 4) & 0xFF,                            # Velocity high byte
            ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF),  # Velocity low + Kp high
            kp_int & 0xFF,                                   # Kp low byte
            (kd_int >> 4) & 0xFF,                           # Kd high byte
            ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF),  # Kd low + Torque high
            t_int & 0xFF                                     # Torque low byte
        ])
        
        # Create and send CAN message
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=data,
            is_extended_id=False
        )
        self.bus.send(msg)
    
    def read_feedback(self, timeout=0.1):
        """
        Read motor state from CAN bus.
        
        After sending a command, the motor responds with its current state.
        This method waits for that response and decodes it.
        
        IMPORTANT: This method filters responses to only return feedback
        from THIS motor (matching self.motor_id). Other motors' responses
        are discarded.
        
        Returns:
            dict: {
                'position': float (radians),
                'velocity': float (rad/s),
                'torque': float (Nm),
                'id': int (motor CAN ID)
            }
            None: If timeout expires or response is malformed
        
        Timing Note:
        -----------
        At 1 Mbps CAN bus speed, an 8-byte frame takes ~80 microseconds.
        Adding processing time, responses typically arrive within 1-2 ms.
        A 100ms timeout (default) is very conservative.
        """
        import time
        start_time = time.time()
        
        while True:
            remaining_timeout = timeout - (time.time() - start_time)
            if remaining_timeout <= 0:
                return None
            
            msg = self.bus.recv(timeout=remaining_timeout)
            
            if msg is None:
                return None
            
            if len(msg.data) < 6:
                # Malformed response, try again
                continue
            
            # Check if this response is from OUR motor
            response_motor_id = msg.data[0]
            if response_motor_id != self.motor_id:
                # This response is from a different motor, discard and keep reading
                continue
            
            # Decode motor response
            # Response layout (6 bytes):
            #   0: Motor ID
            #   1-2: Position (16 bits)
            #   3-4: Velocity (12 bits)
            #   4-5: Torque (12 bits)
            p_int = (msg.data[1] << 8) | msg.data[2]
            v_int = (msg.data[3] << 4) | (msg.data[4] >> 4)
            t_int = ((msg.data[4] & 0xF) << 8) | msg.data[5]
            
            # Convert unsigned ints back to floats
            motor_position = self._uint_to_float(p_int, self.P_MIN, self.P_MAX, 16)
            velocity = self._uint_to_float(v_int, self.V_MIN, self.V_MAX, 12)
            torque = self._uint_to_float(t_int, self.T_MIN, self.T_MAX, 12)
            
            # Apply offset to convert motor position to logical position
            logical_position = motor_position - self.position_offset_rad
            
            # Update state tracking
            self.last_position = logical_position
            self.last_velocity = velocity
            self.last_torque = torque
            
            return {
                'position': logical_position,
                'velocity': velocity,
                'torque': torque,
                'id': response_motor_id
            }
    
    def enter_motor_mode(self):
        """
        Enable motor control mode.
        
        Sends the "enter motor mode" command (0xFFFFFFFFFFFFFFFC).
        After this, the motor will respond to control commands.
        
        Before this command, the motor is in a safe idle state with zero torque.
        """
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
            is_extended_id=False
        )
        self.bus.send(msg)
        print(f"[CubeMars-{self.motor_id}] Entering motor mode")
        time.sleep(0.1)  # Give motor time to transition
    
    def exit_motor_mode(self):
        """
        Disable motor control mode.
        
        Sends the "exit motor mode" command (0xFFFFFFFFFFFFFFFD).
        Motor goes limp with zero torque and stops responding to control commands.
        
        Safety: Always call this before powering down or ending your program.
        """
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
            is_extended_id=False
        )
        self.bus.send(msg)
        print(f"[CubeMars-{self.motor_id}] Exiting motor mode")
        time.sleep(0.1)
    
    def set_zero_position(self):
        """
        Set current position as zero reference.
        
        This is useful for calibration - you can manually position the joint
        where you want zero to be, then call this method.
        
        Note: This is specific to CubeMars motors and not in the MotorDriver
        interface, but it's useful enough to include here.
        """
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE],
            is_extended_id=False
        )
        self.bus.send(msg)
        print(f"[CubeMars-{self.motor_id}] Setting zero position")
        time.sleep(0.1)
    
    def flush_buffer(self):
        """
        Clear CAN receive buffer.
        
        Reads and discards all pending messages. Useful at startup to ensure
        you're reading fresh data, not stale messages from the buffer.
        """
        count = 0
        while self.bus.recv(timeout=0) is not None:
            count += 1
        if count > 0:
            print(f"[CubeMars-{self.motor_id}] Flushed {count} old messages")
    
    def close(self):
        """
        Clean shutdown of the driver.
        
        1. Exits motor mode (motor goes limp)
        2. Closes CAN bus connection
        
        Always call this before program exit to ensure safe shutdown.
        """
        self.exit_motor_mode()
        self.bus.shutdown()
        print(f"[CubeMars-{self.motor_id}] Driver closed")
    
    @classmethod
    def from_config(cls, motor_id, config_path=None):
        """
        Create driver from motor_config.json.
        
        This is a CLASS METHOD - you call it on the class, not an instance.
        
        Python note:
        -----------
        @classmethod means this method gets the class (cls) as first parameter,
        not an instance (self). Useful for alternative constructors.
        
        Java equivalent:
            public static CubeMarsDriver fromConfig(int motorId, String path) {
                // Load config, create instance
                return new CubeMarsDriver(...);
            }
        
        Usage:
            driver = CubeMarsDriver.from_config(motor_id=2)
        
        Args:
            motor_id: Motor CAN ID to load config for
            config_path: Path to motor_config.json (optional, uses default if None)
        
        Returns:
            CubeMarsDriver: Configured driver instance with offset applied
        """
        import json
        from pathlib import Path
        
        if config_path is None:
            # Assume we're in motor_drivers/ directory, config is in ../../configs/
            config_path = Path(__file__).parent.parent.parent / 'configs' / 'motor_config.json'
        else:
            config_path = Path(config_path)
        
        with open(config_path, 'r') as f:
            config = json.load(f)
        
        # Find motor in config
        motor_config = None
        for m in config['motors']:
            if m['id'] == motor_id:
                motor_config = m
                break
        
        if motor_config is None:
            raise ValueError(f"Motor ID {motor_id} not found in config")
        
        offset = motor_config.get('position_offset_deg', 0.0)
        can_channel = config['can_interface']['channel']
        
        return cls(
            motor_id=motor_id,
            can_channel=can_channel,
            position_offset_deg=offset
        )
    
    # ========================================================================
    # Internal Helper Methods
    # ========================================================================
    
    def _float_to_uint(self, x, x_min, x_max, bits):
        """
        Convert float to unsigned int for CAN encoding.
        
        Maps a float in range [x_min, x_max] to an unsigned integer
        using the specified number of bits.
        
        Example: Position range [-12.5, 12.5] rad mapped to 16-bit uint [0, 65535]
        
        Args:
            x: Float value to encode
            x_min: Minimum of float range
            x_max: Maximum of float range
            bits: Number of bits for integer representation
        
        Returns:
            int: Encoded unsigned integer
        """
        span = x_max - x_min
        offset = x_min
        return int((x - offset) * ((1 << bits) - 1) / span)
    
    def _uint_to_float(self, x_int, x_min, x_max, bits):
        """
        Convert unsigned int to float for CAN decoding.
        
        Inverse of _float_to_uint. Maps unsigned integer back to float
        in range [x_min, x_max].
        
        Args:
            x_int: Unsigned integer to decode
            x_min: Minimum of float range
            x_max: Maximum of float range
            bits: Number of bits used in integer representation
        
        Returns:
            float: Decoded value
        """
        span = x_max - x_min
        offset = x_min
        return float(x_int) * span / ((1 << bits) - 1) + offset
