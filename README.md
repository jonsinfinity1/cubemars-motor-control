# CubeMars Motor Control

Python control library for CubeMars AK40-10 motors using CAN bus and MIT Mini Cheetah protocol.

## Hardware Requirements
- CubeMars AK40-10 motor
- CAN interface (e.g., MCP2515 on Raspberry Pi)
- Raspberry Pi or compatible Linux system

## Setup

### 1. Enable CAN interface
```bash
sudo ip link set can0 up type can bitrate 1000000
```

To make this persistent on boot, add to `/etc/network/interfaces`:
```
auto can0
iface can0 inet manual
    pre-up /sbin/ip link set can0 type can bitrate 1000000
    up /sbin/ifconfig can0 up
    down /sbin/ifconfig can0 down
```

### 2. Install dependencies
```bash
pip install -r requirements.txt
```

## Usage

### Interactive Oscillation Demo
```bash
cd src
python degree_oscillation.py
```

This demo features:
- Smooth startup from current motor position
- 20° oscillation range at 0.5 Hz
- **Force-sensitive interaction** - push the motor to make it reverse direction!
- Smooth direction transitions with 0.5 second cooldown

### Basic Motor Control
```bash
cd src
python cubemars_motor.py
```

Simple sinusoidal motion demo showing basic motor control.

## Features
- **Smooth startup** - gradually blends from current position into oscillation
- **Torque-based interaction** - detects when motor is pushed (0.3 Nm threshold)
- **Smooth direction reversal** - gradual transition over 0.5 seconds
- **Configurable parameters** - adjust PID gains, oscillation frequency, amplitude, and torque sensitivity
- **Real-time feedback** - displays position, torque, and direction

## Motor Parameters

The `degree_oscillation.py` script uses these default parameters:
- **Amplitude**: 20° (±10°)
- **Frequency**: 0.5 Hz
- **Torque threshold**: 0.3 Nm
- **Position gain (kp)**: 20.0
- **Velocity gain (kd)**: 1.0

Adjust these in the code to change the motor's behavior.

## Safety Notes
- Always ensure proper motor mounting before operation
- Start with low gains and gradually increase
- Be prepared to press Ctrl+C to emergency stop
- The motor will jump to zero position when entering motor mode

## License
MIT
