# CubeMars Motor Control

Python control library for CubeMars AK40-10 motors using CAN bus and MIT Mini Cheetah protocol.

## Hardware Requirements
- CubeMars AK40-10 motor
- CAN interface (e.g., MCP2515 on Raspberry Pi)
- Raspberry Pi or compatible Linux system

## Setup
1. Enable CAN interface:
```bash
sudo ip link set can0 up type can bitrate 1000000
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage
See `examples/degree_oscillation.py` for interactive oscillation control with force-sensitive direction reversal.

## Features
- Smooth startup from current position
- Torque-based interaction (push to reverse direction)
- Configurable PID gains, oscillation parameters
```

### Create requirements.txt:
```
python-can>=4.0.0