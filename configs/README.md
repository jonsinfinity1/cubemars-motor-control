# Motor Configuration Guide

Complete guide to configuring, calibrating, and controlling your humanoid robot motors.

---

## Table of Contents

1. [Overview](#overview)
2. [Quick Start](#quick-start)
3. [Configuration File](#configuration-file)
4. [Position Offsets](#position-offsets)
5. [Starting Positions & Poses](#starting-positions--poses)
6. [Initialization & Calibration](#initialization--calibration)
7. [Usage Examples](#usage-examples)
8. [Troubleshooting](#troubleshooting)
9. [Learning Notes](#learning-notes)

---

## Overview

This system handles complete motor configuration for your humanoid robot:

- **Position Offsets**: Translate between motor coordinates and logical coordinates
- **Auto-Calibration**: Automatically discover each joint's range of motion
- **Starting Positions**: Define default positions for each motor
- **Named Poses**: Create full-body poses (standing, walking, etc.)

### Files

- **`motor_config.json`** - Main configuration (motor IDs, offsets, poses, control parameters)
- **`calibration_YYYYMMDD_HHMMSS.json`** - Generated calibration results (ranges, safe limits)

### Scripts

- **`configure_offsets.py`** - Interactive tool to set position offsets
- **`initialize_hip.py`** - Auto-calibration script
- **`query_positions.py`** - Read current motor positions
- **`move_to_start.py`** - Move to starting positions or named poses

---

## Quick Start

### First-Time Setup

```bash
# 1. Configure offsets (do this first!)
cd /Users/jon/robot/src
python3 configure_offsets.py

# 2. Run calibration to discover ranges
python3 initialize_hip.py

# 3. Test that it works
python3 query_positions.py
```

### Daily Usage

```bash
# Move to starting positions
python3 move_to_start.py

# Check current positions
python3 query_positions.py
```

---

## Configuration File

### Complete Example

```json
{
  "motors": [
    {
      "id": 1,
      "name": "hip_abduction_right",
      "joint_type": "hip",
      "dof": "abduction",
      "side": "right",
      
      "position_offset_deg": 36.34,
      "starting_position_deg": 0.0,
      "notes": "0° = leg straight down, positive = out to side",
      
      "discovery_params": {
        "torque_threshold": 0.5,
        "creep_velocity": 0.15,
        "max_exploration_time": 15.0
      },
      
      "control_params": {
        "default_kp": 20.0,
        "default_kd": 1.0
      }
    },
    {
      "id": 2,
      "name": "hip_flexion_right",
      "joint_type": "hip",
      "dof": "flexion",
      "side": "right",
      
      "position_offset_deg": 93.23,
      "starting_position_deg": 0.0,
      "notes": "Negative = back, 0° = vertical, positive = forward",
      
      "discovery_params": {
        "torque_threshold": 0.5,
        "creep_velocity": 0.15,
        "max_exploration_time": 15.0
      },
      
      "control_params": {
        "default_kp": 20.0,
        "default_kd": 1.0
      }
    }
  ],
  
  "can_interface": {
    "channel": "can0",
    "bitrate": 1000000
  },
  
  "safety": {
    "safety_margin_deg": 5.0,
    "max_sequential_failures": 2
  },
  
  "poses": {
    "standing": {
      "description": "Natural standing pose",
      "positions": {
        "hip_abduction_right": 0.0,
        "hip_flexion_right": 0.0
      }
    },
    "ready": {
      "description": "Ready to walk - weight forward",
      "positions": {
        "hip_abduction_right": 5.0,
        "hip_flexion_right": 15.0
      }
    }
  }
}
```

### Field Descriptions

#### Motor Entry

| Field | Type | Description |
|-------|------|-------------|
| `id` | int | CAN bus motor ID (1-32) |
| `name` | string | Human-readable identifier |
| `joint_type` | string | Joint category (hip, knee, ankle) |
| `dof` | string | Degree of freedom (abduction, flexion, rotation) |
| `side` | string | left/right/center |
| `position_offset_deg` | float | Software offset (see [Position Offsets](#position-offsets)) |
| `starting_position_deg` | float | Default position after calibration |
| `notes` | string | Documentation for this motor |

#### Discovery Parameters

| Field | Value | Description |
|-------|-------|-------------|
| `torque_threshold` | 0.5 Nm | Torque indicating hard stop |
| `creep_velocity` | 0.15 rad/s | Slow exploration speed |
| `max_exploration_time` | 15.0 s | Safety timeout |

**Tuning tips:**
- Start with conservative values (low threshold, slow speed)
- Increase `torque_threshold` to 0.8-1.0 Nm if getting false positives from friction
- Never exceed 2.0 Nm to avoid motor damage

#### Control Parameters

| Field | Value | Description |
|-------|-------|-------------|
| `default_kp` | 20.0 | Position gain (stiffness) |
| `default_kd` | 1.0 | Damping gain |

**For humanoid robots:**
- Moderate kp (20-50): Compliant but controlled
- Low kd (0.5-2): Smooth, natural movement

#### Safety Settings

| Field | Value | Description |
|-------|-------|-------------|
| `safety_margin_deg` | 5.0° | Buffer from hard stops |
| `max_sequential_failures` | 2 | Abort after N failures |

---

## Position Offsets

### The Problem

Motor encoders have arbitrary zero positions. Your motor might read `41.10°` when the leg is vertical, even though you want that to be logical `0°`.

### The Solution

Software offsets translate between **motor coordinates** (what the encoder reads) and **logical coordinates** (what you want).

```
You command:      -10° (logical)
Driver adds:      +41.10° (offset)
Motor receives:   31.10° (motor coordinate)
Motor moves to:   31.10°
Driver reads:     31.10° (motor coordinate)  
Driver subtracts: -41.10° (offset)
You see:          -10° (logical) ✓
```

### Configuring Offsets

**Step 1: Run the configuration tool**

```bash
cd /Users/jon/robot/src
python3 configure_offsets.py
```

**Step 2: For each motor:**
1. Manually position the motor at logical zero
   - Hip flexion: Leg vertical (straight down)
   - Hip abduction: Leg straight down (not angled to side)
2. Script reads motor's actual position (e.g., `41.10°`)
3. Script saves this as `position_offset_deg` in config

**Step 3: Verify**

```bash
python3 query_positions.py
```

When motor is at logical zero, it should now read ~0°.

### Using Offsets in Code

Offsets are **built into** `CubeMarsDriver` - just use the normal driver:

```python
from motor_drivers import CubeMarsDriver

# Load from config (offset applied automatically)
driver = CubeMarsDriver.from_config(motor_id=2)

# Or specify manually
driver = CubeMarsDriver(motor_id=2, position_offset_deg=93.23)

# For debugging without offset
driver = CubeMarsDriver(motor_id=2, position_offset_deg=0.0)
```

The driver transparently handles all offset math!

### Why Software Offsets?

✅ **Reliable** - Hardware SET_ZERO command doesn't work on all motors  
✅ **Flexible** - Easy to change/update  
✅ **Safe** - No risk of corrupting motor memory  
✅ **Portable** - Version controlled in config files  
✅ **Simple** - Built into the driver, no extra code needed

---

## Starting Positions & Poses

### Starting Positions

After calibration, motors move to their configured `starting_position_deg`. This is often different from the geometric center of the range.

**Example:**
```json
{
  "id": 2,
  "name": "hip_flexion_right",
  "starting_position_deg": -15.0,
  "notes": "Leg slightly back for balance"
}
```

Range might be -60° to +40° (midpoint = -10°), but you want -15° for stability.

### Named Poses

Define complete body configurations:

```json
{
  "poses": {
    "standing": {
      "description": "Natural standing pose",
      "positions": {
        "hip_abduction_right": 0.0,
        "hip_flexion_right": -10.0
      }
    },
    "walking_ready": {
      "description": "Weight forward, ready to step",
      "positions": {
        "hip_abduction_right": 2.0,
        "hip_flexion_right": 15.0
      }
    }
  }
}
```

### Finding Good Starting Positions

**Method 1: Manual experimentation**
1. Run calibration to discover ranges
2. Use control scripts to move joints
3. Find comfortable positions
4. Note the angles
5. Update config

**Method 2: Physical positioning**
1. Power off (motors go limp)
2. Manually position robot
3. Run `query_positions.py`
4. Use those values in config

### Using Poses in Code

```python
import json

with open('configs/motor_config.json', 'r') as f:
    config = json.load(f)

# Get starting position for a motor
motor_config = config['motors'][0]
start_pos = motor_config.get('starting_position_deg', 0.0)

# Get a named pose
standing_pose = config['poses']['standing']
for motor_name, position in standing_pose['positions'].items():
    # Find joint with this name and move it
    joint.move_to(position, duration=2.0)
```

---

## Initialization & Calibration

### What It Does

The initialization script (`initialize_hip.py`) performs automatic calibration:

1. Reads motor configuration
2. Initializes each motor sequentially (lowest ID to highest)
3. Discovers range of motion by slowly moving until detecting hard stops
4. Saves calibration results with safety margins
5. Moves motors to starting positions

### Before Running

Ensure:
- ✓ Hip assembly securely mounted
- ✓ Motors can move freely through full range
- ✓ No obstructions or cables in way
- ✓ 24V power connected
- ✓ CAN bus properly terminated (120Ω resistors)
- ✓ Raspberry Pi connected to CAN interface
- ✓ **Offsets configured first** (run `configure_offsets.py`)

### Running Calibration

```bash
cd /Users/jon/robot/src
python3 initialize_hip.py
```

**What happens:**
1. Shows configuration summary
2. Asks for confirmation
3. Initializes motors sequentially
4. Discovers ranges (motors will move!)
5. Saves timestamped calibration file
6. Moves to starting positions

**Duration:** 2-5 minutes per motor

### Calibration Output

Creates `configs/calibration_YYYYMMDD_HHMMSS.json`:

```json
{
  "calibration_date": "2024-12-07T15:30:22",
  "motors": {
    "1": {
      "name": "hip_abduction_right",
      "min_deg": -52.3,
      "max_deg": 48.7,
      "safe_min_deg": -47.3,
      "safe_max_deg": 43.7,
      "center_deg": -1.8,
      "total_range_deg": 91.0
    }
  }
}
```

### Why Sequential Discovery?

**Safety:** One moving joint is easier to monitor  
**Power:** Lower total current draw  
**Debugging:** Easier to identify problematic joints  
**Mechanical:** Reduces cross-coupling between joints

### Safety Margins

The system adds margins to discovered ranges:

```
Discovered: -60° to +60° (actual hard stops)
Safe range: -55° to +55° (5° margin each side)
```

This prevents constantly hitting hard stops during operation.

---

## Usage Examples

### Check Current Positions

```python
from motor_drivers import CubeMarsDriver
import math
import time

driver = CubeMarsDriver.from_config(motor_id=2)
driver.flush_buffer()
driver.enter_motor_mode()
time.sleep(0.2)

# Query with zero torque (don't move)
driver.send_command(0, 0, kp=0, kd=0, torque=0)
time.sleep(0.05)
feedback = driver.read_feedback()

print(f"Position: {math.degrees(feedback['position']):.2f}°")
driver.close()
```

### Move to Starting Position

```python
from motor_drivers import CubeMarsDriver
from motor_control import Joint
import json

# Load config
with open('configs/motor_config.json', 'r') as f:
    config = json.load(f)

# Get motor config
motor_config = config['motors'][0]  # First motor
start_pos = motor_config['starting_position_deg']

# Create joint
driver = CubeMarsDriver.from_config(motor_id=motor_config['id'])
joint = Joint(
    driver=driver,
    name=motor_config['name'],
    kp=motor_config['control_params']['default_kp'],
    kd=motor_config['control_params']['default_kd']
)

# Initialize and move
joint.initialize()
joint.move_to(start_pos, duration=2.0)
joint.shutdown()
```

### Use Calibration Data

```python
import json

# Load calibration
with open('configs/calibration_20241207_153022.json', 'r') as f:
    cal = json.load(f)

# Get safe range for motor ID 1
motor_1 = cal['motors']['1']
safe_min = motor_1['safe_min_deg']
safe_max = motor_1['safe_max_deg']
center = motor_1['center_deg']

print(f"Safe range: {safe_min}° to {safe_max}°")
print(f"Center: {center}°")

# Use in control code
joint.move_to(center, duration=2.0)
```

---

## Troubleshooting

### CAN Interface Issues

**"Config file not found"**
```bash
ls configs/motor_config.json  # Verify file exists
```

**Motor doesn't respond**
```bash
# Check CAN interface status
ip link show can0

# Bring it up if down
sudo ip link set can0 up type can bitrate 1000000

# Check for CAN traffic
candump can0
```

### Calibration Issues

**"Too many initialization failures"**
- Check CAN bus connection
- Verify motor IDs match physical motors
- Check 24V power to motors
- Review termination resistors (120Ω on each end)

**"Range discovery failed"**
- Check if motor can move freely
- Verify `torque_threshold` isn't too high/low
- Ensure `max_exploration_time` is sufficient
- Check for mechanical binding

**"Starting position outside safe range"**
- Check calibration results for actual safe range
- Adjust `starting_position_deg` to be within range
- Or adjust `safety_margin_deg` (carefully!)

### Offset Issues

**"Position still wrong after setting offset"**
- Re-run `configure_offsets.py`
- Verify motor was at correct logical zero
- Check offset was saved to `motor_config.json`
- Confirm using `CubeMarsDriver.from_config()` to load offset

**"Want to change logical zero"**
- Just re-run `configure_offsets.py` with new desired position

---

## Learning Notes

### Python Concepts

#### JSON Configuration Files

Python's standard way to handle configuration:

```python
import json

# Reading
with open('config.json', 'r') as f:
    config = json.load(f)  # Returns dict

# Writing
with open('config.json', 'w') as f:
    json.dump(config, f, indent=2)  # Pretty formatting

# Safe dictionary access
offset = motor_config.get('position_offset_deg', 0.0)  # Default if not found
```

**Advantages:**
- Human-readable text format
- Easy to edit with any text editor
- Native Python support
- Portable across platforms

#### Context Managers (`with` statement)

Python's `with` statement is equivalent to Java's try-with-resources:

```python
# Python
with open('file.txt', 'r') as f:
    data = f.read()
# File automatically closed

# Java equivalent
try (FileReader reader = new FileReader("file.txt")) {
    // use reader
} // auto-closed
```

#### Class Methods

```python
@classmethod
def from_config(cls, motor_id, config_path=None):
    # cls is the class itself
    return cls(motor_id, offset, ...)

# Usage
driver = CubeMarsDriver.from_config(motor_id=2)
```

Java equivalent:
```java
public static CubeMarsDriver fromConfig(int motorId) {
    return new CubeMarsDriver(motorId, offset);
}
```

#### List Comprehensions & Lambda

```python
# Sort motors by ID
motors = sorted(config['motors'], key=lambda m: m['id'])

# Java equivalent
motors.sort(Comparator.comparingInt(m -> m.getId()));
```

### Robotics Concepts

#### Coordinate Frames

Professional robots use multiple coordinate systems:

- **Motor coordinates**: Raw encoder values (what the hardware reads)
- **Joint coordinates**: Logical angles after offsets (what you command)
- **World coordinates**: Robot's position in space
- **Task coordinates**: Where you want end-effector to be

Offsets translate between motor and joint coordinates. This is fundamental robotics!

#### Impedance Control

The MIT Mini Cheetah protocol provides impedance control:

```python
driver.send_command(
    position=target,  # Where to go
    velocity=0,       # How fast
    kp=20.0,          # How stiff (position gain)
    kd=1.0,           # How damped
    torque=0          # Feed-forward
)
```

**Tuning the "personality":**
- **High kp** (e.g., 100): Stiff, precise, fights external forces
- **Low kp** (e.g., 5): Compliant, yields to forces
- **High kd**: Heavily damped, resists velocity changes
- **Low kd**: Lightly damped, allows faster motion

For humanoids: Moderate kp (20-50) + Low kd (0.5-2) = smooth, natural movement

#### Torque-Based Limit Detection

Instead of hitting hard stops at full speed:

1. Move slowly (`creep_velocity = 0.15 rad/s`)
2. Monitor torque continuously
3. Stop when torque exceeds threshold (`torque_threshold = 0.5 Nm`)
4. Record position as limit

This is gentle on the hardware and provides accurate limit detection.

#### Safety Margins

Always operate inside discovered limits:

```
Hard stop: -60°
Safe min:  -55° (5° margin)
       ↓
  Operating range: -55° to +55°
       ↑
Safe max:  +55° (5° margin)
Hard stop: +60°
```

Prevents constantly hitting mechanical limits during normal operation.

#### Sequential vs Parallel Operations

**Sequential** (used here):
- One motor moves at a time
- Safer, easier to debug
- Lower power draw
- Better for initial calibration

**Parallel** (use later):
- Multiple motors move simultaneously
- Required for coordinated motion (walking, etc.)
- Higher complexity
- Use after individual motors are well-characterized

---

## Summary

### Workflow

1. **Configure offsets** (`configure_offsets.py`)
   - Set logical zero for each motor
   - Offsets saved to `motor_config.json`

2. **Run calibration** (`initialize_hip.py`)
   - Discovers ranges automatically
   - Creates calibration file
   - Moves to starting positions

3. **Normal operation**
   - Use `CubeMarsDriver.from_config()` - offsets applied automatically
   - Load calibration data for safe ranges
   - Use starting positions and named poses

### Key Files

- **`motor_config.json`** - Edit this to configure motors, offsets, poses
- **`calibration_*.json`** - Generated by initialization, contains discovered ranges
- **Scripts** - `configure_offsets.py`, `initialize_hip.py`, `query_positions.py`

### Next Steps

After successful setup:

1. ✓ Review calibration data
2. ✓ Test individual joints with discovered ranges
3. ✓ Define additional named poses
4. → Implement coordinated control (both joints together)
5. → Build higher-level behaviors (walking gaits, balance)

---

*For questions or issues, review the [Troubleshooting](#troubleshooting) section or check script output for detailed error messages.*
