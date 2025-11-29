# Phase 1: Better Trajectory Generation - Detailed Checklist

**Goal**: Replace sinusoidal motion with smooth, natural trajectories
**Estimated Time**: 2-3 weeks (1-2 hours per session, 3-4 sessions per week)
**Prerequisites**: None - you have everything you need!

---

## Task 1.1: Implement TrajectoryGenerator Class

### Step-by-step:
- [X] 1. Create new file `src/trajectory_generator.py`
- [ ] 2. Import required libraries (math, numpy)
- [ ] 3. Create `TrajectoryGenerator` class with `@staticmethod` methods
- [ ] 4. Implement `minimum_jerk_trajectory()`:
  - [ ] Add function signature with docstring
  - [ ] Handle edge cases (t <= 0, t >= duration)
  - [ ] Calculate normalized time tau = t/duration
  - [ ] Calculate minimum jerk polynomial: s = 10τ³ - 15τ⁴ + 6τ⁵
  - [ ] Return: start_pos + (end_pos - start_pos) * s
- [ ] 5. Implement `minimum_jerk_velocity()`:
  - [ ] Add function signature with docstring
  - [ ] Handle edge cases
  - [ ] Calculate velocity polynomial: s_dot = (30τ² - 60τ³ + 30τ⁴) / duration
  - [ ] Return: (end_pos - start_pos) * s_dot
- [ ] 6. Add a simple test at bottom of file:
  ```python
  if __name__ == "__main__":
      # Test trajectory generation
      import numpy as np
      
      start = 0.0
      end = 1.0
      duration = 2.0
      
      print("Time\tPosition\tVelocity")
      for t in np.linspace(0, duration, 21):
          pos = TrajectoryGenerator.minimum_jerk_trajectory(start, end, duration, t)
          vel = TrajectoryGenerator.minimum_jerk_velocity(start, end, duration, t)
          print(f"{t:.2f}\t{pos:.4f}\t\t{vel:.4f}")
  ```
- [ ] 7. Run the test: `python src/trajectory_generator.py`
- [ ] 8. Verify:
  - Position starts at 0.0, ends at 1.0
  - Velocity starts at 0.0, ends at 0.0
  - Velocity is smooth (no jumps)

### Files to create:
- `src/trajectory_generator.py`

### Expected output:
Smooth trajectory from start to end position with zero start/end velocity

---

## Task 1.2: Create Point-to-Point Motion Demo

### Step-by-step:
- [ ] 1. Create new file `src/point_to_point_demo.py`
- [ ] 2. Import necessary modules:
  ```python
  import time
  import math
  from cubemars_control import CubeMarsMotor
  from trajectory_generator import TrajectoryGenerator
  ```
- [ ] 3. Set up motor and enter motor mode
- [ ] 4. Get current position (starting point)
- [ ] 5. Define target position (e.g., math.radians(45) for 45°)
- [ ] 6. Define motion duration (e.g., 2.0 seconds)
- [ ] 7. Create control loop:
  - [ ] Calculate elapsed time
  - [ ] Generate target position using minimum_jerk_trajectory()
  - [ ] Generate target velocity using minimum_jerk_velocity()
  - [ ] Send command to motor with both position and velocity
  - [ ] Read and display feedback
  - [ ] Check if motion is complete (t > duration)
- [ ] 8. Add comparison test:
  - [ ] First do minimum jerk trajectory
  - [ ] Pause 2 seconds
  - [ ] Then do instant step (just change target immediately)
  - [ ] Feel the difference!
- [ ] 9. Test with different durations (0.5s, 1s, 2s, 4s)

### Files to create:
- `src/point_to_point_demo.py`

### What to observe:
- Smooth acceleration at start
- Smooth deceleration at end
- No jerky motions
- Compare torque readings: minimum jerk should have smoother torque

### Expected behavior:
Arm moves smoothly from current position to 45° in specified time

---

## Task 1.3: Create Multi-Waypoint Demo

### Step-by-step:
- [ ] 1. Create new file `src/waypoint_demo.py`
- [ ] 2. Import modules (same as Task 1.2)
- [ ] 3. Define waypoint sequence:
  ```python
  waypoints = [
      math.radians(0),
      math.radians(45),
      math.radians(-30),
      math.radians(60),
      math.radians(0)
  ]
  ```
- [ ] 4. Define timing for each segment:
  - Duration to reach each waypoint (e.g., 2.0 seconds each)
  - Pause time at each waypoint (e.g., 0.5 seconds)
- [ ] 5. Create waypoint manager:
  ```python
  current_waypoint_index = 0
  segment_start_time = time.time()
  segment_start_pos = current_position
  state = "MOVING"  # or "PAUSED"
  ```
- [ ] 6. Control loop logic:
  - [ ] If MOVING:
    - Calculate time in current segment
    - If time < duration: generate trajectory
    - If time >= duration: switch to PAUSED, increment waypoint
  - [ ] If PAUSED:
    - Hold current position
    - If pause_time elapsed: switch to MOVING, start next segment
- [ ] 7. Add loop detection (back to waypoint 0 when done)
- [ ] 8. Display current waypoint and state
- [ ] 9. Test and tune pause durations

### Files to create:
- `src/waypoint_demo.py`

### What to observe:
- Smooth transitions between all waypoints
- Clean stops at each waypoint
- Continuous smooth motion in a sequence

### Challenge extension (optional):
- [ ] Add keyboard control to trigger next waypoint
- [ ] Load waypoints from a file
- [ ] Add different speeds for different segments

---

## Testing & Validation

### For Task 1.1:
```bash
cd src
python trajectory_generator.py
# Should print smooth position/velocity profile
```

### For Task 1.2:
```bash
cd src
python point_to_point_demo.py
# Watch the arm move smoothly
# Feel it - push gently during motion
```

### For Task 1.3:
```bash
cd src
python waypoint_demo.py
# Watch full sequence
# Should loop or stop at end
```

---

## Success Criteria

### You're done with Phase 1 when:
- ✓ TrajectoryGenerator class works correctly
- ✓ Point-to-point motion is visibly smoother than step commands
- ✓ Can command motion with different durations
- ✓ Multi-waypoint sequences work reliably
- ✓ You understand the polynomial math (even if you didn't derive it)
- ✓ Code is documented with comments

### Bonus achievements:
- ○ Logged data and plotted position vs time
- ○ Compared torque profiles (smooth vs stepped)
- ○ Tested at different speeds and angles
- ○ Created your own custom waypoint sequence

---

## Common Issues & Solutions

### Issue: Arm doesn't move smoothly
- Check that you're sending BOTH position and velocity
- Verify kp and kd gains aren't too high
- Add gravity compensation if arm droops

### Issue: Trajectory doesn't complete
- Print elapsed time to debug
- Check that duration is reasonable (1-5 seconds)
- Verify loop rate is fast enough (100 Hz recommended)

### Issue: Math errors in trajectory generation
- Check tau calculation: should be 0 to 1
- Verify polynomial coefficients: 10, -15, +6 for position
- Test with simple values (start=0, end=1, duration=1)

---

## What You'll Learn

### Math concepts:
- Polynomial evaluation (no calculus needed to use it!)
- Normalized time (mapping arbitrary duration to 0-1)
- Derivatives (velocity is derivative of position)

### Robotics concepts:
- Trajectory generation vs path planning
- Feedforward control (using velocity command)
- Importance of smooth motion for mechanical systems

### Python skills:
- Static methods with @staticmethod
- Time management in control loops
- State machine patterns (for waypoint demo)

---

## Next Steps After Phase 1

Once you complete this phase:
1. Measure your arm's mass (for Phase 2)
2. Start Khan Academy vectors lessons
3. Begin Phase 2: Gravity Compensation

---

## Questions to Ask Yourself

After completing Phase 1:
- Can you explain why minimum jerk trajectories feel smooth?
- What happens if you change the polynomial order?
- How would you modify this for acceleration limits?
- What's the relationship between position and velocity commands?

These questions will deepen your understanding!
