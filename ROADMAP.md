# Motor Control Improvement Roadmap

## Project Goal
Improve control of CubeMars AK40-10 motor with 6.5" arm attachment, building foundation for humanoid robot development.

## Learning Tracks (Parallel)

### Math Track (30-60 min/day)
- [ ] **Week 1-2**: Vectors basics (Khan Academy)
  - What vectors are
  - Vector addition, subtraction
  - Dot product, cross product
  - Vector magnitude and normalization
- [ ] **Week 3-4**: Matrices basics (Khan Academy)
  - Matrix multiplication
  - Matrix transpose
  - Identity matrix
  - Matrix inverse
- [ ] **Week 5-6**: 2D/3D Rotations
  - Rotation matrices
  - How rotations compose
  - Euler angles basics

**Resource**: Khan Academy Linear Algebra course OR "No Bullshit Guide to Linear Algebra"

### Robotics Track (Main Development)

---

## Phase 1: Better Trajectory Generation (Weeks 1-2)

### Objective
Replace simple sinusoidal motion with smooth, natural trajectories

### Tasks
- [ ] **Task 1.1**: Implement TrajectoryGenerator class
  - [ ] Create `src/trajectory_generator.py`
  - [ ] Implement `minimum_jerk_trajectory()` method
  - [ ] Implement `minimum_jerk_velocity()` method
  - [ ] Add docstrings with math explanation

- [ ] **Task 1.2**: Create point-to-point motion demo
  - [ ] Create `src/point_to_point_demo.py`
  - [ ] Move from 0° to 45° in 2 seconds
  - [ ] Compare with direct step command
  - [ ] Log data to visualize smoothness

- [ ] **Task 1.3**: Create multi-waypoint demo
  - [ ] Create `src/waypoint_demo.py`
  - [ ] Visit sequence of angles: [0°, 45°, -30°, 60°, 0°]
  - [ ] Smooth transitions between waypoints
  - [ ] Add pause duration at each waypoint

### Success Criteria
- Smooth, jerk-free motion visible to the eye
- No sudden velocity changes
- Clean torque profiles (check feedback)

### Math Concepts Used
- Polynomial evaluation (5th order)
- Time normalization
- Derivatives (velocity from position)

---

## Phase 2: Gravity Compensation (Weeks 3-4)

### Objective
Add feedforward torque to counteract gravity, improving tracking performance

### Tasks
- [ ] **Task 2.1**: Physical measurements
  - [ ] Weigh the arm (in grams)
  - [ ] Measure total length (already: 6.5" = 0.165m)
  - [ ] Estimate center of mass location
    - For uniform rod: length/2
    - Or balance on a point to find it
  - [ ] Document measurements

- [ ] **Task 2.2**: Implement ArmDynamics class
  - [ ] Create `src/arm_dynamics.py`
  - [ ] Implement `gravity_torque()` method
  - [ ] Add validation: torque should be max at horizontal (0°)
  - [ ] Add validation: torque should be zero at vertical (±90°)

- [ ] **Task 2.3**: Test gravity compensation
  - [ ] Modify `degree_oscillation.py` to use gravity compensation
  - [ ] Compare tracking error with/without compensation
  - [ ] Log position error over time
  - [ ] Measure improvement in tracking accuracy

- [ ] **Task 2.4**: Create hold position demo
  - [ ] Create `src/hold_position_demo.py`
  - [ ] Command motor to hold at various angles
  - [ ] Measure steady-state position error
  - [ ] Tune gravity compensation if needed (adjust mass/CoM)

### Success Criteria
- Reduced position droop when holding horizontal
- Better tracking during motion
- Quantified improvement (degrees of error reduction)

### Math Concepts Used
- Trigonometry: cos(θ) for torque projection
- Basic physics: torque = force × distance
- Unit conversions (inches to meters, grams to kg)

---

## Phase 3: PID Tuning & Analysis (Weeks 5-6)

### Objective
Systematically tune control gains and understand system response

### Tasks
- [ ] **Task 3.1**: Implement PIDTuner class
  - [ ] Create `src/pid_tuner.py`
  - [ ] Implement `step_response_test()` method
  - [ ] Implement response analysis (settling time, overshoot, steady-state error)
  - [ ] Add data logging capability

- [ ] **Task 3.2**: Characterize current system
  - [ ] Run step response tests with current gains (kp=20, kd=1)
  - [ ] Test at different positions (horizontal, vertical, angles)
  - [ ] Document baseline performance
  - [ ] Save response plots/data

- [ ] **Task 3.3**: Systematic gain tuning
  - [ ] Test range of kp values: [10, 20, 30, 40, 50]
  - [ ] Test range of kd values: [0.5, 1.0, 1.5, 2.0]
  - [ ] Find optimal gains for:
    - Fast response with minimal overshoot
    - Position holding (stiffness)
    - Compliant interaction (for later human interaction)
  - [ ] Document findings

- [ ] **Task 3.4**: Create gain profiles
  - [ ] Implement switchable gain sets for different behaviors
  - [ ] "Stiff mode" - high kp, high kd
  - [ ] "Compliant mode" - low kp, moderate kd
  - [ ] "Fast mode" - tuned for speed
  - [ ] Add keyboard controls to switch modes

### Success Criteria
- Documented gain settings for different behaviors
- Understanding of kp/kd effects on system response
- Quantified performance metrics

### Math Concepts Used
- Basic statistics (averaging for steady-state)
- Percentage calculations (overshoot)
- Time-series analysis concepts

---

## Phase 4: Kinematics & Cartesian Control (Weeks 7-8)

### Objective
Control the tip of the arm in Cartesian coordinates rather than joint angles

### Tasks
- [ ] **Task 4.1**: Implement SingleArmKinematics class
  - [ ] Create `src/kinematics.py`
  - [ ] Implement `forward_kinematics()` - angle to (x,y) position
  - [ ] Implement `jacobian()` - relates joint velocity to tip velocity
  - [ ] Implement `velocity_control()` - inverse Jacobian
  - [ ] Add visualization helper (print tip position)

- [ ] **Task 4.2**: Forward kinematics visualization
  - [ ] Create `src/fk_demo.py`
  - [ ] Move joint through range of motion
  - [ ] Print/log end effector position
  - [ ] Verify against hand calculations
  - [ ] Optional: Create simple 2D plot

- [ ] **Task 4.3**: Inverse kinematics (analytical)
  - [ ] Implement `inverse_kinematics()` for 1-DOF case (trivial: atan2)
  - [ ] Create `src/ik_demo.py`
  - [ ] Command Cartesian positions, compute required angles
  - [ ] Example: move to (0.1, 0.1), then (0.15, 0.0), etc.

- [ ] **Task 4.4**: Cartesian velocity control
  - [ ] Create `src/cartesian_velocity_demo.py`
  - [ ] Implement circle drawing with tip
  - [ ] Implement straight line following
  - [ ] Try different speeds
  - [ ] Log actual vs desired tip trajectory

- [ ] **Task 4.5**: Path following
  - [ ] Create `src/path_following_demo.py`
  - [ ] Define paths: circle, figure-8, square
  - [ ] Generate trajectory in Cartesian space
  - [ ] Use Jacobian to control joint
  - [ ] Evaluate tracking accuracy

### Success Criteria
- Arm tip follows desired Cartesian paths
- Understanding of forward/inverse kinematics
- Successful Jacobian-based velocity control
- Foundation for multi-joint control

### Math Concepts Used
- **Trigonometry**: sin/cos for forward kinematics
- **Linear algebra**: Matrix operations, pseudo-inverse
- **Calculus**: Jacobian is matrix of partial derivatives
- **NumPy**: np.linalg.pinv(), matrix multiplication with @

---

## Phase 5: Advanced Control (Weeks 9-10+)

### Future Improvements (After Phases 1-4)

- [ ] **Data Logging System**
  - CSV logging of all motor data
  - Plotting tools for analysis
  - Performance comparison tools

- [ ] **Impedance Control**
  - Variable stiffness control
  - Force-based interaction
  - Compliant behavior tuning

- [ ] **Trajectory Optimization**
  - Time-optimal trajectories
  - Energy-optimal trajectories
  - Obstacle avoidance (for multi-joint)

- [ ] **State Machine Architecture**
  - Different control modes
  - Safe transitions
  - Error handling and recovery

---

## Hardware Expansion Planning

### Next Steps After Single Motor Mastery
1. Add second motor for 2-DOF planar arm
2. Implement 2D forward/inverse kinematics
3. Coordinate multiple motors
4. Build toward full humanoid limb

---

## Progress Tracking

### Completed Phases
- [x] Phase 0: Basic motor control (current state)
  - [x] CAN communication
  - [x] MIT Mini Cheetah protocol
  - [x] Basic oscillation
  - [x] Torque-based interaction

### Current Phase
Phase 1: Better Trajectory Generation

### Notes & Learnings
(Add your observations, challenges, and insights here as you work)

---

## Resources

### Math Learning
- Khan Academy: Linear Algebra
- Khan Academy: Trigonometry
- "No Bullshit Guide to Linear Algebra" by Ivan Savov

### Robotics Theory
- "Modern Robotics" by Lynch & Park (reference when ready)
- "Introduction to Autonomous Robots" (more accessible)
- "Robotics, Vision and Control" by Peter Corke

### Python/Programming
- NumPy documentation for linear algebra
- python-can documentation
- Your existing codebase as examples

---

## Quick Reference

### Motor Specifications (AK40-10)
- Position range: -12.5 to 12.5 radians
- Velocity range: -45 to 45 rad/s
- Torque range: -18 to 18 Nm
- Rated torque: ~10 Nm
- Current control gains: kp=20.0, kd=1.0

### Arm Specifications
- Length: 6.5 inches (0.165 meters)
- Mass: [TODO: measure and record]
- Center of mass: [TODO: measure and record]
- Material: 3D printed [material?]

### Development Environment
- Platform: macOS (development) → Raspberry Pi (deployment)
- CAN interface: can0 @ 1Mbps
- Python version: [record your version]
- Key libraries: python-can, NumPy
