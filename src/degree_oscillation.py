import time
import math
from cubemars_control import CubeMarsMotor

motor = CubeMarsMotor(motor_id=1)

try:
    motor.flush_can_buffer()
    motor.enter_motor_mode()
    time.sleep(0.5)

    # Read the position after motor mode entry (wherever it ended up)
    motor.send_command(position=0.0, velocity=0.0, kp=20.0, kd=1.0, torque=0.0)
    time.sleep(0.1)

    feedback = motor.read_feedback(timeout=0.1)
    if feedback:
        initial_position = feedback['position']
    else:
        initial_position = 0.0

    print(f"\nStarting position: {math.degrees(initial_position):.2f}°")
    print("Beginning smooth oscillation: 20° range at 0.6 Hz...")
    print("Press Ctrl+C to stop\n")

    # Oscillation parameters
    amplitude = math.radians(20)  # 20 degrees total range (±10°)
    extra_amplitude_on_reverse = math.radians(10)  # Add 10 degrees when reversing
    frequency = 0.5  # 0.6 oscillations per second
    torque_threshold = 0.3  # 0.5 Nm threshold to reverse direction
    direction_change_cooldown = 0.5  # Wait 0.5 seconds after direction change
    direction_transition_duration = 0.5  # Take 0.5 seconds to smoothly reverse direction

    # Start the oscillation from current position by offsetting the sine wave
    # We'll blend from the initial position into the oscillation over time
    blend_duration = 2.0  # 2 seconds to fully blend into oscillation

    # Direction control: 1 for forward, -1 for reverse
    direction = 1
    last_direction_change_time = 0
    direction_transition_start = None
    old_direction = 1
    new_direction = 1
    current_amplitude = amplitude  # Start with normal amplitude

    start_time = time.time()
    loop_count = 0
    while True:
        t = time.time() - start_time
        loop_count += 1

        # Calculate smooth direction multiplier if we're transitioning
        if direction_transition_start is not None:
            transition_time = t - direction_transition_start
            if transition_time < direction_transition_duration:
                # Smooth S-curve transition from old to new direction
                progress = transition_time / direction_transition_duration
                smooth_progress = 0.5 - 0.5 * math.cos(progress * math.pi)
                direction_multiplier = old_direction * (1 - smooth_progress) + new_direction * smooth_progress
            else:
                # Transition complete
                direction_multiplier = new_direction
                direction_transition_start = None
        else:
            direction_multiplier = direction

        # Calculate the natural oscillation (centered at 0) with smooth direction and current amplitude
        oscillation = direction_multiplier * current_amplitude / 2 * math.sin(2 * math.pi * frequency * t)

        # Blend from initial position to oscillation
        if t < blend_duration:
            blend_factor = 0.5 - 0.5 * math.cos(t / blend_duration * math.pi)  # S-curve
            target_position = initial_position * (1 - blend_factor) + oscillation * blend_factor
        else:
            target_position = oscillation

        # Send command
        motor.send_command(
            position=target_position,
            velocity=0.0,
            kp=20.0,
            kd=1.0,
            torque=0.0
        )

        # Read feedback and display
        feedback = motor.read_feedback(timeout=0.01)
        if feedback:
            current_deg = math.degrees(feedback['position'])
            target_deg = math.degrees(target_position)
            torque_nm = feedback['torque']

            # Check if torque exceeds threshold and enough time has passed since last change
            time_since_last_change = t - last_direction_change_time
            if abs(torque_nm) >= torque_threshold and time_since_last_change >= direction_change_cooldown:
                # Start smooth direction transition
                old_direction = direction
                new_direction = -direction
                direction = new_direction
                direction_transition_start = t
                last_direction_change_time = t
                print(f"\n*** Direction reversing smoothly! Torque: {torque_nm:.3f} Nm ***")

            # Print every 25th iteration (4 times per second at 100Hz loop)
            if loop_count % 25 == 0:
                print(
                    f"[{loop_count:5d}] Target: {target_deg:6.2f}°  Current: {current_deg:6.2f}°  Torque: {torque_nm:6.3f} Nm  Dir: {direction:+d}")
        else:
            if loop_count % 25 == 0:
                print(f"[{loop_count:5d}] No feedback received")

        time.sleep(0.01)  # 100 Hz control loop

except KeyboardInterrupt:
    print("\n\nStopping...")
    time.sleep(0.1)
except Exception as e:
    print(f"\nError: {e}")
finally:
    try:
        motor.close()
    except:
        pass
    print("Motor stopped")