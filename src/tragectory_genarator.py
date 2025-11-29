#!/usr/bin/env python3
"""
Trajectory Generator for Smooth Robot Motion

This module provides trajectory generation functions for creating smooth,
natural-looking robot movements. The primary method is minimum jerk
trajectories, which minimize the rate of change of acceleration (jerk).

Minimum jerk trajectories are what your nervous system naturally uses when
you reach for something - they feel smooth and natural!

Math Background:
    The minimum jerk trajectory uses a 5th order polynomial to interpolate
    between start and end positions. The trajectory is defined as:

    s(τ) = 10τ³ - 15τ⁴ + 6τ⁵

    where τ = t/duration (normalized time from 0 to 1)

    This polynomial has the properties:
    - s(0) = 0, s(1) = 1 (reaches start and end)
    - s'(0) = 0, s'(1) = 0 (zero velocity at start and end)
    - s''(0) = 0, s''(1) = 0 (zero acceleration at start and end)
"""

import math


class TrajectoryGenerator:
    """Generator for smooth robot trajectories"""

    @staticmethod
    def minimum_jerk_trajectory(start_pos, end_pos, duration, t):
        """
        Generate a point on a minimum jerk trajectory

        This creates a smooth path from start_pos to end_pos that minimizes
        jerk (rate of change of acceleration). The result is very smooth,
        natural-looking motion.

        Args:
            start_pos (float): Starting position (radians for revolute joints)
            end_pos (float): Ending position (radians for revolute joints)
            duration (float): Total time for the motion (seconds)
            t (float): Current time in the trajectory (seconds, 0 to duration)

        Returns:
            float: The position at time t

        Example:
            >>> # Move from 0 to 1.57 radians (90 degrees) in 2 seconds
            >>> start = 0.0
            >>> end = 1.57
            >>> duration = 2.0
            >>> 
            >>> # At halfway point (1 second)
            >>> pos = TrajectoryGenerator.minimum_jerk_trajectory(start, end, duration, 1.0)
            >>> print(f"Position at t=1.0s: {pos:.3f} rad")
        """
        # Handle edge cases
        if t <= 0:
            return start_pos
        if t >= duration:
            return end_pos

        # Normalize time to [0, 1]
        tau = t / duration

        # Minimum jerk polynomial (5th order)
        # s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
        s = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5

        # Interpolate between start and end positions
        position = start_pos + (end_pos - start_pos) * s

        return position

    @staticmethod
    def minimum_jerk_velocity(start_pos, end_pos, duration, t):
        """
        Generate the velocity for a minimum jerk trajectory

        This is the time derivative of the minimum jerk position trajectory.
        Useful for feedforward velocity control.

        Args:
            start_pos (float): Starting position (radians)
            end_pos (float): Ending position (radians)
            duration (float): Total time for the motion (seconds)
            t (float): Current time in the trajectory (seconds, 0 to duration)

        Returns:
            float: The velocity at time t (radians/second)

        Math:
            If s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
            Then s'(τ) = 30τ² - 60τ³ + 30τ⁴

            And velocity = (end_pos - start_pos) * s'(τ) / duration

        Example:
            >>> # Get velocity at t=1.0s for a 2-second motion
            >>> vel = TrajectoryGenerator.minimum_jerk_velocity(0.0, 1.57, 2.0, 1.0)
            >>> print(f"Velocity at t=1.0s: {vel:.3f} rad/s")
        """
        # Handle edge cases - velocity is zero at start and end
        if t <= 0 or t >= duration:
            return 0.0

        # Normalize time to [0, 1]
        tau = t / duration

        # Derivative of minimum jerk polynomial
        # s'(τ) = 30τ² - 60τ³ + 30τ⁴
        s_dot = 30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4

        # Scale by distance and account for time normalization
        velocity = (end_pos - start_pos) * s_dot / duration

        return velocity

    @staticmethod
    def minimum_jerk_acceleration(start_pos, end_pos, duration, t):
        """
        Generate the acceleration for a minimum jerk trajectory

        This is the second derivative of position. Can be useful for
        dynamics calculations or feedforward control.

        Args:
            start_pos (float): Starting position (radians)
            end_pos (float): Ending position (radians)
            duration (float): Total time for the motion (seconds)
            t (float): Current time in the trajectory (seconds)

        Returns:
            float: The acceleration at time t (radians/second²)

        Math:
            If s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
            Then s''(τ) = 60τ - 180τ² + 120τ³
        """
        # Handle edge cases - acceleration is zero at start and end
        if t <= 0 or t >= duration:
            return 0.0

        # Normalize time to [0, 1]
        tau = t / duration

        # Second derivative of minimum jerk polynomial
        # s''(τ) = 60τ - 180τ² + 120τ³
        s_ddot = 60 * tau - 180 * tau ** 2 + 120 * tau ** 3

        # Scale by distance and account for time normalization (twice)
        acceleration = (end_pos - start_pos) * s_ddot / (duration ** 2)

        return acceleration


def degrees_to_radians(degrees):
    """Helper: Convert degrees to radians"""
    return degrees * math.pi / 180.0


def radians_to_degrees(radians):
    """Helper: Convert radians to degrees"""
    return radians * 180.0 / math.pi


# Test code - runs when you execute this file directly
if __name__ == "__main__":
    print("=" * 60)
    print("Trajectory Generator Test")
    print("=" * 60)

    # Test parameters
    start = 0.0
    end = math.pi / 2  # 90 degrees
    duration = 2.0

    print(f"\nTest trajectory:")
    print(f"  Start: {radians_to_degrees(start):.1f}°")
    print(f"  End: {radians_to_degrees(end):.1f}°")
    print(f"  Duration: {duration}s")
    print()

    # Generate trajectory points
    print(f"{'Time (s)':<10} {'Position (°)':<15} {'Velocity (°/s)':<18} {'Accel (°/s²)':<15}")
    print("-" * 60)

    num_points = 21
    for i in range(num_points):
        t = i * duration / (num_points - 1)

        pos = TrajectoryGenerator.minimum_jerk_trajectory(start, end, duration, t)
        vel = TrajectoryGenerator.minimum_jerk_velocity(start, end, duration, t)
        acc = TrajectoryGenerator.minimum_jerk_acceleration(start, end, duration, t)

        # Convert to degrees for display
        pos_deg = radians_to_degrees(pos)
        vel_deg = radians_to_degrees(vel)
        acc_deg = radians_to_degrees(acc)

        print(f"{t:<10.2f} {pos_deg:<15.2f} {vel_deg:<18.2f} {acc_deg:<15.2f}")

    print("\n" + "=" * 60)
    print("Validation Checks:")
    print("=" * 60)

    # Validate start and end conditions
    pos_start = TrajectoryGenerator.minimum_jerk_trajectory(start, end, duration, 0.0)
    pos_end = TrajectoryGenerator.minimum_jerk_trajectory(start, end, duration, duration)
    vel_start = TrajectoryGenerator.minimum_jerk_velocity(start, end, duration, 0.0)
    vel_end = TrajectoryGenerator.minimum_jerk_velocity(start, end, duration, duration)
    acc_start = TrajectoryGenerator.minimum_jerk_acceleration(start, end, duration, 0.0)
    acc_end = TrajectoryGenerator.minimum_jerk_acceleration(start, end, duration, duration)

    print(f"✓ Position at t=0: {radians_to_degrees(pos_start):.6f}° (should be {radians_to_degrees(start):.6f}°)")
    print(f"✓ Position at t={duration}: {radians_to_degrees(pos_end):.6f}° (should be {radians_to_degrees(end):.6f}°)")
    print(f"✓ Velocity at t=0: {radians_to_degrees(vel_start):.6f}°/s (should be 0.0°/s)")
    print(f"✓ Velocity at t={duration}: {radians_to_degrees(vel_end):.6f}°/s (should be 0.0°/s)")
    print(f"✓ Accel at t=0: {radians_to_degrees(acc_start):.6f}°/s² (should be 0.0°/s²)")
    print(f"✓ Accel at t={duration}: {radians_to_degrees(acc_end):.6f}°/s² (should be 0.0°/s²)")

    # Find maximum velocity
    max_vel = 0.0
    max_vel_time = 0.0
    for i in range(1000):
        t = i * duration / 999
        vel = TrajectoryGenerator.minimum_jerk_velocity(start, end, duration, t)
        if abs(vel) > abs(max_vel):
            max_vel = vel
            max_vel_time = t

    print(f"\nMax velocity: {radians_to_degrees(max_vel):.2f}°/s at t={max_vel_time:.3f}s")
    print(f"(Should occur around t={duration / 2:.3f}s for symmetric trajectory)")

    print("\n" + "=" * 60)
    print("Test complete! If all checks passed, the trajectory is working.")
    print("=" * 60)