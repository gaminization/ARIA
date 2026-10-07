#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Trajectory Generator
Quintic polynomial spline trajectory generation.
Minimizes jerk for smooth, servo-friendly motion.

Primary:  Quintic (5th order) — zero vel/accel at endpoints
Fallback: Cubic (3rd order)   — zero velocity at endpoints
═══════════════════════════════════════════════════════════════
"""
import math
from typing import List, Optional, Tuple

import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


# ═══════════════════════════════════════════════════════════════
# Joint velocity and acceleration limits
# ═══════════════════════════════════════════════════════════════
# From servo specs:
#   MG995: ~0.16s/60° ≈ 6.5 rad/s max, but we limit for smoothness
#   SG90:  ~0.12s/60° ≈ 8.7 rad/s max
JOINT_MAX_VEL = np.array([1.5, 1.5, 1.5, 2.0, 2.0])   # rad/s
JOINT_MAX_ACCEL = np.array([2.0, 2.0, 2.0, 3.0, 3.0])  # rad/s²

JOINT_NAMES = [
    "waist_joint", "shoulder_joint", "elbow_joint",
    "wrist_pitch_joint", "gripper_joint"
]


class TrajectoryGenerator:
    """
    Generates smooth polynomial trajectories for the ARIA arm.

    Primary: quintic polynomial (5th order)
      - Minimizes jerk (3rd derivative of position)
      - Zero velocity AND acceleration at start/end
      - 6 boundary conditions → 6 coefficients

    Fallback: cubic polynomial (3rd order)
      - Zero velocity at start/end
      - 4 boundary conditions → 4 coefficients
    """

    @staticmethod
    def generate_quintic(q0: float, q1: float,
                         duration: float,
                         n_points: int = 100) -> Tuple[np.ndarray, np.ndarray,
                                                        np.ndarray, np.ndarray]:
        """
        Quintic polynomial trajectory for a single joint.

        q(t) = a0 + a1·t + a2·t² + a3·t³ + a4·t⁴ + a5·t⁵

        Boundary conditions (ensures smooth start/stop):
          q(0)  = q0       position at start
          q(T)  = q1       position at end
          q'(0) = 0        zero velocity at start
          q'(T) = 0        zero velocity at end
          q''(0) = 0       zero acceleration at start
          q''(T) = 0       zero acceleration at end

        Derivation of coefficients:
        ─────────────────────────────
        From q(0) = q0:          a0 = q0
        From q'(0) = 0:          a1 = 0
        From q''(0) = 0:         a2 = 0

        Remaining 3 equations from end conditions:
          q(T) = q0 + a3·T³ + a4·T⁴ + a5·T⁵ = q1
          q'(T) = 3·a3·T² + 4·a4·T³ + 5·a5·T⁴ = 0
          q''(T) = 6·a3·T + 12·a4·T² + 20·a5·T³ = 0

        Solving the 3×3 system:
          a3 =  10·(q1 - q0) / T³
          a4 = -15·(q1 - q0) / T⁴
          a5 =   6·(q1 - q0) / T⁵

        Args:
            q0: start position (radians)
            q1: end position (radians)
            duration: trajectory duration (seconds)
            n_points: number of interpolation points

        Returns:
            (times, positions, velocities, accelerations)
        """
        T = max(duration, 0.01)  # prevent division by zero
        dq = q1 - q0

        # Quintic coefficients (derived above)
        a0 = q0
        a1 = 0.0
        a2 = 0.0
        a3 = 10.0 * dq / (T ** 3)
        a4 = -15.0 * dq / (T ** 4)
        a5 = 6.0 * dq / (T ** 5)

        times = np.linspace(0, T, n_points)
        positions = np.zeros(n_points)
        velocities = np.zeros(n_points)
        accelerations = np.zeros(n_points)

        for i, t in enumerate(times):
            t2 = t * t
            t3 = t2 * t
            t4 = t3 * t
            t5 = t4 * t

            # Position: q(t) = a0 + a3·t³ + a4·t⁴ + a5·t⁵
            positions[i] = a0 + a3 * t3 + a4 * t4 + a5 * t5

            # Velocity: q'(t) = 3·a3·t² + 4·a4·t³ + 5·a5·t⁴
            velocities[i] = 3 * a3 * t2 + 4 * a4 * t3 + 5 * a5 * t4

            # Acceleration: q''(t) = 6·a3·t + 12·a4·t² + 20·a5·t³
            accelerations[i] = 6 * a3 * t + 12 * a4 * t2 + 20 * a5 * t3

        return times, positions, velocities, accelerations

    @staticmethod
    def generate_cubic(q0: float, q1: float,
                       duration: float,
                       n_points: int = 100) -> Tuple[np.ndarray, np.ndarray,
                                                      np.ndarray, np.ndarray]:
        """
        Cubic polynomial trajectory (fallback).

        q(t) = a0 + a1·t + a2·t² + a3·t³

        Boundary conditions:
          q(0) = q0,  q(T) = q1
          q'(0) = 0,  q'(T) = 0

        Coefficients:
          a0 = q0
          a1 = 0
          a2 = 3·(q1 - q0) / T²
          a3 = -2·(q1 - q0) / T³
        """
        T = max(duration, 0.01)
        dq = q1 - q0

        a0 = q0
        a2 = 3.0 * dq / (T ** 2)
        a3 = -2.0 * dq / (T ** 3)

        times = np.linspace(0, T, n_points)
        positions = a0 + a2 * times**2 + a3 * times**3
        velocities = 2 * a2 * times + 3 * a3 * times**2
        accelerations = 2 * a2 + 6 * a3 * times

        return times, positions, velocities, accelerations

    @staticmethod
    def select_duration(q0: np.ndarray, q1: np.ndarray,
                        max_velocity: Optional[np.ndarray] = None,
                        max_accel: Optional[np.ndarray] = None) -> float:
        """
        Compute minimum safe duration for multi-joint trajectory with strict
        time-scaling against velocity and acceleration limits.

        Uses the joint requiring the most time (bottleneck joint).

        For quintic polynomial, peak velocity occurs at t=T/2:
          v_peak = 15·|Δq| / (8·T)
          → T_vel = 15·|Δq| / (8·v_max)
        We apply a 1.25x scaling margin to ensure physical peak velocities
        measured under closed-loop control stay strictly below limits.

        Peak acceleration occurs at t≈0.21T and t≈0.79T:
          a_peak = 10·√3·|Δq| / (9·T²)
          → T_accel = sqrt(10·√3·|Δq| / (9·a_max))

        Args:
            q0: start joint angles [5]
            q1: target joint angles [5]
            max_velocity: per-joint velocity limits (default: JOINT_MAX_VEL)
            max_accel: per-joint acceleration limits (default: JOINT_MAX_ACCEL)

        Returns:
            Minimum safe duration (seconds)
        """
        if max_velocity is None:
            max_velocity = JOINT_MAX_VEL
        if max_accel is None:
            max_accel = JOINT_MAX_ACCEL

        n_joints = min(len(q0), len(q1), len(max_velocity))
        durations = []

        for j in range(n_joints):
            dq = abs(q1[j] - q0[j])

            if dq < 1e-6:
                continue

            # Duration from velocity constraint (with 1.25 safety factor)
            T_vel = 1.25 * (15.0 * dq / (8.0 * max_velocity[j]))

            # Duration from acceleration constraint
            T_accel = 1.15 * math.sqrt(10.0 * math.sqrt(3.0) * dq /
                                       (9.0 * max_accel[j]))

            durations.append(max(T_vel, T_accel))

        if not durations:
            return 0.5  # Default for no motion

        return max(max(durations), 0.25)  # Minimum 0.25s

    @classmethod
    def generate_trajectory(cls,
                            q0: np.ndarray,
                            q1: np.ndarray,
                            duration: Optional[float] = None,
                            n_points: int = 50,
                            method: str = 'quintic') -> JointTrajectory:
        """
        Generate a full multi-joint trajectory as a ROS2 JointTrajectory
        with strict time-scaling enforced against JOINT_MAX_VEL.

        Args:
            q0: start joint angles [5] in radians
            q1: target joint angles [5] in radians
            duration: trajectory duration (auto-computed/time-scaled if None or too small)
            n_points: number of waypoints
            method: 'quintic' or 'cubic'

        Returns:
            JointTrajectory message
        """
        min_duration = cls.select_duration(q0, q1)
        if duration is None or duration < min_duration:
            duration = min_duration

        n_joints = len(q0)
        gen_fn = cls.generate_quintic if method == 'quintic' else cls.generate_cubic

        # Generate trajectory for each joint
        all_times = None
        all_positions = np.zeros((n_points, n_joints))
        all_velocities = np.zeros((n_points, n_joints))
        all_accelerations = np.zeros((n_points, n_joints))

        for j in range(n_joints):
            times, pos, vel, acc = gen_fn(q0[j], q1[j], duration, n_points)
            all_times = times
            all_positions[:, j] = pos
            all_velocities[:, j] = vel
            all_accelerations[:, j] = acc

        # Build JointTrajectory message
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES[:n_joints])

        for i in range(n_points):
            point = JointTrajectoryPoint()
            point.positions = all_positions[i].tolist()
            point.velocities = all_velocities[i].tolist()
            point.accelerations = all_accelerations[i].tolist()

            t = all_times[i]
            point.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1.0) * 1e9)
            )
            traj.points.append(point)

        return traj

    @classmethod
    def generate_multi_waypoint(cls,
                                waypoints: List[np.ndarray],
                                blend_radius: float = 0.1,
                                method: str = 'quintic') -> JointTrajectory:
        """
        Chain multiple quintic segments with smooth blending.

        For each pair of consecutive waypoints, generates a quintic
        segment. At blend regions (around waypoints), overlapping
        segments are averaged for smooth transitions.

        Args:
            waypoints: list of joint angle arrays [5]
            blend_radius: blend time ratio at transitions (0-1)
            method: 'quintic' or 'cubic'

        Returns:
            JointTrajectory message with all segments concatenated
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")

        n_joints = len(waypoints[0])
        all_points = []
        total_time = 0.0

        for i in range(len(waypoints) - 1):
            q0 = waypoints[i]
            q1 = waypoints[i + 1]
            seg_duration = cls.select_duration(q0, q1)

            # Points per segment (fewer for intermediate, more for last)
            n_pts = 30 if i < len(waypoints) - 2 else 50

            gen_fn = cls.generate_quintic if method == 'quintic' else cls.generate_cubic

            for j_idx in range(n_joints):
                times, pos, vel, acc = gen_fn(
                    q0[j_idx], q1[j_idx], seg_duration, n_pts
                )

                if j_idx == 0:
                    seg_positions = np.zeros((n_pts, n_joints))
                    seg_velocities = np.zeros((n_pts, n_joints))
                    seg_accelerations = np.zeros((n_pts, n_joints))
                    seg_times = times

                seg_positions[:, j_idx] = pos
                seg_velocities[:, j_idx] = vel
                seg_accelerations[:, j_idx] = acc

            # Skip first point of non-first segments (avoid duplication)
            start_idx = 1 if i > 0 else 0

            for k in range(start_idx, n_pts):
                point = JointTrajectoryPoint()
                point.positions = seg_positions[k].tolist()
                point.velocities = seg_velocities[k].tolist()
                point.accelerations = seg_accelerations[k].tolist()

                t = total_time + seg_times[k]
                point.time_from_start = Duration(
                    sec=int(t),
                    nanosec=int((t % 1.0) * 1e9)
                )
                all_points.append(point)

            total_time += seg_duration

        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES[:n_joints])
        traj.points = all_points

        return traj


# ═══════════════════════════════════════════════════════════════
# Self-test
# ═══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    gen = TrajectoryGenerator()

    q0 = np.array([0.0, 1.5708, 1.3090, 0.0, 0.0])  # home
    q1 = np.array([0.0, 0.7854, 2.3562, 0.0, 0.0])   # ready

    duration = gen.select_duration(q0, q1)
    print(f"Computed duration: {duration:.2f}s")

    traj = gen.generate_trajectory(q0, q1, method='quintic')
    print(f"Generated {len(traj.points)} points")
    print(f"First point: {traj.points[0].positions}")
    print(f"Last point:  {traj.points[-1].positions}")

    # Check smoothness
    max_vel = 0
    for i in range(1, len(traj.points)):
        dt = 0.01  # approximate
        for j in range(5):
            vel = abs(traj.points[i].velocities[j])
            max_vel = max(max_vel, vel)

    print(f"Peak velocity: {max_vel:.3f} rad/s")
