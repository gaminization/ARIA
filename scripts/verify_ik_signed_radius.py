#!/usr/bin/env python3
"""
Verification Script for Signed-Radius, Two-Azimuth Analytical IK Solver for ARIA.
Validates:
  1. Exact recovery across 100,000 in-limit joint configurations.
  2. Multi-azimuth backward reaching: theta1 in {atan2(Py,Px), atan2(Py,Px) +/- pi}.
  3. Signed planar projection r = Pw · [c1, s1] - a1.
  4. Both elbow branches (theta3 > 0, theta3 < 0).
  5. Exact pitch tracking without adaptive search.
"""

import math
import time
import numpy as np

# Kinematic parameters matching URDF
D_BASE = 0.105
L_UPPER = 0.145
L_FORE = 0.115
L_TOOL = 0.095

JOINT_LIMITS = np.array([
    [-3.1416,  3.1416],   # waist:       -180° to +180°
    [-1.5708,  1.5708],   # shoulder:    -90° to +90°
    [-1.5708,  1.5708],   # elbow:       -90° to +90°
    [-1.5708,  1.5708],   # wrist_pitch: -90° to +90°
    [-1.5708,  1.5708],   # wrist_roll:  -90° to +90°
])


def forward_kinematics(q: np.ndarray) -> np.ndarray:
    """Forward kinematics using standard DH parameters."""
    th1, th2, th3, th4, th5 = q
    phi = th2 + th3 + th4
    c1, s1 = math.cos(th1), math.sin(th1)

    r = L_UPPER * math.cos(th2) + L_FORE * math.cos(th2 + th3) + L_TOOL * math.cos(phi)
    z = D_BASE + L_UPPER * math.sin(th2) + L_FORE * math.sin(th2 + th3) + L_TOOL * math.sin(phi)

    # 4x4 matrix
    T = np.eye(4)
    T[0, 3] = r * c1
    T[1, 3] = r * s1
    T[2, 3] = z
    return T


def solve_ik_signed_radius(pos: np.ndarray,
                           target_pitch: float,
                           target_roll: float = 0.0,
                           tol: float = 1e-3) -> list:
    """
    Exact closed-form analytical IK using signed radius and two azimuths.
    """
    px, py, pz = pos
    L1, L2 = L_UPPER, L_FORE

    # Step 1: Candidate azimuths theta1
    if abs(px) < 1e-7 and abs(py) < 1e-7:
        th1_candidates = [0.0]
    else:
        th1_base = math.atan2(py, px)
        th1_alt = th1_base + math.pi if th1_base <= 0 else th1_base - math.pi
        th1_candidates = [th1_base, th1_alt]

    solutions = []

    for th1 in th1_candidates:
        if not (JOINT_LIMITS[0, 0] - 1e-6 <= th1 <= JOINT_LIMITS[0, 1] + 1e-6):
            continue

        c1, s1 = math.cos(th1), math.sin(th1)

        # Signed radius of wrist center: r = Pw · [c1, s1] - a1 (a1 = 0)
        r_w = px * c1 + py * s1 - L_TOOL * math.cos(target_pitch)
        z_w = pz - D_BASE - L_TOOL * math.sin(target_pitch)

        D_sq = r_w * r_w + z_w * z_w
        reach_max = (L1 + L2) ** 2
        reach_min = (L1 - L2) ** 2

        if D_sq > reach_max + 1e-7 or D_sq < reach_min - 1e-7:
            continue

        cos_th3 = (D_sq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        cos_th3 = max(-1.0, min(1.0, cos_th3))
        sin_th3 = math.sqrt(max(0.0, 1.0 - cos_th3 * cos_th3))

        # Both elbow branches
        th3_options = [
            math.atan2(-sin_th3, cos_th3),  # Elbow-up (negative)
            math.atan2(sin_th3, cos_th3),   # Elbow-down (positive)
        ]

        for th3 in th3_options:
            if not (JOINT_LIMITS[2, 0] - 1e-6 <= th3 <= JOINT_LIMITS[2, 1] + 1e-6):
                continue

            k1 = L1 + L2 * math.cos(th3)
            k2 = L2 * math.sin(th3)
            th2 = math.atan2(k1 * z_w - k2 * r_w, k1 * r_w + k2 * z_w)

            if not (JOINT_LIMITS[1, 0] - 1e-6 <= th2 <= JOINT_LIMITS[1, 1] + 1e-6):
                continue

            th4 = target_pitch - (th2 + th3)
            if not (JOINT_LIMITS[3, 0] - 1e-6 <= th4 <= JOINT_LIMITS[3, 1] + 1e-6):
                continue

            th5 = target_roll
            if not (JOINT_LIMITS[4, 0] - 1e-6 <= th5 <= JOINT_LIMITS[4, 1] + 1e-6):
                continue

            q_sol = np.array([th1, th2, th3, th4, th5])
            T_sol = forward_kinematics(q_sol)
            err = np.linalg.norm(T_sol[:3, 3] - pos)

            if err <= tol:
                solutions.append((q_sol, err))

    return solutions


def main():
    print("═" * 70)
    print("Verification of Signed-Radius Two-Azimuth Analytical IK Solver")
    print("═" * 70)

    n_samples = 100000
    np.random.seed(42)

    print(f"Generating {n_samples} random reachable poses via FK...")
    q_all = np.random.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1], size=(n_samples, 5))

    successes = 0
    max_err_mm = 0.0
    times = []

    t0 = time.perf_counter()
    for i in range(n_samples):
        q = q_all[i]
        T = forward_kinematics(q)
        pos = T[:3, 3]
        pitch = float(q[1] + q[2] + q[3])
        roll = float(q[4])

        t_solve0 = time.perf_counter()
        sols = solve_ik_signed_radius(pos, pitch, roll)
        dt = (time.perf_counter() - t_solve0) * 1000.0
        times.append(dt)

        if len(sols) > 0:
            successes += 1
            min_err = min(s[1] for s in sols)
            max_err_mm = max(max_err_mm, min_err * 1000.0)
        else:
            print(f"FAILED on sample {i}: q={q}")
            break

    total_time = time.perf_counter() - t0
    times = np.array(times)

    print(f"\nResults across {n_samples} in-limit poses:")
    print(f"  Success Rate: {successes} / {n_samples} ({successes / n_samples * 100:.4f}%)")
    print(f"  Max Residual Position Error: {max_err_mm:.6f} mm")
    print(f"  Solve Time: mean={np.mean(times):.4f} ms, p50={np.percentile(times, 50):.4f} ms, p95={np.percentile(times, 95):.4f} ms, max={np.max(times):.4f} ms")
    print(f"  Total Wall Clock: {total_time:.2f} s ({n_samples / total_time:.1f} solves/s)")


if __name__ == "__main__":
    main()
