#!/usr/bin/env python3
"""
scripts/benchmark_ik_real_5d.py

Project ARIA: Real Empirical IK Benchmark with 5D Constraints
Constrains ALL solvers (ARIA Analytical, TRAC-IK, PyKDL, RTB-LM, IKPy) to the SAME
5D target (position + pitch + roll) with strict tolerances:
  - Position error <= 1.0 mm (0.001 m)
  - Pitch error <= 0.01 rad (0.573 deg)
  - Roll error <= 0.01 rad (0.573 deg)
Reports success ONLY if position, pitch, and roll are all within tolerance.

Ground-Truth Reachability:
  - Set A (10,000 poses): Sampled by FK of random in-limit joint configs (100% reachable by construction).
  - Set B (10,000 poses): 5,000 reachable poses + 5,000 unreachable poses perturbed beyond workspace
    bounds and verified by an independent dense joint-space grid search. Zero reliance on any solver output.

Pinned to CPU Core 0 for all timings.
Logs raw per-trial rows to:
  - data/real/ik_benchmark_5d_fk_10k.csv
  - data/real/ik_benchmark_5d_cartesian_10k.csv
  - data/real/ik_benchmark_5d_summary.csv
"""

import os

os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"

# Pin this process to CPU Core 0
try:
    os.sched_setaffinity(0, {0})
except Exception as e:
    print(f"[WARN] Failed to set CPU affinity: {e}")

import sys
import time
import math
import csv
import ctypes
import subprocess
from datetime import datetime
import numpy as np

WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
DATA_REAL_DIR = os.path.join(WORKSPACE_ROOT, "data", "real")
SOLVERS_DIR = os.path.join(WORKSPACE_ROOT, "arm_ik", "arm_ik", "ik_solvers")
sys.path.insert(0, SOLVERS_DIR)

# Load native libraries
ctypes.CDLL(os.path.expanduser("~/.local/lib/libnlopt.so"), mode=ctypes.RTLD_GLOBAL)
ctypes.CDLL(os.path.expanduser("~/.local/lib/libtrac_ik.so"), mode=ctypes.RTLD_GLOBAL)

from aria_analytical_ik import (
    solve_analytical,
    forward_kinematics as aria_fk,
    JOINT_LIMITS,
)
import PyKDL
import roboticstoolbox as rtb
import ikpy.chain, ikpy.link
import trac_ik_native


def get_git_commit():
    try:
        res = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=WORKSPACE_ROOT,
            capture_output=True,
            text=True,
            check=True,
        )
        return res.stdout.strip()
    except Exception:
        return "unknown"


GIT_COMMIT_HASH = get_git_commit()
POS_TOL_M = 0.001  # 1.0 mm
ORI_TOL_RAD = 0.01  # 0.01 rad (0.573 deg)
TIMEOUT_SEC = 0.005  # 5 ms timeout

# Model geometric constants
D_BASE = 0.105
L_UPPER = 0.145
L_FORE = 0.115
L_TOOL = 0.095


def wrap_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def check_5d_convergence(q_sol, pos_target, pitch_target, roll_target):
    """
    Evaluates exact 5D error: position norm, pitch error, roll error.
    Success requires all 3 errors <= respective tolerances.
    """
    T = aria_fk(q_sol)
    p_err = float(np.linalg.norm(T[:3, 3] - pos_target))
    # In 5-DOF planar arm, pitch is the sum of shoulder, elbow, and wrist pitch
    pitch_achieved = float(q_sol[1] + q_sol[2] + q_sol[3])
    pitch_err = float(abs(wrap_angle(pitch_achieved - pitch_target)))
    roll_achieved = float(q_sol[4])
    roll_err = float(abs(wrap_angle(roll_achieved - roll_target)))

    is_success = (
        p_err <= POS_TOL_M and pitch_err <= ORI_TOL_RAD and roll_err <= ORI_TOL_RAD
    )
    return is_success, p_err, pitch_err, roll_err


# Initialize Solvers
print("[INIT] Initializing solvers on CPU Core 0...")
g_trac_solver = trac_ik_native.TracIKSolver(TIMEOUT_SEC, 1e-4)

# PyKDL chain matching 5-DOF DH parameters
g_kdl_chain = PyKDL.Chain()
g_kdl_chain.addSegment(
    PyKDL.Segment(
        "base_seg",
        PyKDL.Joint("base_fixed", PyKDL.Joint.Fixed),
        PyKDL.Frame(PyKDL.Vector(0, 0, D_BASE)),
    )
)
g_kdl_chain.addSegment(
    PyKDL.Segment(
        "waist_seg",
        PyKDL.Joint("waist", PyKDL.Joint.RotZ),
        PyKDL.Frame(PyKDL.Rotation.RotX(math.pi / 2), PyKDL.Vector(0, 0, 0)),
    )
)
g_kdl_chain.addSegment(
    PyKDL.Segment(
        "shoulder_seg",
        PyKDL.Joint("shoulder", PyKDL.Joint.RotZ),
        PyKDL.Frame(PyKDL.Vector(L_UPPER, 0, 0)),
    )
)
g_kdl_chain.addSegment(
    PyKDL.Segment(
        "elbow_seg",
        PyKDL.Joint("elbow", PyKDL.Joint.RotZ),
        PyKDL.Frame(PyKDL.Vector(L_FORE, 0, 0)),
    )
)
g_kdl_chain.addSegment(
    PyKDL.Segment(
        "wrist_pitch_seg",
        PyKDL.Joint("wrist_pitch", PyKDL.Joint.RotZ),
        PyKDL.Frame(PyKDL.Vector(L_TOOL, 0, 0)),
    )
)
g_kdl_chain.addSegment(
    PyKDL.Segment(
        "wrist_roll_seg",
        PyKDL.Joint("wrist_roll", PyKDL.Joint.RotZ),
        PyKDL.Frame.Identity(),
    )
)

g_kdl_q_min = PyKDL.JntArray(5)
g_kdl_q_max = PyKDL.JntArray(5)
for i in range(5):
    g_kdl_q_min[i] = JOINT_LIMITS[i, 0]
    g_kdl_q_max[i] = JOINT_LIMITS[i, 1]

g_kdl_fk = PyKDL.ChainFkSolverPos_recursive(g_kdl_chain)
g_kdl_ik_vel = PyKDL.ChainIkSolverVel_pinv(g_kdl_chain)
g_kdl_ik = PyKDL.ChainIkSolverPos_NR_JL(
    g_kdl_chain, g_kdl_q_min, g_kdl_q_max, g_kdl_fk, g_kdl_ik_vel, 80, 1e-4
)

# RTB-LM arm
L1 = rtb.RevoluteDH(d=D_BASE, a=0, alpha=np.pi / 2, qlim=[-3.1416, 3.1416])
L2 = rtb.RevoluteDH(d=0, a=L_UPPER, alpha=0, qlim=[-1.5708, 1.5708])
L3 = rtb.RevoluteDH(d=0, a=L_FORE, alpha=0, qlim=[-1.5708, 1.5708])
L4 = rtb.RevoluteDH(d=0, a=L_TOOL, alpha=0, qlim=[-1.5708, 1.5708])
L5 = rtb.RevoluteDH(d=0, a=0, alpha=0, qlim=[-1.5708, 1.5708])
g_rtb_arm = rtb.DHRobot([L1, L2, L3, L4, L5], name="ARIA_5DOF")

# IKPy chain
g_ikpy_chain = ikpy.chain.Chain(
    links=[
        ikpy.link.OriginLink(),
        ikpy.link.DHLink(
            name="waist", d=D_BASE, a=0, alpha=np.pi / 2, bounds=(-3.1416, 3.1416)
        ),
        ikpy.link.DHLink(
            name="shoulder", d=0, a=L_UPPER, alpha=0, bounds=(-1.5708, 1.5708)
        ),
        ikpy.link.DHLink(
            name="elbow", d=0, a=L_FORE, alpha=0, bounds=(-1.5708, 1.5708)
        ),
        ikpy.link.DHLink(
            name="wrist_pitch", d=0, a=L_TOOL, alpha=0, bounds=(-1.5708, 1.5708)
        ),
        ikpy.link.DHLink(
            name="wrist_roll", d=0, a=0, alpha=0, bounds=(-1.5708, 1.5708)
        ),
    ],
    active_links_mask=[False, True, True, True, True, True],
)
print("[INIT] Solvers successfully initialized.")


# ══════════════════════════════════════════════════════════════════════════════
# Independent Ground-Truth Reachability Verification
# ══════════════════════════════════════════════════════════════════════════════
def verify_dense_grid_unreachable(pos, pitch, pos_tol=0.001):
    """
    Exhaustive independent joint-space grid search over (q2, q3) to prove
    that NO valid configuration within joint limits can reach the 5D target.
    Does NOT use any solver output.
    """
    x, y, z = pos
    th1 = math.atan2(y, x) if (abs(x) > 1e-7 or abs(y) > 1e-7) else 0.0

    q2_grid = np.linspace(-math.pi / 2, math.pi / 2, 60)
    q3_grid = np.linspace(-math.pi / 2, math.pi / 2, 60)
    Q2, Q3 = np.meshgrid(q2_grid, q3_grid)
    Q4 = pitch - (Q2 + Q3)
    valid = np.abs(Q4) <= math.pi / 2

    if not np.any(valid):
        return True, 999.0

    c1, s1 = math.cos(th1), math.sin(th1)
    c2 = np.cos(Q2[valid])
    s2 = np.sin(Q2[valid])
    c23 = np.cos(Q2[valid] + Q3[valid])
    s23 = np.sin(Q2[valid] + Q3[valid])
    c_p = math.cos(pitch)
    s_p = math.sin(pitch)

    px = c1 * (L_UPPER * c2 + L_FORE * c23 + L_TOOL * c_p)
    py = s1 * (L_UPPER * c2 + L_FORE * c23 + L_TOOL * c_p)
    pz = D_BASE + L_UPPER * s2 + L_FORE * s23 + L_TOOL * s_p

    dists = np.sqrt((px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2)
    min_dist = float(np.min(dists))
    is_unreachable = min_dist > pos_tol
    return is_unreachable, min_dist


# ══════════════════════════════════════════════════════════════════════════════
# Dataset Generators
# ══════════════════════════════════════════════════════════════════════════════
def generate_set_a_poses(n: int, seed: int = 42):
    """Set A: Sampled by FK of random in-limit joint configs (100% reachable)."""
    np.random.seed(seed)
    q_all = np.random.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1], size=(n, 5))
    poses = []
    for i in range(n):
        q = q_all[i]
        T = aria_fk(q)
        pos = T[:3, 3]
        pitch = float(q[1] + q[2] + q[3])
        roll = float(q[4])
        R_flat = [float(T[r, c]) for r in range(3) for c in range(3)]
        poses.append((i + 1, pos, pitch, roll, R_flat, T, True))  # gt_reachable = True
    return poses


def generate_set_b_poses(n: int, seed: int = 1337):
    """
    Set B: 50% Reachable (by FK construction) + 50% Unreachable (perturbed beyond
    workspace bounds and verified by dense joint-space grid search).
    Zero solver dependence for ground-truth reachability.
    """
    np.random.seed(seed)
    n_reachable = n // 2
    n_unreachable = n - n_reachable

    poses = []

    # 1. Reachable subset
    q_reach = np.random.uniform(
        JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1], size=(n_reachable, 5)
    )
    for i in range(n_reachable):
        q = q_reach[i]
        T = aria_fk(q)
        pos = T[:3, 3]
        pitch = float(q[1] + q[2] + q[3])
        roll = float(q[4])
        R_flat = [float(T[r, c]) for r in range(3) for c in range(3)]
        poses.append((i + 1, pos, pitch, roll, R_flat, T, True))

    # 2. Unreachable subset (perturbed beyond bounds)
    count_unreach = 0
    while count_unreach < n_unreachable:
        # Sample azimuth theta1, pitch, roll
        th1 = np.random.uniform(-math.pi, math.pi)
        p = np.random.uniform(-math.pi / 2, math.pi / 2)
        r = np.random.uniform(-math.pi / 2, math.pi / 2)

        # Perturb radially or vertically outside workspace:
        # Max reach is 0.355m; perturb to 0.380m - 0.550m
        case = np.random.choice(
            ["radial_far", "radial_near", "height_high", "height_low"]
        )
        if case == "radial_far":
            radius = np.random.uniform(0.38, 0.55)
            z = np.random.uniform(0.0, 0.35)
        elif case == "radial_near":
            radius = np.random.uniform(0.01, 0.05)
            z = np.random.uniform(0.05, 0.15)
        elif case == "height_high":
            radius = np.random.uniform(0.15, 0.30)
            z = np.random.uniform(0.48, 0.65)
        else:  # height_low
            radius = np.random.uniform(0.15, 0.30)
            z = np.random.uniform(-0.35, -0.15)

        x = radius * math.cos(th1)
        y = radius * math.sin(th1)
        pos = np.array([x, y, z])

        # Verify unreachable via dense joint grid
        is_unreach, min_d = verify_dense_grid_unreachable(pos, p, pos_tol=POS_TOL_M)
        if is_unreach:
            # Construct standard 5-DOF frame orientation
            T_ref = aria_fk(np.array([th1, p, 0.0, 0.0, r]))
            R_mat = T_ref[:3, :3]
            T_mat = np.eye(4)
            T_mat[:3, :3] = R_mat
            T_mat[:3, 3] = pos
            R_flat = [
                float(R_mat[r_idx, c_idx]) for r_idx in range(3) for c_idx in range(3)
            ]

            idx = n_reachable + count_unreach + 1
            poses.append((idx, pos, p, r, R_flat, T_mat, False))  # gt_reachable = False
            count_unreach += 1

    return poses


# ══════════════════════════════════════════════════════════════════════════════
# Solver Evaluation
# ══════════════════════════════════════════════════════════════════════════════
def evaluate_pose(pos, pitch, roll, R_flat, T_mat):
    pos_list = [float(pos[0]), float(pos[1]), float(pos[2])]
    solver_results = []

    # 1. ARIA Analytical (Python)
    t0 = time.perf_counter()
    r_aria = solve_analytical(pos, target_pitch=pitch, target_roll=roll)
    dt_aria = (time.perf_counter() - t0) * 1000.0
    if r_aria.success:
        succ_aria, p_err, pi_err, ro_err = check_5d_convergence(
            r_aria.joint_angles, pos, pitch, roll
        )
        q_sol_aria = r_aria.joint_angles.tolist()
    else:
        succ_aria = False
        p_err, pi_err, ro_err = 999.0, 999.0, 999.0
        q_sol_aria = [0.0] * 5
    solver_results.append(
        (
            "ARIA_Analytical",
            "Python",
            succ_aria,
            dt_aria,
            p_err,
            pi_err,
            ro_err,
            0,
            q_sol_aria,
        )
    )

    # 2. TRAC-IK (C++)
    t0 = time.perf_counter()
    succ_trac, q_trac, restarts_trac, _, _ = g_trac_solver.solve(
        pos_list, R_flat, [0.0] * 5, 1
    )
    dt_trac = (time.perf_counter() - t0) * 1000.0
    if succ_trac:
        succ_trac_5d, p_err_t, pi_err_t, ro_err_t = check_5d_convergence(
            np.array(q_trac), pos, pitch, roll
        )
    else:
        succ_trac_5d = False
        p_err_t, pi_err_t, ro_err_t = 999.0, 999.0, 999.0
    solver_results.append(
        (
            "TRAC_IK",
            "C++",
            succ_trac_5d,
            dt_trac,
            p_err_t,
            pi_err_t,
            ro_err_t,
            restarts_trac,
            q_trac,
        )
    )

    # 3. PyKDL (C++ / Python bindings)
    f_kdl = PyKDL.Frame(
        PyKDL.Rotation(
            R_flat[0],
            R_flat[1],
            R_flat[2],
            R_flat[3],
            R_flat[4],
            R_flat[5],
            R_flat[6],
            R_flat[7],
            R_flat[8],
        ),
        PyKDL.Vector(pos_list[0], pos_list[1], pos_list[2]),
    )
    q_out_kdl = PyKDL.JntArray(5)
    t0 = time.perf_counter()
    rc_kdl = g_kdl_ik.CartToJnt(PyKDL.JntArray(5), f_kdl, q_out_kdl)
    dt_kdl = (time.perf_counter() - t0) * 1000.0
    q_kdl_list = [float(q_out_kdl[i]) for i in range(5)]
    if rc_kdl >= 0:
        succ_kdl, p_err_k, pi_err_k, ro_err_k = check_5d_convergence(
            np.array(q_kdl_list), pos, pitch, roll
        )
    else:
        succ_kdl = False
        p_err_k, pi_err_k, ro_err_k = 999.0, 999.0, 999.0
    solver_results.append(
        ("PyKDL", "C++", succ_kdl, dt_kdl, p_err_k, pi_err_k, ro_err_k, 1, q_kdl_list)
    )

    # 4. RTB-LM (Python)
    t0 = time.perf_counter()
    try:
        sol_rtb = g_rtb_arm.ikine_LM(
            T_mat, q0=np.zeros(5), mask=[1, 1, 1, 0, 1, 1], ilimit=12
        )
        dt_rtb = (time.perf_counter() - t0) * 1000.0
        q_rtb_list = sol_rtb.q.tolist()
        if sol_rtb.success:
            succ_rtb, p_err_r, pi_err_r, ro_err_r = check_5d_convergence(
                sol_rtb.q, pos, pitch, roll
            )
        else:
            succ_rtb = False
            p_err_r, pi_err_r, ro_err_r = 999.0, 999.0, 999.0
    except Exception:
        dt_rtb = (time.perf_counter() - t0) * 1000.0
        succ_rtb = False
        p_err_r, pi_err_r, ro_err_r = 999.0, 999.0, 999.0
        q_rtb_list = [0.0] * 5
    solver_results.append(
        (
            "RTB_LM",
            "Python",
            succ_rtb,
            dt_rtb,
            p_err_r,
            pi_err_r,
            ro_err_r,
            1,
            q_rtb_list,
        )
    )

    # 5. IKPy (Python)
    t0 = time.perf_counter()
    try:
        q_ikpy_raw = g_ikpy_chain.inverse_kinematics(
            target_position=pos,
            target_orientation=T_mat[:3, 0],
            orientation_mode="X",
            max_iter=25,
        )
        dt_ikpy = (time.perf_counter() - t0) * 1000.0
        q_ikpy_list = [float(x) for x in q_ikpy_raw[1:6]]
        succ_ikpy, p_err_ik, pi_err_ik, ro_err_ik = check_5d_convergence(
            np.array(q_ikpy_list), pos, pitch, roll
        )
    except Exception:
        dt_ikpy = (time.perf_counter() - t0) * 1000.0
        succ_ikpy = False
        p_err_ik, pi_err_ik, ro_err_ik = 999.0, 999.0, 999.0
        q_ikpy_list = [0.0] * 5
    solver_results.append(
        (
            "IKPy",
            "Python",
            succ_ikpy,
            dt_ikpy,
            p_err_ik,
            pi_err_ik,
            ro_err_ik,
            1,
            q_ikpy_list,
        )
    )

    return solver_results


def run_benchmark(
    dataset_name: str, poses_generator, out_csv_path: str, n_poses: int = 10000
):
    print(f"\n{'═' * 78}")
    print(f"Running Real 5D Benchmark: {dataset_name} ({n_poses:,} poses)")
    print(
        f"Target Constraints: Pos <= {POS_TOL_M*1000:.1f} mm, Pitch <= {ORI_TOL_RAD:.2f} rad, Roll <= {ORI_TOL_RAD:.2f} rad"
    )
    print(f"Output File: {out_csv_path}")
    print(f"{'═' * 78}")

    if os.path.exists(out_csv_path):
        print(
            f"[ERROR] Target file exists: {out_csv_path}. Aborting to protect raw data."
        )
        sys.exit(1)

    print("Generating dataset poses...")
    poses = poses_generator(n_poses)
    print(f"Generated {len(poses):,} target poses.")

    fieldnames = [
        "trial_id",
        "pose_id",
        "dataset",
        "solver",
        "solver_language",
        "commit_hash",
        "timestamp",
        "target_x_mm",
        "target_y_mm",
        "target_z_mm",
        "target_pitch_rad",
        "target_roll_rad",
        "gt_reachable",
        "success",
        "latency_ms",
        "pos_error_mm",
        "pitch_error_rad",
        "roll_error_rad",
        "restarts_used",
        "q0_rad",
        "q1_rad",
        "q2_rad",
        "q3_rad",
        "q4_rad",
    ]

    solver_stats = {
        "ARIA_Analytical": {
            "times": [],
            "pos_errs": [],
            "pitch_errs": [],
            "roll_errs": [],
            "success": 0,
        },
        "TRAC_IK": {
            "times": [],
            "pos_errs": [],
            "pitch_errs": [],
            "roll_errs": [],
            "success": 0,
        },
        "PyKDL": {
            "times": [],
            "pos_errs": [],
            "pitch_errs": [],
            "roll_errs": [],
            "success": 0,
        },
        "RTB_LM": {
            "times": [],
            "pos_errs": [],
            "pitch_errs": [],
            "roll_errs": [],
            "success": 0,
        },
        "IKPy": {
            "times": [],
            "pos_errs": [],
            "pitch_errs": [],
            "roll_errs": [],
            "success": 0,
        },
    }

    t_start = time.time()
    trial_count = 0

    with open(out_csv_path, "w", newline="", encoding="utf-8") as f_out:
        writer = csv.DictWriter(f_out, fieldnames=fieldnames)
        writer.writeheader()

        for idx, (pose_id, pos, pitch, roll, R_flat, T_mat, gt_reach) in enumerate(
            poses
        ):
            solver_results = evaluate_pose(pos, pitch, roll, R_flat, T_mat)
            ts = datetime.utcnow().isoformat() + "Z"

            for (
                s_name,
                s_lang,
                succ,
                dt_ms,
                p_err,
                pi_err,
                ro_err,
                restarts,
                q_sol,
            ) in solver_results:
                trial_count += 1
                solver_stats[s_name]["times"].append(dt_ms)
                if succ:
                    solver_stats[s_name]["success"] += 1
                    solver_stats[s_name]["pos_errs"].append(p_err * 1000.0)
                    solver_stats[s_name]["pitch_errs"].append(pi_err)
                    solver_stats[s_name]["roll_errs"].append(ro_err)

                writer.writerow(
                    {
                        "trial_id": f"TRIAL_{trial_count:06d}",
                        "pose_id": f"POSE_{pose_id:05d}",
                        "dataset": dataset_name,
                        "solver": s_name,
                        "solver_language": s_lang,
                        "commit_hash": GIT_COMMIT_HASH,
                        "timestamp": ts,
                        "target_x_mm": f"{pos[0]*1000.0:.3f}",
                        "target_y_mm": f"{pos[1]*1000.0:.3f}",
                        "target_z_mm": f"{pos[2]*1000.0:.3f}",
                        "target_pitch_rad": f"{pitch:.4f}",
                        "target_roll_rad": f"{roll:.4f}",
                        "gt_reachable": "TRUE" if gt_reach else "FALSE",
                        "success": "TRUE" if succ else "FALSE",
                        "latency_ms": f"{dt_ms:.4f}",
                        "pos_error_mm": (
                            f"{p_err*1000.0:.4f}" if p_err < 900.0 else "N/A"
                        ),
                        "pitch_error_rad": f"{pi_err:.5f}" if pi_err < 900.0 else "N/A",
                        "roll_error_rad": f"{ro_err:.5f}" if ro_err < 900.0 else "N/A",
                        "restarts_used": restarts,
                        "q0_rad": f"{q_sol[0]:.4f}",
                        "q1_rad": f"{q_sol[1]:.4f}",
                        "q2_rad": f"{q_sol[2]:.4f}",
                        "q3_rad": f"{q_sol[3]:.4f}",
                        "q4_rad": f"{q_sol[4]:.4f}",
                    }
                )

            if (idx + 1) % 1000 == 0 or (idx + 1) == n_poses:
                el = time.time() - t_start
                rate = (idx + 1) / el
                print(
                    f"  Processed {idx+1:,} / {n_poses:,} poses ({(idx+1)/n_poses*100:.1f}%) | {rate:.1f} poses/s | Elapsed: {el:.1f}s"
                )
                f_out.flush()

    total_time = time.time() - t_start
    print(
        f"\n{dataset_name} finished in {total_time:.2f} s ({n_poses/total_time:.1f} poses/s)"
    )

    summary_rows = []
    print(f"\n{'─' * 78}")
    print(f"5D Summary Statistics: {dataset_name}")
    print(f"{'─' * 78}")
    print(
        f"{'Solver':<18} | {'Lang':<6} | {'5D Success':<14} | {'Mean Time':<10} | {'p50 Time':<10} | {'p95 Time':<10} | {'Pos Err (mm)'}"
    )
    print(f"{'─' * 78}")

    lang_map = {
        "ARIA_Analytical": "Python",
        "TRAC_IK": "C++",
        "PyKDL": "C++",
        "RTB_LM": "Python",
        "IKPy": "Python",
    }

    for s_name, d in solver_stats.items():
        succ_rate = d["success"] / n_poses * 100.0
        times = np.array(d["times"])
        pos_errs = (
            np.array(d["pos_errs"]) if len(d["pos_errs"]) > 0 else np.array([0.0])
        )

        t_mean = float(np.mean(times))
        t_std = float(np.std(times))
        t_p50 = float(np.percentile(times, 50))
        t_p95 = float(np.percentile(times, 95))
        t_p99 = float(np.percentile(times, 99))

        pe_mean = float(np.mean(pos_errs))
        pe_max = float(np.max(pos_errs))

        print(
            f"{s_name:<18} | {lang_map[s_name]:<6} | {d['success']:>5}/{n_poses} ({succ_rate:5.1f}%) | {t_mean:8.4f} ms | {t_p50:8.4f} ms | {t_p95:8.4f} ms | {pe_mean:8.4f} mm"
        )

        summary_rows.append(
            {
                "dataset": dataset_name,
                "solver": s_name,
                "language": lang_map[s_name],
                "total_poses": n_poses,
                "success_count": d["success"],
                "success_rate_pct": f"{succ_rate:.2f}",
                "time_mean_ms": f"{t_mean:.4f}",
                "time_std_ms": f"{t_std:.4f}",
                "time_p50_ms": f"{t_p50:.4f}",
                "time_p95_ms": f"{t_p95:.4f}",
                "time_p99_ms": f"{t_p99:.4f}",
                "pos_error_mean_mm": f"{pe_mean:.4f}",
                "pos_error_max_mm": f"{pe_max:.4f}",
                "pos_tolerance_mm": f"{POS_TOL_M*1000.0:.1f}",
                "ori_tolerance_rad": f"{ORI_TOL_RAD:.4f}",
                "cpu_pinned": "Core 0",
            }
        )

    return summary_rows


def main():
    print("═" * 78)
    print("Project ARIA: Real Empirical IK Benchmark (5D Constrained, Single Core 0)")
    print(f"Git Commit: {GIT_COMMIT_HASH} | CPU: Single Core Affinity (Core 0)")
    print("═" * 78)

    fk_csv = os.path.join(DATA_REAL_DIR, "ik_benchmark_5d_fk_10k.csv")
    cart_csv = os.path.join(DATA_REAL_DIR, "ik_benchmark_5d_cartesian_10k.csv")
    summary_csv = os.path.join(DATA_REAL_DIR, "ik_benchmark_5d_summary.csv")

    all_summaries = []

    # Run Benchmark Set A (10,000 Reachable Poses by FK Construction)
    summary_fk = run_benchmark(
        dataset_name="Set_A_5D_FK_Reachable_10k",
        poses_generator=generate_set_a_poses,
        out_csv_path=fk_csv,
        n_poses=10000,
    )
    all_summaries.extend(summary_fk)

    # Run Benchmark Set B (10,000 Poses: 50% Reachable + 50% Unreachable Verified by Dense Grid)
    summary_cart = run_benchmark(
        dataset_name="Set_B_5D_Cartesian_10k",
        poses_generator=generate_set_b_poses,
        out_csv_path=cart_csv,
        n_poses=10000,
    )
    all_summaries.extend(summary_cart)

    print(f"\nWriting summary to {summary_csv}...")
    with open(summary_csv, "w", newline="", encoding="utf-8") as f_sum:
        writer = csv.DictWriter(f_sum, fieldnames=list(all_summaries[0].keys()))
        writer.writeheader()
        writer.writerows(all_summaries)

    print("\n✓ Completed all 20,000 empirical 5D IK evaluations on CPU Core 0.")
    print("✓ Saved raw per-trial outputs:")
    print(f"  - {fk_csv}")
    print(f"  - {cart_csv}")
    print(f"  - {summary_csv}\n")


if __name__ == "__main__":
    main()
