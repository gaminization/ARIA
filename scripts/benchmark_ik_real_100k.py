#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
Project ARIA: Real Empirical IK Benchmark (100,000 Poses)
Strictly Empirical: Calls REAL libraries (ARIA Analytical, TRAC-IK C++, PyKDL,
Robotics Toolbox LM, IKPy). Zero synthetic RNG or scaling multipliers.

Evaluates two 100k sets:
  (a) FK from joint configs uniformly within joint limits (100% reachable)
  (b) Uniform random Cartesian points + pitch + roll (many unreachable)
      Ground-truth reachability for (b) defined by TRAC-IK with 50 restarts.

Outputs saved to data/real/:
  - data/real/ik_benchmark_fk_100k.csv
  - data/real/ik_benchmark_cartesian_100k.csv
  - data/real/ik_benchmark_summary.csv
═══════════════════════════════════════════════════════════════════════════════
"""

import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"

import sys
import time
import math
import csv
import ctypes
import numpy as np
import multiprocessing as mp
from datetime import datetime

# Workspace paths
WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
DATA_REAL_DIR = os.path.join(WORKSPACE_ROOT, "data", "real")
SOLVERS_DIR = os.path.join(WORKSPACE_ROOT, "arm_ik", "arm_ik", "ik_solvers")
sys.path.insert(0, SOLVERS_DIR)

# Ensure local C++ libraries are loaded
ctypes.CDLL(os.path.expanduser("~/.local/lib/libnlopt.so"), mode=ctypes.RTLD_GLOBAL)
ctypes.CDLL(os.path.expanduser("~/.local/lib/libtrac_ik.so"), mode=ctypes.RTLD_GLOBAL)

from aria_analytical_ik import solve_analytical, forward_kinematics as aria_fk, JOINT_LIMITS

GIT_COMMIT_HASH = "8d83b97"
SUCCESS_TOL_MM = 1.5  # Position error threshold for convergence
TIMEOUT_SEC = 0.005   # 5 ms timeout per solver


def init_worker():
    """Initializes real solver instances once per worker process."""
    global g_aria_solve, g_aria_fk, g_trac_solver, g_trac_solver_gt
    global g_kdl_chain, g_kdl_q_min, g_kdl_q_max, g_kdl_fk, g_kdl_ik_vel, g_kdl_ik
    global g_rtb_arm, g_ikpy_chain, g_JL, PyKDL

    import PyKDL
    import roboticstoolbox as rtb
    import ikpy.chain, ikpy.link
    import trac_ik_native

    g_aria_solve = solve_analytical
    g_aria_fk = aria_fk
    g_JL = JOINT_LIMITS

    # TRAC-IK standard (5 ms timeout, eps=1e-4)
    g_trac_solver = trac_ik_native.TracIKSolver(TIMEOUT_SEC, 1e-4)

    # TRAC-IK ground-truth reachability solver for set (b) (50 ms timeout, eps=1e-4)
    g_trac_solver_gt = trac_ik_native.TracIKSolver(0.050, 1e-4)

    # PyKDL Setup (CRITICAL: retain ALL references in global scope to avoid dangling C++ references)
    g_kdl_chain = PyKDL.Chain()
    g_kdl_chain.addSegment(PyKDL.Segment("base_seg", PyKDL.Joint("base_fixed", PyKDL.Joint.Fixed), PyKDL.Frame(PyKDL.Vector(0, 0, 0.105))))
    g_kdl_chain.addSegment(PyKDL.Segment("waist_seg", PyKDL.Joint("waist", PyKDL.Joint.RotZ), PyKDL.Frame(PyKDL.Rotation.RotX(math.pi/2), PyKDL.Vector(0, 0, 0))))
    g_kdl_chain.addSegment(PyKDL.Segment("shoulder_seg", PyKDL.Joint("shoulder", PyKDL.Joint.RotZ), PyKDL.Frame(PyKDL.Vector(0.145, 0, 0))))
    g_kdl_chain.addSegment(PyKDL.Segment("elbow_seg", PyKDL.Joint("elbow", PyKDL.Joint.RotZ), PyKDL.Frame(PyKDL.Vector(0.115, 0, 0))))
    g_kdl_chain.addSegment(PyKDL.Segment("wrist_pitch_seg", PyKDL.Joint("wrist_pitch", PyKDL.Joint.RotZ), PyKDL.Frame(PyKDL.Vector(0.095, 0, 0))))
    g_kdl_chain.addSegment(PyKDL.Segment("wrist_roll_seg", PyKDL.Joint("wrist_roll", PyKDL.Joint.RotZ), PyKDL.Frame.Identity()))

    g_kdl_q_min = PyKDL.JntArray(5)
    g_kdl_q_max = PyKDL.JntArray(5)
    for i in range(5):
        g_kdl_q_min[i] = g_JL[i, 0]
        g_kdl_q_max[i] = g_JL[i, 1]

    g_kdl_fk = PyKDL.ChainFkSolverPos_recursive(g_kdl_chain)
    g_kdl_ik_vel = PyKDL.ChainIkSolverVel_pinv(g_kdl_chain)
    g_kdl_ik = PyKDL.ChainIkSolverPos_NR_JL(g_kdl_chain, g_kdl_q_min, g_kdl_q_max, g_kdl_fk, g_kdl_ik_vel, 80, 1e-4)

    # RTB Setup
    L1 = rtb.RevoluteDH(d=0.105, a=0, alpha=np.pi/2, qlim=[-3.1416, 3.1416])
    L2 = rtb.RevoluteDH(d=0, a=0.145, alpha=0, qlim=[-1.5708, 1.5708])
    L3 = rtb.RevoluteDH(d=0, a=0.115, alpha=0, qlim=[-1.5708, 1.5708])
    L4 = rtb.RevoluteDH(d=0, a=0.095, alpha=0, qlim=[-1.5708, 1.5708])
    L5 = rtb.RevoluteDH(d=0, a=0, alpha=0, qlim=[-1.5708, 1.5708])
    g_rtb_arm = rtb.DHRobot([L1, L2, L3, L4, L5], name="ARIA_5DOF")

    # IKPy Setup
    g_ikpy_chain = ikpy.chain.Chain(links=[
        ikpy.link.OriginLink(),
        ikpy.link.DHLink(name="waist", d=0.105, a=0, alpha=np.pi/2, bounds=(-3.1416, 3.1416)),
        ikpy.link.DHLink(name="shoulder", d=0, a=0.145, alpha=0, bounds=(-1.5708, 1.5708)),
        ikpy.link.DHLink(name="elbow", d=0, a=0.115, alpha=0, bounds=(-1.5708, 1.5708)),
        ikpy.link.DHLink(name="wrist_pitch", d=0, a=0.095, alpha=0, bounds=(-1.5708, 1.5708)),
        ikpy.link.DHLink(name="wrist_roll", d=0, a=0, alpha=0, bounds=(-1.5708, 1.5708)),
    ], active_links_mask=[False, True, True, True, True, True])


def evaluate_single_pose(item):
    """Evaluates all 5 real solvers on a single target pose."""
    pose_id, pos, pitch, roll, R_flat, T_mat, is_set_b = item
    results = []

    pos_list = [float(pos[0]), float(pos[1]), float(pos[2])]

    # 1. ARIA Analytical (Signed-radius, two-azimuth closed-form)
    t0 = time.perf_counter()
    r_aria = g_aria_solve(pos, target_pitch=pitch, target_roll=roll)
    dt_aria = (time.perf_counter() - t0) * 1000.0
    if r_aria.success:
        err_aria = float(np.linalg.norm(g_aria_fk(r_aria.joint_angles)[:3, 3] - pos) * 1000.0)
        succ_aria = (err_aria <= SUCCESS_TOL_MM)
    else:
        err_aria = 999.0
        succ_aria = False
    results.append(("ARIA_Analytical", succ_aria, dt_aria, err_aria, 0, r_aria.joint_angles.tolist() if r_aria.success else [0]*5))

    # 2. TRAC-IK (Official C++ TRACLabs dual-solver)
    t0 = time.perf_counter()
    succ_trac, q_trac, restarts_trac, dt_trac, rc_trac = g_trac_solver.solve(pos_list, R_flat, [0.0]*5, 1)
    # Use accurate wall-clock perf_counter
    dt_trac_wall = (time.perf_counter() - t0) * 1000.0
    if succ_trac:
        err_trac = float(np.linalg.norm(g_aria_fk(np.array(q_trac))[:3, 3] - pos) * 1000.0)
        succ_trac = (err_trac <= SUCCESS_TOL_MM)
    else:
        err_trac = 999.0
    results.append(("TRAC_IK", succ_trac, dt_trac_wall, err_trac, restarts_trac, q_trac))

    # 3. PyKDL (Official Orocos KDL Newton-Raphson with joint limits)
    import PyKDL
    f_kdl = PyKDL.Frame(
        PyKDL.Rotation(R_flat[0], R_flat[1], R_flat[2], R_flat[3], R_flat[4], R_flat[5], R_flat[6], R_flat[7], R_flat[8]),
        PyKDL.Vector(pos_list[0], pos_list[1], pos_list[2])
    )
    q_out_kdl = PyKDL.JntArray(5)
    t0 = time.perf_counter()
    rc_kdl = g_kdl_ik.CartToJnt(PyKDL.JntArray(5), f_kdl, q_out_kdl)
    dt_kdl = (time.perf_counter() - t0) * 1000.0
    q_kdl_list = [q_out_kdl[i] for i in range(5)]
    if rc_kdl >= 0:
        err_kdl = float(np.linalg.norm(g_aria_fk(np.array(q_kdl_list))[:3, 3] - pos) * 1000.0)
        succ_kdl = (err_kdl <= SUCCESS_TOL_MM)
    else:
        err_kdl = 999.0
        succ_kdl = False
    results.append(("PyKDL", succ_kdl, dt_kdl, err_kdl, 1, q_kdl_list))

    # 4. RTB-LM (Robotics Toolbox Levenberg-Marquardt)
    t0 = time.perf_counter()
    try:
        sol_rtb = g_rtb_arm.ikine_LM(T_mat, q0=np.zeros(5), mask=[1, 1, 1, 0, 1, 1], ilimit=12)
        dt_rtb = (time.perf_counter() - t0) * 1000.0
        q_rtb_list = sol_rtb.q.tolist()
        if sol_rtb.success:
            err_rtb = float(np.linalg.norm(g_aria_fk(sol_rtb.q)[:3, 3] - pos) * 1000.0)
            succ_rtb = (err_rtb <= SUCCESS_TOL_MM)
        else:
            err_rtb = 999.0
            succ_rtb = False
    except Exception:
        dt_rtb = (time.perf_counter() - t0) * 1000.0
        succ_rtb = False
        err_rtb = 999.0
        q_rtb_list = [0]*5
    results.append(("RTB_LM", succ_rtb, dt_rtb, err_rtb, 1, q_rtb_list))

    # 5. IKPy (Official IKPy inverse_kinematics)
    t0 = time.perf_counter()
    try:
        q_ikpy_raw = g_ikpy_chain.inverse_kinematics(pos, max_iter=25)
        dt_ikpy = (time.perf_counter() - t0) * 1000.0
        q_ikpy_list = list(q_ikpy_raw[1:6])
        err_ikpy = float(np.linalg.norm(g_aria_fk(np.array(q_ikpy_list))[:3, 3] - pos) * 1000.0)
        succ_ikpy = (err_ikpy <= SUCCESS_TOL_MM)
    except Exception:
        dt_ikpy = (time.perf_counter() - t0) * 1000.0
        succ_ikpy = False
        err_ikpy = 999.0
        q_ikpy_list = [0]*5
    results.append(("IKPy", succ_ikpy, dt_ikpy, err_ikpy, 1, q_ikpy_list))

    # Optional: For set (b), evaluate ground-truth reachability using TRAC-IK with 50 restarts
    gt_reachable = True
    if is_set_b:
        succ_gt, _, _, _, _ = g_trac_solver_gt.solve(pos_list, R_flat, [0.0]*5, 50)
        gt_reachable = succ_gt

    return pose_id, pos_list, pitch, roll, results, gt_reachable


def run_benchmark_dataset(dataset_name: str,
                         poses_generator,
                         out_csv_path: str,
                         n_poses: int = 100000,
                         n_workers: int = 12):
    print(f"\n{'═' * 70}")
    print(f"Running Real IK Benchmark: {dataset_name} ({n_poses:,} poses)")
    print(f"Output File: {out_csv_path}")
    print(f"Workers: {n_workers} CPU cores")
    print(f"{'═' * 70}")

    os.makedirs(os.path.dirname(out_csv_path), exist_ok=True)
    is_set_b = ("Cartesian" in dataset_name)

    print("Generating poses...")
    poses = poses_generator(n_poses)
    print(f"Generated {len(poses):,} target poses.")

    fieldnames = [
        "pose_id", "solver", "dataset_type", "git_commit_hash",
        "target_x_mm", "target_y_mm", "target_z_mm",
        "target_pitch_rad", "target_roll_rad",
        "success", "time_ms", "residual_mm", "restarts_used",
        "gt_reachable", "q0", "q1", "q2", "q3", "q4"
    ]

    solver_stats = {
        "ARIA_Analytical": {"times": [], "errors": [], "success": 0},
        "TRAC_IK": {"times": [], "errors": [], "success": 0},
        "PyKDL": {"times": [], "errors": [], "success": 0},
        "RTB_LM": {"times": [], "errors": [], "success": 0},
        "IKPy": {"times": [], "errors": [], "success": 0},
    }

    t_start = time.time()
    total_processed = 0
    chunk_size = 500

    with open(out_csv_path, "w", newline="", encoding="utf-8") as f_out:
        writer = csv.DictWriter(f_out, fieldnames=fieldnames)
        writer.writeheader()

        with mp.Pool(n_workers, initializer=init_worker) as pool:
            for c_idx in range(0, n_poses, chunk_size):
                chunk = poses[c_idx : c_idx + chunk_size]
                chunk_items = [
                    (p[0], p[1], p[2], p[3], p[4], p[5], is_set_b)
                    for p in chunk
                ]

                results = pool.map(evaluate_single_pose, chunk_items)

                for pose_id, pos_list, pitch, roll, solver_results, gt_reachable in results:
                    for solver_name, succ, dt_ms, err_mm, restarts, q_sol in solver_results:
                        solver_stats[solver_name]["times"].append(dt_ms)
                        if succ:
                            solver_stats[solver_name]["success"] += 1
                            solver_stats[solver_name]["errors"].append(err_mm)

                        writer.writerow({
                            "pose_id": f"POSE_{pose_id:06d}",
                            "solver": solver_name,
                            "dataset_type": dataset_name,
                            "git_commit_hash": GIT_COMMIT_HASH,
                            "target_x_mm": f"{pos_list[0]*1000.0:.3f}",
                            "target_y_mm": f"{pos_list[1]*1000.0:.3f}",
                            "target_z_mm": f"{pos_list[2]*1000.0:.3f}",
                            "target_pitch_rad": f"{pitch:.4f}",
                            "target_roll_rad": f"{roll:.4f}",
                            "success": "TRUE" if succ else "FALSE",
                            "time_ms": f"{dt_ms:.4f}",
                            "residual_mm": f"{err_mm:.4f}" if err_mm < 900.0 else "N/A",
                            "restarts_used": restarts,
                            "gt_reachable": "TRUE" if gt_reachable else "FALSE",
                            "q0": f"{q_sol[0]:.4f}",
                            "q1": f"{q_sol[1]:.4f}",
                            "q2": f"{q_sol[2]:.4f}",
                            "q3": f"{q_sol[3]:.4f}",
                            "q4": f"{q_sol[4]:.4f}",
                        })

                total_processed += len(chunk)
                elapsed = time.time() - t_start
                rate = total_processed / elapsed
                print(f"  Progress: {total_processed:,} / {n_poses:,} poses ({total_processed/n_poses*100:.1f}%) | Speed: {rate:.1f} poses/s | Elapsed: {elapsed:.1f}s")
                f_out.flush()

    total_time = time.time() - t_start
    print(f"\n{dataset_name} finished in {total_time:.2f} s ({n_poses/total_time:.1f} poses/s)")

    # Print and return metrics
    summary_rows = []
    print(f"\n{'─' * 70}")
    print(f"Summary Statistics: {dataset_name}")
    print(f"{'─' * 70}")
    print(f"{'Solver':<18} | {'Success Rate':<14} | {'Mean Time':<10} | {'p50 Time':<10} | {'p95 Time':<10} | {'Mean Error'}")
    print(f"{'─' * 70}")

    for solver_name, d in solver_stats.items():
        succ_rate = d["success"] / n_poses * 100.0
        times = np.array(d["times"])
        errs = np.array(d["errors"]) if len(d["errors"]) > 0 else np.array([0.0])

        t_mean = float(np.mean(times))
        t_std = float(np.std(times))
        t_p50 = float(np.percentile(times, 50))
        t_p95 = float(np.percentile(times, 95))
        t_p99 = float(np.percentile(times, 99))

        e_mean = float(np.mean(errs))
        e_std = float(np.std(errs))
        e_max = float(np.max(errs))

        print(f"{solver_name:<18} | {d['success']:>6}/{n_poses} ({succ_rate:5.1f}%) | {t_mean:8.4f} ms | {t_p50:8.4f} ms | {t_p95:8.4f} ms | {e_mean:8.4f} mm")

        summary_rows.append({
            "dataset": dataset_name,
            "solver": solver_name,
            "total_poses": n_poses,
            "success_count": d["success"],
            "success_rate_pct": f"{succ_rate:.2f}",
            "time_mean_ms": f"{t_mean:.4f}",
            "time_std_ms": f"{t_std:.4f}",
            "time_p50_ms": f"{t_p50:.4f}",
            "time_p95_ms": f"{t_p95:.4f}",
            "time_p99_ms": f"{t_p99:.4f}",
            "error_mean_mm": f"{e_mean:.4f}",
            "error_std_mm": f"{e_std:.4f}",
            "error_max_mm": f"{e_max:.4f}",
        })

    return summary_rows


def generate_fk_poses(n: int, seed: int = 42):
    """Generates 100k poses by sampling joint configs uniformly within limits."""
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
        poses.append((i + 1, pos, pitch, roll, R_flat, T))

    return poses


def generate_cartesian_poses(n: int, seed: int = 1337):
    """Generates 100k uniform random Cartesian points + pitch + roll."""
    np.random.seed(seed)
    x = np.random.uniform(-0.40, 0.40, size=n)
    y = np.random.uniform(-0.40, 0.40, size=n)
    z = np.random.uniform(-0.10, 0.50, size=n)
    pitch = np.random.uniform(-math.pi/2, math.pi/2, size=n)
    roll = np.random.uniform(-math.pi/2, math.pi/2, size=n)

    poses = []
    for i in range(n):
        pos = np.array([x[i], y[i], z[i]])
        p = pitch[i]
        r = roll[i]

        th1 = math.atan2(y[i], x[i]) if (abs(x[i]) > 1e-7 or abs(y[i]) > 1e-7) else 0.0

        # Standard 5-DOF frame orientation matching DH convention
        # The tool orientation is uniquely determined by azimuth th1, planar pitch p, and tool roll r
        T_ref = aria_fk(np.array([th1, p, 0.0, 0.0, r]))
        R_mat = T_ref[:3, :3]
        T_mat = np.eye(4)
        T_mat[:3, :3] = R_mat
        T_mat[:3, 3] = pos
        R_flat = [float(R_mat[r_idx, c_idx]) for r_idx in range(3) for c_idx in range(3)]

        poses.append((i + 1, pos, p, r, R_flat, T_mat))

    return poses


def main():
    print("═" * 70)
    print("Project ARIA: Real 100k IK Benchmark Execution")
    print("Hardware / Environment: Linux, Python 3.10, ROS 2 Humble")
    print("Active Solvers: ARIA Analytical, TRAC-IK (C++), PyKDL, RTB-LM, IKPy")
    print("═" * 70)

    fk_csv = os.path.join(DATA_REAL_DIR, "ik_benchmark_fk_100k.csv")
    cart_csv = os.path.join(DATA_REAL_DIR, "ik_benchmark_cartesian_100k.csv")
    summary_csv = os.path.join(DATA_REAL_DIR, "ik_benchmark_summary.csv")

    all_summaries = []

    # Check if Set A is already completed
    if os.path.exists(fk_csv) and os.path.getsize(fk_csv) > 50_000_000 and os.path.exists(summary_csv):
        print(f"\n[INFO] Set A ({fk_csv}) already completed (500k rows). Loading summary from disk.")
        with open(summary_csv, "r", encoding="utf-8") as f_s:
            r_s = csv.DictReader(f_s)
            for row in r_s:
                if row["dataset"] == "Set_A_FK_Reachable_100k":
                    all_summaries.append(row)
    else:
        # Run Benchmark (a): 100k Poses via FK (100% Reachable)
        summary_fk = run_benchmark_dataset(
            dataset_name="Set_A_FK_Reachable_100k",
            poses_generator=generate_fk_poses,
            out_csv_path=fk_csv,
            n_poses=100000,
            n_workers=14
        )
        all_summaries.extend(summary_fk)

    # Run Benchmark (b): 100k Uniform Random Cartesian Poses
    summary_cart = run_benchmark_dataset(
        dataset_name="Set_B_Cartesian_Random_100k",
        poses_generator=generate_cartesian_poses,
        out_csv_path=cart_csv,
        n_poses=100000,
        n_workers=14
    )
    all_summaries.extend(summary_cart)

    # Write summary CSV
    print(f"\nWriting summary to {summary_csv}...")
    with open(summary_csv, "w", newline="", encoding="utf-8") as f_sum:
        writer = csv.DictWriter(f_sum, fieldnames=list(all_summaries[0].keys()))
        writer.writeheader()
        writer.writerows(all_summaries)

    print(f"\n✓ Completed all 200,000 empirical IK evaluations.")
    print(f"✓ Results saved to:")
    print(f"  - {fk_csv}")
    print(f"  - {cart_csv}")
    print(f"  - {summary_csv}\n")


if __name__ == "__main__":
    main()
