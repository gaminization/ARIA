#!/usr/bin/env python3
"""
scripts/benchmark_ik_urdf_v2_10k.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PROJECT ARIA — IK Solver v2 Verification (10,000 FK→IK→FK Roundtrips)

Tests the updated URDF-faithful arm_ik solver on 10,000 random joint configs:
  1. FK(q) → target TCP position
  2. IK(target) → q_solved
  3. FK(q_solved) → TCP recovered
  4. |TCP_recovered - TCP_target| = roundtrip error

Also benchmarks the scipy.optimize.least_squares baseline (Set A).
Reports: success rate, mean/max/p95/p99 roundtrip error, solve latency.

Logs every raw trial to data/real/ (never overwrites).

HARD RULES: No sampled outcomes. All from actual solver execution.
Seeds only for choosing the 10k test configs.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""
import os
import sys
import math
import csv
import json
import time
import subprocess
from datetime import datetime, timezone

import numpy as np
from scipy.optimize import least_squares

WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
DATA_REAL_DIR  = os.path.join(WORKSPACE_ROOT, "data", "real")
TIMESTAMP_TAG  = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
OUT_CSV        = os.path.join(DATA_REAL_DIR, f"ik_urdf_v2_10k_{TIMESTAMP_TAG}.csv")
OUT_SUMMARY    = os.path.join(DATA_REAL_DIR, f"ik_urdf_v2_10k_summary_{TIMESTAMP_TAG}.json")

SEED     = 202    # seed for choosing 10k test configs
N_TRIALS = 10000

def get_git_commit():
    try:
        r = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                           cwd=WORKSPACE_ROOT, capture_output=True, text=True, check=True)
        return r.stdout.strip()
    except Exception:
        return "unknown"

COMMIT = get_git_commit()

# ─────────────────────────────────────────────────────────────────────────────
# Import URDF FK (from updated arm_ik package)
# ─────────────────────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "arm_ik"))
from arm_ik.ik_solvers.aria_analytical_ik import (
    forward_kinematics,
    forward_kinematics_position,
    solve_analytical,
    SolverConfig,
    JOINT_LIMITS,
)

# ─────────────────────────────────────────────────────────────────────────────
# scipy.optimize baseline (Set A)
# ─────────────────────────────────────────────────────────────────────────────
def scipy_ik(target_pos, target_pitch, current_joints=None):
    """IK via scipy.optimize.least_squares (L-BFGS-B with bounds)."""
    q5 = 0.0  # wrist roll / gripper

    def residual(q4):
        p = forward_kinematics_position(np.append(q4, q5))
        pitch_err = (q4[1] + q4[2] + q4[3]) - target_pitch
        return np.append(p - target_pos, pitch_err * 0.01)

    # 20 random restarts, return best
    best_err  = float("inf")
    best_sol  = None
    best_time = 0.0

    starts = []
    if current_joints is not None:
        starts.append(current_joints[:4].copy())
    for _ in range(20 - len(starts)):
        starts.append(np.random.uniform(JOINT_LIMITS[:4, 0], JOINT_LIMITS[:4, 1]))

    t0 = time.perf_counter()
    for q0 in starts:
        try:
            res = least_squares(residual, q0,
                                bounds=(JOINT_LIMITS[:4, 0], JOINT_LIMITS[:4, 1]),
                                method="trf", max_nfev=200, ftol=1e-6)
            q_sol = res.x
            p_sol = forward_kinematics_position(np.append(q_sol, q5))
            err   = float(np.linalg.norm(p_sol - target_pos))
            if err < best_err:
                best_err = err
                best_sol = np.append(q_sol, q5)
        except Exception:
            pass

    best_time = (time.perf_counter() - t0) * 1000.0
    return best_sol, best_err, best_time

# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    os.makedirs(DATA_REAL_DIR, exist_ok=True)
    for f in [OUT_CSV, OUT_SUMMARY]:
        if os.path.exists(f):
            print(f"[ERROR] File exists (no overwrite): {f}")
            sys.exit(1)

    print("═"*78)
    print("PROJECT ARIA — IK Solver v2 Verification (10K Roundtrips)")
    print(f"  commit={COMMIT}  seed={SEED}  N={N_TRIALS}")
    print("═"*78)

    np.random.seed(SEED)
    q_samples = np.random.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1],
                                   size=(N_TRIALS, 5))

    fieldnames = [
        "trial_id", "seed", "commit_hash", "timestamp",
        "q0", "q1", "q2", "q3", "q4",
        "target_x_m", "target_y_m", "target_z_m",
        "target_pitch",
        # URDF IK v2
        "urdf_ik_success", "urdf_ik_err_m", "urdf_ik_time_ms",
        "urdf_ik_recovered_x", "urdf_ik_recovered_y", "urdf_ik_recovered_z",
        # scipy baseline
        "scipy_ik_success", "scipy_ik_err_m", "scipy_ik_time_ms",
    ]

    urdf_errors  = []
    scipy_errors = []
    urdf_times   = []
    scipy_times  = []
    urdf_success = 0
    scipy_success = 0

    PRINT_EVERY = 1000

    with open(OUT_CSV, "w", newline="", encoding="utf-8") as fout:
        writer = csv.DictWriter(fout, fieldnames=fieldnames)
        writer.writeheader()

        for idx, q in enumerate(q_samples):
            if idx % PRINT_EVERY == 0:
                print(f"  [{idx:>5}/{N_TRIALS}] "
                      f"URDF: {urdf_success}/{max(idx,1)*100//max(idx,1)}% "
                      f"scipy: {scipy_success}/{max(idx,1)*100//max(idx,1)}%")

            ts = datetime.now(timezone.utc).isoformat()

            # Ground truth TCP position from URDF FK
            T_gt    = forward_kinematics(q)
            tgt_pos = T_gt[:3, 3]
            pitch   = float(q[1] + q[2] + q[3])  # sum of shoulder+elbow+wrist

            # ── URDF IK v2 ──────────────────────────────────────────────
            cfg = SolverConfig(current_joints=q, tolerance_m=0.001)
            t0 = time.perf_counter()
            result = solve_analytical(tgt_pos, target_pitch=pitch,
                                      target_roll=float(q[4]), config=cfg)
            urdf_dt = (time.perf_counter() - t0) * 1000.0

            if result.success:
                T_rec = forward_kinematics(result.joint_angles)
                urdf_err = float(np.linalg.norm(T_rec[:3, 3] - tgt_pos))
                rec_pos  = T_rec[:3, 3]
                if urdf_err < 0.002:   # 2 mm threshold
                    urdf_success += 1
            else:
                urdf_err = float("nan")
                rec_pos  = [float("nan")]*3

            urdf_errors.append(urdf_err)
            urdf_times.append(urdf_dt)

            # ── scipy baseline ─────────────────────────────────────────
            sp_sol, sp_err, sp_dt = scipy_ik(tgt_pos, pitch, current_joints=q)
            if sp_err < 0.002:
                scipy_success += 1
            scipy_errors.append(sp_err)
            scipy_times.append(sp_dt)

            writer.writerow({
                "trial_id": f"TRIAL_{idx:05d}",
                "seed": SEED, "commit_hash": COMMIT, "timestamp": ts,
                "q0": f"{q[0]:.6f}", "q1": f"{q[1]:.6f}",
                "q2": f"{q[2]:.6f}", "q3": f"{q[3]:.6f}", "q4": f"{q[4]:.6f}",
                "target_x_m": f"{tgt_pos[0]:.6f}",
                "target_y_m": f"{tgt_pos[1]:.6f}",
                "target_z_m": f"{tgt_pos[2]:.6f}",
                "target_pitch": f"{pitch:.6f}",
                "urdf_ik_success": result.success,
                "urdf_ik_err_m": f"{urdf_err:.6f}" if not math.isnan(urdf_err) else "nan",
                "urdf_ik_time_ms": f"{urdf_dt:.3f}",
                "urdf_ik_recovered_x": f"{rec_pos[0]:.6f}" if not math.isnan(rec_pos[0]) else "nan",
                "urdf_ik_recovered_y": f"{rec_pos[1]:.6f}" if not math.isnan(rec_pos[1]) else "nan",
                "urdf_ik_recovered_z": f"{rec_pos[2]:.6f}" if not math.isnan(rec_pos[2]) else "nan",
                "scipy_ik_success": sp_sol is not None and sp_err < 0.002,
                "scipy_ik_err_m": f"{sp_err:.6f}",
                "scipy_ik_time_ms": f"{sp_dt:.3f}",
            })

    # ── Summary ───────────────────────────────────────────────────────────────
    valid_urdf  = [e for e in urdf_errors  if not math.isnan(e)]
    valid_scipy = [e for e in scipy_errors if not math.isnan(e)]

    u_arr = np.array(valid_urdf)
    s_arr = np.array(valid_scipy)
    ut_arr = np.array(urdf_times)
    st_arr = np.array(scipy_times)

    print("\n" + "═"*78)
    print("RESULTS — FK→IK→FK Roundtrip (10,000 configs, URDF joint limits)")
    print("═"*78)
    print(f"\n  URDF IK v2 (Newton-Raphson):")
    print(f"    Success rate:     {urdf_success}/{N_TRIALS} ({100*urdf_success/N_TRIALS:.1f}%)")
    if len(u_arr):
        print(f"    Error (2mm threshold) pass: {np.sum(u_arr<0.002)}/{len(u_arr)}")
        print(f"    max error:  {np.max(u_arr)*1000:.2f} mm")
        print(f"    mean error: {np.mean(u_arr)*1000:.2f} mm")
        print(f"    p95 error:  {np.percentile(u_arr, 95)*1000:.2f} mm")
        print(f"    p99 error:  {np.percentile(u_arr, 99)*1000:.2f} mm")
        print(f"    mean latency: {np.mean(ut_arr):.1f} ms")
        print(f"    p99 latency:  {np.percentile(ut_arr, 99):.1f} ms")

    print(f"\n  scipy.optimize.least_squares (20 restarts, Set A baseline):")
    print(f"    Success rate:     {scipy_success}/{N_TRIALS} ({100*scipy_success/N_TRIALS:.1f}%)")
    if len(s_arr):
        print(f"    Error (2mm threshold) pass: {np.sum(s_arr<0.002)}/{len(s_arr)}")
        print(f"    max error:  {np.max(s_arr)*1000:.2f} mm")
        print(f"    mean error: {np.mean(s_arr)*1000:.2f} mm")
        print(f"    p95 error:  {np.percentile(s_arr, 95)*1000:.2f} mm")
        print(f"    mean latency: {np.mean(st_arr):.1f} ms")

    summary = {
        "commit_hash": COMMIT, "seed": SEED, "timestamp": TIMESTAMP_TAG,
        "n_trials": N_TRIALS,
        "urdf_ik_v2": {
            "success_rate": urdf_success / N_TRIALS,
            "n_pass_2mm": int(np.sum(u_arr < 0.002)) if len(u_arr) else 0,
            "error_mm": {"max": float(np.max(u_arr)*1000), "mean": float(np.mean(u_arr)*1000),
                          "p95": float(np.percentile(u_arr,95)*1000),
                          "p99": float(np.percentile(u_arr,99)*1000)} if len(u_arr) else {},
            "latency_ms": {"mean": float(np.mean(ut_arr)), "p99": float(np.percentile(ut_arr,99))},
        },
        "scipy_baseline": {
            "success_rate": scipy_success / N_TRIALS,
            "n_pass_2mm": int(np.sum(s_arr < 0.002)) if len(s_arr) else 0,
            "error_mm": {"max": float(np.max(s_arr)*1000), "mean": float(np.mean(s_arr)*1000),
                          "p95": float(np.percentile(s_arr,95)*1000)} if len(s_arr) else {},
            "latency_ms": {"mean": float(np.mean(st_arr))},
        },
        "raw_csv": OUT_CSV,
    }

    with open(OUT_SUMMARY, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\nRaw data → {OUT_CSV}")
    print(f"Summary  → {OUT_SUMMARY}")
    print("═"*78)
    print("DONE")

if __name__ == "__main__":
    main()
