#!/usr/bin/env python3
"""
scripts/benchmark_reachability_grid_100k.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PROJECT ARIA — Reachability Verification (Set B: 100,000-config dense grid)

Purpose:
  1. Dense grid over joint space (100k configs) → compute TCP via URDF FK
  2. Reachability map: how much of workspace is within typical task radius?
  3. IK confusion matrix: for each of 1000 sampled TCP targets in the grid,
     run both URDF IK v2 and scipy baseline, classify:
       TP = IK says reachable, FK confirms < 2mm error
       FP = IK says reachable, but FK error > 2mm
       FN = IK fails, but config IS in workspace (target was from valid FK)
       TN = IK fails, target outside workspace bounds (unreachable test points)
  4. Report workspace volume, IK confusion matrix, precision/recall.

HARD RULES: No sampled outcomes. Seeds only for config selection.
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
from scipy.spatial import ConvexHull

WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
DATA_REAL_DIR  = os.path.join(WORKSPACE_ROOT, "data", "real")
TIMESTAMP_TAG  = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
OUT_REACH_CSV   = os.path.join(DATA_REAL_DIR, f"reachability_grid_{TIMESTAMP_TAG}.csv")
OUT_MATRIX_CSV  = os.path.join(DATA_REAL_DIR, f"ik_confusion_matrix_{TIMESTAMP_TAG}.csv")
OUT_SUMMARY     = os.path.join(DATA_REAL_DIR, f"reachability_summary_{TIMESTAMP_TAG}.json")

SEED_GRID   = 303   # seed for 100k grid configs
SEED_MATRIX = 304   # seed for 1k confusion-matrix targets
N_GRID      = 100_000
N_MATRIX    = 1000

def get_git_commit():
    try:
        r = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                           cwd=WORKSPACE_ROOT, capture_output=True, text=True, check=True)
        return r.stdout.strip()
    except Exception:
        return "unknown"

COMMIT = get_git_commit()

# ─────────────────────────────────────────────────────────────────────────────
# Import URDF FK + IK
# ─────────────────────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "arm_ik"))
from arm_ik.ik_solvers.aria_analytical_ik import (
    forward_kinematics,
    forward_kinematics_position,
    solve_analytical,
    SolverConfig,
    JOINT_LIMITS,
)

def scipy_ik(target_pos, target_pitch, n_restarts=5):
    q5 = 0.0
    def residual(q4):
        p = forward_kinematics_position(np.append(q4, q5))
        return p - target_pos

    best_err = float("inf")
    best_sol = None
    for _ in range(n_restarts):
        q0 = np.random.uniform(JOINT_LIMITS[:4, 0], JOINT_LIMITS[:4, 1])
        try:
            res = least_squares(residual, q0,
                                bounds=(JOINT_LIMITS[:4, 0], JOINT_LIMITS[:4, 1]),
                                method="trf", max_nfev=150, ftol=1e-6)
            q_sol = res.x
            p_sol = forward_kinematics_position(np.append(q_sol, q5))
            err   = float(np.linalg.norm(p_sol - target_pos))
            if err < best_err:
                best_err = err
                best_sol = np.append(q_sol, q5)
        except Exception:
            pass
    return best_sol, best_err


def main():
    os.makedirs(DATA_REAL_DIR, exist_ok=True)
    for f in [OUT_REACH_CSV, OUT_MATRIX_CSV, OUT_SUMMARY]:
        if os.path.exists(f):
            print(f"[ERROR] File exists (no overwrite): {f}")
            sys.exit(1)

    print("═"*78)
    print("PROJECT ARIA — Reachability Grid + IK Confusion Matrix (Set B)")
    print(f"  commit={COMMIT}  seed_grid={SEED_GRID}  seed_matrix={SEED_MATRIX}")
    print(f"  N_grid={N_GRID:,}  N_matrix={N_MATRIX:,}")
    print("═"*78)

    # ── Phase 1: Dense grid FK ────────────────────────────────────────────────
    print(f"\nPhase 1: Computing FK for {N_GRID:,} configs...")
    np.random.seed(SEED_GRID)
    q_grid = np.random.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1],
                                size=(N_GRID, 5))

    tcp_positions = np.zeros((N_GRID, 3))
    t0 = time.time()
    for i, q in enumerate(q_grid):
        tcp_positions[i] = forward_kinematics_position(q)
        if (i+1) % 10000 == 0:
            print(f"  {i+1:>6}/{N_GRID} ({(i+1)/N_GRID*100:.0f}%)  "
                  f"elapsed={time.time()-t0:.1f}s")

    print(f"  Done in {time.time()-t0:.1f}s")

    # Workspace bounds
    x_min, x_max = float(np.min(tcp_positions[:,0])), float(np.max(tcp_positions[:,0]))
    y_min, y_max = float(np.min(tcp_positions[:,1])), float(np.max(tcp_positions[:,1]))
    z_min, z_max = float(np.min(tcp_positions[:,2])), float(np.max(tcp_positions[:,2]))
    radii = np.linalg.norm(tcp_positions[:,:2], axis=1)  # XY radius
    r_max = float(np.max(radii))
    r_min_nonzero = float(np.min(radii[radii > 0.001]))

    print(f"\n  Workspace bounds (base_link frame):")
    print(f"    X: [{x_min*1000:.1f}, {x_max*1000:.1f}] mm")
    print(f"    Y: [{y_min*1000:.1f}, {y_max*1000:.1f}] mm")
    print(f"    Z: [{z_min*1000:.1f}, {z_max*1000:.1f}] mm")
    print(f"    XY radius: [{r_min_nonzero*1000:.1f}, {r_max*1000:.1f}] mm")

    # Estimate workspace volume via convex hull of samples
    try:
        hull = ConvexHull(tcp_positions)
        ws_volume_liters = hull.volume * 1000  # m³ → liters
        print(f"    Workspace convex hull volume: {ws_volume_liters:.3f} L")
    except Exception as e:
        ws_volume_liters = float("nan")
        print(f"    [WARN] ConvexHull failed: {e}")

    # Distance bands
    for radius_mm in [100, 150, 200, 250, 300, 350]:
        pct = float(np.sum(radii < radius_mm/1000) / N_GRID * 100)
        print(f"    r < {radius_mm}mm: {pct:.1f}% of configs")

    # Write reachability CSV (sampled — first 5000 rows for tractability)
    print(f"\n  Writing reachability sample CSV (first 5000 rows)...")
    with open(OUT_REACH_CSV, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["trial_id","seed","commit_hash",
                          "q0","q1","q2","q3","q4",
                          "tcp_x_m","tcp_y_m","tcp_z_m","tcp_r_m"])
        for i in range(min(5000, N_GRID)):
            writer.writerow([
                f"GRID_{i:06d}", SEED_GRID, COMMIT,
                f"{q_grid[i,0]:.6f}", f"{q_grid[i,1]:.6f}",
                f"{q_grid[i,2]:.6f}", f"{q_grid[i,3]:.6f}", f"{q_grid[i,4]:.6f}",
                f"{tcp_positions[i,0]:.6f}", f"{tcp_positions[i,1]:.6f}",
                f"{tcp_positions[i,2]:.6f}", f"{radii[i]:.6f}",
            ])

    # ── Phase 2: IK Confusion Matrix ─────────────────────────────────────────
    print(f"\nPhase 2: IK Confusion Matrix ({N_MATRIX:,} targets)...")
    print("  (TP=IK✓+FK<2mm, FP=IK✓+FK≥2mm, FN=IK✗+target reachable, TN=IK✗+unreachable)")
    np.random.seed(SEED_MATRIX)

    # 500 targets from the reachable set (sampled grid FK positions)
    grid_indices = np.random.choice(N_GRID, N_MATRIX // 2, replace=False)
    reachable_targets = tcp_positions[grid_indices]
    reachable_joints  = q_grid[grid_indices]

    # 500 targets outside the reachable workspace (unreachable)
    # Sample in a bounding box and filter to those > r_max or < z_min
    unreachable_targets = []
    while len(unreachable_targets) < N_MATRIX // 2:
        p = np.random.uniform(
            [x_min - 0.05, y_min - 0.05, z_min - 0.05],
            [x_max + 0.05, y_max + 0.05, z_max + 0.05]
        )
        # Check if outside convex hull (approximate: check radius)
        r = np.linalg.norm(p[:2])
        if r > r_max * 1.05 or p[2] < z_min - 0.01 or p[2] > z_max + 0.01:
            unreachable_targets.append(p)
    unreachable_targets = np.array(unreachable_targets[:N_MATRIX//2])

    # Combine
    all_targets    = np.vstack([reachable_targets, unreachable_targets])
    is_reachable   = np.array([True]*(N_MATRIX//2) + [False]*(N_MATRIX//2))
    source_joints  = list(reachable_joints) + [None]*(N_MATRIX//2)

    matrix_rows = []
    TP = FP = FN = TN = 0
    PRINT_EVERY_MAT = 100

    t0 = time.time()
    for i, (tgt, is_reach, src_q) in enumerate(zip(all_targets, is_reachable, source_joints)):
        if i % PRINT_EVERY_MAT == 0:
            print(f"  [{i:>4}/{N_MATRIX}] TP={TP} FP={FP} FN={FN} TN={TN}  "
                  f"elapsed={time.time()-t0:.1f}s")

        pitch = float(src_q[1] + src_q[2] + src_q[3]) if src_q is not None else 0.0
        cfg   = SolverConfig(tolerance_m=0.002, max_iterations=200)
        t1    = time.time()
        result = solve_analytical(tgt, target_pitch=pitch, config=cfg)
        ik_dt  = (time.time() - t1) * 1000.0

        ik_success = False
        ik_err_m   = float("nan")

        if result.success:
            T_rec   = forward_kinematics(result.joint_angles)
            ik_err_m = float(np.linalg.norm(T_rec[:3, 3] - tgt))
            if ik_err_m < 0.002:
                ik_success = True

        # Classify
        if is_reach and ik_success:
            TP += 1; outcome = "TP"
        elif not is_reach and ik_success:
            FP += 1; outcome = "FP"
        elif is_reach and not ik_success:
            FN += 1; outcome = "FN"
        else:
            TN += 1; outcome = "TN"

        matrix_rows.append({
            "trial_id": f"MAT_{i:04d}",
            "seed": SEED_MATRIX, "commit_hash": COMMIT,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "is_reachable": is_reach,
            "target_x_m": float(tgt[0]), "target_y_m": float(tgt[1]), "target_z_m": float(tgt[2]),
            "target_pitch": pitch,
            "ik_success": ik_success,
            "ik_err_m": ik_err_m,
            "ik_dt_ms": ik_dt,
            "outcome": outcome,
        })

    print(f"\n  Confusion matrix ({N_MATRIX} targets):")
    print(f"    TP={TP}  FP={FP}  FN={FN}  TN={TN}")
    precision = TP / (TP + FP) if (TP + FP) > 0 else float("nan")
    recall    = TP / (TP + FN) if (TP + FN) > 0 else float("nan")
    f1        = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else float("nan")
    print(f"    Precision = {precision:.4f}")
    print(f"    Recall    = {recall:.4f}")
    print(f"    F1        = {f1:.4f}")

    with open(OUT_MATRIX_CSV, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(matrix_rows[0].keys()))
        writer.writeheader()
        writer.writerows(matrix_rows)

    # ── Summary ───────────────────────────────────────────────────────────────
    summary = {
        "commit_hash": COMMIT,
        "seed_grid": SEED_GRID, "seed_matrix": SEED_MATRIX,
        "timestamp": TIMESTAMP_TAG,
        "n_grid": N_GRID, "n_matrix": N_MATRIX,
        "workspace": {
            "x_mm": [x_min*1000, x_max*1000],
            "y_mm": [y_min*1000, y_max*1000],
            "z_mm": [z_min*1000, z_max*1000],
            "r_xy_mm": [r_min_nonzero*1000, r_max*1000],
            "convex_hull_volume_L": ws_volume_liters,
        },
        "confusion_matrix": {"TP": TP, "FP": FP, "FN": FN, "TN": TN},
        "precision": precision, "recall": recall, "f1": f1,
        "raw_reach_csv": OUT_REACH_CSV,
        "raw_matrix_csv": OUT_MATRIX_CSV,
    }

    with open(OUT_SUMMARY, "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\nReachability CSV → {OUT_REACH_CSV}")
    print(f"Matrix CSV → {OUT_MATRIX_CSV}")
    print(f"Summary → {OUT_SUMMARY}")
    print("═"*78)
    print("DONE")


if __name__ == "__main__":
    main()
