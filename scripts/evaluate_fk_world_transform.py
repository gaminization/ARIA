#!/usr/bin/env python3
"""
scripts/evaluate_fk_world_transform.py

Analyzes Stage 1b FK Comparison (10,000 trials from data/real/fk_comparison_10k.csv):
Applies the world->base_link transform (+90 deg yaw, 0.614 m z) and evaluates:
Does URDF vs arm_ik difference shrink?

Outputs summary statistics and saves data/real/fk_comparison_world_transform.csv.
"""
import os
import sys
import numpy as np
import pandas as pd

WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
DATA_REAL_DIR = os.path.join(WORKSPACE_ROOT, "data", "real")
IN_CSV = os.path.join(DATA_REAL_DIR, "fk_comparison_10k.csv")
OUT_CSV = os.path.join(DATA_REAL_DIR, "fk_comparison_world_transform.csv")

def run_analysis():
    print("═" * 78)
    print("STAGE 1b FORWARD KINEMATICS TRANSFORM AUDIT")
    print("Evaluating world->base_link transform (+90 deg yaw, 0.614 m z)")
    print("═" * 78)

    df = pd.read_csv(IN_CSV)
    n_samples = len(df)
    print(f"Loaded {n_samples} configurations from {IN_CSV}")

    # URDF and arm_ik Cartesian points in original base_link frame
    urdf_p = df[["urdf_x_m", "urdf_y_m", "urdf_z_m"]].values
    arm_ik_p = df[["arm_ik_x_m", "arm_ik_y_m", "arm_ik_z_m"]].values
    original_diff = df["diff_urdf_arm_ik_m"].values

    print("\n--- Original Comparison (base_link frame) ---")
    print(f"Mean difference:   {original_diff.mean()*1000.0:.2f} mm ({original_diff.mean():.4f} m)")
    print(f"Median difference: {np.median(original_diff)*1000.0:.2f} mm ({np.median(original_diff):.4f} m)")
    print(f"Max difference:    {original_diff.max()*1000.0:.2f} mm ({original_diff.max():.4f} m)")
    print(f"Min difference:    {original_diff.min()*1000.0:.2f} mm ({original_diff.min():.4f} m)")

    # Transform T_world_base:
    # Origin: (0, 0, 0.614), RPY: (0, 0, +pi/2)
    # Forward: p_world = R_z(+pi/2) @ p_base + [0, 0, 0.614]
    # R_z(+pi/2) = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    # Thus: x_w = -y_b, y_w = x_b, z_w = z_b + 0.614
    # Inverse: x_b = y_w, y_b = -x_w, z_b = z_w - 0.614

    # Scenario 1: Applying T_world_base to URDF (URDF mapped to world, arm_ik remains in base)
    urdf_world = np.column_stack([-urdf_p[:, 1], urdf_p[:, 0], urdf_p[:, 2] + 0.614])
    diff_s1 = np.linalg.norm(urdf_world - arm_ik_p, axis=1)

    # Scenario 2: Applying T_world_base to arm_ik (arm_ik mapped to world, URDF in base)
    arm_ik_world = np.column_stack([-arm_ik_p[:, 1], arm_ik_p[:, 0], arm_ik_p[:, 2] + 0.614])
    diff_s2 = np.linalg.norm(urdf_p - arm_ik_world, axis=1)

    # Scenario 3: If arm_ik was in world frame, transforming arm_ik to base via T_base_world
    arm_ik_to_base = np.column_stack([arm_ik_p[:, 1], -arm_ik_p[:, 0], arm_ik_p[:, 2] - 0.614])
    diff_s3 = np.linalg.norm(urdf_p - arm_ik_to_base, axis=1)

    # Scenario 4: Applying only the +90 deg yaw rotation (ignoring 0.614m z offset)
    rot_arm_ik = np.column_stack([-arm_ik_p[:, 1], arm_ik_p[:, 0], arm_ik_p[:, 2]])
    diff_s4 = np.linalg.norm(urdf_p - rot_arm_ik, axis=1)

    # Scenario 5: Applying both to world frame (both transformed to world)
    diff_s5 = np.linalg.norm(urdf_world - arm_ik_world, axis=1)

    print("\n--- Transformed Comparison Scenarios ---")
    print(f"Scenario 1 (URDF in world vs arm_ik in base):")
    print(f"  Mean: {diff_s1.mean()*1000.0:.2f} mm, Max: {diff_s1.max()*1000.0:.2f} mm -> GROWS by {(diff_s1.mean() - original_diff.mean())*1000.0:+.2f} mm")

    print(f"Scenario 2 (arm_ik in world vs URDF in base):")
    print(f"  Mean: {diff_s2.mean()*1000.0:.2f} mm, Max: {diff_s2.max()*1000.0:.2f} mm -> GROWS by {(diff_s2.mean() - original_diff.mean())*1000.0:+.2f} mm")

    print(f"Scenario 3 (arm_ik assumed world, inverted to base vs URDF):")
    print(f"  Mean: {diff_s3.mean()*1000.0:.2f} mm, Max: {diff_s3.max()*1000.0:.2f} mm -> GROWS by {(diff_s3.mean() - original_diff.mean())*1000.0:+.2f} mm")

    print(f"Scenario 4 (Yaw +90 deg rotation only):")
    print(f"  Mean: {diff_s4.mean()*1000.0:.2f} mm, Max: {diff_s4.max()*1000.0:.2f} mm -> Change: {(diff_s4.mean() - original_diff.mean())*1000.0:+.2f} mm")

    print(f"Scenario 5 (Both transformed to world frame):")
    print(f"  Mean: {diff_s5.mean()*1000.0:.2f} mm, Max: {diff_s5.max()*1000.0:.2f} mm -> Identical to base (rigid isometry preserves Euclidean distances)")

    # Save summary dataframe
    out_df = pd.DataFrame({
        "trial_id": df["trial_id"],
        "orig_diff_m": original_diff,
        "scen1_urdf_world_diff_m": diff_s1,
        "scen2_arm_ik_world_diff_m": diff_s2,
        "scen3_inv_world_diff_m": diff_s3,
        "scen4_yaw90_only_diff_m": diff_s4,
    })
    out_df.to_csv(OUT_CSV, index=False)
    print(f"\nSaved transform evaluation to {OUT_CSV}")

if __name__ == "__main__":
    run_analysis()
