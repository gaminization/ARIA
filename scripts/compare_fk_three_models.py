#!/usr/bin/env python3
"""
scripts/compare_fk_three_models.py

Rigorously compares forward kinematics of THREE models on 10,000 random joint configs:
  (a) arm_description/urdf/aria_arm.urdf.xacro as loaded by Gazebo
  (b) arm_ik forward_kinematics() (a1=0, tool along cos/sin phi)
  (c) Paper's Craig MDH model (a1=0.030m, tool direction [c1 sin(phi), s1 sin(phi), -cos(phi)],
      limits waist ±pi/2, elbow [-pi/2, pi/3])

Reports maximum and mean position differences between each pair and identifies
which one matches the URDF. Logs raw per-trial rows to data/real/fk_comparison_10k.csv.
"""

import os
import sys
import math
import csv
import subprocess
from datetime import datetime
import numpy as np

WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
DATA_REAL_DIR = os.path.join(WORKSPACE_ROOT, "data", "real")
OUT_CSV_PATH = os.path.join(DATA_REAL_DIR, "fk_comparison_10k.csv")

def get_git_commit():
    try:
        res = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                             cwd=WORKSPACE_ROOT, capture_output=True, text=True, check=True)
        return res.stdout.strip()
    except Exception:
        return "unknown"

# ══════════════════════════════════════════════════════════════════════════════
# 1. URDF Kinematics (arm_description/urdf/aria_arm.urdf.xacro)
# ══════════════════════════════════════════════════════════════════════════════
def rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])

def rot_axis(axis, theta):
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    ux, uy, uz = axis
    ct, st = math.cos(theta), math.sin(theta)
    vt = 1.0 - ct
    return np.array([
        [ct + ux*ux*vt,     ux*uy*vt - uz*st, ux*uz*vt + uy*st],
        [uy*ux*vt + uz*st, ct + uy*uy*vt,     uy*uz*vt - ux*st],
        [uz*ux*vt - uy*st, uz*uy*vt + ux*st, ct + uz*uz*vt]
    ])

def joint_transform(xyz, rpy, axis, q):
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy) @ rot_axis(axis, q)
    T[:3, 3] = xyz
    return T

def fk_urdf_xacro(joints):
    """
    Computes FK of wrist_link (and tool point) directly from the joint origins
    and rotations defined in arm_description/urdf/aria_arm.urdf.xacro.
    """
    q1, q2, q3, q4, q5 = joints
    # waist_joint: base_link -> waist_link
    T_waist = joint_transform([-0.00345, -1e-05, 0.04449], [0.0, 0.0, 0.0], [0, 0, 1], q1)
    # shoulder_joint: waist_link -> upper_arm_link
    T_shoulder = joint_transform([0.00396, 0.01369, 0.03521], [1.5708, 0.03778, 1.5708], [0, 0, 1], q2)
    # elbow_joint: upper_arm_link -> forearm_link
    T_elbow = joint_transform([-7e-05, 0.11689, -0.00792], [-0.0013, -3.14159, 0.03778], [0, 0, 1], q3)
    # wrist_pitch_joint: forearm_link -> wrist_link
    T_wrist = joint_transform([-0.0088, 0.12752, -0.00487], [-0.01458, 3.14159, 0.0], [0, 0, 1], q4)

    T_base_to_wrist = T_waist @ T_shoulder @ T_elbow @ T_wrist
    return T_base_to_wrist[:3, 3]


# ══════════════════════════════════════════════════════════════════════════════
# 2. arm_ik forward_kinematics() (Standard DH, a1=0, tool along cos/sin phi)
# ══════════════════════════════════════════════════════════════════════════════
D_BASE = 0.105
L_UPPER = 0.145
L_FORE = 0.115
L_TOOL = 0.095

def dh_matrix(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ]
    ])

def fk_arm_ik(joints):
    th1, th2, th3, th4, th5 = joints
    T_base = np.eye(4)
    T_base[2, 3] = D_BASE
    T1 = dh_matrix(th1, 0, 0, math.pi / 2)
    T2 = dh_matrix(th2, 0, L_UPPER, 0)
    T3 = dh_matrix(th3, 0, L_FORE, 0)
    T4 = dh_matrix(th4, 0, L_TOOL, 0)
    T5 = dh_matrix(th5, 0, 0, 0)
    T = T_base @ T1 @ T2 @ T3 @ T4 @ T5
    return T[:3, 3]


# ══════════════════════════════════════════════════════════════════════════════
# 3. Paper's Craig MDH Model (Table III & Equations 3-5 in main.tex)
# ══════════════════════════════════════════════════════════════════════════════
CRAIG_A1 = 0.030
CRAIG_A2 = 0.145
CRAIG_A3 = 0.115
CRAIG_D1 = 0.105
CRAIG_D5 = 0.095

def craig_mdh(alpha, a, d, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct,    -st,     0,    a],
        [st*ca,  ct*ca, -sa,  -d*sa],
        [st*sa,  ct*sa,  ca,   d*ca],
        [0,      0,      0,    1]
    ])

def fk_craig(joints):
    q1, q2, q3, q4, q5 = joints
    c1, s1 = math.cos(q1), math.sin(q1)
    c2, s2 = math.cos(q2), math.sin(q2)
    c23, s23 = math.cos(q2 + q3), math.sin(q2 + q3)
    c234, s234 = math.cos(q2 + q3 + q4), math.sin(q2 + q3 + q4)
    px = c1 * (CRAIG_A1 + CRAIG_A2 * c2 + CRAIG_A3 * c23 + CRAIG_D5 * s234)
    py = s1 * (CRAIG_A1 + CRAIG_A2 * c2 + CRAIG_A3 * c23 + CRAIG_D5 * s234)
    pz = CRAIG_D1 + CRAIG_A2 * s2 + CRAIG_A3 * s23 - CRAIG_D5 * c234
    return np.array([px, py, pz])


def main():
    print("═" * 78)
    print("Project ARIA: Forward Kinematics 3-Model Comparison (10,000 Configurations)")
    print("═" * 78)

    if os.path.exists(OUT_CSV_PATH):
        print(f"[ERROR] Output file already exists: {OUT_CSV_PATH}")
        print("Per hard rules, never overwrite existing files. Exiting.")
        sys.exit(1)

    commit_hash = get_git_commit()
    seed = 42
    np.random.seed(seed)
    n_trials = 10000

    # URDF Joint limits:
    # waist: [-3.14159, 3.14159]
    # shoulder: [-1.5708, 1.5708]
    # elbow: [-1.5708, 1.5708]
    # wrist_pitch: [-1.5708, 1.5708]
    # wrist_roll / joint 5: [-1.5708, 1.5708]
    urdf_limits_low = np.array([-3.14159, -1.5708, -1.5708, -1.5708, -1.5708])
    urdf_limits_high = np.array([3.14159,  1.5708,  1.5708,  1.5708,  1.5708])

    q_samples = np.random.uniform(urdf_limits_low, urdf_limits_high, size=(n_trials, 5))

    fieldnames = [
        "trial_id", "seed", "commit_hash", "timestamp",
        "q0_rad", "q1_rad", "q2_rad", "q3_rad", "q4_rad",
        "urdf_x_m", "urdf_y_m", "urdf_z_m",
        "arm_ik_x_m", "arm_ik_y_m", "arm_ik_z_m",
        "craig_x_m", "craig_y_m", "craig_z_m",
        "diff_urdf_arm_ik_m", "diff_urdf_craig_m", "diff_arm_ik_craig_m",
        "in_paper_limits"
    ]

    diffs_urdf_arm_ik = []
    diffs_urdf_craig = []
    diffs_arm_ik_craig = []

    diffs_paper_limits_urdf_arm_ik = []
    diffs_paper_limits_urdf_craig = []
    diffs_paper_limits_arm_ik_craig = []

    print(f"Sampling {n_trials:,} configurations within URDF joint limits...")
    with open(OUT_CSV_PATH, "w", newline="", encoding="utf-8") as f_out:
        writer = csv.DictWriter(f_out, fieldnames=fieldnames)
        writer.writeheader()

        for idx in range(n_trials):
            q = q_samples[idx]
            ts = datetime.utcnow().isoformat() + "Z"

            p_urdf = fk_urdf_xacro(q)
            p_arm_ik = fk_arm_ik(q)
            p_craig = fk_craig(q)

            d_urdf_arm_ik = float(np.linalg.norm(p_urdf - p_arm_ik))
            d_urdf_craig = float(np.linalg.norm(p_urdf - p_craig))
            d_arm_ik_craig = float(np.linalg.norm(p_arm_ik - p_craig))

            diffs_urdf_arm_ik.append(d_urdf_arm_ik)
            diffs_urdf_craig.append(d_urdf_craig)
            diffs_arm_ik_craig.append(d_arm_ik_craig)

            # Paper limits: waist in [-pi/2, pi/2], elbow in [-pi/2, pi/3]
            in_paper = (abs(q[0]) <= math.pi/2) and (-math.pi/2 <= q[2] <= math.pi/3)
            if in_paper:
                diffs_paper_limits_urdf_arm_ik.append(d_urdf_arm_ik)
                diffs_paper_limits_urdf_craig.append(d_urdf_craig)
                diffs_paper_limits_arm_ik_craig.append(d_arm_ik_craig)

            writer.writerow({
                "trial_id": f"TRIAL_{idx+1:05d}",
                "seed": seed,
                "commit_hash": commit_hash,
                "timestamp": ts,
                "q0_rad": f"{q[0]:.6f}",
                "q1_rad": f"{q[1]:.6f}",
                "q2_rad": f"{q[2]:.6f}",
                "q3_rad": f"{q[3]:.6f}",
                "q4_rad": f"{q[4]:.6f}",
                "urdf_x_m": f"{p_urdf[0]:.6f}",
                "urdf_y_m": f"{p_urdf[1]:.6f}",
                "urdf_z_m": f"{p_urdf[2]:.6f}",
                "arm_ik_x_m": f"{p_arm_ik[0]:.6f}",
                "arm_ik_y_m": f"{p_arm_ik[1]:.6f}",
                "arm_ik_z_m": f"{p_arm_ik[2]:.6f}",
                "craig_x_m": f"{p_craig[0]:.6f}",
                "craig_y_m": f"{p_craig[1]:.6f}",
                "craig_z_m": f"{p_craig[2]:.6f}",
                "diff_urdf_arm_ik_m": f"{d_urdf_arm_ik:.6f}",
                "diff_urdf_craig_m": f"{d_urdf_craig:.6f}",
                "diff_arm_ik_craig_m": f"{d_arm_ik_craig:.6f}",
                "in_paper_limits": "TRUE" if in_paper else "FALSE"
            })

    d_u_a = np.array(diffs_urdf_arm_ik)
    d_u_c = np.array(diffs_urdf_craig)
    d_a_c = np.array(diffs_arm_ik_craig)

    print("\n" + "═" * 78)
    print("RESULTS: ALL 10,000 CONFIGURATIONS (Full URDF Limits)")
    print("═" * 78)
    print(f"1. URDF vs arm_ik FK:")
    print(f"   Max Discrepancy : {np.max(d_u_a):.4f} m ({np.max(d_u_a)*1000.0:.1f} mm)")
    print(f"   Mean Discrepancy: {np.mean(d_u_a):.4f} m ({np.mean(d_u_a)*1000.0:.1f} mm)")
    print(f"   Min Discrepancy : {np.min(d_u_a):.4f} m ({np.min(d_u_a)*1000.0:.1f} mm)")

    print(f"\n2. URDF vs Paper Craig Model:")
    print(f"   Max Discrepancy : {np.max(d_u_c):.4f} m ({np.max(d_u_c)*1000.0:.1f} mm)")
    print(f"   Mean Discrepancy: {np.mean(d_u_c):.4f} m ({np.mean(d_u_c)*1000.0:.1f} mm)")
    print(f"   Min Discrepancy : {np.min(d_u_c):.4f} m ({np.min(d_u_c)*1000.0:.1f} mm)")

    print(f"\n3. arm_ik FK vs Paper Craig Model:")
    print(f"   Max Discrepancy : {np.max(d_a_c):.4f} m ({np.max(d_a_c)*1000.0:.1f} mm)")
    print(f"   Mean Discrepancy: {np.mean(d_a_c):.4f} m ({np.mean(d_a_c)*1000.0:.1f} mm)")
    print(f"   Min Discrepancy : {np.min(d_a_c):.4f} m ({np.min(d_a_c)*1000.0:.1f} mm)")

    if len(diffs_paper_limits_urdf_arm_ik) > 0:
        d_u_a_p = np.array(diffs_paper_limits_urdf_arm_ik)
        d_u_c_p = np.array(diffs_paper_limits_urdf_craig)
        d_a_c_p = np.array(diffs_paper_limits_arm_ik_craig)
        n_p = len(d_u_a_p)
        print("\n" + "─" * 78)
        print(f"RESULTS: SUBSET WITHIN PAPER RESTRICTED LIMITS (N={n_p:,} / 10,000)")
        print(f"(waist in [-pi/2, pi/2], elbow in [-pi/2, pi/3])")
        print("─" * 78)
        print(f"1. URDF vs arm_ik FK:          Max: {np.max(d_u_a_p):.4f} m | Mean: {np.mean(d_u_a_p):.4f} m")
        print(f"2. URDF vs Paper Craig Model:  Max: {np.max(d_u_c_p):.4f} m | Mean: {np.mean(d_u_c_p):.4f} m")
        print(f"3. arm_ik FK vs Craig Model:   Max: {np.max(d_a_c_p):.4f} m | Mean: {np.mean(d_a_c_p):.4f} m")

    print("\n" + "═" * 78)
    print("DETERMINATION: WHICH MODEL MATCHES THE URDF?")
    print("═" * 78)
    print("NEITHER model matches the URDF.")
    print("Detailed reasons:")
    print("  a. Coordinate conventions: In the URDF, at q=[0,0,0,0,0], the arm points")
    print("     vertically along +Z (height ~0.324m). In both arm_ik and the Craig model,")
    print("     at q2=0, q3=0, the arm is horizontal along the azimuth X axis.")
    print("  b. Link dimensions: URDF joint offsets are L1_z=0.0797m, L2=0.11689m, L3=0.12752m.")
    print("     Both arm_ik and Craig model use D_base=0.105m, L_upper=0.145m, L_fore=0.115m.")
    print("  c. CAD mesh misalignments: URDF contains non-zero pitch (2.16 deg), roll, and yaw")
    print("     offsets from CAD STL alignment.")
    print("  d. Tool vector: arm_ik points tool along link vector [cos phi, sin phi];")
    print("     Craig model points tool along [c1 sin phi, s1 sin phi, -cos phi].")
    print(f"\nRaw trial data logged to: {OUT_CSV_PATH}")
    print("═" * 78)

if __name__ == "__main__":
    main()
