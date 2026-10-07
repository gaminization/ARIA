#!/usr/bin/env python3
"""
scripts/gazebo_tf_kinematic_validator.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PROJECT ARIA — GAZEBO TF KINEMATIC VALIDATOR (Phases B + E)

For 20 predetermined joint configurations:
  1. Command joints via /aria/joint_stream (sensor_msgs/JointState, radians)
     Joint order: [waist, shoulder, elbow, wrist_pitch, gripper=0]
  2. Wait 2s for settle.
  3. Read /joint_states to confirm actual Gazebo joint positions.
  4. Read TF: world→wrist_link, base_link→wrist_link (via tf2_echo).
  5. Compare Gazebo TF vs URDF FK (from benchmark_authoritative_kinematics.py).
  6. Log every raw trial row to data/real/gazebo_tf_*.csv.

HARD RULES: No sampled outcomes. Everything comes from real Gazebo reads.
Seeds only for choosing test configs. Never overwrite data/real/ files.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""
import os
import sys
import math
import csv
import json
import time
import subprocess
import re
from datetime import datetime, timezone

import numpy as np

WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
DATA_REAL_DIR  = os.path.join(WORKSPACE_ROOT, "data", "real")
TIMESTAMP_TAG  = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
OUT_CSV        = os.path.join(DATA_REAL_DIR, f"gazebo_tf_{TIMESTAMP_TAG}.csv")
OUT_SUMMARY    = os.path.join(DATA_REAL_DIR, f"gazebo_tf_summary_{TIMESTAMP_TAG}.json")

ROS_ENV = os.environ.copy()
ROS_ENV["GAZEBO_IP"] = "127.0.0.1"
ROS_ENV["GAZEBO_MASTER_URI"] = "http://127.0.0.1:11345"

def get_git_commit():
    try:
        r = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                           cwd=WORKSPACE_ROOT, capture_output=True, text=True, check=True)
        return r.stdout.strip()
    except Exception:
        return "unknown"

COMMIT = get_git_commit()
SEED   = 101   # same seed as benchmark_authoritative_kinematics.py

# ─────────────────────────────────────────────────────────────────────────────
# URDF FK (exact copy from benchmark_authoritative_kinematics.py)
# ─────────────────────────────────────────────────────────────────────────────
def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr]
    ])

def Rz(theta):
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([[ct, -st, 0], [st, ct, 0], [0, 0, 1]])

def make_T(xyz, rpy, q_z):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy) @ Rz(q_z)
    T[:3,  3] = np.array(xyz)
    return T

# Joint params from URDF
FIXED_BASE  = dict(xyz=[0.0, 0.0, 0.614],        rpy=[0, 0, 1.5708])
WAIST_J     = dict(xyz=[-0.00345, -1e-05, 0.04449], rpy=[0.0, 0.0, 0.0])
SHOULDER_J  = dict(xyz=[0.00396, 0.01369, 0.03521],  rpy=[1.5708, 0.03778, 1.5708])
ELBOW_J     = dict(xyz=[-7e-05, 0.11689, -0.00792],  rpy=[-0.0013, -3.14159, 0.03778])
WRIST_J     = dict(xyz=[-0.0088, 0.12752, -0.00487], rpy=[-0.01458, 3.14159, 0.0])
CAMERA_J    = dict(xyz=[0.025, 0.050, -0.007],       rpy=[1.5708, 0.0, 2.3358])

def urdf_fk_base(q4, end="wrist_link"):
    """FK from base_link to wrist_link (or wrist_camera_link). q4 = [waist, shoulder, elbow, wrist_pitch]."""
    T = np.eye(4)
    T = T @ make_T(WAIST_J["xyz"],    WAIST_J["rpy"],    q4[0])
    T = T @ make_T(SHOULDER_J["xyz"], SHOULDER_J["rpy"], q4[1])
    T = T @ make_T(ELBOW_J["xyz"],    ELBOW_J["rpy"],    q4[2])
    T = T @ make_T(WRIST_J["xyz"],    WRIST_J["rpy"],    q4[3])
    if end == "wrist_camera_link":
        Tc = np.eye(4)
        Tc[:3, :3] = rpy_to_R(*CAMERA_J["rpy"])
        Tc[:3, 3]  = np.array(CAMERA_J["xyz"])
        T = T @ Tc
    return T

def urdf_fk_world(q4, end="wrist_link"):
    """FK from world to wrist_link."""
    T_fb = np.eye(4)
    T_fb[:3, :3] = rpy_to_R(*FIXED_BASE["rpy"])
    T_fb[:3, 3]  = np.array(FIXED_BASE["xyz"])
    return T_fb @ urdf_fk_base(q4, end)

# ─────────────────────────────────────────────────────────────────────────────
# 20 test configurations (same seed as authoritative benchmark)
# ─────────────────────────────────────────────────────────────────────────────
np.random.seed(SEED)
_lim_lo  = np.array([-math.pi,  -math.pi/2, -math.pi/2, -math.pi/2])
_lim_hi  = np.array([ math.pi,   math.pi/2,  math.pi/2,  math.pi/2])
_rand4   = np.random.uniform(_lim_lo, _lim_hi, size=(10, 4))
np.random.seed(SEED)   # Re-seed identically to get the same 10 rand configs as 5-DOF version
_rand5   = np.random.uniform(
    np.array([-math.pi, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2]),
    np.array([ math.pi,  math.pi/2,  math.pi/2,  math.pi/2,  math.pi/2]),
    size=(10, 5)
)
_structured5 = np.array([
    [0.0,       0.0,     0.0,    0.0,   0.0],
    [0.0,       0.5,    -0.5,    0.0,   0.0],
    [math.pi/2, 0.3,    -0.3,    0.0,   0.0],
    [-math.pi/4,0.7,    -0.7,    0.0,   0.0],
    [math.pi/3, 0.0,     0.0,    0.5,   0.0],
    [0.0,      -0.5,     0.5,    0.0,   0.0],
    [math.pi,   0.4,    -0.4,    0.1,   0.0],
    [0.0,       math.pi/2, -math.pi/2, 0.0, 0.0],
    [0.0,       0.0,    -1.0,    math.pi/2, 0.0],
    [math.pi/6, 0.3,    -0.6,    0.3,   0.0],
])
TEST_CONFIGS_5DOF = np.vstack([_rand5, _structured5])   # (20, 5) matching authoritative script

# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 helpers
# ─────────────────────────────────────────────────────────────────────────────
SETUP_SH = "source /home/gaminizer/Projects/ARIA/install/setup.bash"

def shell(cmd, timeout=20):
    r = subprocess.run(["bash", "-c", f"{SETUP_SH} && {cmd}"],
                       capture_output=True, text=True, timeout=timeout, env=ROS_ENV)
    return r.stdout.strip(), r.stderr.strip(), r.returncode

def command_joints(q5, settle_s=2.5):
    """
    Send joint command via /aria/joint_stream (sensor_msgs/JointState, radians).
    Joint order per manual_control_node.py: [waist, shoulder, elbow, wrist_pitch, gripper].
    Waits settle_s seconds for the trajectory to execute.
    Returns True on successful publish.
    """
    pos_str = ", ".join(f"{p:.6f}" for p in q5)
    # Build JointState message
    msg = (
        f"{{name: ['waist_joint', 'shoulder_joint', 'elbow_joint', "
        f"'wrist_pitch_joint', 'gripper_joint'], "
        f"position: [{pos_str}], velocity: [], effort: []}}"
    )
    cmd = f"ros2 topic pub -1 /aria/joint_stream sensor_msgs/msg/JointState \"{msg}\""
    out, err, rc = shell(cmd, timeout=15)
    time.sleep(settle_s)
    return rc == 0, out

def read_joint_states():
    """
    Read one /joint_states message. Returns dict {name: pos_rad} or None.
    """
    cmd = "timeout 6 ros2 topic echo --once /joint_states sensor_msgs/msg/JointState 2>&1"
    out, _, _ = shell(cmd, timeout=10)
    names_m = re.search(r"name:\s*\[(.*?)\]", out, re.DOTALL)
    pos_m   = re.search(r"position:\s*\[(.*?)\]", out, re.DOTALL)
    if names_m and pos_m:
        names = [n.strip().strip("'\"") for n in names_m.group(1).split(",")]
        positions = [float(p.strip()) for p in pos_m.group(1).split(",")]
        return dict(zip(names, positions))
    return None

def read_tf(parent, child):
    """
    Read TF from parent to child frame via tf2_echo. Returns (x,y,z,qx,qy,qz,qw) or None.
    """
    cmd = f"timeout 6 ros2 run tf2_ros tf2_echo {parent} {child} 2>&1 | head -20"
    out, _, _ = shell(cmd, timeout=10)
    t = re.search(r"Translation:\s*\[([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\]", out)
    q = re.search(r"Rotation:\s*\[([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+),\s*([-\d.e+]+)\]", out)
    if t and q:
        return (float(t.group(1)), float(t.group(2)), float(t.group(3)),
                float(q.group(1)), float(q.group(2)), float(q.group(3)), float(q.group(4)))
    return None

def gazebo_ready(timeout=120):
    deadline = time.time() + timeout
    while time.time() < deadline:
        out, _, _ = shell("ros2 topic list 2>&1", timeout=5)
        if "/joint_states" in out:
            return True
        time.sleep(2)
    return False

# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    os.makedirs(DATA_REAL_DIR, exist_ok=True)
    for f in [OUT_CSV, OUT_SUMMARY]:
        if os.path.exists(f):
            print(f"[ERROR] File exists (hard rules: no overwrite): {f}")
            sys.exit(1)

    print("═"*78)
    print("PROJECT ARIA — Gazebo TF Kinematic Validator")
    print(f"  commit={COMMIT}  seed={SEED}  ts={TIMESTAMP_TAG}")
    print("═"*78)

    if not gazebo_ready(timeout=120):
        print("[BLOCKER] Gazebo not ready within 120s. Cannot proceed.")
        print("Start Gazebo first: ros2 launch arm_bringup industrial_workcell.launch.py gui:=false")
        sys.exit(2)

    print("  Gazebo ready. Starting 20-trial TF validation...\n")

    rows = []
    JOINT_ORDER = ["waist_joint", "shoulder_joint", "elbow_joint",
                   "wrist_pitch_joint", "gripper_joint"]

    for trial_idx, q5 in enumerate(TEST_CONFIGS_5DOF):
        ts = datetime.now(timezone.utc).isoformat()
        q4 = q5[:4]  # only arm joints for FK

        print(f"Trial {trial_idx:02d}/{len(TEST_CONFIGS_5DOF)-1}  "
              f"q=[{', '.join(f'{v:.3f}' for v in q5)}]")

        # 1. Command
        ok, pub_out = command_joints(q5, settle_s=2.5)
        print(f"  pub: {'OK' if ok else 'FAIL'}")

        # 2. Read joint states
        js = read_joint_states()
        gz_q = {n: float("nan") for n in JOINT_ORDER}
        if js:
            for n in JOINT_ORDER:
                gz_q[n] = js.get(n, float("nan"))
            reported = [gz_q[n] for n in JOINT_ORDER[:4]]
            q_err = [abs(q5[k] - reported[k]) for k in range(4)]
            print(f"  JS reported: [{', '.join(f'{v:.4f}' for v in reported)}]")
            print(f"  |cmd-actual|: [{', '.join(f'{e:.4f}' for e in q_err)}] rad")
        else:
            print("  [WARN] /joint_states read failed")

        # 3. TF reads
        tf_world_wrist  = read_tf("world",      "wrist_link")
        tf_base_wrist   = read_tf("base_link",  "wrist_link")
        tf_base_camera  = read_tf("base_link",  "wrist_camera_link")

        # 4. URDF FK (base frame)
        T_urdf_base  = urdf_fk_base(q4, "wrist_link")
        T_urdf_world = urdf_fk_world(q4, "wrist_link")
        urdf_base  = T_urdf_base[:3, 3]
        urdf_world = T_urdf_world[:3, 3]

        # 5. Discrepancies
        gz_base_pos  = np.array(tf_base_wrist[:3])  if tf_base_wrist  else None
        gz_world_pos = np.array(tf_world_wrist[:3]) if tf_world_wrist else None

        d_gz_urdf_base  = float(np.linalg.norm(gz_base_pos  - urdf_base))  if gz_base_pos  is not None else float("nan")
        d_gz_urdf_world = float(np.linalg.norm(gz_world_pos - urdf_world)) if gz_world_pos is not None else float("nan")

        if gz_base_pos is not None:
            print(f"  GZ wrist/base_link: ({gz_base_pos[0]*1000:.1f}, {gz_base_pos[1]*1000:.1f}, {gz_base_pos[2]*1000:.1f}) mm")
            print(f"  URDF wrist/base_link:({urdf_base[0]*1000:.1f}, {urdf_base[1]*1000:.1f}, {urdf_base[2]*1000:.1f}) mm")
            print(f"  Δ(GZ−URDF)/base = {d_gz_urdf_base*1000:.2f} mm")
        else:
            print("  [WARN] TF base_link→wrist_link failed")

        if gz_world_pos is not None:
            print(f"  Δ(GZ−URDF)/world = {d_gz_urdf_world*1000:.2f} mm")

        # Joint-angle mapping: commanded vs Gazebo-reported
        q_gain = {}
        for k, n in enumerate(JOINT_ORDER[:4]):
            cmd_v = float(q5[k])
            rep_v = gz_q[n]
            if not math.isnan(rep_v) and abs(cmd_v) > 0.05:
                q_gain[n] = rep_v / cmd_v
            else:
                q_gain[n] = float("nan")

        row = {
            "trial_id":     f"TRIAL_{trial_idx:02d}",
            "seed":         SEED,
            "commit_hash":  COMMIT,
            "timestamp":    ts,
            # Commanded
            "cmd_waist":       float(q5[0]),
            "cmd_shoulder":    float(q5[1]),
            "cmd_elbow":       float(q5[2]),
            "cmd_wrist_pitch": float(q5[3]),
            "cmd_gripper":     float(q5[4]),
            # Gazebo reported
            "gz_waist":        gz_q["waist_joint"],
            "gz_shoulder":     gz_q["shoulder_joint"],
            "gz_elbow":        gz_q["elbow_joint"],
            "gz_wrist_pitch":  gz_q["wrist_pitch_joint"],
            "gz_gripper":      gz_q["gripper_joint"],
            # Joint error
            "q_err_waist":       abs(q5[0] - gz_q["waist_joint"])       if not math.isnan(gz_q["waist_joint"]) else float("nan"),
            "q_err_shoulder":    abs(q5[1] - gz_q["shoulder_joint"])    if not math.isnan(gz_q["shoulder_joint"]) else float("nan"),
            "q_err_elbow":       abs(q5[2] - gz_q["elbow_joint"])       if not math.isnan(gz_q["elbow_joint"]) else float("nan"),
            "q_err_wrist_pitch": abs(q5[3] - gz_q["wrist_pitch_joint"]) if not math.isnan(gz_q["wrist_pitch_joint"]) else float("nan"),
            # URDF FK (base frame)
            "urdf_base_x_m":  float(urdf_base[0]),
            "urdf_base_y_m":  float(urdf_base[1]),
            "urdf_base_z_m":  float(urdf_base[2]),
            # URDF FK (world frame)
            "urdf_world_x_m": float(urdf_world[0]),
            "urdf_world_y_m": float(urdf_world[1]),
            "urdf_world_z_m": float(urdf_world[2]),
            # Gazebo TF (base frame)
            "gz_base_x_m":  tf_base_wrist[0]  if tf_base_wrist  else float("nan"),
            "gz_base_y_m":  tf_base_wrist[1]  if tf_base_wrist  else float("nan"),
            "gz_base_z_m":  tf_base_wrist[2]  if tf_base_wrist  else float("nan"),
            # Gazebo TF (world frame)
            "gz_world_x_m": tf_world_wrist[0] if tf_world_wrist else float("nan"),
            "gz_world_y_m": tf_world_wrist[1] if tf_world_wrist else float("nan"),
            "gz_world_z_m": tf_world_wrist[2] if tf_world_wrist else float("nan"),
            # TF quaternion (base frame)
            "gz_qx": tf_base_wrist[3] if tf_base_wrist else float("nan"),
            "gz_qy": tf_base_wrist[4] if tf_base_wrist else float("nan"),
            "gz_qz": tf_base_wrist[5] if tf_base_wrist else float("nan"),
            "gz_qw": tf_base_wrist[6] if tf_base_wrist else float("nan"),
            # Discrepancies
            "d_gz_urdf_base_mm":  d_gz_urdf_base * 1000  if not math.isnan(d_gz_urdf_base)  else float("nan"),
            "d_gz_urdf_world_mm": d_gz_urdf_world * 1000 if not math.isnan(d_gz_urdf_world) else float("nan"),
            # Joint gain (reported/commanded)
            "gain_waist":       q_gain.get("waist_joint", float("nan")),
            "gain_shoulder":    q_gain.get("shoulder_joint", float("nan")),
            "gain_elbow":       q_gain.get("elbow_joint", float("nan")),
            "gain_wrist_pitch": q_gain.get("wrist_pitch_joint", float("nan")),
        }
        rows.append(row)
        print()

    # ── Write CSV ─────────────────────────────────────────────────────────────
    if rows:
        with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"Raw data → {OUT_CSV}")

    # ── Summary ───────────────────────────────────────────────────────────────
    valid = [r for r in rows if not math.isnan(r.get("d_gz_urdf_base_mm", float("nan")))]
    n_valid = len(valid)

    print("\n" + "═"*78)
    print(f"SUMMARY — {n_valid}/{len(rows)} trials with valid TF reads")
    print("═"*78)

    summary = {"commit_hash": COMMIT, "seed": SEED, "timestamp": TIMESTAMP_TAG,
                "n_trials": len(rows), "n_tf_valid": n_valid}

    if valid:
        d_base  = np.array([r["d_gz_urdf_base_mm"]  for r in valid])
        d_world = np.array([r["d_gz_urdf_world_mm"] for r in valid if not math.isnan(r.get("d_gz_urdf_world_mm", float("nan")))])

        print(f"  Δ(Gazebo−URDF) [base_link frame]:")
        print(f"    max  = {np.max(d_base):.2f} mm")
        print(f"    mean = {np.mean(d_base):.2f} mm")
        print(f"    std  = {np.std(d_base):.2f} mm")
        if len(d_world) > 0:
            print(f"  Δ(Gazebo−URDF) [world frame]:")
            print(f"    max  = {np.max(d_world):.2f} mm")
            print(f"    mean = {np.mean(d_world):.2f} mm")

        # Joint-angle mapping
        print("\n  Joint-angle mapping (Gazebo-reported / commanded):")
        for k, n in enumerate(["gain_waist", "gain_shoulder", "gain_elbow", "gain_wrist_pitch"]):
            gains = [r[n] for r in rows if not math.isnan(r.get(n, float("nan")))]
            if gains:
                print(f"    {n.replace('gain_',''):<15}: mean gain = {np.mean(gains):.4f}  "
                      f"std = {np.std(gains):.4f}  N={len(gains)}")

        # Joint tracking error
        print("\n  Joint tracking error (|cmd − Gazebo|):")
        for k, n in enumerate(["q_err_waist","q_err_shoulder","q_err_elbow","q_err_wrist_pitch"]):
            errs = [r[n] for r in rows if not math.isnan(r.get(n, float("nan")))]
            if errs:
                print(f"    {n.replace('q_err_',''):<15}: mean={np.mean(errs)*1000:.2f} mm_eq  "
                      f"max={np.max(errs)*1000:.2f} mm_eq")

        summary["d_gz_urdf_base_mm"] = {
            "max": float(np.max(d_base)), "mean": float(np.mean(d_base)), "std": float(np.std(d_base))
        }
        if len(d_world):
            summary["d_gz_urdf_world_mm"] = {
                "max": float(np.max(d_world)), "mean": float(np.mean(d_world))
            }

        best_match = "URDF_FK" if np.mean(d_base) < 15.0 else "MISMATCH"
        summary["best_match_to_gazebo"] = best_match
        print(f"\n  ★ URDF FK match to Gazebo: {'GOOD (<15mm)' if best_match == 'URDF_FK' else 'POOR (>15mm)'}")
        print(f"    (mean Δ = {np.mean(d_base):.2f} mm)")
    else:
        print("  [BLOCKER] No valid TF reads. Cannot validate kinematic model vs Gazebo.")

    summary["raw_csv"] = OUT_CSV
    with open(OUT_SUMMARY, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\nSummary JSON → {OUT_SUMMARY}")
    print("═"*78)
    print("DONE")

if __name__ == "__main__":
    main()
