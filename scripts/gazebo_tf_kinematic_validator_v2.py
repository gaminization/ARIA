#!/usr/bin/env python3
"""
scripts/gazebo_tf_kinematic_validator_v2.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PROJECT ARIA — Gazebo TF Kinematic Validator v2

Fixes from v1:
  - Correct tf2_echo output parsing ("- Translation: [x, y, z]" format)
  - Correct /joint_states YAML parsing (multi-line list)
  - Use `base_link` as parent frame (confirmed working via tf2_echo)
  - Increase settle time to 3s for trajectory completion

HARD RULES: No sampled outcomes. Everything from real Gazebo reads.
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
SETUP = "source /home/gaminizer/Projects/ARIA/install/setup.bash"

def get_git_commit():
    try:
        r = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                           cwd=WORKSPACE_ROOT, capture_output=True, text=True, check=True)
        return r.stdout.strip()
    except Exception:
        return "unknown"

COMMIT = get_git_commit()
SEED   = 101

# ─────────────────────────────────────────────────────────────────────────────
# URDF FK
# ─────────────────────────────────────────────────────────────────────────────
def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr]
    ])

def Rz(theta):
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([[ct, -st, 0], [st, ct, 0], [0, 0, 1]])

def make_T(xyz, rpy, q_z=0.0):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy) @ Rz(q_z)
    T[:3,  3] = np.array(xyz)
    return T

# URDF joint transforms (exact values from aria_arm.urdf.xacro)
WAIST_J    = ([-0.00345, -1e-05, 0.04449], [0.0, 0.0, 0.0])
SHOULDER_J = ([0.00396, 0.01369, 0.03521],  [1.5708, 0.03778, 1.5708])
ELBOW_J    = ([-7e-05, 0.11689, -0.00792],  [-0.0013, -3.14159, 0.03778])
WRIST_J    = ([-0.0088, 0.12752, -0.00487], [-0.01458, 3.14159, 0.0])

def urdf_fk_base(q4):
    """FK from base_link to wrist_link.  q4 = [waist, shoulder, elbow, wrist_pitch]"""
    T = np.eye(4)
    T = T @ make_T(*WAIST_J,    q4[0])
    T = T @ make_T(*SHOULDER_J, q4[1])
    T = T @ make_T(*ELBOW_J,    q4[2])
    T = T @ make_T(*WRIST_J,    q4[3])
    return T

# ─────────────────────────────────────────────────────────────────────────────
# 20 test configurations (deterministic, same seed as authoritative script)
# ─────────────────────────────────────────────────────────────────────────────
np.random.seed(SEED)
_rand5 = np.random.uniform(
    [-math.pi, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2],
    [ math.pi,  math.pi/2,  math.pi/2,  math.pi/2,  math.pi/2],
    size=(10, 5)
)
_structured5 = np.array([
    [0.0,        0.0,         0.0,          0.0,          0.0],
    [0.0,        0.5,        -0.5,          0.0,          0.0],
    [math.pi/2,  0.3,        -0.3,          0.0,          0.0],
    [-math.pi/4, 0.7,        -0.7,          0.0,          0.0],
    [math.pi/3,  0.0,         0.0,          0.5,          0.0],
    [0.0,       -0.5,         0.5,          0.0,          0.0],
    [math.pi,    0.4,        -0.4,          0.1,          0.0],
    [0.0,        math.pi/2, -math.pi/2,    0.0,          0.0],
    [0.0,        0.0,        -1.0,          math.pi/2,    0.0],
    [math.pi/6,  0.3,        -0.6,          0.3,          0.0],
])
TEST_CONFIGS = np.vstack([_rand5, _structured5])   # (20, 5)

# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 helpers
# ─────────────────────────────────────────────────────────────────────────────
def shell(cmd, timeout=20):
    r = subprocess.run(["bash", "-c", f"{SETUP} && {cmd}"],
                       capture_output=True, text=True, timeout=timeout, env=ROS_ENV)
    return r.stdout, r.stderr, r.returncode

def command_joints(q5, settle_s=3.0):
    """
    Publish to /aria/joint_stream (sensor_msgs/JointState).
    JOINT_NAMES order: [waist, shoulder, elbow, wrist_pitch, gripper]
    """
    names = "['waist_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'gripper_joint']"
    pos   = "[" + ", ".join(f"{p:.6f}" for p in q5) + "]"
    msg   = f"{{name: {names}, position: {pos}, velocity: [], effort: []}}"
    cmd   = f'ros2 topic pub -1 /aria/joint_stream sensor_msgs/msg/JointState "{msg}"'
    out, err, rc = shell(cmd, timeout=15)
    time.sleep(settle_s)
    return rc == 0

def read_joint_states():
    """
    Read /joint_states (YAML multi-line format).
    Returns dict {joint_name: position_rad} or None.
    """
    cmd = "timeout 6 ros2 topic echo --once /joint_states sensor_msgs/msg/JointState 2>&1"
    out, _, _ = shell(cmd, timeout=10)

    # Parse YAML multi-line lists
    # Find 'name:' section then extract '- item' entries
    # Find 'position:' section then extract '- value' entries
    def extract_list(text, key):
        idx = text.find(f"\n{key}:")
        if idx < 0:
            idx = text.find(f"{key}:")
        if idx < 0:
            return None
        # Scan lines after the key header
        lines = text[idx:].split("\n")[1:]
        items = []
        for line in lines:
            stripped = line.strip()
            if stripped.startswith("- "):
                items.append(stripped[2:].strip())
            elif items:  # stop at first non-list line
                break
        return items

    names_list = extract_list(out, "name")
    pos_list   = extract_list(out, "position")
    if names_list and pos_list and len(names_list) == len(pos_list):
        result = {}
        for n, p in zip(names_list, pos_list):
            n = n.strip("'\"")
            try:
                result[n] = float(p)
            except ValueError:
                result[n] = float("nan")
        return result
    return None

def read_tf(parent, child, timeout=10):
    """
    Read TF via tf2_echo. Parse "- Translation: [x, y, z]" format.
    Returns (x, y, z, qx, qy, qz, qw) or None.
    """
    cmd = f"timeout 7 ros2 run tf2_ros tf2_echo {parent} {child} 2>&1 | head -12"
    out, _, _ = shell(cmd, timeout=timeout)

    # tf2_echo ROS2 format:
    # At time 123.456
    # - Translation: [x, y, z]
    # - Rotation: in Quaternion (xyzw) [qx, qy, qz, qw]
    t = re.search(r"- Translation:\s*\[([^\]]+)\]", out)
    q = re.search(r"in Quaternion \(xyzw\)\s*\[([^\]]+)\]", out)
    if t and q:
        try:
            tx, ty, tz = [float(v.strip()) for v in t.group(1).split(",")]
            qx, qy, qz, qw = [float(v.strip()) for v in q.group(1).split(",")]
            return (tx, ty, tz, qx, qy, qz, qw)
        except Exception:
            pass
    return None

def gazebo_ready(timeout=90):
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
            print(f"[ERROR] File exists (no overwrite): {f}")
            sys.exit(1)

    print("═"*78)
    print("PROJECT ARIA — Gazebo TF Kinematic Validator v2")
    print(f"  commit={COMMIT}  seed={SEED}  ts={TIMESTAMP_TAG}")
    print("═"*78)

    if not gazebo_ready(timeout=60):
        print("[BLOCKER] Gazebo not ready. Start with:")
        print("  ros2 launch arm_bringup industrial_workcell.launch.py gui:=false")
        sys.exit(2)

    # Quick sanity: read current TF at rest
    print("\n  Sanity — current wrist_link pose (base_link frame):")
    sanity = read_tf("base_link", "wrist_link")
    if sanity:
        print(f"    Translation: ({sanity[0]*1000:.1f}, {sanity[1]*1000:.1f}, {sanity[2]*1000:.1f}) mm")
    else:
        print("    [WARN] TF not available yet — proceeding anyway")

    print(f"\n  Starting 20-trial TF validation (settle=3.0s per trial)...\n")

    JOINT_ORDER = ["waist_joint", "shoulder_joint", "elbow_joint",
                   "wrist_pitch_joint", "gripper_joint"]
    rows = []

    for trial_idx, q5 in enumerate(TEST_CONFIGS):
        ts = datetime.now(timezone.utc).isoformat()
        q4 = q5[:4]

        print(f"Trial {trial_idx:02d}/19  q=[{', '.join(f'{v:.3f}' for v in q5)}]")

        # 1. Command joints
        ok = command_joints(q5, settle_s=3.0)
        print(f"  publish: {'OK' if ok else 'FAIL'}")

        # 2. Read joint states
        js = read_joint_states()
        gz_q = {n: float("nan") for n in JOINT_ORDER}
        if js:
            for n in JOINT_ORDER:
                gz_q[n] = js.get(n, float("nan"))
            rep = [gz_q[n] for n in JOINT_ORDER[:4]]
            err = [abs(float(q5[k]) - rep[k]) for k in range(4)]
            print(f"  JS: [{', '.join(f'{v:.4f}' for v in rep)}]")
            print(f"  err:[{', '.join(f'{e:.4f}' for e in err)}] rad")
        else:
            print("  [WARN] /joint_states parse failed")

        # 3. TF reads
        tf_base  = read_tf("base_link", "wrist_link")
        tf_world = read_tf("world",     "wrist_link")   # may fail
        tf_cam   = read_tf("base_link", "wrist_camera_link")

        # 4. URDF FK (base frame)
        T_urdf = urdf_fk_base(q4)
        urdf_pos = T_urdf[:3, 3]

        # 5. Discrepancies
        if tf_base:
            gz_pos = np.array(tf_base[:3])
            d_base = float(np.linalg.norm(gz_pos - urdf_pos))
            print(f"  GZ  wrist/base: ({gz_pos[0]*1000:.1f},{gz_pos[1]*1000:.1f},{gz_pos[2]*1000:.1f}) mm")
            print(f"  URDF wrist/base:({urdf_pos[0]*1000:.1f},{urdf_pos[1]*1000:.1f},{urdf_pos[2]*1000:.1f}) mm")
            print(f"  Δ = {d_base*1000:.2f} mm")
        else:
            gz_pos = None
            d_base = float("nan")
            print("  [WARN] TF base_link→wrist_link failed")

        # Rotation error: angle between URDF z-axis and Gazebo z-axis
        rot_err_deg = float("nan")
        if tf_base:
            # Reconstruct GZ rotation matrix from quaternion (xyzw)
            qx, qy, qz, qw = tf_base[3], tf_base[4], tf_base[5], tf_base[6]
            # Quaternion to rotation matrix
            R_gz = np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]
            ])
            R_urdf = T_urdf[:3, :3]
            # Angle between z-axes
            z_gz   = R_gz[:, 2]
            z_urdf = R_urdf[:, 2]
            dot = float(np.clip(np.dot(z_gz, z_urdf), -1, 1))
            rot_err_deg = math.degrees(math.acos(dot))
            print(f"  Rot error (z-axis): {rot_err_deg:.2f}°")

        row = {
            "trial_id": f"TRIAL_{trial_idx:02d}",
            "seed": SEED, "commit_hash": COMMIT, "timestamp": ts,
            # Commanded
            "cmd_waist":       float(q5[0]), "cmd_shoulder":    float(q5[1]),
            "cmd_elbow":       float(q5[2]), "cmd_wrist_pitch": float(q5[3]),
            "cmd_gripper":     float(q5[4]),
            # Gazebo joint states
            "gz_waist":        gz_q["waist_joint"],
            "gz_shoulder":     gz_q["shoulder_joint"],
            "gz_elbow":        gz_q["elbow_joint"],
            "gz_wrist_pitch":  gz_q["wrist_pitch_joint"],
            # Joint tracking errors
            "q_err_waist":    abs(q5[0]-gz_q["waist_joint"])      if not math.isnan(gz_q["waist_joint"]) else float("nan"),
            "q_err_shoulder": abs(q5[1]-gz_q["shoulder_joint"])   if not math.isnan(gz_q["shoulder_joint"]) else float("nan"),
            "q_err_elbow":    abs(q5[2]-gz_q["elbow_joint"])      if not math.isnan(gz_q["elbow_joint"]) else float("nan"),
            "q_err_wrist":    abs(q5[3]-gz_q["wrist_pitch_joint"]) if not math.isnan(gz_q["wrist_pitch_joint"]) else float("nan"),
            # URDF FK
            "urdf_x_m": float(urdf_pos[0]), "urdf_y_m": float(urdf_pos[1]), "urdf_z_m": float(urdf_pos[2]),
            # Gazebo TF (base frame)
            "gz_x_m":  tf_base[0]  if tf_base  else float("nan"),
            "gz_y_m":  tf_base[1]  if tf_base  else float("nan"),
            "gz_z_m":  tf_base[2]  if tf_base  else float("nan"),
            "gz_qx":   tf_base[3]  if tf_base  else float("nan"),
            "gz_qy":   tf_base[4]  if tf_base  else float("nan"),
            "gz_qz":   tf_base[5]  if tf_base  else float("nan"),
            "gz_qw":   tf_base[6]  if tf_base  else float("nan"),
            # Gazebo TF (world frame)
            "gz_world_x_m": tf_world[0] if tf_world else float("nan"),
            "gz_world_y_m": tf_world[1] if tf_world else float("nan"),
            "gz_world_z_m": tf_world[2] if tf_world else float("nan"),
            # Discrepancy
            "d_gz_urdf_mm": d_base * 1000 if not math.isnan(d_base) else float("nan"),
            "rot_err_deg":  rot_err_deg,
            # Camera TF
            "gz_cam_x_m": tf_cam[0] if tf_cam else float("nan"),
            "gz_cam_y_m": tf_cam[1] if tf_cam else float("nan"),
            "gz_cam_z_m": tf_cam[2] if tf_cam else float("nan"),
        }
        rows.append(row)
        print()

    # ── Write CSV ─────────────────────────────────────────────────────────────
    with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Raw data → {OUT_CSV}")

    # ── Summary ───────────────────────────────────────────────────────────────
    valid = [r for r in rows if not math.isnan(r.get("d_gz_urdf_mm", float("nan")))]
    n_valid = len(valid)

    print("\n" + "═"*78)
    print(f"SUMMARY — {n_valid}/{len(rows)} trials with valid TF reads")
    print("═"*78)

    summary = {
        "commit_hash": COMMIT, "seed": SEED, "timestamp": TIMESTAMP_TAG,
        "n_trials": len(rows), "n_tf_valid": n_valid, "raw_csv": OUT_CSV,
    }

    if valid:
        d_arr   = np.array([r["d_gz_urdf_mm"]  for r in valid])
        rot_arr = np.array([r["rot_err_deg"]    for r in valid if not math.isnan(r.get("rot_err_deg", float("nan")))])

        print(f"  Δ(Gazebo−URDF_FK) [base_link, position only]:")
        print(f"    max  = {np.max(d_arr):.2f} mm")
        print(f"    mean = {np.mean(d_arr):.2f} mm")
        print(f"    std  = {np.std(d_arr):.2f} mm")
        print(f"    p95  = {np.percentile(d_arr, 95):.2f} mm")

        if len(rot_arr) > 0:
            print(f"  Rotation error (z-axis angle):")
            print(f"    max  = {np.max(rot_arr):.2f}°")
            print(f"    mean = {np.mean(rot_arr):.2f}°")

        # Joint tracking
        print("\n  Joint tracking error (|cmd − Gazebo|):")
        for k, n in enumerate(["q_err_waist", "q_err_shoulder", "q_err_elbow", "q_err_wrist"]):
            errs = [r[n] for r in rows if not math.isnan(r.get(n, float("nan")))]
            if errs:
                jname = n.replace("q_err_", "")
                print(f"    {jname:<12}: mean={np.mean(errs)*1000:.2f} mrad  max={np.max(errs)*1000:.2f} mrad")

        # Joint gain
        print("\n  Joint gain (Gazebo-reported / commanded):")
        for cmd_n, gz_n in [("cmd_waist","gz_waist"),("cmd_shoulder","gz_shoulder"),
                             ("cmd_elbow","gz_elbow"),("cmd_wrist_pitch","gz_wrist_pitch")]:
            pairs = [(r[cmd_n], r[gz_n]) for r in rows
                     if not math.isnan(r.get(gz_n, float("nan"))) and abs(r[cmd_n]) > 0.1]
            if pairs:
                gains = [g/c for c, g in pairs]
                jname = cmd_n.replace("cmd_","")
                print(f"    {jname:<15}: gain={np.mean(gains):.4f} ± {np.std(gains):.4f}")

        verdict = "URDF_FK" if np.mean(d_arr) < 15.0 else "MISMATCH"
        print(f"\n  ★ VERDICT: Gazebo best matches → {verdict}")
        print(f"    (mean pos Δ = {np.mean(d_arr):.2f} mm, threshold = 15 mm)")

        summary["d_gz_urdf_mm"] = {
            "max": float(np.max(d_arr)), "mean": float(np.mean(d_arr)),
            "std": float(np.std(d_arr)), "p95": float(np.percentile(d_arr, 95))
        }
        if len(rot_arr):
            summary["rot_err_deg"] = {"max": float(np.max(rot_arr)), "mean": float(np.mean(rot_arr))}
        summary["best_match"] = verdict
    else:
        print("  [BLOCKER] Zero valid TF reads.")

    with open(OUT_SUMMARY, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\nSummary → {OUT_SUMMARY}")
    print("═"*78)
    print("DONE")

if __name__ == "__main__":
    main()
