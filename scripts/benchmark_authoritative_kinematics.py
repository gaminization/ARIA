#!/usr/bin/env python3
"""
scripts/benchmark_authoritative_kinematics.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PROJECT ARIA — AUTHORITATIVE KINEMATIC MODEL VALIDATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

PHASES:
  Phase A: URDF FK derivation — parse joint transforms directly from URDF,
           compute wrist_link + camera TCP positions for 20 deterministic
           joint configs, and print per-joint DH-equivalent parameters.
  Phase B: Gazebo TF validation — command each of 20 joints in Gazebo,
           wait for settle, read link poses from /tf, compare to URDF FK.
  Phase C: arm_ik FK comparison — run arm_ik forward_kinematics on same 20
           configs, quantify deviation from URDF ground truth.
  Phase D: Craig FK comparison — same 20 configs, report deviation.
  Phase E: Joint-angle mapping analysis — compare commanded angles to
           Gazebo-reported joint states (sign/offset/scale).
  Phase F: Axis/origin extraction — print per-joint axis, origin, RPY from
           URDF and derived DH parameters (a, d, alpha, theta_offset).

HARD RULES:
  - No sampled outcomes. All numbers from real Gazebo/TF reads.
  - Logs every raw trial row to data/real/
  - Never overwrites existing files.
  - Seeds only for choosing the 20 test configs, never for outcomes.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import os
import sys
import math
import csv
import json
import time
import subprocess
import threading
from datetime import datetime, timezone

import numpy as np

WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
DATA_REAL_DIR = os.path.join(WORKSPACE_ROOT, "data", "real")
TIMESTAMP_TAG = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
OUT_CSV   = os.path.join(DATA_REAL_DIR, f"authoritative_kin_{TIMESTAMP_TAG}.csv")
OUT_SUMMARY = os.path.join(DATA_REAL_DIR, f"authoritative_kin_summary_{TIMESTAMP_TAG}.json")

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
SEED   = 101   # seed only for choosing 20 test joint configs

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 1 — URDF-FAITHFUL FK
#   Exact joint origins and RPY from aria_arm.urdf.xacro.
#   All joints rotate about their local Z-axis after the parent-frame RPY is
#   applied (standard URDF convention: T = Trans(xyz) · Rot(rpy) · Rot_z(q)).
# ─────────────────────────────────────────────────────────────────────────────

def rpy_to_R(roll, pitch, yaw):
    """Convert roll-pitch-yaw (extrinsic XYZ) to 3×3 rotation matrix."""
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
    """
    Build 4×4 homogeneous transform for a URDF revolute joint about local Z.
    T = Trans(xyz) · Rot_rpy(rpy) · Rot_z(q_z)
    """
    T = np.eye(4)
    R_fixed = rpy_to_R(*rpy)
    R_total = R_fixed @ Rz(q_z)
    T[:3, :3] = R_total
    T[:3,  3] = np.array(xyz)
    return T

# Joint parameters exactly from aria_arm.urdf.xacro
#  Joint name:  xyz,                          rpy,                           axis (always Z)
URDF_JOINTS = [
    # fixed_base: world → base_link
    dict(name="fixed_base", xyz=[0.0, 0.0, 0.614], rpy=[0, 0, 1.5708], is_fixed=True),
    # waist_joint: base_link → waist_link
    dict(name="waist_joint",      xyz=[-0.00345, -1e-05, 0.04449], rpy=[0.0, 0.0, 0.0],    is_fixed=False),
    # shoulder_joint: waist_link → upper_arm_link
    dict(name="shoulder_joint",   xyz=[0.00396, 0.01369, 0.03521],  rpy=[1.5708, 0.03778, 1.5708], is_fixed=False),
    # elbow_joint: upper_arm_link → forearm_link
    dict(name="elbow_joint",      xyz=[-7e-05, 0.11689, -0.00792],  rpy=[-0.0013, -3.14159, 0.03778], is_fixed=False),
    # wrist_pitch_joint: forearm_link → wrist_link
    dict(name="wrist_pitch_joint",xyz=[-0.0088, 0.12752, -0.00487], rpy=[-0.01458, 3.14159, 0.0], is_fixed=False),
]

# Additional fixed joints for sensor frames
FIXED_CAMERA_JOINT = dict(
    name="wrist_camera_joint",
    xyz=[0.025, 0.050, -0.007], rpy=[1.5708, 0.0, 2.3358], is_fixed=True
)

def fk_urdf(joints_5dof, end="wrist_link"):
    """
    Compute FK from world frame to wrist_link (or wrist_camera_link).
    joints_5dof = [q_waist, q_shoulder, q_elbow, q_wrist_pitch, q_wrist_roll]
    q_wrist_roll is passive/gripper — wrist_link frame does not include it.
    Returns 4×4 world→end transform.
    """
    q = list(joints_5dof)  # [q1..q4, q5(unused for wrist_link)]
    T = np.eye(4)

    # fixed_base: world → base_link  (fixed, no joint angle)
    T_fb = np.eye(4)
    T_fb[:3, :3] = rpy_to_R(*URDF_JOINTS[0]["rpy"])
    T_fb[:3, 3]  = np.array(URDF_JOINTS[0]["xyz"])
    T = T @ T_fb

    # waist: base_link → waist_link  (q[0])
    T = T @ make_T(URDF_JOINTS[1]["xyz"], URDF_JOINTS[1]["rpy"], q[0])
    # shoulder: waist_link → upper_arm_link  (q[1])
    T = T @ make_T(URDF_JOINTS[2]["xyz"], URDF_JOINTS[2]["rpy"], q[1])
    # elbow: upper_arm_link → forearm_link  (q[2])
    T = T @ make_T(URDF_JOINTS[3]["xyz"], URDF_JOINTS[3]["rpy"], q[2])
    # wrist_pitch: forearm_link → wrist_link  (q[3])
    T = T @ make_T(URDF_JOINTS[4]["xyz"], URDF_JOINTS[4]["rpy"], q[3])

    if end == "wrist_camera_link":
        T_cam = np.eye(4)
        T_cam[:3, :3] = rpy_to_R(*FIXED_CAMERA_JOINT["rpy"])
        T_cam[:3, 3]  = np.array(FIXED_CAMERA_JOINT["xyz"])
        T = T @ T_cam

    return T

def fk_urdf_base_frame(joints_5dof, end="wrist_link"):
    """
    FK from base_link frame (not world) to end link.
    Strips the fixed_base transform.
    """
    q = list(joints_5dof)
    T = np.eye(4)
    T = T @ make_T(URDF_JOINTS[1]["xyz"], URDF_JOINTS[1]["rpy"], q[0])
    T = T @ make_T(URDF_JOINTS[2]["xyz"], URDF_JOINTS[2]["rpy"], q[1])
    T = T @ make_T(URDF_JOINTS[3]["xyz"], URDF_JOINTS[3]["rpy"], q[2])
    T = T @ make_T(URDF_JOINTS[4]["xyz"], URDF_JOINTS[4]["rpy"], q[3])

    if end == "wrist_camera_link":
        T_cam = np.eye(4)
        T_cam[:3, :3] = rpy_to_R(*FIXED_CAMERA_JOINT["rpy"])
        T_cam[:3, 3]  = np.array(FIXED_CAMERA_JOINT["xyz"])
        T = T @ T_cam
    return T

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 2 — arm_ik forward_kinematics (as in aria_analytical_ik.py)
# ─────────────────────────────────────────────────────────────────────────────
D_BASE   = 0.105
L_UPPER  = 0.145
L_FORE   = 0.115
L_WRIST  = 0.055
L_GRIP   = 0.040
L_TOOL   = L_WRIST + L_GRIP   # 0.095

def dh_matrix(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha),  math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])

def fk_arm_ik(joints):
    th1, th2, th3, th4, th5 = joints
    T_base = np.eye(4)
    T_base[2, 3] = D_BASE
    T1 = dh_matrix(th1, 0, 0,       math.pi/2)
    T2 = dh_matrix(th2, 0, L_UPPER, 0)
    T3 = dh_matrix(th3, 0, L_FORE,  0)
    T4 = dh_matrix(th4, 0, L_TOOL,  0)
    T5 = dh_matrix(th5, 0, 0,       0)
    return T_base @ T1 @ T2 @ T3 @ T4 @ T5

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 3 — Craig MDH FK (paper model: Table III)
# ─────────────────────────────────────────────────────────────────────────────
CRAIG_A1 = 0.030
CRAIG_A2 = 0.145
CRAIG_A3 = 0.115
CRAIG_D1 = 0.105
CRAIG_D5 = 0.095

def craig_mdh_matrix(alpha, a, d, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct,    -st,     0,    a   ],
        [st*ca,  ct*ca, -sa,  -d*sa],
        [st*sa,  ct*sa,  ca,   d*ca],
        [0,      0,      0,    1   ]
    ])

def fk_craig(joints):
    q1, q2, q3, q4, q5 = joints
    T01 = craig_mdh_matrix(0,         0,        CRAIG_D1, q1)
    T12 = craig_mdh_matrix(math.pi/2, CRAIG_A1, 0,        q2)
    T23 = craig_mdh_matrix(0,         CRAIG_A2, 0,        q3)
    T34 = craig_mdh_matrix(0,         CRAIG_A3, 0,        q4)
    T45 = craig_mdh_matrix(math.pi/2, 0,        CRAIG_D5, q5)
    return T01 @ T12 @ T23 @ T34 @ T45

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 4 — ROS2 / Gazebo interaction helpers
# ─────────────────────────────────────────────────────────────────────────────

def ros2_cmd(args, timeout=10):
    """Run a ros2 CLI command, return (stdout, stderr, returncode)."""
    r = subprocess.run(
        ["bash", "-c",
         f"source /home/gaminizer/Projects/ARIA/install/setup.bash && {' '.join(args)}"],
        capture_output=True, text=True, timeout=timeout, env=ROS_ENV
    )
    return r.stdout.strip(), r.stderr.strip(), r.returncode

def gazebo_is_ready(timeout=90):
    """Poll until /joint_states appears."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        out, _, _ = ros2_cmd(["ros2 topic list"], timeout=5)
        if "/joint_states" in out or "/aria/joint_states" in out:
            return True
        time.sleep(2)
    return False

def publish_joint_state(joint_names, positions, timeout=5):
    """
    Command joints via ros2 topic pub (single-shot).
    Uses /aria/joint_stream if available, falls back to /joint_states.
    """
    # Build Float64MultiArray message
    pos_str = ", ".join(str(p) for p in positions)
    cmd = (
        f"source /home/gaminizer/Projects/ARIA/install/setup.bash && "
        f"ros2 topic pub -1 /aria/joint_stream std_msgs/msg/Float64MultiArray "
        f"\"{{data: [{pos_str}]}}\" 2>&1"
    )
    r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True,
                       timeout=timeout, env=ROS_ENV)
    return r.returncode == 0

def read_tf_pose(parent_frame, child_frame, timeout=8):
    """
    Read a TF pose via ros2 run tf2_ros tf2_echo.
    Returns (x, y, z, qx, qy, qz, qw) or None on failure.
    """
    cmd = (
        f"source /home/gaminizer/Projects/ARIA/install/setup.bash && "
        f"timeout 5 ros2 run tf2_ros tf2_echo {parent_frame} {child_frame} 2>&1 | head -20"
    )
    r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True,
                       timeout=timeout, env=ROS_ENV)
    out = r.stdout
    # Parse translation
    import re
    trans_m = re.search(r"Translation:\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]", out)
    rot_m   = re.search(r"Rotation:\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]", out)
    if trans_m and rot_m:
        x, y, z = float(trans_m.group(1)), float(trans_m.group(2)), float(trans_m.group(3))
        qx, qy, qz, qw = float(rot_m.group(1)), float(rot_m.group(2)), \
                          float(rot_m.group(3)), float(rot_m.group(4))
        return (x, y, z, qx, qy, qz, qw)
    return None

def read_tf_pose_stamped(parent_frame, child_frame, timeout=8):
    """
    Read a TF pose using ros2 topic echo on /tf_static then /tf.
    Returns (x, y, z) or None.
    """
    # Try tf2_echo first
    pose = read_tf_pose(parent_frame, child_frame, timeout)
    if pose is not None:
        return pose[:3]  # (x, y, z)
    return None

def read_joint_states_once(timeout=8):
    """
    Read /joint_states once.  Returns dict {joint_name: position_rad} or None.
    """
    import re
    cmd = (
        f"source /home/gaminizer/Projects/ARIA/install/setup.bash && "
        f"timeout 5 ros2 topic echo --once /joint_states sensor_msgs/msg/JointState 2>&1"
    )
    r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True,
                       timeout=timeout, env=ROS_ENV)
    out = r.stdout
    # Parse names
    names_m = re.search(r"name:\s*\[(.*?)\]", out, re.DOTALL)
    pos_m   = re.search(r"position:\s*\[(.*?)\]", out, re.DOTALL)
    if names_m and pos_m:
        names = [n.strip().strip("'\"") for n in names_m.group(1).split(",")]
        positions = [float(p.strip()) for p in pos_m.group(1).split(",")]
        return dict(zip(names, positions))
    return None

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 5 — Joint configs for testing (20 deterministic configs)
# ─────────────────────────────────────────────────────────────────────────────

np.random.seed(SEED)
# Joint limits: waist ±π, shoulder/elbow/wrist ±π/2
LIMITS_LOW  = np.array([-math.pi,  -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2])
LIMITS_HIGH = np.array([ math.pi,   math.pi/2,  math.pi/2,  math.pi/2,  math.pi/2])
# 10 random + 10 structured (home, extreme, singularity-adjacent)
_rand_configs = np.random.uniform(LIMITS_LOW, LIMITS_HIGH, size=(10, 5))
_structured = np.array([
    [0.0,    0.0,   0.0,   0.0,   0.0],     # home: all zero
    [0.0,    0.5,  -0.5,   0.0,   0.0],     # elbow-up moderate
    [math.pi/2, 0.3, -0.3, 0.0, 0.0],      # 90-deg waist
    [-math.pi/4, 0.7, -0.7, 0.0, 0.0],     # elbow-up extended
    [math.pi/3,  0.0, 0.0,  0.5, 0.0],     # wrist pitch
    [0.0, -0.5,   0.5,  0.0,  0.0],        # elbow-down
    [math.pi, 0.4, -0.4, 0.1, 0.0],        # back-facing
    [0.0, math.pi/2, -math.pi/2, 0.0, 0.0], # fully folded
    [0.0, 0.0, -1.0, math.pi/2, 0.0],      # wrist-up
    [math.pi/6, 0.3, -0.6, 0.3, 0.0],      # mixed
])
TEST_CONFIGS = np.vstack([_rand_configs, _structured])  # shape (20, 5)

JOINT_NAMES_CTRL = ["waist_joint", "shoulder_joint", "elbow_joint",
                    "wrist_pitch_joint", "wrist_roll_joint"]

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 6 — DH-equivalent parameter extraction from URDF
# ─────────────────────────────────────────────────────────────────────────────

def extract_dh_from_urdf():
    """
    Given the URDF joint transforms, derive DH-equivalent parameters.
    For each joint i: compute the transform from joint i-1 frame to joint i frame
    at q=0, extract the rotation matrix, and identify:
      - z_i_in_parent: z-axis of frame i expressed in frame i-1
      - x_i_in_parent: x-axis of frame i (defines alpha, a)
      - origin_in_parent: joint i origin in frame i-1

    Prints table of (joint, origin_xyz_mm, axis_parent_frame, RPY_deg, derived_a, derived_d, derived_alpha).
    """
    print("\n" + "═"*78)
    print("PHASE F: DH-EQUIVALENT PARAMETERS EXTRACTED FROM URDF")
    print("═"*78)
    print(f"{'Joint':<20} {'Origin (mm)':>30} {'Axis in parent':>20}")
    print("─"*78)

    frames = []  # accumulate T_world_to_joint at q=0

    # world frame
    T_world = np.eye(4)
    frames.append(("world", T_world.copy()))

    # fixed_base
    T_fb = np.eye(4)
    T_fb[:3, :3] = rpy_to_R(*URDF_JOINTS[0]["rpy"])
    T_fb[:3, 3]  = np.array(URDF_JOINTS[0]["xyz"])
    T_world = T_world @ T_fb
    frames.append(("base_link", T_world.copy()))

    joint_entries = URDF_JOINTS[1:]  # 4 revolute joints
    for jd in joint_entries:
        T_joint = make_T(jd["xyz"], jd["rpy"], 0.0)  # q=0
        T_world = T_world @ T_joint
        frames.append((jd["name"], T_world.copy()))

    # Print per-joint info
    for i, jd in enumerate(URDF_JOINTS[1:], 1):
        T_parent = frames[i+1-1]  # parent frame (base_link is index 1)
        T_child  = frames[i+1]

        origin_in_parent = np.array(jd["xyz"])
        R_fixed = rpy_to_R(*jd["rpy"])
        z_axis_in_parent = R_fixed[:, 2]  # local Z (joint axis) expressed in parent

        print(f"  {jd['name']:<20} "
              f"origin=({origin_in_parent[0]*1000:.2f}, "
              f"{origin_in_parent[1]*1000:.2f}, "
              f"{origin_in_parent[2]*1000:.2f}) mm  "
              f"z_axis=({z_axis_in_parent[0]:.3f}, "
              f"{z_axis_in_parent[1]:.3f}, "
              f"{z_axis_in_parent[2]:.3f})")

    # Compute link lengths along the kinematic chain
    print("\n  Zero-configuration TCP (wrist_link) in base_link frame:")
    T0 = fk_urdf_base_frame([0,0,0,0,0], "wrist_link")
    print(f"    position = ({T0[0,3]*1000:.2f}, {T0[1,3]*1000:.2f}, {T0[2,3]*1000:.2f}) mm")
    print(f"    z-axis direction = ({T0[0,2]:.4f}, {T0[1,2]:.4f}, {T0[2,2]:.4f})")
    print(f"    x-axis direction = ({T0[0,0]:.4f}, {T0[1,0]:.4f}, {T0[2,0]:.4f})")

    # Probe link lengths by sweeping individual joints
    print("\n  Link-length probing (change single joint, measure TCP displacement):")
    for j_idx in range(4):
        q_base = [0.0]*5
        q_pos  = [0.0]*5
        q_pos[j_idx] = 0.1   # tiny rotation
        T_base = fk_urdf_base_frame(q_base, "wrist_link")
        T_pos  = fk_urdf_base_frame(q_pos,  "wrist_link")
        disp = np.linalg.norm(T_pos[:3,3] - T_base[:3,3])
        print(f"    joint[{j_idx}] 0.1 rad → wrist_link displacement: {disp*1000:.2f} mm")

    return frames

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 7 — MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    os.makedirs(DATA_REAL_DIR, exist_ok=True)
    for f in [OUT_CSV, OUT_SUMMARY]:
        if os.path.exists(f):
            print(f"[ERROR] Output already exists: {f}  (hard rules: no overwrite)")
            sys.exit(1)

    print("═"*78)
    print("PROJECT ARIA — AUTHORITATIVE KINEMATIC MODEL VALIDATION")
    print(f"  commit={COMMIT}  seed={SEED}  timestamp={TIMESTAMP_TAG}")
    print("═"*78)

    # ── Phase F first (no Gazebo needed): DH extraction ──────────────────────
    frames_at_zero = extract_dh_from_urdf()

    # ── Phase A: URDF FK for all 20 configs ──────────────────────────────────
    print("\n" + "═"*78)
    print("PHASE A: URDF FK — 20 Joint Configurations")
    print("═"*78)
    urdf_results = []
    for i, q in enumerate(TEST_CONFIGS):
        T_w = fk_urdf(q, "wrist_link")
        T_b = fk_urdf_base_frame(q, "wrist_link")
        T_cam = fk_urdf(q, "wrist_camera_link")
        T_cam_b = fk_urdf_base_frame(q, "wrist_camera_link")
        urdf_results.append({
            "config_id": i,
            "q": q.tolist(),
            "wrist_world": T_w[:3,3].tolist(),
            "wrist_base":  T_b[:3,3].tolist(),
            "cam_world":   T_cam[:3,3].tolist(),
            "cam_base":    T_cam_b[:3,3].tolist(),
            "wrist_R_base": T_b[:3,:3].tolist(),
        })
        print(f"  Config {i:02d}: q={[f'{v:.3f}' for v in q]}  "
              f"wrist_base=({T_b[0,3]*1000:.1f},{T_b[1,3]*1000:.1f},{T_b[2,3]*1000:.1f}) mm")

    # ── Phase C: arm_ik FK for all 20 configs ────────────────────────────────
    print("\n" + "═"*78)
    print("PHASE C: arm_ik FK — 20 Joint Configurations")
    print("═"*78)
    arm_ik_results = []
    for i, q in enumerate(TEST_CONFIGS):
        T = fk_arm_ik(q)
        arm_ik_results.append({
            "config_id": i,
            "tool_pos": T[:3,3].tolist(),
        })
        urdf_wrist = np.array(urdf_results[i]["wrist_base"])
        arm_ik_pos = T[:3,3]
        diff = np.linalg.norm(urdf_wrist - arm_ik_pos)
        print(f"  Config {i:02d}: arm_ik=({arm_ik_pos[0]*1000:.1f},{arm_ik_pos[1]*1000:.1f},{arm_ik_pos[2]*1000:.1f}) mm  "
              f"vs URDF wrist_base → Δ={diff*1000:.1f} mm")

    # ── Phase D: Craig FK for all 20 configs ─────────────────────────────────
    print("\n" + "═"*78)
    print("PHASE D: Craig MDH FK — 20 Joint Configurations")
    print("═"*78)
    craig_results = []
    for i, q in enumerate(TEST_CONFIGS):
        T = fk_craig(q)
        craig_results.append({
            "config_id": i,
            "tool_pos": T[:3,3].tolist(),
        })
        urdf_wrist = np.array(urdf_results[i]["wrist_base"])
        craig_pos = T[:3,3]
        diff = np.linalg.norm(urdf_wrist - craig_pos)
        print(f"  Config {i:02d}: craig=({craig_pos[0]*1000:.1f},{craig_pos[1]*1000:.1f},{craig_pos[2]*1000:.1f}) mm  "
              f"vs URDF wrist_base → Δ={diff*1000:.1f} mm")

    # ── Phase B: Gazebo TF validation ────────────────────────────────────────
    print("\n" + "═"*78)
    print("PHASE B: Gazebo TF Validation — Wait for Gazebo ready...")
    print("═"*78)
    gazebo_available = False
    gazebo_rows = []

    if gazebo_is_ready(timeout=120):
        print("  Gazebo is ready. Starting TF validation...")
        gazebo_available = True

        for trial_idx, q in enumerate(TEST_CONFIGS):
            ts = datetime.now(timezone.utc).isoformat()
            q_list = q.tolist()
            print(f"\n  Trial {trial_idx:02d}: commanding q={[f'{v:.3f}' for v in q]}")

            # Command joints
            cmd_ok = publish_joint_state(JOINT_NAMES_CTRL, q_list[:5])
            print(f"    → publish: {'OK' if cmd_ok else 'FAIL'}")

            # Wait for settle
            time.sleep(1.5)

            # Read joint states from Gazebo
            js = read_joint_states_once(timeout=10)
            gz_q = None
            if js:
                gz_q = [js.get(n, float("nan")) for n in JOINT_NAMES_CTRL]
                print(f"    Gazebo JS: {[f'{v:.4f}' for v in gz_q]}")
                q_err = [abs(gz_q[k] - q_list[k]) for k in range(min(4, len(gz_q)))]
                print(f"    Joint error (|cmd-actual|): {[f'{e:.4f}' for e in q_err]} rad")

            # Read wrist_link TF (world frame)
            wrist_tf_world = read_tf_pose("world", "wrist_link")
            # Read wrist_link TF (base_link frame)
            wrist_tf_base  = read_tf_pose("base_link", "wrist_link")
            cam_tf_base    = read_tf_pose("base_link", "wrist_camera_link")

            row = {
                "trial_id": f"TRIAL_{trial_idx:02d}",
                "seed": SEED,
                "commit_hash": COMMIT,
                "timestamp": ts,
                "cmd_q0_rad": q_list[0], "cmd_q1_rad": q_list[1],
                "cmd_q2_rad": q_list[2], "cmd_q3_rad": q_list[3],
                "cmd_q4_rad": q_list[4],
                "gz_q0_rad": gz_q[0] if gz_q else float("nan"),
                "gz_q1_rad": gz_q[1] if gz_q else float("nan"),
                "gz_q2_rad": gz_q[2] if gz_q else float("nan"),
                "gz_q3_rad": gz_q[3] if gz_q else float("nan"),
                "gz_q4_rad": gz_q[4] if gz_q else float("nan"),
                # URDF FK (base frame)
                "urdf_wrist_x_m": urdf_results[trial_idx]["wrist_base"][0],
                "urdf_wrist_y_m": urdf_results[trial_idx]["wrist_base"][1],
                "urdf_wrist_z_m": urdf_results[trial_idx]["wrist_base"][2],
                # Gazebo TF (base frame)
                "gz_wrist_base_x_m": wrist_tf_base[0] if wrist_tf_base else float("nan"),
                "gz_wrist_base_y_m": wrist_tf_base[1] if wrist_tf_base else float("nan"),
                "gz_wrist_base_z_m": wrist_tf_base[2] if wrist_tf_base else float("nan"),
                # Gazebo TF (world frame)
                "gz_wrist_world_x_m": wrist_tf_world[0] if wrist_tf_world else float("nan"),
                "gz_wrist_world_y_m": wrist_tf_world[1] if wrist_tf_world else float("nan"),
                "gz_wrist_world_z_m": wrist_tf_world[2] if wrist_tf_world else float("nan"),
                # arm_ik FK
                "arm_ik_x_m": arm_ik_results[trial_idx]["tool_pos"][0],
                "arm_ik_y_m": arm_ik_results[trial_idx]["tool_pos"][1],
                "arm_ik_z_m": arm_ik_results[trial_idx]["tool_pos"][2],
                # Craig FK
                "craig_x_m": craig_results[trial_idx]["tool_pos"][0],
                "craig_y_m": craig_results[trial_idx]["tool_pos"][1],
                "craig_z_m": craig_results[trial_idx]["tool_pos"][2],
            }

            # Compute discrepancies
            if wrist_tf_base:
                gz_pos = np.array(wrist_tf_base[:3])
                urdf_pos = np.array(urdf_results[trial_idx]["wrist_base"])
                arm_ik_pos = np.array(arm_ik_results[trial_idx]["tool_pos"])
                craig_pos  = np.array(craig_results[trial_idx]["tool_pos"])
                row["diff_gz_urdf_m"]    = float(np.linalg.norm(gz_pos - urdf_pos))
                row["diff_gz_arm_ik_m"]  = float(np.linalg.norm(gz_pos - arm_ik_pos))
                row["diff_gz_craig_m"]   = float(np.linalg.norm(gz_pos - craig_pos))
                print(f"    GZ wrist(base): ({gz_pos[0]*1000:.1f},{gz_pos[1]*1000:.1f},{gz_pos[2]*1000:.1f}) mm")
                print(f"    URDF wrist(base): ({urdf_pos[0]*1000:.1f},{urdf_pos[1]*1000:.1f},{urdf_pos[2]*1000:.1f}) mm")
                print(f"    Δ(GZ-URDF) = {row['diff_gz_urdf_m']*1000:.2f} mm")
                print(f"    Δ(GZ-arm_ik) = {row['diff_gz_arm_ik_m']*1000:.2f} mm")
                print(f"    Δ(GZ-craig) = {row['diff_gz_craig_m']*1000:.2f} mm")
            else:
                row["diff_gz_urdf_m"]   = float("nan")
                row["diff_gz_arm_ik_m"] = float("nan")
                row["diff_gz_craig_m"]  = float("nan")
                print(f"    [WARN] TF read failed for trial {trial_idx}")

            gazebo_rows.append(row)

    else:
        print("  [BLOCKER] Gazebo not ready within 120s — TF comparison skipped.")
        print("  URDF FK, arm_ik FK, and Craig FK comparisons completed above.")

    # ── Write CSV ─────────────────────────────────────────────────────────────
    if gazebo_rows:
        fieldnames = list(gazebo_rows[0].keys())
        with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(gazebo_rows)
        print(f"\n  Raw trial data → {OUT_CSV}")
    else:
        # Write FK-only CSV (no Gazebo columns)
        csv_rows = []
        for i in range(20):
            csv_rows.append({
                "trial_id": f"TRIAL_{i:02d}", "seed": SEED, "commit_hash": COMMIT,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "cmd_q0_rad": TEST_CONFIGS[i][0], "cmd_q1_rad": TEST_CONFIGS[i][1],
                "cmd_q2_rad": TEST_CONFIGS[i][2], "cmd_q3_rad": TEST_CONFIGS[i][3],
                "cmd_q4_rad": TEST_CONFIGS[i][4],
                "urdf_wrist_x_m": urdf_results[i]["wrist_base"][0],
                "urdf_wrist_y_m": urdf_results[i]["wrist_base"][1],
                "urdf_wrist_z_m": urdf_results[i]["wrist_base"][2],
                "arm_ik_x_m": arm_ik_results[i]["tool_pos"][0],
                "arm_ik_y_m": arm_ik_results[i]["tool_pos"][1],
                "arm_ik_z_m": arm_ik_results[i]["tool_pos"][2],
                "craig_x_m": craig_results[i]["tool_pos"][0],
                "craig_y_m": craig_results[i]["tool_pos"][1],
                "craig_z_m": craig_results[i]["tool_pos"][2],
                "diff_gz_urdf_m": float("nan"),
                "diff_gz_arm_ik_m": float("nan"),
                "diff_gz_craig_m": float("nan"),
                "gz_available": False,
            })
        with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(csv_rows[0].keys()))
            writer.writeheader()
            writer.writerows(csv_rows)
        print(f"\n  FK-only trial data → {OUT_CSV}")

    # ── Summary statistics ───────────────────────────────────────────────────
    print("\n" + "═"*78)
    print("SUMMARY: FK MODEL DISCREPANCIES (base_link frame)")
    print("═"*78)

    urdf_wrist_base = np.array([r["wrist_base"] for r in urdf_results])
    arm_ik_pos_arr  = np.array([r["tool_pos"]   for r in arm_ik_results])
    craig_pos_arr   = np.array([r["tool_pos"]    for r in craig_results])

    d_urdf_armik = np.linalg.norm(urdf_wrist_base - arm_ik_pos_arr, axis=1)
    d_urdf_craig = np.linalg.norm(urdf_wrist_base - craig_pos_arr,  axis=1)
    d_armik_craig = np.linalg.norm(arm_ik_pos_arr - craig_pos_arr,  axis=1)

    print(f"  URDF-wrist vs arm_ik FK:   max={np.max(d_urdf_armik)*1000:.1f} mm  mean={np.mean(d_urdf_armik)*1000:.1f} mm")
    print(f"  URDF-wrist vs Craig FK:    max={np.max(d_urdf_craig)*1000:.1f} mm  mean={np.mean(d_urdf_craig)*1000:.1f} mm")
    print(f"  arm_ik FK vs Craig FK:     max={np.max(d_armik_craig)*1000:.1f} mm  mean={np.mean(d_armik_craig)*1000:.1f} mm")

    if gazebo_rows and not all(math.isnan(r.get("diff_gz_urdf_m", float("nan"))) for r in gazebo_rows):
        valid = [r for r in gazebo_rows if not math.isnan(r.get("diff_gz_urdf_m", float("nan")))]
        d_gz_urdf   = np.array([r["diff_gz_urdf_m"]   for r in valid])
        d_gz_armik  = np.array([r["diff_gz_arm_ik_m"] for r in valid])
        d_gz_craig  = np.array([r["diff_gz_craig_m"]  for r in valid])
        print(f"\n  [Gazebo TF validation — N={len(valid)} trials]")
        print(f"  Gazebo vs URDF FK:        max={np.max(d_gz_urdf)*1000:.1f} mm  mean={np.mean(d_gz_urdf)*1000:.1f} mm")
        print(f"  Gazebo vs arm_ik FK:      max={np.max(d_gz_armik)*1000:.1f} mm  mean={np.mean(d_gz_armik)*1000:.1f} mm")
        print(f"  Gazebo vs Craig FK:        max={np.max(d_gz_craig)*1000:.1f} mm  mean={np.mean(d_gz_craig)*1000:.1f} mm")
        best = "URDF" if np.mean(d_gz_urdf) < min(np.mean(d_gz_armik), np.mean(d_gz_craig)) else \
               ("arm_ik" if np.mean(d_gz_armik) < np.mean(d_gz_craig) else "Craig")
        print(f"\n  ★ BEST MATCH TO GAZEBO: {best} FK")

    # ── Coordinate convention analysis ──────────────────────────────────────
    print("\n" + "═"*78)
    print("COORDINATE CONVENTION ANALYSIS (zero config, base_link frame)")
    print("═"*78)
    q0 = [0.0, 0.0, 0.0, 0.0, 0.0]
    T_urdf_0 = fk_urdf_base_frame(q0, "wrist_link")
    T_armik_0 = fk_arm_ik(q0)
    T_craig_0 = fk_craig(q0)

    print(f"  URDF wrist_link at q=0:")
    print(f"    position = ({T_urdf_0[0,3]*1000:.2f}, {T_urdf_0[1,3]*1000:.2f}, {T_urdf_0[2,3]*1000:.2f}) mm")
    print(f"    x-axis   = ({T_urdf_0[0,0]:.4f}, {T_urdf_0[1,0]:.4f}, {T_urdf_0[2,0]:.4f})")
    print(f"    y-axis   = ({T_urdf_0[0,1]:.4f}, {T_urdf_0[1,1]:.4f}, {T_urdf_0[2,1]:.4f})")
    print(f"    z-axis   = ({T_urdf_0[0,2]:.4f}, {T_urdf_0[1,2]:.4f}, {T_urdf_0[2,2]:.4f})")

    print(f"\n  arm_ik TCP at q=0:")
    print(f"    position = ({T_armik_0[0,3]*1000:.2f}, {T_armik_0[1,3]*1000:.2f}, {T_armik_0[2,3]*1000:.2f}) mm")

    print(f"\n  Craig FK TCP at q=0:")
    print(f"    position = ({T_craig_0[0,3]*1000:.2f}, {T_craig_0[1,3]*1000:.2f}, {T_craig_0[2,3]*1000:.2f}) mm")

    # shoulder at q1=pi/2 (straight up)
    q_up = [0.0, math.pi/2, 0.0, 0.0, 0.0]
    T_up = fk_urdf_base_frame(q_up, "wrist_link")
    print(f"\n  URDF wrist_link at shoulder=π/2 (arm up?):")
    print(f"    position = ({T_up[0,3]*1000:.2f}, {T_up[1,3]*1000:.2f}, {T_up[2,3]*1000:.2f}) mm")

    # ── Joint angle mapping: commanded vs reported ───────────────────────────
    print("\n" + "═"*78)
    print("PHASE E: Joint-Angle Mapping (cmd vs Gazebo-reported)")
    print("  (if Gazebo rows available)")
    print("═"*78)
    if gazebo_rows:
        for k, name in enumerate(JOINT_NAMES_CTRL):
            gz_vals = [r.get(f"gz_q{k}_rad", float("nan")) for r in gazebo_rows]
            cmd_vals = [r[f"cmd_q{k}_rad"] for r in gazebo_rows]
            valid_pairs = [(c, g) for c, g in zip(cmd_vals, gz_vals) if not math.isnan(g)]
            if valid_pairs:
                errors = [abs(c - g) for c, g in valid_pairs]
                slopes = [g/c for c, g in valid_pairs if abs(c) > 0.05]
                avg_slope = np.mean(slopes) if slopes else float("nan")
                print(f"  {name:<25} mean|err|={np.mean(errors)*1000:.2f} mm_eq  "
                      f"gain≈{avg_slope:.4f}")

    # ── Save summary JSON ─────────────────────────────────────────────────────
    summary = {
        "commit_hash": COMMIT,
        "seed": SEED,
        "timestamp": TIMESTAMP_TAG,
        "n_configs": 20,
        "gazebo_available": gazebo_available,
        "discrepancies_mm": {
            "urdf_vs_arm_ik": {"max": float(np.max(d_urdf_armik)*1000),
                                "mean": float(np.mean(d_urdf_armik)*1000)},
            "urdf_vs_craig":  {"max": float(np.max(d_urdf_craig)*1000),
                                "mean": float(np.mean(d_urdf_craig)*1000)},
            "arm_ik_vs_craig":{"max": float(np.max(d_armik_craig)*1000),
                                "mean": float(np.mean(d_armik_craig)*1000)},
        },
        "zero_config": {
            "urdf_wrist_base_mm": [T_urdf_0[i,3]*1000 for i in range(3)],
            "arm_ik_tcp_mm":      [T_armik_0[i,3]*1000 for i in range(3)],
            "craig_tcp_mm":       [T_craig_0[i,3]*1000 for i in range(3)],
        },
        "raw_csv": OUT_CSV,
    }
    with open(OUT_SUMMARY, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\n  Summary JSON → {OUT_SUMMARY}")
    print("═"*78)
    print("DONE")
    print("═"*78)

if __name__ == "__main__":
    main()
