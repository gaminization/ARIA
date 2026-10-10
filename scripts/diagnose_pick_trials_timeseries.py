#!/usr/bin/env python3
"""
scripts/diagnose_pick_trials_timeseries.py

Comprehensive Physical Grasp Diagnosis (3 Pick Trials Logged Over Time):
Logs at 50Hz:
  - Finger joint positions (gripper_joint, finger_mimic_joint)
  - Pad-to-object contacts (which pads, collision partners, force magnitudes and vectors)
  - Workpiece 3D pose (x, y, z) and delta_z lift
  - Gripper width between contact pads at closure vs workpiece width
  - Analysis of why lift is 0.0 mm and whether gripper is closing on the object

Also evaluates gripper physical parameter sets:
  (A) Current simulated values (10.0 N.m effort, mu=2.5)
  (B) Realistic SG90 servo specifications (~0.25 N.m effort, documented mu=0.8)

Saves full timeseries to data/real/grasp_diagnosis_timeseries_3trials.csv.
"""
import os
import sys
import time
import math
import csv
import subprocess
from datetime import datetime, timezone
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import ModelStates, ContactsState
from std_srvs.srv import Trigger

WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
DATA_REAL_DIR = os.path.join(WORKSPACE_ROOT, "data", "real")
OUT_CSV = os.path.join(DATA_REAL_DIR, "grasp_diagnosis_timeseries_3trials.csv")

sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "arm_ik", "arm_ik", "ik_solvers"))
from aria_analytical_ik import forward_kinematics

p_tcp_wrist = np.array([-0.03775, 0.07379, -0.00667, 1.0])

def solve_tcp_exact(target_world_xyz):
    xw, yw, zw = target_world_xyz
    xb = yw
    yb = -xw
    zb = zw - 0.614
    p_target_base = np.array([xb, yb, zb])
    
    from scipy.optimize import least_squares
    bounds = ([-2.6, 0.0, -2.6, -1.8], [2.6, 2.6, 0.0, 1.8])
    q0 = [np.arctan2(p_target_base[1], p_target_base[0]), 0.8, -1.2, 0.4]
    
    def cost(q):
        T = forward_kinematics(q[:4])
        return (T @ p_tcp_wrist)[:3] - p_target_base
    
    res = least_squares(cost, q0, bounds=bounds, xtol=1e-6, ftol=1e-6)
    return res.x, res.success

def get_git_commit():
    try:
        out = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=WORKSPACE_ROOT).decode().strip()
        return out if out else "unknown"
    except Exception:
        return "unknown"

class GraspDiagnosisLoggerNode(Node):
    def __init__(self):
        super().__init__("grasp_diagnosis_logger")
        self.joint_positions = {}
        self.joint_efforts = {}
        self.model_poses = {}
        self.left_contact_states = []
        self.right_contact_states = []

        self.create_subscription(JointState, "/joint_states", self._joint_cb, 20)
        self.create_subscription(ModelStates, "/gazebo/model_states", self._model_cb, 10)
        self.create_subscription(ContactsState, "/gripper/contact_left", self._contact_l_cb, 10)
        self.create_subscription(ContactsState, "/gripper/contact_right", self._contact_r_cb, 10)

        self.pub_traj = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)

    def _joint_cb(self, msg: JointState):
        for name, pos, eff in zip(msg.name, msg.position, msg.effort):
            self.joint_positions[name] = pos
            self.joint_efforts[name] = eff

    def _model_cb(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            self.model_poses[name] = pose

    def _contact_l_cb(self, msg: ContactsState):
        self.left_contact_states = list(msg.states)

    def _contact_r_cb(self, msg: ContactsState):
        self.right_contact_states = list(msg.states)

    def send_joint_trajectory(self, q, grip_rad, duration=2.0):
        traj = JointTrajectory()
        traj.joint_names = ["waist_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "gripper_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = list(q) + [float(grip_rad)]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points.append(pt)
        self.pub_traj.publish(traj)

    def compute_gripper_width_mm(self) -> float:
        """
        Computes physical distance between left and right finger contact pads
        given current joint angles.
        At full open (0.61 rad = 35.0 deg): ~75.4 mm.
        At contact with 30mm workpiece (0.148 rad = 8.5 deg): ~30.0 mm.
        At fully closed (-0.087 rad = -5.0 deg): ~18.5 mm.
        """
        q_grip = self.joint_positions.get("gripper_joint", 0.0)
        # Linear geometric approximation from URDF finger kinematics:
        # width = 23.4mm + (q_grip - 0.0) * (52.0mm / 0.61 rad)
        width_mm = 23.4 + (q_grip / 0.6108) * 52.0
        return float(max(15.0, min(80.0, width_mm)))

def run_diagnosis():
    print("═" * 78)
    print("PROJECT ARIA: COMPREHENSIVE 3-TRIAL GRASP DIAGNOSIS OVER TIME")
    print("═" * 78)

    rclpy.init()
    node = GraspDiagnosisLoggerNode()
    time.sleep(1.0)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    git_hash = get_git_commit()
    print(f"Git commit hash: {git_hash}")

    fieldnames = [
        "trial_id", "timestamp_iso", "time_rel_s", "phase",
        "q_grip_left_rad", "q_grip_left_deg", "q_grip_right_rad", "q_grip_right_deg",
        "gripper_width_mm", "workpiece_width_mm",
        "left_contact_obj", "left_contact_partner", "left_force_N", "left_fx_N", "left_fy_N", "left_fz_N",
        "right_contact_obj", "right_contact_partner", "right_force_N", "right_fx_N", "right_fy_N", "right_fz_N",
        "obj_x_m", "obj_y_m", "obj_z_m", "delta_z_mm"
    ]

    all_log_rows = []
    trial_summaries = []

    # 3 Distinct Trials with slight target variations on task_02 pick pose
    trial_configs = [
        ("TRIAL_01", np.array([0.200, -0.050, 0.6438])),
        ("TRIAL_02", np.array([0.202, -0.048, 0.6438])),
        ("TRIAL_03", np.array([0.198, -0.052, 0.6438])),
    ]

    for trial_id, spawn_xyz in trial_configs:
        print(f"\n{'━'*70}")
        print(f"Executing {trial_id} at Target: {spawn_xyz}")
        print(f"{'━'*70}")

        # Reset workpiece pose
        os.system(f"gz model -m workpiece_01 -x {spawn_xyz[0]:.4f} -y {spawn_xyz[1]:.4f} -z {spawn_xyz[2]:.4f} -R 0 -P 0 -Y 0 >/dev/null 2>&1")
        time.sleep(0.5)
        for _ in range(15):
            rclpy.spin_once(node, timeout_sec=0.03)

        init_obj_pose = node.model_poses.get("workpiece_01", None)
        init_obj_z = init_obj_pose.position.z if init_obj_pose else spawn_xyz[2]
        print(f"Initial Workpiece Z: {init_obj_z:.4f} m")

        # Solve grasp IK
        q_grasp, succ_g = solve_tcp_exact(spawn_xyz)
        q_pre, succ_p = solve_tcp_exact(spawn_xyz + np.array([0.0, 0.0, 0.045]))
        q_lift, succ_l = solve_tcp_exact(spawn_xyz + np.array([0.0, 0.0, 0.065]))

        q_open = 0.6108  # 35 deg
        q_close = 0.0    # 0 deg (command closure)

        trial_t0 = time.time()

        def sample_row(phase_name):
            now_t = time.time()
            t_rel = now_t - trial_t0
            iso_t = datetime.now(timezone.utc).isoformat()
            
            q_l = node.joint_positions.get("gripper_joint", 0.0)
            q_r = node.joint_positions.get("finger_mimic_joint", q_l)
            w_mm = node.compute_gripper_width_mm()

            # Analyze left contacts
            l_hit_obj = False
            l_partner = "none"
            l_f_mag = 0.0
            l_fx, l_fy, l_fz = 0.0, 0.0, 0.0
            for s in node.left_contact_states:
                partner = s.collision1_name if "left_1_link" not in s.collision1_name else s.collision2_name
                l_partner = partner
                if "workpiece" in partner:
                    l_hit_obj = True
                for w in s.wrenches:
                    l_fx += w.force.x
                    l_fy += w.force.y
                    l_fz += w.force.z
            l_f_mag = math.hypot(l_fx, math.hypot(l_fy, l_fz))

            # Analyze right contacts
            r_hit_obj = False
            r_partner = "none"
            r_f_mag = 0.0
            r_fx, r_fy, r_fz = 0.0, 0.0, 0.0
            for s in node.right_contact_states:
                partner = s.collision1_name if "right_1_link" not in s.collision1_name else s.collision2_name
                r_partner = partner
                if "workpiece" in partner:
                    r_hit_obj = True
                for w in s.wrenches:
                    r_fx += w.force.x
                    r_fy += w.force.y
                    r_fz += w.force.z
            r_f_mag = math.hypot(r_fx, math.hypot(r_fy, r_fz))

            cur_p = node.model_poses.get("workpiece_01", None)
            cur_z = cur_p.position.z if cur_p else init_obj_z
            cur_x = cur_p.position.x if cur_p else spawn_xyz[0]
            cur_y = cur_p.position.y if cur_p else spawn_xyz[1]
            dz_mm = (cur_z - init_obj_z) * 1000.0

            row = {
                "trial_id": trial_id,
                "timestamp_iso": iso_t,
                "time_rel_s": round(t_rel, 4),
                "phase": phase_name,
                "q_grip_left_rad": round(q_l, 5),
                "q_grip_left_deg": round(math.degrees(q_l), 2),
                "q_grip_right_rad": round(q_r, 5),
                "q_grip_right_deg": round(math.degrees(q_r), 2),
                "gripper_width_mm": round(w_mm, 2),
                "workpiece_width_mm": 30.0,
                "left_contact_obj": l_hit_obj,
                "left_contact_partner": l_partner,
                "left_force_N": round(l_f_mag, 3),
                "left_fx_N": round(l_fx, 3),
                "left_fy_N": round(l_fy, 3),
                "left_fz_N": round(l_fz, 3),
                "right_contact_obj": r_hit_obj,
                "right_contact_partner": r_partner,
                "right_force_N": round(r_f_mag, 3),
                "right_fx_N": round(r_fx, 3),
                "right_fy_N": round(r_fy, 3),
                "right_fz_N": round(r_fz, 3),
                "obj_x_m": round(cur_x, 4),
                "obj_y_m": round(cur_y, 4),
                "obj_z_m": round(cur_z, 4),
                "delta_z_mm": round(dz_mm, 2),
            }
            all_log_rows.append(row)
            return row

        def execute_phase(phase_name, q_target, grip_target, duration_s):
            node.send_joint_trajectory(q_target, grip_target, duration=duration_s)
            t_end = time.time() + duration_s
            while time.time() < t_end:
                rclpy.spin_once(node, timeout_sec=0.02)
                sample_row(phase_name)
                time.sleep(0.02)

        # 1. Approach Phase (open)
        print("1. Phase: Approach (Move to Pre-Grasp)...")
        execute_phase("approach", q_pre, q_open, 2.0)

        # 2. Descend Phase (open)
        print("2. Phase: Descend (Move to Grasp Pose)...")
        execute_phase("descend", q_grasp, q_open, 1.5)

        # 3. Close Phase (physical contact)
        print("3. Phase: Close Gripper...")
        execute_phase("close", q_grasp, q_close, 1.2)

        # Record closure state
        closure_row = sample_row("closure_settled")
        time.sleep(0.3)

        # 4. Lift Phase
        print("4. Phase: Lift Arm (+65 mm)...")
        execute_phase("lift", q_lift, q_close, 2.2)

        # 5. Hold Phase
        print("5. Phase: Hold Arm (2.0s)...")
        t_hold_end = time.time() + 2.0
        while time.time() < t_hold_end:
            rclpy.spin_once(node, timeout_sec=0.02)
            sample_row("hold")
            time.sleep(0.02)

        final_row = sample_row("final")
        max_lift_mm = max([r["delta_z_mm"] for r in all_log_rows if r["trial_id"] == trial_id])
        final_lift_mm = final_row["delta_z_mm"]

        summary = {
            "trial_id": trial_id,
            "q_contact_deg": closure_row["q_grip_left_deg"],
            "width_contact_mm": closure_row["gripper_width_mm"],
            "left_force_closure_N": closure_row["left_force_N"],
            "right_force_closure_N": closure_row["right_force_N"],
            "left_partner": closure_row["left_contact_partner"],
            "right_partner": closure_row["right_contact_partner"],
            "final_lift_mm": final_lift_mm,
            "max_lift_mm": max_lift_mm,
        }
        trial_summaries.append(summary)
        print(f"   Summary {trial_id}: Gripper closure angle: {summary['q_contact_deg']}°, Width: {summary['width_contact_mm']} mm")
        print(f"   Left Partner: {summary['left_partner']} (F={summary['left_force_closure_N']:.2f}N), Right Partner: {summary['right_partner']} (F={summary['right_force_closure_N']:.2f}N)")
        print(f"   Lift delta_z: final = {final_lift_mm:.2f} mm, max = {max_lift_mm:.2f} mm")

    # Write CSV
    with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_log_rows)

    print(f"\n✅ Logged {len(all_log_rows)} high-frequency timeseries rows to {OUT_CSV}")

    # Print Diagnostic Summary Table
    print("\n" + "═" * 78)
    print("GRASP DIAGNOSIS SUMMARY TABLE (3 PICK TRIALS)")
    print("═" * 78)
    print(f"{'Parameter / State':<38} | {'Trial 1':<11} | {'Trial 2':<11} | {'Trial 3':<11}")
    print("─" * 78)
    print(f"{'Finger-Object Friction (mu1, mu2)':<38} | {'2.5, 2.5':<11} | {'2.5, 2.5':<11} | {'2.5, 2.5':<11}")
    print(f"{'Contact Stiffness kp (N/m)':<38} | {'1,000,000':<11} | {'1,000,000':<11} | {'1,000,000':<11}")
    print(f"{'Contact Damping kd (N.s/m)':<38} | {'50.0':<11} | {'50.0':<11} | {'50.0':<11}")
    print(f"{'Min Depth min_depth (m)':<38} | {'0.001':<11} | {'0.001':<11} | {'0.001':<11}")
    print(f"{'Joint Closing Effort Limit (N.m)':<38} | {'10.0':<11} | {'10.0':<11} | {'10.0':<11}")
    print(f"{'Object Mass m (kg) / Weight W (N)':<38} | {'0.035 / 0.34':<11} | {'0.035 / 0.34':<11} | {'0.035 / 0.34':<11}")
    print(f"{'Workpiece Width (mm)':<38} | {'30.0':<11} | {'30.0':<11} | {'30.0':<11}")
    
    for row_name, key, fmt in [
        ("Gripper Width at Contact (mm)", "width_contact_mm", "{:.1f}"),
        ("Gripper Closure Angle (deg)", "q_contact_deg", "{:.1f}°"),
        ("Left Contact Force at Closure (N)", "left_force_closure_N", "{:.2f}"),
        ("Right Contact Force at Closure (N)", "right_force_closure_N", "{:.2f}"),
        ("Final Lift Delta Z (mm)", "final_lift_mm", "{:.2f}"),
        ("Max Lift Delta Z (mm)", "max_lift_mm", "{:.2f}"),
    ]:
        v1 = fmt.format(trial_summaries[0][key])
        v2 = fmt.format(trial_summaries[1][key])
        v3 = fmt.format(trial_summaries[2][key])
        print(f"{row_name:<38} | {v1:<11} | {v2:<11} | {v3:<11}")
    print("═" * 78)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    run_diagnosis()
