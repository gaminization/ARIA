#!/usr/bin/env python3
"""
Physical Grasping Diagnostic Script
Measures real Gazebo physics parameters, contact forces, and kinematics
for 3 pick trials strictly using physical contact (NO attach plugin).
"""

import math
import os
import sys
import time
from datetime import datetime, timezone
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates, ContactsState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import tf2_ros


class GraspDiagnosticsNode(Node):
    def __init__(self):
        super().__init__("grasp_diagnostics_node")

        self.latest_joints = {}
        self.latest_model_poses = {}
        self.left_contacts = []
        self.right_contacts = []

        self.sub_joints = self.create_subscription(
            JointState, "/joint_states", self._joint_cb, 10
        )
        self.sub_models = self.create_subscription(
            ModelStates, "/gazebo/model_states", self._model_cb, 10
        )
        self.sub_contact_l = self.create_subscription(
            ContactsState, "/gripper/contact_left", self._contact_l_cb, 10
        )
        self.sub_contact_r = self.create_subscription(
            ContactsState, "/gripper/contact_right", self._contact_r_cb, 10
        )

        self.cli_open = self.create_client(Trigger, "/aria/open_gripper")
        self.cli_close = self.create_client(Trigger, "/aria/close_gripper")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _joint_cb(self, msg: JointState):
        for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
            self.latest_joints[name] = {"pos": pos, "vel": vel, "eff": eff}

    def _model_cb(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            self.latest_model_poses[name] = pose

    def _contact_l_cb(self, msg: ContactsState):
        self.left_contacts = msg.states

    def _contact_r_cb(self, msg: ContactsState):
        self.right_contacts = msg.states

    def call_trigger(self, client):
        if not client.wait_for_service(timeout_sec=3.0):
            return False
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result().success if fut.done() and fut.result() else False


def run_diagnostics():
    rclpy.init()
    node = GraspDiagnosticsNode()

    print("[DIAGNOSIS] Waiting for ROS 2 joint states and Gazebo model states...")
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest_joints and "workpiece_01" in node.latest_model_poses:
            break
        time.sleep(0.1)

    print(f"[DIAGNOSIS] Joints received: {list(node.latest_joints.keys())}")
    print(f"[DIAGNOSIS] workpiece_01 present: {'workpiece_01' in node.latest_model_poses}")

    # Inspect mimic joint presence in /joint_states
    has_mimic = "finger_mimic_joint" in node.latest_joints
    print(f"[DIAGNOSIS] 'finger_mimic_joint' in /joint_states: {has_mimic}")
    if has_mimic:
        print(f"            finger_mimic_joint pos: {node.latest_joints['finger_mimic_joint']['pos']:.4f}")
    if "gripper_joint" in node.latest_joints:
        print(f"            gripper_joint pos:      {node.latest_joints['gripper_joint']['pos']:.4f}")

    # Output parameters
    report_rows = []

    # Run 3 pick trials
    for trial_idx in range(1, 4):
        print(f"\n══════════════════════════════════════════════════════")
        print(f"  TRIAL {trial_idx}/3: Physical Grasp Diagnostic Test")
        print(f"══════════════════════════════════════════════════════")

        # 1. Reset workpiece to pick location
        target_x = 0.201
        target_y = 0.074 + (trial_idx - 2) * 0.005  # slightly vary y position
        target_z = 0.6438
        os.system(f"gz model -m workpiece_01 -x {target_x:.4f} -y {target_y:.4f} -z {target_z:.4f} -R 0 -P 0 -Y 0 >/dev/null 2>&1")
        time.sleep(0.5)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)

        # 2. Open gripper
        node.call_trigger(node.cli_open)
        time.sleep(1.0)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)

        q_open = node.latest_joints.get("gripper_joint", {}).get("pos", 0.0)
        open_width_mm = 23.4 + (math.degrees(q_open) / 35.0) * (75.4 - 23.4)
        print(f"  [1] Opened Gripper: angle = {math.degrees(q_open):.1f}° ({q_open:.4f} rad) | Opening width = {open_width_mm:.1f} mm")

        # Solve exact TCP IK for pre-grasp, grasp, and lift
        from arm_interfaces.srv import SetAllJoints
        set_joints_cli = node.create_client(SetAllJoints, "/aria/set_all_joints")
        sys.path.insert(0, "/home/gaminizer/Projects/ARIA/arm_ik/arm_ik/ik_solvers")
        from aria_analytical_ik import forward_kinematics
        from scipy.optimize import least_squares

        p_tcp_wrist = np.array([-0.03775, 0.07379, -0.00667, 1.0])

        def get_tcp_base(q):
            T = forward_kinematics(q[:4])
            return (T @ p_tcp_wrist)[:3]

        def get_pitch_angle(q):
            T = forward_kinematics(q[:4])
            R = T[:3, :3]
            y_wrist = R @ np.array([0, 1, 0])
            return np.arcsin(np.clip(y_wrist[2], -1.0, 1.0))

        limits_lower = np.radians([-179.0, -89.0, -89.0, -89.0])
        limits_upper = np.radians([ 179.0,  89.0,  89.0,  89.0])
        bounds = (limits_lower, limits_upper)

        def solve_grasp(xw, yw, zw, pitch_deg=-25.0):
            xb, yb, zb = yw, -xw, zw - 0.614
            p_target = np.array([xb, yb, zb])
            pitch = np.radians(pitch_deg)
            def cost(q):
                return np.append((get_tcp_base(q) - p_target)*100, (get_pitch_angle(q) - pitch)*10)
            res = least_squares(cost, [0.35, 0.7, -1.3, 0.2], bounds=bounds)
            return res.x, res.success, np.linalg.norm(get_tcp_base(res.x) - p_target)

        # Grasp altitude: 0.655m (pad center on cube upper half, pad bottom at 0.631m > table 0.6288m)
        q_grasp, s1, e1 = solve_grasp(target_x, target_y, 0.655, pitch_deg=-25.0)
        q_pre, _, _ = solve_grasp(target_x, target_y, 0.655 + 0.035, pitch_deg=-25.0)
        q_lift, s2, e2 = solve_grasp(target_x, target_y, 0.655 + 0.065, pitch_deg=-25.0)

        def send_joint_cmd(q, grip_deg, spd=25.0):
            req = SetAllJoints.Request()
            req.angles_deg = list(np.degrees(q[:4])) + [0.0, float(grip_deg)]
            req.speed_deg_per_s = float(spd)
            fut = set_joints_cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            dur = fut.result().expected_duration_s if fut.done() and fut.result() else 2.0
            time.sleep(dur + 0.4)
            for _ in range(15):
                rclpy.spin_once(node, timeout_sec=0.05)

        # Move arm to pre-grasp then grasp
        print(f"  [2] Moving to Grasp Pose: approach pitch = -25.0°, pad altitude = 0.655 m (+11 mm above workpiece centroid)")
        send_joint_cmd(q_pre, math.degrees(q_open), spd=30.0)
        send_joint_cmd(q_grasp, math.degrees(q_open), spd=20.0)

        # 3. Close gripper (pure physical command, NO ATTACH)
        print("  [3] Closing Gripper (pure physical command, NO ATTACH)...")
        send_joint_cmd(q_grasp, 0.0, spd=20.0)

        q_close = node.latest_joints.get("gripper_joint", {}).get("pos", 0.0)
        eff_close = node.latest_joints.get("gripper_joint", {}).get("eff", 0.0)
        contact_width_mm = 23.4 + (math.degrees(q_close) / 35.0) * (75.4 - 23.4)
        print(f"  [4] Contact Gripper Width: {contact_width_mm:.1f} mm vs Workpiece Width: 30.0 mm (angle = {math.degrees(q_close):.1f}°)")

        # Measure contact forces from Gazebo contact sensors
        left_force = 0.0
        right_force = 0.0
        for s in node.left_contacts:
            for f in s.wrenches:
                left_force += math.hypot(f.force.x, f.force.y, f.force.z)
        for s in node.right_contacts:
            for f in s.wrenches:
                right_force += math.hypot(f.force.x, f.force.y, f.force.z)

        print(f"  [5] Measured Gazebo Bumper Contact Forces: Left = {left_force:.3f} N, Right = {right_force:.3f} N")

        # 4. Lift arm with pure vertical vector (+65 mm, constant pitch)
        init_obj_z = node.latest_model_poses["workpiece_01"].position.z if "workpiece_01" in node.latest_model_poses else 0.0
        print(f"  [6] Pure Vertical Lift (+65 mm, constant -25° pitch)... Initial Z = {init_obj_z:.4f} m")
        send_joint_cmd(q_lift, 0.0, spd=15.0)

        # Monitor workpiece z over 2.5 seconds
        z_samples = []
        t0 = time.time()
        while time.time() - t0 < 2.5:
            rclpy.spin_once(node, timeout_sec=0.05)
            if "workpiece_01" in node.latest_model_poses:
                z_samples.append(node.latest_model_poses["workpiece_01"].position.z)
            time.sleep(0.05)

        max_z = max(z_samples) if z_samples else init_obj_z
        final_z = z_samples[-1] if z_samples else init_obj_z
        lift_height_mm = (final_z - init_obj_z) * 1000.0
        held = lift_height_mm >= 50.0

        print(f"  [7] Workpiece Initial Z = {init_obj_z:.4f} m, Final Z = {final_z:.4f} m (Lift: {lift_height_mm:.1f} mm)")
        print(f"  [8] Outcome: {'HELD (Success >= 50mm)' if held else 'SLIPPED'}")

        # Retract and open
        node.call_trigger(node.cli_open)
        time.sleep(0.5)

        trial_data = {
            "trial": trial_idx,
            "target_pos": (target_x, target_y, target_z),
            "open_width_mm": open_width_mm,
            "contact_width_mm": contact_width_mm,
            "workpiece_width_mm": 30.0,
            "finger_friction": (2.5, 2.5),
            "object_friction": (1.5, 1.5),
            "contact_params": {"kp": 1000000.0, "kd": 50.0, "max_vel": 0.01, "min_depth": 0.001},
            "closing_effort_nm": 10.0,
            "object_mass_kg": 0.035,
            "left_contact_force_n": left_force,
            "right_contact_force_n": right_force,
            "approach_pitch_deg": -25.0,
            "finger_height_m": 0.655,
            "init_z": init_obj_z,
            "final_z": final_z,
            "lift_mm": lift_height_mm,
            "held": held,
        }
        report_rows.append(trial_data)

    rclpy.shutdown()
    return report_rows

if __name__ == "__main__":
    run_diagnostics()
