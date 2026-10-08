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
        target_z = 0.644
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
        print(f"  [1] Opened Gripper: angle = {math.degrees(q_open):.1f}° ({q_open:.4f} rad)")

        # Move arm to grasp pose over workpiece:
        # In manual_control_node: conveyor_pick pose is [20.0, 78.0, -18.0, -28.0]
        # Let's use service /aria/set_all_joints
        from arm_interfaces.srv import SetAllJoints
        set_joints_cli = node.create_client(SetAllJoints, "/aria/set_all_joints")
        if set_joints_cli.wait_for_service(timeout_sec=2.0):
            req = SetAllJoints.Request()
            # Descend to workpiece body: approach pitch ~25°, finger height
            req.angles_deg = [20.0, 78.0, -18.0, -28.0, float(math.degrees(q_open))]
            req.speed_deg_per_s = 25.0
            fut = set_joints_cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            time.sleep(1.5)
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)

        # Record TF EE pose and workpiece pose
        wp_pose = node.latest_model_poses.get("workpiece_01", None)
        wp_pos = (wp_pose.position.x, wp_pose.position.y, wp_pose.position.z) if wp_pose else (0, 0, 0)

        # 3. Close gripper (NO ATTACH)
        print("  [2] Closing Gripper (pure physical command, NO ATTACH)...")
        req = SetAllJoints.Request()
        req.angles_deg = [20.0, 78.0, -18.0, -28.0, 0.0]  # Command closed
        req.speed_deg_per_s = 20.0
        fut = set_joints_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        time.sleep(1.5)
        for _ in range(15):
            rclpy.spin_once(node, timeout_sec=0.05)

        q_close = node.latest_joints.get("gripper_joint", {}).get("pos", 0.0)
        eff_close = node.latest_joints.get("gripper_joint", {}).get("eff", 0.0)
        print(f"  [3] Closed Gripper: angle = {math.degrees(q_close):.1f}° ({q_close:.4f} rad), effort = {eff_close:.3f}")

        # Measure contact forces from contact sensors
        left_force = 0.0
        right_force = 0.0
        for s in node.left_contacts:
            for f in s.wrenches:
                left_force += math.hypot(f.force.x, f.force.y, f.force.z)
        for s in node.right_contacts:
            for f in s.wrenches:
                right_force += math.hypot(f.force.x, f.force.y, f.force.z)

        print(f"  [4] Contact forces: Left = {left_force:.3f} N, Right = {right_force:.3f} N")

        # 4. Lift arm by commanding shoulder up
        print("  [5] Lifting arm (Shoulder: 78° -> 50°)...")
        init_obj_z = node.latest_model_poses["workpiece_01"].position.z if "workpiece_01" in node.latest_model_poses else 0.0
        req = SetAllJoints.Request()
        req.angles_deg = [20.0, 50.0, -18.0, -28.0, 0.0]
        req.speed_deg_per_s = 15.0
        fut = set_joints_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)

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
        slipped = lift_height_mm < 50.0

        print(f"  [6] Initial Z = {init_obj_z:.4f} m, Final Z = {final_z:.4f} m (Lift: {lift_height_mm:.1f} mm)")
        print(f"  [7] Outcome: {'SLIPPED (Lift < 50mm)' if slipped else 'HELD'}")

        trial_data = {
            "trial": trial_idx,
            "target_pos": (target_x, target_y, target_z),
            "q_open_deg": math.degrees(q_open),
            "q_close_deg": math.degrees(q_close),
            "closing_effort": eff_close,
            "left_contact_n": left_force,
            "right_contact_n": right_force,
            "init_z": init_obj_z,
            "final_z": final_z,
            "lift_mm": lift_height_mm,
            "slipped": slipped,
        }
        report_rows.append(trial_data)

    rclpy.shutdown()
    return report_rows

if __name__ == "__main__":
    run_diagnostics()
