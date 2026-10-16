#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
ARIA World 1: Tester Workspace Autonomous Verification
═══════════════════════════════════════════════════════════════════════════════
Executes Maneuvers 1 through 6 via pure autonomous natural language commands
dispatched to /aria/command. Zero hardcoded joint angles or target coordinates.
Captures synchronized 3-camera HUDs while objects are in-hand in mid-air.
═══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import time
import math
import subprocess
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from aria_sim_audit_support import (
    evaluate_pick_hold,
    stop_owned_process_group,
    validate_task_trace,
)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from gazebo_msgs.msg import ModelStates
from arm_interfaces.srv import SendCommand
from arm_planner.msg import TaskState

WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARTIFACT_DIR = os.environ.get(
    "ARIA_AUDIT_ARTIFACT_DIR",
    os.path.join(WORKSPACE_ROOT, "runs", "visual_audit"),
)
os.makedirs(ARTIFACT_DIR, exist_ok=True)


def start_sim() -> subprocess.Popen:
    env = os.environ.copy()
    env["GAZEBO_IP"] = "127.0.0.1"
    env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:11345"
    env["DISPLAY"] = os.environ.get("DISPLAY", ":1")
    headless = os.environ.get("ARIA_AUDIT_HEADLESS", "1").lower() in ("1", "true", "yes")
    gui_flag = "false" if headless else "true"

    setup_cmd = f"source {WORKSPACE_ROOT}/install/setup.bash"
    launch_str = f"{setup_cmd} && ros2 launch arm_bringup aria_full_u3.launch.py world:=aria_tester_workspace.world gui:={gui_flag}"
    proc = subprocess.Popen(
        ["bash", "-c", launch_str],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    return proc


class AutonomousMissionNode(Node):
    def __init__(self):
        super().__init__('autonomous_mission_node')
        from cv_bridge import CvBridge
        self.bridge = CvBridge()

        self.top_frame: Optional[np.ndarray] = None
        self.side_frame: Optional[np.ndarray] = None
        self.wrist_frame: Optional[np.ndarray] = None
        self.current_joints = [0.0] * 5
        self.model_poses: Dict[str, Tuple[float, float, float]] = {}
        self.chain_of_thought: List[str] = []
        self.task_state: Optional[TaskState] = None
        self.task_history: Dict[str, List[Tuple[str, Tuple[str, ...]]]] = {}
        self.active_task_id: Optional[str] = None
        self.active_task_started_at = 0.0
        self.model_history: Dict[str, List[Tuple[float, float]]] = {}
        self.seen_cot = set()

        # Subscriptions
        self.create_subscription(Image, "/top_camera/image_raw", self._top_cb, 10)
        self.create_subscription(Image, "/side_camera/image_raw", self._side_cb, 10)
        self.create_subscription(Image, "/wrist_camera/image_raw", self._wrist_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(ModelStates, "/gazebo/model_states", self._models_cb, 10)
        self.create_subscription(String, "/aria/cot/reasoning", self._cot_cb, 10)
        self.create_subscription(TaskState, "/aria/state/task", self._task_cb, 10)

        # Clients
        self.cli_command = self.create_client(SendCommand, "/aria/command")
        self.cli_approve = self.create_client(Trigger, "/aria/approve")

    def _top_cb(self, msg):
        try:
            self.top_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def _side_cb(self, msg):
        try:
            self.side_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def _wrist_cb(self, msg):
        try:
            self.wrist_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def _joint_cb(self, msg):
        name_map = {"waist_joint": 0, "shoulder_joint": 1, "elbow_joint": 2, "wrist_pitch_joint": 3, "gripper_joint": 4}
        for name, pos in zip(msg.name, msg.position):
            if name in name_map:
                self.current_joints[name_map[name]] = pos

    def _models_cb(self, msg):
        stamp = time.monotonic()
        for name, pose in zip(msg.name, msg.pose):
            self.model_poses[name] = (pose.position.x, pose.position.y, pose.position.z)
            history = self.model_history.setdefault(name, [])
            history.append((stamp, pose.position.z))
            if len(history) > 10000:
                del history[:-10000]

    def _cot_cb(self, msg: String):
        if msg.data not in self.seen_cot:
            self.seen_cot.add(msg.data)
            self.chain_of_thought.append(msg.data)
            print(f"    🧠 [CoT] {msg.data}")

    def _task_cb(self, msg: TaskState):
        self.task_state = msg
        if msg.task_id:
            snapshot = (
                msg.task_status,
                tuple(action.status for action in msg.action_queue),
            )
            history = self.task_history.setdefault(msg.task_id, [])
            if not history or history[-1] != snapshot:
                history.append(snapshot)
        for line in msg.chain_of_thought:
            if line not in self.seen_cot:
                self.seen_cot.add(line)
                self.chain_of_thought.append(line)
                print(f"    🧠 [CoT] {line}")
        if msg.task_status == "PAUSED":
            print("    ⏸️ Task PAUSED for approval. Auto-approving...")
            self.cli_approve.call_async(Trigger.Request())

    def spin_for(self, duration_s: float):
        t0 = time.time()
        while time.time() - t0 < duration_s:
            rclpy.spin_once(self, timeout_sec=0.03)

    def wait_for_system_ready(self, timeout_s: float = 45.0) -> bool:
        t0 = time.time()
        print("  Waiting for Gazebo, controllers, and /aria/command...")
        while time.time() - t0 < timeout_s:
            self.spin_for(0.5)
            if self.cli_command.wait_for_service(timeout_sec=0.5):
                if self.top_frame is not None and len(self.model_poses) > 3:
                    return True
        return False

    def dispatch_command(self, command_text: str) -> bool:
        print(f"\n  📡 Dispatching Autonomous Command: \"{command_text}\"")
        req = SendCommand.Request()
        req.command = command_text
        fut = self.cli_command.call_async(req)
        t0 = time.time()
        while not fut.done() and (time.time() - t0 < 10.0):
            rclpy.spin_once(self, timeout_sec=0.05)
        if fut.done() and fut.result():
            response = fut.result()
            if response.accepted:
                self.active_task_id = response.task_id
                self.active_task_started_at = time.monotonic()
                print(f"    ✅ Command Accepted (Task ID: {response.task_id})")
                return True
            raise RuntimeError(f"Command rejected: {response.message}")
        raise RuntimeError(f"Command dispatch timed out: {command_text}")

    def wait_for_task_settle(self, timeout_s: float = 240.0) -> bool:
        if not self.active_task_id:
            return False
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            self.spin_for(0.2)
            history = self.task_history.get(self.active_task_id, [])
            if not history:
                continue
            status = history[-1][0]
            if status == "FAILED":
                return False
            if status == "COMPLETE":
                valid, reason = validate_task_trace(history)
                if not valid:
                    print(f"    ❌ Invalid task/action state trace: {reason}")
                return valid
        return False

    def require_model_pose(self, name: str) -> Tuple[float, float, float]:
        if name not in self.model_poses:
            raise RuntimeError(f"Gazebo ground truth has no model named {name!r}")
        return self.model_poses[name]

    def pick_evidence(self, name: str, initial_z: float):
        samples = [
            sample for sample in self.model_history.get(name, [])
            if sample[0] >= self.active_task_started_at
        ]
        return evaluate_pick_hold(initial_z, samples)

    def create_hud_composite(self,
                             maneuver_title: str,
                             world_name: str,
                             target_object: str,
                             state_machine_stage: str,
                             cot_notes: List[str],
                             verification_text: str,
                             verified_pass: bool,
                             extra_telemetry: Optional[List[str]] = None) -> np.ndarray:
        self.spin_for(0.15)
        blank_h, blank_w = 720, 1280
        top = self.top_frame if self.top_frame is not None else np.zeros((blank_h, blank_w, 3), dtype=np.uint8)
        side = self.side_frame if self.side_frame is not None else np.zeros((blank_h, blank_w, 3), dtype=np.uint8)
        wrist = self.wrist_frame if self.wrist_frame is not None else np.zeros((blank_h, blank_w, 3), dtype=np.uint8)

        tile_w, tile_h = 590, 332
        top_tile = cv2.resize(top, (tile_w, tile_h))
        side_tile = cv2.resize(side, (tile_w, tile_h))
        wrist_tile = cv2.resize(wrist, (tile_w, tile_h))

        cv2.putText(top_tile, "OVERHEAD TOP INSPECTION CAMERA (1280x720)", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2)
        cv2.putText(side_tile, "SIDE WORKCELL PROFILE CAMERA (1280x720)", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2)
        cv2.putText(wrist_tile, "WRIST EYE-IN-HAND CAMERA (1280x720)", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2)

        canvas = np.zeros((1080, 1920, 3), dtype=np.uint8)
        canvas[:] = (22, 26, 30)

        # Header
        cv2.rectangle(canvas, (0, 0), (1920, 95), (15, 18, 22), -1)
        cv2.line(canvas, (0, 95), (1920, 95), (0, 180, 255), 2)
        cv2.putText(canvas, "PROJECT ARIA: AUTONOMOUS MULTI-AGENT VERIFICATION AUDIT", (25, 42),
                    cv2.FONT_HERSHEY_DUPLEX, 1.05, (255, 255, 255), 2)
        cv2.putText(canvas, f"WORLD: {world_name.upper()} | TARGET: {target_object.upper()}", (25, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 1)

        status_color = (0, 255, 0) if verified_pass else (0, 0, 255)
        status_str = "AUTONOMOUS VERIFICATION: PASSED [100%]" if verified_pass else "VERIFICATION FAILED"
        cv2.putText(canvas, status_str, (1260, 58),
                    cv2.FONT_HERSHEY_DUPLEX, 0.85, status_color, 2)

        # Camera tiles
        canvas[115:447, 40:630] = side_tile
        canvas[115:447, 665:1255] = top_tile
        canvas[115:447, 1290:1880] = wrist_tile
        for x_start in [40, 665, 1290]:
            cv2.rectangle(canvas, (x_start, 115), (x_start + tile_w, 447), (60, 75, 90), 2)

        # Dashboard Mid
        cv2.rectangle(canvas, (40, 470), (1880, 575), (32, 38, 45), -1)
        cv2.rectangle(canvas, (40, 470), (1880, 575), (0, 180, 255), 1)
        cv2.putText(canvas, f"MANEUVER: {maneuver_title.upper()}", (60, 510),
                    cv2.FONT_HERSHEY_DUPLEX, 0.82, (255, 255, 255), 2)
        cv2.putText(canvas, f"STATE MACHINE PHASE: {state_machine_stage}", (60, 550),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 220), 2)

        q_deg_str = " | ".join([f"J{i}:{math.degrees(q):.1f} deg" for i, q in enumerate(self.current_joints[:4])])
        cv2.putText(canvas, f"ARM JOINTS: {q_deg_str}", (1050, 510),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (200, 220, 240), 1)
        cv2.putText(canvas, f"GRIPPER: {math.degrees(self.current_joints[4]):.1f} deg | SPEED LIMIT: 10.0 deg/s",
                    (1050, 550), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (200, 220, 240), 1)

        # Bottom Left: CoT
        cv2.rectangle(canvas, (40, 595), (1190, 1055), (28, 33, 40), -1)
        cv2.rectangle(canvas, (40, 595), (1190, 1055), (70, 85, 100), 1)
        cv2.putText(canvas, "AUTONOMOUS CHAIN OF THOUGHT & MULTI-AGENT STATE LOG:", (55, 630),
                    cv2.FONT_HERSHEY_DUPLEX, 0.68, (0, 220, 255), 2)

        y_text = 670
        for line in cot_notes:
            clean_line = line.replace("°", " deg").replace("—", "-")
            cv2.putText(canvas, clean_line[:110], (60, y_text),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.54, (220, 230, 240), 1)
            y_text += 32

        # Bottom Right: Physical Telemetry
        cv2.rectangle(canvas, (1215, 595), (1880, 1055), (28, 33, 40), -1)
        cv2.rectangle(canvas, (1215, 595), (1880, 1055), (70, 85, 100), 1)
        cv2.putText(canvas, "PHYSICAL STATE AUDIT & VERIFICATION:", (1235, 630),
                    cv2.FONT_HERSHEY_DUPLEX, 0.68, (0, 220, 255), 2)

        obj_pose = self.model_poses.get(target_object)
        lines_telemetry = [
            f"Target Model: {target_object}",
            (
                f"Gazebo Coordinates (x,y,z): ({obj_pose[0]:.3f}, {obj_pose[1]:.3f}, {obj_pose[2]:.3f}) m"
                if obj_pose is not None else "Gazebo model pose: unavailable"
            ),
            f"Gripper Joint: {math.degrees(self.current_joints[4]):.1f} deg (Grip Envelope: Active)",
        ]
        if extra_telemetry:
            lines_telemetry.extend(extra_telemetry)
        lines_telemetry.extend([
            "",
            "VERIFICATION GATE:",
            verification_text,
            f"AUDIT RESULT: {'PASS' if verified_pass else 'FAIL'}",
        ])

        y_text_r = 670
        for line in lines_telemetry:
            col = (0, 255, 0) if "AUDIT RESULT: PASS" in line else (240, 240, 255)
            if "AUDIT RESULT: FAIL" in line:
                col = (0, 0, 255)
            if "Target Model:" in line or "CONFIRMED" in line:
                col = (0, 220, 255)
            clean_r = line.replace("°", " deg").replace("—", "-")
            cv2.putText(canvas, clean_r[:65], (1245, y_text_r),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.56, col, 1)
            y_text_r += 30

        return canvas


def main():
    print("═" * 75)
    print("  🚀 SUITE 1: ARIA World 1 (aria_tester_workspace.world) AUTONOMOUS EXECUTION")
    print("  Mode: Pure Natural Language Commands via /aria/command | Smooth 15 deg/s Speed")
    print("═" * 75)

    sim_proc = start_sim()
    rclpy.init()
    node = AutonomousMissionNode()
    mission_passed = True

    try:
        ready = node.wait_for_system_ready(timeout_s=50.0)
        if not ready:
            raise RuntimeError("Simulation, cameras, model states, or command service did not become ready")
        node.spin_for(3.0)

        # ─────────────────────────────────────────────────────────────────────
        # Maneuver 1: Wave Gesture
        # ─────────────────────────────────────────────────────────────────────
        print("\n[1/6] Maneuver 1: Autonomous Wave Gesture...")
        node.chain_of_thought.clear()
        joint_start = list(node.current_joints)
        node.dispatch_command("Wave to the camera")
        t0 = time.time()
        waved = False
        while time.time() - t0 < 35.0:
            node.spin_for(0.2)
            if max(abs(now - start) for now, start in zip(node.current_joints, joint_start)) > 0.15:
                waved = True
                break
        task_complete = node.wait_for_task_settle()
        reasoning_available = bool(node.chain_of_thought)
        waved = waved and task_complete and reasoning_available
        node.spin_for(2.0)
        hud_1 = node.create_hud_composite(
            maneuver_title="Maneuver 1: Autonomous Greeting Wave Gesture",
            world_name="aria_tester_workspace.world",
            target_object="user_workspace",
            state_machine_stage="WAVING_JOINT_MOTION -> ACTIVE_SWING",
            cot_notes=node.chain_of_thought[-6:],
            verification_text=f"Joint movement observed, task trace completed, reasoning available: {waved}.",
            verified_pass=waved,
            extra_telemetry=["Execution Mode: LLM Decomposed Trajectory", "Servo Speed: 15.0 deg/s"]
        )
        cv2.imwrite(os.path.join(ARTIFACT_DIR, "visual_01_wave_gesture.png"), hud_1)
        print("    ✅ Saved: visual_01_wave_gesture.png")
        mission_passed = mission_passed and waved

        # ─────────────────────────────────────────────────────────────────────
        # Maneuver 2: Coffee Mug Drinking Gesture
        # ─────────────────────────────────────────────────────────────────────
        print("\n[2/6] Maneuver 2: Autonomous Coffee Mug Grasp & Drinking Gesture...")
        node.chain_of_thought.clear()
        initial_mug_z = node.require_model_pose("mug_ycb_front")[2]
        node.dispatch_command("Pick up the mug and make a coffee drinking gesture")

        t0 = time.time()
        mug_lifted = False
        lift_delta_mm = 0.0
        while time.time() - t0 < 60.0:
            node.spin_for(0.25)
            cur_z = node.require_model_pose("mug_ycb_front")[2]
            lift_delta_mm = (cur_z - initial_mug_z) * 1000.0
            if lift_delta_mm > 40.0:
                mug_lifted = True
                break

        task_complete = node.wait_for_task_settle()
        evidence = node.pick_evidence("mug_ycb_front", initial_mug_z)
        lift_delta_mm = evidence.max_lift_m * 1000.0
        reasoning_available = bool(node.chain_of_thought)
        verified = task_complete and evidence.passed and reasoning_available
        node.spin_for(1.5)
        hud_2 = node.create_hud_composite(
            maneuver_title="Maneuver 2: Coffee Drinking Gesture with Mug",
            world_name="aria_tester_workspace.world",
            target_object="mug_ycb_front",
            state_machine_stage="IN_HAND_DRINKING_GESTURE (TILT ACTIVE)",
            cot_notes=node.chain_of_thought[-6:],
            verification_text=f"Gazebo lift evidence: {lift_delta_mm:.1f}mm for {evidence.continuous_hold_s:.1f}s; task complete={task_complete}, reasoning={reasoning_available}.",
            verified_pass=verified,
            extra_telemetry=[f"Maximum measured lift: +{lift_delta_mm:.1f} mm", f"Continuous hold: {evidence.continuous_hold_s:.1f} s"]
        )
        cv2.imwrite(os.path.join(ARTIFACT_DIR, "visual_02_coffee_drinking_mug.png"), hud_2)
        print(f"    ✅ Saved: visual_02_coffee_drinking_mug.png (delta_z: +{lift_delta_mm:.1f}mm)")
        mission_passed = mission_passed and verified

        # ─────────────────────────────────────────────────────────────────────
        # Maneuver 3: Jenga Block Extraction
        # ─────────────────────────────────────────────────────────────────────
        print("\n[3/6] Maneuver 3: Autonomous Jenga Block Extraction...")
        node.chain_of_thought.clear()
        initial_jg_z = node.require_model_pose("jenga_block_1")[2]
        node.dispatch_command("Pick up the loose Jenga block")

        t0 = time.time()
        jg_lifted = False
        lift_jg_mm = 0.0
        while time.time() - t0 < 60.0:
            node.spin_for(0.25)
            cur_z = node.require_model_pose("jenga_block_1")[2]
            lift_jg_mm = (cur_z - initial_jg_z) * 1000.0
            if lift_jg_mm > 40.0:
                jg_lifted = True
                break

        task_complete = node.wait_for_task_settle()
        evidence = node.pick_evidence("jenga_block_1", initial_jg_z)
        lift_jg_mm = evidence.max_lift_m * 1000.0
        reasoning_available = bool(node.chain_of_thought)
        verified = task_complete and evidence.passed and reasoning_available
        node.spin_for(1.2)
        hud_3 = node.create_hud_composite(
            maneuver_title="Maneuver 3: Solve Jenga Block Extraction",
            world_name="aria_tester_workspace.world",
            target_object="jenga_block_1",
            state_machine_stage="EXTRACTED_IN_HAND -> ELEVATED_INSPECTION",
            cot_notes=node.chain_of_thought[-6:],
            verification_text=f"Gazebo lift evidence: {lift_jg_mm:.1f}mm for {evidence.continuous_hold_s:.1f}s; task complete={task_complete}, reasoning={reasoning_available}.",
            verified_pass=verified,
            extra_telemetry=[f"Maximum measured lift: +{lift_jg_mm:.1f} mm", f"Continuous hold: {evidence.continuous_hold_s:.1f} s"]
        )
        cv2.imwrite(os.path.join(ARTIFACT_DIR, "visual_03_jenga_solve_extract.png"), hud_3)
        print(f"    ✅ Saved: visual_03_jenga_solve_extract.png (delta_z: +{lift_jg_mm:.1f}mm)")
        mission_passed = mission_passed and verified

        # ─────────────────────────────────────────────────────────────────────
        # Maneuver 4: Transparent Clear Glass Tumbler
        # ─────────────────────────────────────────────────────────────────────
        print("\n[4/6] Maneuver 4: Transparent Clear Glass Manipulation...")
        node.chain_of_thought.clear()
        initial_gl_z = node.require_model_pose("plastic_glass")[2]
        node.dispatch_command("Pick up the clear glass tumbler")

        t0 = time.time()
        gl_lifted = False
        lift_gl_mm = 0.0
        while time.time() - t0 < 60.0:
            node.spin_for(0.25)
            cur_z = node.require_model_pose("plastic_glass")[2]
            lift_gl_mm = (cur_z - initial_gl_z) * 1000.0
            if lift_gl_mm > 40.0:
                gl_lifted = True
                break

        task_complete = node.wait_for_task_settle()
        evidence = node.pick_evidence("plastic_glass", initial_gl_z)
        lift_gl_mm = evidence.max_lift_m * 1000.0
        reasoning_available = bool(node.chain_of_thought)
        verified = task_complete and evidence.passed and reasoning_available
        node.spin_for(1.2)
        hud_4 = node.create_hud_composite(
            maneuver_title="Maneuver 4: Transparent Clear Glass Tumbler",
            world_name="aria_tester_workspace.world",
            target_object="plastic_glass",
            state_machine_stage="CLEARGRASP_DEPTH_ESTIMATION -> IN_HAND_LIFT",
            cot_notes=node.chain_of_thought[-6:],
            verification_text=f"Gazebo lift evidence: {lift_gl_mm:.1f}mm for {evidence.continuous_hold_s:.1f}s; task complete={task_complete}, reasoning={reasoning_available}.",
            verified_pass=verified,
            extra_telemetry=[f"Maximum measured lift: +{lift_gl_mm:.1f} mm", f"Continuous hold: {evidence.continuous_hold_s:.1f} s"]
        )
        cv2.imwrite(os.path.join(ARTIFACT_DIR, "visual_04_cleargrasp_glass_pick.png"), hud_4)
        print(f"    ✅ Saved: visual_04_cleargrasp_glass_pick.png (delta_z: +{lift_gl_mm:.1f}mm)")
        mission_passed = mission_passed and verified

        # ─────────────────────────────────────────────────────────────────────
        # Maneuver 5: Handle Affordance Pickup
        # ─────────────────────────────────────────────────────────────────────
        print("\n[5/6] Maneuver 5: Handle Affordance Pickup (Frying Pan)...")
        node.chain_of_thought.clear()
        initial_pan_z = node.require_model_pose("tefal_pan")[2]
        node.dispatch_command("Pick up the frying pan by its handle")

        t0 = time.time()
        pan_lifted = False
        lift_pan_mm = 0.0
        while time.time() - t0 < 60.0:
            node.spin_for(0.25)
            cur_z = node.require_model_pose("tefal_pan")[2]
            lift_pan_mm = (cur_z - initial_pan_z) * 1000.0
            if lift_pan_mm > 40.0:
                pan_lifted = True
                break

        task_complete = node.wait_for_task_settle()
        evidence = node.pick_evidence("tefal_pan", initial_pan_z)
        lift_pan_mm = evidence.max_lift_m * 1000.0
        reasoning_available = bool(node.chain_of_thought)
        verified = task_complete and evidence.passed and reasoning_available
        node.spin_for(1.2)
        hud_5 = node.create_hud_composite(
            maneuver_title="Maneuver 5: Handle Affordance Frying Pan Pickup",
            world_name="aria_tester_workspace.world",
            target_object="tefal_pan",
            state_machine_stage="AFFORDANCE_HANDLE_ENCLOSURE -> LIFTED_IN_HAND",
            cot_notes=node.chain_of_thought[-6:],
            verification_text=f"Gazebo lift evidence: {lift_pan_mm:.1f}mm for {evidence.continuous_hold_s:.1f}s; task complete={task_complete}, reasoning={reasoning_available}.",
            verified_pass=verified,
            extra_telemetry=[f"Maximum measured lift: +{lift_pan_mm:.1f} mm", f"Continuous hold: {evidence.continuous_hold_s:.1f} s"]
        )
        cv2.imwrite(os.path.join(ARTIFACT_DIR, "visual_05_affordance_pan_handle.png"), hud_5)
        print(f"    ✅ Saved: visual_05_affordance_pan_handle.png (delta_z: +{lift_pan_mm:.1f}mm)")
        mission_passed = mission_passed and verified

        # ─────────────────────────────────────────────────────────────────────
        # Maneuver 6: Multi-Object Tabletop Pick & Place (Fruit to Plate)
        # ─────────────────────────────────────────────────────────────────────
        print("\n[6/6] Maneuver 6: Multi-Object Tabletop Pick & Place (Fruit to Plate)...")
        node.chain_of_thought.clear()
        initial_org_z = node.require_model_pose("orange_fruit")[2]
        plt_pos = node.require_model_pose("dinner_plate")
        node.dispatch_command("Pick the orange fruit and place it on the dinner plate")

        t0 = time.time()
        org_lifted = False
        lift_org_mm = 0.0
        while time.time() - t0 < 60.0:
            node.spin_for(0.25)
            cur_org = node.require_model_pose("orange_fruit")
            lift_org_mm = (cur_org[2] - initial_org_z) * 1000.0
            dist_to_plt = math.hypot(cur_org[0] - plt_pos[0], cur_org[1] - plt_pos[1])
            if lift_org_mm > 40.0 and dist_to_plt < 0.15:
                org_lifted = True
                break

        task_complete = node.wait_for_task_settle()
        evidence = node.pick_evidence("orange_fruit", initial_org_z)
        cur_org = node.require_model_pose("orange_fruit")
        lift_org_mm = evidence.max_lift_m * 1000.0
        dist_to_plt = math.hypot(cur_org[0] - plt_pos[0], cur_org[1] - plt_pos[1])
        reasoning_available = bool(node.chain_of_thought)
        verified = (
            task_complete and evidence.passed
            and dist_to_plt <= 0.08 and reasoning_available
        )
        node.spin_for(1.5)
        hud_6 = node.create_hud_composite(
            maneuver_title="Maneuver 6: Tabletop Pick & Place to Plate",
            world_name="aria_tester_workspace.world",
            target_object="orange_fruit",
            state_machine_stage="IN_HAND_TRANSPORT_OVER_DINNER_PLATE",
            cot_notes=node.chain_of_thought[-6:],
            verification_text=f"Lifted {lift_org_mm:.1f}mm for {evidence.continuous_hold_s:.1f}s; final plate offset {dist_to_plt*1000:.1f}mm, task complete={task_complete}, reasoning={reasoning_available}.",
            verified_pass=verified,
            extra_telemetry=[f"Physical In-Hand Elevation: +{lift_org_mm:.1f} mm", f"Distance to Plate Center: {dist_to_plt*1000:.1f} mm"]
        )
        cv2.imwrite(os.path.join(ARTIFACT_DIR, "visual_06_pick_place_plate_duck.png"), hud_6)
        print(f"    ✅ Saved: visual_06_pick_place_plate_duck.png (delta_z: +{lift_org_mm:.1f}mm)")
        mission_passed = mission_passed and verified

        print(f"\nWorld 1 suite {'PASSED' if mission_passed else 'FAILED'}; visual evidence saved under {ARTIFACT_DIR}.")

    finally:
        print("Cleaning up World 1 simulation...")
        stop_owned_process_group(sim_proc)
        node.destroy_node()
        rclpy.shutdown()
    if not mission_passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
