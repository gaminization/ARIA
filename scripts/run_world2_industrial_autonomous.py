#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
ARIA World 2: Industrial Workcell Autonomous Verification
═══════════════════════════════════════════════════════════════════════════════
Executes Maneuvers 7 & 8 via pure autonomous natural language commands
dispatched to /aria/command. Zero hardcoded joint angles or target coordinates.
Captures synchronized 3-camera HUDs in the industrial workcell.
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
from gazebo_msgs.msg import ContactsState, ModelStates
from arm_interfaces.srv import SendCommand, SetConveyorPower
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
    env["ARIA_WORKCELL"] = "industrial"

    setup_cmd = f"source {WORKSPACE_ROOT}/install/setup.bash"
    headless = os.environ.get("ARIA_AUDIT_HEADLESS", "1").lower() in ("1", "true", "yes")
    gui_flag = "false" if headless else "true"
    launch_str = f"{setup_cmd} && ros2 launch arm_bringup industrial_workcell.launch.py gui:={gui_flag} with_agents:=true auto_cycle:=false"
    log_path = env.get("ARIA_AUDIT_GAZEBO_LOG")
    log_file = open(log_path, "w") if log_path else None
    try:
        proc = subprocess.Popen(
            ["bash", "-c", launch_str],
            env=env,
            stdout=log_file if log_file else subprocess.DEVNULL,
            stderr=subprocess.STDOUT if log_file else subprocess.DEVNULL,
            start_new_session=True,
        )
    finally:
        if log_file:
            log_file.close()
    return proc


class AutonomousIndustrialNode(Node):
    def __init__(self):
        super().__init__('autonomous_industrial_node')
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
        self.part_present: bool = False
        self.seen_cot = set()

        # Subscriptions
        self.create_subscription(Image, "/top_camera/image_raw", self._top_cb, 10)
        self.create_subscription(Image, "/side_camera/image_raw", self._side_cb, 10)
        self.create_subscription(Image, "/wrist_camera/image_raw", self._wrist_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(ModelStates, "/gazebo/model_states", self._models_cb, 10)
        self.button_contact_time = 0.0
        self.create_subscription(
            ContactsState, "/aria/industrial/estop_contact",
            self._button_contact_cb, 10)
        self.create_subscription(Bool, "/aria/conveyor/part_present", self._pres_cb, 10)
        self.create_subscription(String, "/aria/cot/reasoning", self._cot_cb, 10)
        self.create_subscription(TaskState, "/aria/state/task", self._task_cb, 10)

        # Clients
        self.cli_command = self.create_client(SendCommand, "/aria/command")
        self.cli_approve = self.create_client(Trigger, "/aria/approve")
        self.cli_conveyor = self.create_client(SetConveyorPower, "/aria/conveyor/set_power")

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

    def _pres_cb(self, msg: Bool):
        self.part_present = msg.data

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

    def _button_contact_cb(self, msg: ContactsState):
        if msg.states:
            self.button_contact_time = time.monotonic()

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
        print("  Waiting for Gazebo, controllers, and /aria/command in industrial workcell...")
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

    def set_conveyor(self, power: float):
        req = SetConveyorPower.Request()
        req.power = float(power)
        if not self.cli_conveyor.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("Conveyor power service is unavailable")
        future = self.cli_conveyor.call_async(req)
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError("Conveyor power request timed out")
        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else "no response"
            raise RuntimeError(f"Conveyor power request failed: {message}")
        print(f"    Conveyor: {response.message}")

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
        cv2.putText(canvas, f"GRIPPER: {math.degrees(self.current_joints[4]):.1f} deg | CONVEYOR SENSOR: {'TRIGGERED' if self.part_present else 'CLEAR'}",
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
    print("  🚀 SUITE 2: ARIA World 2 (aria_industrial_workcell.world) AUTONOMOUS EXECUTION")
    print("  Mode: Pure Natural Language Commands via /aria/command | Smooth 15 deg/s Speed")
    print("═" * 75)

    sim_proc = start_sim()
    rclpy.init()
    node = AutonomousIndustrialNode()
    mission_passed = True

    try:
        ready = node.wait_for_system_ready(timeout_s=90.0)
        if not ready:
            raise RuntimeError("Industrial simulation, cameras, model states, or command service did not become ready")
        node.spin_for(3.0)

        # ─────────────────────────────────────────────────────────────────────
        # Maneuver 7: Industrial Conveyor Motion Prediction & Reject Bin Sort
        # ─────────────────────────────────────────────────────────────────────
        print("\n[7/10] Maneuver 7: Autonomous Conveyor Sort to Red Box (Reject Bin)...")
        node.chain_of_thought.clear()

        # Step infeed conveyor to feed workpiece to docked pick station
        conveyor_start = {
            name: pose for name, pose in node.model_poses.items()
            if name.startswith("workpiece_")
        }
        node.set_conveyor(20.0)
        conveyor_motion_m = 0.0
        conveyor_deadline = time.monotonic() + 30.0
        try:
            while time.monotonic() < conveyor_deadline:
                node.spin_for(0.25)
                conveyor_motion_m = max(
                    (
                        math.dist(start, node.model_poses[name])
                        for name, start in conveyor_start.items()
                        if name in node.model_poses
                    ),
                    default=0.0,
                )
                if conveyor_motion_m >= 0.01:
                    break
        finally:
            node.set_conveyor(0.0)
        node.spin_for(1.0)
        conveyor_displacements = {
            name: math.dist(start, node.model_poses[name])
            for name, start in conveyor_start.items()
            if name in node.model_poses
        }
        conveyor_motion_m = max(conveyor_displacements.values(), default=0.0)
        print(
            "    Measured conveyor displacement: "
            + ", ".join(
                f"{name}={distance * 1000:.1f}mm"
                for name, distance in sorted(conveyor_displacements.items())
            )
        )

        initial_wp_z = node.require_model_pose("workpiece_01")[2]
        reject_bin = node.require_model_pose("reject_bin")
        rej_x, rej_y = reject_bin[:2]

        node.dispatch_command("Keep all of the blue items in the red box")

        # Wait until workpiece is lifted and transported over Red Box
        t0 = time.time()
        wp_in_hand = False
        lift_wp_mm = 0.0
        dist_to_bin = 1.0
        while time.time() - t0 < 120.0:
            node.spin_for(0.25)
            cur_wp = node.require_model_pose("workpiece_01")
            lift_wp_mm = (cur_wp[2] - initial_wp_z) * 1000.0
            dist_to_bin = math.hypot(cur_wp[0] - rej_x, cur_wp[1] - rej_y)
            if lift_wp_mm > 40.0 and dist_to_bin < 0.18:
                wp_in_hand = True
                break

        task_complete = node.wait_for_task_settle()
        evidence = node.pick_evidence("workpiece_01", initial_wp_z)
        cur_wp = node.require_model_pose("workpiece_01")
        lift_wp_mm = evidence.max_lift_m * 1000.0
        dist_to_bin = math.hypot(cur_wp[0] - rej_x, cur_wp[1] - rej_y)
        in_bin = (
            abs(cur_wp[0] - rej_x) <= 0.05
            and abs(cur_wp[1] - rej_y) <= 0.06
            and reject_bin[2] <= cur_wp[2] <= reject_bin[2] + 0.12
        )
        reasoning_available = bool(node.chain_of_thought)
        verified = (
            task_complete
            and evidence.passed
            and in_bin
            and conveyor_motion_m >= 0.01
            and reasoning_available
        )
        node.spin_for(1.5)

        hud_7 = node.create_hud_composite(
            maneuver_title="Maneuver 7: Industrial Conveyor Sort to Red Box",
            world_name="aria_industrial_workcell.world",
            target_object="workpiece_01",
            state_machine_stage="CONVEYOR_MOTION -> SORT -> VERIFIED_BIN_PLACEMENT",
            cot_notes=node.chain_of_thought[-6:],
            verification_text=f"Task={task_complete}, lift={lift_wp_mm:.1f}mm/hold={evidence.continuous_hold_s:.1f}s, in-bin={in_bin}, belt-motion={conveyor_motion_m*1000:.1f}mm, reasoning={reasoning_available}.",
            verified_pass=verified,
            extra_telemetry=[f"Maximum measured lift: +{lift_wp_mm:.1f} mm", f"Distance to Reject Bin: {dist_to_bin*100:.1f} cm", f"Conveyor displacement: {conveyor_motion_m*1000:.1f} mm"]
        )
        cv2.imwrite(os.path.join(ARTIFACT_DIR, "visual_07_industrial_conveyor_sort.png"), hud_7)
        print(f"    ✅ Saved: visual_07_industrial_conveyor_sort.png (delta_z: +{lift_wp_mm:.1f}mm, dist: {dist_to_bin*100:.1f}cm)")
        mission_passed = mission_passed and verified

        # ─────────────────────────────────────────────────────────────────────
        # Maneuver 8: Industrial Cabinet Control Button Pressing
        # ─────────────────────────────────────────────────────────────────────
        print("\n[8/10] Maneuver 8: Autonomous Workcell E-Stop Test Button Press...")
        node.chain_of_thought.clear()
        node.button_contact_time = 0.0
        node.dispatch_command("Press the reachable emergency stop test button on the workcell")
        task_complete = node.wait_for_task_settle()
        press_retracted = any(
            "PRESS: ✓ contact verified and retracted" in line
            for line in node.chain_of_thought
        )
        button_contact = node.button_contact_time > 0.0
        reasoning_available = bool(node.chain_of_thought)
        verified = task_complete and press_retracted and button_contact and reasoning_available
        node.spin_for(1.5)
        hud_8 = node.create_hud_composite(
            maneuver_title="Maneuver 8: Workcell E-Stop Test Button Pressing",
            world_name="aria_industrial_workcell.world",
            target_object="emergency_stop_test_button",
            state_machine_stage="PRESS_ACTION -> RETRACT",
            cot_notes=node.chain_of_thought[-6:],
            verification_text=f"Task={task_complete}, press/retract={press_retracted}, Gazebo button contact={button_contact}, reasoning={reasoning_available}.",
            verified_pass=verified,
            extra_telemetry=["Verification source: completed task trace, skill press result, and Gazebo contact sensor"]
        )
        cv2.imwrite(os.path.join(ARTIFACT_DIR, "visual_08_industrial_button_press.png"), hud_8)
        print("    ✅ Saved: visual_08_industrial_button_press.png")
        mission_passed = mission_passed and verified

        print(f"\nWorld 2 suite {'PASSED' if mission_passed else 'FAILED'}; visual evidence saved under {ARTIFACT_DIR}.")

    finally:
        print("Cleaning up World 2 simulation...")
        stop_owned_process_group(sim_proc)
        node.destroy_node()
        rclpy.shutdown()
    if not mission_passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
