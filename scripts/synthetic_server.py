#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
PROJECT ARIA: SYNTHETIC TASK COMMAND SERVER & STANDBY ORCHESTRATOR
═══════════════════════════════════════════════════════════════════════════════
Listens on /aria/command (arm_interfaces/srv/SendCommand) and /aria/synthetic/prompt
(std_msgs/String) for operator prompts.

DOES NOT EXECUTE TASKS AUTOMATICALLY OR ALL TOGETHER.
Stands by in IDLE state until the operator dispatches a prompt via:
  1. The Web Dashboard command box (POST /api/command -> /aria/command)
  2. Terminal interactive prompt CLI (interactive input loop)
  3. ROS 2 service call: `ros2 service call /aria/command arm_interfaces/srv/SendCommand "{command: 'wave'}"`
  4. ROS 2 topic: `ros2 topic pub --once /aria/synthetic/prompt std_msgs/String "{data: 'wave'}"`

Upon receiving a prompt:
  - Intelligently matches natural language intent
  - Updates /aria/state/task (TaskState) to EXECUTING
  - Dispatches CoT reasoning to /aria/cot/reasoning & dialogue to /aria/dialogue/output
  - Executes ONLY that specific maneuver
  - Verifies artifact HUD in runs/synthetic_audit/
  - Returns to home neutral pose
  - Updates /aria/state/task to COMPLETE then resets to IDLE standby
═══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import time
import math
import uuid
import queue
import re
import threading
import argparse
from typing import Dict, List, Optional, Tuple, Callable

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from gazebo_msgs.msg import ModelStates

from arm_interfaces.srv import SendCommand
from arm_planner.msg import TaskState

# Add workspace scripts to import path
WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WORKSPACE_ROOT not in sys.path:
    sys.path.insert(0, WORKSPACE_ROOT)
scripts_dir = os.path.join(WORKSPACE_ROOT, "scripts")
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

try:
    from scripts.synthetic_tester_suite import SyntheticTesterSuite
    from scripts.synthetic_industrial_suite import SyntheticIndustrialSuite
except ImportError:
    from synthetic_tester_suite import SyntheticTesterSuite
    from synthetic_industrial_suite import SyntheticIndustrialSuite


class SyntheticTaskServer(Node):
    """ROS 2 Command Server for On-Demand Synthetic Task Execution."""

    def __init__(self, target_world: str = "auto"):
        super().__init__("synthetic_task_server")
        self.target_world_arg = target_world
        self.active_world: Optional[str] = None
        self.suite = None

        # State tracking
        self.is_executing = False
        self.estop_active = False
        self.current_task_id = ""
        self.current_action_name = ""
        self.command_queue = queue.Queue()

        # QoS for state bus (TRANSIENT_LOCAL latched)
        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # ── Publishers ──────────────────────────────────────────────────────
        self.pub_task_state = self.create_publisher(TaskState, "/aria/state/task", qos_state)
        self.pub_dialogue = self.create_publisher(String, "/aria/dialogue/output", 10)
        self.pub_cot = self.create_publisher(String, "/aria/cot/reasoning", 10)

        # ── Subscriptions ───────────────────────────────────────────────────
        self.sub_prompt = self.create_subscription(
            String, "/aria/synthetic/prompt", self._prompt_topic_cb, 10
        )
        self.sub_models = self.create_subscription(
            ModelStates, "/gazebo/model_states", self._models_cb, 10
        )
        self.detected_models: List[str] = []

        # ── Services ────────────────────────────────────────────────────────
        self.srv_command = self.create_service(
            SendCommand, "/aria/command", self._command_srv_cb
        )
        self.srv_estop = self.create_service(
            Trigger, "/aria/estop", self._estop_srv_cb
        )
        self.srv_release_estop = self.create_service(
            Trigger, "/aria/release_estop", self._release_estop_srv_cb
        )
        self.srv_cancel = self.create_service(
            Trigger, "/aria/cancel", self._cancel_srv_cb
        )

        self.get_logger().info("Synthetic Task Server initialized. Initializing simulation world connection...")

    def _models_cb(self, msg: ModelStates):
        self.detected_models = list(msg.name)

    def determine_and_init_suite(self, executor: MultiThreadedExecutor, timeout_s: float = 30.0) -> bool:
        """Determines active world and attaches the corresponding synthetic suite."""
        print("  Detecting active Gazebo simulation world...")
        t0 = time.time()
        while time.time() - t0 < timeout_s and not self.detected_models:
            time.sleep(0.2)

        # World heuristic
        if self.target_world_arg == "industrial" or any("conveyor" in m for m in self.detected_models):
            self.active_world = "industrial"
            print("  ✓ Detected Industrial Workcell world. Initializing SyntheticIndustrialSuite...")
            self.suite = SyntheticIndustrialSuite()
        else:
            self.active_world = "tester"
            print("  ✓ Detected Standard Tester Workspace world. Initializing SyntheticTesterSuite...")
            self.suite = SyntheticTesterSuite()

        self.suite._spinning_externally = True
        executor.add_node(self.suite)

        # Wait for robot controllers & camera readiness
        ready = self.suite.wait_for_system_ready(timeout_s=25.0)
        if not ready:
            print("  ⚠️ System controllers reported partial readiness; arming anyway...")

        # Position robot at neutral home configuration
        self.suite.go_home()

        # Publish initial STANDBY state
        self.publish_task_state(
            status="IDLE",
            command="",
            goal="Standby: Awaiting user prompt for synthetic execution",
            subgoals=["System ready in standby mode", "Awaiting operator prompt via Dashboard or CLI"],
            confidence=1.0,
            task_id=""
        )

        ready_msg = f"ARIA Synthetic Task Server Online ({self.active_world.upper()} WORLD). Ready for user prompt."
        self.publish_dialogue(ready_msg)
        self.publish_cot(f"[SERVER] {ready_msg}")

        print("\n" + "═" * 78)
        print(f"  🤖 ARIA SYNTHETIC TASK SERVER ONLINE — {self.active_world.upper()} WORLD")
        print("  Status: STANDBY (Idle) — Ready for user prompt")
        print("  Tasks will execute ONLY when a prompt is dispatched.")
        print("  Endpoints:")
        print("    - Web Dashboard:     http://localhost:8000 (Natural Language input box)")
        print("    - ROS 2 Service:     ros2 service call /aria/command arm_interfaces/srv/SendCommand \"{command: 'wave'}\"")
        print("    - ROS 2 Topic:       ros2 topic pub --once /aria/synthetic/prompt std_msgs/String \"{data: 'wave'}\"")
        print("═" * 78 + "\n")
        return True

    # ── State Broadcasting ──────────────────────────────────────────────────
    def publish_task_state(self, status: str, command: str, goal: str, subgoals: List[str], confidence: float = 1.0, task_id: str = ""):
        msg = TaskState()
        msg.task_status = status
        msg.current_command = command
        msg.current_goal = goal
        msg.subgoals = subgoals
        msg.confidence = float(confidence)
        msg.task_id = task_id
        msg.awaiting_user_approval = False
        self.pub_task_state.publish(msg)

    def publish_dialogue(self, text: str):
        msg = String()
        msg.data = text
        self.pub_dialogue.publish(msg)

    def publish_cot(self, text: str):
        msg = String()
        msg.data = text
        self.pub_cot.publish(msg)

    # ── Natural Language Prompt Matcher ──────────────────────────────────────
    def match_prompt(self, raw_prompt: str) -> Optional[Tuple[str, Callable[[], bool], List[str]]]:
        """Maps operator natural language command to a specific synthetic maneuver."""
        p = raw_prompt.lower().strip()
        clean = re.sub(r"[^a-z0-9_ ]+", " ", p)
        words = clean.split()

        # ── 1. Neutral / Home Command
        if any(w in words for w in ["home", "rest", "neutral", "reset", "park"]) or "go home" in p:
            return (
                "Return to Home Neutral Posture",
                self.suite.go_home,
                ["Moving 5DOF arm to neutral home posture [0, 25°, -45°, 15°, 0°]"]
            )

        # ── 2. Industrial World Maneuvers
        if self.active_world == "industrial":
            # Conveyor speed switching
            if any(k in p for k in ["all speed", "conveyor speed", "switch speed", "change speed", "different speed", "all the speed", "speeds"]):
                return (
                    "Conveyor Speed Range & Dynamic Switching",
                    self.suite.maneuver_conveyor_all_speeds_and_switching,
                    ["Running creep speed (20%)", "Switching to nominal speed (50%)", "Switching to high speed (100%)", "Switching to reverse speed (-40%)", "Stopping belt (0%)"]
                )

            # Drop boxes on ground
            if any(k in p for k in ["drop box", "drop the box", "drop on ground", "drop to ground", "on the ground", "throw box", "floor"]):
                return (
                    "Drop Boxes on Ground (Disposal)",
                    self.suite.maneuver_drop_boxes_on_ground,
                    ["Picking disposal workpiece at stopper", "Swinging waist 75° beyond perimeter table edge", "Releasing workpiece into gravitational drop to floor"]
                )

            # Stack boxes
            if any(k in p for k in ["stack box", "stack the box", "pallet", "box stack"]):
                return (
                    "Multi-Tier Box Pallet Stacking",
                    self.suite.maneuver_stack_boxes,
                    ["Picking base box -> staging pallet", "Picking second box -> stacking directly on top (+35mm)", "Releasing and returning to home pose"]
                )

            # Conveyor sort boxes (blue vs red)
            if any(k in p for k in ["sort box", "sort the box", "sort blue", "defect", "conveyor sort", "starts the conveyor belt and sorts", "sort"]):
                return (
                    "Conveyor Infeed & Dual-Bin Box Sorting",
                    self.suite.maneuver_conveyor_sort_boxes,
                    ["Activating conveyor infeed (35% power)", "Picking good workpiece at stopper -> finished tray", "Advancing queue, picking defective workpiece -> reject bin"]
                )

            # E-Stop button press
            if any(k in p for k in ["estop", "e-stop", "emergency stop", "button", "mushroom", "hit estop"]):
                return (
                    "Emergency Stop Mushroom Button Press",
                    self.suite.maneuver_estop_button_press,
                    ["Approaching industrial mushroom E-Stop station", "Downward depression actuation", "Verifying emergency safety cutoff"]
                )

            # Continuous conveyor cycle
            if any(k in p for k in ["continuous", "cycle", "infeed"]):
                return (
                    "Continuous Conveyor Infeed Cycle",
                    self.suite.maneuver_continuous_conveyor_cycle,
                    ["Continuous infeed cycle with pick and sorting loop"]
                )

        # ── 3. Tester World Maneuvers
        else:
            # Wave gesture
            if any(w in words for w in ["wave", "greet", "greeting", "hello", "hi", "salute"]) or "wave" in p:
                return (
                    "Greeting Wave Gesture",
                    self.suite.maneuver_wave,
                    ["Elevating arm into greeting posture", "Oscillating wrist/waist wave cycles", "Returning to home pose"]
                )

            # Nod ("Yes") and Shake ("No")
            if any(k in p for k in ["nod", "shake", "say no", "gesture no", "head nod", "shake head", "nodding", "say yes"]):
                return (
                    "Nod ('Yes') and Shake ('No') Gestures",
                    self.suite.maneuver_nod_and_no,
                    ["Affirmative 'Yes' pitch nod oscillation", "Negative 'No' waist sweep gesture", "Returning to home pose"]
                )

            # Drink coffee
            if any(k in p for k in ["drink", "coffee", "sip", "tea", "take a drink"]):
                return (
                    "Coffee Drinking Gesture",
                    self.suite.maneuver_drink_coffee,
                    ["Grasping coffee mug", "Lifting and tilting 35° towards head/camera", "Pause sipping motion", "Returning mug to table"]
                )

            # Pick mug using handle
            if any(k in p for k in ["handle", "by handle", "mug handle", "using handle", "pick handle", "grab handle"]):
                target_m = "mug_bullet_right" if any(c in p for c in ["cyan", "blue", "right", "gym"]) else ("all" if "all" in p else "mug_ycb_front")
                return (
                    "Pick Mug Using Handle",
                    lambda tm=target_m: self.suite.maneuver_mug_handle_pick(tm),
                    ["Aligning end-effector along horizontal handle axis", f"Clamping loop handle of '{target_m}'", "Hoisting mug +80mm into mid-air", "Restoring mug upright on table"]
                )

            # Stack mugs
            if any(k in p for k in ["stack mug", "stack the mug", "mugs on top", "stack cups"]):
                return (
                    "Stack Mugs on Top of Each Other",
                    self.suite.maneuver_stack_mugs,
                    ["Picking upper ceramic mug", "Aligning above base mug rim", "Descending and nesting mugs", "Releasing grasp"]
                )

            # Banana in pan
            if any(k in p for k in ["banana in pan", "banana into pan", "pan banana", "fry banana", "cook banana"]) or ("banana" in p and "pan" in p):
                return (
                    "Put Banana into Frying Pan",
                    self.suite.maneuver_banana_in_pan,
                    ["Picking yellow banana workpiece", "Transporting over Tefal frying pan cavity", "Depositing banana into pan center"]
                )

            # Orange ball in glass
            if any(k in p for k in ["orange in glass", "orange into glass", "ball in glass"]) or ("orange" in p and "glass" in p):
                return (
                    "Put Orange Ball into Glass",
                    self.suite.maneuver_orange_ball_in_glass,
                    ["Grasping spherical orange workpiece", "Lifting and centering directly above glass rim", "Dropping orange ball into glass"]
                )

            # Smaller into bigger (duck in bowl)
            if any(k in p for k in ["smaller", "bigger", "duck in bowl", "duck into bowl", "nesting", "containment"]) or ("duck" in p and "bowl" in p):
                return (
                    "Smaller Object into Bigger Object (Duck into Bowl)",
                    self.suite.maneuver_smaller_in_bigger_objects,
                    ["Picking smaller object (duck)", "Transporting to larger container (bowl)", "Placing duck inside bowl"]
                )

            # Stack / Solve Jenga tower
            if any(k in p for k in ["jenga", "stack tower", "solve tower", "solve jenga", "tower", "timber", "block"]):
                return (
                    "Precision Jenga Tower Assembly & Stacking",
                    self.suite.maneuver_stack_jenga_tower,
                    [
                        "Anchoring foundation base at arena datum (-0.22, 0.00)",
                        "Systematically picking and transporting cross-tier blocks",
                        "Stacking 6-tier interlocking timber tower [0° / 90° alternating]",
                        "Verifying structural stability and vertical alignment"
                    ]
                )

            # Sort by color
            if any(k in p for k in ["color", "sort by color", "color sort"]):
                return (
                    "Sort Objects by Color",
                    self.suite.maneuver_sort_by_color,
                    ["Sorting yellow objects to tray", "Sorting blue/cyan objects to left sector", "Sorting red/orange objects to right sector"]
                )

            # Pick every single object (all 15)
            if any(k in p for k in ["pick every", "pick all", "every object", "all object", "everything", "all 15", "all items"]):
                return (
                    "Pick Every Object in World (15/15)",
                    self.suite.maneuver_pick_every_object,
                    ["Systematically approaching all 15 objects", "Verifying mid-air lift >= 70mm for each", "Returning to home pose"]
                )

            # Pick specific individual object
            obj_map = {
                "banana": "bullet_banana",
                "duck": "bullet_duck",
                "bottle": "beverage_bottle",
                "plate": "dinner_plate",
                "glass": "plastic_glass",
                "bowl": "plastic_bowl",
                "pan": "tefal_pan",
                "orange": "orange_fruit",
                "tea mug": "ceramic_tea_mug",
                "ceramic mug": "ceramic_tea_mug",
                "blue mug": "mug_bullet_right",
                "red mug": "mug_ycb_front",
                "mug": "mug_ycb_front",
                "tray": "sorting_tray",
                "pop tart": "pop_tarts_box",
                "jenga block": "jenga_block_1",
                "jenga": "jenga_block_1",
            }
            for alias, target_name in obj_map.items():
                if alias in p and any(verb in p for verb in ["pick", "lift", "grab", "get", "take"]):
                    return (
                        f"Pick Specific Object ({target_name})",
                        lambda name=target_name: self.suite.maneuver_pick_single_object(name),
                        [f"Approaching target '{target_name}'", "Grasping with gripper", "Lifting into mid-air (+70mm)", "Returning to surface"]
                    )

            # 360 Workspace inspection scan
            if any(k in p for k in ["scan", "360", "panoramic", "inspect", "survey"]):
                return (
                    "360° Panoramic Inspection Scan",
                    self.suite.maneuver_workspace_inspection_scan,
                    ["360° waist rotational sweep (-85° to +85°)", "Capturing camera vantage perspectives", "Returning to home pose"]
                )

            # Pedestal transfer
            if any(k in p for k in ["pedestal", "transfer"]):
                return (
                    "Pedestal-to-Pedestal Transfer",
                    self.suite.maneuver_pedestal_transfer,
                    ["Picking object from left elevated pedestal", "Transporting across workspace", "Placing on right elevated pedestal"]
                )

            # Table wipe / cleaning
            if any(k in p for k in ["wipe", "clean", "surface"]):
                return (
                    "Tabletop Wipe / Cleaning Motion",
                    self.suite.maneuver_table_wipe,
                    ["Positioning gripper at surface contact level", "Executing sinusoidal cleaning raster pattern", "Returning to home pose"]
                )

        return None

    def get_help_summary(self) -> str:
        if self.active_world == "industrial":
            return (
                "Valid Industrial Prompts: 'sort boxes', 'stack boxes', 'drop boxes on ground', "
                "'switch conveyor speed', 'press estop button', 'continuous infeed', 'go home'"
            )
        else:
            return (
                "Valid Tester Prompts: 'wave', 'nod'/'say no', 'drink coffee', 'pick every object', "
                "'pick up [banana|duck|plate|mug|...]', 'pick mug using handle', 'stack mugs', "
                "'banana in pan', 'orange in glass', 'smaller in bigger (duck in bowl)', 'stack jenga', "
                "'sort by color', '360 scan', 'pedestal transfer', 'table wipe', 'go home'"
            )

    # ── Service & Topic Callbacks ───────────────────────────────────────────
    def _command_srv_cb(self, request, response):
        cmd = request.command.strip()
        print(f"\n📩 [Command Received] User Prompt: '{cmd}'")

        if not cmd:
            response.accepted = False
            response.message = "Empty command"
            return response

        if self.estop_active:
            response.accepted = False
            response.message = "E-STOP active. Release E-stop before issuing commands."
            return response

        if self.is_executing:
            response.accepted = False
            response.message = f"Busy executing maneuver '{self.current_action_name}'. Please wait until complete."
            return response

        matched = self.match_prompt(cmd)
        if not matched:
            response.accepted = False
            help_str = self.get_help_summary()
            response.message = f"Unrecognized prompt: '{cmd}'. {help_str}"
            self.publish_dialogue(f"Unrecognized prompt: '{cmd}'. Try: {help_str}")
            return response

        action_name, func, subgoals = matched
        task_id = f"synthetic_{uuid.uuid4().hex[:8]}"

        response.accepted = True
        response.task_id = task_id
        response.message = f"Accepted: {action_name}"

        # Enqueue for asynchronous background execution
        self.command_queue.put((task_id, cmd, action_name, func, subgoals))
        return response

    def _prompt_topic_cb(self, msg: String):
        """Allows dispatching prompts via ROS 2 topic /aria/synthetic/prompt."""
        cmd = msg.data.strip()
        if not cmd:
            return
        if self.is_executing or self.estop_active:
            self.publish_dialogue(f"Ignored prompt '{cmd}' — system busy or in E-stop.")
            return
        matched = self.match_prompt(cmd)
        if matched:
            action_name, func, subgoals = matched
            task_id = f"synthetic_{uuid.uuid4().hex[:8]}"
            self.command_queue.put((task_id, cmd, action_name, func, subgoals))
        else:
            self.publish_dialogue(f"Unrecognized prompt '{cmd}'. {self.get_help_summary()}")

    def _estop_srv_cb(self, request, response):
        self.estop_active = True
        self.publish_task_state(
            status="EMERGENCY_STOP",
            command="",
            goal="E-STOP TRIGGERED — Motion Inhibited",
            subgoals=["Hardware E-STOP engaged"],
            confidence=0.0
        )
        self.publish_dialogue("EMERGENCY STOP ACTIVATED by operator.")
        response.success = True
        response.message = "E-STOP triggered"
        return response

    def _release_estop_srv_cb(self, request, response):
        self.estop_active = False
        self.publish_task_state(
            status="IDLE",
            command="",
            goal="Standby: Awaiting user prompt for synthetic execution",
            subgoals=["E-Stop released", "Ready for operator command"],
            confidence=1.0
        )
        self.publish_dialogue("E-STOP RELEASED. System returned to standby.")
        response.success = True
        response.message = "E-STOP released"
        return response

    def _cancel_srv_cb(self, request, response):
        self.publish_task_state(
            status="IDLE",
            command="",
            goal="Standby: Awaiting user prompt",
            subgoals=["Ready for next command"],
            confidence=1.0
        )
        response.success = True
        response.message = "Task cancelled / reset to IDLE"
        return response

    # ── Background Execution Worker ─────────────────────────────────────────
    def run_execution_worker(self):
        """Worker thread processing queued synthetic commands one at a time."""
        while rclpy.ok():
            try:
                task_id, cmd, action_name, func, subgoals = self.command_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            self.is_executing = True
            self.current_task_id = task_id
            self.current_action_name = action_name

            print("\n" + "═" * 78)
            print(f"  ▶️  DISPATCHING SYNTHETIC MANEUVER: {action_name.upper()}")
            print(f"  User Prompt: \"{cmd}\"")
            print(f"  Task ID:     {task_id}")
            print("═" * 78)

            # Broadcast EXECUTING status to dashboard and ROS
            self.publish_task_state(
                status="EXECUTING",
                command=cmd,
                goal=f"Executing Synthetic Maneuver: {action_name}",
                subgoals=subgoals,
                confidence=1.0,
                task_id=task_id,
            )
            self.publish_dialogue(f"Dispatched synthetic task: {action_name}")
            self.publish_cot(f"[SERVER] Commencing execution of '{action_name}'...")

            t0 = time.time()
            success = False
            try:
                success = func()
            except Exception as e:
                print(f"  ❌ Exception during maneuver execution: {e}")
                self.publish_cot(f"[ERROR] Maneuver exception: {e}")

            duration = time.time() - t0
            result_str = "SUCCESS" if success else "FAILED"
            print(f"  🏁 Maneuver '{action_name}' finished in {duration:.1f}s ({result_str})")

            # Clean return to neutral home posture
            try:
                self.suite.go_home()
            except Exception:
                pass

            # Broadcast COMPLETE status
            self.publish_task_state(
                status="COMPLETE" if success else "FAILED",
                command=cmd,
                goal=f"Completed Synthetic Maneuver: {action_name} ({result_str})",
                subgoals=subgoals,
                confidence=1.0,
                task_id=task_id,
            )
            self.publish_dialogue(f"Task '{action_name}' completed ({result_str}) in {duration:.1f}s. Returning to standby.")
            self.publish_cot(f"[SERVER] Task complete. System resetting to standby.")

            # Standby pause before resetting to clean IDLE state
            time.sleep(1.5)
            self.publish_task_state(
                status="IDLE",
                command="",
                goal="Standby: Awaiting next user prompt",
                subgoals=["Ready for next command..."],
                confidence=1.0,
                task_id="",
            )

            self.is_executing = False
            self.current_task_id = ""
            self.current_action_name = ""
            print("  ⚪ Standby: Ready for next user prompt.\n")


def interactive_terminal_loop(server: SyntheticTaskServer):
    """Allows operator to type prompts directly in the terminal."""
    time.sleep(2.0)
    print("\n" + "─" * 78)
    print("  ⌨️  TERMINAL INTERACTIVE PROMPT CONSOLE ACTIVE")
    print("  Type any command (e.g. 'wave', 'stack mugs', 'sort boxes') or 'help' / 'exit':")
    print("─" * 78)

    while rclpy.ok():
        try:
            line = input("\n[ARIA Prompt] >>> ").strip()
            if not line:
                continue
            if line.lower() in ("exit", "quit", "q"):
                print("Exiting synthetic server...")
                os._exit(0)
            if line.lower() in ("help", "list", "?"):
                print("\n" + server.get_help_summary() + "\n")
                continue

            # Dispatch via service simulation
            req = SendCommand.Request()
            req.command = line
            resp = SendCommand.Response()
            server._command_srv_cb(req, resp)
            if not resp.accepted:
                print(f"  ❌ {resp.message}")
            else:
                print(f"  ✓ {resp.message}")
        except (EOFError, KeyboardInterrupt):
            break
        except Exception as e:
            print(f"Input error: {e}")


def main():
    parser = argparse.ArgumentParser(description="ARIA Synthetic Standby Server")
    parser.add_argument("-w", "--world", choices=["tester", "industrial", "auto"], default="auto",
                        help="Active world ('tester', 'industrial', or 'auto' for auto-detection)")
    parser.add_argument("--interactive", action="store_true", help="Enable terminal interactive prompt input loop")
    args = parser.parse_args()

    rclpy.init()
    server = SyntheticTaskServer(target_world=args.world)
    executor = MultiThreadedExecutor()
    executor.add_node(server)

    # Initialize suite in executor
    server.determine_and_init_suite(executor)

    # Start background execution worker
    worker_th = threading.Thread(target=server.run_execution_worker, daemon=True)
    worker_th.start()

    # If running interactively in terminal, start CLI reader thread
    if args.interactive or sys.stdin.isatty():
        cli_th = threading.Thread(target=interactive_terminal_loop, args=(server,), daemon=True)
        cli_th.start()

    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
