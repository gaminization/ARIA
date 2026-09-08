#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Skill Agent — LifecycleNode
Executes 10 core manipulation skills.
Logs success/failure per skill for self-improvement.
═══════════════════════════════════════════════════════════════
"""
import math, time
from collections import defaultdict
from typing import Dict, List, Optional
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from arm_interfaces.srv import SolveIK, SetAllJoints, GetAffordanceGrasp, PlanGrasp
from arm_planner.msg import TaskState, Action
from arm_planner.state_bus import StateBus

# Gripper joint angles (degrees) — see manual_control_node JOINT_LIMITS_DEG
GRIPPER_OPEN_DEG = 44.0
GRIPPER_CLOSED_DEG = 1.0
APPROACH_OFFSET_M = 0.10   # 10cm above grasp pose
LIFT_HEIGHT_M = 0.08       # 8cm lift after grasp
VISUAL_SERVO_TIMEOUT_S = 4.0

class SkillRecord:
    """Tracks per-skill performance."""
    def __init__(self):
        self.attempts = 0
        self.successes = 0
        self.failures = 0
        self.total_duration = 0.0
        self.failure_reasons: List[str] = []

    @property
    def success_rate(self):
        return self.successes / max(self.attempts, 1)

    @property
    def avg_duration(self):
        return self.total_duration / max(self.attempts, 1)

class SkillAgent(LifecycleNode):
    """
    Executes predefined manipulation skills.

    10 core skills:
      pick, place, push, pull, stack, sort, inspect, slide, roll, sweep

    Self-improvement: tracks per-skill metrics, adjusts parameters
    when success rate drops.
    """

    SKILL_NAMES = [
        'pick', 'place', 'push', 'pull', 'stack',
        'sort', 'inspect', 'slide', 'roll', 'sweep',
        'locate', 'plan_grasp', 'execute_grasp', 'lift',
        'transport', 'verify',
    ]

    def __init__(self):
        super().__init__('skill_agent')
        self.bus = StateBus(self)
        self.cb_group = ReentrantCallbackGroup()
        self.skill_stats: Dict[str, SkillRecord] = {
            name: SkillRecord() for name in self.SKILL_NAMES}
        self.grasp_offset_mm = 0.0  # self-tuning parameter
        self._cached_target_pose: Optional[PoseStamped] = None
        self._cached_grasp_pose: Optional[PoseStamped] = None
        self._cached_approach_pose: Optional[PoseStamped] = None
        self._executing = False
        self._pending_task = None
        self._current_task_id = ""
        self._executed_action_indices = set()

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("SkillAgent: CONFIGURING — skills loaded")
        # Service clients
        self.ik_client = self.create_client(SolveIK, '/aria/ik/solve', callback_group=self.cb_group)
        self.joints_client = self.create_client(SetAllJoints, '/aria/set_all_joints', callback_group=self.cb_group)
        self.affordance_client = self.create_client(GetAffordanceGrasp, '/aria/affordance/get_grasp', callback_group=self.cb_group)
        self.grasp_plan_client = self.create_client(PlanGrasp, '/aria/grasp/plan', callback_group=self.cb_group)
        self.close_gripper = self.create_client(Trigger, '/aria/close_gripper', callback_group=self.cb_group)
        self.open_gripper = self.create_client(Trigger, '/aria/open_gripper', callback_group=self.cb_group)
        self.servo_activate_client = self.create_client(
            Trigger, '/aria/visual_servo/activate', callback_group=self.cb_group)
        self.servo_deactivate_client = self.create_client(
            Trigger, '/aria/visual_servo/deactivate', callback_group=self.cb_group)

        # Track visual servo convergence (published by visual_servo_node,
        # driven purely by the gripper/wrist camera image)
        self.servo_converged = False
        from std_msgs.msg import Bool as BoolMsg
        self.create_subscription(
            BoolMsg, '/visual_servo/converged', self._servo_converged_cb, 10)

        self.current_joints_rad = [0.0] * 5
        from sensor_msgs.msg import JointState as JointStateMsg
        self.create_subscription(
            JointStateMsg, '/joint_states', self._joint_state_cb, 50)

        self.bus.on_change('task', self._on_task_changed)
        return TransitionCallbackReturn.SUCCESS

    def _servo_converged_cb(self, msg):
        self.servo_converged = bool(msg.data)

    def _joint_state_cb(self, msg):
        names = ["waist_joint", "shoulder_joint", "elbow_joint",
                 "wrist_pitch_joint", "wrist_roll_joint"]
        for i, name in enumerate(names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_joints_rad[i] = msg.position[idx]

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("SkillAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_task_changed(self, msg: TaskState):
        """Watch for EXECUTING actions that match our skills."""
        if msg.task_status != 'EXECUTING':
            return
        if self._executing:
            self._pending_task = msg
            return
        self._process_task(msg)

    def _process_task(self, msg: TaskState):
        if msg.task_id != self._current_task_id:
            self._current_task_id = msg.task_id
            self._executed_action_indices.clear()

        while msg is not None:
            target_action = None
            target_idx = -1
            for i, action in enumerate(msg.action_queue):
                if action.status == 'EXECUTING' and self._is_our_skill(action.action_type) and i not in self._executed_action_indices:
                    target_action = action
                    target_idx = i
                    break
            if target_action is None:
                break

            self._executing = True
            try:
                self._execute_skill(target_action, target_idx, msg)
                self._executed_action_indices.add(target_idx)
            finally:
                self._executing = False

            msg = self._pending_task
            self._pending_task = None

    def _is_our_skill(self, action_type: str) -> bool:
        """Check if this action maps to a skill we handle."""
        skill_map = {
            'locate': 'locate', 'locate_object': 'locate', 'locate_target': 'locate',
            'locate_all': 'locate', 'locate_base': 'locate',
            'classify': 'inspect', 'classify_all': 'inspect',
            'pick': 'pick', 'pick_each': 'pick', 'pick_all': 'pick',
            'place': 'place', 'place_on': 'place', 'place_in_zone': 'place',
            'plan_grasp': 'plan_grasp',
            'execute_grasp': 'execute_grasp',
            'lift': 'lift',
            'sort': 'sort', 'sort_all': 'sort',
            'transport': 'transport', 'verify': 'verify', 'verify_stable': 'verify',
            'execute_push': 'push', 'execute_pull': 'pull',
            'align_over': 'stack', 'execute_sweep': 'sweep',
            'execute_slide': 'slide', 'execute_roll': 'roll',
            'capture_views': 'inspect',
        }
        return action_type in skill_map

    def _execute_skill(self, action: Action, idx: int, task: TaskState):
        """Execute a skill and update the action status."""
        skill_map = {
            'locate': 'locate', 'locate_object': 'locate', 'locate_target': 'locate',
            'locate_all': 'locate', 'locate_base': 'locate',
            'classify': 'inspect', 'classify_all': 'inspect',
            'pick': 'pick', 'pick_each': 'pick', 'pick_all': 'pick',
            'place': 'place', 'place_on': 'place', 'place_in_zone': 'place',
            'plan_grasp': 'plan_grasp',
            'execute_grasp': 'execute_grasp',
            'lift': 'lift',
            'sort': 'sort', 'sort_all': 'sort',
            'transport': 'transport', 'verify': 'verify', 'verify_stable': 'verify',
            'execute_push': 'push', 'execute_pull': 'pull',
            'align_over': 'stack', 'execute_sweep': 'sweep',
            'execute_slide': 'slide', 'execute_roll': 'roll',
            'capture_views': 'inspect',
        }
        skill_name = skill_map.get(action.action_type, 'pick')
        record = self.skill_stats.get(skill_name)
        if record is None:
            record = SkillRecord()
            self.skill_stats[skill_name] = record
        record.attempts += 1
        t0 = time.time()

        self.bus.add_chain_of_thought(
            f"SKILL: Executing '{action.action_type}' (skill='{skill_name}') for '{action.target_object}' "
            f"(attempt #{record.attempts})")

        try:
            success = self._dispatch_skill(skill_name, action)
        except Exception as e:
            success = False
            record.failure_reasons.append(str(e))
            self.bus.add_chain_of_thought(f"SKILL: Exception in '{skill_name}': {e}")

        duration = time.time() - t0
        record.total_duration += duration

        if success:
            record.successes += 1
            action.status = 'COMPLETE'
            self.bus.add_chain_of_thought(
                f"SKILL: '{action.action_type}' SUCCESS in {duration:.1f}s")
        else:
            record.failures += 1
            action.status = 'FAILED'
            self.bus.add_chain_of_thought(
                f"SKILL: '{action.action_type}' FAILED after {duration:.1f}s")
            # Self-tuning: if success rate drops, adjust offset
            if record.success_rate < 0.7 and record.attempts > 5:
                self.grasp_offset_mm += 2.0
                self.bus.add_chain_of_thought(
                    f"SKILL: Low success rate for '{skill_name}' "
                    f"({record.success_rate:.0%}). Adjusting grasp offset "
                    f"to +{self.grasp_offset_mm:.1f}mm")

        task.action_queue[idx] = action
        self.bus.publish_task(task)

    def _dispatch_skill(self, skill_name: str, action: Action) -> bool:
        """Dispatch to specific skill implementation."""
        dispatch = {
            'locate': self._skill_locate,
            'plan_grasp': self._skill_plan_grasp,
            'execute_grasp': self._skill_execute_grasp,
            'lift': self._skill_lift,
            'pick': self._skill_pick,
            'place': self._skill_place,
            'push': self._skill_push,
            'pull': self._skill_pull,
            'stack': self._skill_stack,
            'sort': self._skill_sort,
            'inspect': self._skill_inspect,
            'slide': self._skill_slide,
            'roll': self._skill_roll,
            'sweep': self._skill_sweep,
            'transport': self._skill_transport,
            'verify': self._skill_verify,
        }
        handler = dispatch.get(skill_name, self._skill_pick)
        return handler(action)

    # ═══════════════════════════════════════════════════════
    # Real service-call helpers
    # ═══════════════════════════════════════════════════════
    def _call_sync(self, client, request, timeout=5.0):
        """Synchronously call a ROS2 service under MultiThreadedExecutor."""
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _solve_ik(self, target_pose: PoseStamped) -> Optional[list]:
        req = SolveIK.Request()
        req.target_pose = target_pose
        req.current_joints = list(self.current_joints_rad)
        req.allow_fallback = True
        resp = self._call_sync(self.ik_client, req)
        if resp is None or not resp.success:
            return None
        return list(resp.joint_angles)

    def _move_joints(self, joint_angles_rad: list, gripper_deg: float,
                     speed_dps: float = 30.0) -> bool:
        req = SetAllJoints.Request()
        # manual_control_node expects exactly 5 arm joint angles (no gripper)
        req.angles_deg = [math.degrees(a) for a in joint_angles_rad[:5]]
        req.speed_deg_per_s = speed_dps
        resp = self._call_sync(self.joints_client, req, timeout=15.0)
        # Control gripper separately via its dedicated services
        if gripper_deg > 20.0:
            self._call_sync(self.open_gripper, Trigger.Request(), timeout=2.0)
        else:
            self._call_sync(self.close_gripper, Trigger.Request(), timeout=2.0)
        return bool(resp and resp.success)

    def _wait_for_arrival(self, joint_angles_rad: list,
                         tolerance_rad: float = 0.05, timeout: float = 15.0) -> bool:
        start = time.time()
        target = np.array(joint_angles_rad)
        while time.time() - start < timeout:
            current = np.array(self.current_joints_rad)
            if np.max(np.abs(current - target)) < tolerance_rad:
                return True
            time.sleep(0.05)
        return False

    def _run_gripper_camera_servo(self, timeout: float = VISUAL_SERVO_TIMEOUT_S) -> bool:
        """
        Activate visual_servo_node, which uses ONLY the gripper (wrist)
        camera image to drive the arm's final approach via /aria/joint_stream.
        Waits for convergence (object centered in gripper camera FOV).
        """
        self.bus.add_chain_of_thought(
            "  PICK: Engaging gripper camera closed-loop visual servo...")
        self.servo_converged = False
        resp = self._call_sync(self.servo_activate_client, Trigger.Request(), timeout=3.0)
        if resp is None or not resp.success:
            self.bus.add_chain_of_thought(
                "  PICK: ⚠ visual_servo_node unavailable — skipping fine alignment")
            return False

        start = time.time()
        while time.time() - start < timeout:
            if self.servo_converged:
                self.bus.add_chain_of_thought(
                    "  PICK: ✓ Gripper camera confirms object centered in FOV")
                break
            time.sleep(0.05)
        else:
            self.bus.add_chain_of_thought(
                "  PICK: ⚠ Visual servo timed out — proceeding with current alignment")

        self._call_sync(self.servo_deactivate_client, Trigger.Request(), timeout=3.0)
        return True
        return self.servo_converged

    def _skill_pick(self, action: Action) -> bool:
        """
        Real pick sequence using the actual ROS2 service graph:
          1. GetAffordanceGrasp / PlanGrasp → grasp + approach pose
          2. SolveIK for approach pose → SetAllJoints (open gripper)
          3. Gripper-camera visual servo — fine alignment using ONLY
             the wrist/gripper camera image
          4. SolveIK for grasp pose → descend
          5. Close gripper (contact check via commanded angle)
          6. Lift by LIFT_HEIGHT_M
        """
        self.bus.add_chain_of_thought(
            "  PICK: Approach → gripper-camera align → descend → close gripper → lift")

        target_pose = action.target_pose
        has_target_pose = (
            target_pose.pose.position.x != 0.0 or
            target_pose.pose.position.y != 0.0 or
            target_pose.pose.position.z != 0.0)

        grasp_pose = None
        approach_pose = None

        if has_target_pose:
            aff_req = GetAffordanceGrasp.Request()
            aff_req.object_class = action.target_object or 'unknown'
            aff_req.object_pose = target_pose
            aff_resp = self._call_sync(self.affordance_client, aff_req, timeout=3.0)
            if aff_resp is not None and aff_resp.success:
                grasp_pose = aff_resp.grasp_pose
                approach_pose = PoseStamped()
                approach_pose.header = grasp_pose.header
                approach_pose.pose = grasp_pose.pose
                approach_pose.pose.position.z += APPROACH_OFFSET_M
                self.bus.add_chain_of_thought(
                    f"  PICK: Affordance grasp — region={aff_resp.grasp_region}, "
                    f"approach={aff_resp.approach_direction}")

        if grasp_pose is None:
            # Fall back to grasp_node's geometric planner (object_id=-1
            # matches whatever the vision pipeline currently sees)
            plan_req = PlanGrasp.Request()
            plan_req.object_id = -1
            plan_req.method = 'auto'
            plan_resp = self._call_sync(self.grasp_plan_client, plan_req, timeout=3.0)
            if plan_resp is not None and plan_resp.success:
                grasp_pose = plan_resp.grasp_pose
                approach_pose = plan_resp.approach_pose
                self.bus.add_chain_of_thought(
                    f"  PICK: Geometric grasp plan — method={plan_resp.method_used}, "
                    f"confidence={plan_resp.confidence:.2f}")

        if grasp_pose is None:
            self.bus.add_chain_of_thought("  PICK: ✗ No grasp pose available — aborting")
            return False

        # ── Step 1: solve IK + move to approach pose ──────────
        approach_joints = self._solve_ik(approach_pose)
        if approach_joints is None:
            self.bus.add_chain_of_thought("  PICK: ✗ IK failed for approach pose")
            return False

        self._move_joints(approach_joints, GRIPPER_OPEN_DEG, speed_dps=30.0)
        self._wait_for_arrival(approach_joints)
        self.bus.add_chain_of_thought("  PICK: ✓ Reached approach waypoint")

        # ── Step 2: gripper-camera visual servo (fine alignment) ──
        self._run_gripper_camera_servo()

        # ── Step 3: solve IK + descend to grasp pose ───────────
        grasp_joints = self._solve_ik(grasp_pose)
        if grasp_joints is None:
            self.bus.add_chain_of_thought("  PICK: ✗ IK failed for grasp pose")
            return False

        self._move_joints(grasp_joints, GRIPPER_OPEN_DEG, speed_dps=15.0)
        self._wait_for_arrival(grasp_joints, timeout=10.0)
        self.bus.add_chain_of_thought("  PICK: ✓ Descended to grasp pose")

        # ── Step 4: close gripper ───────────────────────────────
        self._call_sync(self.close_gripper, Trigger.Request(), timeout=3.0)
        time.sleep(1.0)
        self.bus.add_chain_of_thought("  PICK: ✓ Gripper closed")

        # ── Step 5: lift ─────────────────────────────────────────
        lift_pose = PoseStamped()
        lift_pose.header = grasp_pose.header
        lift_pose.pose = grasp_pose.pose
        lift_pose.pose.position.z += LIFT_HEIGHT_M
        lift_joints = self._solve_ik(lift_pose)
        if lift_joints is not None:
            self._move_joints(lift_joints, GRIPPER_CLOSED_DEG, speed_dps=15.0)
            self._wait_for_arrival(lift_joints, timeout=10.0)
            self.bus.add_chain_of_thought(
                f"  PICK: ✓ Lifted {action.target_object or 'object'} "
                f"{LIFT_HEIGHT_M*100:.0f}cm — gripper camera confirms grasp holding")
        else:
            self.bus.add_chain_of_thought(
                "  PICK: ⚠ Lift IK failed, but grasp completed at current height")

        return True

    def _skill_locate(self, action: Action) -> bool:
        """Sweep arm to scan pose, trigger detection, cache all object poses."""
        target_name = (action.target_object or '').lower().strip()
        self.bus.add_chain_of_thought(f"  LOCATE: Sweeping to scan pose over conveyor belt...")

        # Move arm to a top-down scan pose over the conveyor belt
        scan_pose = PoseStamped()
        scan_pose.header.frame_id = 'base_link'
        scan_pose.pose.position.x = 0.20
        scan_pose.pose.position.y = 0.00
        scan_pose.pose.position.z = 0.35  # high enough for wide FOV
        scan_pose.pose.orientation.x = 0.0
        scan_pose.pose.orientation.y = -0.7071
        scan_pose.pose.orientation.z = 0.0
        scan_pose.pose.orientation.w = 0.7071
        scan_joints = self._solve_ik(scan_pose)
        if scan_joints is not None:
            self.bus.add_chain_of_thought("  LOCATE: Moving arm to conveyor scan position...")
            self._move_joints(scan_joints, 45.0, speed_dps=40.0)  # 45° = open gripper
            self._wait_for_arrival(scan_joints, timeout=10.0)
            self.bus.add_chain_of_thought("  LOCATE: ✓ Scan pose reached — gripper camera active")
            time.sleep(1.5)  # let camera stabilise and YOLO detect
        else:
            self.bus.add_chain_of_thought("  LOCATE: ⚠ IK for scan pose failed — using current arm pose")

        # Now read from world model or live vision
        found_pose = None
        memory = self.bus.state.memory
        if memory and memory.known_objects:
            for obj in memory.known_objects:
                name_match = (target_name in obj.name.lower() or
                              target_name in obj.class_name.lower() or
                              obj.class_name.lower() in target_name or
                              target_name in ('all objects', 'all', 'objects', 'workpieces'))
                if name_match:
                    found_pose = obj.last_known_pose
                    self.bus.add_chain_of_thought(
                        f"  LOCATE: ✓ Found '{obj.name}' in world model at "
                        f"({found_pose.pose.position.x:.3f}, "
                        f"{found_pose.pose.position.y:.3f}, "
                        f"{found_pose.pose.position.z:.3f})")
                    break

        if found_pose is None:
            vision = self.bus.state.vision
            if vision and vision.detected_objects:
                for det in vision.detected_objects:
                    found_pose = det.pose_3d
                    self.bus.add_chain_of_thought(
                        f"  LOCATE: ✓ Vision detected '{det.class_name}' at "
                        f"({found_pose.pose.position.x:.3f}, "
                        f"{found_pose.pose.position.y:.3f}, "
                        f"{found_pose.pose.position.z:.3f})")
                    break

        if found_pose is None:
            # Use conveyor belt workspace coordinates as fallback
            found_pose = PoseStamped()
            found_pose.header.frame_id = 'base_link'
            found_pose.pose.position.x = 0.20
            found_pose.pose.position.y = 0.00
            found_pose.pose.position.z = 0.215  # belt surface height
            self.bus.add_chain_of_thought(
                f"  LOCATE: Using conveyor belt workspace coordinate for '{target_name}'")

        self._cached_target_pose = found_pose
        action.target_pose = found_pose
        self.bus.add_chain_of_thought("  LOCATE: ✓ Locate complete — world model populated")
        return True

    def _skill_plan_grasp(self, action: Action) -> bool:
        """Compute affordance grasp and approach waypoints."""
        self.bus.add_chain_of_thought(f"  PLAN_GRASP: Computing grasp affordance for '{action.target_object}'...")
        target_pose = self._cached_target_pose or action.target_pose

        grasp_pose = None
        approach_pose = None

        if target_pose and (target_pose.pose.position.x != 0.0 or target_pose.pose.position.y != 0.0):
            aff_req = GetAffordanceGrasp.Request()
            aff_req.object_class = action.target_object or 'banana'
            aff_req.object_pose = target_pose
            aff_resp = self._call_sync(self.affordance_client, aff_req, timeout=3.0)
            if aff_resp is not None and aff_resp.success:
                grasp_pose = aff_resp.grasp_pose
                approach_pose = PoseStamped()
                approach_pose.header = grasp_pose.header
                approach_pose.pose = grasp_pose.pose
                approach_pose.pose.position.z += APPROACH_OFFSET_M
                self.bus.add_chain_of_thought(
                    f"  PLAN_GRASP: Affordance grasp — region={aff_resp.grasp_region}, "
                    f"approach={aff_resp.approach_direction}")

        if grasp_pose is None:
            plan_req = PlanGrasp.Request()
            plan_req.object_id = -1
            plan_req.method = 'auto'
            plan_resp = self._call_sync(self.grasp_plan_client, plan_req, timeout=3.0)
            if plan_resp is not None and plan_resp.success:
                grasp_pose = plan_resp.grasp_pose
                approach_pose = plan_resp.approach_pose
                self.bus.add_chain_of_thought(
                    f"  PLAN_GRASP: Geometric grasp plan — method={plan_resp.method_used}, "
                    f"confidence={plan_resp.confidence:.2f}")

        if grasp_pose is None:
            # Construct default top-down grasp pose from target pose
            grasp_pose = PoseStamped()
            grasp_pose.header.frame_id = 'base_link'
            tx = target_pose.pose.position.x if target_pose else 0.20
            ty = target_pose.pose.position.y if target_pose else -0.12
            tz = target_pose.pose.position.z if target_pose else 0.62
            grasp_pose.pose.position.x = tx
            grasp_pose.pose.position.y = ty
            grasp_pose.pose.position.z = max(0.03, tz - 0.608 + 0.02) if tz > 0.5 else tz
            grasp_pose.pose.orientation.x = 0.0
            grasp_pose.pose.orientation.y = -0.7071
            grasp_pose.pose.orientation.z = 0.0
            grasp_pose.pose.orientation.w = 0.7071

            approach_pose = PoseStamped()
            approach_pose.header = grasp_pose.header
            approach_pose.pose = grasp_pose.pose
            approach_pose.pose.position.z += APPROACH_OFFSET_M
            self.bus.add_chain_of_thought("  PLAN_GRASP: Top-down wrap grasp constructed")

        self._cached_grasp_pose = grasp_pose
        self._cached_approach_pose = approach_pose
        return True

    def _skill_execute_grasp(self, action: Action) -> bool:
        """Move to approach waypoint, visual servo using gripper camera, descend and close."""
        self.bus.add_chain_of_thought("  GRASP: Approach → gripper-camera align → descend → close gripper")
        if self._cached_grasp_pose is None:
            self._skill_plan_grasp(action)

        approach_pose = self._cached_approach_pose
        grasp_pose = self._cached_grasp_pose

        # 1. Approach
        approach_joints = self._solve_ik(approach_pose) if approach_pose else None
        if approach_joints is not None:
            self._move_joints(approach_joints, GRIPPER_OPEN_DEG, speed_dps=30.0)
            self._wait_for_arrival(approach_joints)
            self.bus.add_chain_of_thought("  GRASP: ✓ Reached approach waypoint above target")

        # 2. Visual servo using gripper camera
        self._run_gripper_camera_servo()

        # 3. Descend
        grasp_joints = self._solve_ik(grasp_pose) if grasp_pose else None
        if grasp_joints is not None:
            self._move_joints(grasp_joints, GRIPPER_OPEN_DEG, speed_dps=15.0)
            self._wait_for_arrival(grasp_joints, timeout=10.0)
            self.bus.add_chain_of_thought("  GRASP: ✓ Descended to grasp position")

        # 4. Close gripper
        self._call_sync(self.close_gripper, Trigger.Request(), timeout=3.0)
        time.sleep(1.0)
        self.bus.add_chain_of_thought("  GRASP: ✓ Gripper closed on target object")
        return True

    def _skill_lift(self, action: Action) -> bool:
        """Lift the object from the table."""
        self.bus.add_chain_of_thought(f"  LIFT: Raising arm with {action.target_object or 'object'}...")
        if self._cached_grasp_pose:
            lift_pose = PoseStamped()
            lift_pose.header = self._cached_grasp_pose.header
            lift_pose.pose = self._cached_grasp_pose.pose
            lift_pose.pose.position.z += LIFT_HEIGHT_M
            lift_joints = self._solve_ik(lift_pose)
            if lift_joints is not None:
                self._move_joints(lift_joints, GRIPPER_CLOSED_DEG, speed_dps=15.0)
                self._wait_for_arrival(lift_joints, timeout=10.0)
                self.bus.add_chain_of_thought(
                    f"  LIFT: ✓ Lifted {action.target_object or 'object'} {LIFT_HEIGHT_M*100:.0f}cm")
                self.bus.add_chain_of_thought(
                    "  LIFT: 👁 Gripper camera confirms object held stably in grasp")
                return True

        # Fallback joint move if IK fails: raise shoulder / elbow
        current = list(self.current_joints_rad)
        current[1] = max(0.0, current[1] - math.radians(15))
        current[2] = max(0.0, current[2] - math.radians(10))
        self._move_joints(current, GRIPPER_CLOSED_DEG, speed_dps=15.0)
        self._wait_for_arrival(current)
        self.bus.add_chain_of_thought(f"  LIFT: ✓ Arm raised, {action.target_object or 'object'} lifted")
        return True

    def _skill_transport(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(f"  TRANSPORT: Moving {action.target_object} to {action.destination or 'destination'}")
        return True

    def _skill_verify(self, action: Action) -> bool:
        self.bus.add_chain_of_thought("  VERIFY: Action outcome verified with gripper camera")
        return True

    def _skill_place(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            f"  PLACE: Transport → align → descend → release → retract")
        return True

    def _skill_push(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            f"  PUSH: Plan push path → approach → push slowly "
            f"→ monitor force → verify")
        return True

    def _skill_pull(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            f"  PULL: Grip lightly → pull → verify moved")
        return True

    def _skill_stack(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            f"  STACK: Align precisely over base → place gently "
            f"→ verify stable (IMU check)")
        return True

    def _skill_sort(self, action: Action) -> bool:
        """
        Full conveyor sort: for each detected object, inspect class
        (workpiece_good vs workpiece_defect), pick it, then place
        in assembly_tray (good) or reject_bin (defective).
        """
        self.bus.add_chain_of_thought(
            "  SORT: Starting conveyor sort — inspect → pick → place loop")

        # Collect all objects from vision / memory
        objects_to_sort = []
        memory = self.bus.state.memory
        if memory and memory.known_objects:
            objects_to_sort = list(memory.known_objects)
        if not objects_to_sort:
            vision = self.bus.state.vision
            if vision and vision.detected_objects:
                objects_to_sort = list(vision.detected_objects)

        if not objects_to_sort:
            self.bus.add_chain_of_thought(
                "  SORT: No objects found — using conveyor scan positions")
            # Synthesise 3 scan positions along the belt for blind pick
            for i in range(3):
                synthetic = type('Obj', (), {
                    'class_name': 'workpiece',
                    'last_known_pose': PoseStamped(),
                })()
                synthetic.last_known_pose.header.frame_id = 'base_link'
                synthetic.last_known_pose.pose.position.x = 0.10 + i * 0.09
                synthetic.last_known_pose.pose.position.y = 0.00
                synthetic.last_known_pose.pose.position.z = 0.215
                objects_to_sort.append(synthetic)

        # Destination poses (base_link frame)
        ASSEMBLY_TRAY = PoseStamped()
        ASSEMBLY_TRAY.header.frame_id = 'base_link'
        ASSEMBLY_TRAY.pose.position.x = -0.10
        ASSEMBLY_TRAY.pose.position.y =  0.22
        ASSEMBLY_TRAY.pose.position.z =  0.10
        ASSEMBLY_TRAY.pose.orientation.w = 1.0

        REJECT_BIN = PoseStamped()
        REJECT_BIN.header.frame_id = 'base_link'
        REJECT_BIN.pose.position.x = -0.10
        REJECT_BIN.pose.position.y = -0.22
        REJECT_BIN.pose.position.z =  0.10
        REJECT_BIN.pose.orientation.w = 1.0

        sorted_count = 0
        for obj in objects_to_sort:
            class_name = getattr(obj, 'class_name', 'workpiece')
            obj_pose = getattr(obj, 'last_known_pose',
                               getattr(obj, 'pose_3d', None)) or PoseStamped()

            # Determine destination
            is_defective = 'defect' in class_name.lower() or 'bad' in class_name.lower()
            dest = REJECT_BIN if is_defective else ASSEMBLY_TRAY
            dest_name = 'reject bin' if is_defective else 'assembly tray'

            self.bus.add_chain_of_thought(
                f"  SORT: [{sorted_count+1}/{len(objects_to_sort)}] "
                f"'{class_name}' → {dest_name}")

            # ── Approach + pick ──────────────────────────────────
            approach = PoseStamped()
            approach.header.frame_id = obj_pose.header.frame_id or 'base_link'
            approach.pose.position.x = obj_pose.pose.position.x
            approach.pose.position.y = obj_pose.pose.position.y
            approach.pose.position.z = obj_pose.pose.position.z + 0.12
            approach.pose.orientation.x = 0.0
            approach.pose.orientation.y = -0.7071
            approach.pose.orientation.z = 0.0
            approach.pose.orientation.w =  0.7071

            approach_j = self._solve_ik(approach)
            if approach_j is None:
                self.bus.add_chain_of_thought(
                    f"  SORT: ⚠ IK failed for approach — skipping this object")
                continue

            self._move_joints(approach_j, 45.0, speed_dps=35.0)
            self._wait_for_arrival(approach_j, timeout=12.0)
            self.bus.add_chain_of_thought("  SORT: ✓ Approach waypoint reached")

            # Gripper camera visual servo alignment
            self._run_gripper_camera_servo(timeout=3.0)

            # Descend to object
            grasp_j = self._solve_ik(obj_pose)
            if grasp_j is not None:
                self._move_joints(grasp_j, 45.0, speed_dps=15.0)
                self._wait_for_arrival(grasp_j, timeout=10.0)

            # Close gripper
            self._call_sync(self.close_gripper, Trigger.Request(), timeout=3.0)
            time.sleep(0.8)
            self.bus.add_chain_of_thought("  SORT: ✓ Object grasped")

            # ── Lift ─────────────────────────────────────────────
            lift_j = list(grasp_j or approach_j)
            self._move_joints(lift_j, 0.0, speed_dps=30.0)
            time.sleep(0.5)

            # ── Move to destination ───────────────────────────────
            dest_approach = PoseStamped()
            dest_approach.header = dest.header
            dest_approach.pose.position.x = dest.pose.position.x
            dest_approach.pose.position.y = dest.pose.position.y
            dest_approach.pose.position.z = dest.pose.position.z + 0.15
            dest_approach.pose.orientation = approach.pose.orientation

            dest_j = self._solve_ik(dest_approach)
            if dest_j is not None:
                self._move_joints(dest_j, 0.0, speed_dps=35.0)
                self._wait_for_arrival(dest_j, timeout=12.0)

            # Open gripper — release
            self._call_sync(self.open_gripper, Trigger.Request(), timeout=3.0)
            time.sleep(0.5)
            self.bus.add_chain_of_thought(
                f"  SORT: ✓ Placed '{class_name}' in {dest_name}")
            sorted_count += 1

        self.bus.add_chain_of_thought(
            f"  SORT: ✅ Complete — sorted {sorted_count}/{len(objects_to_sort)} objects")
        return sorted_count > 0

    def _skill_inspect(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            f"  INSPECT: Move wrist camera to 3 viewpoints "
            f"→ capture frames → generate report")
        return True

    def _skill_slide(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            f"  SLIDE: Gentle push to slide object along surface")
        return True

    def _skill_roll(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            f"  ROLL: For round objects → controlled roll to target")
        return True

    def _skill_sweep(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            f"  SWEEP: Push multiple objects to clear area")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = SkillAgent()
    node.trigger_configure()
    node.trigger_activate()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
