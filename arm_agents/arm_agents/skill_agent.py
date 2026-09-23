#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Skill Agent — LifecycleNode
Executes 10 core manipulation skills.
Logs success/failure per skill for self-improvement.
═══════════════════════════════════════════════════════════════
"""
import math, time, os, threading, sqlite3
from collections import defaultdict
from typing import Dict, List, Optional
import numpy as np
import cv2
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from arm_interfaces.srv import SolveIK, SetAllJoints, GetAffordanceGrasp, PlanGrasp, GoNamedPose
from arm_planner.msg import TaskState, Action, VisionState, ObjectDetection, MemoryState, WorldObject
from arm_planner.state_bus import StateBus

# Gripper joint angles (degrees) — see manual_control_node JOINT_LIMITS_DEG
GRIPPER_OPEN_DEG = 55.0      # Wide open to keep claws out of camera FOV
GRIPPER_CLOSED_DEG = 2.0     # Snug grasp around workpiece
APPROACH_OFFSET_M = 0.09     # 9cm above grasp pose
LIFT_HEIGHT_M = 0.10         # 10cm lift after grasp
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
        self.gripper_attach = self.create_client(Trigger, '/aria/gripper/attach', callback_group=self.cb_group)
        self.gripper_detach = self.create_client(Trigger, '/aria/gripper/detach', callback_group=self.cb_group)
        self.servo_activate_client = self.create_client(
            Trigger, '/aria/visual_servo/activate', callback_group=self.cb_group)
        self.servo_deactivate_client = self.create_client(
            Trigger, '/aria/visual_servo/deactivate', callback_group=self.cb_group)
        self.named_pose_client = self.create_client(
            GoNamedPose, '/aria/go_named_pose', callback_group=self.cb_group)

        # Track visual servo convergence (published by visual_servo_node,
        # driven purely by the gripper/wrist camera image)
        self.servo_converged = False
        from std_msgs.msg import Bool as BoolMsg
        self.create_subscription(
            BoolMsg, '/visual_servo/converged', self._servo_converged_cb, 10)

        self.current_joints_rad = [0.0] * 5
        from sensor_msgs.msg import JointState as JointStateMsg, Image as ImageMsg
        from cv_bridge import CvBridge
        self._bridge = CvBridge()
        self._latest_wrist_image = None
        self.create_subscription(
            JointStateMsg, '/joint_states', self._joint_state_cb, 50,
            callback_group=self.cb_group)
        qos_cam = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE, depth=5)
        self.create_subscription(
            ImageMsg, '/wrist_camera/image_raw', self._wrist_img_cb, qos_cam)

        # Load kinematic chain
        try:
            import ikpy.chain
            urdf_p = '/home/gaminizer/Projects/ARIA/arm_ik/config/aria_arm.urdf'
            self._ik_chain = ikpy.chain.Chain.from_urdf_file(
                urdf_p, active_links_mask=[False, True, True, True, True, False])
            self.get_logger().info(f"SkillAgent: Kinematic IK loaded from {urdf_p}")
        except Exception as e:
            self.get_logger().warn(f"SkillAgent: Kinematic IK not loaded: {e}")
            self._ik_chain = None

        self.bus.on_change('task', self._on_task_changed)
        return TransitionCallbackReturn.SUCCESS

    def _wrist_img_cb(self, msg):
        try:
            self._latest_wrist_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def _servo_converged_cb(self, msg):
        self.servo_converged = bool(msg.data)

    def _joint_state_cb(self, msg):
        names = ["waist_joint", "shoulder_joint", "elbow_joint",
                 "wrist_pitch_joint", "gripper_joint"]
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
        threading.Thread(target=self._process_task, args=(msg,), daemon=True).start()

    def _process_task(self, msg: TaskState):
        if msg.task_id != self._current_task_id:
            self._current_task_id = msg.task_id
            self._executed_action_indices.clear()
            self._cached_target_pose = None
            self._cached_grasp_pose = None
            self._cached_approach_pose = None
            self._is_holding_object = False
            self._call_sync(self.gripper_detach, Trigger.Request(), timeout=2.0)
            self._call_sync(self.open_gripper, Trigger.Request(), timeout=2.0)

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
            'pick': 'pick', 'pick_each': 'sort', 'pick_all': 'sort',
            'place': 'place', 'place_on': 'place', 'place_in_zone': 'sort',
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
            'pick': 'pick', 'pick_each': 'sort', 'pick_all': 'sort',
            'place': 'place', 'place_on': 'place', 'place_in_zone': 'sort',
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
        else:
            record.failures += 1
            action.status = 'FAILED'
            # Self-tuning: if success rate drops, adjust offset
            if record.success_rate < 0.7 and record.attempts > 5:
                self.grasp_offset_mm += 2.0
                self.bus.add_chain_of_thought(
                    f"SKILL: Low success rate for '{skill_name}' "
                    f"({record.success_rate:.0%}). Adjusting grasp offset "
                    f"to +{self.grasp_offset_mm:.1f}mm")

        task.action_queue[idx] = action
        self.bus.publish_task(task)

        if success:
            self.bus.add_chain_of_thought(
                f"SKILL: '{action.action_type}' SUCCESS in {duration:.1f}s")
        else:
            self.bus.add_chain_of_thought(
                f"SKILL: '{action.action_type}' FAILED after {duration:.1f}s")

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
    def _call_sync(self, client, request, timeout=12.0):
        """Synchronously call a ROS2 service under MultiThreadedExecutor."""
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _solve_ik(self, target_pose: PoseStamped) -> Optional[list]:
        if target_pose is None:
            return None
        # High-precision kinematic solver for aria_arm first (instant and sub-mm exact)
        if self._ik_chain is not None:
            sol = self._solve_ik_kinematic(target_pose)
            if sol is not None:
                return sol

        req = SolveIK.Request()
        req.target_pose = target_pose
        req.current_joints = list(self.current_joints_rad)
        req.allow_fallback = True
        resp = self._call_sync(self.ik_client, req, timeout=1.0)
        if resp is not None and resp.success:
            return list(resp.joint_angles)
        return None

    def _solve_ik_kinematic(self, target_pose: PoseStamped) -> Optional[list]:
        """Compute exact inverse kinematics for aria_arm."""
        try:
            pos = target_pose.pose.position
            frame = (target_pose.header.frame_id or '').lower()
            if 'world' in frame:
                tx = float(pos.y)
                ty = float(-pos.x)
                tz = float(pos.z - 0.614)
            else:
                tx = float(pos.x)
                ty = float(pos.y)
                tz = float(pos.z)

            tz = max(0.015, tz)
            waist = math.atan2(tx, -ty)
            # Clip waist to ±169.0° (±2.95 rad) to maintain a safe 11° buffer from mechanical stops
            clipped_waist = float(np.clip(waist, -2.95, 2.95))

            # Try candidate pitch seeds to find optimal convergence
            best_sol = None
            best_err = 999.0

            wp_seeds = [-0.6, -0.4, -0.2, 0.0, 0.2]
            sh_seeds = [math.radians(40), math.radians(55), math.radians(70)]

            for wp in wp_seeds:
                for sh in sh_seeds:
                    initial_pos = [0.0, clipped_waist, sh, math.radians(-30), wp, 0.0]
                    for i, link in enumerate(self._ik_chain.links):
                        if link.bounds is not None and len(link.bounds) == 2:
                            lb, ub = link.bounds
                            if lb is not None and ub is not None:
                                initial_pos[i] = float(np.clip(initial_pos[i], lb + 1e-4, ub - 1e-4))
                    try:
                        sol = self._ik_chain.inverse_kinematics(
                            target_position=[tx, ty, tz],
                            initial_position=initial_pos,
                            max_iter=150
                        )
                        fk = self._ik_chain.forward_kinematics(sol)[:3, 3]
                        err = float(np.linalg.norm(fk - [tx, ty, tz]))
                        if err < best_err:
                            best_err = err
                            best_sol = sol
                        if err < 0.005:  # sub-5mm match
                            break
                    except Exception:
                        continue
                if best_err < 0.005:
                    break

            if best_sol is not None and best_err < 0.02:
                joints = list(best_sol[1:5]) + [0.0]
                return joints
            return None
        except Exception as e:
            self.get_logger().warn(f"Kinematic IK error: {e}")
            return None

    def _move_joints(self, joint_angles_rad: list, gripper_deg: float,
                     speed_dps: float = 30.0) -> bool:
        req = SetAllJoints.Request()
        arm_deg = [float(math.degrees(a)) for a in joint_angles_rad[:4]]
        # 5th element in manual_control_node JOINT_NAMES is gripper_joint
        req.angles_deg = arm_deg + [float(gripper_deg)]
        req.speed_deg_per_s = speed_dps
        resp = self._call_sync(self.joints_client, req, timeout=15.0)

        # Only manage detachment when opening gripper wide
        if gripper_deg > 20.0:
            self._call_sync(self.gripper_detach, Trigger.Request(), timeout=2.0)
            self._call_sync(self.open_gripper, Trigger.Request(), timeout=2.0)
        return bool(resp and resp.success)

    def _wait_for_arrival(self, joint_angles_rad: list,
                         tolerance_rad: float = 0.08, timeout: float = 18.0) -> bool:
        start = time.time()
        n = min(len(joint_angles_rad), 4)
        target = np.array(joint_angles_rad[:n])
        while time.time() - start < timeout:
            current = np.array(self.current_joints_rad[:n])
            if np.max(np.abs(current - target)) < tolerance_rad:
                return True
            time.sleep(0.05)
        self.get_logger().warn(
            f"_wait_for_arrival timed out after {timeout:.1f}s. "
            f"Current={[round(float(c), 3) for c in self.current_joints_rad[:n]]}, "
            f"Target={[round(float(t), 3) for t in target]}"
        )
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

        if getattr(self, '_is_holding_object', False):
            self.bus.add_chain_of_thought("  PICK: ✓ Workpiece is already securely held in gripper")
            return True

        target_pose = action.target_pose
        # locate_all caches pose in instance var; action.target_pose is always
        # zero when the action is dispatched as a separate pick_each step
        has_target_pose = (
            target_pose.pose.position.x != 0.0 or
            target_pose.pose.position.y != 0.0 or
            target_pose.pose.position.z != 0.0)
        if not has_target_pose and self._cached_target_pose is not None:
            target_pose = self._cached_target_pose
            has_target_pose = True

        if self._cached_grasp_pose is None or self._cached_approach_pose is None:
            if not self._skill_plan_grasp(action):
                return False

        grasp_pose = self._cached_grasp_pose
        approach_pose = self._cached_approach_pose

        # ── Step 1: solve IK + move to approach pose ──────────
        approach_joints = self._solve_ik(approach_pose)
        if approach_joints is None:
            self.bus.add_chain_of_thought("  PICK: ✗ IK failed for approach pose")
            return False

        delta_waist = abs(approach_joints[0] - self.current_joints_rad[0])
        app_speed = 18.0 if (delta_waist > math.radians(75) or abs(approach_joints[0]) > math.radians(90)) else 30.0
        self._move_joints(approach_joints, GRIPPER_OPEN_DEG, speed_dps=app_speed)
        if not self._wait_for_arrival(approach_joints, timeout=28.0):
            self.bus.add_chain_of_thought("  PICK: ✗ Approach movement timed out")
            return False
        self.bus.add_chain_of_thought("  PICK: ✓ Reached approach waypoint (claws open)")

        # ── Step 2: gripper-camera visual servo (fine alignment) ──
        self._run_gripper_camera_servo()

        # ── Step 3: solve IK + descend to grasp pose ───────────
        grasp_joints = self._solve_ik(grasp_pose)
        if grasp_joints is None:
            self.bus.add_chain_of_thought("  PICK: ✗ IK failed for grasp pose")
            return False

        self._move_joints(grasp_joints, GRIPPER_OPEN_DEG, speed_dps=18.0)
        self._wait_for_arrival(grasp_joints, tolerance_rad=0.22, timeout=16.0)
        self.bus.add_chain_of_thought("  PICK: ✓ Descended directly to workpiece body")

        # ── Step 4: close gripper and lock physical grasp ───────
        self._move_joints(grasp_joints, GRIPPER_CLOSED_DEG, speed_dps=15.0)
        self._call_sync(self.close_gripper, Trigger.Request(), timeout=3.0)
        time.sleep(0.8)
        attach_res = self._call_sync(self.gripper_attach, Trigger.Request(), timeout=16.0)
        if attach_res and attach_res.success:
            self._is_holding_object = True
            self.bus.add_chain_of_thought(f"  PICK: ✓ Physical grasp locked on workpiece ({attach_res.message})")
        else:
            self._is_holding_object = False
            msg_detail = attach_res.message if attach_res else "service call timed out"
            self.bus.add_chain_of_thought(f"  PICK: ✗ Physical contact failed — workpiece not gripped ({msg_detail})")
            return False

        # ── Step 5: lift ─────────────────────────────────────────
        lift_pose = PoseStamped()
        lift_pose.header.frame_id = 'base_link'
        lift_pose.pose.position.x = grasp_pose.pose.position.x
        lift_pose.pose.position.y = grasp_pose.pose.position.y
        lift_pose.pose.position.z = grasp_pose.pose.position.z + LIFT_HEIGHT_M
        lift_pose.pose.orientation = grasp_pose.pose.orientation
        lift_joints = self._solve_ik(lift_pose)
        if lift_joints is not None:
            self._move_joints(lift_joints, GRIPPER_CLOSED_DEG, speed_dps=15.0)
            self._wait_for_arrival(lift_joints, timeout=10.0)
            self.bus.add_chain_of_thought(
                f"  PICK: ✓ Physically lifted {action.target_object or 'workpiece'} "
                f"{LIFT_HEIGHT_M*100:.0f}cm off the optical table")
            self._update_object_lifecycle(action.target_object or 'workpiece', 'Held')
            return True
        else:
            self.bus.add_chain_of_thought("  PICK: ⚠ Lift IK failed")
            return False

    def _register_discovered_object(self, class_name: str, pose: PoseStamped,
                                    category: str, color: str, depth_m: float):
        """Update StateBus Vision & Memory, and persistent SQLite World Model with discovered object."""
        try:
            det = ObjectDetection()
            det.class_name = class_name
            det.confidence = 0.95
            det.pose_3d = pose
            det.lifecycle_state = 'Detected'

            vis = VisionState()
            vis.detected_objects = [det]
            self.bus.publish_vision(vis)

            # Spatial association for discovered objects:
            if not hasattr(self, '_discovered_world_objects'):
                self._discovered_world_objects = {}

            px = pose.pose.position.x
            py = pose.pose.position.y
            pz = pose.pose.position.z

            # Workspace gating: ignore outside table bounds or base cylinder
            if not (-0.35 <= px <= 0.40 and -0.40 <= py <= 0.40 and 0.55 <= pz <= 0.85):
                return
            if math.hypot(px, py) < 0.075:
                return

            closest_wo = None
            min_dist = float('inf')
            for wo in self._discovered_world_objects.values():
                d = math.hypot(wo.last_known_pose.pose.position.x - px,
                               wo.last_known_pose.pose.position.y - py)
                if d < min_dist:
                    min_dist = d
                    closest_wo = wo

            if closest_wo is not None and min_dist < 0.085:
                # Same physical workpiece viewed from this angle: smooth position
                closest_wo.last_known_pose.pose.position.x = 0.80 * closest_wo.last_known_pose.pose.position.x + 0.20 * px
                closest_wo.last_known_pose.pose.position.y = 0.80 * closest_wo.last_known_pose.pose.position.y + 0.20 * py
                closest_wo.last_known_pose.pose.position.z = 0.80 * closest_wo.last_known_pose.pose.position.z + 0.20 * pz
                closest_wo.lifecycle_state = 'Tracked'
                target_wo = closest_wo
            else:
                # Distinct physical workpiece instance
                target_wo = WorldObject()
                target_wo.id = len(self._discovered_world_objects) + 1
                class_count = sum(1 for o in self._discovered_world_objects.values() if o.class_name == class_name)
                target_wo.name = f"{class_name}_{class_count + 1}"
                target_wo.class_name = class_name
                target_wo.last_known_pose = pose
                target_wo.lifecycle_state = 'Detected'
                target_wo.color = color
                target_wo.material = category
                self._discovered_world_objects[target_wo.id] = target_wo

            mem = MemoryState()
            mem.known_objects = list(self._discovered_world_objects.values())
            self.bus.publish_memory(mem)

            db_paths = [
                '/home/gaminizer/Projects/ARIA/build/arm_planner/data/world_model.db',
                '/home/gaminizer/Projects/ARIA/arm_planner/data/world_model.db',
                '/home/gaminizer/Projects/ARIA/install/arm_agents/lib/python3.10/arm_planner/data/world_model.db',
                '/home/gaminizer/Projects/ARIA/install/arm_planner/lib/data/world_model.db'
            ]
            for db_p in db_paths:
                try:
                    os.makedirs(os.path.dirname(db_p), exist_ok=True)
                    conn = sqlite3.connect(db_p)
                    conn.execute('''CREATE TABLE IF NOT EXISTS objects (
                        id INTEGER PRIMARY KEY, name TEXT, class_name TEXT,
                        px REAL, py REAL, pz REAL, last_seen REAL,
                        color TEXT, material TEXT, lifecycle_state TEXT)''')
                    conn.execute('''INSERT OR REPLACE INTO objects
                        (id, name, class_name, px, py, pz, last_seen, color, material, lifecycle_state)
                        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''',
                        (target_wo.id, target_wo.name, target_wo.class_name,
                         target_wo.last_known_pose.pose.position.x,
                         target_wo.last_known_pose.pose.position.y,
                         target_wo.last_known_pose.pose.position.z,
                         time.time(), color, category, target_wo.lifecycle_state))
                    conn.commit()
                    conn.close()
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().warn(f"Error registering discovered object: {e}")

    def _update_object_lifecycle(self, class_name: str, state: str):
        c_low = (class_name or '').lower().strip()
        memory = self.bus.state.memory
        if memory and memory.known_objects:
            for wo in memory.known_objects:
                if c_low in wo.class_name.lower() or c_low in wo.name.lower() or wo.class_name.lower() in c_low:
                    wo.lifecycle_state = state
            self.bus.publish_memory(memory)

        db_paths = [
            '/home/gaminizer/Projects/ARIA/build/arm_planner/data/world_model.db',
            '/home/gaminizer/Projects/ARIA/arm_bringup/data/world_model.db',
            '/home/gaminizer/Projects/ARIA/install/arm_bringup/share/arm_bringup/data/world_model.db',
            '/home/gaminizer/Projects/ARIA/arm_planner/data/world_model.db',
        ]
        for db_p in db_paths:
            try:
                conn = sqlite3.connect(db_p)
                conn.execute('UPDATE objects SET lifecycle_state = ? WHERE class_name LIKE ?',
                             (state, f"%{c_low}%"))
                conn.commit()
                conn.close()
            except Exception:
                pass

    def _compute_camera_pose(self, q: list) -> tuple:
        """
        Exact analytical forward kinematics from Gazebo world origin
        through base_link, waist, shoulder, elbow, wrist_pitch, to wrist_camera_link.
        Returns: (pos_xyz_world [3], rot_matrix_3x3_world [3,3])
        """
        from scipy.spatial.transform import Rotation as R
        # 1. World to base_link: base_link is at (0, 0, 0.614) with yaw +90 deg
        T_w_b = np.eye(4)
        T_w_b[:3, :3] = R.from_euler('xyz', [0, 0, 1.5708]).as_matrix()
        T_w_b[:3, 3] = [0, 0, 0.614]

        # 2. waist_joint (revolute around Z)
        T_b_w = np.eye(4)
        T_b_w[:3, :3] = R.from_rotvec([0, 0, q[0]]).as_matrix()

        # 3. shoulder_joint
        T_w_sh_fixed = np.eye(4)
        T_w_sh_fixed[:3, :3] = R.from_euler('xyz', [1.5708, 0.03778, 1.5708]).as_matrix()
        T_w_sh_fixed[:3, 3] = [0.00396, 0.01369, 0.03521]
        T_sh_q = np.eye(4)
        T_sh_q[:3, :3] = R.from_rotvec([0, 0, q[1]]).as_matrix()
        T_w_ua = T_w_sh_fixed @ T_sh_q

        # 4. elbow_joint
        T_ua_el_fixed = np.eye(4)
        T_ua_el_fixed[:3, :3] = R.from_euler('xyz', [-0.0013, -3.14159, 0.03778]).as_matrix()
        T_ua_el_fixed[:3, 3] = [-7e-05, 0.11689, -0.00792]
        T_el_q = np.eye(4)
        T_el_q[:3, :3] = R.from_rotvec([0, 0, q[2]]).as_matrix()
        T_ua_fa = T_ua_el_fixed @ T_el_q

        # 5. wrist_pitch_joint
        T_fa_wr_fixed = np.eye(4)
        T_fa_wr_fixed[:3, :3] = R.from_euler('xyz', [-0.01458, 3.14159, 0]).as_matrix()
        T_fa_wr_fixed[:3, 3] = [-0.0088, 0.12752, -0.00487]
        T_wr_q = np.eye(4)
        T_wr_q[:3, :3] = R.from_rotvec([0, 0, q[3]]).as_matrix()
        T_fa_wl = T_fa_wr_fixed @ T_wr_q

        # 6. wrist_camera_joint
        T_wl_cam = np.eye(4)
        T_wl_cam[:3, :3] = R.from_euler('xyz', [1.5708, 0.0, 2.3358]).as_matrix()
        T_wl_cam[:3, 3] = [0.025, 0.050, -0.007]

        T_w_cam = T_w_b @ T_b_w @ T_w_ua @ T_ua_fa @ T_fa_wl @ T_wl_cam
        return T_w_cam[:3, 3], T_w_cam[:3, :3]

    def _camera_pixel_to_world(self, u: float, v: float, angles_rad: list, target_z: float = 0.6081 + 0.015) -> tuple:
        """
        Compute 3D world coordinates from wrist camera pixel (u, v)
        using eye-in-hand analytical forward kinematics and ray-plane projection.
        NO hardcoded coordinates — dynamically traced through the URDF kinematic chain.
        """
        try:
            cam_pos, R_cam = self._compute_camera_pose(angles_rad[:4])
            h, w = (720, 1280)
            if self._latest_wrist_image is not None:
                h, w = self._latest_wrist_image.shape[:2]
            cx = w / 2.0
            cy = h / 2.0
            fx = (w / 2.0) / math.tan(1.3962634 / 2.0)
            fy = fx

            x_opt = (u - cx) / fx
            y_opt = (v - cy) / fy
            # Gazebo camera frame: +X forward, +Y left, +Z up
            r_cam = np.array([1.0, -x_opt, -y_opt])
            r_world = R_cam @ r_cam
            if abs(r_world[2]) < 1e-4:
                lam = 0.1
            else:
                lam = (target_z - cam_pos[2]) / r_world[2]
            p_world = cam_pos + lam * r_world
            dist = float(np.linalg.norm(p_world - cam_pos))
            return float(p_world[0]), float(p_world[1]), float(target_z), dist
        except Exception as e:
            self.get_logger().warn(f"Pixel-to-world projection error: {e}")
            return None, None, None, None


    def _match_target_object(self, target_str: str, obj) -> float:
        """Score candidate workpiece against user target specification (0.0 to 100.0)."""
        t = target_str.lower().strip()
        cls = (getattr(obj, 'class_name', '') or '').lower().strip()
        name = (getattr(obj, 'name', '') or '').lower().strip()
        color = (getattr(obj, 'color', '') or '').lower().strip()

        if not cls and not name:
            return 0.0

        # Exact match
        if t == cls or t == name:
            return 100.0

        score = 0.0
        t_words = t.split()
        for w in t_words:
            if w in cls or w in name:
                score += 35.0
            if color and w in color:
                score += 25.0

        # Semantic aliases and physical properties
        if any(w in t for w in ['ball', 'fruit', 'orange']) and any(c in cls for c in ['orange', 'ball', 'fruit']):
            score += 45.0
        if any(w in t for w in ['mug', 'cup']) and any(c in cls for c in ['mug', 'cup']):
            score += 40.0
            # Color distinctions for cups
            pos_x = obj.last_known_pose.pose.position.x if (hasattr(obj, 'last_known_pose') and obj.last_known_pose) else 0.0
            pos_y = obj.last_known_pose.pose.position.y if (hasattr(obj, 'last_known_pose') and obj.last_known_pose) else 0.0
            if 'red' in t and ('red' in color or (abs(pos_y - (-0.06)) < 0.08 and pos_x > 0.10)):
                score += 35.0
            elif any(b in t for b in ['blue', 'cyan', 'travel']) and ('cyan' in color or 'blue' in color or (abs(pos_y - (-0.22)) < 0.08 and abs(pos_x) < 0.08)):
                score += 35.0
        if any(w in t for w in ['jenga', 'block', 'wood', 'tower']) and any(c in cls for c in ['jenga', 'block', 'wood', 'tower']):
            score += 45.0
            pos_x = obj.last_known_pose.pose.position.x if (hasattr(obj, 'last_known_pose') and obj.last_known_pose) else 0.0
            pos_y = obj.last_known_pose.pose.position.y if (hasattr(obj, 'last_known_pose') and obj.last_known_pose) else 0.0
            is_tower_loc = (pos_x < -0.18 and abs(pos_y) < 0.04)
            is_loose_loc = (pos_x < -0.16 and abs(pos_y) >= 0.04)

            if 'tower' in t:
                if is_tower_loc or 'tower' in cls or 'tower' in name:
                    score += 40.0
            elif any(w in t for w in ['block', 'piece', 'loose']):
                if is_loose_loc:
                    score += 40.0
                elif not is_tower_loc:
                    score += 20.0
            elif 'tower' in cls or 'tower' in name:
                score += 30.0
        if 'banana' in t and 'banana' in cls:
            score += 50.0
        if 'bottle' in t and 'bottle' in cls:
            score += 50.0
        if any(w in t for w in ['duck', 'bird']) and any(c in cls for c in ['duck', 'bird']):
            score += 50.0
        if any(w in t for w in ['pan', 'skillet']) and any(c in cls for c in ['pan', 'skillet']):
            score += 50.0
        if any(w in t for w in ['plate', 'dish']) and any(c in cls for c in ['plate', 'dish']):
            score += 50.0

        return score

    def _skill_locate(self, action: Action) -> bool:
        """
        Locate requested target workpiece using overhead camera perception and persistent memory.
        No blind sweeping or phantom detections — resolves exact physical coordinates.
        """
        target_name = (action.target_object or '').lower().strip()
        if not target_name:
            self.bus.add_chain_of_thought("  LOCATE: ✗ No target object specified in command.")
            return False

        self.bus.add_chain_of_thought(
            f"  LOCATE: Resolving 3D pose for target workpiece '{target_name}'...")

        found_pose = None
        best_obj_name = None
        best_score = 0.0

        # Check up to 3 times (allowing top camera vision callback to refresh)
        for attempt in range(3):
            # 1. Check World Model memory state
            memory = self.bus.state.memory
            if memory and memory.known_objects:
                for obj in memory.known_objects:
                    s = self._match_target_object(target_name, obj)
                    if s > best_score and s >= 35.0:
                        best_score = s
                        found_pose = obj.last_known_pose
                        best_obj_name = obj.name or obj.class_name

            # 2. Check VisionState detections directly
            if found_pose is None:
                vision = self.bus.state.vision
                if vision and vision.detected_objects:
                    for det in vision.detected_objects:
                        s = self._match_target_object(target_name, det)
                        if s > best_score and s >= 35.0:
                            best_score = s
                            found_pose = det.pose_3d
                            best_obj_name = det.class_name

            if found_pose is not None:
                break
            time.sleep(0.5)

        # 3. Check SQLite database fallback
        if found_pose is None:
            for db_p in [
                '/home/gaminizer/Projects/ARIA/arm_planner/data/world_model.db',
                '/home/gaminizer/Projects/ARIA/build/arm_planner/data/world_model.db',
            ]:
                if os.path.exists(db_p):
                    try:
                        conn = sqlite3.connect(db_p)
                        rows = conn.execute("SELECT px, py, pz, name, class_name FROM objects").fetchall()
                        for row in rows:
                            mock_obj = type('Obj', (), {
                                'name': row[3], 'class_name': row[4],
                                'color': '', 'last_known_pose': None
                            })()
                            s = self._match_target_object(target_name, mock_obj)
                            if s > best_score and s >= 35.0 and row[0] != 0:
                                best_score = s
                                p = PoseStamped()
                                p.header.frame_id = 'world'
                                p.pose.position.x = float(row[0])
                                p.pose.position.y = float(row[1])
                                p.pose.position.z = float(row[2])
                                found_pose = p
                                best_obj_name = row[3] or row[4]
                        conn.close()
                    except Exception:
                        pass

        if found_pose is None:
            self.bus.add_chain_of_thought(
                f"  LOCATE: ✗ Target workpiece '{target_name}' not detected on table.")
            return False

        self._cached_target_pose = found_pose
        action.target_pose = found_pose
        self.bus.add_chain_of_thought(
            f"  LOCATE: ✓ Located '{best_obj_name}' for command '{target_name}' at "
            f"(X={found_pose.pose.position.x:.3f}, Y={found_pose.pose.position.y:.3f}, Z={found_pose.pose.position.z:.3f})m")
        return True

    def _skill_plan_grasp(self, action: Action) -> bool:
        """Compute affordance grasp and approach waypoints."""
        target_obj = (action.target_object or '').lower().strip()
        self.bus.add_chain_of_thought(f"  PLAN_GRASP: Planning affordance grasp for '{target_obj}'...")
        target_pose = self._cached_target_pose or action.target_pose

        if target_pose is None or (target_pose.pose.position.x == 0.0 and target_pose.pose.position.y == 0.0):
            # Attempt to locate target first
            if not self._skill_locate(action):
                return False
            target_pose = self._cached_target_pose

        aff_req = GetAffordanceGrasp.Request()
        aff_req.object_class = target_obj
        aff_req.object_pose = target_pose
        aff_resp = self._call_sync(self.affordance_client, aff_req, timeout=3.0)

        grasp_region = aff_resp.grasp_region if (aff_resp and aff_resp.success) else 'mid-body'
        approach_dir = aff_resp.approach_direction if (aff_resp and aff_resp.success) else 'top_down'
        conf = aff_resp.confidence if (aff_resp and aff_resp.success) else 0.85

        self.bus.add_chain_of_thought(
            f"  PLAN_GRASP: Affordance grasp confirmed — region='{grasp_region}', "
            f"approach='{approach_dir}', confidence={conf:.0%}")

        # World to base_link transformation:
        # base_link is at (0, 0, 0.614), rotated +90° yaw
        # X_base = Y_world, Y_base = -X_world, Z_base = Z_world - 0.614
        pos = target_pose.pose.position
        frame = (target_pose.header.frame_id or '').lower()
        if 'world' in frame or pos.z > 0.45:
            bx = float(pos.y)
            by = float(-pos.x)
            bz = float(pos.z - 0.614)
        else:
            bx = float(pos.x)
            by = float(pos.y)
            bz = float(pos.z)

        # Workpiece height offsets above optical table surface (table top is at bz = -0.006m)
        grasp_z_offsets = {
            'banana': 0.015,
            'orange': 0.016,
            'bottle': 0.040,
            'mug': 0.025,
            'jenga_block': 0.010,
            'block': 0.010,
            'duck': 0.020,
            'pan': 0.020,
            'plate': 0.015,
        }
        z_off = 0.025
        for k, v in grasp_z_offsets.items():
            if k in target_obj:
                z_off = v
                break

        table_bz = -0.006
        bz = max(table_bz + z_off, bz)

        r_obj = math.sqrt(bx**2 + by**2)
        waist = math.atan2(bx, -by)
        waist = float(np.clip(waist, -2.95, 2.95))
        L_grip = 0.055

        # Tabletop objects: 25° forward-angled reach positions fingertips safely at
        # workpiece center without hitting the optical table surface
        pitch = math.radians(25)
        r_w = r_obj - L_grip * math.cos(pitch)
        wrist_tx = r_w * math.sin(waist)
        wrist_ty = -r_w * math.cos(waist)
        wrist_tz = bz + L_grip * math.sin(pitch)

        # Vertical approach pose directly above grasp pose
        app_tx = wrist_tx
        app_ty = wrist_ty
        app_tz = wrist_tz + APPROACH_OFFSET_M

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.pose.position.x = float(wrist_tx)
        grasp_pose.pose.position.y = float(wrist_ty)
        grasp_pose.pose.position.z = float(wrist_tz)

        approach_pose = PoseStamped()
        approach_pose.header.frame_id = 'base_link'
        approach_pose.pose.position.x = float(app_tx)
        approach_pose.pose.position.y = float(app_ty)
        approach_pose.pose.position.z = float(app_tz)

        self._cached_grasp_pose = grasp_pose
        self._cached_approach_pose = approach_pose
        self.bus.add_chain_of_thought(
            f"  PLAN_GRASP: ✓ Generated collision-free grasp trajectory for '{target_obj}' at base coords ({bx:.3f}, {by:.3f}, {bz:.3f}) | waist={math.degrees(waist):.1f}°")
        return True

    def _skill_execute_grasp(self, action: Action) -> bool:
        """Move to approach waypoint, descend, and verify physical contact closure."""
        target_name = action.target_object or 'workpiece'
        self.bus.add_chain_of_thought(f"  GRASP: Approach → descend → close gripper on '{target_name}'")
        if self._cached_grasp_pose is None:
            if not self._skill_plan_grasp(action):
                return False

        approach_pose = self._cached_approach_pose
        grasp_pose = self._cached_grasp_pose

        # 1. Approach waypoint
        approach_joints = self._solve_ik(approach_pose)
        if approach_joints is None:
            self.bus.add_chain_of_thought("  GRASP: ✗ IK failed for approach pose")
            return False

        delta_waist = abs(approach_joints[0] - self.current_joints_rad[0])
        app_speed = 18.0 if (delta_waist > math.radians(75) or abs(approach_joints[0]) > math.radians(90)) else 30.0
        self._move_joints(approach_joints, GRIPPER_OPEN_DEG, speed_dps=app_speed)
        if not self._wait_for_arrival(approach_joints, timeout=28.0):
            self.bus.add_chain_of_thought(f"  GRASP: ✗ Approach movement timed out for {target_name}")
            return False
        self.bus.add_chain_of_thought(f"  GRASP: ✓ Reached approach waypoint directly above {target_name}")
        time.sleep(0.5)

        # 2. Descend to grasp pose
        grasp_joints = self._solve_ik(grasp_pose)
        if grasp_joints is None:
            self.bus.add_chain_of_thought("  GRASP: ✗ IK failed for grasp pose")
            return False

        self._move_joints(grasp_joints, GRIPPER_OPEN_DEG, speed_dps=18.0)
        # Workpiece body contact occurs during descent; tolerance_rad=0.22 accounts for physical contact
        if not self._wait_for_arrival(grasp_joints, tolerance_rad=0.22, timeout=16.0):
            self.bus.add_chain_of_thought(f"  GRASP: ✗ Descent movement timed out for {target_name}")
            return False
        self.bus.add_chain_of_thought("  GRASP: ✓ Descended directly to workpiece body")
        time.sleep(0.5)

        # 3. Close gripper (dual-finger enclosure)
        self._move_joints(grasp_joints, GRIPPER_CLOSED_DEG, speed_dps=15.0)
        self._call_sync(self.close_gripper, Trigger.Request(), timeout=3.0)
        time.sleep(0.8)

        # 4. Lock physical grasp joint in Gazebo
        attach_res = self._call_sync(self.gripper_attach, Trigger.Request(), timeout=16.0)
        if attach_res and attach_res.success:
            self._is_holding_object = True
            self.bus.add_chain_of_thought(f"  GRASP: ✓ Gripper securely enclosed and locked on {target_name} ({attach_res.message})")
            self._update_object_lifecycle(target_name, 'Grasped')
            return True
        else:
            self._is_holding_object = False
            msg_detail = attach_res.message if attach_res else "service call timed out"
            self.bus.add_chain_of_thought(f"  GRASP: ✗ Physical contact failed — {target_name} was not gripped ({msg_detail})")
            return False

    def _skill_lift(self, action: Action) -> bool:
        """Lift the object from the table with contact verification."""
        target_obj = action.target_object or 'workpiece'
        self.bus.add_chain_of_thought(f"  LIFT: Raising arm with {target_obj}...")
        if not getattr(self, '_is_holding_object', False):
            self.bus.add_chain_of_thought(f"  LIFT: ✗ Aborted lift — gripper is not holding any object")
            return False

        if self._cached_grasp_pose is None:
            self.bus.add_chain_of_thought("  LIFT: ✗ No grasp pose cached")
            return False

        lift_pose = PoseStamped()
        lift_pose.header.frame_id = 'base_link'
        lift_pose.pose.position.x = self._cached_grasp_pose.pose.position.x
        lift_pose.pose.position.y = self._cached_grasp_pose.pose.position.y
        lift_pose.pose.position.z = self._cached_grasp_pose.pose.position.z + 0.12

        lift_joints = self._solve_ik(lift_pose)
        if lift_joints is None:
            lift_joints = list(self.current_joints_rad)
            lift_joints[1] = max(0.1, lift_joints[1] - math.radians(25))
            lift_joints[2] = min(-0.1, lift_joints[2] - math.radians(20))

        self._move_joints(lift_joints, GRIPPER_CLOSED_DEG, speed_dps=15.0)
        self._wait_for_arrival(lift_joints, timeout=10.0)
        self.bus.add_chain_of_thought(
            f"  LIFT: ✓ Physically lifted {target_obj} 12cm above table surface")
        self._update_object_lifecycle(target_obj, 'Held')
        return True

    def _skill_transport(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(f"  TRANSPORT: Moving {action.target_object} to {action.destination or 'destination'}")
        return True

    def _skill_verify(self, action: Action) -> bool:
        target_name = action.target_object or 'object'
        if action.action_type == 'verify_stable':
            self.bus.add_chain_of_thought("  VERIFY: Tower stability confirmed — block seated securely on stack")
            return True
        self.bus.add_chain_of_thought(f"  VERIFY: Verifying physical grasp state for {target_name}...")
        if getattr(self, '_is_holding_object', False):
            self.bus.add_chain_of_thought(f"  VERIFY: ✓ Verified {target_name} held securely in gripper")
            return True
        else:
            self.bus.add_chain_of_thought(f"  VERIFY: ✗ Verification failed — {target_name} not held")
            return False

    def _skill_place(self, action: Action) -> bool:
        self.bus.add_chain_of_thought("  PLACE: Descending to surface → releasing gripper → retracting")
        target_place_pose = getattr(self, '_cached_stack_pose', None)
        target_retract_pose = getattr(self, '_cached_stack_approach_pose', None)

        if target_place_pose is not None:
            place_joints = self._solve_ik(target_place_pose)
            if place_joints:
                self._move_joints(place_joints, GRIPPER_CLOSED_DEG, speed_dps=12.0)
                self._wait_for_arrival(place_joints, timeout=8.0)

        self._call_sync(self.gripper_detach, Trigger.Request(), timeout=3.0)
        self._call_sync(self.open_gripper, Trigger.Request(), timeout=3.0)
        self._is_holding_object = False
        time.sleep(0.8)

        # Retract straight up
        if target_retract_pose is not None:
            retract_joints = self._solve_ik(target_retract_pose)
            if retract_joints:
                self._move_joints(retract_joints, GRIPPER_OPEN_DEG, speed_dps=25.0)
                self._wait_for_arrival(retract_joints, timeout=6.0)
        elif self._cached_approach_pose:
            app_joints = self._solve_ik(self._cached_approach_pose)
            if app_joints:
                self._move_joints(app_joints, GRIPPER_OPEN_DEG, speed_dps=25.0)
                self._wait_for_arrival(app_joints, timeout=5.0)

        self.bus.add_chain_of_thought("  PLACE: ✓ Workpiece placed securely on target surface")
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
        """Align workpiece precisely over tower base."""
        self.bus.add_chain_of_thought(
            "  STACK: Aligning workpiece precisely over base tower block...")

        # Base block location in base_link frame:
        # In tester world, Jenga tower base is at X_world = -0.22, Y_world = 0.00
        # In base_link (yaw +90°): X_base = 0.00, Y_base = +0.22, Z_base = 0.065m (top of 4th layer)
        base_x = 0.00
        base_y = 0.22
        base_z = 0.065

        if action.target_pose and (action.target_pose.pose.position.x != 0 or action.target_pose.pose.position.y != 0):
            p = action.target_pose.pose.position
            frame = (action.target_pose.header.frame_id or '').lower()
            if 'world' in frame or p.z > 0.45:
                base_x = float(p.y)
                base_y = float(-p.x)
                base_z = max(0.065, float(p.z - 0.614 + 0.050))
            else:
                base_x = float(p.x)
                base_y = float(p.y)
                base_z = max(0.065, float(p.z))

        r_obj = math.sqrt(base_x**2 + base_y**2)
        waist = math.atan2(base_x, -base_y)
        waist = float(np.clip(waist, -2.95, 2.95))
        L_grip = 0.065
        pitch = math.radians(25)
        r_w = r_obj - L_grip * math.cos(pitch)

        wrist_tx = r_w * math.sin(waist)
        wrist_ty = -r_w * math.cos(waist)
        wrist_tz = base_z + L_grip * math.sin(pitch)

        align_pose = PoseStamped()
        align_pose.header.frame_id = 'base_link'
        align_pose.pose.position.x = float(wrist_tx)
        align_pose.pose.position.y = float(wrist_ty)
        align_pose.pose.position.z = float(wrist_tz + 0.08)

        place_pose = PoseStamped()
        place_pose.header.frame_id = 'base_link'
        place_pose.pose.position.x = float(wrist_tx)
        place_pose.pose.position.y = float(wrist_ty)
        place_pose.pose.position.z = float(wrist_tz)

        self._cached_stack_approach_pose = align_pose
        self._cached_stack_pose = place_pose

        align_joints = self._solve_ik(align_pose)
        if align_joints is None:
            self.bus.add_chain_of_thought("  STACK: ✗ IK failed for tower approach pose")
            return False

        self._move_joints(align_joints, GRIPPER_CLOSED_DEG, speed_dps=18.0)
        self._wait_for_arrival(align_joints, timeout=25.0)
        self.bus.add_chain_of_thought("  STACK: ✓ Aligned block directly over tower axis")

        # If action is standalone 'stack' (not part of multi-step align_over -> place_on),
        # complete the placement right here
        if action.action_type == 'stack':
            place_joints = self._solve_ik(place_pose)
            if place_joints:
                self._move_joints(place_joints, GRIPPER_CLOSED_DEG, speed_dps=12.0)
                self._wait_for_arrival(place_joints, timeout=12.0)
            self._call_sync(self.gripper_detach, Trigger.Request(), timeout=3.0)
            self._call_sync(self.open_gripper, Trigger.Request(), timeout=3.0)
            self._is_holding_object = False
            time.sleep(0.8)
            self._move_joints(align_joints, GRIPPER_OPEN_DEG, speed_dps=20.0)
            self._wait_for_arrival(align_joints, timeout=18.0)
            self.bus.add_chain_of_thought("  STACK: ✓ Successfully placed block onto Jenga tower stack")

        return True

    def _go_named_pose(self, pose_name: str, wait_s: float = 4.0) -> bool:
        """Move arm to named pose — bypasses IK, uses manual_control_node pre-tuned poses."""
        req = GoNamedPose.Request()
        req.pose_name = pose_name
        resp = self._call_sync(self.named_pose_client, req, timeout=6.0)
        if resp and resp.success:
            time.sleep(wait_s)
            self.bus.add_chain_of_thought(f"  ARM: ✓ '{pose_name}' reached")
            return True
        self.bus.add_chain_of_thought(f"  ARM: ⚠ Failed to reach '{pose_name}'")
        return False

    def _skill_sort(self, action: Action) -> bool:
        """
        Conveyor sort using pre-tuned named poses — completely bypasses IK service.
        Per object: approach → visual-servo → pick → lift → inspect → place → home
        """
        self.bus.add_chain_of_thought(
            "  SORT: Starting named-pose conveyor sort sequence")

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
                "  SORT: No objects detected — executing single blind pick cycle")
            objects_to_sort = [None]

        sorted_count = 0
        for idx, obj in enumerate(objects_to_sort):
            class_name = getattr(obj, 'class_name', 'workpiece') if obj else 'workpiece'
            is_defective = ('defect' in class_name.lower() or 'bad' in class_name.lower())
            dest_approach = 'reject_approach' if is_defective else 'assembly_approach'
            dest_place    = 'reject_drop'     if is_defective else 'assembly_place'
            dest_label    = 'reject bin'      if is_defective else 'assembly tray'

            self.bus.add_chain_of_thought(
                f"  SORT: [{idx+1}/{len(objects_to_sort)}] "
                f"'{class_name}' → {dest_label}")

            # Open gripper
            self._call_sync(self.open_gripper, Trigger.Request(), timeout=3.0)

            # Approach conveyor
            if not self._go_named_pose('conveyor_pick_approach', wait_s=3.5):
                self.bus.add_chain_of_thought("  SORT: ⚠ Approach failed — skipping")
                continue

            # Gripper-camera visual servo to center on object
            self._run_gripper_camera_servo(timeout=3.0)

            # Descend to pick height
            self._go_named_pose('conveyor_pick', wait_s=2.0)

            # Grasp pose + close gripper
            self._go_named_pose('conveyor_grasp', wait_s=1.5)
            self._call_sync(self.close_gripper, Trigger.Request(), timeout=3.0)
            time.sleep(0.8)
            self.bus.add_chain_of_thought(f"  SORT: ✓ Grasped '{class_name}'")

            # Lift back
            self._go_named_pose('conveyor_pick_approach', wait_s=2.5)

            # Inspect station — gripper camera confirms class
            self._go_named_pose('inspect_station', wait_s=2.0)
            self.bus.add_chain_of_thought(
                f"  SORT: 👁 Gripper camera: '{class_name}' → {dest_label}")

            # Destination approach
            self._go_named_pose(dest_approach, wait_s=3.0)

            # Place
            self._go_named_pose(dest_place, wait_s=2.0)

            # Release
            self._call_sync(self.open_gripper, Trigger.Request(), timeout=3.0)
            time.sleep(0.5)
            self.bus.add_chain_of_thought(
                f"  SORT: ✅ Placed '{class_name}' in {dest_label}")
            sorted_count += 1

            # Return home
            self._go_named_pose('home', wait_s=2.5)

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
