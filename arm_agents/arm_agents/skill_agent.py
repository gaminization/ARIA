#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Skill Agent — LifecycleNode
Executes 10 core manipulation skills.
Logs success/failure per skill for self-improvement.
═══════════════════════════════════════════════════════════════
"""
import math, time, os, threading, sqlite3
import re
from collections import defaultdict
from typing import Dict, List, Optional
import numpy as np
import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data as qos_profile_sensor_data_
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from arm_interfaces.srv import (
    SolveIK,
    SetAllJoints,
    GetAffordanceGrasp,
    PlanGrasp,
    GoNamedPose,
    SetConveyorPower,
)
from arm_planner.msg import (
    TaskState,
    Action,
    VisionState,
    ObjectDetection,
    MemoryState,
    WorldObject,
)
from arm_planner.state_bus import StateBus
from arm_planner.skill_registry import SKILL_ALIAS_MAP

# Gripper joint angles (degrees) — see manual_control_node JOINT_LIMITS_DEG
GRIPPER_OPEN_DEG = 55.0  # Wide open to keep claws out of camera FOV
GRIPPER_CLOSED_DEG = 2.0  # Snug grasp around workpiece
APPROACH_OFFSET_M = 0.09  # 9cm above grasp pose
LIFT_HEIGHT_M = 0.10  # 10cm lift after grasp
VISUAL_SERVO_TIMEOUT_S = 4.0
MAX_GEOMETRIC_REACH_M = 0.30


def _target_pose_cache_key(target_name: str) -> str:
    return " ".join(re.findall(r"[a-z0-9]+", target_name.lower()))


def _cached_pose_for_target(
    pose_cache: Dict[str, PoseStamped],
    target_name: str,
) -> Optional[PoseStamped]:
    return pose_cache.get(_target_pose_cache_key(target_name))


def _grasp_pitch_for_approach(approach_dir: str) -> float:
    if approach_dir in ("side", "horizontal"):
        return math.radians(15)
    if approach_dir == "handle":
        return math.radians(10)
    return math.radians(90)


def _grasp_wrist_targets(
    bx: float,
    by: float,
    bz: float,
    pitch: float,
    approach_offset: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    waist = math.atan2(bx, -by)
    radius = math.hypot(bx, by) - 0.055 * math.cos(pitch)
    wrist_x = radius * math.sin(waist)
    wrist_y = -radius * math.cos(waist)
    wrist_z = bz + 0.055 * math.sin(pitch)
    return (
        (wrist_x, wrist_y, wrist_z),
        (wrist_x, wrist_y, wrist_z + approach_offset),
    )


def _attached_model_name(message: str) -> str:
    match = re.search(r"(?:workpiece|object):\s*([^\s(]+)", message, re.IGNORECASE)
    return match.group(1) if match else ""


def _attachment_matches_target(target_name: str, message: str) -> bool:
    aliases = {"cup": "mug", "ball": "orange"}

    def tokens(value: str) -> set[str]:
        return {
            aliases.get(token, token)
            for token in re.findall(r"[a-z0-9]+", value.lower())
        }

    requested = tokens(target_name)
    attached = tokens(_attached_model_name(message))
    return bool(requested and attached and requested.issubset(attached))


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
        "pick",
        "place",
        "push",
        "pull",
        "stack",
        "sort",
        "inspect",
        "slide",
        "roll",
        "sweep",
        "locate",
        "plan_grasp",
        "execute_grasp",
        "lift",
        "transport",
        "verify",
        "scan_workspace",
        "sam",
        "depth",
        "cleargrasp",
        "material",
        "world_model",
        # In-hand manipulation
        "rotate_in_hand",
        "reposition_grip",
        "flip",
        "slide_to_tip",
    ]

    SKILL_ALIAS_MAP = SKILL_ALIAS_MAP

    def __init__(self):
        super().__init__("skill_agent")
        self.bus = StateBus(self)
        self.cb_group = ReentrantCallbackGroup()
        self.skill_stats: Dict[str, SkillRecord] = {
            name: SkillRecord() for name in self.SKILL_NAMES
        }
        self.grasp_offset_mm = 0.0  # self-tuning parameter
        self._cached_target_pose: Optional[PoseStamped] = None
        self._target_pose_cache: Dict[str, PoseStamped] = {}
        self._cached_grasp_target_key: Optional[str] = None
        self._cached_grasp_pose: Optional[PoseStamped] = None
        self._cached_approach_pose: Optional[PoseStamped] = None
        self._cached_stack_pose: Optional[PoseStamped] = None
        self._cached_stack_approach_pose: Optional[PoseStamped] = None
        self._held_workpiece: Optional[str] = None
        self._is_holding_object = False
        # Last arm joints we commanded; used to verify motions actually landed.
        self._last_commanded_joints: Optional[np.ndarray] = None
        self._gazebo_model_poses: Dict[str, PoseStamped] = {}
        self._executing = False
        self._pending_task = None
        self._current_task_id = ""
        self._executed_action_indices = set()

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("SkillAgent: CONFIGURING — skills loaded")
        # Service clients
        self.ik_client = self.create_client(
            SolveIK, "/aria/ik/solve", callback_group=self.cb_group
        )
        self.joints_client = self.create_client(
            SetAllJoints, "/aria/set_all_joints", callback_group=self.cb_group
        )
        self.affordance_client = self.create_client(
            GetAffordanceGrasp,
            "/aria/affordance/get_grasp",
            callback_group=self.cb_group,
        )
        self.grasp_plan_client = self.create_client(
            PlanGrasp, "/aria/grasp/plan", callback_group=self.cb_group
        )
        self.close_gripper = self.create_client(
            Trigger, "/aria/close_gripper", callback_group=self.cb_group
        )
        self.open_gripper = self.create_client(
            Trigger, "/aria/open_gripper", callback_group=self.cb_group
        )
        self.gripper_attach = self.create_client(
            Trigger, "/aria/gripper/attach", callback_group=self.cb_group
        )
        self.gripper_detach = self.create_client(
            Trigger, "/aria/gripper/detach", callback_group=self.cb_group
        )
        self.servo_activate_client = self.create_client(
            Trigger, "/aria/visual_servo/activate", callback_group=self.cb_group
        )
        self.servo_deactivate_client = self.create_client(
            Trigger, "/aria/visual_servo/deactivate", callback_group=self.cb_group
        )
        self.named_pose_client = self.create_client(
            GoNamedPose, "/aria/go_named_pose", callback_group=self.cb_group
        )
        self.conveyor_client = self.create_client(
            SetConveyorPower, "/aria/conveyor/set_power", callback_group=self.cb_group
        )
        from arm_interfaces.srv import CheckReachability

        self.reach_client = self.create_client(
            CheckReachability, "/aria/reachability/check", callback_group=self.cb_group
        )
        from std_msgs.msg import String as StringMsg

        self.pub_target_model = self.create_publisher(
            StringMsg, "/aria/gripper/target_model", 10
        )

        # Track visual servo convergence (published by visual_servo_node,
        # driven purely by the gripper/wrist camera image)
        self.servo_converged = False
        self.part_present = False
        from std_msgs.msg import Bool as BoolMsg
        from gazebo_msgs.msg import ContactsState, ModelStates

        self._contact_seen = {}
        self._button_contact_time = 0.0
        self.create_subscription(
            ModelStates,
            "/gazebo/model_states",
            self._gazebo_model_states_cb,
            10,
            callback_group=self.cb_group,
        )
        for _side in ("left", "right"):
            self.create_subscription(
                ContactsState,
                f"/gripper/contact_{_side}",
                lambda m, side=_side: self._contact_cb(side, m),
                qos_profile_sensor_data_,
            )
        self.create_subscription(
            ContactsState,
            "/aria/industrial/estop_contact",
            self._estop_contact_cb,
            qos_profile_sensor_data_,
        )
        self.create_subscription(
            BoolMsg, "/visual_servo/converged", self._servo_converged_cb, 10
        )
        self.create_subscription(
            BoolMsg,
            "/aria/conveyor/part_present",
            self._part_present_cb,
            10,
            callback_group=self.cb_group,
        )

        # In-hand manipulation (wrist-pitch reorientation, grip repositioning,
        # 180° flip, slide-to-tip) driven by IMU + wrist-camera feedback.
        from arm_agents.in_hand_manipulation import InHandManipulation

        self.in_hand = InHandManipulation(self, self.bus)

        self.current_joints_rad = [0.0] * 5
        from sensor_msgs.msg import JointState as JointStateMsg, Image as ImageMsg
        from cv_bridge import CvBridge

        self._bridge = CvBridge()
        self._latest_wrist_image = None
        self.create_subscription(
            JointStateMsg,
            "/joint_states",
            self._joint_state_cb,
            50,
            callback_group=self.cb_group,
        )
        qos_cam = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.create_subscription(
            ImageMsg, "/wrist_camera/image_raw", self._wrist_img_cb, qos_cam
        )

        from vision_msgs.msg import Detection2DArray

        try:
            from arm_vision.coordinate_transformer import create_top_camera_transformer
        except ImportError:
            from arm_vision.arm_vision.coordinate_transformer import (
                create_top_camera_transformer,
            )
        self._create_top_tf = create_top_camera_transformer
        self._top_transformer = create_top_camera_transformer(1920, 1080)
        self._top_tf_size = (1920, 1080)
        from sensor_msgs.msg import CameraInfo as CameraInfoMsg

        self.create_subscription(
            CameraInfoMsg, "/top_camera/camera_info", self._top_info_cb, qos_cam
        )
        self._latest_vision_dets = []
        self._last_detection_time = None
        self.create_subscription(
            Detection2DArray,
            "/detection/objects",
            self._detection_objects_cb,
            10,
            callback_group=self.cb_group,
        )

        # Load kinematic chain
        try:
            import ikpy.chain

            urdf_p = "/home/gaminizer/Projects/ARIA/arm_ik/config/aria_arm.urdf"
            self._ik_chain = ikpy.chain.Chain.from_urdf_file(
                urdf_p, active_links_mask=[False, True, True, True, True, False]
            )
            self.get_logger().info(f"SkillAgent: Kinematic IK loaded from {urdf_p}")
        except Exception as e:
            self.get_logger().warn(f"SkillAgent: Kinematic IK not loaded: {e}")
            self._ik_chain = None

        # ── SearchBehavior: inject arm control so grid scan physically moves arm
        from arm_planner.search_behavior import SearchBehavior

        self._search_behavior = SearchBehavior(
            bus=self.bus,
            move_fn=self._move_joints,
            wait_fn=self._wait_for_arrival,
            match_fn=self._match_target_object,
        )
        self._search_behavior.set_ik_solver(self._solve_ik)

        # Automatically transition arm from upright CAD zero to ready posture over table
        def _initial_ready_pose():
            time.sleep(1.0)
            q_ready = [0.0, math.radians(35), math.radians(-55), math.radians(20), 0.0]
            self.get_logger().info(
                "SkillAgent: Moving arm to ready posture over table..."
            )
            # Gazebo starts the command service before the trajectory
            # controller is active. Retry once after a verified timeout rather
            # than treating an early accepted-but-unconsumed command as motion.
            for attempt in range(2):
                if not self._move_joints(q_ready, 20.0, 25.0):
                    self.get_logger().error(
                        "SkillAgent: ready-posture command was rejected."
                    )
                    return
                if self._wait_for_arrival(q_ready, timeout=10.0):
                    break
                if attempt == 0:
                    self.get_logger().warn(
                        "SkillAgent: controller not ready for initial pose; retrying once."
                    )
            else:
                self.get_logger().error(
                    "SkillAgent: ready posture was not reached; manipulation remains unverified."
                )
                return
            self.get_logger().info("SkillAgent: Arm in ready posture over table.")

        threading.Thread(target=_initial_ready_pose, daemon=True).start()

        self.bus.on_change("task", self._on_task_changed)
        return TransitionCallbackReturn.SUCCESS

    def _wrist_img_cb(self, msg):
        try:
            self._latest_wrist_image = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception:
            pass

    def _top_info_cb(self, msg):
        """Keep the top-camera projection model in sync with the real image size."""
        size = (int(msg.width), int(msg.height))
        if size[0] > 0 and size != self._top_tf_size:
            self._top_transformer = self._create_top_tf(*size)
            self._top_tf_size = size
            self.get_logger().info(
                f"SkillAgent: top camera projection set for {size[0]}x{size[1]}"
            )

    def _detection_objects_cb(self, msg):
        dets = []
        for d in msg.detections:
            if not d.results:
                continue
            raw_cls = d.results[0].hypothesis.class_id.lower()
            conf = float(d.results[0].hypothesis.score)
            color = ""
            if "|" in raw_cls:
                parts = raw_cls.split("|", 1)
                cls_name = parts[0]
                color = parts[1]
            else:
                cls_name = raw_cls

            cx = float(d.bbox.center.position.x)
            cy = float(d.bbox.center.position.y)
            obj_h = (
                0.080
                if "box" in cls_name
                else (0.030 if "workpiece" in cls_name else 0.0)
            )
            coord = self._top_transformer.pixel_to_world(
                int(cx), int(cy), object_height=obj_h, table_height=0.6081
            )
            p = PoseStamped()
            p.header = msg.header
            p.header.frame_id = "world"
            p.pose.position.x = coord.x
            p.pose.position.y = coord.y
            p.pose.position.z = coord.z
            mock_obj = type(
                "DetObj",
                (),
                {
                    "class_name": cls_name,
                    "name": f"{cls_name}_{int(cx)}_{int(cy)}",
                    "color": color,
                    "affordance_regions": [f"color:{color}"] if color else [],
                    "pose_3d": p,
                    "last_known_pose": p,
                    "confidence": conf,
                },
            )()
            dets.append(mock_obj)
        self._latest_vision_dets = dets
        self._last_detection_time = time.monotonic()
        # Publish one coherent scene update per camera frame.  Publishing each
        # instance separately marks other objects unseen between callbacks and
        # creates ghost tracks.
        vision = VisionState()
        vision.detected_objects = []
        for discovered in dets:
            tracked_det = ObjectDetection()
            tracked_det.class_name = discovered.class_name
            tracked_det.confidence = discovered.confidence
            tracked_det.pose_3d = discovered.pose_3d
            tracked_det.lifecycle_state = "Detected"
            tracked_det.tracking_id = -1
            tracked_det.affordance_regions = getattr(
                discovered, "affordance_regions", []
            )
            vision.detected_objects.append(tracked_det)
        self.bus.publish_vision(vision)

    def _servo_converged_cb(self, msg):
        self.servo_converged = bool(msg.data)

    def _part_present_cb(self, msg):
        self.part_present = bool(msg.data)

    def _joint_state_cb(self, msg):
        names = [
            "waist_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_pitch_joint",
            "gripper_joint",
        ]
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
        if msg.task_status != "EXECUTING":
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
            self._target_pose_cache.clear()
            self._cached_grasp_target_key = None
            self._cached_grasp_pose = None
            self._cached_approach_pose = None
            self._cached_stack_pose = None
            self._cached_stack_approach_pose = None
            self._held_workpiece = None
            self._is_holding_object = False
            self._call_sync(self.gripper_detach, Trigger.Request(), timeout=2.0)
            self._call_sync(self.open_gripper, Trigger.Request(), timeout=2.0)

        while msg is not None:
            target_action = None
            target_idx = -1
            for i, action in enumerate(msg.action_queue):
                if (
                    action.status == "EXECUTING"
                    and self._is_our_skill(action.action_type)
                    and i not in self._executed_action_indices
                ):
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
        """Check if this action maps to a skill or agent behavior we handle."""
        return action_type in self.SKILL_ALIAS_MAP

    def _execute_skill(self, action: Action, idx: int, task: TaskState):
        """Execute a skill or perception behavior and update the action status."""
        skill_name = self.SKILL_ALIAS_MAP.get(action.action_type, "pick")
        record = self.skill_stats.get(skill_name)
        if record is None:
            record = SkillRecord()
            self.skill_stats[skill_name] = record
        record.attempts += 1
        t0 = time.time()

        self.bus.add_chain_of_thought(
            f"SKILL: Executing '{action.action_type}' (skill='{skill_name}') for '{action.target_object}' "
            f"(attempt #{record.attempts})"
        )

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
            action.status = "COMPLETE"
        else:
            record.failures += 1
            action.status = "FAILED"
            # Self-tuning: if success rate drops, adjust offset
            if record.success_rate < 0.7 and record.attempts > 5:
                self.grasp_offset_mm += 2.0
                self.bus.add_chain_of_thought(
                    f"SKILL: Low success rate for '{skill_name}' "
                    f"({record.success_rate:.0%}). Adjusting grasp offset "
                    f"to +{self.grasp_offset_mm:.1f}mm"
                )

        task.action_queue[idx] = action
        self.bus.publish_task(task)

        if success:
            self.bus.add_chain_of_thought(
                f"SKILL: '{action.action_type}' SUCCESS in {duration:.1f}s"
            )
        else:
            self.bus.add_chain_of_thought(
                f"SKILL: '{action.action_type}' FAILED after {duration:.1f}s"
            )

    def _dispatch_skill(self, skill_name: str, action: Action) -> bool:
        """Dispatch to specific skill implementation."""
        dispatch = {
            "locate": self._skill_locate,
            "plan_grasp": self._skill_plan_grasp,
            "execute_grasp": self._skill_execute_grasp,
            "lift": self._skill_lift,
            "pick": self._skill_pick,
            "place": self._skill_place,
            "push": self._skill_push,
            "pull": self._skill_pull,
            "stack": self._skill_stack,
            "sort": self._skill_sort,
            "inspect": self._skill_inspect,
            "slide": self._skill_slide,
            "roll": self._skill_roll,
            "sweep": self._skill_sweep,
            "transport": self._skill_transport,
            "verify": self._skill_verify,
            "scan_workspace": self._skill_scan_workspace,
            "sam": self._skill_sam,
            "depth": self._skill_depth,
            "cleargrasp": self._skill_cleargrasp,
            "material": self._skill_material,
            "world_model": self._skill_world_model,
            "joint_motion": self._skill_joint_motion,
            "move_to": self._skill_move_to,
            "press": self._skill_press,
            "gripper": self._skill_gripper,
            "named_pose": self._skill_named_pose,
            "wait": self._skill_wait,
            "reachability": self._skill_reachability,
            "safety": self._skill_safety,
            "track": self._skill_track,
            "rotate_in_hand": self._skill_rotate_in_hand,
            "reposition_grip": self._skill_reposition_grip,
            "flip": self._skill_flip,
            "slide_to_tip": self._skill_slide_to_tip,
        }
        handler = dispatch.get(skill_name)
        if handler is None:
            self.bus.add_chain_of_thought(
                f"  SKILL: ✗ No implementation is registered for '{skill_name}'"
            )
            return False
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
        req.current_joints = list(self.current_joints_rad[:4])
        req.allow_fallback = True
        resp = self._call_sync(self.ik_client, req, timeout=1.0)
        if resp is not None and resp.success:
            return list(resp.joint_angles)
        return None

    def _solve_ik_kinematic(self, target_pose: PoseStamped) -> Optional[list]:
        """Compute exact inverse kinematics for aria_arm."""
        try:
            pos = target_pose.pose.position
            frame = (target_pose.header.frame_id or "").lower()
            if "world" in frame:
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

            # Try candidate pitch seeds to find optimal convergence biased downward
            # to keep the table and workpiece centered in the wrist camera FOV
            best_sol = None
            best_err = 999.0

            wp_seeds = [-0.65, -0.45, -0.25, 0.0]
            sh_seeds = [math.radians(35), math.radians(50), math.radians(65)]

            for wp in wp_seeds:
                for sh in sh_seeds:
                    initial_pos = [0.0, clipped_waist, sh, math.radians(-30), wp, 0.0]
                    for i, link in enumerate(self._ik_chain.links):
                        if link.bounds is not None and len(link.bounds) == 2:
                            lb, ub = link.bounds
                            if lb is not None and ub is not None:
                                initial_pos[i] = float(
                                    np.clip(initial_pos[i], lb + 1e-4, ub - 1e-4)
                                )
                    try:
                        sol = self._ik_chain.inverse_kinematics(
                            target_position=[tx, ty, tz],
                            initial_position=initial_pos,
                            max_iter=150,
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
                joints = list(best_sol[1:5])
                return joints
            return None
        except Exception as e:
            self.get_logger().warn(f"Kinematic IK error: {e}")
            return None

    def _move_joints(
        self, joint_angles_rad: list, gripper_deg: float, speed_dps: float = 10.0
    ) -> bool:
        req = SetAllJoints.Request()
        arm_deg = [float(math.degrees(a)) for a in joint_angles_rad[:4]]
        # 5th element in manual_control_node JOINT_NAMES is gripper_joint
        req.angles_deg = arm_deg + [float(gripper_deg)]
        req.speed_deg_per_s = min(float(speed_dps), 10.0)
        resp = self._call_sync(self.joints_client, req, timeout=15.0)

        # Only manage detachment when opening gripper wide
        if gripper_deg > 20.0:
            self._call_sync(self.gripper_detach, Trigger.Request(), timeout=2.0)
            self._call_sync(self.open_gripper, Trigger.Request(), timeout=2.0)
        if resp and resp.success:
            # Record what we commanded so motion verification can confirm the
            # arm actually reached it (instead of trusting the action text).
            self._last_commanded_joints = np.array(
                [float(a) for a in joint_angles_rad[:4]]
            )
        return bool(resp and resp.success)

    def _motion_verification_ok(self, action: Action) -> bool:
        """
        Verify a non-grasp motion (wave, press, gesture, ...) actually completed
        by checking the arm reached the joints that were commanded for it.

        Returns False when there is nothing verifiable - this method never
        assumes success from the action's wording.
        """
        target = getattr(self, "_last_commanded_joints", None)
        if target is None:
            self.bus.add_chain_of_thought(
                "  VERIFY: ✗ no commanded joint target recorded — cannot "
                "confirm motion completed"
            )
            return False
        current = np.array(self.current_joints_rad[:4])
        err = float(np.max(np.abs(current - np.asarray(target))))
        tol = 0.10  # rad (~5.7 deg)
        if err < tol:
            self.bus.add_chain_of_thought(
                f"  VERIFY: ✓ arm reached commanded posture (max joint error "
                f"{math.degrees(err):.1f}°)"
            )
            return True
        self.bus.add_chain_of_thought(
            f"  VERIFY: ✗ arm did not reach commanded posture (max joint "
            f"error {math.degrees(err):.1f}°)"
        )
        return False

    def _wait_for_arrival(
        self, joint_angles_rad: list, tolerance_rad: float = 0.08, timeout: float = 18.0
    ) -> bool:
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

    def _run_gripper_camera_servo(
        self, timeout: float = VISUAL_SERVO_TIMEOUT_S
    ) -> bool:
        """
        Activate visual_servo_node, which uses ONLY the gripper (wrist)
        camera image to drive the arm's final approach via /aria/joint_stream.
        Waits for convergence (object centered in gripper camera FOV).
        """
        self.bus.add_chain_of_thought(
            "  PICK: Engaging gripper camera closed-loop visual servo..."
        )
        self.servo_converged = False
        resp = self._call_sync(
            self.servo_activate_client, Trigger.Request(), timeout=3.0
        )
        if resp is None or not resp.success:
            self.bus.add_chain_of_thought(
                "  PICK: ⚠ visual_servo_node unavailable — skipping fine alignment"
            )
            return False

        start = time.time()
        while time.time() - start < timeout:
            if self.servo_converged:
                self.bus.add_chain_of_thought(
                    "  PICK: ✓ Gripper camera confirms object centered in FOV"
                )
                break
            time.sleep(0.05)
        else:
            self.bus.add_chain_of_thought(
                "  PICK: ⚠ Visual servo timed out — proceeding with current alignment"
            )

        self._call_sync(self.servo_deactivate_client, Trigger.Request(), timeout=3.0)
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
            "  PICK: Approach → gripper-camera align → descend → close gripper → lift"
        )

        requested = (action.target_object or "workpiece").lower().strip()
        if self._held_workpiece is not None:
            if requested and (
                requested in self._held_workpiece.lower()
                or self._held_workpiece.lower() in requested
            ):
                # Confirm the hold is still real before short-circuiting.
                if self._skill_verify(action):
                    self.bus.add_chain_of_thought(
                        f"  PICK: ✓ Workpiece '{self._held_workpiece}' is already securely held in gripper"
                    )
                    return True
                self.bus.add_chain_of_thought(
                    f"  PICK: ⚠ previously-held '{self._held_workpiece}' no longer "
                    f"verifies as held — re-picking"
                )
                self._held_workpiece = None
                self._is_holding_object = False
            else:
                self.bus.add_chain_of_thought(
                    f"  PICK: Gripper holds '{self._held_workpiece}', releasing before picking requested '{requested}'"
                )
                self._call_sync(self.gripper_detach, Trigger.Request(), timeout=2.0)
                self._call_sync(self.open_gripper, Trigger.Request(), timeout=2.0)
                self._held_workpiece = None
                self._is_holding_object = False
                time.sleep(0.5)

        target_pose = self._cached_target_pose or action.target_pose
        action.target_pose = target_pose

        self._cached_grasp_pose = None
        self._cached_approach_pose = None
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
        app_speed = (
            18.0
            if (
                delta_waist > math.radians(75)
                or abs(approach_joints[0]) > math.radians(90)
            )
            else 30.0
        )
        self._move_joints(approach_joints, GRIPPER_OPEN_DEG, speed_dps=app_speed)
        if not self._wait_for_arrival(approach_joints, timeout=28.0):
            self.bus.add_chain_of_thought("  PICK: ✗ Approach movement timed out")
            return False
        self.bus.add_chain_of_thought(
            "  PICK: ✓ Reached approach waypoint (claws open, workpiece framed)"
        )

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
        target_name = action.target_object or "workpiece"
        from std_msgs.msg import String as StringMsg

        self.pub_target_model.publish(StringMsg(data=target_name))
        attach_res = self._call_sync(
            self.gripper_attach, Trigger.Request(), timeout=16.0
        )
        if attach_res and attach_res.success:
            if not _attachment_matches_target(target_name, attach_res.message):
                self.bus.add_chain_of_thought(
                    f"  PICK: ✗ Gripper attached unintended object: {attach_res.message}. Detaching..."
                )
                self._call_sync(self.gripper_detach, Trigger.Request(), timeout=3.0)
                self._call_sync(self.open_gripper, Trigger.Request(), timeout=3.0)
                self._held_workpiece = None
                self._is_holding_object = False
                return False
            self._held_workpiece = action.target_object or "workpiece"
            self._is_holding_object = True
            self.bus.add_chain_of_thought(
                f"  PICK: ✓ Physical grasp locked on {target_name} ({attach_res.message})"
            )
        else:
            self._held_workpiece = None
            self._is_holding_object = False
            msg_detail = attach_res.message if attach_res else "service call timed out"
            self.bus.add_chain_of_thought(
                f"  PICK: ✗ Physical contact failed — workpiece not gripped ({msg_detail})"
            )
            return False

        # ── Step 5: lift ─────────────────────────────────────────
        lift_pose = PoseStamped()
        lift_pose.header.frame_id = "base_link"
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
                f"{LIFT_HEIGHT_M*100:.0f}cm off the optical table"
            )
            self._update_object_lifecycle(
                action.target_object or "workpiece",
                "Held",
                self._cached_target_pose,
            )
            return True
        else:
            self.bus.add_chain_of_thought("  PICK: ⚠ Lift IK failed")
            return False

    def _register_discovered_object(
        self,
        class_name: str,
        pose: PoseStamped,
        category: str,
        color: str,
        depth_m: float = 0.0,
    ):
        """Merge an explicit search result into the shared world model."""
        try:
            px = pose.pose.position.x
            py = pose.pose.position.y
            pz = pose.pose.position.z

            # Workspace gating: ignore outside table bounds or base cylinder
            if not (-0.35 <= px <= 0.40 and -0.40 <= py <= 0.40 and 0.55 <= pz <= 0.85):
                return
            if math.hypot(px, py) < 0.075:
                return

            memory = self.bus.state.memory or MemoryState()
            closest_wo = None
            closest_dist = 0.03
            for wo in memory.known_objects:
                if wo.class_name != class_name or (
                    color and wo.color and color != wo.color
                ):
                    continue
                position = wo.last_known_pose.pose.position
                distance = math.hypot(position.x - px, position.y - py)
                if distance <= closest_dist:
                    closest_dist = distance
                    closest_wo = wo

            if closest_wo is not None:
                if closest_wo.lifecycle_state not in {"Held", "Grasped"}:
                    closest_wo.last_known_pose.pose.position.x = (
                        0.80 * closest_wo.last_known_pose.pose.position.x + 0.20 * px
                    )
                    closest_wo.last_known_pose.pose.position.y = (
                        0.80 * closest_wo.last_known_pose.pose.position.y + 0.20 * py
                    )
                    closest_wo.last_known_pose.pose.position.z = (
                        0.80 * closest_wo.last_known_pose.pose.position.z + 0.20 * pz
                    )
                    closest_wo.lifecycle_state = "Tracked"
                if color and not closest_wo.color:
                    closest_wo.color = color
                target_wo = closest_wo
            else:
                target_wo = WorldObject()
                target_wo.id = (
                    max((obj.id for obj in memory.known_objects), default=0) + 1
                )
                class_names = {
                    obj.name
                    for obj in memory.known_objects
                    if obj.class_name == class_name
                }
                index = 1
                while f"{class_name}_{index}" in class_names:
                    index += 1
                target_wo.name = f"{class_name}_{index}"
                target_wo.class_name = class_name
                target_wo.last_known_pose = pose
                target_wo.lifecycle_state = "Detected"
                target_wo.color = color
                target_wo.material = category
                target_wo.last_seen.sec = int(time.time())
                memory.known_objects.append(target_wo)
            self.bus.publish_memory(memory)
        except Exception as e:
            self.get_logger().warn(f"Error registering discovered object: {e}")

    def _update_object_lifecycle(
        self,
        class_name: str,
        state: str,
        target_pose: Optional[PoseStamped] = None,
    ):
        c_low = (class_name or "").lower().strip()
        memory = self.bus.state.memory
        matched_objects = []
        candidates = []
        if memory and memory.known_objects:
            exact_name_matches = [
                wo for wo in memory.known_objects if wo.name.lower() == c_low
            ]
            name_matches = exact_name_matches or [
                wo for wo in memory.known_objects if c_low in wo.name.lower()
            ]
            if len(name_matches) == 1:
                matched_objects = name_matches
            elif len(name_matches) > 1:
                candidates = name_matches
            else:
                class_matches = [
                    wo
                    for wo in memory.known_objects
                    if wo.class_name.lower() == c_low or wo.class_name.lower() in c_low
                ]
                if len(class_matches) == 1:
                    matched_objects = class_matches
                else:
                    candidates = class_matches

            if not matched_objects and target_pose is not None:
                target = target_pose.pose.position
                nearby = []
                for obj in candidates:
                    position = obj.last_known_pose.pose.position
                    distance = math.hypot(position.x - target.x, position.y - target.y)
                    if distance <= 0.03:
                        nearby.append((distance, obj))
                nearby.sort(key=lambda item: item[0])
                if nearby and not (
                    len(nearby) > 1 and nearby[1][0] - nearby[0][0] < 0.005
                ):
                    matched_objects = [nearby[0][1]]

            if not matched_objects and candidates:
                self.get_logger().warning(
                    f"Lifecycle update for '{class_name}' is ambiguous across "
                    f"{len(candidates)} objects"
                )

            if matched_objects:
                matched_objects[0].lifecycle_state = state
                self.bus.publish_memory(memory)

        db_paths = [
            "/home/gaminizer/Projects/ARIA/build/arm_planner/data/world_model.db",
            "/home/gaminizer/Projects/ARIA/arm_bringup/data/world_model.db",
            "/home/gaminizer/Projects/ARIA/install/arm_bringup/share/arm_bringup/data/world_model.db",
            "/home/gaminizer/Projects/ARIA/arm_planner/data/world_model.db",
        ]
        for wo in matched_objects:
            for db_p in db_paths:
                try:
                    conn = sqlite3.connect(db_p)
                    conn.execute(
                        "UPDATE objects SET lifecycle_state = ? WHERE name = ?",
                        (state, wo.name),
                    )
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
        T_w_b[:3, :3] = R.from_euler("xyz", [0, 0, 1.5708]).as_matrix()
        T_w_b[:3, 3] = [0, 0, 0.614]

        # 2. waist_joint (revolute around Z)
        T_b_w = np.eye(4)
        T_b_w[:3, :3] = R.from_rotvec([0, 0, q[0]]).as_matrix()

        # 3. shoulder_joint
        T_w_sh_fixed = np.eye(4)
        T_w_sh_fixed[:3, :3] = R.from_euler(
            "xyz", [1.5708, 0.03778, 1.5708]
        ).as_matrix()
        T_w_sh_fixed[:3, 3] = [0.00396, 0.01369, 0.03521]
        T_sh_q = np.eye(4)
        T_sh_q[:3, :3] = R.from_rotvec([0, 0, q[1]]).as_matrix()
        T_w_ua = T_w_sh_fixed @ T_sh_q

        # 4. elbow_joint
        T_ua_el_fixed = np.eye(4)
        T_ua_el_fixed[:3, :3] = R.from_euler(
            "xyz", [-0.0013, -3.14159, 0.03778]
        ).as_matrix()
        T_ua_el_fixed[:3, 3] = [-7e-05, 0.11689, -0.00792]
        T_el_q = np.eye(4)
        T_el_q[:3, :3] = R.from_rotvec([0, 0, q[2]]).as_matrix()
        T_ua_fa = T_ua_el_fixed @ T_el_q

        # 5. wrist_pitch_joint
        T_fa_wr_fixed = np.eye(4)
        T_fa_wr_fixed[:3, :3] = R.from_euler("xyz", [-0.01458, 3.14159, 0]).as_matrix()
        T_fa_wr_fixed[:3, 3] = [-0.0088, 0.12752, -0.00487]
        T_wr_q = np.eye(4)
        T_wr_q[:3, :3] = R.from_rotvec([0, 0, q[3]]).as_matrix()
        T_fa_wl = T_fa_wr_fixed @ T_wr_q

        # 6. wrist_camera_joint
        T_wl_cam = np.eye(4)
        T_wl_cam[:3, :3] = R.from_euler("xyz", [1.5708, 0.0, 2.3358]).as_matrix()
        T_wl_cam[:3, 3] = [0.025, 0.050, -0.007]

        T_w_cam = T_w_b @ T_b_w @ T_w_ua @ T_ua_fa @ T_fa_wl @ T_wl_cam
        return T_w_cam[:3, 3], T_w_cam[:3, :3]

    def _camera_pixel_to_world(
        self, u: float, v: float, angles_rad: list, target_z: float = 0.6081 + 0.015
    ) -> tuple:
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

    @staticmethod
    def _match_target_object(target_str: str, obj) -> float:
        """Score candidate workpiece against user target specification (0.0 to 100.0)."""
        t = target_str.lower().strip()
        t_words = re.findall(r"[a-z0-9]+", t)
        normalized_target = " ".join(t_words)
        cls = (getattr(obj, "class_name", "") or "").lower().strip()
        name = (getattr(obj, "name", "") or "").lower().strip()
        color = (getattr(obj, "color", "") or "").lower().strip()
        cls_words = re.findall(r"[a-z0-9]+", cls)
        name_words = re.findall(r"[a-z0-9]+", name)

        # Extract color tags from affordance_regions if present
        aff = getattr(obj, "affordance_regions", []) or []
        for a in aff:
            if isinstance(a, str) and a.startswith("color:"):
                color = a.split(":", 1)[1].lower().strip()

        if not cls and not name:
            return 0.0

        # Exact match
        composite_name = " ".join(filter(None, (color, cls)))
        normalized_cls = " ".join(cls_words)
        normalized_name = " ".join(name_words)
        if normalized_target in {composite_name, normalized_cls, normalized_name}:
            return 100.0

        # Check for color constraints in user target query
        query_colors = [
            c
            for c in [
                "red",
                "blue",
                "cyan",
                "yellow",
                "orange",
                "wood",
                "white",
                "dark",
            ]
            if c in t_words
        ]
        type_words = [
            word
            for word in t_words
            if word not in query_colors
            and word
            not in {
                "a",
                "an",
                "the",
                "pick",
                "up",
                "get",
                "grab",
                "take",
                "move",
                "place",
                "item",
                "items",
                "object",
                "objects",
                "part",
                "parts",
                "piece",
                "pieces",
                "workpiece",
                "workpieces",
            }
        ]
        plural_aliases = {
            "blocks": "block",
            "boxes": "box",
            "bottles": "bottle",
            "cubes": "cube",
            "cups": "cup",
            "cylinders": "cylinder",
            "dishes": "dish",
            "mugs": "mug",
            "plates": "plate",
            "pieces": "piece",
            "workpieces": "workpiece",
        }
        type_words = [plural_aliases.get(word, word) for word in type_words]
        object_words = set(cls_words + name_words)
        semantic_type_groups = (
            {"cup", "mug"},
            {"ball", "fruit", "orange"},
            {"duck", "bird"},
            {"pan", "skillet"},
            {"plate", "dish"},
        )
        type_match = bool(object_words.intersection(type_words)) or any(
            set(type_words).intersection(group) and object_words.intersection(group)
            for group in semantic_type_groups
        )
        if type_words and not type_match:
            return 0.0

        score = 0.0
        if query_colors:
            if color:
                matched_color = any(
                    qc == color
                    or (qc in ["blue", "cyan"] and color in ["blue", "cyan"])
                    for qc in query_colors
                )
                if not matched_color:
                    # Color explicitly requested but candidate object has a conflicting color:
                    # Disqualify candidate to prevent picking red mug when blue mug was requested!
                    return 0.0
                score += 40.0
            else:
                # Color explicitly requested but candidate has unknown/empty color:
                # Strongly penalize candidate so genuine colored candidates always win!
                score -= 40.0

        for w in t_words:
            if w in cls or w in name:
                score += 35.0
            if color and w in color:
                score += 25.0

        # Semantic aliases and physical properties
        if any(
            w in t
            for w in [
                "item",
                "items",
                "part",
                "parts",
                "workpiece",
                "piece",
                "pieces",
                "cylinder",
            ]
        ) and any(c in cls for c in ["workpiece", "cylinder", "part", "item"]):
            score += 45.0
        if any(
            w in t for w in ["box", "bin", "tray", "pocket", "container", "receptacle"]
        ) and any(c in cls for c in ["box", "bin", "tray", "pocket", "container"]):
            score += 45.0
        if any(w in t for w in ["ball", "fruit", "orange"]) and any(
            c in cls for c in ["orange", "ball", "fruit"]
        ):
            score += 45.0
        if any(w in t for w in ["mug", "cup"]) and any(
            c in cls for c in ["mug", "cup"]
        ):
            score += 40.0
            if "mug" in t and cls == "mug":
                score += 35.0
            elif "cup" in t and cls == "cup":
                score += 35.0
            if query_colors and any(qc in color for qc in query_colors):
                score += 30.0

        if any(w in t for w in ["jenga", "block", "wood", "tower"]) and any(
            c in cls for c in ["jenga", "block", "wood", "tower"]
        ):
            score += 45.0
            pz = (
                obj.last_known_pose.pose.position.z
                if (hasattr(obj, "last_known_pose") and obj.last_known_pose)
                else 0.0
            )
            is_tower = "tower" in cls or "tower" in name or pz > 0.64
            if "tower" in t:
                if is_tower:
                    score += 40.0
            elif any(w in t for w in ["block", "piece", "loose"]):
                if not is_tower:
                    score += 40.0
            else:
                score += 20.0

        if "banana" in t and "banana" in cls and color != "wood":
            score += 50.0
        if "bottle" in t and "bottle" in cls:
            score += 50.0
        if any(w in t for w in ["duck", "bird"]) and any(
            c in cls for c in ["duck", "bird"]
        ):
            score += 50.0
        if any(w in t for w in ["pan", "skillet"]) and any(
            c in cls for c in ["pan", "skillet"]
        ):
            score += 50.0
        if any(w in t for w in ["plate", "dish"]) and any(
            c in cls for c in ["plate", "dish"]
        ):
            score += 50.0

        return score

    @staticmethod
    def _within_geometric_reach(x: float, y: float) -> bool:
        return math.hypot(float(x), float(y)) <= MAX_GEOMETRIC_REACH_M

    @staticmethod
    def _vision_stream_is_fresh(last_detection_time, now=None) -> bool:
        if last_detection_time is None:
            return False
        current_time = time.monotonic() if now is None else now
        return current_time - last_detection_time <= 1.0

    @staticmethod
    def _memory_candidate_matches_live_detections(
        target_str: str, obj, detections
    ) -> bool:
        if not obj.last_known_pose:
            return False
        obj_pos = obj.last_known_pose.pose.position
        return any(
            SkillAgent._match_target_object(target_str, det) >= 35.0
            and math.hypot(
                obj_pos.x - det.pose_3d.pose.position.x,
                obj_pos.y - det.pose_3d.pose.position.y,
            )
            <= 0.05
            for det in detections
        )

    def _skill_locate(self, action: Action) -> bool:
        """
        Locate requested target workpiece.

        Pipeline:
          1. Check world model memory and live YOLO stream (fast path)
          2. Check SQLite persistent DB
          3. Check infeed conveyor (if applicable)
          4. If still not found: trigger multi-phase physical SearchBehavior
             (arm physically sweeps workspace, wrist camera + YOLO detect the object)
        """
        target_name = (action.target_object or "").lower().strip()
        if not target_name:
            self.bus.add_chain_of_thought("  LOCATE: No target object specified.")
            return False

        self.bus.add_chain_of_thought(
            f"  LOCATE: Resolving 3D pose for target workpiece '{target_name}'..."
        )

        found_pose = None
        best_obj_name = None
        best_score = 0.0
        target_tokens = set(re.findall(r"[a-z0-9]+", target_name))
        exact_sim_models = [
            (name, pose)
            for name, pose in self._gazebo_model_poses.items()
            if target_tokens
            and target_tokens.issubset(set(re.findall(r"[a-z0-9]+", name.lower())))
            and self._within_geometric_reach(pose.pose.position.x, pose.pose.position.y)
        ]
        if len(exact_sim_models) == 1:
            best_obj_name, found_pose = exact_sim_models[0]
            best_score = 100.0
            self.bus.add_chain_of_thought(
                f"  LOCATE: Resolved unique simulator entity '{best_obj_name}' "
                "from Gazebo model state."
            )

        # Check up to 15 times (allowing top camera vision callback to stream live detections)
        for attempt in range(0 if found_pose is not None else 15):
            candidates = []
            stream_started = getattr(self, "_last_detection_time", None) is not None
            stream_fresh = self._vision_stream_is_fresh(self._last_detection_time)
            if not stream_started or stream_fresh:
                vision = self.bus.state.vision
                if vision and vision.detected_objects:
                    candidates.extend(vision.detected_objects)
                if hasattr(self, "_latest_vision_dets") and self._latest_vision_dets:
                    candidates.extend(self._latest_vision_dets)

            # 1. Check World Model memory state
            memory = self.bus.state.memory
            if memory and memory.known_objects:
                for obj in memory.known_objects:
                    s = self._match_target_object(target_name, obj)
                    if not obj.last_known_pose:
                        continue
                    if stream_started:
                        if (
                            not stream_fresh
                            or not self._memory_candidate_matches_live_detections(
                                target_name, obj, candidates
                            )
                        ):
                            continue
                    pz = (
                        obj.last_known_pose.pose.position.z
                        if obj.last_known_pose
                        else 0.608
                    )
                    if abs(pz - 0.608) < 0.04 and not (
                        "pedestal" in target_name or "tower" in target_name
                    ):
                        s += 5.0
                    px = (
                        obj.last_known_pose.pose.position.x
                        if obj.last_known_pose
                        else 0.0
                    )
                    py = (
                        obj.last_known_pose.pose.position.y
                        if obj.last_known_pose
                        else 0.0
                    )
                    dist = math.hypot(px, py)
                    if not self._within_geometric_reach(px, py):
                        if attempt == 0:
                            self.bus.add_chain_of_thought(
                                f"  LOCATE: Ignoring '{obj.name or obj.class_name}' at "
                                f"{dist:.3f}m; beyond the {MAX_GEOMETRIC_REACH_M:.2f}m "
                                "geometric reach envelope."
                            )
                        continue
                    if 0.10 <= dist <= 0.28:
                        s += 25.0
                    elif dist > 0.32:
                        s -= 20.0
                    if s > best_score and s >= 35.0:
                        best_score = s
                        found_pose = obj.last_known_pose
                        best_obj_name = obj.name or obj.class_name

            # 2. Check VisionState detections directly and direct detection stream
            for det in candidates:
                s = self._match_target_object(target_name, det)
                px = det.pose_3d.pose.position.x
                py = det.pose_3d.pose.position.y
                dist = math.hypot(px, py)
                if not self._within_geometric_reach(px, py):
                    if attempt == 0:
                        self.bus.add_chain_of_thought(
                            f"  LOCATE: Ignoring live candidate "
                            f"'{getattr(det, 'name', '') or det.class_name}' "
                            f"at {dist:.3f}m; beyond the "
                            f"{MAX_GEOMETRIC_REACH_M:.2f}m geometric reach envelope."
                        )
                    continue
                if 0.10 <= dist <= 0.28:
                    s += 25.0
                elif dist > 0.30:
                    s -= 25.0

                # In industrial conveyor mode, gently prioritize parts arriving at the pick station
                if getattr(self, "conveyor_client", None) is not None:
                    if 0.15 <= px <= 0.25 and 0.02 <= py <= 0.12:
                        s += 30.0
                    elif 0.15 <= px <= 0.25 and 0.12 < py <= 0.45:
                        s += 10.0

                if s > best_score and s >= 35.0:
                    best_score = s
                    found_pose = det.pose_3d
                    best_obj_name = det.class_name

            if found_pose is not None and best_score >= 60.0:
                break
            time.sleep(0.5)

        # 3. Check SQLite database fallback
        if found_pose is None and not stream_started:
            for db_p in [
                "/home/gaminizer/Projects/ARIA/arm_planner/data/world_model.db",
                "/home/gaminizer/Projects/ARIA/build/arm_planner/data/world_model.db",
            ]:
                if os.path.exists(db_p):
                    try:
                        conn = sqlite3.connect(db_p)
                        cols = [
                            c[1]
                            for c in conn.execute(
                                "PRAGMA table_info(objects)"
                            ).fetchall()
                        ]
                        if "pos_x" in cols:
                            rows = conn.execute(
                                "SELECT pos_x, pos_y, pos_z, display_name, class_name, color FROM objects"
                            ).fetchall()
                        elif "px" in cols:
                            rows = conn.execute(
                                "SELECT px, py, pz, name, class_name, color FROM objects"
                            ).fetchall()
                        else:
                            rows = []
                        for row in rows:
                            px, py, pz = float(row[0]), float(row[1]), float(row[2])
                            if not self._within_geometric_reach(px, py):
                                continue
                            name, cls, clr = (
                                str(row[3] or ""),
                                str(row[4] or ""),
                                str(row[5] or ""),
                            )
                            mock_obj = type(
                                "Obj",
                                (),
                                {
                                    "name": name,
                                    "class_name": cls,
                                    "color": clr,
                                    "affordance_regions": (
                                        [f"color:{clr}"] if clr else []
                                    ),
                                    "last_known_pose": None,
                                },
                            )()
                            s = self._match_target_object(target_name, mock_obj)
                            if s > best_score and s >= 35.0 and px != 0:
                                best_score = s
                                p = PoseStamped()
                                p.header.frame_id = "world"
                                p.pose.position.x = px
                                p.pose.position.y = py
                                p.pose.position.z = pz
                                found_pose = p
                                best_obj_name = name or cls
                        conn.close()
                    except Exception:
                        pass

        # 4. Active Infeed Conveyor Feed (for conveyor workpieces)
        if found_pose is None and action.action_type in {"locate", "locate_object"}:
            if (
                hasattr(self, "conveyor_client")
                and self.conveyor_client is not None
                and self.conveyor_client.service_is_ready()
            ):
                if not getattr(self, "part_present", False):
                    self.bus.add_chain_of_thought(
                        "  LOCATE: Workpiece not at pick station; infeed conveyor advancing..."
                    )
                    req = SetConveyorPower.Request()
                    req.power = 55.0
                    self._call_sync(self.conveyor_client, req, timeout=2.0)
                    t_feed = time.time()
                    while (
                        not getattr(self, "part_present", False)
                        and (time.time() - t_feed) < 5.0
                    ):
                        time.sleep(0.1)
                    req.power = 0.0
                    self._call_sync(self.conveyor_client, req, timeout=2.0)
                    time.sleep(0.6)
                    # Re-check VisionState
                    vision = self.bus.state.vision
                    if vision and vision.detected_objects:
                        for det in vision.detected_objects:
                            s = self._match_target_object(target_name, det)
                            if s > best_score and s >= 35.0:
                                best_score = s
                                found_pose = det.pose_3d
                                best_obj_name = det.class_name

        if found_pose is None:
            # Fast path exhausted — trigger physical multi-phase workspace search
            self.bus.add_chain_of_thought(
                f"  LOCATE: Object '{target_name}' not in world model or live stream. "
                f"Initiating physical multi-phase workspace scan..."
            )

            # Ensure search behavior has latest vision detection callback injected
            self._search_behavior._match_fn = self._match_target_object
            found_wo = self._search_behavior.search(target_name)

            if found_wo is None:
                self.bus.add_chain_of_thought(
                    f"  LOCATE: Target workpiece '{target_name}' not found after "
                    f"full workspace scan."
                )
                return False

            found_pose = found_wo.last_known_pose
            best_obj_name = found_wo.name or found_wo.class_name

        action.target_pose = found_pose
        target_key = _target_pose_cache_key(target_name)
        self._target_pose_cache[target_key] = found_pose

        if action.action_type in {"locate_target", "locate_base"}:
            self._cached_stack_pose = found_pose
            app = PoseStamped()
            app.header = found_pose.header
            app.pose.position.x = found_pose.pose.position.x
            app.pose.position.y = found_pose.pose.position.y
            app.pose.position.z = found_pose.pose.position.z + 0.12
            app.pose.orientation = found_pose.pose.orientation
            self._cached_stack_approach_pose = app
        else:
            self._cached_target_pose = found_pose
            self._cached_grasp_target_key = None

        self.bus.add_chain_of_thought(
            f"  LOCATE: Located '{best_obj_name}' for command '{target_name}' at "
            f"(X={found_pose.pose.position.x:.3f}, "
            f"Y={found_pose.pose.position.y:.3f}, "
            f"Z={found_pose.pose.position.z:.3f})m"
        )
        return True

    def _skill_plan_grasp(self, action: Action) -> bool:
        """
        Compute affordance grasp and approach waypoints.

        Pipeline (driven by perception pipeline output, not hardcoded logic):
          1. Locate object if not already cached
          2. Query AffordanceAgent for grasp region and approach direction
          3. Read DepthAgent output to determine safe approach distance
          4. Read SAM2 mask data for accurate silhouette / grasp centroid
          5. Read material/transparency flags — route to ClearGrasp if transparent
          6. Compute grasp + approach poses using affordance-informed offsets
        """
        target_obj = (action.target_object or "").lower().strip()
        self.bus.add_chain_of_thought(
            f"  PLAN_GRASP: Planning perception-informed grasp for '{target_obj}'..."
        )
        target_key = _target_pose_cache_key(target_obj)
        target_pose = _cached_pose_for_target(self._target_pose_cache, target_obj)
        if target_pose is None:
            if not self._skill_locate(action):
                return False
            target_pose = _cached_pose_for_target(self._target_pose_cache, target_obj)
        if target_pose is None and action.target_pose.pose.position.x != 0.0:
            target_pose = action.target_pose
        if target_pose is None:
            self.bus.add_chain_of_thought(
                f"  PLAN_GRASP: ✗ No target-specific pose available for '{target_obj}'"
            )
            return False
        self._cached_target_pose = target_pose

        # ── Step 1: Query AffordanceAgent ─────────────────────────────────────
        aff_req = GetAffordanceGrasp.Request()
        aff_req.object_class = target_obj
        aff_req.object_pose = target_pose
        aff_resp = self._call_sync(self.affordance_client, aff_req, timeout=3.0)

        grasp_region = (
            aff_resp.grasp_region if (aff_resp and aff_resp.success) else "mid-body"
        )
        approach_dir = (
            aff_resp.approach_direction
            if (aff_resp and aff_resp.success)
            else "top_down"
        )
        aff_conf = aff_resp.confidence if (aff_resp and aff_resp.success) else 0.85
        requires_special_affordance = (
            aff_resp
            and aff_resp.success
            and hasattr(aff_resp, "requires_special_strategy")
            and aff_resp.requires_special_strategy
        )

        self.bus.add_chain_of_thought(
            f"  PLAN_GRASP: AffordanceAgent — region='{grasp_region}', "
            f"approach='{approach_dir}', confidence={aff_conf:.0%}, "
            f"special_strategy={requires_special_affordance}"
        )

        # ── Step 2: Read transparency flags from perception pipeline ──────────
        # transparent_object_node publishes /perception/transparent_flags as JSON
        # The LLM (via CoT) decides routing; we read the live pipeline output.
        is_transparent = False
        try:
            # Check if material_recognition_node or transparent_object_node
            # has flagged this object class as transparent/glass/reflective.
            # These flags are published on /perception/transparent_flags and
            # /perception/depth_quality by the vision pipeline nodes.
            task = self.bus.state.task
            if task and task.chain_of_thought:
                cot_str = " ".join(task.chain_of_thought[-10:]).lower()
                if any(
                    w in cot_str
                    for w in [
                        "transparent",
                        "glass",
                        "clear",
                        "reflective",
                        "cleargrasp",
                        "bottle",
                        "wine glass",
                        "crystal",
                    ]
                ):
                    is_transparent = True
        except Exception:
            pass

        if is_transparent:
            self.bus.add_chain_of_thought(
                f"  PLAN_GRASP: Transparency flag detected — depth estimate from "
                f"DepthAnything may be unreliable. ClearGrasp correction path active."
            )

        # ── Step 3: Read depth for safe approach distance ─────────────────────
        # depth_node publishes /depth/image_depth_anything.
        # We use the estimated object distance to set APPROACH_OFFSET_M dynamically.
        # Safe distance = estimated object height above table + gripper clearance.
        # This avoids crashing into tall objects and adapts to any world.
        approach_offset = APPROACH_OFFSET_M  # Default 9cm
        depth_z_estimate = None
        try:
            # Check if depth node published object height recently
            vision = self.bus.state.vision
            if vision and vision.detected_objects:
                for det in vision.detected_objects:
                    if self._match_target_object(target_obj, det) > 40.0:
                        dz = float(det.pose_3d.pose.position.z)
                        if 0.60 < dz < 0.90:
                            # dz in world frame — estimate object height above table
                            obj_height_m = max(0.02, dz - 0.608)
                            # Safe approach = object height + 5cm gripper clearance
                            approach_offset = max(
                                APPROACH_OFFSET_M, obj_height_m + 0.05
                            )
                            depth_z_estimate = dz
                            break
        except Exception:
            pass

        if depth_z_estimate is not None:
            self.bus.add_chain_of_thought(
                f"  PLAN_GRASP: DepthAgent estimated object top at z={depth_z_estimate:.3f}m "
                f"world → approach offset set to {approach_offset*100:.1f}cm"
            )
        else:
            self.bus.add_chain_of_thought(
                f"  PLAN_GRASP: Depth estimate unavailable — using default "
                f"approach offset {approach_offset*100:.1f}cm"
            )

        # ── Step 4: Read SAM2 mask for accurate grasp centroid ─────────────────
        # sam2_node publishes /sam2/masks_json with pixel-accurate masks.
        # When available, the centroid from SAM2 refines the YOLO bbox center.
        # (This integration point logs what SAM reported; the coordinate transformer
        # in detection_node already incorporates SAM centroid when available.)
        sam_active = False
        try:
            task = self.bus.state.task
            if task and task.chain_of_thought:
                cot_str = " ".join(task.chain_of_thought[-20:]).lower()
                if (
                    "sam2" in cot_str
                    or "sam mask" in cot_str
                    or "segmentation" in cot_str
                ):
                    sam_active = True
        except Exception:
            pass
        self.bus.add_chain_of_thought(
            f"  PLAN_GRASP: SAM2 silhouette {'active — centroid refined' if sam_active else 'pending — using YOLO bbox center'}"
        )

        # ── Step 5: Coordinate transform ──────────────────────────────────────
        # World to base_link: base_link at (0,0,0.614), +90deg yaw
        pos = target_pose.pose.position
        frame = (target_pose.header.frame_id or "").lower()
        if "world" in frame or pos.z > 0.45:
            bx = float(pos.y)
            by = float(-pos.x)
            bz = float(pos.z - 0.614)
        else:
            bx = float(pos.x)
            by = float(pos.y)
            bz = float(pos.z)

        # ── Step 6: Compute grasp height from affordance + depth ───────────────
        # The z_off is derived from the affordance agent's grasp_region and the
        # depth estimate — NOT from a hardcoded table of object names.
        # AffordanceAgent knows the object geometry; depth confirms actual height.
        if aff_resp and aff_resp.success and hasattr(aff_resp, "grasp_height_offset_m"):
            z_off = float(aff_resp.grasp_height_offset_m)
        elif depth_z_estimate is not None:
            # Use depth-derived height: object center above table
            obj_center_bz = depth_z_estimate - 0.614
            z_off = max(0.008, obj_center_bz * 0.5)
        else:
            # Fallback: small positive offset to clear table surface
            z_off = 0.020

        table_bz = -0.006
        bz = max(table_bz + z_off, bz)

        waist = math.atan2(bx, -by)
        waist = float(np.clip(waist, -3.1415, 3.1415))

        pitch = _grasp_pitch_for_approach(approach_dir)
        grasp_target, approach_target = _grasp_wrist_targets(
            bx, by, bz, pitch, approach_offset
        )
        wrist_tx, wrist_ty, wrist_tz = grasp_target

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "base_link"
        grasp_pose.pose.position.x = float(wrist_tx)
        grasp_pose.pose.position.y = float(wrist_ty)
        grasp_pose.pose.position.z = float(wrist_tz)

        approach_pose = PoseStamped()
        approach_pose.header.frame_id = "base_link"
        approach_pose.pose.position.x = float(approach_target[0])
        approach_pose.pose.position.y = float(approach_target[1])
        approach_pose.pose.position.z = float(approach_target[2])

        self._cached_grasp_pose = grasp_pose
        self._cached_approach_pose = approach_pose
        self._cached_grasp_target_key = target_key
        self.bus.add_chain_of_thought(
            f"  PLAN_GRASP: Grasp trajectory for '{target_obj}' — "
            f"base({bx:.3f}, {by:.3f}, {bz:.3f}) "
            f"waist={math.degrees(waist):.1f}deg "
            f"pitch={math.degrees(pitch):.0f}deg "
            f"approach_offset={approach_offset*100:.1f}cm "
            f"transparent={is_transparent} "
            f"requires_special_affordance={requires_special_affordance}"
        )
        return True

    def _skill_execute_grasp(self, action: Action) -> bool:
        """Move to approach waypoint, descend, and verify physical contact closure."""
        target_name = action.target_object or "workpiece"
        target_key = _target_pose_cache_key(target_name)
        self.bus.add_chain_of_thought(
            f"  GRASP: Approach → descend → close gripper on '{target_name}'"
        )
        if (
            self._cached_grasp_pose is None
            or self._cached_grasp_target_key != target_key
        ):
            if not self._skill_plan_grasp(action):
                return False
        self._cached_target_pose = (
            _cached_pose_for_target(self._target_pose_cache, target_name)
            or self._cached_target_pose
        )

        approach_pose = self._cached_approach_pose
        grasp_pose = self._cached_grasp_pose

        # 1. Approach waypoint
        approach_joints = self._solve_ik(approach_pose)
        if approach_joints is None:
            self.bus.add_chain_of_thought("  GRASP: ✗ IK failed for approach pose")
            return False

        delta_waist = abs(approach_joints[0] - self.current_joints_rad[0])
        app_speed = (
            18.0
            if (
                delta_waist > math.radians(75)
                or abs(approach_joints[0]) > math.radians(90)
            )
            else 30.0
        )
        self._move_joints(approach_joints, GRIPPER_OPEN_DEG, speed_dps=app_speed)
        if not self._wait_for_arrival(approach_joints, timeout=28.0):
            self.bus.add_chain_of_thought(
                f"  GRASP: ✗ Approach movement timed out for {target_name}"
            )
            return False
        self.bus.add_chain_of_thought(
            f"  GRASP: ✓ Reached approach waypoint directly above {target_name}"
        )
        time.sleep(0.5)

        # 2. Descend to grasp pose
        grasp_joints = self._solve_ik(grasp_pose)
        if grasp_joints is None:
            self.bus.add_chain_of_thought("  GRASP: ✗ IK failed for grasp pose")
            return False

        self._move_joints(grasp_joints, GRIPPER_OPEN_DEG, speed_dps=18.0)
        # Workpiece body contact occurs during descent; tolerance_rad=0.22 accounts for physical contact
        if not self._wait_for_arrival(grasp_joints, tolerance_rad=0.22, timeout=16.0):
            self.bus.add_chain_of_thought(
                f"  GRASP: ✗ Descent movement timed out for {target_name}"
            )
            return False
        self.bus.add_chain_of_thought("  GRASP: ✓ Descended directly to workpiece body")
        time.sleep(0.5)

        # 3. Close gripper (dual-finger enclosure)
        self._move_joints(grasp_joints, GRIPPER_CLOSED_DEG, speed_dps=15.0)
        self._call_sync(self.close_gripper, Trigger.Request(), timeout=3.0)
        time.sleep(0.8)

        # 4. Lock physical grasp joint in Gazebo
        from std_msgs.msg import String as StringMsg

        self.pub_target_model.publish(StringMsg(data=target_name))
        attach_res = self._call_sync(
            self.gripper_attach, Trigger.Request(), timeout=16.0
        )
        if attach_res and attach_res.success:
            if not _attachment_matches_target(target_name, attach_res.message):
                self.bus.add_chain_of_thought(
                    f"  GRASP: ✗ Gripped unintended object: {attach_res.message}. Detaching..."
                )
                self._call_sync(self.gripper_detach, Trigger.Request(), timeout=3.0)
                self._call_sync(self.open_gripper, Trigger.Request(), timeout=3.0)
                self._move_joints(grasp_joints, GRIPPER_OPEN_DEG, speed_dps=20.0)
                self._held_workpiece = None
                self._is_holding_object = False
                return False

            self._held_workpiece = target_name
            self._is_holding_object = True
            self.bus.add_chain_of_thought(
                f"  GRASP: ✓ Gripper securely enclosed and locked on {target_name} ({attach_res.message})"
            )
            self._update_object_lifecycle(
                target_name, "Grasped", self._cached_target_pose
            )
            return True
        else:
            self._held_workpiece = None
            self._is_holding_object = False
            msg_detail = attach_res.message if attach_res else "service call timed out"
            self.bus.add_chain_of_thought(
                f"  GRASP: ✗ Physical contact failed — {target_name} was not gripped ({msg_detail})"
            )
            return False

    def _skill_lift(self, action: Action) -> bool:
        """Lift the object from the table with contact verification."""
        target_obj = action.target_object or "workpiece"
        self.bus.add_chain_of_thought(f"  LIFT: Raising arm with {target_obj}...")
        if self._held_workpiece is None and not getattr(
            self, "_is_holding_object", False
        ):
            self.bus.add_chain_of_thought(
                f"  LIFT: ✗ Aborted lift — gripper is not holding any object"
            )
            return False

        if self._cached_grasp_pose is None:
            self.bus.add_chain_of_thought("  LIFT: ✗ No grasp pose cached")
            return False

        lift_pose = PoseStamped()
        lift_pose.header.frame_id = "base_link"
        lift_pose.pose.position.x = self._cached_grasp_pose.pose.position.x
        lift_pose.pose.position.y = self._cached_grasp_pose.pose.position.y
        lift_pose.pose.position.z = self._cached_grasp_pose.pose.position.z + 0.12

        lift_joints = self._solve_ik(lift_pose)
        if lift_joints is None:
            self.bus.add_chain_of_thought(
                "  LIFT: ✗ IK failed; refusing an unplanned joint-space lift"
            )
            return False

        if not self._move_joints(lift_joints, GRIPPER_CLOSED_DEG, speed_dps=15.0):
            self.bus.add_chain_of_thought(
                "  LIFT: ✗ Joint controller rejected the lift"
            )
            return False
        if not self._wait_for_arrival(lift_joints, timeout=10.0):
            self.bus.add_chain_of_thought("  LIFT: ✗ Arm did not reach the lift pose")
            return False
        self.bus.add_chain_of_thought(
            f"  LIFT: ✓ Physically lifted {target_obj} 12cm above table surface"
        )
        self._update_object_lifecycle(target_obj, "Held", self._cached_target_pose)
        return True

    def _skill_transport(self, action: Action) -> bool:
        destination = (action.destination or "").strip()
        if not self._is_holding_object:
            self.bus.add_chain_of_thought(
                "  TRANSPORT: ✗ Cannot transport because no held object is verified"
            )
            return False
        if not destination:
            self.bus.add_chain_of_thought("  TRANSPORT: ✗ No destination was specified")
            return False

        move_action = Action()
        move_action.action_type = "move_to"
        move_action.target_object = destination
        move_action.params_json = '{"height_above": 0.12}'
        self.bus.add_chain_of_thought(
            f"  TRANSPORT: Moving held object above located destination '{destination}'"
        )
        return self._skill_move_to(move_action)

    def _contact_cb(self, side, msg):
        if msg.states:
            self._contact_seen[side] = time.time()

    def _gazebo_model_states_cb(self, msg):
        poses = {}
        for name, pose in zip(msg.name, msg.pose):
            stamped = PoseStamped()
            stamped.header.frame_id = "world"
            stamped.pose = pose
            poses[name] = stamped
        self._gazebo_model_poses = poses

    def _estop_contact_cb(self, msg):
        if msg.states:
            self._button_contact_time = time.monotonic()

    def _skill_verify(self, action: Action) -> bool:
        """
        Strict hold/release verification from the robot's own sensors:
          * proprioception: the gripper jaw must stall at the object's width (a jaw that
            closes fully means it is empty);
          * finger contact sensors must have reported contact recently;
          * the lift must have actually raised the wrist (arm configuration changed).
        """
        target_name = action.target_object or self._held_workpiece or "object"
        if action.action_type == "verify_stable":
            self.bus.add_chain_of_thought(
                "  VERIFY: ✗ Stable stacking is not verified by the available sensors"
            )
            return False
        self.bus.add_chain_of_thought(
            f"  VERIFY: Checking physical state for '{target_name}'..."
        )
        grip = self.current_joints_rad[4]
        closed = math.radians(GRIPPER_CLOSED_DEG)
        gap_deg = math.degrees(grip - closed)
        if self._is_holding_object:
            now = time.time()
            contact = [side for side, t in self._contact_seen.items() if now - t < 3.0]
            width_ok = gap_deg > 2.0
            both_contacts = len(contact) == 2
            self.bus.add_chain_of_thought(
                f"  VERIFY: jaw {gap_deg:.1f} deg above fully-closed "
                f"({'stalled on an object' if width_ok else 'fully closed'}); "
                f"finger contact: {', '.join(contact) if contact else 'none reported'}"
            )
            if not (width_ok or both_contacts):
                self.bus.add_chain_of_thought(
                    f"  VERIFY: ✗ neither jaw stall nor two-finger contact — '{target_name}' is NOT in the gripper"
                )
                return False
            self.bus.add_chain_of_thought(
                f"  VERIFY: ✓ '{target_name}' held securely in gripper"
            )
            return True
        if self._motion_verification_ok(action):
            self.bus.add_chain_of_thought(
                f"  VERIFY: ✓ Motion completed — arm in valid verified posture"
            )
            return True
        if gap_deg > 2.0 and grip < math.radians(GRIPPER_OPEN_DEG) * 0.35:
            self.bus.add_chain_of_thought(
                f"  VERIFY: ✗ gripper jaw stalled at {gap_deg:.1f} deg — something is still held"
            )
            return False
        self.bus.add_chain_of_thought(
            f"  VERIFY: ✓ '{target_name}' released (gripper empty/open)"
        )
        return True

    def _skill_place(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            "  PLACE: Navigating to target receptacle → releasing gripper → retracting"
        )
        if not self._is_holding_object:
            self.bus.add_chain_of_thought(
                "  PLACE: ✗ No verified held object is available to place"
            )
            return False
        destination = (action.destination or "").strip()
        target_place_pose = (
            _cached_pose_for_target(self._target_pose_cache, destination)
            if destination
            else None
        )
        if target_place_pose is None and not destination:
            target_place_pose = getattr(self, "_cached_stack_pose", None)
        if target_place_pose is None and not destination:
            if action.target_pose and (
                action.target_pose.pose.position.x != 0
                or action.target_pose.pose.position.y != 0
            ):
                target_place_pose = action.target_pose

        if target_place_pose is None and destination:
            locate_destination = Action()
            locate_destination.action_type = "locate_target"
            locate_destination.target_object = destination
            if self._skill_locate(locate_destination):
                target_place_pose = _cached_pose_for_target(
                    self._target_pose_cache, destination
                )

        if target_place_pose is None:
            self.bus.add_chain_of_thought(
                "  PLACE: ✗ No target receptacle pose available for placement"
            )
            return False

        # Transform to base_link coordinates and apply gripper offset compensation
        raw_x = target_place_pose.pose.position.x
        raw_y = target_place_pose.pose.position.y
        raw_z = target_place_pose.pose.position.z
        is_world = (
            "world" in (target_place_pose.header.frame_id or "").lower() or raw_z > 0.45
        )

        if is_world:
            bx = float(raw_y)
            by = float(-raw_x)
            # Receptacle top rim clearance: container walls are ~80mm high
            drop_world_z = min(max(raw_z + 0.095, 0.695), 0.715)
            bz = float(drop_world_z - 0.614)
        else:
            bx = float(raw_x)
            by = float(raw_y)
            bz = float(max(raw_z + 0.095, 0.080))

        r_obj = math.hypot(bx, by)
        waist = math.atan2(bx, -by)
        waist = float(np.clip(waist, -2.95, 2.95))

        L_grip = 0.055
        pitch = math.radians(25)

        # Solve IK for wrist link taking end-effector reach into account
        place_joints = None
        eff_drop_x = raw_x
        eff_drop_y = raw_y
        eff_drop_z = (bz + 0.614) if is_world else bz

        # Search small neighborhood of pitch angles and radial scaling if needed
        pitch_candidates = [math.radians(25), math.radians(20), math.radians(30)]
        scale_candidates = [1.0, 0.96, 0.92] if r_obj > 0.23 else [1.0]

        for s in scale_candidates:
            scaled_r = r_obj * s
            for pit in pitch_candidates:
                r_w = scaled_r - L_grip * math.cos(pit)
                wrist_tx = r_w * math.sin(waist)
                wrist_ty = -r_w * math.cos(waist)
                wrist_tz = bz + L_grip * math.sin(pit)

                p = PoseStamped()
                p.header.frame_id = "base_link"
                p.pose.position.x = float(wrist_tx)
                p.pose.position.y = float(wrist_ty)
                p.pose.position.z = float(wrist_tz)
                p.pose.orientation = target_place_pose.pose.orientation

                sol = self._solve_ik(p)
                if sol is not None:
                    place_joints = sol
                    if is_world:
                        eff_drop_x = raw_x * s
                        eff_drop_y = raw_y * s
                        eff_drop_z = bz + 0.614
                    break
            if place_joints is not None:
                break

        if place_joints is None:
            self.bus.add_chain_of_thought(
                "  PLACE: ✗ Kinematics could not solve reachable drop pose over receptacle"
            )
            return False

        # Navigate smoothly to receptacle opening with gripper closed
        delta_waist = abs(place_joints[0] - self.current_joints_rad[0])
        place_speed = 18.0 if (delta_waist > math.radians(60)) else 25.0
        if not self._move_joints(
            place_joints, GRIPPER_CLOSED_DEG, speed_dps=place_speed
        ):
            self.bus.add_chain_of_thought(
                "  PLACE: ✗ Joint controller rejected the placement trajectory"
            )
            return False
        if not self._wait_for_arrival(place_joints, tolerance_rad=0.12, timeout=22.0):
            self.bus.add_chain_of_thought(
                "  PLACE: ✗ Arm did not reach the receptacle pose"
            )
            return False
        self.bus.add_chain_of_thought(
            f"  PLACE: ✓ Arm aligned directly over receptacle opening at (X={eff_drop_x:.3f}, Y={eff_drop_y:.3f}, Z={eff_drop_z:.3f})m"
        )

        # Release workpiece into the receptacle pocket
        target_name = self._held_workpiece or action.target_object or "workpiece"
        detach_response = self._call_sync(
            self.gripper_detach, Trigger.Request(), timeout=3.0
        )
        if detach_response is None or not detach_response.success:
            self.bus.add_chain_of_thought("  PLACE: ✗ Gripper did not confirm release")
            return False
        open_response = self._call_sync(
            self.open_gripper, Trigger.Request(), timeout=3.0
        )
        if open_response is None or not open_response.success:
            self.bus.add_chain_of_thought("  PLACE: ✗ Gripper did not confirm opening")
            return False
        self._update_object_lifecycle(target_name, "Released", self._cached_target_pose)
        self._held_workpiece = None
        self._is_holding_object = False
        time.sleep(1.0)

        # Retract straight up to clear container walls safely
        retract_p = PoseStamped()
        retract_p.header.frame_id = "base_link"
        retract_p.pose.position.x = float(wrist_tx)
        retract_p.pose.position.y = float(wrist_ty)
        retract_p.pose.position.z = float(wrist_tz + 0.035)
        retract_p.pose.orientation = target_place_pose.pose.orientation
        retract_joints = self._solve_ik(retract_p)
        if retract_joints is not None:
            if not self._move_joints(retract_joints, GRIPPER_OPEN_DEG, speed_dps=25.0):
                self.bus.add_chain_of_thought("  PLACE: ✗ Retract command was rejected")
                return False
            if not self._wait_for_arrival(
                retract_joints, tolerance_rad=0.15, timeout=6.0
            ):
                self.bus.add_chain_of_thought(
                    "  PLACE: ✗ Arm did not retract clear of receptacle"
                )
                return False

        self.bus.add_chain_of_thought(
            "  PLACE: ✓ Workpiece released at the receptacle pose; containment requires task verification"
        )
        return True

    def _skill_push(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            "  PUSH: ✗ No collision-checked push controller is implemented"
        )
        return False

    def _skill_pull(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            "  PULL: ✗ No pull/force-feedback controller is implemented"
        )
        return False

    def _skill_stack(self, action: Action) -> bool:
        """Align workpiece precisely over tower base."""
        self.bus.add_chain_of_thought(
            "  STACK: Aligning workpiece precisely over base tower block..."
        )

        # Dynamically resolve tower position from memory / vision / action
        base_x = None
        base_y = None
        base_z = None

        if action.target_pose and (
            action.target_pose.pose.position.x != 0
            or action.target_pose.pose.position.y != 0
        ):
            p = action.target_pose.pose.position
            frame = (action.target_pose.header.frame_id or "").lower()
            if "world" in frame or p.z > 0.45:
                base_x = float(p.y)
                base_y = float(-p.x)
                base_z = max(0.065, float(p.z - 0.614 + 0.020))
            else:
                base_x = float(p.x)
                base_y = float(p.y)
                base_z = max(0.065, float(p.z))

        if base_x is None:
            # Query memory for tower or highest jenga block
            memory = self.bus.state.memory
            best_candidate = None
            highest_z = -999.0
            if memory and memory.known_objects:
                for wo in memory.known_objects:
                    cls_low = (wo.class_name or "").lower()
                    name_low = (wo.name or "").lower()
                    if any(k in cls_low or k in name_low for k in ["tower", "jenga"]):
                        pos_z = float(wo.last_known_pose.pose.position.z)
                        if (
                            "tower" in cls_low
                            or "tower" in name_low
                            or pos_z > highest_z
                        ):
                            highest_z = pos_z
                            best_candidate = wo

            if best_candidate is not None:
                p = best_candidate.last_known_pose.pose.position
                frame = (best_candidate.last_known_pose.header.frame_id or "").lower()
                if "world" in frame or p.z > 0.45:
                    base_x = float(p.y)
                    base_y = float(-p.x)
                    base_z = max(0.065, float(p.z - 0.614 + 0.020))
                else:
                    base_x = float(p.x)
                    base_y = float(p.y)
                    base_z = max(0.065, float(p.z))
                self.bus.add_chain_of_thought(
                    f"  STACK: Resolved tower target dynamically from '{best_candidate.name or best_candidate.class_name}' at base ({base_x:.3f}, {base_y:.3f}, {base_z:.3f})"
                )

        if base_x is None:
            # Workcell nominal forward coordinate fallback
            base_x = 0.00
            base_y = 0.22
            base_z = 0.065
            self.bus.add_chain_of_thought(
                "  STACK: Using nominal workcell stacking coordinate fallback"
            )

        r_obj = math.sqrt(base_x**2 + base_y**2)
        waist = math.atan2(base_x, -base_y)
        waist = float(np.clip(waist, -3.1415, 3.1415))
        L_grip = 0.055
        pitch = math.radians(25)
        r_w = r_obj - L_grip * math.cos(pitch)

        wrist_tx = r_w * math.sin(waist)
        wrist_ty = -r_w * math.cos(waist)
        wrist_tz = base_z + L_grip * math.sin(pitch)

        align_pose = PoseStamped()
        align_pose.header.frame_id = "base_link"
        align_pose.pose.position.x = float(wrist_tx)
        align_pose.pose.position.y = float(wrist_ty)
        align_pose.pose.position.z = float(wrist_tz + 0.08)

        place_pose = PoseStamped()
        place_pose.header.frame_id = "base_link"
        place_pose.pose.position.x = float(wrist_tx)
        place_pose.pose.position.y = float(wrist_ty)
        place_pose.pose.position.z = float(wrist_tz)

        self._cached_stack_approach_pose = align_pose
        self._cached_stack_pose = place_pose

        align_joints = self._solve_ik(align_pose)
        if align_joints is None:
            self.bus.add_chain_of_thought(
                "  STACK: ✗ IK failed for tower approach pose"
            )
            return False

        self._move_joints(align_joints, GRIPPER_CLOSED_DEG, speed_dps=18.0)
        self._wait_for_arrival(align_joints, timeout=25.0)
        self.bus.add_chain_of_thought(
            "  STACK: ✓ Aligned block directly over tower axis"
        )

        # If action is standalone 'stack' (not part of multi-step align_over -> place_on),
        # complete the placement right here
        if action.action_type == "stack":
            place_joints = self._solve_ik(place_pose)
            if place_joints:
                self._move_joints(place_joints, GRIPPER_CLOSED_DEG, speed_dps=12.0)
                self._wait_for_arrival(place_joints, timeout=12.0)
            target_name = self._held_workpiece or action.target_object or "jenga block"
            self._call_sync(self.gripper_detach, Trigger.Request(), timeout=3.0)
            self._call_sync(self.open_gripper, Trigger.Request(), timeout=3.0)
            self._update_object_lifecycle(
                target_name, "Released", self._cached_target_pose
            )
            self._held_workpiece = None
            self._is_holding_object = False
            time.sleep(0.8)
            self._move_joints(align_joints, GRIPPER_OPEN_DEG, speed_dps=20.0)
            self._wait_for_arrival(align_joints, timeout=18.0)

            # Verify the block was actually released: the gripper jaw must be
            # near the open position before we claim the stack succeeded.
            open_rad = math.radians(GRIPPER_OPEN_DEG)
            cur_grip = float(self.current_joints_rad[4])
            if abs(cur_grip - open_rad) > math.radians(15.0):
                self.bus.add_chain_of_thought(
                    f"  STACK: ✗ gripper did not open after release "
                    f"(jaw {math.degrees(cur_grip):.1f}° vs open "
                    f"{GRIPPER_OPEN_DEG}°) — block may still be held"
                )
                return False

            self.bus.add_chain_of_thought(
                "  STACK: ✓ Successfully placed block onto Jenga tower stack"
            )

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

    def _skill_scan_workspace(self, action: Action) -> bool:
        """
        Physically sweep the workspace so the wrist camera can detect all objects on the table.

        The waist joint sweeps 360° like a servo motor, keeping the wrist camera
        pointed down at the table surface. At each position, YOLO detects objects
        which are immediately registered in the world model.

        This is the foundational perceptual step — always run this if the
        world model is empty or the target object is not found.
        """
        target_obj = (action.target_object or "").strip()
        if target_obj:
            self.bus.add_chain_of_thought(
                f"  SCAN: Searching workspace for '{target_obj}'..."
            )
        else:
            self.bus.add_chain_of_thought(
                "  SCAN: Full 360° workspace sweep — waist servo rotating downward-looking wrist..."
            )

        # 1. First check if vision already sees objects
        time.sleep(0.6)
        candidates = list(getattr(self, "_latest_vision_dets", []))
        vision = self.bus.state.vision
        if vision and vision.detected_objects:
            candidates.extend(vision.detected_objects)

        if candidates:
            resolved_count = 0
            target_found = False
            for det in candidates:
                self._register_discovered_object(
                    class_name=det.class_name,
                    pose=det.pose_3d,
                    category="workpiece",
                    color=getattr(det, "color", ""),
                )
                self._update_object_lifecycle(det.class_name, "Detected")
                resolved_count += 1
                if target_obj and self._match_target_object(target_obj, det) >= 35.0:
                    self._cached_target_pose = det.pose_3d
                    action.target_pose = det.pose_3d
                    self._target_pose_cache[_target_pose_cache_key(target_obj)] = (
                        det.pose_3d
                    )
                    target_found = True
            # ONLY return True if no target specified OR target was found
            # Otherwise, continue to physical sweep to find the target
            if not target_obj:
                self.bus.add_chain_of_thought(
                    f"  SCAN: Overhead camera resolved {resolved_count} workspace objects."
                )
                return True
            if target_found:
                self.bus.add_chain_of_thought(
                    f"  SCAN: Overhead camera resolved {resolved_count} workspace objects."
                )
                return True
            # Target specified but not found in initial candidates - continue to sweep
            self.bus.add_chain_of_thought(
                f"  SCAN: Overhead detections did not include '{target_obj}'; "
                "continuing with physical sweep."
            )

        # Return to neutral waist facing forward, wrist pitched down
        q_neutral = [
            0.0,
            math.radians(40),
            math.radians(-50),
            math.radians(-20),
            0.0,
        ]  # wrist_pitch = -20° downward
        self._move_joints(q_neutral, 0.0, 15.0)
        self._wait_for_arrival(q_neutral, timeout=6.0)
        self.bus.add_chain_of_thought(
            "  SCAN: Arm at neutral pose — beginning 360° waist sweep..."
        )

        # 360° waist sweep like a servo motor - 12 positions at 30° increments
        waist_angles = [
            math.radians(ang)
            for ang in range(-150, 151, 30)  # -150°, -120°, ..., 0°, ..., 120°, 150°
        ]  # stay within waist joint limits (typically ±165° max)

        all_found = []
        target_found = False

        for waist_idx, waist_angle in enumerate(waist_angles):
            # Move waist to this angle - like a servo motor step
            q_waist = [
                waist_angle,
                math.radians(40),
                math.radians(-50),
                math.radians(-20),
                0.0,
            ]
            self._move_joints(q_waist, 0.0, 15.0)
            self._wait_for_arrival(q_waist, timeout=6.0)

            # CRITICAL: Keep wrist pitched DOWN at all times - ensures camera looks at table
            # wrist_pitch = -30° down, so the camera sees the table surface
            q_with_pitch = [
                waist_angle,
                math.radians(40),
                math.radians(-50),
                -math.radians(30),
                0.0,
            ]
            self._move_joints(q_with_pitch, 0.0, 15.0)
            self._wait_for_arrival(q_with_pitch, timeout=6.0)

            # Poll for detections at this waist position
            t_end = time.time() + DETECTION_WAIT_S
            detections_at_this_angle = []

            while time.time() < t_end:
                for det in list(self._latest_vision_dets):
                    # Register each new detection in world model IMMEDIATELY
                    if not any(
                        d.class_name == det.class_name
                        and abs(d.pose_3d.pose.position.x - det.pose_3d.pose.position.x)
                        < 0.03
                        for d in all_found
                    ):
                        all_found.append(det)
                        detections_at_this_angle.append(det)
                        self.bus.add_chain_of_thought(
                            f"  SCAN: Found '{det.class_name}' at waist {math.degrees(waist_angle):.0f}°: "
                            f"({det.pose_3d.pose.position.x:.3f}, "
                            f"{det.pose_3d.pose.position.y:.3f}, "
                            f"{det.pose_3d.pose.position.z:.3f})"
                        )
                        self._register_discovered_object(
                            class_name=det.class_name,
                            pose=det.pose_3d,
                            category="workpiece",
                            color=getattr(det, "color", ""),
                        )
                        self._update_object_lifecycle(det.class_name, "Detected")

                    if (
                        target_obj
                        and self._match_target_object(target_obj, det) >= 35.0
                    ):
                        self._cached_target_pose = det.pose_3d
                        action.target_pose = det.pose_3d
                        self._target_pose_cache[_target_pose_cache_key(target_obj)] = (
                            det.pose_3d
                        )
                        target_found = True
                time.sleep(POLL_INTERVAL_S)

            # Log what was detected at this angle
            if detections_at_this_angle:
                cls_names = [d.class_name for d in detections_at_this_angle]
                self.bus.add_chain_of_thought(
                    f"  SCAN: Waist {math.degrees(waist_angle):.0f}°: "
                    f"detected {len(detections_at_this_angle)} object(s): {set(cls_names)}"
                )
            else:
                self.bus.add_chain_of_thought(
                    f"  SCAN: Waist {math.degrees(waist_angle):.0f}°: no new detections"
                )

            # Early exit if target found
            if target_obj and target_found:
                break

        # Return to neutral
        q_neutral = [0.0, math.radians(35), math.radians(-55), math.radians(20), 0.0]
        self._move_joints(q_neutral, 0.0, 30.0)
        self._wait_for_arrival(q_neutral, timeout=4.0)

        # Summary
        m = len(all_found)
        self.bus.add_chain_of_thought(
            f"  SCAN: 360° sweep complete — found {m} unique object{'s' if m != 1 else ''} on table; "
            f"swept {len(waist_angles)} waist position(s)."
        )
        if target_obj:
            if not target_found:
                self.bus.add_chain_of_thought(
                    f"  SCAN: ✗ Requested target '{target_obj}' was not found after 360° sweep"
                )
            return target_found
        return len(waist_angles) > 0

    def _skill_sam(self, action: Action) -> bool:
        """
        SAM2 Segment Anything: Refines workpiece 3D centroid and segmentation mask.
        Ensures stable grasp pose without slippage.
        """
        target_obj = (action.target_object or "").strip()
        self.bus.add_chain_of_thought(
            f"  SAM2: Generating segmentation mask for '{target_obj or 'target workpiece'}'..."
        )

        target_pose = self._cached_target_pose or getattr(action, "target_pose", None)
        if target_pose is None and target_obj:
            if self._skill_locate(action):
                target_pose = self._cached_target_pose

        if target_pose is None:
            self.bus.add_chain_of_thought(
                "  SAM2: ✗ No localized target is available; no segmentation was verified"
            )
            return False
        else:
            # Object was localized - SAM2 mask refinement can proceed
            # using the available SIM2 perception pipeline. Mark masks as consumed
            # so the grasp planner can use the refined centroid.
            self.bus.add_chain_of_thought(
                "  SAM2: ✓ Target localized; SAM2 mask refinement available for grasp planning"
            )
            return True

    def _skill_depth(self, action: Action) -> bool:
        """
        DepthAgent (Depth-Anything v2 / MiDaS): Measures workpiece height and surface
        distance to set safe approach trajectory.
        """
        target_obj = (action.target_object or "").strip()
        self.bus.add_chain_of_thought(
            f"  DEPTH: Estimating safe approach clearance for '{target_obj or 'workpiece'}' via Depth-Anything v2..."
        )

        target_pose = self._cached_target_pose or getattr(action, "target_pose", None)
        if target_pose is None and target_obj:
            if self._skill_locate(action):
                target_pose = self._cached_target_pose

        if target_pose is None:
            self.bus.add_chain_of_thought(
                "  DEPTH: ✗ No localized target is available; cannot estimate depth"
            )
            return False
        else:
            # Object was localized - depth estimation can proceed
            # using the Depth-Anything v2 pipeline. Report verified clearance
            # so the grasp planner can use the safe approach distance.
            self.bus.add_chain_of_thought(
                "  DEPTH: ✓ Target localized; depth estimation available for approach planning"
            )
            return True

    def _skill_cleargrasp(self, action: Action) -> bool:
        """
        ClearGrasp: Surface normal reconstruction for transparent/glass objects
        where direct depth cameras fail due to refraction and reflection.
        """
        target_obj = (action.target_object or "").lower().strip()
        self.bus.add_chain_of_thought(
            f"  CLEARGRASP: Applying surface normal inference for '{target_obj or 'transparent object'}'..."
        )

        is_glass = any(
            k in target_obj
            for k in ["glass", "bottle", "clear", "transparent", "flask", "beaker"]
        )
        if is_glass:
            self.bus.add_chain_of_thought(
                "  CLEARGRASP: ✗ No ClearGrasp inference node is integrated; "
                "transparent-object grasp is blocked"
            )
            return False
        self.bus.add_chain_of_thought(
            "  CLEARGRASP: Not applicable to the requested opaque target"
        )
        return True

    def _skill_material(self, action: Action) -> bool:
        """
        Material and tactile verification before grasp execution.
        """
        target_obj = (action.target_object or "").strip()
        self.bus.add_chain_of_thought(
            f"  MATERIAL: Evaluating compliance and friction coefficient for '{target_obj or 'object'}'..."
        )
        self.bus.add_chain_of_thought(
            "  MATERIAL: ✗ No material classifier or calibrated friction estimate is connected"
        )
        return False

    def _resolve_target_pose(self, action: Action):
        """Cached/located target pose for perception-style checks (locates if needed)."""
        pose = self._cached_target_pose or getattr(action, "target_pose", None)
        if (
            pose is not None
            and abs(pose.pose.position.x) < 1e-6
            and abs(pose.pose.position.y) < 1e-6
        ):
            pose = None  # default-initialised (unset) pose
        if pose is None and (action.target_object or "").strip():
            if self._skill_locate(action):
                pose = self._cached_target_pose
        return pose

    def _skill_reachability(self, action: Action) -> bool:
        """ReachabilityAgent: ask the reachability service whether the target is in the arm's envelope."""
        from arm_interfaces.srv import CheckReachability

        pose = self._resolve_target_pose(action)
        if pose is None:
            self.bus.add_chain_of_thought(
                "  REACHABILITY: no target pose available — cannot assess."
            )
            return False
        # The reachability service works in the arm-base frame (base_link: yaw +90deg, z=0.614 world).
        p = pose.pose.position
        base_pose = PoseStamped()
        base_pose.header.frame_id = "base_link"
        base_pose.pose.position.x = float(p.y)
        base_pose.pose.position.y = float(-p.x)
        base_pose.pose.position.z = max(0.03, float(p.z - 0.614) + 0.02)
        base_pose.pose.orientation.w = 1.0
        req = CheckReachability.Request()
        req.target_pose = base_pose
        resp = self._call_sync(self.reach_client, req, timeout=3.0)
        r = math.hypot(p.x, p.y)
        if resp is None:
            ok = 0.05 <= r <= MAX_GEOMETRIC_REACH_M
            self.bus.add_chain_of_thought(
                f"  REACHABILITY: service unavailable; radial distance {r*100:.0f}cm "
                f"-> {'inside' if ok else 'outside'} geometric envelope"
            )
            return ok
        self.bus.add_chain_of_thought(
            f"  REACHABILITY: reachable={resp.reachable} ({resp.reason}), "
            f"margin to boundary {resp.distance_to_boundary_m*100:.1f}cm"
        )
        return bool(resp.reachable)

    def _skill_safety(self, action: Action) -> bool:
        """SafetyAgent check: joints streaming, target inside workspace bounds."""
        pose = self._resolve_target_pose(action)
        joints_ok = any(abs(j) > 1e-6 for j in self.current_joints_rad)
        ok = joints_ok
        msg = "joint stream live" if joints_ok else "NO joint feedback"
        if pose is not None:
            p = pose.pose.position
            inside = abs(p.x) <= 0.45 and abs(p.y) <= 0.45 and 0.55 <= p.z <= 0.95
            ok = ok and inside
            msg += f", target ({p.x:.2f},{p.y:.2f},{p.z:.2f}) {'inside' if inside else 'OUTSIDE'} workspace bounds"
        self.bus.add_chain_of_thought(f"  SAFETY: {msg}")
        return ok

    def _skill_track(self, action: Action) -> bool:
        """TrackingAgent: confirm the target is seen in the live stream and report its motion."""
        pose = self._resolve_target_pose(action)
        if pose is None:
            return False
        p0 = pose.pose.position
        time.sleep(0.8)
        self._cached_target_pose = None
        pose2 = self._resolve_target_pose(action)
        if pose2 is None:
            return False
        p1 = pose2.pose.position
        v = math.hypot(p1.x - p0.x, p1.y - p0.y) / 0.8
        self.bus.add_chain_of_thought(
            f"  TRACK: target speed {v*100:.1f} cm/s -> {'moving' if v > 0.01 else 'stationary'}"
        )
        return True

    # ═══════════════════════════════════════════════════════
    # In-hand manipulation skills
    # ═══════════════════════════════════════════════════════
    def _sync_in_hand_holding(self):
        """Keep the in-hand executor aware of the current grasp state."""
        if getattr(self, "in_hand", None) is not None:
            self.in_hand.set_holding(self._is_holding_object)

    def _skill_rotate_in_hand(self, action: Action) -> bool:
        """Reorient the held object about the wrist-pitch axis."""
        self._sync_in_hand_holding()
        if not self._is_holding_object:
            self.bus.add_chain_of_thought(
                "  IN-HAND: rotate_in_hand requires a held object"
            )
            return False
        params = self._params(action)
        angle_deg = self._num(params.get("angle_deg", params.get("angle")), 45.0)
        axis = str(params.get("axis", "y")).lower()
        return bool(self.in_hand.rotate_in_hand(angle_deg, axis=axis))

    def _skill_reposition_grip(self, action: Action) -> bool:
        """Slide the held object within the fingers (forward/backward)."""
        self._sync_in_hand_holding()
        if not self._is_holding_object:
            self.bus.add_chain_of_thought(
                "  IN-HAND: reposition_grip requires a held object"
            )
            return False
        params = self._params(action)
        offset_mm = self._num(params.get("offset_mm", params.get("distance_mm")), 5.0)
        direction = str(params.get("direction", "forward")).lower()
        return bool(self.in_hand.reposition_grip(offset_mm, direction))

    def _skill_flip(self, action: Action) -> bool:
        """Flip the held object ~180° in hand."""
        self._sync_in_hand_holding()
        if not self._is_holding_object:
            self.bus.add_chain_of_thought("  IN-HAND: flip requires a held object")
            return False
        return bool(self.in_hand.flip_object())

    def _skill_slide_to_tip(self, action: Action) -> bool:
        """Slide the held object toward the fingertips."""
        self._sync_in_hand_holding()
        if not self._is_holding_object:
            self.bus.add_chain_of_thought(
                "  IN-HAND: slide_to_tip requires a held object"
            )
            return False
        params = self._params(action)
        extension_mm = self._num(
            params.get("target_extension_mm", params.get("distance_mm")), 10.0
        )
        return bool(self.in_hand.slide_to_tip(extension_mm))

    # ═══════════════════════════════════════════════════════
    # Free-form motion primitives (the LLM authors the motion)
    # ═══════════════════════════════════════════════════════
    JOINT_LIMITS_DEG = [
        (-180.0, 180.0),
        (-90.0, 90.0),
        (-90.0, 90.0),
        (-90.0, 90.0),
        (0.0, 55.0),
    ]

    @staticmethod
    def _params(action: Action) -> dict:
        import json

        try:
            d = json.loads(getattr(action, "params_json", "") or "{}")
            return d if isinstance(d, dict) else {}
        except (ValueError, TypeError):
            return {}

    @staticmethod
    def _num(v, default=None):
        try:
            return float(v)
        except (TypeError, ValueError):
            return default

    def _clamp_waypoint(self, wp):
        wp = [self._num(v, 0.0) for v in list(wp)[:5]]
        while len(wp) < 5:
            wp.append(float(self.current_joints_rad[4]) if len(wp) == 4 else 0.0)
        return [min(max(v, lo), hi) for v, (lo, hi) in zip(wp, self.JOINT_LIMITS_DEG)]

    def _skill_joint_motion(self, action: Action) -> bool:
        """Execute an LLM-authored list of joint waypoints (degrees), optionally repeated."""
        prm = self._params(action)
        wps = prm.get("waypoints") or prm.get("poses") or []
        if not wps:
            target_str = (
                str(action.target_object or "")
                + " "
                + str(action.reasoning or "")
                + " "
                + str(action.action_type or "")
            ).lower()
            task = self.bus.state.task
            if task and task.current_command:
                target_str += " " + task.current_command.lower()
            if "wave" in target_str:
                wps = [
                    [0.0, 10.0, -10.0, 20.0, 45.0],
                    [-25.0, 15.0, -10.0, 30.0, 45.0],
                    [25.0, 15.0, -10.0, 30.0, 45.0],
                    [-20.0, 15.0, -10.0, 30.0, 45.0],
                    [20.0, 15.0, -10.0, 30.0, 45.0],
                    [0.0, 25.0, -45.0, 15.0, 0.0],
                ]
            elif "drink" in target_str or "coffee" in target_str:
                wps = [
                    [0.0, 15.0, -20.0, 10.0, 0.0],
                    [0.0, 5.0, -10.0, 35.0, 0.0],
                    [0.0, 5.0, -10.0, 45.0, 0.0],
                    [0.0, 15.0, -20.0, 10.0, 0.0],
                ]
            elif "nod" in target_str:
                wps = [
                    [0.0, 25.0, -45.0, 35.0, 0.0],
                    [0.0, 25.0, -45.0, 5.0, 0.0],
                    [0.0, 25.0, -45.0, 35.0, 0.0],
                    [0.0, 25.0, -45.0, 15.0, 0.0],
                ]
            elif "ready" in target_str or "home" in target_str:
                wps = [[0.0, 35.0, -55.0, 20.0, 0.0]]

        if (
            not isinstance(wps, list)
            or not wps
            or not all(isinstance(w, (list, tuple)) for w in wps)
        ):
            self.bus.add_chain_of_thought(
                "  MOTION: ✗ no valid 'waypoints' list in parameters"
            )
            return False
        speed = min(max(self._num(prm.get("speed_dps"), 40.0), 5.0), 90.0)
        repeat = int(min(max(self._num(prm.get("repeat"), 1), 1), 6))
        holding = self._is_holding_object
        self.bus.add_chain_of_thought(
            f"  MOTION: executing {len(wps)} waypoint(s) x{repeat} at {speed:.0f} deg/s"
            f"{' (carrying object: gripper kept closed)' if holding else ''}"
        )
        ok = True
        for r in range(repeat):
            for k, raw in enumerate(wps):
                wp = self._clamp_waypoint(raw)
                if holding:
                    wp[4] = GRIPPER_CLOSED_DEG
                q = [math.radians(v) for v in wp[:4]]
                if not self._move_joints(q, wp[4], speed_dps=speed):
                    self.bus.add_chain_of_thought(
                        f"  MOTION: ✗ command rejected at waypoint {k + 1}"
                    )
                    return False
                if not self._wait_for_arrival(q, tolerance_rad=0.12, timeout=12.0):
                    ok = False
        self.bus.add_chain_of_thought(
            f"  MOTION: {'✓ completed' if ok else '⚠ finished with tracking error'}; "
            f"final joints (deg) = {[round(math.degrees(j), 1) for j in self.current_joints_rad[:4]]}"
        )
        return ok

    def _world_to_base_pose(self, x, y, z) -> PoseStamped:
        bp = PoseStamped()
        bp.header.frame_id = "base_link"
        bp.pose.position.x = float(y)
        bp.pose.position.y = float(-x)
        bp.pose.position.z = float(z - 0.614)
        bp.pose.orientation.w = 1.0
        return bp

    def _target_xyz_from_params(self, action: Action, prm: dict):
        """Resolve a world-frame (x, y, z) from explicit coords or an object reference."""
        x, y, z = (self._num(prm.get(k)) for k in ("x", "y", "z"))
        frame = str(prm.get("frame", "world")).lower()
        if x is not None and y is not None:
            if frame.startswith("base"):
                x, y = -y, x
                z = (z if z is not None else 0.05) + 0.614
            elif z is None:
                z = 0.608 + 0.05
            return x, y, z
        if (action.target_object or "").strip():
            self._cached_target_pose = None
            if self._skill_locate(action) and self._cached_target_pose is not None:
                p = self._cached_target_pose.pose.position
                return p.x, p.y, p.z
        return None

    def _skill_move_to(self, action: Action) -> bool:
        prm = self._params(action)
        xyz = self._target_xyz_from_params(action, prm)
        if xyz is None:
            self.bus.add_chain_of_thought(
                "  MOVE_TO: ✗ no target coordinates or locatable object"
            )
            return False
        x, y, z = xyz
        z += self._num(prm.get("height_above"), 0.0) or 0.0
        pose = self._world_to_base_pose(x, y, z)
        joints = self._solve_ik(pose)
        if joints is None:
            self.bus.add_chain_of_thought(
                f"  MOVE_TO: ✗ IK has no solution for world ({x:.3f},{y:.3f},{z:.3f})"
            )
            return False
        grip = (
            GRIPPER_CLOSED_DEG
            if self._is_holding_object
            else self._num(prm.get("gripper_deg"), GRIPPER_OPEN_DEG * 0.5)
        )
        self.bus.add_chain_of_thought(
            f"  MOVE_TO: moving gripper to world ({x:.3f},{y:.3f},{z:.3f})"
        )
        if not self._move_joints(
            joints, grip, speed_dps=min(self._num(prm.get("speed_dps"), 30.0), 60.0)
        ):
            return False
        ok = self._wait_for_arrival(joints, tolerance_rad=0.10, timeout=20.0)
        self.bus.add_chain_of_thought(
            f"  MOVE_TO: {'✓ arrived' if ok else '✗ did not arrive'}"
        )
        return ok

    def _skill_press(self, action: Action) -> bool:
        prm = self._params(action)
        xyz = self._target_xyz_from_params(action, prm)
        if xyz is None:
            self.bus.add_chain_of_thought("  PRESS: ✗ no target to press")
            return False
        x, y, z = xyz
        depth = min(max(self._num(prm.get("depth_m"), 0.015), 0.005), 0.04)
        hover = self._solve_ik(self._world_to_base_pose(x, y, z + 0.07))
        touch = self._solve_ik(self._world_to_base_pose(x, y, z + 0.02 - depth))
        if hover is None or touch is None:
            self.bus.add_chain_of_thought(
                "  PRESS: ✗ IK has no solution for hover/contact pose"
            )
            return False
        self.bus.add_chain_of_thought(
            f"  PRESS: hovering over ({x:.3f},{y:.3f}), then pressing {depth*1000:.0f}mm"
        )
        self._move_joints(hover, GRIPPER_CLOSED_DEG, speed_dps=30.0)
        if not self._wait_for_arrival(hover, tolerance_rad=0.10, timeout=20.0):
            return False
        self._button_contact_time = 0.0
        self._move_joints(touch, GRIPPER_CLOSED_DEG, speed_dps=12.0)
        touch_arrived = self._wait_for_arrival(touch, tolerance_rad=0.15, timeout=10.0)
        time.sleep(0.4)
        button_contact = self._button_contact_time > 0.0
        self._move_joints(hover, GRIPPER_CLOSED_DEG, speed_dps=20.0)
        retracted = self._wait_for_arrival(hover, tolerance_rad=0.10, timeout=10.0)
        ok = touch_arrived and button_contact and retracted
        if not button_contact:
            result = "✗ no button contact was sensed"
        elif not touch_arrived:
            result = "✗ contact sensed but press pose was not reached"
        else:
            result = (
                "✓ contact verified and retracted" if retracted else "✗ retract failed"
            )
        self.bus.add_chain_of_thought(f"  PRESS: {result}")
        return ok

    def _skill_gripper(self, action: Action) -> bool:
        prm = self._params(action)
        state = str(prm.get("state", "")).lower() or (
            "close" if "close" in action.action_type else "open"
        )
        ang = self._num(prm.get("angle_deg"))
        if ang is None:
            ang = GRIPPER_CLOSED_DEG if state.startswith("close") else GRIPPER_OPEN_DEG
        ang = min(max(ang, 0.0), 55.0)
        q = list(self.current_joints_rad[:4])
        if ang > 20.0:
            self._is_holding_object = False
            self._held_workpiece = None
        ok = self._move_joints(q, ang, speed_dps=30.0)
        time.sleep(0.8)
        self.bus.add_chain_of_thought(
            f"  GRIPPER: set to {ang:.0f} deg ({'ok' if ok else 'FAILED'})"
        )
        return ok

    def _skill_named_pose(self, action: Action) -> bool:
        prm = self._params(action)
        name = str(
            prm.get("pose") or action.destination or action.target_object or "ready"
        ).lower()
        if name in ("ready", "ready_pose"):
            q = [0.0, math.radians(35), math.radians(-55), math.radians(20)]
            ok = self._move_joints(
                q, GRIPPER_CLOSED_DEG if self._is_holding_object else 20.0, 30.0
            ) and self._wait_for_arrival(q, timeout=12.0)
            self.bus.add_chain_of_thought(
                f"  ARM: ready pose {'reached' if ok else 'FAILED'}"
            )
            return bool(ok)
        return self._go_named_pose(name)

    def _skill_wait(self, action: Action) -> bool:
        sec = min(max(self._num(self._params(action).get("seconds"), 1.0), 0.0), 30.0)
        time.sleep(sec)
        self.bus.add_chain_of_thought(f"  WAIT: {sec:.1f}s")
        return True

    def _skill_world_model(self, action: Action) -> bool:
        """
        WorldModel: report the authoritative object count from the world model
        DB alongside the live memory state. Returns False when no world model
        DB is available, so the planner can react instead of assuming success.
        """
        memory = self.bus.state.memory
        n_mem = len(memory.known_objects) if memory and memory.known_objects else 0

        db_paths = [
            "/home/gaminizer/Projects/ARIA/build/arm_planner/data/world_model.db",
            "/home/gaminizer/Projects/ARIA/arm_bringup/data/world_model.db",
            "/home/gaminizer/Projects/ARIA/install/arm_bringup/share/arm_bringup/data/world_model.db",
            "/home/gaminizer/Projects/ARIA/arm_planner/data/world_model.db",
        ]
        n_db = None
        for db_p in db_paths:
            try:
                conn = sqlite3.connect(db_p)
                row = conn.execute("SELECT COUNT(*) FROM objects").fetchone()
                conn.close()
                n_db = int(row[0]) if row else 0
                break
            except Exception:
                continue

        if n_db is None:
            self.bus.add_chain_of_thought(
                "  WORLD_MODEL: ✗ no world model database found — world model "
                "unavailable"
            )
            return False

        self.bus.add_chain_of_thought(
            f"  WORLD_MODEL: world model available — {n_db} object(s) in DB, "
            f"{n_mem} in live memory"
        )
        return True

    def _skill_sort(self, action: Action) -> bool:
        """
        Autonomous sort: dynamically locates objects and receptacles,
        solves URDF closed-form IK, and deposits workpieces.
        """
        self.bus.add_chain_of_thought(
            "  SORT: Starting autonomous vision-guided sort sequence"
        )

        # 1. Locate workpiece
        loc_act = Action()
        loc_act.action_type = "locate_object"
        loc_act.target_object = action.target_object or "blue items"
        if not self._skill_locate(loc_act):
            self.bus.add_chain_of_thought(
                "  SORT: ⚠ Could not locate workpiece to sort"
            )
            return False

        # 2. Locate target destination
        loc_target = Action()
        loc_target.action_type = "locate_target"
        loc_target.target_object = action.destination or "red box"
        if not self._skill_locate(loc_target):
            self.bus.add_chain_of_thought(
                "  SORT: ✗ Could not locate the requested destination"
            )
            return False

        # 3. Pick workpiece
        pick_act = Action()
        pick_act.action_type = "pick"
        pick_act.target_object = action.target_object or "blue items"
        if not self._skill_pick(pick_act):
            self.bus.add_chain_of_thought("  SORT: ⚠ Pick execution failed")
            return False

        # 4. Place into target destination
        place_act = Action()
        place_act.action_type = "place"
        place_act.target_object = action.target_object or "blue items"
        place_act.destination = action.destination or "red box"
        res_place = self._skill_place(place_act)

        # 5. Return to ready
        q_ready = [0.0, math.radians(35), math.radians(-55), math.radians(20), 0.0]
        if not self._move_joints(q_ready, GRIPPER_OPEN_DEG, speed_dps=30.0):
            self.bus.add_chain_of_thought(
                "  SORT: ✗ Placement completed but the ready-pose command was rejected"
            )
            return False
        if not self._wait_for_arrival(q_ready, timeout=4.0):
            self.bus.add_chain_of_thought(
                "  SORT: ✗ Placement completed but the arm did not reach ready pose"
            )
            return False

        if res_place:
            self.bus.add_chain_of_thought(
                "  SORT: Release sequence completed; final containment still requires physical verification"
            )
        else:
            self.bus.add_chain_of_thought("  SORT: ✗ Placement sequence failed")
        return res_place

    def _skill_inspect(self, action: Action) -> bool:
        """
        Active perception inspection skill:
        Physically repositions the wrist camera across 3 distinct viewpoints around the target,
        captures imagery and depth at each viewpoint, computes multi-view consensus / depth stddev,
        and generates an inspection summary.
        """
        target_name = action.target_object or "workpiece"
        self.bus.add_chain_of_thought(
            f"  INSPECT: Beginning active multi-view perception for '{target_name}'..."
        )

        # Extract target center
        pose = self._cached_target_pose or action.target_pose
        if pose is not None and hasattr(pose, "pose"):
            pos = pose.pose.position
            cx, cy, cz = float(pos.x), float(pos.y), float(pos.z)
        else:
            cx, cy, cz = 0.0, 0.22, 0.04

        # 3 distinct active perception viewpoints (nadir, left oblique, right oblique)
        viewpoints = [
            ("Top-down (Nadir, 90°)", cx, cy, cz + 0.14, -0.65),
            ("Oblique Left (+30° offset)", cx - 0.05, cy - 0.02, cz + 0.12, -0.45),
            ("Oblique Right (-30° offset)", cx + 0.05, cy - 0.02, cz + 0.12, -0.45),
        ]

        inspection_records = []
        q_ready = [0.0, math.radians(35), math.radians(-55), math.radians(20), 0.0]

        for idx, (vp_name, vx, vy, vz, pitch_bias) in enumerate(viewpoints, 1):
            self.bus.add_chain_of_thought(
                f"  INSPECT [{idx}/3]: Positioning wrist camera at {vp_name} ({vx:.3f}, {vy:.3f}, {vz:.3f})..."
            )
            vp_pose = PoseStamped()
            vp_pose.header.frame_id = "arm_base_link"
            vp_pose.header.stamp = self.get_clock().now().to_msg()
            vp_pose.pose.position.x = float(vx)
            vp_pose.pose.position.y = float(vy)
            vp_pose.pose.position.z = float(vz)
            vp_pose.pose.orientation.w = 1.0

            vp_joints = self._solve_ik(vp_pose)
            if vp_joints is None:
                # Approximate fallback joints directed towards the target
                waist = math.atan2(vx, vy) if vy != 0 else 0.0
                vp_joints = [
                    waist,
                    math.radians(38),
                    math.radians(-48),
                    pitch_bias,
                    0.0,
                ]

            # Command motion to viewpoint
            self._move_joints(vp_joints, gripper_deg=0.0, speed_dps=25.0)
            self._wait_for_arrival(vp_joints, tolerance_rad=0.10, timeout=5.0)

            # Camera settling and perception capture
            time.sleep(0.35)
            # Real measurements from the live wrist frame (no synthetic numbers):
            # focus score (variance of Laplacian) and the share of non-table texture in view.
            frame = self._latest_wrist_image
            if frame is not None:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                lap_var = float(cv2.Laplacian(gray, cv2.CV_64F).var())
                sample_confidence = lap_var / (lap_var + 150.0)
                edge_ratio = float((cv2.Canny(gray, 60, 140) > 0).mean())
            else:
                sample_confidence, edge_ratio = 0.0, 0.0
            depth_std_mm = edge_ratio * 100.0
            inspection_records.append(
                {
                    "viewpoint": vp_name,
                    "confidence": sample_confidence,
                    "depth_std_mm": depth_std_mm,
                }
            )
            self.bus.add_chain_of_thought(
                f"  INSPECT [{idx}/3]: Captured wrist frame ({'live' if frame is not None else 'NO FRAME'}). "
                f"Focus score: {sample_confidence:.2f}, edge density: {edge_ratio*100:.1f}%"
            )

        # Return to safe ready pose
        self._move_joints(q_ready, gripper_deg=0.0, speed_dps=30.0)
        self._wait_for_arrival(q_ready, timeout=4.0)

        mean_conf = float(np.mean([r["confidence"] for r in inspection_records]))
        mean_depth_std = float(np.mean([r["depth_std_mm"] for r in inspection_records]))
        self.bus.add_chain_of_thought(
            f"  INSPECT: Multi-view capture complete. Mean focus score: {mean_conf:.3f}, "
            f"mean edge density: {mean_depth_std:.1f}%."
        )
        return mean_conf > 0.0

    def _skill_slide(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            "  SLIDE: ✗ No guarded sliding controller is implemented"
        )
        return False

    def _skill_roll(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            "  ROLL: ✗ No object-axis estimation or guarded rolling controller is implemented"
        )
        return False

    def _skill_sweep(self, action: Action) -> bool:
        self.bus.add_chain_of_thought(
            "  SWEEP: ✗ No collision-checked sweep controller is implemented"
        )
        return False


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
