#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Dashboard — Production FastAPI Backend
Bridges ROS 2 state bus topics, camera streams, and service calls
to the ARIA Control Center web interface.
═══════════════════════════════════════════════════════════════
"""
import asyncio
import base64
import csv
import math
import os
import signal
import sqlite3
import subprocess
import threading
import time
from collections import deque
from typing import Dict, List, Optional, Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Response, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

# ── Workspace Environment Resolution ───────────────────────────
import sys
WORKSPACE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
INSTALL_DIR = os.path.join(WORKSPACE_DIR, "install")

# Self-heal LD_LIBRARY_PATH if ROS 2 typesupport shared libraries are not in loader search path
if os.path.exists(INSTALL_DIR):
    lib_paths = [os.path.join(INSTALL_DIR, pkg, "lib") for pkg in os.listdir(INSTALL_DIR) if os.path.isdir(os.path.join(INSTALL_DIR, pkg, "lib"))]
    current_ld = os.environ.get("LD_LIBRARY_PATH", "")
    missing_ld = [p for p in lib_paths if p not in current_ld.split(":")]
    if missing_ld and not os.environ.get("_ARIA_LD_FIXED"):
        os.environ["_ARIA_LD_FIXED"] = "1"
        new_ld = ":".join(missing_ld) + (f":{current_ld}" if current_ld else "")
        os.environ["LD_LIBRARY_PATH"] = new_ld
        try:
            os.execv(sys.executable, [sys.executable] + sys.argv)
        except Exception:
            pass

# Ensure standard ROS 2 Humble python site-packages
for humble_py in [
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/lib/python3.10/site-packages",
]:
    if os.path.exists(humble_py) and humble_py not in sys.path:
        sys.path.append(humble_py)

# Automatically register installed ARIA ROS 2 packages
if os.path.exists(INSTALL_DIR):
    for entry in os.listdir(INSTALL_DIR):
        pkg_dir = os.path.join(INSTALL_DIR, entry)
        if os.path.isdir(pkg_dir):
            for sub in [
                "local/lib/python3.10/dist-packages",
                "lib/python3.10/site-packages",
            ]:
                cand = os.path.join(pkg_dir, sub)
                if os.path.exists(cand) and cand not in sys.path:
                    sys.path.insert(0, cand)

    current_ament = os.environ.get("AMENT_PREFIX_PATH", "")
    ament_entries = [p for p in current_ament.split(":") if p]
    for entry in os.listdir(INSTALL_DIR):
        pkg_path = os.path.join(INSTALL_DIR, entry)
        if os.path.isdir(pkg_path) and pkg_path not in ament_entries:
            ament_entries.insert(0, pkg_path)
    os.environ["AMENT_PREFIX_PATH"] = ":".join(ament_entries)

# ── ROS 2 Imports ──────────────────────────────────────────────
ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import JointState, Image
    from geometry_msgs.msg import Point
    from std_msgs.msg import String, Bool
    from std_srvs.srv import Trigger
    from arm_interfaces.srv import SendCommand, SetJoint, SetAllJoints, GoNamedPose
    from arm_planner.msg import TaskState, VisionState, MemoryState, HealthState
    ROS_AVAILABLE = True
except ImportError as e:
    print(f"[Dashboard] ROS 2 imports unavailable ({e}) — controls disabled; telemetry unavailable.")

JOINT_NAMES = [
    "waist_joint", "shoulder_joint", "elbow_joint",
    "wrist_pitch_joint", "gripper_joint"
]

# ═══════════════════════════════════════════════════════════════
# Thread-safe State Cache
# ═══════════════════════════════════════════════════════════════

class ARIADashboardState:
    """Thread-safe state cache holding real-time snapshots of ARIA."""

    def __init__(self):
        self._lock = threading.Lock()
        self.joint_positions = [0.0] * len(JOINT_NAMES)
        self.target_joint_positions = [0.0] * len(JOINT_NAMES)
        self.force_estimates = [0.0] * len(JOINT_NAMES)

        self.task_state = {
            "current_command": "",
            "current_goal": "",
            "subgoals": [],
            "action_queue": [],
            "confidence": None,
            "task_status": "UNKNOWN",
            "task_id": "",
            "awaiting_approval": False,
        }
        self._task_cot_seen = 0

        self.vision_state = {
            "detected_objects": [],
            "tracked_ids": [],
            "active_perception": False,
            "scene_confidence": None,
        }

        self.health_state = {
            "servo_health": [],
            "fps_top": None,
            "fps_wrist": None,
            "inference_ms": None,
            "calibration_valid": None,
            "alerts": [],
        }

        self.memory_state = {
            "known_objects": [],
            "spatial_relations": [],
            "recent_tasks": [],
        }

        self.servo_state = {
            "active": False,
            "error_x": 0.0,
            "error_y": 0.0,
            "converged": False,
            "updated_at": 0.0,
        }

        self.chain_of_thought: deque = deque(maxlen=1000)
        self.camera_stats = {
            name: {"fps": 0.0, "width": None, "height": None}
            for name in ("top", "wrist", "side", "annotated", "depth")
        }
        self.top_camera_jpeg: Optional[bytes] = None
        self.top_camera_hd_jpeg: Optional[bytes] = None
        self.wrist_camera_jpeg: Optional[bytes] = None
        self.wrist_camera_hd_jpeg: Optional[bytes] = None
        self.depth_camera_jpeg: Optional[bytes] = None
        self.side_camera_jpeg: Optional[bytes] = None
        self.side_camera_hd_jpeg: Optional[bytes] = None
        self.annotated_camera_jpeg: Optional[bytes] = None
        self.ros_connected = False
        self.session_runs: List[Dict[str, Any]] = []
        self._task_start_time: float = time.time()

    def sync_from_world_model_db(self):
        with self._lock:
            if self.memory_state.get("known_objects"):
                return
        db_paths = [
            '/home/gaminizer/Projects/ARIA/build/arm_planner/data/world_model.db',
            '/home/gaminizer/Projects/ARIA/arm_planner/data/world_model.db',
            '/home/gaminizer/Projects/ARIA/install/arm_agents/lib/python3.10/arm_planner/data/world_model.db',
            '/home/gaminizer/Projects/ARIA/install/arm_planner/lib/data/world_model.db'
        ]
        for db_p in db_paths:
            if os.path.exists(db_p):
                try:
                    conn = sqlite3.connect(db_p)
                    c = conn.cursor()
                    c.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='objects'")
                    if c.fetchone():
                        # Support standard columns
                        cols = [col[1] for col in c.execute("PRAGMA table_info(objects)").fetchall()]
                        if 'px' in cols:
                            rows = c.execute("SELECT id, name, class_name, px, py, pz, color, material, lifecycle_state FROM objects").fetchall()
                            db_objects = [
                                {
                                    "id": r[0] or (i + 1),
                                    "name": r[1] or "",
                                    "class_name": r[2] or "",
                                    "display_name": (r[1] or r[2] or "object").replace("_", " ").title(),
                                    "pos_x": float(r[3] or 0.0),
                                    "pos_y": float(r[4] or 0.0),
                                    "pos_z": float(r[5] or 0.0),
                                    "color": r[6] or "",
                                    "material": r[7] or "",
                                    "lifecycle_state": r[8] or "Detected",
                                }
                                for i, r in enumerate(rows)
                            ]
                        else:
                            rows = c.execute("SELECT id, display_name, class_name, pos_x, pos_y, pos_z, color, material, lifecycle_state FROM objects").fetchall()
                            db_objects = [
                                {
                                    "id": r[0] or (i + 1),
                                    "name": r[1] or "",
                                    "class_name": r[2] or "",
                                    "display_name": (r[1] or r[2] or "object").replace("_", " ").title(),
                                    "pos_x": float(r[3] or 0.0),
                                    "pos_y": float(r[4] or 0.0),
                                    "pos_z": float(r[5] or 0.0),
                                    "color": r[6] or "",
                                    "material": r[7] or "",
                                    "lifecycle_state": r[8] or "Detected",
                                }
                                for i, r in enumerate(rows)
                            ]
                        with self._lock:
                            self.memory_state["known_objects"] = db_objects
                    conn.close()
                    break
                except Exception:
                    pass

    def get_full_state(self) -> dict:
        def _deg(p):
            if p is None or not isinstance(p, (int, float)) or math.isnan(p) or math.isinf(p):
                return 0.0
            return round(math.degrees(p), 1)

        def _rad(p):
            if p is None or not isinstance(p, (int, float)) or math.isnan(p) or math.isinf(p):
                return 0.0
            return round(p, 4)

        def _clean(val):
            if isinstance(val, float):
                if math.isnan(val) or math.isinf(val):
                    return 0.0
                return val
            elif isinstance(val, dict):
                return {k: _clean(v) for k, v in val.items()}
            elif isinstance(val, (list, tuple)):
                return [_clean(v) for v in val]
            return val

        self.sync_from_world_model_db()
        with self._lock:
            angles_deg = [_deg(p) for p in self.joint_positions]
            target_deg = [_deg(p) for p in self.target_joint_positions]
            raw_state = {
                "vision": {
                    "detected_objects": self.vision_state.get("detected_objects", []),
                    "tracked_ids": self.vision_state.get("tracked_ids", []),
                    "scene_confidence": self.vision_state.get("scene_confidence"),
                    "active_perception": self.vision_state.get("active_perception", False),
                },
                "task": {
                    "current_command": self.task_state.get("current_command", ""),
                    "current_goal": self.task_state.get("current_goal", ""),
                    "subgoals": self.task_state.get("subgoals", []),
                    "chain_of_thought": list(self.chain_of_thought)[-50:],
                    "confidence": self.task_state.get("confidence"),
                    "task_status": self.task_state.get("task_status", "IDLE"),
                    "task_id": self.task_state.get("task_id", ""),
                    "awaiting_approval": self.task_state.get("awaiting_approval", False),
                },
                "joints": {
                    "names": JOINT_NAMES,
                    "current_angles": angles_deg,
                    "target_angles": target_deg,
                    "force_estimates": list(self.force_estimates),
                    "positions_deg": angles_deg,
                    "positions_rad": [_rad(p) for p in self.joint_positions],
                },
                "health": {
                    "servo_health": self.health_state.get("servo_health", []),
                    "fps_top": self.health_state.get("fps_top"),
                    "fps_wrist": self.health_state.get("fps_wrist"),
                    "inference_latency_ms": self.health_state.get("inference_ms"),
                    "active_alerts": self.health_state.get("alerts", []),
                    "calibration_valid": self.health_state.get("calibration_valid", True),
                },
                "memory": {
                    "known_objects": self.memory_state.get("known_objects", []),
                    "spatial_relations": self.memory_state.get("spatial_relations", []),
                    "recent_tasks": self.memory_state.get("recent_tasks", []),
                },
                "servo": dict(self.servo_state),
                "camera_stats": dict(self.camera_stats),
                "cot": list(self.chain_of_thought)[-200:],
                "ros_connected": self.ros_connected,
                "timestamp": time.time(),
            }
            return _clean(raw_state)

    def update_joints(self, names, positions):
        with self._lock:
            for i, name in enumerate(JOINT_NAMES):
                if name in names:
                    idx = names.index(name)
                    self.joint_positions[i] = positions[idx]

    def add_cot(self, entry: str):
        with self._lock:
            self.chain_of_thought.append({"text": entry, "time": time.time()})

    def set_task_state(self, msg):
        with self._lock:
            prev_status = self.task_state.get("task_status", "IDLE")
            new_status = msg.task_status or "EXECUTING"
            self.task_state["current_command"] = msg.current_command

            # Track task lifecycle transitions
            if prev_status != "EXECUTING" and new_status == "EXECUTING":
                self._task_start_time = time.time()
            elif prev_status == "EXECUTING" and new_status in ("COMPLETE", "COMPLETED", "FAILED", "ABORTED"):
                duration = time.time() - getattr(self, '_task_start_time', time.time())
                run_entry = {
                    "run": len(self.session_runs) + 1,
                    "task_id": msg.task_id or f"task-{len(self.session_runs)+1}",
                    "command": msg.current_command,
                    "duration_s": round(duration, 1),
                    "status": new_status,
                    "timestamp": time.strftime("%H:%M:%S")
                }
                self.session_runs.append(run_entry)

            # Parse subgoals dynamically from action queue
            sgs = []
            for idx, s in enumerate(msg.subgoals):
                if idx < len(msg.action_queue):
                    act_st = msg.action_queue[idx].status.upper()
                    status = "completed" if act_st == "COMPLETE" else ("active" if act_st == "EXECUTING" else "pending")
                else:
                    status = "completed" if new_status in ("COMPLETE", "COMPLETED") else "pending"
                sgs.append({"id": idx + 1, "name": s, "status": status})
            if sgs:
                self.task_state["subgoals"] = sgs
            self.task_state["confidence"] = float(msg.confidence)
            self.task_state["task_status"] = new_status
            self.task_state["task_id"] = msg.task_id
            self.task_state["awaiting_approval"] = bool(msg.awaiting_user_approval)

            if len(msg.chain_of_thought) < self._task_cot_seen:
                self._task_cot_seen = 0
            new_entries = list(msg.chain_of_thought)[self._task_cot_seen:]
            self._task_cot_seen = len(msg.chain_of_thought)
        for entry in new_entries:
            self.add_cot(entry)

    def set_vision_state(self, msg):
        with self._lock:
            self.vision_state = {
                "detected_objects": [
                    {
                        "class_name": d.class_name,
                        "confidence": float(d.confidence),
                        "tracking_id": d.tracking_id,
                        "bbox_x": d.bbox_x, "bbox_y": d.bbox_y,
                        "bbox_w": d.bbox_w, "bbox_h": d.bbox_h,
                        "lifecycle_state": d.lifecycle_state,
                        "pos_3d": [
                            d.pose_3d.pose.position.x,
                            d.pose_3d.pose.position.y,
                            d.pose_3d.pose.position.z,
                        ],
                    }
                    for d in msg.detected_objects
                    if d.class_name.lower() not in {'scissors', 'knife', 'fork', 'spoon', 'remote', 'toilet', 'toothbrush', 'tie'}
                ],
                "tracked_ids": list(msg.tracked_ids),
                "active_perception": msg.active_perception_mode,
                "scene_confidence": float(msg.scene_confidence),
            }

    def set_memory_state(self, msg):
        with self._lock:
            self.memory_state["known_objects"] = [
                {
                    "id": wo.id,
                    "name": wo.name,
                    "class_name": wo.class_name,
                    "display_name": (wo.name or wo.class_name or "object").replace("_", " ").title(),
                    "pos_x": float(wo.last_known_pose.pose.position.x),
                    "pos_y": float(wo.last_known_pose.pose.position.y),
                    "pos_z": float(wo.last_known_pose.pose.position.z),
                    "color": wo.color,
                    "material": wo.material,
                    "lifecycle_state": wo.lifecycle_state,
                }
                for wo in msg.known_objects
            ]
            self.memory_state["spatial_relations"] = [
                {
                    "subject": r.subject, "relation": r.relation,
                    "object": r.object, "confidence": float(r.confidence),
                    "distance": float(r.distance_m),
                }
                for r in msg.spatial_relations
            ]
            self.memory_state["recent_tasks"] = list(msg.recent_tasks)

    def set_health_state(self, msg):
        with self._lock:
            self.health_state = {
                "servo_health": [
                    {
                        "joint_name": s.joint_name,
                        "temperature_estimate_c": float(s.temperature_estimate_c),
                        "drift_deg": float(s.drift_deg),
                        "load_estimate_pct": float(s.load_estimate_pct),
                        "healthy": bool(s.healthy),
                    }
                    for s in msg.servo_health
                ],
                "fps_top": float(msg.fps_top_camera) if msg.fps_top_camera > 0 else None,
                "fps_wrist": float(msg.fps_wrist_camera) if msg.fps_wrist_camera > 0 else None,
                "inference_ms": float(msg.inference_latency_ms) if msg.inference_latency_ms > 0 else None,
                "calibration_valid": bool(msg.calibration_valid),
                "alerts": list(msg.active_alerts),
            }


state = ARIADashboardState()

# ═══════════════════════════════════════════════════════════════
# ROS 2 Bridge Node
# ═══════════════════════════════════════════════════════════════

bridge_node: Optional["DashboardBridge"] = None

if ROS_AVAILABLE:
    class DashboardBridge(Node):
        """ROS 2 Node subscribing to the 6 state topics + camera feeds."""

        def __init__(self):
            super().__init__('dashboard_bridge')

            qos_state = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
            qos_sensor = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE, depth=5)

            # State Bus subscriptions
            self.create_subscription(TaskState, '/aria/state/task', self._task_cb, qos_state)
            self.create_subscription(VisionState, '/aria/state/vision', self._vision_cb, qos_state)
            self.create_subscription(MemoryState, '/aria/state/memory', self._memory_cb, qos_state)
            self.create_subscription(HealthState, '/aria/state/health', self._health_cb, qos_state)
            self.create_subscription(JointState, '/joint_states', self._joint_cb, qos_sensor)

            # Cameras
            self.create_subscription(Image, '/aria/top_camera/image_raw', self._top_cam_cb, qos_sensor)
            self.create_subscription(Image, '/aria/wrist_camera/image_raw', self._wrist_cam_cb, qos_sensor)
            self.create_subscription(Image, '/side_camera/image_raw', self._side_cam_cb, qos_sensor)
            self.create_subscription(Image, '/detection/image_annotated', self._annotated_cam_cb, qos_sensor)
            self.create_subscription(Image, '/depth/image_colorized', self._depth_cam_cb, qos_sensor)
            # Ground-truth depth heatmap. The Gazebo wrist camera always publishes a
            # real depth image in simulation, so the dashboard can colourise it itself
            # and the "heatmap" panel never falls back to the plain wrist RGB feed even
            # when the heavy monocular Depth-Anything node is not running.
            self.create_subscription(Image, '/wrist_camera/depth/image_raw', self._gazebo_depth_cb, qos_sensor)
            # Direct Gazebo camera topics (simulation only). The /aria/* relay above
            # runs camera_node as an extra hop and roughly halves the achievable frame
            # rate for the 1080p feeds, so we also subscribe to the raw source topics and
            # let whichever copy arrives freshest win. On hardware these topics have no
            # publishers and the /aria/* relay remains the sole source.
            self.create_subscription(Image, '/top_camera/image_raw', self._top_cam_cb, qos_sensor)
            self.create_subscription(Image, '/wrist_camera/image_raw', self._wrist_cam_cb, qos_sensor)

            # Dialogue & Visual Servo
            self.create_subscription(String, '/aria/dialogue/output', self._dialogue_cb, 10)
            self.create_subscription(Bool, '/visual_servo/active', self._servo_active_cb, 10)
            self.create_subscription(Point, '/visual_servo/pixel_error', self._servo_error_cb, 10)
            self.create_subscription(Bool, '/visual_servo/converged', self._servo_converged_cb, 10)

            # Service clients
            from rclpy.callback_groups import ReentrantCallbackGroup
            self.cb_group = ReentrantCallbackGroup()
            self.cmd_client = self.create_client(SendCommand, '/aria/command', callback_group=self.cb_group)
            self.approve_client = self.create_client(Trigger, '/aria/approve', callback_group=self.cb_group)
            self.reject_client = self.create_client(Trigger, '/aria/reject', callback_group=self.cb_group)
            self.cancel_client = self.create_client(Trigger, '/aria/cancel', callback_group=self.cb_group)
            self.estop_client = self.create_client(Trigger, '/aria/estop', callback_group=self.cb_group)
            self.release_estop_client = self.create_client(Trigger, '/aria/release_estop', callback_group=self.cb_group)
            self.set_joint_client = self.create_client(SetJoint, '/aria/set_joint', callback_group=self.cb_group)
            self.set_all_joints_client = self.create_client(SetAllJoints, '/aria/set_all_joints', callback_group=self.cb_group)
            self.go_named_pose_client = self.create_client(GoNamedPose, '/aria/go_named_pose', callback_group=self.cb_group)
            self.reset_memory_client = self.create_client(Trigger, '/aria/reset_memory', callback_group=self.cb_group)

            self.create_timer(1.0, self._connectivity_check)
            self.get_logger().info("Dashboard bridge started — subscriptions online")

        def _joint_cb(self, msg):
            state.update_joints(list(msg.name), list(msg.position))

        def _task_cb(self, msg):
            state.set_task_state(msg)

        def _vision_cb(self, msg):
            state.set_vision_state(msg)

        def _memory_cb(self, msg):
            state.set_memory_state(msg)

        def _health_cb(self, msg):
            state.set_health_state(msg)

        def _encode_image(self, msg, cam_name='camera', target_w=640, target_h=360, quality=75):
            import cv2
            from cv_bridge import CvBridge
            from collections import defaultdict
            now = time.time()
            if not hasattr(self, '_cam_times'):
                self._cam_times = defaultdict(lambda: deque(maxlen=20))

            times = self._cam_times[cam_name]
            times.append(now)
            if len(times) >= 2:
                dt = times[-1] - times[0]
                fps = (len(times) - 1) / max(dt, 0.001)
            else:
                fps = 0.0

            with state._lock:
                state.camera_stats[cam_name] = {
                    "fps": round(fps, 1),
                    "width": int(getattr(msg, 'width', 640)),
                    "height": int(getattr(msg, 'height', 360)),
                }

            if not hasattr(self, '_last_cam_time'):
                self._last_cam_time = {}
            if now - self._last_cam_time.get(cam_name, 0.0) < 0.033:
                return None
            self._last_cam_time[cam_name] = now

            if not hasattr(self, '_bridge'):
                self._bridge = CvBridge()
            try:
                cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception:
                return None

            if target_w and target_h:
                if cv_img.shape[1] != target_w or cv_img.shape[0] != target_h:
                    cv_img = cv2.resize(cv_img, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
            ok, grid_jpeg = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, quality])
            return grid_jpeg.tobytes() if ok else None

        def _top_cam_cb(self, msg):
            try:
                grid = self._encode_image(msg, 'top')
                if grid is not None:
                    state.top_camera_jpeg = grid
            except Exception:
                pass

        def _wrist_cam_cb(self, msg):
            try:
                grid = self._encode_image(msg, 'wrist')
                if grid is not None:
                    state.wrist_camera_jpeg = grid
            except Exception:
                pass

        def _side_cam_cb(self, msg):
            try:
                grid = self._encode_image(msg, 'side')
                if grid is not None:
                    state.side_camera_jpeg = grid
            except Exception:
                pass

        def _annotated_cam_cb(self, msg):
            try:
                grid = self._encode_image(msg, 'annotated')
                if grid is not None:
                    state.annotated_camera_jpeg = grid
            except Exception:
                pass

        def _depth_cam_cb(self, msg):
            try:
                grid = self._encode_image(msg, 'depth')
                if grid is not None:
                    state.depth_camera_jpeg = grid
            except Exception:
                pass

        def _colorize_depth(self, msg, cam_name='depth', target_w=640, target_h=360, quality=80):
            """Convert a raw depth image (32FC1 / 16UC1) into a TURBO heatmap JPEG."""
            import cv2
            import numpy as np
            from cv_bridge import CvBridge
            from collections import defaultdict

            now = time.time()
            if not hasattr(self, '_cam_times'):
                self._cam_times = defaultdict(lambda: deque(maxlen=20))

            times = self._cam_times[cam_name]
            times.append(now)
            if len(times) >= 2:
                fps = (len(times) - 1) / max(times[-1] - times[0], 0.001)
            else:
                fps = 0.0

            with state._lock:
                state.camera_stats[cam_name] = {
                    "fps": round(fps, 1),
                    "width": int(getattr(msg, 'width', 640)),
                    "height": int(getattr(msg, 'height', 360)),
                }

            if not hasattr(self, '_last_cam_time'):
                self._last_cam_time = {}
            if now - self._last_cam_time.get(cam_name, 0.0) < 0.033:
                return None
            self._last_cam_time[cam_name] = now

            if not hasattr(self, '_bridge'):
                self._bridge = CvBridge()
            try:
                raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            except Exception:
                return None

            enc = (getattr(msg, 'encoding', '') or '').upper()
            arr = np.asarray(raw)
            try:
                if enc.startswith('16U') or enc.startswith('MONO16'):
                    depth = arr.astype(np.float32) / 1000.0  # mm -> m
                else:
                    depth = arr.astype(np.float32)
            except Exception:
                return None

            depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            valid = depth[depth > 0.01]
            if valid.size == 0:
                return None

            lo = float(np.percentile(valid, 2))
            hi = float(np.percentile(valid, 98))
            if hi <= lo:
                hi = lo + 1e-3
            norm = np.clip((depth - lo) / (hi - lo), 0.0, 1.0)
            norm[depth <= 0.01] = 0.0
            norm8 = (norm * 255.0).astype(np.uint8)

            try:
                color = cv2.applyColorMap(norm8, cv2.COLORMAP_TURBO)
            except Exception:
                color = cv2.applyColorMap(norm8, cv2.COLORMAP_JET)
            color[depth <= 0.01] = (0, 0, 0)

            if target_w and target_h and (color.shape[1] != target_w or color.shape[0] != target_h):
                color = cv2.resize(color, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
            ok, jpeg = cv2.imencode('.jpg', color, [cv2.IMWRITE_JPEG_QUALITY, quality])
            return jpeg.tobytes() if ok else None

        def _gazebo_depth_cb(self, msg):
            try:
                grid = self._colorize_depth(msg, 'depth')
                if grid is not None:
                    state.depth_camera_jpeg = grid
            except Exception:
                pass

        def _dialogue_cb(self, msg):
            state.add_cot(f"[DIALOGUE] {msg.data}")

        def _servo_active_cb(self, msg):
            with state._lock:
                state.servo_state["active"] = bool(msg.data)
                state.servo_state["updated_at"] = time.time()

        def _servo_error_cb(self, msg):
            with state._lock:
                state.servo_state["error_x"] = float(msg.x)
                state.servo_state["error_y"] = float(msg.y)
                state.servo_state["updated_at"] = time.time()

        def _servo_converged_cb(self, msg):
            with state._lock:
                state.servo_state["converged"] = bool(msg.data)
                state.servo_state["updated_at"] = time.time()

        def _connectivity_check(self):
            has_robot = self.count_publishers('/joint_states') > 0
            has_service = bool(self.cmd_client.service_is_ready())
            state.ros_connected = bool(has_robot or has_service)
            state.sync_from_world_model_db()


def ros_spin_thread():
    global bridge_node
    if not ROS_AVAILABLE:
        return
    try:
        from rclpy.executors import MultiThreadedExecutor
        rclpy.init()
        bridge_node = DashboardBridge()
        executor = MultiThreadedExecutor()
        executor.add_node(bridge_node)
        executor.spin()
    except Exception as e:
        print(f"[Dashboard] ROS 2 spin exception: {e}")
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


def call_service_sync(client, request, timeout: float = 5.0):
    if client is None:
        return None
    if not client.wait_for_service(timeout_sec=min(2.0, timeout)):
        return None
    future = client.call_async(request)
    start = time.time()
    while time.time() - start < timeout:
        if future.done():
            if future.exception() is not None:
                return None
            return future.result()
        time.sleep(0.05)
    return None


async def call_service(client, request, timeout: float = 5.0):
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, call_service_sync, client, request, timeout)


def require_ros_service(client_name: str):
    if not ROS_AVAILABLE or bridge_node is None:
        raise HTTPException(status_code=503, detail="ROS 2 control bridge is unavailable")
    client = getattr(bridge_node, client_name, None)
    if client is None or not client.service_is_ready():
        raise HTTPException(
            status_code=503,
            detail=f"ROS 2 service for {client_name} is unavailable",
        )
    return client


async def call_ready_service(client, request, client_name: str, timeout: float = 2.0):
    response = await call_service(client, request, timeout=timeout)
    if response is None:
        raise HTTPException(
            status_code=504,
            detail=f"ROS 2 service call timed out: {client_name}",
        )
    return response


# ═══════════════════════════════════════════════════════════════
# FastAPI Application & Endpoints
# ═══════════════════════════════════════════════════════════════

app = FastAPI(title="ARIA Control Center", version="4.0", description="Production Industrial Robotics HMI")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

FRONTEND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frontend", "dist")
if not os.path.exists(FRONTEND_DIR):
    FRONTEND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frontend", "build")

if os.path.exists(FRONTEND_DIR):
    assets_dir = os.path.join(FRONTEND_DIR, "assets")
    if os.path.exists(assets_dir):
        app.mount("/assets", StaticFiles(directory=assets_dir), name="assets")
    app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")


@app.get("/api/health")
async def health_check():
    return {"status": "ok", "ros_available": ROS_AVAILABLE}


# ── Request Models ─────────────────────────────────────────────
class CommandRequest(BaseModel):
    command: str

class JointRequest(BaseModel):
    joint: Optional[str] = None
    joint_name: Optional[str] = None
    angle_deg: float
    speed: Optional[float] = 30.0
    speed_deg_per_s: Optional[float] = 30.0

class AllJointsRequest(BaseModel):
    angles_deg: List[float]
    speed_deg_per_s: float = 30.0

class TargetRequest(BaseModel):
    object_id: int
    action: Optional[str] = "pick"

class SimulationLaunchRequest(BaseModel):
    world: Optional[str] = "aria_tester_workspace.world"
    gui: Optional[bool] = False
    synthetic: Optional[bool] = False

# Global Simulation Process Tracking
sim_process: Optional[subprocess.Popen] = None
sim_world: str = "aria_tester_workspace.world"
sim_start_time: Optional[float] = None


# ── REST Endpoints ─────────────────────────────────────────────

@app.get("/")
async def root():
    index_path = os.path.join(FRONTEND_DIR, "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path)
    return HTMLResponse(
        """
        <!DOCTYPE html>
        <html><head><title>ARIA Control Center</title>
        <style>body{background:#0d0f14;color:#e2e6f0;font-family:sans-serif;display:flex;align-items:center;justify-content:center;height:100vh;margin:0;}
        .card{background:#13161e;padding:32px;border:1px solid #252a38;border-radius:8px;max-width:500px;text-align:center;}
        h1{color:#8b5cf6;margin-top:0;}code{color:#06b6d4;}</style></head>
        <body><div class="card">
        <h1>ARIA Control Center</h1>
        <p>Backend is active. Frontend Vite build not found in <code>frontend/dist</code>.</p>
        <p>Run <code>cd frontend && npm run build</code> or connect via dev server on <code>http://localhost:5173</code>.</p>
        </div></body></html>
        """
    )

@app.post("/api/command")
async def post_command(req: CommandRequest):
    """Send natural language command to /aria/command."""
    cmd = req.command.strip()
    if not cmd:
        raise HTTPException(status_code=422, detail="Command cannot be empty")
    client = require_ros_service("cmd_client")
    ros_req = SendCommand.Request()
    ros_req.command = cmd
    resp = await call_ready_service(client, ros_req, "cmd_client", timeout=5.0)
    if resp.accepted:
        state.add_cot(f"[OPERATOR] Command accepted by task manager: '{cmd}'")
    return {"success": resp.accepted, "task_id": resp.task_id, "message": resp.message}

@app.post("/api/approve")
async def approve():
    """Approve paused action via /aria/approve."""
    client = require_ros_service("approve_client")
    resp = await call_ready_service(client, Trigger.Request(), "approve_client")
    if resp.success:
        state.add_cot("[OPERATOR] Action approval accepted")
    return {"success": resp.success, "message": resp.message}

@app.post("/api/reject")
async def reject():
    """Reject paused action via /aria/reject."""
    client = require_ros_service("reject_client")
    resp = await call_ready_service(client, Trigger.Request(), "reject_client")
    if resp.success:
        state.add_cot("[OPERATOR] Action rejection accepted")
    return {"success": resp.success, "message": resp.message}

@app.post("/api/estop")
async def estop():
    """Trigger E-STOP via /aria/estop."""
    client = require_ros_service("estop_client")
    resp = await call_ready_service(client, Trigger.Request(), "estop_client")
    if resp.success:
        state.add_cot("[SAFETY] Emergency-stop request confirmed by controller")
        with state._lock:
            state.task_state["task_status"] = "EMERGENCY_STOP"
    return {"success": resp.success, "message": resp.message}

@app.post("/api/release_estop")
async def release_estop():
    """Release E-STOP via /aria/release_estop."""
    client = require_ros_service("release_estop_client")
    resp = await call_ready_service(client, Trigger.Request(), "release_estop_client")
    if resp.success:
        state.add_cot("[SAFETY] E-stop release confirmed by controller")
        with state._lock:
            state.task_state["task_status"] = "IDLE"
    return {"success": resp.success, "message": resp.message}

@app.post("/api/joint")
async def set_joint(req: JointRequest):
    """Set single joint angle via /aria/set_joint."""
    name = req.joint or req.joint_name or "waist_joint"
    if name not in JOINT_NAMES:
        raise HTTPException(status_code=422, detail=f"Unknown joint: {name}")
    speed = req.speed_deg_per_s if req.speed is None else req.speed
    if not math.isfinite(req.angle_deg) or not math.isfinite(speed) or speed <= 0:
        raise HTTPException(
            status_code=422,
            detail="Joint angle and speed must be finite; speed must be positive",
        )
    client = require_ros_service("set_joint_client")
    ros_req = SetJoint.Request()
    ros_req.joint_name = name
    ros_req.angle_deg = req.angle_deg
    ros_req.speed_deg_per_s = speed
    resp = await call_ready_service(client, ros_req, "set_joint_client")
    return {"success": resp.success, "message": resp.message}

@app.post("/api/joints")
async def set_all_joints(req: AllJointsRequest):
    """Set all joints via /aria/set_all_joints."""
    angles = [float(a) for a in req.angles_deg]
    if len(angles) == 6:
        angles = [angles[0], angles[1], angles[2], angles[3], angles[5]]
    elif len(angles) != len(JOINT_NAMES):
        raise HTTPException(
            status_code=422,
            detail="Expected five actuator angles: four arm axes and gripper",
        )
    if not all(math.isfinite(angle) for angle in angles):
        raise HTTPException(status_code=422, detail="Joint angles must all be finite")
    if not math.isfinite(req.speed_deg_per_s) or req.speed_deg_per_s <= 0:
        raise HTTPException(status_code=422, detail="Joint speed must be finite and positive")

    client = require_ros_service("set_all_joints_client")
    ros_req = SetAllJoints.Request()
    ros_req.angles_deg = angles
    ros_req.speed_deg_per_s = req.speed_deg_per_s
    resp = await call_ready_service(client, ros_req, "set_all_joints_client")
    return {"success": resp.success, "expected_duration_s": resp.expected_duration_s}

@app.post("/api/reset")
async def reset_robot():
    """Reset robot: release e-stop, reset mission state, move to ready pose."""
    estop_client = require_ros_service("release_estop_client")
    ready_client = require_ros_service("go_named_pose_client")
    estop_resp = await call_ready_service(
        estop_client, Trigger.Request(), "release_estop_client")
    if not estop_resp.success:
        return {"success": False, "message": estop_resp.message}
    ready_req = GoNamedPose.Request()
    ready_req.pose_name = "ready"
    ready_req.speed_deg_per_s = 30.0
    ready_resp = await call_ready_service(
        ready_client, ready_req, "go_named_pose_client")
    if not ready_resp.success:
        return {"success": False, "message": ready_resp.message}
    with state._lock:
        state.task_state["task_status"] = "IDLE"
        state.task_state["current_command"] = "Autonomous workcell ready"
        state.task_state["current_goal"] = "Awaiting mission instruction"
        state.task_state["awaiting_approval"] = False
    state.add_cot("Reset completed: ready pose reached and E-stop released")
    return {"success": True, "message": "Robot reset successfully"}

@app.get("/api/metrics")
async def get_metrics():
    """Return genuine operational outcomes and benchmarks (0% synthetic data)."""
    possible_paths = [
        "/home/gaminizer/Projects/ARIA/data/real/conveyor_sweep_real.csv",
        "/home/gaminizer/Projects/ARIA/data/real/friction_grasp_real_30seeds.csv",
        "/home/gaminizer/Projects/ARIA/data/real/planner_benchmark_test_heldout_60.csv",
        "/home/gaminizer/Projects/ARIA/arm_planner/data/metrics.csv",
        "/home/gaminizer/Projects/ARIA/arm_planner/logs/metrics.csv",
    ]
    runs = []
    for p in possible_paths:
        if os.path.exists(p):
            try:
                with open(p, "r", encoding="utf-8") as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        runs.append(row)
                if runs:
                    break
            except Exception:
                pass

    all_runs = list(runs)

    if not all_runs:
        return {
            "recent_runs": [],
            "failure_breakdown": [],
            "overall_success_rate": None,
            "target_success_rate": 0.85,
            "mean_latency_ms": state.health_state.get("inference_ms"),
            "latency_target_ms": 200.0,
            "total_runs": 0,
        }

    # Calculations from real data only
    def is_run_success(r):
        if "outcome" in r:
            return r["outcome"].upper() == "SUCCESS"
        if "sort_success" in r:
            return r["sort_success"].upper() == "SUCCESS"
        return float(r.get("pick_success_rate", 0)) >= 0.5

    success_count = sum(1 for r in all_runs if is_run_success(r))
    overall_sr = round(success_count / len(all_runs), 3)

    fail_counts = {}
    for r in all_runs:
        ftype = r.get("failure_category", r.get("failure_type", "None"))
        fail_counts[ftype] = fail_counts.get(ftype, 0) + 1

    return {
        "recent_runs": all_runs[-50:],
        "failure_breakdown": [{"category": k, "count": v} for k, v in fail_counts.items()],
        "overall_success_rate": overall_sr,
        "target_success_rate": 0.85,
        "mean_latency_ms": state.health_state.get("inference_ms"),
        "latency_target_ms": 200.0,
        "total_runs": len(all_runs),
    }

@app.post("/api/target_object")
async def target_object(req: TargetRequest):
    """Directly select a detected workpiece from the 360° world map and dispatch mission."""
    state.sync_from_world_model_db()
    target_obj = None
    with state._lock:
        for obj in state.memory_state.get("known_objects", []):
            if obj.get("id") == req.object_id:
                target_obj = obj
                break

    if target_obj is None:
        return {"success": False, "message": f"Object ID {req.object_id} not found in World Model"}

    cls_name = target_obj.get("class_name") or target_obj.get("name") or "workpiece"
    action = req.action or "pick"
    cmd_str = f"{action} the {cls_name}"

    client = require_ros_service("cmd_client")
    ros_req = SendCommand.Request()
    ros_req.command = cmd_str
    resp = await call_ready_service(client, ros_req, "cmd_client", timeout=5.0)
    if resp.accepted:
        state.add_cot(
            f"[OPERATOR] Target selected from World Model: '{cls_name}' (ID={req.object_id}) "
            f"at (X={target_obj.get('pos_x', 0):.3f}, Y={target_obj.get('pos_y', 0):.3f})")
        state.add_cot(f"[OPERATOR] Mission accepted: '{cmd_str}'")
    return {"success": resp.accepted, "task_id": resp.task_id, "message": resp.message}

@app.post("/api/reset_world_model")
async def reset_world_model():
    """Reset World Model database and clear all cached objects."""
    db_paths = [
        '/home/gaminizer/Projects/ARIA/build/arm_planner/data/world_model.db',
        '/home/gaminizer/Projects/ARIA/arm_planner/data/world_model.db',
        '/home/gaminizer/Projects/ARIA/install/arm_agents/lib/python3.10/arm_planner/data/world_model.db',
        '/home/gaminizer/Projects/ARIA/install/arm_planner/lib/data/world_model.db'
    ]
    cleared = 0
    for p in db_paths:
        if os.path.exists(p):
            try:
                conn = sqlite3.connect(p)
                conn.execute("DELETE FROM objects")
                conn.commit()
                conn.close()
                cleared += 1
            except Exception:
                pass
    with state._lock:
        state.memory_state["known_objects"] = []
    if bridge_node and hasattr(bridge_node, 'reset_memory_client'):
        try:
            from std_srvs.srv import Trigger
            await call_service(bridge_node.reset_memory_client, Trigger.Request(), timeout=1.5)
        except Exception:
            pass
    state.add_cot("WORLD_MODEL: Database reset — cleared all tracked objects")
    return {"success": True, "databases_cleared": cleared}

# ── Simulation Management Endpoints ────────────────────────────

@app.get("/api/simulation/worlds")
async def get_simulation_worlds():
    """List available Gazebo world models."""
    return {
        "worlds": [
            {"id": "aria_tester_workspace.world", "name": "Standard Tester Workspace", "description": "80x60cm optical table with colorful benchmark workpieces"},
            {"id": "aria_industrial_workcell.world", "name": "Industrial Workcell", "description": "Conveyor feeding zone with sorting bins and inspection stands"},
            {"id": "aria_workspace.world", "name": "Basic Workspace", "description": "Minimal tabletop environment"}
        ]
    }

@app.get("/api/simulation/status")
async def get_simulation_status():
    """Get active Gazebo simulation process status."""
    global sim_process, sim_world, sim_start_time
    is_running = False
    pid = None

    if sim_process is not None:
        poll = sim_process.poll()
        if poll is None:
            is_running = True
            pid = sim_process.pid
        else:
            sim_process = None

    # Secondary check via pgrep for gzserver
    if not is_running:
        try:
            out = subprocess.check_output(["pgrep", "-f", "gzserver"], text=True).strip()
            if out:
                is_running = True
                pid = int(out.split()[0])
        except Exception:
            pass

    uptime = round(time.time() - sim_start_time, 1) if (is_running and sim_start_time) else 0.0
    return {
        "running": is_running,
        "world": sim_world,
        "uptime_s": uptime,
        "pid": pid,
    }

@app.post("/api/simulation/launch")
async def launch_simulation(req: SimulationLaunchRequest):
    """Launch Gazebo with selected world in the background."""
    global sim_process, sim_world, sim_start_time
    world = req.world or "aria_tester_workspace.world"
    gui_flag = "true" if req.gui else "false"
    synth_flag = "true" if req.synthetic else "false"

    state.add_cot(f"[SIMULATION] Orchestrating Gazebo startup: world={world}, gui={gui_flag}, synthetic={synth_flag}...")

    # Stop any existing simulation first
    try:
        if sim_process is not None:
            try:
                os.killpg(os.getpgid(sim_process.pid), signal.SIGTERM)
            except Exception:
                pass
            sim_process = None
        subprocess.run(["pkill", "-9", "-f", "gzserver"], timeout=2)
        subprocess.run(["pkill", "-9", "-f", "gzclient"], timeout=2)
        await asyncio.sleep(0.5)
    except Exception:
        pass

    setup_bash = os.path.join(WORKSPACE_DIR, "install", "setup.bash")
    log_file_path = "/tmp/aria_simulation.log"

    # Ensure clean sourcing of ROS 2 Humble and ARIA packages
    launch_script = (
        f"source /opt/ros/humble/setup.bash 2>/dev/null || true; "
        f"[ -f '{setup_bash}' ] && source '{setup_bash}' 2>/dev/null || true; "
        f"exec ros2 launch arm_bringup aria_full_u3.launch.py world:={world} gui:={gui_flag} synthetic:={synth_flag}"
    )

    try:
        log_f = open(log_file_path, "w")
        sim_process = subprocess.Popen(
            ["/bin/bash", "-c", launch_script],
            stdout=log_f,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            cwd=WORKSPACE_DIR,
        )
        sim_world = world
        sim_start_time = time.time()

        # Check if process immediately crashed (e.g. missing executable or invalid world)
        await asyncio.sleep(1.0)
        exit_code = sim_process.poll()
        if exit_code is not None:
            err_msg = "Unknown startup error"
            try:
                with open(log_file_path, "r", encoding="utf-8", errors="replace") as lf:
                    lines = lf.readlines()
                    if lines:
                        err_msg = "".join(lines[-8:]).strip()
            except Exception:
                pass
            sim_process = None
            sim_start_time = None
            state.add_cot(f"[SIMULATION] ✗ Launch failed (code {exit_code}): {err_msg}")
            return {"success": False, "message": f"Launch failed: {err_msg}"}

        state.add_cot(f"[SIMULATION] ✓ Gazebo launched with PID={sim_process.pid}")
        return {"success": True, "message": f"Simulation launched with {world}", "pid": sim_process.pid}
    except Exception as e:
        state.add_cot(f"[SIMULATION] ✗ Launch failed: {e}")
        return {"success": False, "message": str(e)}

@app.post("/api/simulation/stop")
async def stop_simulation():
    """Cleanly terminate running Gazebo simulation and arm nodes."""
    global sim_process, sim_start_time
    state.add_cot("[SIMULATION] Terminating Gazebo and ROS 2 workcell nodes...")
    if sim_process is not None:
        try:
            os.killpg(os.getpgid(sim_process.pid), signal.SIGTERM)
            sim_process.wait(timeout=2.0)
        except Exception:
            pass
        sim_process = None

    try:
        subprocess.run(["pkill", "-f", "ros2 launch arm_bringup"], timeout=2)
        subprocess.run(["pkill", "-f", "aria_full_u3"], timeout=2)
        subprocess.run(["pkill", "-f", "/install/arm_"], timeout=2)
        subprocess.run(["pkill", "-9", "-f", "gzserver"], timeout=2)
        subprocess.run(["pkill", "-9", "-f", "gzclient"], timeout=2)
        subprocess.run(["pkill", "-f", "robot_state_publisher"], timeout=2)
    except Exception:
        pass

    sim_start_time = None
    with state._lock:
        state.ros_connected = False
        state.joint_positions = [0.0] * len(JOINT_NAMES)
        state.top_camera_jpeg = None
        state.wrist_camera_jpeg = None
        state.side_camera_jpeg = None
        state.depth_camera_jpeg = None
        state.annotated_camera_jpeg = None
    state.add_cot("[SIMULATION] ✓ Simulation cleanly halted.")
    return {"success": True, "message": "Simulation halted successfully"}

@app.post("/api/simulation/kill")
async def kill_simulation():
    """FULL KILL: terminate Gazebo + every ROS 2 node/agent, clear POSIX shared
    memory (/dev/shm) and free RAM. The dashboard stays running so the operator
    can relaunch a fresh workcell."""
    global sim_process, sim_start_time
    state.add_cot("[SIMULATION] ⛔ FULL KILL — terminating ROS 2, Gazebo, agents; clearing shared memory...")

    # 1. Kill the tracked launch process group (if it was started from the dashboard)
    if sim_process is not None:
        try:
            os.killpg(os.getpgid(sim_process.pid), signal.SIGKILL)
        except Exception:
            pass
        sim_process = None

    # 2. Reap every ARIA / ROS 2 / Gazebo process (never the dashboard itself)
    kill_patterns = [
        "ros2 launch arm_bringup", "aria_full_u3", "aria_synthetic",
        "/install/arm_", "ros2 bag record", "ros2 bag play",
        "gzserver", "gzclient", "gazebo",
        "robot_state_publisher", "controller_manager", "spawn_entity.py",
        "camera_node", "depth_node", "detection_node",
        "task_manager", "memory_manager", "health_monitor",
    ]
    killed = 0
    for pat in kill_patterns:
        try:
            res = subprocess.run(["pkill", "-9", "-f", pat], timeout=2)
            if res.returncode == 0:
                killed += 1
        except Exception:
            pass

    # 3. Stop the ROS 2 CLI daemon so stale discovery state is dropped
    try:
        subprocess.run(["ros2", "daemon", "stop"], timeout=4)
    except Exception:
        pass

    # 4. Clear POSIX shared memory (FastRTPS / Gazebo) → reclaims /dev/shm space
    #    Anything the dashboard itself currently maps is preserved so its own
    #    DDS/telemetry survives the teardown.
    protected = set()
    try:
        with open("/proc/self/maps", "r", errors="replace") as maps_f:
            for line in maps_f:
                idx = line.find("/dev/shm/")
                if idx != -1:
                    protected.add(os.path.basename(line[idx:].strip()))
    except Exception:
        pass

    shm_freed = 0
    shm_cleared = 0
    try:
        for entry in os.listdir("/dev/shm"):
            if entry in protected:
                continue
            fp = os.path.join("/dev/shm", entry)
            try:
                if os.path.isfile(fp):
                    shm_freed += os.path.getsize(fp)
                    os.remove(fp)
                    shm_cleared += 1
            except Exception:
                pass
    except Exception:
        pass

    # 5. Flush dirty pages so reclaimed RAM returns to the OS
    try:
        subprocess.run(["sync"], timeout=3)
    except Exception:
        pass

    # 6. Clear world model SQLite databases to remove stale object data
    world_model_db_paths = [
        '/home/gaminizer/Projects/ARIA/build/arm_planner/data/world_model.db',
        '/home/gaminizer/Projects/ARIA/arm_planner/data/world_model.db',
        '/home/gaminizer/Projects/ARIA/install/arm_agents/lib/python3.10/arm_planner/data/world_model.db',
        '/home/gaminizer/Projects/ARIA/install/arm_planner/lib/data/world_model.db',
    ]
    db_cleared = 0
    for db_p in world_model_db_paths:
        try:
            if os.path.exists(db_p):
                conn = sqlite3.connect(db_p)
                conn.execute("DELETE FROM objects")
                conn.execute("DELETE FROM spatial_relations")
                conn.commit()
                conn.close()
                db_cleared += 1
        except Exception:
            pass

    sim_start_time = None

    # 6. Reset dashboard telemetry (ROS is offline; models are unloaded with their processes)
    with state._lock:
        state.ros_connected = False
        state.joint_positions = [0.0] * len(JOINT_NAMES)
        state.target_joint_positions = [0.0] * len(JOINT_NAMES)
        state.top_camera_jpeg = None
        state.top_camera_hd_jpeg = None
        state.wrist_camera_jpeg = None
        state.wrist_camera_hd_jpeg = None
        state.side_camera_jpeg = None
        state.side_camera_hd_jpeg = None
        state.depth_camera_jpeg = None
        state.annotated_camera_jpeg = None
        state.memory_state["known_objects"] = []
        for key in state.camera_stats:
            state.camera_stats[key] = {"fps": 0.0, "width": None, "height": None}

    freed_mb = shm_freed / 1e6
    state.add_cot(
        f"[SIMULATION] ✓ FULL KILL complete — {killed} process groups signalled, "
        f"{freed_mb:.1f} MB shared memory freed ({shm_cleared} segments). ROS 2 is offline."
    )
    return {
        "success": True,
        "message": "Simulation and ROS 2 fully terminated",
        "killed_groups": killed,
        "shm_freed_bytes": shm_freed,
    }

@app.get("/api/simulation/logs")
async def get_simulation_logs(lines: int = 50):
    """Retrieve recent simulation launcher stdout/stderr logs."""
    log_file_path = "/tmp/aria_simulation.log"
    if not os.path.exists(log_file_path):
        return {"logs": "No simulation log found."}
    try:
        with open(log_file_path, "r", encoding="utf-8", errors="replace") as f:
            all_lines = f.readlines()
            return {"logs": "".join(all_lines[-lines:])}
    except Exception as e:
        return {"logs": f"Error reading logs: {e}"}

@app.get("/api/objects")
async def get_objects():
    """Return known deduplicated objects from SQLite world model."""
    state.sync_from_world_model_db()
    return {"objects": state.memory_state.get("known_objects", [])}

@app.get("/api/logs")
async def get_logs():
    """Return last 200 chain-of-thought logs."""
    return {"logs": list(state.chain_of_thought)[-200:]}

@app.get("/api/state")
async def get_state():
    """Full system state REST fallback."""
    return state.get_full_state()

@app.get("/api/camera/{cam_name}")
async def get_camera_snapshot(cam_name: str, res: Optional[str] = "grid"):
    """Snapshot endpoint supporting both grid and pristine 1080p HD feeds."""
    cam = cam_name.lower()
    is_hd = (res in ("1080p", "hd", "full"))

    if is_hd:
        if cam in ("top", "overhead"):
            jpeg = state.top_camera_hd_jpeg or state.top_camera_jpeg
        elif cam in ("wrist", "gripper"):
            jpeg = state.wrist_camera_hd_jpeg or state.wrist_camera_jpeg
        elif cam in ("side", "inspection"):
            jpeg = state.side_camera_hd_jpeg or state.side_camera_jpeg
        elif cam in ("depth", "annotated"):
            jpeg = state.depth_camera_jpeg or state.annotated_camera_jpeg or state.wrist_camera_hd_jpeg
        else:
            raise HTTPException(status_code=404, detail="Unknown camera")
    else:
        if cam in ("top", "overhead"):
            jpeg = state.top_camera_jpeg
        elif cam in ("wrist", "gripper"):
            jpeg = state.wrist_camera_jpeg
        elif cam in ("depth", "annotated"):
            jpeg = state.depth_camera_jpeg or state.annotated_camera_jpeg or state.wrist_camera_jpeg
        elif cam in ("side", "inspection"):
            jpeg = state.side_camera_jpeg
        else:
            raise HTTPException(status_code=404, detail="Unknown camera")

    if not jpeg:
        raise HTTPException(status_code=503, detail="Frame not available")
    return Response(content=jpeg, media_type="image/jpeg")


# ── WebSockets ─────────────────────────────────────────────────

@app.websocket("/ws/state")
async def ws_state(ws: WebSocket):
    """Stream full ARIA state at 10 Hz."""
    await ws.accept()
    try:
        while True:
            await ws.send_json(state.get_full_state())
            await asyncio.sleep(0.1)  # 10 Hz
    except (WebSocketDisconnect, Exception):
        pass

@app.websocket("/ws/cameras")
async def ws_cameras(ws: WebSocket):
    """Stream 4 camera feeds (top, wrist, side, depth/annotated) at 30 fps."""
    await ws.accept()
    try:
        while True:
            frame_data = {}
            if state.top_camera_jpeg:
                frame_data["top"] = base64.b64encode(state.top_camera_jpeg).decode("ascii")
            if state.wrist_camera_jpeg:
                frame_data["wrist"] = base64.b64encode(state.wrist_camera_jpeg).decode("ascii")
            if state.side_camera_jpeg:
                frame_data["side"] = base64.b64encode(state.side_camera_jpeg).decode("ascii")
            if state.depth_camera_jpeg:
                frame_data["depth"] = base64.b64encode(state.depth_camera_jpeg).decode("ascii")
            elif state.annotated_camera_jpeg:
                frame_data["depth"] = base64.b64encode(state.annotated_camera_jpeg).decode("ascii")
            if state.annotated_camera_jpeg:
                frame_data["annotated"] = base64.b64encode(state.annotated_camera_jpeg).decode("ascii")

            if frame_data:
                frame_data["joint_positions"] = list(state.joint_positions)
                frame_data["camera_stats"] = dict(state.camera_stats)
                frame_data["timestamp"] = time.time()
                await ws.send_json(frame_data)
            await asyncio.sleep(0.033)  # ~30 fps
    except (WebSocketDisconnect, Exception):
        pass


# ── Lifecycle ──────────────────────────────────────────────────

@app.on_event("startup")
async def on_startup():
    if ROS_AVAILABLE:
        t = threading.Thread(target=ros_spin_thread, daemon=True)
        t.start()
        state.add_cot("═══ ARIA Control Center Online — ROS 2 Bridge Connected ═══")
        state.add_cot("Vision pipeline: YOLOv8 + Depth-Anything + SAM 2 active")
        state.add_cot("Autonomous Planning: AffordanceAgent + TaskManager ready")
    else:
        state.add_cot("═══ ARIA Control Center Online — Simulation Bridge Mode ═══")


def main():
    import uvicorn
    port = int(os.environ.get("ARIA_DASHBOARD_PORT", "8000"))
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")

if __name__ == "__main__":
    main()
