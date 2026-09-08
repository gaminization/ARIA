#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Dashboard — FastAPI Backend

Thin client over the real ARIA ROS2 agent graph. This backend does
NOT simulate, re-implement, or fake any part of the pipeline — it
only bridges the real state bus topics (/aria/state/*) and the real
services (/aria/command, /aria/approve, /aria/reject, /aria/estop,
/aria/set_joint, /aria/set_all_joints, ...) to the web UI.

All perception (YOLO, depth), planning (PlanningAgent), grasp
planning (AffordanceAgent/GraspNode), IK (IKNode), execution
(SkillAgent/ControlAgent/manual_control_node) and world modeling
(MemoryAgent/WorldModelAgent) happen inside the real ROS2 nodes
launched by aria_full.launch.py. This file only observes and
forwards commands.
═══════════════════════════════════════════════════════════════
"""
import asyncio
import base64
import math
import os
import threading
import time
from collections import deque
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

# ── ROS2 bridge (runs in background thread) ───────────────
ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import JointState, Image
    from std_msgs.msg import String, Bool
    from std_srvs.srv import Trigger
    from arm_interfaces.srv import SendCommand, SetJoint, SetAllJoints
    from arm_planner.msg import TaskState, VisionState, MemoryState, HealthState
    ROS_AVAILABLE = True
except ImportError as e:
    print(f"[Dashboard] ROS2 imports unavailable ({e}) — dashboard will run "
          f"in read-only demo mode. Source install/setup.bash and run inside "
          f"the ARIA workspace to enable real control.")


# ═══════════════════════════════════════════════════════════════
# State Cache (thread-safe, updated by ROS bridge)
# ═══════════════════════════════════════════════════════════════

JOINT_NAMES = [
    "waist_joint", "shoulder_joint", "elbow_joint",
    "wrist_pitch_joint", "wrist_roll_joint", "gripper_joint"]


class StateCache:
    """Thread-safe cache mirroring the real ARIA state bus."""

    def __init__(self):
        self._lock = threading.Lock()
        self.joint_positions = [0.0] * len(JOINT_NAMES)

        self.task_state = {
            "command": "", "goal": "", "subgoals": [],
            "action_queue": [], "confidence": 0.0, "status": "IDLE",
            "task_id": "", "awaiting_approval": False,
        }
        self._task_cot_seen = 0  # how many TaskState.chain_of_thought entries consumed

        self.vision_state = {
            "detected_objects": [], "tracked_ids": [],
            "active_perception": False, "scene_confidence": 0.0,
        }

        self.health_state = {
            "servo_health": [], "fps_top": 0.0, "fps_wrist": 0.0,
            "inference_ms": 0.0, "calibration_valid": True,
            "alerts": [],
        }

        self.memory_state = {
            "known_objects": [], "spatial_relations": [],
            "recent_tasks": [],
        }

        self.chain_of_thought: deque = deque(maxlen=1000)
        self.top_camera_jpeg: Optional[bytes] = None
        self.wrist_camera_jpeg: Optional[bytes] = None
        self.ros_connected = False

    def get_full_state(self) -> dict:
        with self._lock:
            return {
                "joints": {
                    "names": JOINT_NAMES,
                    "positions_deg": [
                        round(math.degrees(p), 1) for p in self.joint_positions],
                    "positions_rad": [round(p, 4) for p in self.joint_positions],
                },
                "task": dict(self.task_state),
                "vision": dict(self.vision_state),
                "health": dict(self.health_state),
                "memory": dict(self.memory_state),
                "cot": list(self.chain_of_thought)[-100:],
                "ros_connected": self.ros_connected,
                "timestamp": time.time(),
            }

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
        """Convert a real TaskState message into the dashboard's dict shape."""
        with self._lock:
            self.task_state = {
                "command": msg.current_command,
                "goal": msg.current_goal,
                "subgoals": list(msg.subgoals),
                "action_queue": [
                    {"name": a.action_type, "target": a.target_object,
                     "status": a.status, "confidence": a.confidence,
                     "reasoning": a.reasoning}
                    for a in msg.action_queue
                ],
                "confidence": msg.confidence,
                "status": msg.task_status or "IDLE",
                "task_id": msg.task_id,
                "awaiting_approval": msg.awaiting_user_approval,
            }
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
                        "confidence": d.confidence,
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
                ],
                "tracked_ids": list(msg.tracked_ids),
                "active_perception": msg.active_perception_mode,
                "scene_confidence": msg.scene_confidence,
            }

    def set_memory_state(self, msg):
        with self._lock:
            self.memory_state = {
                "known_objects": [
                    {
                        "id": wo.id,
                        "name": wo.name,
                        "class_name": wo.class_name,
                        "display_name": (wo.name or wo.class_name or "object").replace("_", " ").title(),
                        "pos_x": wo.last_known_pose.pose.position.x,
                        "pos_y": wo.last_known_pose.pose.position.y,
                        "pos_z": wo.last_known_pose.pose.position.z,
                        "color": wo.color,
                        "material": wo.material,
                        "lifecycle_state": wo.lifecycle_state,
                    }
                    for wo in msg.known_objects
                ],
                "spatial_relations": [
                    {
                        "subject": r.subject, "relation": r.relation,
                        "object": r.object, "confidence": r.confidence,
                        "distance": r.distance_m,
                    }
                    for r in msg.spatial_relations
                ],
                "recent_tasks": list(msg.recent_tasks),
            }

    def set_health_state(self, msg):
        with self._lock:
            self.health_state = {
                "servo_health": [
                    {
                        "joint_name": s.joint_name,
                        "temperature_estimate_c": s.temperature_estimate_c,
                        "drift_deg": s.drift_deg,
                        "load_estimate_pct": s.load_estimate_pct,
                        "healthy": s.healthy,
                    }
                    for s in msg.servo_health
                ],
                "fps_top": msg.fps_top_camera,
                "fps_wrist": msg.fps_wrist_camera,
                "inference_ms": msg.inference_latency_ms,
                "calibration_valid": msg.calibration_valid,
                "alerts": list(msg.active_alerts),
            }


state = StateCache()


# ═══════════════════════════════════════════════════════════════
# ROS2 Bridge Node (background thread)
# ═══════════════════════════════════════════════════════════════

bridge_node: Optional["DashboardBridge"] = None


class DashboardBridge(Node):
    """
    ROS2 node bridging the real ARIA state bus + services to the dashboard.

    Reads:  /aria/state/task, /aria/state/vision, /aria/state/memory,
            /aria/state/health, /joint_states, /top_camera/image_raw,
            /wrist_camera/image_raw, /aria/dialogue/output
    Calls:  /aria/command, /aria/approve, /aria/reject, /aria/cancel,
            /aria/estop, /aria/release_estop, /aria/set_joint,
            /aria/set_all_joints
    """

    def __init__(self):
        super().__init__('dashboard_bridge')

        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=5)

        # ── State bus subscriptions (must match StateBus QoS) ──
        self.create_subscription(TaskState, '/aria/state/task', self._task_cb, qos_state)
        self.create_subscription(VisionState, '/aria/state/vision', self._vision_cb, qos_state)
        self.create_subscription(MemoryState, '/aria/state/memory', self._memory_cb, qos_state)
        self.create_subscription(HealthState, '/aria/state/health', self._health_cb, qos_state)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, qos_sensor)

        # Camera feeds — top (overhead) + wrist/gripper (eye-in-hand)
        self.create_subscription(Image, '/top_camera/image_raw', self._top_cam_cb, qos_sensor)
        self.create_subscription(Image, '/wrist_camera/image_raw', self._wrist_cam_cb, qos_sensor)

        # Narrated dialogue from DialogueAgent
        self.create_subscription(String, '/aria/dialogue/output', self._dialogue_cb, 10)

        # ── Real service clients (forward REST calls to the actual nodes) ──
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

        self.create_timer(1.0, self._connectivity_check)
        self.get_logger().info("Dashboard bridge started — bridging real ARIA state bus")

    # ── Subscription callbacks ──────────────────────────────
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

    def _top_cam_cb(self, msg):
        try:
            import cv2
            from cv_bridge import CvBridge
            if not hasattr(self, '_cv_bridge'):
                self._cv_bridge = CvBridge()
            cv_img = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if msg.encoding == 'rgb8':
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
            _, jpeg = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            state.top_camera_jpeg = jpeg.tobytes()
        except Exception as e:
            self.get_logger().warn(f"Top cam cb error: {e}")

    def _wrist_cam_cb(self, msg):
        try:
            import cv2
            from cv_bridge import CvBridge
            if not hasattr(self, '_cv_bridge'):
                self._cv_bridge = CvBridge()
            cv_img = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if msg.encoding == 'rgb8':
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
            _, jpeg = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 75])
            state.wrist_camera_jpeg = jpeg.tobytes()
        except Exception as e:
            self.get_logger().warn(f"Wrist cam cb error: {e}")

    def _dialogue_cb(self, msg):
        state.add_cot(f"[DIALOGUE] {msg.data}")

    def _connectivity_check(self):
        """Consider ROS 'connected' once task_manager's command service appears."""
        state.ros_connected = bool(self.cmd_client.service_is_ready() or self.cmd_client.wait_for_service(timeout_sec=0.05))


def ros_spin_thread():
    """Background thread for ROS2 spinning."""
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
        print(f"[Dashboard] ROS bridge error: {e}")
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


def call_service_sync(client, request, timeout: float = 5.0):
    """
    Call a ROS2 service from a non-ROS thread and block for the result.
    Safe to call from FastAPI's event loop thread with MultiThreadedExecutor.
    """
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
    """Async wrapper around call_service_sync for use in FastAPI handlers."""
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, call_service_sync, client, request, timeout)


# ═══════════════════════════════════════════════════════════════
# FastAPI Application
# ═══════════════════════════════════════════════════════════════

app = FastAPI(title="ARIA Control Center", version="4.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Serve static frontend
FRONTEND_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "frontend", "dist")
if os.path.exists(FRONTEND_DIR):
    assets_dir = os.path.join(FRONTEND_DIR, "assets")
    if os.path.exists(assets_dir):
        app.mount("/assets", StaticFiles(directory=assets_dir), name="assets")
    app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")


# ── Pydantic models ────────────────────────────────────────

class CommandRequest(BaseModel):
    command: str

class JointRequest(BaseModel):
    joint_name: str
    angle_deg: float
    speed_deg_per_s: float = 30.0

class AllJointsRequest(BaseModel):
    angles_deg: list
    speed_deg_per_s: float = 30.0


def _ros_unavailable_response():
    return {
        "success": False,
        "message": (
            "ROS2 bridge not connected. Launch the real ARIA system first: "
            "ros2 launch arm_bringup aria_full.launch.py"
        ),
    }


# ── REST Endpoints ─────────────────────────────────────────

@app.get("/")
async def root():
    """Serve dashboard frontend."""
    index_path = os.path.join(FRONTEND_DIR, "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path)
    return HTMLResponse(
        "<h1>ARIA Dashboard</h1><p>Frontend build not found — "
        "run <code>npm run build</code> in arm_dashboard/frontend.</p>")


@app.post("/api/command")
async def post_command(req: CommandRequest):
    """
    Send a natural-language command to the REAL task_manager over
    /aria/command (SendCommand service). This is the actual ARIA
    planning → perception → grasp → IK → execution pipeline — the
    dashboard does not run any of that logic itself.
    """
    command = req.command.strip()
    if not command:
        return {"success": False, "message": "Empty command"}

    if bridge_node is None:
        return _ros_unavailable_response()

    ros_req = SendCommand.Request()
    ros_req.command = command
    resp = await call_service(bridge_node.cmd_client, ros_req, timeout=5.0)
    if resp is None:
        return {"success": False, "message": "task_manager did not respond (timeout)"}

    return {"success": resp.accepted, "task_id": resp.task_id, "message": resp.message}


@app.post("/api/reset")
async def reset_state():
    """Cancel the active task on the real task_manager (/aria/cancel)."""
    if bridge_node is None:
        return _ros_unavailable_response()
    resp = await call_service(bridge_node.cancel_client, Trigger.Request(), timeout=3.0)
    if resp is None:
        return {"success": False, "message": "task_manager did not respond (timeout)"}
    return {"success": resp.success, "message": resp.message}


@app.post("/api/approve")
async def approve():
    """Approve the paused action via the real task_manager (/aria/approve)."""
    if bridge_node is None:
        return _ros_unavailable_response()
    resp = await call_service(bridge_node.approve_client, Trigger.Request(), timeout=3.0)
    if resp is None:
        return {"success": False, "message": "task_manager did not respond (timeout)"}
    return {"success": resp.success, "message": resp.message}


@app.post("/api/reject")
async def reject():
    """Reject the paused action via the real task_manager (/aria/reject)."""
    if bridge_node is None:
        return _ros_unavailable_response()
    resp = await call_service(bridge_node.reject_client, Trigger.Request(), timeout=3.0)
    if resp is None:
        return {"success": False, "message": "task_manager did not respond (timeout)"}
    return {"success": resp.success, "message": resp.message}


@app.post("/api/estop")
async def estop():
    """Trigger the real hardware/sim e-stop (/aria/estop on manual_control_node)."""
    if bridge_node is None:
        return _ros_unavailable_response()
    resp = await call_service(bridge_node.estop_client, Trigger.Request(), timeout=3.0)
    if resp is None:
        return {"success": False, "message": "manual_control_node did not respond (timeout)"}
    return {"success": resp.success, "message": resp.message}


@app.post("/api/release_estop")
async def release_estop():
    """Release the real e-stop (/aria/release_estop)."""
    if bridge_node is None:
        return _ros_unavailable_response()
    resp = await call_service(bridge_node.release_estop_client, Trigger.Request(), timeout=3.0)
    if resp is None:
        return {"success": False, "message": "manual_control_node did not respond (timeout)"}
    return {"success": resp.success, "message": resp.message}


@app.get("/api/metrics")
async def get_metrics():
    """Get evaluation metrics summary (from real health/task state where available)."""
    return {
        "inference_ms": state.health_state.get("inference_ms", 0),
        "fps_top": state.health_state.get("fps_top", 0),
        "fps_wrist": state.health_state.get("fps_wrist", 0),
        "ros_connected": state.ros_connected,
    }


@app.get("/api/logs")
async def get_logs():
    """Get last 200 chain-of-thought entries."""
    return {"logs": list(state.chain_of_thought)[-200:]}


@app.post("/api/joint")
async def set_joint(req: JointRequest):
    """Manual joint control — forwarded to the real /aria/set_joint service."""
    if bridge_node is None:
        return _ros_unavailable_response()
    ros_req = SetJoint.Request()
    ros_req.joint_name = req.joint_name
    ros_req.angle_deg = req.angle_deg
    ros_req.speed_deg_per_s = req.speed_deg_per_s
    resp = await call_service(bridge_node.set_joint_client, ros_req, timeout=5.0)
    if resp is None:
        return {"success": False, "message": "manual_control_node did not respond (timeout)"}
    return {"success": resp.success, "message": resp.message}


@app.post("/api/joints")
async def set_all_joints(req: AllJointsRequest):
    """Set all joint positions — forwarded to the real /aria/set_all_joints service."""
    if bridge_node is None:
        return _ros_unavailable_response()
    ros_req = SetAllJoints.Request()
    ros_req.angles_deg = [float(a) for a in req.angles_deg]
    ros_req.speed_deg_per_s = req.speed_deg_per_s
    resp = await call_service(bridge_node.set_all_joints_client, ros_req, timeout=5.0)
    if resp is None:
        return {"success": False, "message": "manual_control_node did not respond (timeout)"}
    return {"success": resp.success, "expected_duration_s": resp.expected_duration_s}


@app.get("/api/state")
async def get_state():
    """Get full system state (REST fallback for non-WS clients)."""
    return state.get_full_state()


# ── WebSocket: State Stream (10Hz) ────────────────────────

@app.websocket("/ws/state")
async def ws_state(ws: WebSocket):
    """Stream full state at 10Hz over WebSocket."""
    await ws.accept()
    try:
        while True:
            data = state.get_full_state()
            await ws.send_json(data)
            await asyncio.sleep(0.1)  # 10Hz
    except WebSocketDisconnect:
        pass
    except Exception:
        pass


# ── WebSocket: Camera Stream (JPEG over WS) ─────────────

@app.websocket("/ws/cameras")
async def ws_cameras(ws: WebSocket):
    """Stream camera JPEG frames (top + gripper/wrist) over WebSocket."""
    await ws.accept()
    try:
        while True:
            frame_data = {}
            if state.top_camera_jpeg:
                frame_data["top"] = base64.b64encode(state.top_camera_jpeg).decode("ascii")
            if state.wrist_camera_jpeg:
                frame_data["wrist"] = base64.b64encode(state.wrist_camera_jpeg).decode("ascii")
            if frame_data:
                await ws.send_json(frame_data)
            await asyncio.sleep(0.033)  # ~30fps
    except WebSocketDisconnect:
        pass
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════
# Startup
# ═══════════════════════════════════════════════════════════════

@app.on_event("startup")
async def startup():
    """Start ROS2 bridge in background thread."""
    if ROS_AVAILABLE:
        t = threading.Thread(target=ros_spin_thread, daemon=True)
        t.start()
        print("[Dashboard] ROS2 bridge started in background")
        state.add_cot("═══ ARIA Control Center Online — connecting to ROS2 ═══")
        state.add_cot(
            "Waiting for task_manager (/aria/command). Launch the full "
            "system with: ros2 launch arm_bringup aria_full.launch.py")
    else:
        print("[Dashboard] ROS2 not available — running in READ-ONLY demo mode")
        state.add_cot(
            "⚠ ROS2 not available in this Python environment. "
            "Source install/setup.bash and restart the dashboard.")


def main():
    """Run the dashboard server."""
    import uvicorn
    port = int(os.environ.get("ARIA_DASHBOARD_PORT", "8080"))
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")


if __name__ == "__main__":
    main()
