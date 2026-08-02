#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Bag Recorder Node
Automatic ROS2 bag recording with smart topic selection,
circular pre-buffer, and failure-triggered capture.

Recording modes:
  STANDARD: joint states + task state + detections (small)
  FULL:     + camera images at reduced rate (medium)
  INVESTIGATION: all topics, triggered after failure (large)

Publishes:
  /aria/bag/status  (String JSON)

Services:
  /aria/bag/start_recording
  /aria/bag/stop_recording
  /aria/bag/set_mode
═══════════════════════════════════════════════════════════════
"""
import json
import os
import subprocess
import time
import threading
from collections import deque
from dataclasses import dataclass, field, asdict
from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional

import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool


# ═══════════════════════════════════════════════════════════════
# Recording modes and topic sets
# ═══════════════════════════════════════════════════════════════
class RecordingMode(str, Enum):
    STANDARD = 'standard'
    FULL = 'full'
    INVESTIGATION = 'investigation'


STANDARD_TOPICS = [
    '/joint_states',
    '/aria/state/task',
    '/aria/state/health',
    '/detection/objects',
    '/depth/object_positions',
    '/aria/servo_states',
    '/aria/planning/llm_plan',
    '/perception/mode',
    '/perception/unified_scene',
]

FULL_TOPICS = STANDARD_TOPICS + [
    '/top_camera/image_raw',
    '/wrist_camera/image_raw',
    '/detection/image_annotated',
    '/depth/image_colorized',
    '/grasp/candidates_viz',
    '/grasp/candidates_viz_v2',
    '/sam2/masks_json',
    '/material/predictions',
]

# Investigation mode records ALL topics (no filter)


@dataclass
class SessionMetadata:
    session_id: str = ''
    start_time: str = ''
    end_time: str = ''
    mode: str = 'sim'
    recording_mode: str = 'standard'
    bag_filepath: str = ''
    tasks_attempted: int = 0
    tasks_completed: int = 0
    objects_in_scene: List[str] = field(default_factory=list)
    failure_events: List[dict] = field(default_factory=list)
    system_version: str = ''
    duration_s: float = 0.0
    size_bytes: int = 0
    topics_recorded: List[str] = field(default_factory=list)


# ═══════════════════════════════════════════════════════════════
# ROS2 Node
# ═══════════════════════════════════════════════════════════════
class BagRecorderNode(Node):
    """
    Automatic ROS2 bag recording for all ARIA sessions.
    """

    def __init__(self):
        super().__init__('bag_recorder_node')
        self.get_logger().info("═══ ARIA Bag Recorder starting ═══")

        # Configuration
        self.declare_parameter('bag_dir', os.path.expanduser('~/aria_bags'))
        self.declare_parameter('auto_start', True)
        self.declare_parameter('default_mode', 'standard')
        self.declare_parameter('pre_buffer_seconds', 30)
        self.declare_parameter('post_failure_seconds', 60)

        self._bag_dir = self.get_parameter('bag_dir').value
        self._auto_start = self.get_parameter('auto_start').value
        self._pre_buffer_s = self.get_parameter('pre_buffer_seconds').value
        self._post_failure_s = self.get_parameter('post_failure_seconds').value

        os.makedirs(self._bag_dir, exist_ok=True)

        # State
        self._recording = False
        self._mode = RecordingMode(
            self.get_parameter('default_mode').value)
        self._bag_process: Optional[subprocess.Popen] = None
        self._session_meta: Optional[SessionMetadata] = None
        self._start_time: Optional[float] = None
        self._task_count = 0
        self._task_success = 0
        self._failure_events: List[dict] = []
        self._detected_objects: set = set()
        self._lock = threading.Lock()

        # Circular pre-buffer for failure investigation
        self._pre_buffer: deque = deque(maxlen=self._pre_buffer_s * 10)

        # Get git SHA for metadata
        self._git_sha = self._get_git_sha()

        # ── Subscribers ────────────────────────────────────
        self.create_subscription(
            String, '/aria/state/task',
            self._task_cb, 10)
        self.create_subscription(
            String, '/detection/objects',
            self._detection_cb, 10)

        # ── Publishers ─────────────────────────────────────
        self._status_pub = self.create_publisher(
            String, '/aria/bag/status', 10)

        # ── Services ───────────────────────────────────────
        self.create_service(
            Trigger, '/aria/bag/start_recording',
            self._start_recording_cb)
        self.create_service(
            Trigger, '/aria/bag/stop_recording',
            self._stop_recording_cb)
        self.create_service(
            SetBool, '/aria/bag/set_mode',
            self._set_mode_cb)

        # ── Timer ──────────────────────────────────────────
        self.create_timer(2.0, self._tick)

        # Auto-start recording
        if self._auto_start:
            self._start_recording(self._mode)

        self.get_logger().info(
            f"Bag Recorder ready (mode={self._mode.value}, "
            f"dir={self._bag_dir})")

    # ── Recording control ──────────────────────────────────
    def _start_recording(self, mode: RecordingMode) -> bool:
        """Start ros2 bag recording process."""
        with self._lock:
            if self._recording:
                self.get_logger().warn("Already recording")
                return False

            self._mode = mode
            now = datetime.now()
            date_dir = os.path.join(
                self._bag_dir, now.strftime('%Y-%m-%d'))
            os.makedirs(date_dir, exist_ok=True)

            session_id = now.strftime('%H%M%S')
            bag_name = f"session_{session_id}_{mode.value}"
            bag_path = os.path.join(date_dir, bag_name)

            # Build ros2 bag record command
            cmd = [
                'ros2', 'bag', 'record',
                '-o', bag_path,
                '--compression-mode', 'message',
                '--compression-format', 'zstd',
            ]

            # Topic selection based on mode
            if mode == RecordingMode.INVESTIGATION:
                cmd.append('--all')
            elif mode == RecordingMode.FULL:
                cmd.extend(FULL_TOPICS)
            else:
                cmd.extend(STANDARD_TOPICS)

            try:
                self._bag_process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                )
                self._recording = True
                self._start_time = time.time()

                # Initialize session metadata
                topics = (
                    ['--all'] if mode == RecordingMode.INVESTIGATION
                    else (FULL_TOPICS if mode == RecordingMode.FULL
                          else STANDARD_TOPICS))
                self._session_meta = SessionMetadata(
                    session_id=session_id,
                    start_time=now.isoformat(),
                    recording_mode=mode.value,
                    bag_filepath=bag_path,
                    system_version=self._git_sha,
                    topics_recorded=topics,
                )

                self._task_count = 0
                self._task_success = 0
                self._failure_events = []
                self._detected_objects = set()

                self.get_logger().info(
                    f"Recording started: {bag_name} (mode={mode.value})")
                return True

            except Exception as e:
                self.get_logger().error(f"Failed to start recording: {e}")
                return False

    def _stop_recording(self) -> Optional[SessionMetadata]:
        """Stop recording and save metadata."""
        with self._lock:
            if not self._recording or self._bag_process is None:
                return None

            # Terminate ros2 bag process
            self._bag_process.terminate()
            try:
                self._bag_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._bag_process.kill()

            self._recording = False
            duration = time.time() - (self._start_time or time.time())

            # Finalize metadata
            meta = self._session_meta
            if meta:
                meta.end_time = datetime.now().isoformat()
                meta.duration_s = round(duration, 1)
                meta.tasks_attempted = self._task_count
                meta.tasks_completed = self._task_success
                meta.objects_in_scene = list(self._detected_objects)
                meta.failure_events = self._failure_events

                # Get bag size
                bag_path = meta.bag_filepath
                if os.path.isdir(bag_path):
                    total_size = sum(
                        os.path.getsize(os.path.join(dp, f))
                        for dp, dn, fnames in os.walk(bag_path)
                        for f in fnames)
                    meta.size_bytes = total_size

                # Save metadata YAML
                meta_path = bag_path + '_metadata.yaml'
                with open(meta_path, 'w') as f:
                    yaml.dump(asdict(meta), f, default_flow_style=False)

                self.get_logger().info(
                    f"Recording stopped: {meta.bag_filepath} "
                    f"({meta.duration_s}s, {meta.size_bytes / 1024 / 1024:.1f}MB)")

            self._bag_process = None
            self._session_meta = None
            return meta

    # ── Failure-triggered recording ────────────────────────
    def _trigger_failure_recording(self, failure_info: dict):
        """Start a short investigation recording after failure."""
        self.get_logger().warn(
            f"Failure detected — triggering investigation recording "
            f"({self._post_failure_s}s)")

        now = datetime.now()
        date_dir = os.path.join(
            self._bag_dir, now.strftime('%Y-%m-%d'))
        os.makedirs(date_dir, exist_ok=True)

        task_name = failure_info.get('task', 'unknown')
        bag_name = f"failure_{now.strftime('%H%M%S')}_{task_name}"
        bag_path = os.path.join(date_dir, bag_name)

        # Record all topics for investigation
        cmd = [
            'ros2', 'bag', 'record',
            '-o', bag_path,
            '--all',
            '--max-bag-duration', str(self._post_failure_s),
        ]

        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

            # Auto-stop after post_failure_seconds
            def _auto_stop():
                time.sleep(self._post_failure_s)
                proc.terminate()
            threading.Thread(target=_auto_stop, daemon=True).start()

            self._failure_events.append({
                'timestamp': now.isoformat(),
                'task': task_name,
                'bag_path': bag_path,
                **failure_info,
            })

        except Exception as e:
            self.get_logger().error(f"Failure recording failed: {e}")

    # ── Callbacks ──────────────────────────────────────────
    def _task_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            status = data.get('status', '')
            if status == 'STARTED':
                self._task_count += 1
            elif status == 'COMPLETED':
                self._task_success += 1
            elif status == 'FAILED':
                self._trigger_failure_recording({
                    'task': data.get('task_name', 'unknown'),
                    'failure_reason': data.get('failure_reason', ''),
                })
        except (json.JSONDecodeError, AttributeError):
            pass

    def _detection_cb(self, msg: String):
        # Track objects seen during session
        try:
            if hasattr(msg, 'data'):
                data = json.loads(msg.data)
                for obj in data.get('objects', []):
                    cls = obj.get('class_name', '')
                    if cls:
                        self._detected_objects.add(cls)
        except (json.JSONDecodeError, AttributeError, TypeError):
            pass

    # ── Service handlers ───────────────────────────────────
    def _start_recording_cb(self, request, response):
        success = self._start_recording(self._mode)
        response.success = success
        response.message = (
            f"Recording {self._mode.value}" if success
            else "Already recording or failed")
        return response

    def _stop_recording_cb(self, request, response):
        meta = self._stop_recording()
        response.success = meta is not None
        response.message = (
            f"Stopped: {meta.bag_filepath}" if meta
            else "Not recording")
        return response

    def _set_mode_cb(self, request, response):
        """SetBool: True=FULL, False=STANDARD."""
        new_mode = RecordingMode.FULL if request.data else RecordingMode.STANDARD
        was_recording = self._recording

        if was_recording:
            self._stop_recording()

        self._mode = new_mode

        if was_recording:
            self._start_recording(self._mode)

        response.success = True
        response.message = f"Mode: {self._mode.value}"
        return response

    # ── Status tick ────────────────────────────────────────
    def _tick(self):
        duration = 0.0
        size_mb = 0.0

        if self._recording and self._start_time:
            duration = time.time() - self._start_time

        if self._session_meta and self._session_meta.bag_filepath:
            bag_path = self._session_meta.bag_filepath
            if os.path.isdir(bag_path):
                try:
                    size = sum(
                        os.path.getsize(os.path.join(dp, f))
                        for dp, dn, fnames in os.walk(bag_path)
                        for f in fnames)
                    size_mb = size / 1024 / 1024
                except OSError:
                    pass

        status = {
            'recording': self._recording,
            'mode': self._mode.value,
            'duration_s': round(duration, 1),
            'size_mb': round(size_mb, 1),
            'tasks': self._task_count,
            'failures': len(self._failure_events),
        }

        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # ── Utility ────────────────────────────────────────────
    def _get_git_sha(self) -> str:
        try:
            result = subprocess.run(
                ['git', 'rev-parse', '--short', 'HEAD'],
                capture_output=True, text=True, timeout=5)
            return result.stdout.strip() if result.returncode == 0 else ''
        except Exception:
            return ''


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop_recording()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
