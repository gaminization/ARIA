#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA State Bus — Central Pub/Sub State System
All agents read/write through this bus. No direct agent calls.
Thread-safe, asyncio-compatible.

Topics:
  /aria/state/vision   VisionState   10Hz
  /aria/state/memory   MemoryState    2Hz
  /aria/state/task     TaskState     10Hz
  /aria/state/health   HealthState    1Hz
  /aria/state/joints   JointState    50Hz (relay from ros2_control)
═══════════════════════════════════════════════════════════════
"""
import threading
from typing import Any, Callable, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

from arm_planner.msg import (
    VisionState, MemoryState, TaskState, HealthState,
    ObjectDetection, WorldObject, FailureEvent, Action,
)


# ═══════════════════════════════════════════════════════════════
# State Snapshot — thread-safe in-memory view
# ═══════════════════════════════════════════════════════════════
class StateSnapshot:
    """
    Thread-safe snapshot of all system state.
    Updated by StateBus subscribers, read by all agents.
    """

    def __init__(self):
        self._lock = threading.RLock()
        self._vision: Optional[VisionState] = None
        self._memory: Optional[MemoryState] = None
        self._task: Optional[TaskState] = None
        self._health: Optional[HealthState] = None
        self._joints: Optional[JointState] = None

    @property
    def vision(self) -> Optional[VisionState]:
        with self._lock:
            return self._vision

    @vision.setter
    def vision(self, val: VisionState):
        with self._lock:
            self._vision = val

    @property
    def memory(self) -> Optional[MemoryState]:
        with self._lock:
            return self._memory

    @memory.setter
    def memory(self, val: MemoryState):
        with self._lock:
            self._memory = val

    @property
    def task(self) -> Optional[TaskState]:
        with self._lock:
            return self._task

    @task.setter
    def task(self, val: TaskState):
        with self._lock:
            self._task = val

    @property
    def health(self) -> Optional[HealthState]:
        with self._lock:
            return self._health

    @health.setter
    def health(self, val: HealthState):
        with self._lock:
            self._health = val

    @property
    def joints(self) -> Optional[JointState]:
        with self._lock:
            return self._joints

    @joints.setter
    def joints(self, val: JointState):
        with self._lock:
            self._joints = val


# ═══════════════════════════════════════════════════════════════
# State Bus — Central publisher/subscriber
# ═══════════════════════════════════════════════════════════════
class StateBus:
    """
    Central state bus for ARIA agent communication.

    Architecture:
      - Every agent gets a StateBus instance
      - StateBus attaches publishers and subscribers to the agent's node
      - All state flows through ROS2 topics
      - In-memory StateSnapshot for fast synchronous reads

    Usage in agents:
      self.bus = StateBus(self)  # self is a LifecycleNode
      # Read state
      vision = self.bus.state.vision
      # Write state
      self.bus.publish_vision(vision_msg)
    """

    # QoS profiles
    _QOS_STATE = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=1,
    )
    _QOS_SENSOR = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        depth=5,
    )

    def __init__(self, node: Node):
        """
        Attach state bus to a ROS2 node.

        Args:
            node: The LifecycleNode that owns this bus
        """
        self._node = node
        self.state = StateSnapshot()
        self._change_callbacks: Dict[str, list] = {
            'vision': [], 'memory': [], 'task': [],
            'health': [], 'joints': [],
        }

        # ── Subscribers (all agents read) ──────────────────
        self._sub_vision = node.create_subscription(
            VisionState, '/aria/state/vision',
            self._on_vision, self._QOS_STATE)
        self._sub_memory = node.create_subscription(
            MemoryState, '/aria/state/memory',
            self._on_memory, self._QOS_STATE)
        self._sub_task = node.create_subscription(
            TaskState, '/aria/state/task',
            self._on_task, self._QOS_STATE)
        self._sub_health = node.create_subscription(
            HealthState, '/aria/state/health',
            self._on_health, self._QOS_STATE)
        self._sub_joints = node.create_subscription(
            JointState, '/joint_states',
            self._on_joints, self._QOS_SENSOR)

        # ── Publishers (agents that produce state) ─────────
        self._pub_vision = node.create_publisher(
            VisionState, '/aria/state/vision', self._QOS_STATE)
        self._pub_memory = node.create_publisher(
            MemoryState, '/aria/state/memory', self._QOS_STATE)
        self._pub_task = node.create_publisher(
            TaskState, '/aria/state/task', self._QOS_STATE)
        self._pub_health = node.create_publisher(
            HealthState, '/aria/state/health', self._QOS_STATE)

    # ── Subscriber callbacks ───────────────────────────────
    def _on_vision(self, msg: VisionState):
        self.state.vision = msg
        for cb in self._change_callbacks['vision']:
            cb(msg)

    def _on_memory(self, msg: MemoryState):
        self.state.memory = msg
        for cb in self._change_callbacks['memory']:
            cb(msg)

    def _on_task(self, msg: TaskState):
        self.state.task = msg
        for cb in self._change_callbacks['task']:
            cb(msg)

    def _on_health(self, msg: HealthState):
        self.state.health = msg
        for cb in self._change_callbacks['health']:
            cb(msg)

    def _on_joints(self, msg: JointState):
        self.state.joints = msg
        for cb in self._change_callbacks['joints']:
            cb(msg)

    # ── Publishers ─────────────────────────────────────────
    def publish_vision(self, msg: VisionState):
        """Publish vision state update."""
        self.state.vision = msg
        self._pub_vision.publish(msg)

    def publish_memory(self, msg: MemoryState):
        """Publish memory state update."""
        self.state.memory = msg
        self._pub_memory.publish(msg)

    def publish_task(self, msg: TaskState):
        """Publish task state update."""
        self.state.task = msg
        self._pub_task.publish(msg)

    def publish_health(self, msg: HealthState):
        """Publish health state update."""
        self.state.health = msg
        self._pub_health.publish(msg)

    # ── Change callbacks ───────────────────────────────────
    def on_change(self, state_name: str, callback: Callable):
        """
        Register a callback for state changes.

        Args:
            state_name: 'vision', 'memory', 'task', 'health', 'joints'
            callback: function(msg) called on change
        """
        if state_name in self._change_callbacks:
            self._change_callbacks[state_name].append(callback)

    # ── Convenience helpers ────────────────────────────────
    def get_object_by_id(self, tracking_id: int) -> Optional[ObjectDetection]:
        """Find a detected object by tracking ID."""
        vision = self.state.vision
        if vision is None:
            return None
        for obj in vision.detected_objects:
            if obj.tracking_id == tracking_id:
                return obj
        return None

    def get_known_object(self, name: str) -> Optional[WorldObject]:
        """Find a known object by name from memory."""
        memory = self.state.memory
        if memory is None:
            return None
        for obj in memory.known_objects:
            if obj.name == name or obj.class_name == name:
                return obj
        return None

    def get_task_status(self) -> str:
        """Get current task status."""
        task = self.state.task
        return task.task_status if task else 'IDLE'

    def add_chain_of_thought(self, entry: str):
        """Append an entry to the chain of thought log."""
        task = self.state.task
        if task is not None:
            task.chain_of_thought.append(entry)
            self.publish_task(task)
        self._node.get_logger().info(f"[ARIA] {entry}")

    def log_failure(self, failure: FailureEvent):
        """Log a failure event to the task state."""
        task = self.state.task
        if task is not None:
            task.failure_log.append(failure)
            self.publish_task(task)
