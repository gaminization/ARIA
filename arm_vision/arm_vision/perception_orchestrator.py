#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Perception Orchestrator
Manages all perception modules: decides which to load/unload
based on task type, available VRAM, and scene context.

Module sets by task type:
  SIMPLE_PICK:      YOLO + Depth + SAM2 + Material        (~2.0 GB)
  PRECISION_PICK:   + FoundationPose                      (~4.0 GB)
  TRANSPARENT_PICK: + ClearGrasp                          (~2.6 GB)
  FULL_PRECISION:   + FoundationPose + ClearGrasp         (~4.6 GB)
  SCENE_UNDERSTAND: + GaussianSplatting render + SAM2     (~3.3 GB)

Subscribes:
  /aria/state/task  (current task info)
  /detection/objects
  /pose_6d/objects          (PoseArray — orientations when 6D is active)
  /perception/transparent_flags
  /material/predictions     (arm_interfaces/MaterialPredictions — typed)

Publishes:
  /perception/unified_scene       (arm_interfaces/UnifiedScene — typed)
  /perception/unified_scene_json  (String JSON — mirror for bag recording)
  /perception/mode           (String — current mode)
  /perception/vram_usage     (String JSON)

Also drives on-demand module activation (Bool) on:
  /pose_6d/activate, /transparent/activate, /gaussian/activate
═══════════════════════════════════════════════════════════════
"""
import json
import time
from enum import Enum
from typing import Dict, List, Optional, Set

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point, Quaternion
from std_msgs.msg import String, Bool
from vision_msgs.msg import Detection2DArray

from arm_interfaces.msg import MaterialPredictions, UnifiedScene

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


# ═══════════════════════════════════════════════════════════════
# Perception modes
# ═══════════════════════════════════════════════════════════════
class PerceptionMode(str, Enum):
    SIMPLE_PICK = "simple_pick"
    PRECISION_PICK = "precision_pick"
    TRANSPARENT_PICK = "transparent_pick"
    FULL_PRECISION = "full_precision"
    SCENE_UNDERSTANDING = "scene_understanding"


# Module definitions with VRAM cost
MODULE_INFO = {
    'yolo': {
        'vram_gb': 0.8,
        'always_on': True,
        'topic_activate': None,
        'description': 'YOLOv8m object detection',
    },
    'depth_anything': {
        'vram_gb': 0.5,
        'always_on': True,
        'topic_activate': None,
        'description': 'Depth-Anything v2 monocular depth',
    },
    'sam2': {
        'vram_gb': 0.5,
        'always_on': True,
        'topic_activate': None,
        'description': 'SAM2-tiny segmentation masks',
    },
    'material': {
        'vram_gb': 0.2,
        'always_on': True,
        'topic_activate': None,
        'description': 'MobileNetV3 material classifier',
    },
    'foundation_pose': {
        'vram_gb': 2.0,
        'always_on': False,
        'topic_activate': '/pose_6d/activate',
        'description': 'FoundationPose 6D estimation',
    },
    'cleargrasp': {
        'vram_gb': 0.6,
        'always_on': False,
        'topic_activate': '/transparent/activate',
        'description': 'ClearGrasp depth completion',
    },
    'gaussian_splatting': {
        'vram_gb': 0.3,
        'always_on': False,
        'topic_activate': '/gaussian/activate',
        'description': 'Gaussian Splatting workspace render',
    },
}

# Which modules each mode requires
MODE_MODULES = {
    PerceptionMode.SIMPLE_PICK: {
        'yolo', 'depth_anything', 'sam2', 'material',
    },
    PerceptionMode.PRECISION_PICK: {
        'yolo', 'depth_anything', 'sam2', 'material', 'foundation_pose',
    },
    PerceptionMode.TRANSPARENT_PICK: {
        'yolo', 'depth_anything', 'sam2', 'material', 'cleargrasp',
    },
    PerceptionMode.FULL_PRECISION: {
        'yolo', 'depth_anything', 'sam2', 'material',
        'foundation_pose', 'cleargrasp',
    },
    PerceptionMode.SCENE_UNDERSTANDING: {
        'yolo', 'depth_anything', 'sam2', 'material',
        'gaussian_splatting',
    },
}

# Task type → perception mode mapping
TASK_TO_MODE = {
    'pick': PerceptionMode.SIMPLE_PICK,
    'pick_up': PerceptionMode.SIMPLE_PICK,
    'place': PerceptionMode.SIMPLE_PICK,
    'pick_and_place': PerceptionMode.SIMPLE_PICK,
    'insert': PerceptionMode.PRECISION_PICK,
    'insertion': PerceptionMode.PRECISION_PICK,
    'assembly': PerceptionMode.FULL_PRECISION,
    'tool_use': PerceptionMode.PRECISION_PICK,
    'sort': PerceptionMode.SIMPLE_PICK,
    'organize': PerceptionMode.SCENE_UNDERSTANDING,
    'scan': PerceptionMode.SCENE_UNDERSTANDING,
    'survey': PerceptionMode.SCENE_UNDERSTANDING,
    'inspect': PerceptionMode.SCENE_UNDERSTANDING,
}

# Classes that trigger auto-upgrade from SIMPLE to PRECISION
PRECISION_TRIGGER_CLASSES = {
    'cup', 'mug', 'bottle', 'screwdriver', 'scissors', 'key',
    'wrench', 'knife', 'fork', 'spoon', 'pen', 'marker',
}

TRANSPARENT_TRIGGER_CLASSES = {
    'wine glass', 'glass', 'vase', 'bottle',
}

VRAM_OVERHEAD_GB = 0.8
TOTAL_VRAM_GB = 8.0


# ═══════════════════════════════════════════════════════════════
# ROS2 Node
# ═══════════════════════════════════════════════════════════════
class PerceptionOrchestrator(Node):
    """
    Central perception module manager.

    Monitors task context and detected objects.
    Automatically selects the optimal perception mode.
    Manages VRAM budget by activating/deactivating modules.
    """

    def __init__(self):
        super().__init__('perception_orchestrator')
        self.get_logger().info(
            "═══ ARIA Perception Orchestrator starting ═══")

        # Current state
        self._current_mode = PerceptionMode.SIMPLE_PICK
        self._active_modules: Set[str] = set()
        self._target_modules: Set[str] = set()

        # Data
        self._latest_detections: Optional[Detection2DArray] = None
        self._latest_poses: Optional[PoseArray] = None
        self._transparent_flags: dict = {}
        self._material_data: dict = {}
        self._task_type: str = 'pick'

        # Activation publishers (to toggle on-demand modules)
        self._activation_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        for mod_name, info in MODULE_INFO.items():
            topic = info.get('topic_activate')
            if topic:
                self._activation_pubs[mod_name] = self.create_publisher(
                    Bool, topic, 10)

        # ── Subscribers ────────────────────────────────────
        self.create_subscription(
            Detection2DArray, '/detection/objects',
            self._detection_cb, 10)
        self.create_subscription(
            String, '/perception/transparent_flags',
            self._transparent_cb, 10)
        self.create_subscription(
            MaterialPredictions, '/material/predictions',
            self._material_cb, 10)
        self.create_subscription(
            PoseArray, '/pose_6d/objects',
            self._pose_cb, 10)
        self.create_subscription(
            String, '/aria/state/task',
            self._task_cb, 10)

        # ── Publishers ─────────────────────────────────────
        self.unified_pub = self.create_publisher(
            UnifiedScene, '/perception/unified_scene', 10)
        self.unified_json_pub = self.create_publisher(
            String, '/perception/unified_scene_json', 10)
        self.mode_pub = self.create_publisher(
            String, '/perception/mode', 10)
        self.vram_pub = self.create_publisher(
            String, '/perception/vram_usage', 10)

        # ── Timer ──────────────────────────────────────────
        self.create_timer(0.5, self._tick)  # 2 Hz

        # Initialize with SIMPLE_PICK
        self._apply_mode(PerceptionMode.SIMPLE_PICK)

        self.get_logger().info(
            f"Perception orchestrator ready (mode={self._current_mode.value})")

    # ── Data callbacks ─────────────────────────────────────
    def _detection_cb(self, msg: Detection2DArray):
        self._latest_detections = msg

    def _transparent_cb(self, msg: String):
        try:
            self._transparent_flags = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _pose_cb(self, msg: PoseArray):
        self._latest_poses = msg

    def _material_cb(self, msg: MaterialPredictions):
        self._material_data = {
            'object_ids': [int(i) for i in msg.object_ids],
            'material_classes': list(msg.material_classes),
            'confidences': [float(c) for c in msg.confidences],
        }

    def _task_cb(self, msg: String):
        """Parse task type from task state topic."""
        try:
            data = json.loads(msg.data)
            task_type = data.get('task_type', data.get('type', 'pick'))
            self._task_type = task_type.lower()
        except (json.JSONDecodeError, AttributeError):
            pass

    # ── Main tick ──────────────────────────────────────────
    def _tick(self):
        """Evaluate perception needs and adjust modules."""
        # Determine optimal mode
        new_mode = self._evaluate_mode()

        if new_mode != self._current_mode:
            self.get_logger().info(
                f"Perception mode: {self._current_mode.value} "
                f"→ {new_mode.value}")
            self._apply_mode(new_mode)

        # Publish unified scene data
        self._publish_unified_scene()

        # Publish current mode
        mode_msg = String()
        mode_msg.data = self._current_mode.value
        self.mode_pub.publish(mode_msg)

        # Publish VRAM usage
        self._publish_vram()

    def _evaluate_mode(self) -> PerceptionMode:
        """
        Decide optimal perception mode based on:
          1. Current task type
          2. Detected objects requiring precision
          3. Transparent objects detected
        """
        # Base mode from task type
        base_mode = TASK_TO_MODE.get(
            self._task_type, PerceptionMode.SIMPLE_PICK)

        # Check if any detected objects need precision
        if self._latest_detections:
            for det in self._latest_detections.detections:
                cls_name = 'unknown'
                if det.results:
                    cls_name = det.results[0].hypothesis.class_id.lower()

                # Transparent detected → upgrade
                if cls_name in TRANSPARENT_TRIGGER_CLASSES:
                    if base_mode == PerceptionMode.SIMPLE_PICK:
                        base_mode = PerceptionMode.TRANSPARENT_PICK
                    elif base_mode == PerceptionMode.PRECISION_PICK:
                        base_mode = PerceptionMode.FULL_PRECISION

                # Precision object detected → upgrade
                elif cls_name in PRECISION_TRIGGER_CLASSES:
                    if base_mode == PerceptionMode.SIMPLE_PICK:
                        base_mode = PerceptionMode.PRECISION_PICK

        # Check transparent flags from TransparentObjectNode
        if self._transparent_flags:
            has_transparent = any(
                v.get('is_transparent', False)
                for v in self._transparent_flags.values()
                if isinstance(v, dict)
            )
            if has_transparent:
                if base_mode == PerceptionMode.SIMPLE_PICK:
                    base_mode = PerceptionMode.TRANSPARENT_PICK
                elif base_mode == PerceptionMode.PRECISION_PICK:
                    base_mode = PerceptionMode.FULL_PRECISION

        return base_mode

    def _apply_mode(self, mode: PerceptionMode):
        """Switch to a new perception mode, activating/deactivating modules."""
        self._current_mode = mode
        target_modules = MODE_MODULES[mode]

        # Determine changes
        to_activate = target_modules - self._active_modules
        to_deactivate = self._active_modules - target_modules

        # Deactivate first (free VRAM)
        for mod in to_deactivate:
            if mod in self._activation_pubs:
                msg = Bool()
                msg.data = False
                self._activation_pubs[mod].publish(msg)
                self.get_logger().info(f"  Deactivated: {mod}")

        # Activate
        for mod in to_activate:
            if mod in self._activation_pubs:
                msg = Bool()
                msg.data = True
                self._activation_pubs[mod].publish(msg)
                self.get_logger().info(f"  Activated: {mod}")

        self._active_modules = target_modules.copy()

    def _publish_unified_scene(self):
        """Build and publish unified scene from all active perception data.

        Emits a typed arm_interfaces/UnifiedScene plus a JSON mirror.
        """
        scene = {
            'objects': [],
            'scene_type': 'simple',
            'depth_source': 'depth_anything',
            'active_modules': list(self._active_modules),
            'total_vram_used_gb': self._compute_vram(),
        }

        typed = UnifiedScene()
        typed.header.stamp = self.get_clock().now().to_msg()
        typed.header.frame_id = 'base_link'

        pose_6d_active = 'foundation_pose' in self._active_modules
        typed_poses = self._latest_poses.poses if (
            pose_6d_active and self._latest_poses is not None) else []

        # Populate object data from latest detections
        if self._latest_detections:
            for det_idx, det in enumerate(self._latest_detections.detections):
                cls_name = 'unknown'
                conf = 0.0
                if det.results:
                    cls_name = det.results[0].hypothesis.class_id
                    conf = det.results[0].hypothesis.score

                try:
                    obj_id = int(det.id) if det.id else -1
                except (ValueError, AttributeError):
                    obj_id = -1

                material = 'unknown'
                material_conf = 0.0
                depth_conf = 0.9

                # Add material info if available
                if self._material_data:
                    ids = self._material_data.get('object_ids', [])
                    mats = self._material_data.get('material_classes', [])
                    confs = self._material_data.get('confidences', [])
                    if obj_id in ids:
                        idx = ids.index(obj_id)
                        if idx < len(mats):
                            material = mats[idx]
                        if idx < len(confs):
                            material_conf = confs[idx]

                # Add transparency info
                obj_key = str(obj_id)
                if obj_key in self._transparent_flags:
                    flag = self._transparent_flags[obj_key]
                    if isinstance(flag, dict) and flag.get('is_transparent'):
                        depth_conf = 0.5
                        scene['scene_type'] = 'transparent_present'

                obj_data = {
                    'id': obj_id,
                    'class_name': cls_name,
                    'detection_confidence': float(conf),
                    'pose_6d_available': pose_6d_active,
                    'material': material,
                    'material_confidence': float(material_conf),
                    'depth_confidence': float(depth_conf),
                }
                scene['objects'].append(obj_data)

                # ── typed message accumulation ─────────────
                typed.object_ids.append(int(obj_id))
                typed.class_names.append(cls_name)
                typed.detection_confidences.append(float(conf))
                typed.materials.append(material)
                typed.material_confidences.append(float(material_conf))
                typed.depth_confidences.append(float(depth_conf))

                if det_idx < len(typed_poses):
                    p = typed_poses[det_idx]
                    typed.positions.append(Point(
                        x=p.position.x, y=p.position.y, z=p.position.z))
                    typed.orientations.append(Quaternion(
                        x=p.orientation.x, y=p.orientation.y,
                        z=p.orientation.z, w=p.orientation.w))
                    typed.pose_6d_available.append(True)
                else:
                    typed.positions.append(Point(x=0.0, y=0.0, z=0.0))
                    typed.orientations.append(Quaternion(
                        x=0.0, y=0.0, z=0.0, w=1.0))
                    typed.pose_6d_available.append(False)

        # Set depth source based on active modules
        if 'gaussian_splatting' in self._active_modules:
            scene['depth_source'] = 'hybrid'
        if 'cleargrasp' in self._active_modules:
            scene['depth_source'] = 'hybrid'

        typed.scene_type = scene['scene_type']
        typed.depth_source = scene['depth_source']
        typed.active_modules = list(self._active_modules)
        typed.total_vram_used_gb = float(self._compute_vram())

        self.unified_pub.publish(typed)

        mirror = String()
        mirror.data = json.dumps(scene)
        self.unified_json_pub.publish(mirror)

    def _compute_vram(self) -> float:
        """Compute estimated VRAM usage."""
        total = VRAM_OVERHEAD_GB
        for mod in self._active_modules:
            info = MODULE_INFO.get(mod, {})
            total += info.get('vram_gb', 0)
        return round(total, 1)

    def _publish_vram(self):
        """Publish VRAM breakdown."""
        breakdown = {
            'total_gb': self._compute_vram(),
            'budget_gb': TOTAL_VRAM_GB,
            'modules': {},
        }
        for mod in self._active_modules:
            info = MODULE_INFO.get(mod, {})
            breakdown['modules'][mod] = info.get('vram_gb', 0)
        breakdown['modules']['cuda_overhead'] = VRAM_OVERHEAD_GB

        msg = String()
        msg.data = json.dumps(breakdown)
        self.vram_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
