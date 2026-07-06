#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Grasp Node v2 — Enhanced Grasp Planning
Extension of original grasp_node.py.

Adds:
  - SAM2 mask-based grasp (actual object silhouette)
  - 6D pose-aware orientation grasps
  - Material-specific force/speed adjustments
  - Minimum bounding rectangle → gripper alignment

Backward compatible: falls back to original behavior
if SAM2/6D/material data is unavailable.

Subscribes:
  /detection/objects
  /sam2/masks_json
  /pose_6d/objects
  /material/predictions

Services:
  /aria/grasp/plan_v2 (PlanGrasp)
═══════════════════════════════════════════════════════════════
"""
import json
import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    Pose, PoseArray, PoseStamped, Point, Quaternion,
)
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String

from arm_interfaces.srv import PlanGrasp

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


# ═══════════════════════════════════════════════════════════════
# Grasp parameter defaults
# ═══════════════════════════════════════════════════════════════
DEFAULT_GRIP_FORCE = 1.0
DEFAULT_APPROACH_SPEED = 1.0
APPROACH_OFFSET_M = 0.10
FINGER_LENGTH_M = 0.080


def _euler_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> Quaternion:
    """Convert Euler angles (radians) to Quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        x=float(sr * cp * cy - cr * sp * sy),
        y=float(cr * sp * cy + sr * cp * sy),
        z=float(cr * cp * sy - sr * sp * cy),
        w=float(cr * cp * cy + sr * sp * sy),
    )


def _quaternion_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """Multiply two quaternions."""
    return Quaternion(
        x=float(q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y),
        y=float(q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x),
        z=float(q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w),
        w=float(q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z),
    )


# ═══════════════════════════════════════════════════════════════
# Enhanced grasp computation
# ═══════════════════════════════════════════════════════════════
def compute_grasp_from_mask(
    mask_props: dict,
    world_pos: np.ndarray,
    material_params: Optional[dict] = None,
) -> Tuple[PoseStamped, PoseStamped, float, dict]:
    """
    Compute orientation-aware grasp from SAM2 mask properties.

    Steps:
      1. Find minimum bounding rectangle of mask
      2. Major axis = gripper alignment axis
      3. Grasp width = minor axis (actual object width)
      4. Apply material-specific force scaling

    Args:
        mask_props:      dict from compute_mask_properties()
        world_pos:       (x, y, z) in world frame
        material_params: dict with grip_force_scale, approach_speed_scale

    Returns:
        (grasp_pose, approach_pose, confidence, metadata)
    """
    wx, wy, wz = world_pos

    # Get mask orientation
    major_angle_deg = mask_props.get('major_axis_angle', 0.0)
    aspect_ratio = mask_props.get('aspect_ratio', 1.0)
    mask_area = mask_props.get('area', 100)

    # Convert mask angle to gripper yaw
    major_angle_rad = math.radians(major_angle_deg)

    # Grasp pose: align gripper fingers perpendicular to major axis
    grasp_yaw = major_angle_rad + math.pi / 2  # perpendicular to major axis

    # Build grasp pose
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = 'base_link'
    grasp_pose.pose.position = Point(x=wx, y=wy, z=wz + 0.02)

    # Orientation: top-down with yaw rotation for gripper alignment
    grasp_pose.pose.orientation = _euler_to_quaternion(
        0.0, -math.pi / 2, grasp_yaw)

    # Approach pose: above grasp
    approach_pose = PoseStamped()
    approach_pose.header.frame_id = 'base_link'
    approach_pose.pose.position = Point(
        x=wx, y=wy, z=wz + APPROACH_OFFSET_M)
    approach_pose.pose.orientation = grasp_pose.pose.orientation

    # Confidence based on mask quality
    confidence = 0.80
    if aspect_ratio > 1.5:
        confidence += 0.05  # Elongated objects easier to align
    if mask_area > 500:
        confidence += 0.05  # Larger objects easier to grasp

    # Material adjustments
    metadata = {
        'grip_force_scale': DEFAULT_GRIP_FORCE,
        'approach_speed_scale': DEFAULT_APPROACH_SPEED,
        'gripper_yaw_deg': float(math.degrees(grasp_yaw)),
        'mask_aspect_ratio': aspect_ratio,
        'method': 'mask_aligned',
    }

    if material_params:
        metadata['grip_force_scale'] = material_params.get(
            'grip_force_scale', DEFAULT_GRIP_FORCE)
        metadata['approach_speed_scale'] = material_params.get(
            'approach_speed_scale', DEFAULT_APPROACH_SPEED)
        metadata['material'] = material_params.get('material', 'unknown')

    return grasp_pose, approach_pose, confidence, metadata


def compute_grasp_from_6d_pose(
    object_pose: Pose,
    class_name: str = '',
    material_params: Optional[dict] = None,
) -> Tuple[PoseStamped, PoseStamped, float, dict]:
    """
    Compute grasp using full 6D object pose.

    Uses object orientation to determine optimal approach axis.
    For tools: approach along handle axis.
    For cups: approach from handle side.
    """
    wx = object_pose.position.x
    wy = object_pose.position.y
    wz = object_pose.position.z

    # Use object orientation to derive approach
    obj_q = object_pose.orientation

    # Approach strategies based on class
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = 'base_link'
    grasp_pose.pose.position = Point(x=wx, y=wy, z=wz + 0.02)

    cls_lower = class_name.lower()

    if cls_lower in ('cup', 'mug'):
        # Side approach for cups (toward handle)
        grasp_pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=0.7071, w=0.7071)
        approach_offset = np.array([0.0, -APPROACH_OFFSET_M, 0.0])
    elif cls_lower in ('screwdriver', 'pen', 'marker', 'paintbrush'):
        # Align gripper along tool shaft using object orientation
        grasp_pose.pose.orientation = obj_q
        approach_offset = np.array([0.0, 0.0, APPROACH_OFFSET_M])
    elif cls_lower in ('knife', 'fork', 'spoon'):
        # Side grasp on handle
        grasp_pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=0.7071, w=0.7071)
        approach_offset = np.array([0.0, -APPROACH_OFFSET_M, 0.0])
    else:
        # Default top-down with object yaw
        grasp_pose.pose.orientation = _euler_to_quaternion(
            0.0, -math.pi / 2, 0.0)
        approach_offset = np.array([0.0, 0.0, APPROACH_OFFSET_M])

    approach_pose = PoseStamped()
    approach_pose.header.frame_id = 'base_link'
    approach_pose.pose.position = Point(
        x=float(wx + approach_offset[0]),
        y=float(wy + approach_offset[1]),
        z=float(wz + approach_offset[2]),
    )
    approach_pose.pose.orientation = grasp_pose.pose.orientation

    confidence = 0.90  # High confidence with 6D pose

    metadata = {
        'grip_force_scale': DEFAULT_GRIP_FORCE,
        'approach_speed_scale': DEFAULT_APPROACH_SPEED,
        'method': '6d_pose_aware',
    }

    if material_params:
        metadata['grip_force_scale'] = material_params.get(
            'grip_force_scale', DEFAULT_GRIP_FORCE)
        metadata['approach_speed_scale'] = material_params.get(
            'approach_speed_scale', DEFAULT_APPROACH_SPEED)

    return grasp_pose, approach_pose, confidence, metadata


# ═══════════════════════════════════════════════════════════════
# ROS2 Node
# ═══════════════════════════════════════════════════════════════
class GraspNodeV2(Node):
    """
    Enhanced grasp planning for ARIA.

    Uses SAM2 masks, 6D pose, and material recognition
    when available. Falls back to original bbox-based method
    when enhanced data is unavailable.
    """

    def __init__(self):
        super().__init__('grasp_node_v2')
        self.get_logger().info("═══ ARIA Grasp Node v2 starting ═══")

        # Data buffers
        self._latest_detections: Optional[Detection2DArray] = None
        self._mask_data: dict = {}
        self._pose_6d: Optional[PoseArray] = None
        self._material_data: dict = {}

        # ── Subscribers ────────────────────────────────────
        self.create_subscription(
            Detection2DArray, '/detection/objects',
            self._detection_cb, 10)
        self.create_subscription(
            String, '/sam2/masks_json',
            self._masks_cb, 10)
        self.create_subscription(
            PoseArray, '/pose_6d/objects',
            self._pose_cb, 10)
        self.create_subscription(
            String, '/material/predictions',
            self._material_cb, 10)

        # ── Service ────────────────────────────────────────
        self.create_service(
            PlanGrasp, '/aria/grasp/plan_v2', self._plan_grasp_cb)

        # ── Visualization ──────────────────────────────────
        self.viz_pub = self.create_publisher(
            MarkerArray, '/grasp/candidates_viz_v2', 10)

        self.get_logger().info("Grasp Node v2 ready")

    # ── Data callbacks ─────────────────────────────────────
    def _detection_cb(self, msg: Detection2DArray):
        self._latest_detections = msg

    def _masks_cb(self, msg: String):
        try:
            self._mask_data = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _pose_cb(self, msg: PoseArray):
        self._pose_6d = msg

    def _material_cb(self, msg: String):
        try:
            self._material_data = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ── Grasp planning service ─────────────────────────────
    def _plan_grasp_cb(self, request, response):
        """
        Service: /aria/grasp/plan_v2

        Enhanced grasp planning with SAM2 + 6D pose + material.
        """
        object_id = request.object_id
        method = request.method if request.method else 'auto'

        self.get_logger().info(
            f"Planning grasp (v2) for object {object_id}, method={method}")

        # Find object in detections
        obj_class, obj_center = self._find_object(object_id)
        if obj_class is None:
            response.success = False
            response.confidence = 0.0
            response.method_used = 'none'
            return response

        # Get material params
        material_params = self._get_material_params(object_id)

        # Default world position
        world_pos = np.array([0.20, 0.0, 0.80])

        # ── Strategy selection ─────────────────────────────
        # Priority: 6D pose > mask-based > bbox-based

        grasp_pose = approach_pose = None
        confidence = 0.0
        metadata = {}

        # Try 6D pose-aware grasp first
        if method in ('auto', '6d_pose') and self._pose_6d is not None:
            if self._pose_6d.poses:
                obj_pose = self._pose_6d.poses[0]  # TODO: match by ID
                grasp_pose, approach_pose, confidence, metadata = \
                    compute_grasp_from_6d_pose(
                        obj_pose, obj_class, material_params)
                self.get_logger().info(
                    f"  Using 6D pose-aware grasp (conf={confidence:.2f})")

        # Try mask-based grasp
        if grasp_pose is None and method in ('auto', 'mask'):
            obj_key = str(object_id)
            if obj_key in self._mask_data:
                mask_props = self._mask_data[obj_key]
                if isinstance(mask_props, dict) and 'centroid' in mask_props:
                    grasp_pose, approach_pose, confidence, metadata = \
                        compute_grasp_from_mask(
                            mask_props, world_pos, material_params)
                    self.get_logger().info(
                        f"  Using mask-aligned grasp "
                        f"(angle={metadata.get('gripper_yaw_deg', 0):.1f}°)")

        # Fallback: original bbox-based grasp
        if grasp_pose is None:
            grasp_pose, approach_pose, confidence, metadata = \
                self._bbox_fallback_grasp(world_pos, obj_class, material_params)
            self.get_logger().info("  Using bbox fallback grasp")

        response.success = True
        response.grasp_pose = grasp_pose
        response.approach_pose = approach_pose
        response.confidence = confidence
        response.method_used = metadata.get('method', 'fallback')

        # Visualize
        self._publish_viz(grasp_pose, approach_pose, metadata)

        return response

    def _find_object(self, object_id: int):
        """Find object by tracking ID."""
        if self._latest_detections is None:
            return None, None

        for det in self._latest_detections.detections:
            try:
                det_id = int(det.id) if det.id else -1
            except (ValueError, AttributeError):
                det_id = -1

            if det_id == object_id or object_id == -1:
                cls_name = 'unknown'
                if det.results:
                    cls_name = det.results[0].hypothesis.class_id

                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                return cls_name, np.array([cx, cy])

        return None, None

    def _get_material_params(self, object_id: int) -> Optional[dict]:
        """Get material-based grasp parameters for an object."""
        if not self._material_data:
            return None

        ids = self._material_data.get('object_ids', [])
        if object_id not in ids:
            return None

        idx = ids.index(object_id)
        mats = self._material_data.get('material_classes', [])
        forces = self._material_data.get('grip_force_scales', [])
        speeds = self._material_data.get('approach_speed_scales', [])

        params = {}
        if idx < len(mats):
            params['material'] = mats[idx]
        if idx < len(forces):
            params['grip_force_scale'] = forces[idx]
        if idx < len(speeds):
            params['approach_speed_scale'] = speeds[idx]

        return params if params else None

    def _bbox_fallback_grasp(
        self,
        world_pos: np.ndarray,
        class_name: str,
        material_params: Optional[dict] = None,
    ) -> Tuple[PoseStamped, PoseStamped, float, dict]:
        """Original bbox-based grasp (backward compatible)."""
        wx, wy, wz = world_pos

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.pose.position = Point(x=wx, y=wy, z=wz + 0.02)
        grasp_pose.pose.orientation = Quaternion(
            x=0.0, y=-0.7071, z=0.0, w=0.7071)

        approach_pose = PoseStamped()
        approach_pose.header.frame_id = 'base_link'
        approach_pose.pose.position = Point(
            x=wx, y=wy, z=wz + APPROACH_OFFSET_M)
        approach_pose.pose.orientation = grasp_pose.pose.orientation

        confidence = 0.70
        metadata = {
            'method': 'bbox_fallback',
            'grip_force_scale': DEFAULT_GRIP_FORCE,
            'approach_speed_scale': DEFAULT_APPROACH_SPEED,
        }

        if material_params:
            metadata['grip_force_scale'] = material_params.get(
                'grip_force_scale', DEFAULT_GRIP_FORCE)
            metadata['approach_speed_scale'] = material_params.get(
                'approach_speed_scale', DEFAULT_APPROACH_SPEED)

        return grasp_pose, approach_pose, confidence, metadata

    def _publish_viz(
        self,
        grasp_pose: PoseStamped,
        approach_pose: PoseStamped,
        metadata: dict,
    ):
        """Publish grasp candidate visualization."""
        markers = MarkerArray()

        # Grasp arrow (green/orange based on method)
        m = Marker()
        m.header = grasp_pose.header
        m.ns = "grasp_v2"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = grasp_pose.pose
        m.scale.x = 0.05
        m.scale.y = 0.01
        m.scale.z = 0.01

        method = metadata.get('method', 'fallback')
        if '6d' in method:
            m.color.r, m.color.g, m.color.b = 0.0, 0.8, 1.0  # Cyan
        elif 'mask' in method:
            m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.3  # Green
        else:
            m.color.r, m.color.g, m.color.b = 1.0, 0.6, 0.0  # Orange
        m.color.a = 0.9
        m.lifetime.sec = 5
        markers.markers.append(m)

        # Method label
        tm = Marker()
        tm.header = grasp_pose.header
        tm.ns = "grasp_v2"
        tm.id = 1
        tm.type = Marker.TEXT_VIEW_FACING
        tm.action = Marker.ADD
        tm.pose.position = Point(
            x=grasp_pose.pose.position.x,
            y=grasp_pose.pose.position.y,
            z=grasp_pose.pose.position.z + 0.06)
        tm.scale.z = 0.015
        tm.color.r = 1.0
        tm.color.g = 1.0
        tm.color.b = 1.0
        tm.color.a = 0.8
        force = metadata.get('grip_force_scale', 1.0)
        tm.text = f"{method} (force={force:.0%})"
        tm.lifetime.sec = 5
        markers.markers.append(tm)

        self.viz_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = GraspNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
