#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Affordance Agent v2 — Enhanced with SAM2 + Material
Extension of original affordance_agent.py.

Adds:
  - Handle detection via SAM2 mask connected component analysis
  - Material-aware affordance (glass cup vs metal mug)
  - Sub-region grasp point localization

Backward compatible: if SAM2/material data unavailable,
falls back to the original class-based affordance lookup.

Subscribes:
  /sam2/masks_json
  /material/predictions
  /detection/objects

Services:
  /aria/affordance/get_grasp_v2  (GetAffordanceGrasp)
═══════════════════════════════════════════════════════════════
"""
import json
import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String

from arm_interfaces.srv import GetAffordanceGrasp
from arm_planner.state_bus import StateBus

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# ═══════════════════════════════════════════════════════════════
# Enhanced affordance database
# ═══════════════════════════════════════════════════════════════
AFFORDANCE_DB: Dict[str, dict] = {
    'cup': {
        'grasp': 'handle',
        'avoid': ['rim', 'interior'],
        'approach': 'horizontal',
        'success_rate': 0.8,
        'has_handle': True,
        'handle_side': 'right',  # Default, overridden by mask analysis
    },
    'mug': {
        'grasp': 'handle',
        'avoid': ['rim'],
        'approach': 'horizontal',
        'success_rate': 0.8,
        'has_handle': True,
    },
    'bottle': {
        'grasp': 'neck',
        'avoid': ['cap'],
        'approach': 'side',
        'success_rate': 0.85,
        'has_handle': False,
    },
    'screwdriver': {
        'grasp': 'shaft',
        'avoid': ['tip'],
        'approach': 'horizontal',
        'success_rate': 0.7,
        'has_handle': True,
    },
    'scissors': {
        'grasp': 'handle_holes',
        'avoid': ['blade'],
        'approach': 'top_down',
        'success_rate': 0.65,
        'has_handle': True,
    },
    'cube': {
        'grasp': 'top',
        'avoid': [],
        'approach': 'top_down',
        'success_rate': 0.9,
        'has_handle': False,
    },
    'ball': {
        'grasp': 'top',
        'avoid': [],
        'approach': 'top_down',
        'success_rate': 0.7,
        'has_handle': False,
    },
    'pen': {
        'grasp': 'body',
        'avoid': ['tip'],
        'approach': 'side',
        'success_rate': 0.7,
        'has_handle': False,
    },
    'knife': {
        'grasp': 'handle',
        'avoid': ['blade'],
        'approach': 'side',
        'success_rate': 0.6,
        'has_handle': True,
    },
    'fork': {
        'grasp': 'handle',
        'avoid': ['tines'],
        'approach': 'side',
        'success_rate': 0.7,
        'has_handle': True,
    },
    'spoon': {
        'grasp': 'handle',
        'avoid': ['bowl'],
        'approach': 'side',
        'success_rate': 0.75,
        'has_handle': True,
    },
}

# Material → affordance modifiers
MATERIAL_AFFORDANCE_MODIFIERS = {
    'glass': {
        'grip_force_scale': 0.4,
        'approach_speed_scale': 0.5,
        'extra_avoid': ['thin_walls'],
        'notes': 'fragile — gentle grip required',
    },
    'rigid_metal': {
        'grip_force_scale': 1.2,
        'approach_speed_scale': 0.8,
        'extra_avoid': [],
        'notes': 'heavy — firm grip, may be slippery',
    },
    'ceramic': {
        'grip_force_scale': 0.5,
        'approach_speed_scale': 0.6,
        'extra_avoid': [],
        'notes': 'fragile — careful handling',
    },
    'cardboard': {
        'grip_force_scale': 0.5,
        'approach_speed_scale': 0.9,
        'extra_avoid': ['edges'],
        'notes': 'can deform — avoid point pressure',
    },
    'soft_foam': {
        'grip_force_scale': 0.3,
        'approach_speed_scale': 0.9,
        'extra_avoid': [],
        'notes': 'compressible — grip will deform object',
    },
}

APPROACH_QUATERNIONS = {
    'top_down': Quaternion(x=0.0, y=-0.7071, z=0.0, w=0.7071),
    'horizontal': Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    'side': Quaternion(x=0.0, y=0.0, z=0.7071, w=0.7071),
}


# ═══════════════════════════════════════════════════════════════
# Handle detection from mask
# ═══════════════════════════════════════════════════════════════
def locate_handle_in_mask(
    mask_props: dict,
    class_name: str,
) -> Optional[dict]:
    """
    Locate handle sub-region within an object mask.

    For cups/mugs: handle is the protruding connected region
    to the side of the main circular body.

    Method: analyze mask aspect ratio and connected components.
    The handle region is typically:
      - Thinner than the main body
      - Protruding to one side
      - Part of the same connected mask

    Returns:
        {handle_center: (cx, cy), handle_side: str, angle: float}
        or None if no handle detected
    """
    cls = class_name.lower()

    # Only attempt handle detection for known handle classes
    has_handle = AFFORDANCE_DB.get(cls, {}).get('has_handle', False)
    if not has_handle:
        return None

    aspect_ratio = mask_props.get('aspect_ratio', 1.0)
    centroid = mask_props.get('centroid', None)
    major_angle = mask_props.get('major_axis_angle', 0)
    min_rect = mask_props.get('min_rect_size', (50, 50))

    if centroid is None:
        return None

    cx, cy = centroid

    # For cups/mugs with handles: aspect ratio > 1.2 suggests handle
    if cls in ('cup', 'mug') and aspect_ratio > 1.15:
        # Handle is on the side where the mask is wider
        handle_angle = major_angle
        w, h = min_rect

        # Estimate handle position (offset from centroid along major axis)
        offset = w * 0.3  # 30% of width
        handle_cx = cx + offset * math.cos(math.radians(handle_angle))
        handle_cy = cy + offset * math.sin(math.radians(handle_angle))

        handle_side = 'right' if handle_cx > cx else 'left'

        return {
            'handle_center': (int(handle_cx), int(handle_cy)),
            'handle_side': handle_side,
            'handle_angle_deg': float(handle_angle),
            'confidence': min(0.9, 0.5 + (aspect_ratio - 1.0) * 0.5),
        }

    # For tools (screwdriver, knife, etc): handle is the thicker end
    if cls in ('screwdriver', 'knife', 'fork', 'spoon', 'paintbrush'):
        # Handle is typically at one end of an elongated mask
        # Use major axis direction: handle is the end with more mass
        handle_angle = major_angle
        length = max(min_rect)

        # Offset toward handle end (heuristic: 40% from center)
        offset = length * 0.4
        handle_cx = cx - offset * math.cos(math.radians(handle_angle))
        handle_cy = cy - offset * math.sin(math.radians(handle_angle))

        return {
            'handle_center': (int(handle_cx), int(handle_cy)),
            'handle_side': 'end',
            'handle_angle_deg': float(handle_angle),
            'confidence': 0.6 if aspect_ratio > 2.0 else 0.4,
        }

    return None


def material_aware_affordance(
    class_name: str,
    material: str,
    base_affordance: dict,
) -> dict:
    """
    Combine class-based affordance with material properties.

    Examples:
      glass_cup:  handle grasp + gentle force (40%)
      metal_mug:  handle grasp + firm force (120%, heavier)
      paper_cup:  body grasp + gentle force (handles tear)
    """
    result = dict(base_affordance)

    modifier = MATERIAL_AFFORDANCE_MODIFIERS.get(material, {})
    if modifier:
        result['grip_force_scale'] = modifier.get('grip_force_scale', 1.0)
        result['approach_speed_scale'] = modifier.get(
            'approach_speed_scale', 1.0)

        extra_avoid = modifier.get('extra_avoid', [])
        if extra_avoid:
            current_avoid = result.get('avoid', [])
            result['avoid'] = list(set(current_avoid + extra_avoid))

        notes = modifier.get('notes', '')
        if notes:
            result['material_notes'] = notes

    # Special case: paper cup — don't grasp handle (it tears)
    if class_name.lower() in ('cup', 'mug') and material == 'paper':
        result['grasp'] = 'body'
        result['approach'] = 'top_down'
        result['material_notes'] = 'paper cup — handle may tear, use body grasp'

    return result


# ═══════════════════════════════════════════════════════════════
# ROS2 Node
# ═══════════════════════════════════════════════════════════════
class AffordanceAgentV2(LifecycleNode):
    """
    Enhanced affordance-based grasp strategy with SAM2 + material.
    """

    def __init__(self):
        super().__init__('affordance_agent_v2')
        self.bus = StateBus(self)
        self.affordances = dict(AFFORDANCE_DB)
        self.grasp_history: list = []
        self._mask_data: dict = {}
        self._material_data: dict = {}

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("AffordanceAgentV2: CONFIGURING")

        # Load persistent affordance DB
        self.declare_parameter('db_path', '')
        db_path = self.get_parameter('db_path').value
        if not db_path:
            db_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'data', 'affordance_db_v2.yaml')
        self.db_path = db_path
        self._load_db()

        # Service
        self.create_service(
            GetAffordanceGrasp,
            '/aria/affordance/get_grasp_v2',
            self._get_grasp_cb)

        # Subscribers for enhanced data
        self.create_subscription(
            String, '/sam2/masks_json', self._masks_cb, 10)
        self.create_subscription(
            String, '/material/predictions', self._material_cb, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("AffordanceAgentV2: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._save_db()
        return TransitionCallbackReturn.SUCCESS

    # ── Data callbacks ─────────────────────────────────────
    def _masks_cb(self, msg: String):
        try:
            self._mask_data = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _material_cb(self, msg: String):
        try:
            self._material_data = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ── Database persistence ───────────────────────────────
    def _load_db(self):
        if YAML_AVAILABLE and os.path.exists(self.db_path):
            try:
                with open(self.db_path, 'r') as f:
                    loaded = yaml.safe_load(f)
                if loaded:
                    self.affordances.update(loaded)
                    self.get_logger().info(
                        f"Loaded {len(self.affordances)} affordances (v2)")
            except Exception as e:
                self.get_logger().warn(f"Could not load affordance DB: {e}")

    def _save_db(self):
        if YAML_AVAILABLE:
            os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
            with open(self.db_path, 'w') as f:
                yaml.dump(
                    self.affordances, f, default_flow_style=False)
            self.get_logger().info(f"Saved affordance DB to {self.db_path}")

    # ── Bayesian update ────────────────────────────────────
    def update_from_outcome(
        self,
        object_class: str,
        grasp_region: str,
        success: bool,
        material: str = '',
    ):
        """Update affordance success rate from grasp outcome."""
        key = object_class.lower()
        if key not in self.affordances:
            self.affordances[key] = {
                'grasp': grasp_region, 'avoid': [],
                'approach': 'top_down', 'success_rate': 0.5,
                'has_handle': False,
            }
        aff = self.affordances[key]
        alpha = 0.1
        old_rate = aff.get('success_rate', 0.5)
        aff['success_rate'] = old_rate * (1 - alpha) + (
            1.0 if success else 0.0) * alpha

        self.grasp_history.append({
            'class': key, 'region': grasp_region,
            'material': material, 'success': success,
            'new_rate': aff['success_rate'],
        })

        self.bus.add_chain_of_thought(
            f"AFFORDANCE_V2: Updated {key} "
            f"(material={material}): "
            f"{old_rate:.2f} → {aff['success_rate']:.2f}")
        self._save_db()

    # ── Main service handler ───────────────────────────────
    def _get_grasp_cb(self, request, response):
        """Service: /aria/affordance/get_grasp_v2"""
        cls = request.object_class.lower()

        # Get base affordance
        aff = self.affordances.get(
            cls, AFFORDANCE_DB.get(cls, AFFORDANCE_DB.get('cube')))

        # Get material for this object
        material = self._get_object_material(request.object_class)

        # Apply material modifiers
        if material:
            aff = material_aware_affordance(cls, material, aff)

        approach = aff.get('approach', 'top_down')
        region = aff.get('grasp', 'top')
        conf = aff.get('success_rate', 0.5)
        avoid = aff.get('avoid', [])

        # ── Handle detection from SAM2 mask ────────────────
        handle_info = None
        obj_key = self._find_object_key(request.object_class)
        if obj_key and obj_key in self._mask_data:
            mask_props = self._mask_data[obj_key]
            if isinstance(mask_props, dict):
                handle_info = locate_handle_in_mask(mask_props, cls)

        # Build grasp pose
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.pose.position = request.object_pose.pose.position

        if handle_info and handle_info.get('confidence', 0) > 0.4:
            # Use handle-based approach
            region = 'handle'
            approach = 'horizontal'
            conf = min(conf + 0.1, 0.95)  # Boost confidence with handle

        grasp_pose.pose.orientation = APPROACH_QUATERNIONS.get(
            approach, APPROACH_QUATERNIONS['top_down'])

        # Adjust Z for approach
        if approach in ('top_down', 'top'):
            grasp_pose.pose.position.z += 0.02

        # Build reasoning string
        reasoning_parts = [
            f"class='{cls}', region='{region}', approach='{approach}'",
            f"avoid={avoid}, success_rate={conf:.0%}",
        ]
        if material:
            force = aff.get('grip_force_scale', 1.0)
            speed = aff.get('approach_speed_scale', 1.0)
            reasoning_parts.append(
                f"material='{material}', force={force:.0%}, speed={speed:.0%}")
        if handle_info:
            reasoning_parts.append(
                f"handle detected ({handle_info['handle_side']}, "
                f"conf={handle_info['confidence']:.0%})")
        if aff.get('material_notes'):
            reasoning_parts.append(f"note: {aff['material_notes']}")

        reasoning = " | ".join(reasoning_parts)
        self.bus.add_chain_of_thought(f"AFFORDANCE_V2: {reasoning}")

        response.success = True
        response.grasp_pose = grasp_pose
        response.confidence = float(conf)
        response.grasp_region = region
        response.approach_direction = approach
        response.reasoning = reasoning
        return response

    def _get_object_material(self, class_name: str) -> str:
        """Get material for an object from MaterialRecognitionNode data."""
        if not self._material_data:
            return ''

        # Try to match by class name
        ids = self._material_data.get('object_ids', [])
        mats = self._material_data.get('material_classes', [])

        # Return first material found (simplified matching)
        if mats:
            return mats[0]
        return ''

    def _find_object_key(self, class_name: str) -> Optional[str]:
        """Find mask data key for object class."""
        for key, data in self._mask_data.items():
            if isinstance(data, dict):
                if data.get('class_name', '').lower() == class_name.lower():
                    return key
        # Return first key as fallback
        if self._mask_data:
            return next(iter(self._mask_data))
        return None


def main(args=None):
    rclpy.init(args=args)
    node = AffordanceAgentV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
