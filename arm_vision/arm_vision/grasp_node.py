#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Grasp Planning Node
Geometric grasp generator for known object shapes.
GraspNet integration stub for future use.
═══════════════════════════════════════════════════════════════
"""
import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray

from arm_interfaces.srv import PlanGrasp
try:
    from arm_planner.msg import VisionState
    VISION_STATE_AVAILABLE = True
except ImportError:
    VISION_STATE_AVAILABLE = False


class GraspNode(Node):
    """
    Grasp pose estimation for ARIA.

    Primary: Geometric grasp generator (rule-based for known shapes)
    Future:  GraspNet integration for unknown objects
    """

    # Grasp approach distance above object
    APPROACH_OFFSET_M = 0.10  # 10cm above grasp pose

    # Gripper finger length (for grasp height calculation)
    FINGER_LENGTH_M = 0.080

    def __init__(self):
        super().__init__('grasp_node')
        self.get_logger().info("═══ ARIA Grasp Node starting ═══")

        # Detection state (updated by subscription)
        self.latest_detections = None
        self.latest_vision_state = None  # has real 3D pose_3d per object

        # Subscriber for detections
        self.det_sub = self.create_subscription(
            Detection2DArray, '/detection/objects',
            self._detection_cb, 10
        )

        # Subscriber for VisionState — carries real 3D positions computed
        # by VisionAgent's coordinate transformer (ray-plane intersection
        # with the table plane, fused with monocular depth).
        if VISION_STATE_AVAILABLE:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            qos_state = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
            self.vision_state_sub = self.create_subscription(
                VisionState, '/aria/state/vision',
                self._vision_state_cb, qos_state)

        # Service
        self.create_service(
            PlanGrasp, '/aria/grasp/plan', self._plan_grasp_cb
        )

        # Visualization publisher
        self.viz_pub = self.create_publisher(
            MarkerArray, '/grasp/candidates_viz', 10
        )

        self.get_logger().info("Grasp node ready")

    def _detection_cb(self, msg: Detection2DArray):
        """Store latest detections."""
        self.latest_detections = msg

    def _vision_state_cb(self, msg):
        """Store latest VisionState (has real 3D pose_3d per object)."""
        self.latest_vision_state = msg

    def _plan_grasp_cb(self, request, response):
        """
        Service: /aria/grasp/plan

        Plans a grasp for the specified object.
        """
        object_id = request.object_id
        method = request.method if request.method else 'auto'

        self.get_logger().info(
            f"Planning grasp for object {object_id}, method={method}"
        )

        # Get object info from detections (2D bbox center, for fallback/logging)
        obj_class, obj_center = self._find_object(object_id)

        # Get the REAL 3D world position from VisionState (published by
        # VisionAgent using the coordinate transformer). This is what makes
        # the grasp actually land on the object instead of a placeholder.
        world_pos = self._find_object_world_pos(object_id)

        if obj_class is None and world_pos is None:
            response.success = False
            response.confidence = 0.0
            response.method_used = 'none'
            return response

        # Select grasp strategy
        if method == 'auto':
            method = self._select_strategy(obj_class or 'unknown')

        # Generate grasp pose using the real detected 3D position when
        # available; otherwise fall back to the default workspace position.
        if method == 'top_down':
            grasp_pose, approach_pose, conf = self._top_down_grasp(obj_center, world_pos)
        elif method == 'side':
            grasp_pose, approach_pose, conf = self._side_grasp(obj_center, world_pos)
        else:
            grasp_pose, approach_pose, conf = self._top_down_grasp(obj_center, world_pos)

        if world_pos is None:
            conf *= 0.5  # no verified 3D fix — halve confidence
            self.get_logger().warn(
                f"No 3D pose available for object {object_id} — "
                f"using default workspace position (low confidence)")

        response.success = True
        response.grasp_pose = grasp_pose
        response.approach_pose = approach_pose
        response.confidence = conf
        response.method_used = method

        # Publish visualization
        self._publish_viz(grasp_pose, approach_pose)

        self.get_logger().info(
            f"Grasp planned: method={method}, conf={conf:.2f}"
        )
        return response

    def _find_object_world_pos(self, object_id: int):
        """Look up the real 3D world position from the latest VisionState."""
        if self.latest_vision_state is None:
            return None
        for det in self.latest_vision_state.detected_objects:
            if det.tracking_id == object_id or object_id == -1:
                pos = det.pose_3d.pose.position
                if pos.x != 0.0 or pos.y != 0.0 or pos.z != 0.0:
                    return np.array([pos.x, pos.y, pos.z])
        return None

    def _find_object(self, object_id: int):
        """Find object by tracking ID in latest detections."""
        if self.latest_detections is None:
            return None, None

        for det in self.latest_detections.detections:
            try:
                det_id = int(det.id) if det.id else -1
            except (ValueError, AttributeError):
                det_id = -1

            if det_id == object_id or object_id == -1:
                # Get class name
                cls_name = 'unknown'
                if det.results:
                    cls_name = det.results[0].hypothesis.class_id

                # Get center position (pixels for now)
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y

                return cls_name, np.array([cx, cy])

        return None, None

    def _select_strategy(self, class_name: str) -> str:
        """Select grasp strategy based on object class."""
        strategies = {
            'cube': 'top_down',
            'box': 'top_down',
            'block': 'top_down',
            'sphere': 'top_down',
            'ball': 'top_down',
            'cylinder': 'side',
            'bottle': 'side',
            'cup': 'side',
            'pen': 'side',
        }
        # Default to top-down for unknown objects
        return strategies.get(class_name.lower(), 'top_down')

    def _top_down_grasp(self, object_center: np.ndarray,
                        object_world_pos: Optional[np.ndarray] = None):
        """
        Generate top-down grasp: approach from directly above, descend.

        Grasp pose: end-effector directly above object,
                    gripper pointing down (pitch = -90°)
        Approach:   10cm above grasp pose

        Returns:
            (grasp_pose, approach_pose, confidence)
        """
        # Use provided world position or estimate
        if object_world_pos is not None:
            wx, wy, wz = object_world_pos
        else:
            # Placeholder: use detection center with estimated world coords
            # In production, this would use the coordinate transformer
            wx, wy, wz = 0.20, 0.0, 0.80  # Default workspace position

        # Grasp pose: gripper pointing down, fingers parallel to X
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.header.stamp = self.get_clock().now().to_msg()

        # Position: above object, lowered to finger contact height
        grasp_pose.pose.position = Point(
            x=wx, y=wy,
            z=wz + 0.02  # 2cm above object center
        )

        # Orientation: pointing down (pitch = -90°)
        # Quaternion for -90° pitch: [0, -0.707, 0, 0.707]
        grasp_pose.pose.orientation = Quaternion(
            x=0.0, y=-0.7071, z=0.0, w=0.7071
        )

        # Approach pose: 10cm above grasp
        approach_pose = PoseStamped()
        approach_pose.header = grasp_pose.header
        approach_pose.pose.position = Point(
            x=wx, y=wy,
            z=wz + self.APPROACH_OFFSET_M
        )
        approach_pose.pose.orientation = grasp_pose.pose.orientation

        return grasp_pose, approach_pose, 0.85

    def _side_grasp(self, object_center: np.ndarray,
                    object_world_pos: Optional[np.ndarray] = None,
                    approach_axis: str = 'y'):
        """
        Generate side grasp: approach horizontally.

        For cylindrical objects, approach perpendicular to object axis.
        Gripper horizontal, fingers wrap around object.

        Returns:
            (grasp_pose, approach_pose, confidence)
        """
        if object_world_pos is not None:
            wx, wy, wz = object_world_pos
        else:
            wx, wy, wz = 0.20, 0.0, 0.80

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.header.stamp = self.get_clock().now().to_msg()

        # Position: at object height, approach from side
        grasp_pose.pose.position = Point(
            x=wx, y=wy, z=wz
        )

        # Orientation: gripper horizontal, approaching from +Y
        # Quaternion for 90° yaw: [0, 0, 0.707, 0.707]
        grasp_pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=0.7071, w=0.7071
        )

        # Approach from side (10cm offset in approach direction)
        approach_pose = PoseStamped()
        approach_pose.header = grasp_pose.header
        approach_pose.pose.position = Point(
            x=wx,
            y=wy - self.APPROACH_OFFSET_M,  # approach from -Y
            z=wz
        )
        approach_pose.pose.orientation = grasp_pose.pose.orientation

        return grasp_pose, approach_pose, 0.75

    def _publish_viz(self, grasp_pose: PoseStamped,
                     approach_pose: PoseStamped):
        """Publish grasp candidates as RViz arrows."""
        markers = MarkerArray()

        # Grasp arrow (green)
        m = Marker()
        m.header = grasp_pose.header
        m.ns = "grasp_candidates"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = grasp_pose.pose
        m.scale.x = 0.05   # arrow length
        m.scale.y = 0.01   # arrow width
        m.scale.z = 0.01
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.8
        m.lifetime.sec = 5
        markers.markers.append(m)

        # Approach arrow (yellow)
        m2 = Marker()
        m2.header = approach_pose.header
        m2.ns = "grasp_candidates"
        m2.id = 1
        m2.type = Marker.ARROW
        m2.action = Marker.ADD
        m2.pose = approach_pose.pose
        m2.scale.x = 0.08
        m2.scale.y = 0.008
        m2.scale.z = 0.008
        m2.color.r = 1.0
        m2.color.g = 1.0
        m2.color.b = 0.0
        m2.color.a = 0.6
        m2.lifetime.sec = 5
        markers.markers.append(m2)

        self.viz_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = GraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
