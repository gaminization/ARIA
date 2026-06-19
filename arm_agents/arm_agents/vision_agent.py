#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Vision Agent — LifecycleNode
Manages cameras. Triggers active perception when needed.
Publishes VisionState to state bus.
═══════════════════════════════════════════════════════════════
"""
import time
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from arm_planner.msg import VisionState, ObjectDetection
from arm_planner.state_bus import StateBus
from arm_vision.arm_vision.coordinate_transformer import create_top_camera_transformer

ACTIVE_PERCEPTION_CONFIDENCE_THRESHOLD = 0.6

class VisionAgent(LifecycleNode):
    """
    Manages camera pipeline and publishes VisionState.
    Triggers active perception when:
      - Object confidence < 0.6
      - Bounding box near image edge (>90% of width/height)
    Active perception: move wrist camera to 3 viewpoints, fuse.
    """

    def __init__(self):
        super().__init__('vision_agent')
        self.bus = StateBus(self)
        self.transformer = create_top_camera_transformer()
        self.detections: list = []
        self.tracked_ids: list = []
        self.active_perception = False
        self.search_mode = False
        self.scene_confidence = 0.0
        self.frame_count = 0
        self.top_fps = 0.0
        self.wrist_fps = 0.0
        self._top_last_time = time.time()
        self._wrist_last_time = time.time()
        self._top_frame_count = 0
        self._wrist_frame_count = 0

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("VisionAgent: CONFIGURING")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=5)
        self.det_sub = self.create_subscription(
            Detection2DArray, '/detection/objects', self._det_cb, 10)
        self.top_sub = self.create_subscription(
            Image, '/top_camera/image_raw', self._top_cb, qos)
        self.wrist_sub = self.create_subscription(
            Image, '/wrist_camera/image_raw', self._wrist_cb, qos)
        self.pub_timer = self.create_timer(0.1, self._publish_state)  # 10Hz
        self.fps_timer = self.create_timer(1.0, self._compute_fps)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("VisionAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("VisionAgent: DEACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def _top_cb(self, msg: Image):
        self._top_frame_count += 1

    def _wrist_cb(self, msg: Image):
        self._wrist_frame_count += 1

    def _compute_fps(self):
        now = time.time()
        dt_top = now - self._top_last_time
        dt_wrist = now - self._wrist_last_time
        self.top_fps = self._top_frame_count / max(dt_top, 0.01)
        self.wrist_fps = self._wrist_frame_count / max(dt_wrist, 0.01)
        self._top_frame_count = 0
        self._wrist_frame_count = 0
        self._top_last_time = now
        self._wrist_last_time = now

    def _det_cb(self, msg: Detection2DArray):
        """Process detections from YOLO, build ObjectDetection list."""
        self.detections.clear()
        self.tracked_ids.clear()
        low_confidence_count = 0

        for det in msg.detections:
            obj = ObjectDetection()
            # Tracking ID
            try:
                obj.tracking_id = int(det.id) if det.id else -1
            except (ValueError, AttributeError):
                obj.tracking_id = -1
            if obj.tracking_id >= 0:
                self.tracked_ids.append(obj.tracking_id)

            # Class and confidence
            if det.results:
                obj.class_name = det.results[0].hypothesis.class_id
                obj.confidence = det.results[0].hypothesis.score
            else:
                obj.class_name = 'unknown'
                obj.confidence = 0.0

            # Bounding box
            obj.bbox_x = float(det.bbox.center.position.x)
            obj.bbox_y = float(det.bbox.center.position.y)
            obj.bbox_w = float(det.bbox.size_x)
            obj.bbox_h = float(det.bbox.size_y)

            # 3D Coordinates
            coord = self.transformer.pixel_to_world(int(obj.bbox_x), int(obj.bbox_y))
            if coord.confidence > 0:
                obj.pose_3d.header = msg.header
                obj.pose_3d.header.frame_id = 'world'
                obj.pose_3d.pose.position.x = coord.x
                obj.pose_3d.pose.position.y = coord.y
                obj.pose_3d.pose.position.z = coord.z


            # Lifecycle state
            obj.lifecycle_state = 'Detected'
            if obj.tracking_id >= 0:
                obj.lifecycle_state = 'Tracked'

            # Check for low confidence
            if obj.confidence < ACTIVE_PERCEPTION_CONFIDENCE_THRESHOLD:
                low_confidence_count += 1

            # Check if near image edge (trigger active perception)
            img_w, img_h = 1280.0, 720.0
            edge_margin = 0.1  # 10% of image
            near_edge = (
                obj.bbox_x - obj.bbox_w / 2 < img_w * edge_margin or
                obj.bbox_x + obj.bbox_w / 2 > img_w * (1 - edge_margin) or
                obj.bbox_y - obj.bbox_h / 2 < img_h * edge_margin or
                obj.bbox_y + obj.bbox_h / 2 > img_h * (1 - edge_margin)
            )
            if near_edge:
                low_confidence_count += 1

            self.detections.append(obj)

        # Scene confidence = mean of all detection confidences
        if self.detections:
            self.scene_confidence = float(np.mean(
                [d.confidence for d in self.detections]))
        else:
            self.scene_confidence = 0.0

        # Active perception decision
        self.active_perception = low_confidence_count > 0

        if self.active_perception:
            self.bus.add_chain_of_thought(
                f"VISION: Active perception triggered — "
                f"{low_confidence_count} objects need better view"
            )

    def _publish_state(self):
        """Publish VisionState at 10Hz."""
        msg = VisionState()
        msg.detected_objects = list(self.detections)
        msg.tracked_ids = list(self.tracked_ids)
        msg.active_perception_mode = self.active_perception
        msg.search_mode = self.search_mode
        msg.scene_confidence = self.scene_confidence
        self.bus.publish_vision(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
