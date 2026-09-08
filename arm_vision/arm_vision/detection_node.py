#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA YOLO Detection Node
YOLOv8 object detection on GPU (RTX 5060).
Clean input: no noise preprocessing.
═══════════════════════════════════════════════════════════════
"""
import time
from typing import Dict, List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D
from vision_msgs.msg import (
    Detection2D, Detection2DArray,
    ObjectHypothesisWithPose,
)

try:
    from cv_bridge import CvBridge
    CV_BRIDGE = True
except ImportError:
    CV_BRIDGE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class DetectionNode(Node):
    """
    YOLOv8 object detection.

    Loads yolov8m.pt (medium model) on GPU.
    Runs at 30fps on RTX 5060 with fp16.
    Publishes 2D detections with tracking IDs (ByteTrack).
    """

    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info("═══ ARIA Detection Node starting ═══")

        self.declare_parameter('model', 'yolov8m.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('half_precision', True)
        # ── Gripper-camera-only mode ──────────────────────────────
        # Default: wrist/gripper camera (eye-in-hand). Override with
        # camera_topic:=/top_camera/image_raw for overhead mode.
        self.declare_parameter('camera_topic', '/wrist_camera/image_raw')

        model_name = self.get_parameter('model').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value
        use_half = self.get_parameter('half_precision').value
        self.camera_topic = self.get_parameter('camera_topic').value

        # Load YOLO model
        self.model = None
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(model_name)
                self.get_logger().info(f"Loaded YOLO model: {model_name}")

                # Warm up with dummy inference
                dummy = np.zeros((640, 640, 3), dtype=np.uint8)
                self.model.predict(
                    dummy, verbose=False, device=device,
                    half=use_half, conf=self.conf_threshold
                )
                self.get_logger().info(f"YOLO warmed up on {device}")
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLO: {e}")
                self.model = None
        else:
            self.get_logger().warn("ultralytics not installed")

        self.device = device
        self.use_half = use_half
        self.bridge = CvBridge() if CV_BRIDGE else None

        # Tracking state
        self.track_id_map: Dict[int, int] = {}  # YOLO ID → persistent ID
        self.next_track_id = 0

        # Performance stats
        self.frame_count = 0
        self.total_inference_ms = 0.0
        self.max_inference_ms = 0.0

        # Subscribers — always the gripper/wrist camera (eye-in-hand)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        self.image_sub = self.create_subscription(
            Image, self.camera_topic,
            self._image_cb, qos
        )
        self.get_logger().info(
            f"Detection subscribing to camera: {self.camera_topic}")

        # Publishers
        self.det_pub = self.create_publisher(
            Detection2DArray, '/detection/objects', 10)
        self.annotated_pub = self.create_publisher(
            Image, '/detection/image_annotated', 10)

        self.get_logger().info(
            f"Detection node ready — YOLOv8 tracking on {self.camera_topic}")

    def _image_cb(self, msg: Image):
        """Process incoming camera frame with YOLO."""
        if self.model is None or self.bridge is None:
            return

        try:
            # Convert ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        # Run YOLO inference with tracking
        t_start = time.perf_counter()

        results = self.model.track(
            cv_image,
            persist=True,
            verbose=False,
            device=self.device,
            half=self.use_half,
            conf=self.conf_threshold,
            tracker="bytetrack.yaml",
        )

        inference_ms = (time.perf_counter() - t_start) * 1000

        # Update stats
        self.frame_count += 1
        self.total_inference_ms += inference_ms
        self.max_inference_ms = max(self.max_inference_ms, inference_ms)

        if self.frame_count % 100 == 0:
            avg_ms = self.total_inference_ms / self.frame_count
            self.get_logger().info(
                f"Detection stats ({self.frame_count} frames): "
                f"mean={avg_ms:.1f}ms, max={self.max_inference_ms:.1f}ms"
            )

        # Build Detection2DArray
        det_array = Detection2DArray()
        det_array.header = msg.header

        if results and len(results) > 0:
            result = results[0]

            if result.boxes is not None:
                for box in result.boxes:
                    det = Detection2D()

                    # Bounding box (center + size)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    det.bbox.center.position.x = float((x1 + x2) / 2)
                    det.bbox.center.position.y = float((y1 + y2) / 2)
                    det.bbox.size_x = float(x2 - x1)
                    det.bbox.size_y = float(y2 - y1)

                    # Class and confidence
                    cls_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    class_name = self.model.names.get(cls_id, f"class_{cls_id}")

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = class_name
                    hyp.hypothesis.score = confidence
                    det.results.append(hyp)

                    # Tracking ID
                    if box.id is not None:
                        track_id = int(box.id[0])
                        det.id = str(track_id)

                    det_array.detections.append(det)

        self.det_pub.publish(det_array)

        # Publish annotated image
        if CV2_AVAILABLE and results:
            annotated = results[0].plot()
            try:
                ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                ann_msg.header = msg.header
                self.annotated_pub.publish(ann_msg)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
