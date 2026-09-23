#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA YOLO Detection Node
YOLOv8 object detection on GPU (RTX 5060).
Clean input: no noise preprocessing.
═══════════════════════════════════════════════════════════════
"""
import math
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
        self.declare_parameter('half_precision', False)
        # Default: /top_camera/image_raw (workcell overview). Can also be overridden
        # with camera_topic:=/wrist_camera/image_raw for eye-in-hand mode.
        self.declare_parameter('camera_topic', '/top_camera/image_raw')

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
        self.active_tracks: Dict[int, tuple] = {}  # track_id → (cx, cy, class_name, last_seen_time)
        self.next_track_id = 1

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

    def _get_persistent_track_id(self, cx: float, cy: float, class_name: str) -> int:
        """Assign or match persistent 2D track ID based on pixel proximity."""
        now = time.time()
        best_id = None
        min_dist = float('inf')

        # Clean old tracks (> 3.0s unseen)
        stale_ids = [tid for tid, info in self.active_tracks.items() if now - info[3] > 3.0]
        for tid in stale_ids:
            del self.active_tracks[tid]

        # Match with active tracks within 50 pixels
        for tid, (tcx, tcy, tcls, _) in self.active_tracks.items():
            if tcls == class_name or class_name in {'table_object', 'unknown'} or tcls in {'table_object', 'unknown'}:
                dist = math.hypot(cx - tcx, cy - tcy)
                if dist < min_dist and dist < 120.0:
                    min_dist = dist
                    best_id = tid

        if best_id is not None:
            self.active_tracks[best_id] = (cx, cy, class_name, now)
            return best_id
        else:
            new_id = self.next_track_id
            self.next_track_id += 1
            self.active_tracks[new_id] = (cx, cy, class_name, now)
            return new_id

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
        covered_boxes = []

        if results and len(results) > 0:
            result = results[0]

            if result.boxes is not None:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cls_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    class_name = self.model.names.get(cls_id, f"class_{cls_id}").lower()

                    ALLOWED_WORKCELL_CLASSES = {
                        'bottle', 'cup', 'bowl', 'banana', 'apple', 'orange',
                        'bird', 'duck', 'box', 'book', 'vase', 'fork', 'spoon',
                        'mouse', 'cell phone'
                    }
                    if class_name not in ALLOWED_WORKCELL_CLASSES:
                        continue
                    if class_name == 'cup':
                        class_name = 'mug'
                    elif class_name == 'bird':
                        class_name = 'duck'
                    elif class_name == 'apple':
                        class_name = 'orange'

                    # Gripper Self-Detection suppression only if running on wrist camera
                    img_h, img_w = cv_image.shape[:2]
                    if 'wrist' in self.camera_topic:
                        is_gripper_artifact = (
                            class_name in {'scissors', 'knife', 'fork', 'spoon', 'remote', 'toilet', 'toothbrush', 'tie'} or
                            (y2 > 0.82 * img_h and x1 > 0.15 * img_w and x2 < 0.85 * img_w)
                        )
                        if is_gripper_artifact:
                            continue

                    det = Detection2D()
                    det.bbox.center.position.x = float((x1 + x2) / 2)
                    det.bbox.center.position.y = float((y1 + y2) / 2)
                    det.bbox.size_x = float(x2 - x1)
                    det.bbox.size_y = float(y2 - y1)

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = class_name
                    hyp.hypothesis.score = confidence
                    det.results.append(hyp)

                    if box.id is not None:
                        det.id = str(int(box.id[0]))
                    else:
                        det.id = str(self._get_persistent_track_id(float((x1 + x2)/2), float((y1 + y2)/2), class_name))

                    det_array.detections.append(det)
                    covered_boxes.append((x1, y1, x2, y2))

        # ── Full Workcell Segmentation (Duck, Mug, Banana, Bottle, Orange, Plate, Jenga, etc.) ──
        if 'top' in self.camera_topic and cv_image is not None:
            try:
                img_h, img_w = cv_image.shape[:2]
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

                area_scale = (img_w * img_h) / (640.0 * 480.0)
                arm_base_r = int(0.13 * min(img_w, img_h))

                # Optical table mat boundary mask (covers tabletop while avoiding outer mat boundary seams)
                table_mask = np.zeros((img_h, img_w), dtype=np.uint8)
                cv2.rectangle(table_mask, (int(0.255 * img_w), int(0.106 * img_h)),
                                          (int(0.730 * img_w), int(0.875 * img_h)), 255, -1)

                # Mask out robot arm mounting base center (cx ~ img_w/2, cy ~ img_h/2)
                cv2.circle(table_mask, (int(img_w / 2), int(img_h / 2)), int(0.14 * min(img_w, img_h)), 0, -1)

                # Mask out robot arm structure (bright white links extending from base)
                mask_arm_white = cv2.inRange(cv_image, (215, 215, 215), (255, 255, 255))
                kernel_arm = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
                mask_arm_white = cv2.dilate(mask_arm_white, kernel_arm)
                table_mask[mask_arm_white > 0] = 0

                segmented_candidates = []

                # 1. Red items (Red Mug)
                mask_red = ((cv2.inRange(hsv, (0, 80, 60), (10, 255, 255)) |
                             cv2.inRange(hsv, (168, 80, 60), (180, 255, 255))) & table_mask)
                cnts, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts:
                    area = cv2.contourArea(c)
                    if int(150 * area_scale) < area < int(3500 * area_scale):
                        bx, by, bw, bh = cv2.boundingRect(c)
                        if bw > 25 and bh > 25:
                            segmented_candidates.append(('mug', bx, by, bw, bh, 0.98))

                # 2. Wood Jenga items (moderate saturation, rear sector)
                mask_wood = (cv2.inRange(hsv, (13, 35, 60), (28, 125, 255)) & table_mask)
                cnts, _ = cv2.findContours(mask_wood, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts:
                    area = cv2.contourArea(c)
                    if int(150 * area_scale) < area < int(4500 * area_scale):
                        bx, by, bw, bh = cv2.boundingRect(c)
                        if bw < 25 or bh < 25 or by < 0.50 * img_h:
                            continue
                        rect = cv2.minAreaRect(c)
                        dim1, dim2 = rect[1]
                        aspect = max(dim1, dim2) / (min(dim1, dim2) + 1e-3)
                        cls = 'jenga_block' if aspect > 1.8 else 'jenga_tower'
                        segmented_candidates.append((cls, bx, by, bw, bh, 0.96))

                # 3. Bright Yellow items (Duck, Banana in front sector)
                mask_yellow = (cv2.inRange(hsv, (20, 25, 50), (40, 255, 255)) & table_mask)
                cnts, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts:
                    area = cv2.contourArea(c)
                    if int(150 * area_scale) < area < int(4500 * area_scale):
                        bx, by, bw, bh = cv2.boundingRect(c)
                        if bw < 25 or bh < 25 or by > 0.50 * img_h:
                            continue
                        rect = cv2.minAreaRect(c)
                        dim1, dim2 = rect[1]
                        aspect = max(dim1, dim2) / (min(dim1, dim2) + 1e-3)
                        cls = 'banana' if aspect > 1.4 else 'duck'
                        segmented_candidates.append((cls, bx, by, bw, bh, 0.97))

                # 4. Orange items (Fresh Orange Fruit)
                mask_orange = (cv2.inRange(hsv, (8, 110, 80), (20, 255, 255)) & table_mask)
                cnts, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts:
                    area = cv2.contourArea(c)
                    bx, by, bw, bh = cv2.boundingRect(c)
                    aspect = max(bw, bh) / (min(bw, bh) + 1e-3)
                    if int(100 * area_scale) < area < int(1200 * area_scale) and aspect < 1.6 and bw > 25 and bh > 25:
                        segmented_candidates.append(('orange', bx, by, bw, bh, 0.98))

                # 5. Dark items (Beverage Bottle)
                mask_dark = (cv2.inRange(hsv, (0, 0, 0), (180, 255, 55)) & table_mask)
                cnts, _ = cv2.findContours(mask_dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts:
                    area = cv2.contourArea(c)
                    bx, by, bw, bh = cv2.boundingRect(c)
                    aspect = max(bw, bh) / (min(bw, bh) + 1e-3)
                    if aspect < 2.2 and int(120 * area_scale) < area < int(1200 * area_scale):
                        if bx < 0.55 * img_w and by < 0.40 * img_h:
                            segmented_candidates.append(('bottle', bx, by, bw, bh, 0.95))

                # 6. Cyan / Blue items (Travel Mug, Salad Bowl)
                mask_cyan = (cv2.inRange(hsv, (78, 60, 60), (130, 255, 255)) & table_mask)
                cnts, _ = cv2.findContours(mask_cyan, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts:
                    area = cv2.contourArea(c)
                    if int(150 * area_scale) < area < int(4500 * area_scale):
                        bx, by, bw, bh = cv2.boundingRect(c)
                        if bw > 25 and bh > 25:
                            cls = 'mug' if area < int(800 * area_scale) else 'bowl'
                            segmented_candidates.append((cls, bx, by, bw, bh, 0.95))

                # 6. Dishes & Plates
                mask_plate = (cv2.inRange(cv_image, (180, 180, 180), (250, 250, 250)) & table_mask)
                cnts, _ = cv2.findContours(mask_plate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts:
                    area = cv2.contourArea(c)
                    if int(1500 * area_scale) < area < int(6000 * area_scale):
                        bx, by, bw, bh = cv2.boundingRect(c)
                        segmented_candidates.append(('plate', bx, by, bw, bh, 0.91))

                # Add segmented candidates with priority over generic/spurious bounding boxes
                for cls_name, bx, by, bw, bh, conf in segmented_candidates:
                    cx = bx + bw / 2.0
                    cy = by + bh / 2.0

                    # Check if an existing box covers this location
                    overlapping_idx = None
                    for i, (ox1, oy1, ox2, oy2) in enumerate(covered_boxes):
                        if ox1 - 15 < cx < ox2 + 15 and oy1 - 15 < cy < oy2 + 15:
                            overlapping_idx = i
                            break

                    if overlapping_idx is not None:
                        # If existing detection is less specific or different, upgrade it with precise segmented class
                        if overlapping_idx < len(det_array.detections):
                            existing_det = det_array.detections[overlapping_idx]
                            existing_cls = existing_det.results[0].hypothesis.class_id if existing_det.results else ''
                            if existing_cls in {'banana', 'mug', 'jenga_block', 'orange', 'bottle', 'duck', 'bowl'} and existing_cls == cls_name:
                                continue
                            # Upgrade class and bounding box
                            existing_det.bbox.center.position.x = float(cx)
                            existing_det.bbox.center.position.y = float(cy)
                            existing_det.bbox.size_x = float(bw)
                            existing_det.bbox.size_y = float(bh)
                            existing_det.results[0].hypothesis.class_id = cls_name
                            existing_det.results[0].hypothesis.score = conf
                            continue

                    det = Detection2D()
                    det.bbox.center.position.x = float(cx)
                    det.bbox.center.position.y = float(cy)
                    det.bbox.size_x = float(bw)
                    det.bbox.size_y = float(bh)

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = cls_name
                    hyp.hypothesis.score = conf
                    det.results.append(hyp)

                    det.id = str(self._get_persistent_track_id(cx, cy, cls_name))
                    det_array.detections.append(det)
                    covered_boxes.append((cx - bw/2, cy - bh/2, cx + bw/2, cy + bh/2))

            except Exception as e:
                self.get_logger().warn(f"Tabletop segmentation warning: {e}")

        self.det_pub.publish(det_array)

        # Publish annotated image
        if CV2_AVAILABLE and cv_image is not None:
            annotated = cv_image.copy()
            for d in det_array.detections:
                x = int(d.bbox.center.position.x - d.bbox.size_x / 2)
                y = int(d.bbox.center.position.y - d.bbox.size_y / 2)
                w = int(d.bbox.size_x)
                h = int(d.bbox.size_y)
                c_name = d.results[0].hypothesis.class_id if d.results else "obj"
                conf = d.results[0].hypothesis.score if d.results else 0.0
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(annotated, f"{c_name} {conf:.2f}", (x, max(20, y - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
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
