#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA YOLO Detection Node
YOLOv8 object detection on GPU (RTX 5060).
Clean input: no noise preprocessing.
═══════════════════════════════════════════════════════════════
"""
import os
import sys
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
        self.declare_parameter('confidence_threshold', 0.10)
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

        # Resolve model path from models/ directory if needed
        if not os.path.exists(model_name):
            candidates = [
                os.path.join(os.getcwd(), 'models', os.path.basename(model_name)),
                f"/home/gaminizer/Projects/ARIA/arm_vision/models/{os.path.basename(model_name)}",
                f"/home/gaminizer/Projects/ARIA/models/{os.path.basename(model_name)}",
            ]
            for cand in candidates:
                if os.path.exists(cand):
                    model_name = cand
                    break

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

    def _extract_dominant_color(self, cv_img: np.ndarray, x1: float, y1: float, x2: float, y2: float) -> str:
        """Extract dominant color name from bounding box ROI without hardcoded coordinates."""
        if cv_img is None:
            return ""
        h, w = cv_img.shape[:2]
        ix1, iy1 = max(0, int(x1)), max(0, int(y1))
        ix2, iy2 = min(w, int(x2)), min(h, int(y2))
        if ix2 <= ix1 or iy2 <= iy1:
            return ""

        roi = cv_img[iy1:iy2, ix1:ix2]
        if roi.size == 0:
            return ""

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        total_px = float(roi.shape[0] * roi.shape[1])

        # Color masks (physically verified for Gazebo workcell)
        mask_red = cv2.inRange(hsv, (0, 60, 50), (8, 255, 255)) | cv2.inRange(hsv, (168, 60, 50), (180, 255, 255))
        mask_orange = cv2.inRange(hsv, (9, 170, 70), (21, 255, 255))
        mask_wood = cv2.inRange(hsv, (10, 25, 40), (24, 165, 255))
        mask_yellow = cv2.inRange(hsv, (23, 130, 70), (38, 255, 255))
        mask_blue = cv2.inRange(hsv, (86, 60, 40), (135, 255, 255))
        mask_white = cv2.inRange(hsv, (0, 0, 180), (180, 40, 255))
        mask_dark = cv2.inRange(hsv, (0, 0, 0), (180, 255, 55))

        counts = {
            'red': cv2.countNonZero(mask_red),
            'orange': cv2.countNonZero(mask_orange),
            'yellow': cv2.countNonZero(mask_yellow),
            'wood': cv2.countNonZero(mask_wood),
            'blue': cv2.countNonZero(mask_blue),
            'white': cv2.countNonZero(mask_white),
            'dark': cv2.countNonZero(mask_dark),
        }

        best_color, max_count = max(counts.items(), key=lambda item: item[1])
        if max_count / total_px > 0.12:
            return best_color
        return ""

    def _get_persistent_track_id(self, cx: float, cy: float, class_name: str) -> int:
        """Assign or match persistent 2D track ID based on pixel proximity."""
        base_cls = class_name.split('|')[0]
        now = time.time()
        best_id = None
        min_dist = float('inf')

        # Clean old tracks (> 3.0s unseen)
        stale_ids = [tid for tid, info in self.active_tracks.items() if now - info[3] > 3.0]
        for tid in stale_ids:
            del self.active_tracks[tid]

        # Match with active tracks within 120 pixels
        for tid, (tcx, tcy, tcls, _) in self.active_tracks.items():
            t_base = tcls.split('|')[0]
            if t_base == base_cls or base_cls in {'table_object', 'unknown'} or t_base in {'table_object', 'unknown'}:
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
        """Process incoming camera frame with YOLO and physical color extraction."""
        if self.model is None or self.bridge is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        t_start = time.perf_counter()

        results = self.model.predict(
            cv_image,
            verbose=False,
            device=self.device,
            conf=self.conf_threshold,
        )

        inference_ms = (time.perf_counter() - t_start) * 1000

        self.frame_count += 1
        self.total_inference_ms += inference_ms
        self.max_inference_ms = max(self.max_inference_ms, inference_ms)

        det_array = Detection2DArray()
        det_array.header = msg.header
        covered_boxes = []

        if results and len(results) > 0:
            result = results[0]

            if result.boxes is not None:
                # 1. Collect all candidate detections
                raw_boxes = []
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cls_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    class_name = self.model.names.get(cls_id, f"class_{cls_id}").lower()

                    ALLOWED_WORKCELL_CLASSES = {
                        'bottle', 'cup', 'bowl', 'banana', 'apple', 'orange',
                        'bird', 'duck', 'box', 'book', 'vase', 'fork', 'spoon',
                        'mouse', 'cell phone', 'plate', 'frisbee', 'clock', 'donut',
                        'sports ball', 'baseball'
                    }
                    if class_name not in ALLOWED_WORKCELL_CLASSES:
                        continue

                    # Extract true physical color for this detection FIRST
                    color = self._extract_dominant_color(cv_image, x1, y1, x2, y2)

                    # Dynamic class mapping based on true geometry and physical color
                    if color == 'wood':
                        aspect = max(x2 - x1, y2 - y1) / (min(x2 - x1, y2 - y1) + 1e-3)
                        class_name = 'jenga_block' if aspect > 1.65 else 'jenga_tower'
                    elif class_name in {'sports ball', 'baseball'}:
                        if color == 'orange':
                            class_name = 'orange'
                        else:
                            class_name = 'mug'
                    elif class_name == 'frisbee':
                        box_diag = math.hypot(x2 - x1, y2 - y1)
                        if color == 'white' and box_diag > 200:
                            class_name = 'plate'
                        else:
                            class_name = 'mug'
                    elif class_name == 'clock':
                        class_name = 'box'
                    elif class_name == 'donut':
                        class_name = 'mug'
                    elif class_name == 'bird':
                        class_name = 'duck'
                    elif class_name == 'apple':
                        class_name = 'banana' if color == 'yellow' else 'orange'
                    elif class_name == 'banana' and color == 'orange':
                        class_name = 'orange'

                    img_h, img_w = cv_image.shape[:2]
                    if 'wrist' in self.camera_topic:
                        is_gripper_artifact = (
                            class_name in {'scissors', 'knife', 'fork', 'spoon', 'remote', 'toilet', 'toothbrush', 'tie'} or
                            (y2 > 0.82 * img_h and x1 > 0.15 * img_w and x2 < 0.85 * img_w)
                        )
                        if is_gripper_artifact:
                            continue

                    raw_boxes.append((x1, y1, x2, y2, confidence, class_name, color, box))

                # 2. Sort by confidence descending and apply class-agnostic NMS
                raw_boxes.sort(key=lambda b: b[4], reverse=True)
                suppressed_boxes = []
                for b in raw_boxes:
                    x1, y1, x2, y2, confidence, class_name, color, box = b
                    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                    is_dup = False
                    for sb in suppressed_boxes:
                        sx1, sy1, sx2, sy2 = sb[:4]
                        scx, scy = (sx1 + sx2) / 2.0, (sy1 + sy2) / 2.0
                        ix1, iy1 = max(x1, sx1), max(y1, sy1)
                        ix2, iy2 = min(x2, sx2), min(y2, sy2)
                        inter_w = max(0.0, ix2 - ix1)
                        inter_h = max(0.0, iy2 - iy1)
                        inter_area = inter_w * inter_h
                        area1 = (x2 - x1) * (y2 - y1)
                        area2 = (sx2 - sx1) * (sy2 - sy1)
                        iou = inter_area / (area1 + area2 - inter_area + 1e-6)
                        dist = math.hypot(cx - scx, cy - scy)
                        if iou > 0.35 or dist < 35.0:
                            is_dup = True
                            break
                    if not is_dup:
                        suppressed_boxes.append(b)

                for b in suppressed_boxes:
                    x1, y1, x2, y2, confidence, class_name, color, box = b
                    full_class_id = f"{class_name}|{color}" if color else class_name

                    det = Detection2D()
                    det.bbox.center.position.x = float((x1 + x2) / 2)
                    det.bbox.center.position.y = float((y1 + y2) / 2)
                    det.bbox.size_x = float(x2 - x1)
                    det.bbox.size_y = float(y2 - y1)

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = full_class_id
                    hyp.hypothesis.score = confidence
                    det.results.append(hyp)

                    if box.id is not None:
                        det.id = str(int(box.id[0]))
                    else:
                        det.id = str(self._get_persistent_track_id(float((x1 + x2)/2), float((y1 + y2)/2), full_class_id))

                    det_array.detections.append(det)
                    covered_boxes.append((x1, y1, x2, y2))

        # ── Generic Tabletop Segmentation for Non-COCO Objects (e.g. Jenga Blocks & Tower) ──
        if 'top' in self.camera_topic and cv_image is not None:
            try:
                img_h, img_w = cv_image.shape[:2]
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                area_scale = (img_w * img_h) / (640.0 * 480.0)

                is_industrial = os.environ.get("ARIA_WORKCELL", "").lower() == "industrial"
                # Optical table workspace mask
                table_mask = np.zeros((img_h, img_w), dtype=np.uint8)
                if is_industrial:
                    # In industrial workcell: conveyor line extends across FOV, table and bins span the scene.
                    # Arm base is at bottom center (near y=580), not in the middle.
                    # Use generous active workspace mask with edge margin.
                    cv2.rectangle(table_mask, (int(0.02 * img_w), int(0.02 * img_h)),
                                              (int(0.98 * img_w), int(0.98 * img_h)), 255, -1)
                else:
                    cv2.rectangle(table_mask, (int(0.24 * img_w), int(0.09 * img_h)),
                                              (int(0.76 * img_w), int(0.91 * img_h)), 255, -1)
                    # Mask out robot arm mounting base center (only in center-mounted desktop stage)
                    cv2.circle(table_mask, (int(img_w / 2), int(img_h / 2)), int(0.14 * min(img_w, img_h)), 0, -1)

                # Mask out robot arm structure
                mask_arm_white = cv2.inRange(cv_image, (215, 215, 215), (255, 255, 255))
                kernel_arm = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
                mask_arm_white = cv2.dilate(mask_arm_white, kernel_arm)
                table_mask[mask_arm_white > 0] = 0

                segmented_candidates = []

                # Wood Jenga items (natural pine/wood color, any position on table)
                mask_wood = (cv2.inRange(hsv, (10, 25, 40), (24, 165, 255)) & table_mask)
                k_wood = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
                mask_wood_closed = cv2.morphologyEx(mask_wood, cv2.MORPH_CLOSE, k_wood)
                cnts, _ = cv2.findContours(mask_wood_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts:
                    area = cv2.contourArea(c)
                    if int(200 * area_scale) < area < int(7000 * area_scale):
                        bx, by, bw, bh = cv2.boundingRect(c)
                        if bw < 14 or bh < 14:
                            continue
                        cx = bx + bw / 2.0
                        # Central station (|cx - center| < 55px) is the Jenga Tower; side positions are loose blocks
                        cls = 'jenga_tower' if abs(cx - img_w / 2.0) < 55.0 else 'jenga_block'
                        segmented_candidates.append((cls, 'wood', bx, by, bw, bh, 0.96))

                # Blue items (conveyor workpieces and tabletop objects)
                mask_blue = (cv2.inRange(hsv, (86, 60, 40), (135, 255, 255)) & table_mask)
                cnts_b, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts_b:
                    area = cv2.contourArea(c)
                    bx, by, bw, bh = cv2.boundingRect(c)
                    aspect = max(bw, bh) / max(min(bw, bh), 1)
                    min_dim = min(bw, bh)
                    min_area = int(220 * area_scale) if is_industrial else 350
                    if min_area <= area <= 4000 and aspect < 2.2 and min_dim >= 20:
                        segmented_candidates.append(('workpiece', 'blue', bx, by, bw, bh, 0.95))
                    elif 4000 <= area < 7500 and not is_industrial:
                        segmented_candidates.append(('mug', 'blue', bx, by, bw, bh, 0.92))

                # Red box / reject bin container
                mask_red = ((cv2.inRange(hsv, (0, 70, 50), (10, 255, 255)) |
                             cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))) & table_mask)
                cnts_r, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts_r:
                    area = cv2.contourArea(c)
                    if area > 4000:
                        bx, by, bw, bh = cv2.boundingRect(c)
                        segmented_candidates.append(('box', 'red', bx, by, bw, bh, 0.95))

                # Merge segmented candidates
                for cls_name, color, bx, by, bw, bh, conf in segmented_candidates:
                    cx = bx + bw / 2.0
                    cy = by + bh / 2.0
                    full_cls = f"{cls_name}|{color}"

                    # Check if an existing box covers this location
                    overlapping_idx = None
                    for i, (ox1, oy1, ox2, oy2) in enumerate(covered_boxes):
                        if ox1 - 15 < cx < ox2 + 15 and oy1 - 15 < cy < oy2 + 15:
                            overlapping_idx = i
                            break

                    if overlapping_idx is not None:
                        # Override confused YOLO classifications (e.g., banana on wood or plate on red mug)
                        if overlapping_idx < len(det_array.detections):
                            exist_hyp = det_array.detections[overlapping_idx].results[0].hypothesis
                            if 'wood' in color and 'banana' in exist_hyp.class_id:
                                exist_hyp.class_id = full_cls
                            elif 'mug' in cls_name and any(k in exist_hyp.class_id for k in ('plate', 'box', 'sports ball', 'unknown')):
                                exist_hyp.class_id = full_cls
                        continue

                    det = Detection2D()
                    det.bbox.center.position.x = float(cx)
                    det.bbox.center.position.y = float(cy)
                    det.bbox.size_x = float(bw)
                    det.bbox.size_y = float(bh)

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = full_cls
                    hyp.hypothesis.score = conf
                    det.results.append(hyp)

                    det.id = str(self._get_persistent_track_id(cx, cy, full_cls))
                    det_array.detections.append(det)
                    covered_boxes.append((cx - bw/2, cy - bh/2, cx + bw/2, cy + bh/2))

                # Dishes & Plates
                mask_plate = (cv2.inRange(cv_image, (180, 180, 180), (250, 250, 250)) & table_mask)
                cnts_p, _ = cv2.findContours(mask_plate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts_p:
                    area = cv2.contourArea(c)
                    if int(1500 * area_scale) < area < int(6500 * area_scale):
                        bx, by, bw, bh = cv2.boundingRect(c)
                        cx = bx + bw / 2.0
                        cy = by + bh / 2.0
                        # Check overlap with existing YOLO detections
                        if not any(ox1 - 15 < cx < ox2 + 15 and oy1 - 15 < cy < oy2 + 15 for ox1, oy1, ox2, oy2 in covered_boxes):
                            det = Detection2D()
                            det.bbox.center.position.x = float(cx)
                            det.bbox.center.position.y = float(cy)
                            det.bbox.size_x = float(bw)
                            det.bbox.size_y = float(bh)
                            hyp = ObjectHypothesisWithPose()
                            hyp.hypothesis.class_id = "plate|white"
                            hyp.hypothesis.score = 0.92
                            det.results.append(hyp)
                            det.id = str(self._get_persistent_track_id(cx, cy, "plate|white"))
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
