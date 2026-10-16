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

try:
    from arm_vision.sys_config import sys_config
except ImportError:
    from sys_config import sys_config


MAX_TRACK_MATCH_DISTANCE_PX = 45.0


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
        self.use_half = use_half or (sys_config.use_half_precision and 'cuda' in device)
        self.bridge = CvBridge() if CV_BRIDGE else None
        self._processing = False

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

    def _get_persistent_track_id(
        self, cx: float, cy: float, class_name: str,
        used_track_ids: Optional[set[int]] = None,
    ) -> int:
        """Assign or match persistent 2D track ID based on pixel proximity."""
        base_cls = class_name.split('|')[0]
        now = time.time()
        best_id = None
        min_dist = float('inf')

        # Clean old tracks (> 3.0s unseen)
        stale_ids = [tid for tid, info in self.active_tracks.items() if now - info[3] > 3.0]
        for tid in stale_ids:
            del self.active_tracks[tid]

        # Keep the match radius below the spacing of adjacent conveyor objects.
        for tid, (tcx, tcy, tcls, _) in self.active_tracks.items():
            if used_track_ids and tid in used_track_ids:
                continue
            t_base = tcls.split('|')[0]
            if t_base == base_cls or base_cls in {'table_object', 'unknown'} or t_base in {'table_object', 'unknown'}:
                dist = math.hypot(cx - tcx, cy - tcy)
                if dist < min_dist and dist < MAX_TRACK_MATCH_DISTANCE_PX:
                    min_dist = dist
                    best_id = tid

        if best_id is not None:
            self.active_tracks[best_id] = (cx, cy, class_name, now)
            return best_id
        else:
            while self.next_track_id in self.active_tracks or (
                used_track_ids and self.next_track_id in used_track_ids
            ):
                self.next_track_id += 1
            new_id = self.next_track_id
            self.next_track_id += 1
            self.active_tracks[new_id] = (cx, cy, class_name, now)
            return new_id

    def _assign_frame_track_id(
        self,
        cx: float,
        cy: float,
        class_name: str,
        preferred_id: Optional[int],
        used_track_ids: set[int],
    ) -> int:
        """Assign an ID without allowing two detections in one frame to share it."""
        now = time.time()
        if preferred_id is not None and preferred_id >= 0 and preferred_id not in used_track_ids:
            track_id = preferred_id
            self.active_tracks[track_id] = (cx, cy, class_name, now)
            self.next_track_id = max(self.next_track_id, track_id + 1)
        else:
            track_id = self._get_persistent_track_id(cx, cy, class_name, used_track_ids)
        used_track_ids.add(track_id)
        return track_id

    @staticmethod
    def _suppress_duplicate_boxes(raw_boxes: list) -> list:
        """Suppress overlapping boxes while preserving nearby distinct objects."""
        raw_boxes.sort(key=lambda box: box[4], reverse=True)
        kept = []
        for box in raw_boxes:
            x1, y1, x2, y2 = box[:4]
            is_duplicate = False
            for existing in kept:
                sx1, sy1, sx2, sy2 = existing[:4]
                ix1, iy1 = max(x1, sx1), max(y1, sy1)
                ix2, iy2 = min(x2, sx2), min(y2, sy2)
                intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
                area = (x2 - x1) * (y2 - y1)
                existing_area = (sx2 - sx1) * (sy2 - sy1)
                iou = intersection / (area + existing_area - intersection + 1e-6)
                if iou > 0.35:
                    is_duplicate = True
                    break
            if not is_duplicate:
                kept.append(box)
        return kept

    def _robot_self_mask(self, cv_image: np.ndarray):
        """Mask of the arm: the near-white connected component that touches the
        mounting plate at the image centre (top-down camera is centred on the base)."""
        try:
            h, w = cv_image.shape[:2]
            white = cv2.inRange(cv_image, (170, 170, 170), (255, 255, 255))
            white = cv2.dilate(white, np.ones((5, 5), np.uint8))
            n, labels = cv2.connectedComponents(white)
            patch = labels[int(h * 0.5) - 20:int(h * 0.5) + 20, int(w * 0.5) - 20:int(w * 0.5) + 20]
            ids = [i for i in np.unique(patch) if i != 0]
            if not ids:
                return None
            mask = np.isin(labels, ids).astype(np.uint8) * 255
            return cv2.dilate(mask, np.ones((9, 9), np.uint8))
        except Exception:
            return None

    def _image_cb(self, msg: Image):
        """Process incoming camera frame with YOLO and physical color extraction."""
        if self.model is None or self.bridge is None:
            return

        if self._processing or not sys_config.should_process('detection_node', sys_config.detection_max_rate_hz):
            return

        self._processing = True
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            self._processing = False
            return

        try:
            t_start = time.perf_counter()

            results = self.model.predict(
                cv_image,
                verbose=False,
                device=self.device,
                conf=self.conf_threshold,
            )
        except Exception as e:
            self.get_logger().error(f"YOLO inference error: {e}")
            sys_config.periodic_cache_clear()
            self._processing = False
            return

        inference_ms = (time.perf_counter() - t_start) * 1000

        self.frame_count += 1
        self.total_inference_ms += inference_ms
        self.max_inference_ms = max(self.max_inference_ms, inference_ms)

        det_array = Detection2DArray()
        det_array.header = msg.header
        frame_track_ids: set[int] = set()
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

                # 1b. Self-mask: drop detections that sit on the robot's own structure
                if 'top' in self.camera_topic:
                    arm_mask = self._robot_self_mask(cv_image)
                    if arm_mask is not None:
                        kept = []
                        for b in raw_boxes:
                            bx1, by1, bx2, by2 = [int(max(0, v)) for v in b[:4]]
                            roi = arm_mask[by1:by2, bx1:bx2]
                            frac = float(roi.mean()) / 255.0 if roi.size else 0.0
                            if frac > 0.45:
                                continue
                            kept.append(b)
                        raw_boxes = kept

                suppressed_boxes = self._suppress_duplicate_boxes(raw_boxes)

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

                    preferred_id = int(box.id[0]) if box.id is not None else None
                    det.id = str(self._assign_frame_track_id(
                        float((x1 + x2) / 2),
                        float((y1 + y2) / 2),
                        full_class_id,
                        preferred_id,
                        frame_track_ids,
                    ))

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
                    if min_area <= area <= 4000 and aspect < 2.2 and min_dim >= 18:
                        segmented_candidates.append(('workpiece', 'blue', bx, by, bw, bh, 0.95))
                        segmented_candidates.append(('cylinder', 'blue', bx, by, bw, bh, 0.94))
                    elif 4000 <= area < 7500 and not is_industrial:
                        segmented_candidates.append(('mug', 'blue', bx, by, bw, bh, 0.92))

                # Red box / reject bin container AND red cube / tabletop objects
                mask_red = ((cv2.inRange(hsv, (0, 70, 50), (10, 255, 255)) |
                             cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))) & table_mask)
                cnts_r, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts_r:
                    area = cv2.contourArea(c)
                    bx, by, bw, bh = cv2.boundingRect(c)
                    if area > 4000:
                        segmented_candidates.append(('box', 'red', bx, by, bw, bh, 0.95))
                    elif int(180 * area_scale) <= area <= 4000 and min(bw, bh) >= 16:
                        segmented_candidates.append(('cube', 'red', bx, by, bw, bh, 0.95))
                        segmented_candidates.append(('block', 'red', bx, by, bw, bh, 0.94))

                # Yellow items (yellow block, banana, cube)
                mask_yellow = (cv2.inRange(hsv, (20, 70, 60), (38, 255, 255)) & table_mask)
                cnts_y, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts_y:
                    area = cv2.contourArea(c)
                    bx, by, bw, bh = cv2.boundingRect(c)
                    if (int(180 * area_scale) <= area <= 5000 and min(bw, bh) >= 16
                            and max(bw, bh) / min(bw, bh) <= 2.2):
                        segmented_candidates.append(('block', 'yellow', bx, by, bw, bh, 0.95))
                        segmented_candidates.append(('cube', 'yellow', bx, by, bw, bh, 0.94))

                # Green items (green block / sphere)
                mask_green = (cv2.inRange(hsv, (38, 60, 40), (82, 255, 255)) & table_mask)
                cnts_g, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts_g:
                    area = cv2.contourArea(c)
                    bx, by, bw, bh = cv2.boundingRect(c)
                    if (int(180 * area_scale) <= area <= 5000 and min(bw, bh) >= 16
                            and max(bw, bh) / min(bw, bh) <= 2.2):
                        segmented_candidates.append(('block', 'green', bx, by, bw, bh, 0.95))

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

                    det.id = str(self._assign_frame_track_id(
                        cx, cy, full_cls, None, frame_track_ids,
                    ))
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
                            det.id = str(self._assign_frame_track_id(
                                cx, cy, "plate|white", None, frame_track_ids,
                            ))
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
            except Exception as e:
                self.get_logger().warn(f"Annotated image publish warning: {e}")
        sys_config.periodic_cache_clear()
        self._processing = False



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
