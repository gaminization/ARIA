#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA SAM2 Segmentation Node
Segment Anything Model v2 — pixel-accurate object masks.

Model: SAM2-tiny (smallest, fastest, 0.5GB VRAM)
Always on — runs on every YOLO detection update.

Input:  YOLO bounding boxes as box prompts
Output: Per-object binary masks + annotated image

Improvements over bounding boxes:
  1. Better grasp planning (actual object silhouette)
  2. Better depth estimation (mean within mask, not bbox center)
  3. Better affordance (locate handle sub-regions)
  4. Occlusion handling (partial mask → estimate full shape)

Subscribes:
  /top_camera/image_raw
  /detection/objects (YOLO detections)

Publishes:
  /sam2/masks       (arm_interfaces/SAM2Masks)
  /sam2/masks_json  (String — serialized mask data)
  /sam2/image_masked (Image — RGB with colored overlays)
═══════════════════════════════════════════════════════════════
"""
import json
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

try:
    from cv_bridge import CvBridge
    CV_BRIDGE = True
except ImportError:
    CV_BRIDGE = False

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


# ═══════════════════════════════════════════════════════════════
# SAM2 Model Wrapper
# ═══════════════════════════════════════════════════════════════
class SAM2Model:
    """
    Wrapper for SAM2 inference.

    If sam2 is installed: uses the real model (SAM2-tiny, 0.5GB).
    Otherwise: generates masks from bounding boxes with
    GrabCut refinement as a fallback.
    """

    # Mask overlay colors (BGR) for visualization
    COLORS = [
        (255, 100, 100), (100, 255, 100), (100, 100, 255),
        (255, 255, 100), (255, 100, 255), (100, 255, 255),
        (200, 150, 100), (100, 200, 150), (150, 100, 200),
        (255, 200, 100), (200, 100, 255), (100, 255, 200),
    ]

    def __init__(self, device: str = 'cuda:0'):
        self.device = device
        self.predictor = None
        self.loaded = False
        self._use_real_model = False

    def load(self) -> bool:
        """Load SAM2/SAM2.1 tiny model."""
        if self.loaded:
            return True

        try:
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
            import os

            # Prefer SAM2.0 checkpoint + config (exact match)
            # Fall back to SAM2.1 checkpoint + SAM2.1 config
            aria_dir = os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))))

            candidates = [
                # SAM2.0 in models/ or root
                ("sam2_hiera_t.yaml",
                 os.path.join(aria_dir, "models", "sam2_hiera_tiny.pt")),
                ("sam2_hiera_t.yaml",
                 os.path.join(aria_dir, "sam2_hiera_tiny.pt")),
                # SAM2.1 in models/ or root
                ("sam2.1/sam2.1_hiera_t.yaml",
                 os.path.join(aria_dir, "models", "sam2.1_hiera_tiny.pt")),
                ("sam2.1/sam2.1_hiera_t.yaml",
                 os.path.join(aria_dir, "sam2.1_hiera_tiny.pt")),
            ]

            loaded = False
            for cfg, ckpt in candidates:
                if not os.path.exists(ckpt):
                    continue
                try:
                    sam2_model = build_sam2(cfg, ckpt, device=self.device)
                    self.predictor = SAM2ImagePredictor(sam2_model)
                    loaded = True
                    print(f"[SAM2] Loaded {os.path.basename(ckpt)} with {cfg}")
                    break
                except Exception as e:
                    print(f"[SAM2] {cfg} + {os.path.basename(ckpt)} failed: {e}")

            self.loaded = True
            self._use_real_model = loaded
            return True

        except (ImportError, RuntimeError) as e:
            # SAM2 not available — use GrabCut fallback
            self.loaded = True
            self._use_real_model = False
            return True


    def unload(self):
        """Free VRAM."""
        if self.predictor is not None:
            del self.predictor
            self.predictor = None
        if TORCH_AVAILABLE and torch.cuda.is_available():
            torch.cuda.empty_cache()
        self.loaded = False
        self._use_real_model = False

    def segment(
        self,
        image: np.ndarray,
        boxes: List[Tuple[int, int, int, int]],
    ) -> List[Tuple[np.ndarray, float]]:
        """
        Generate masks for each bounding box prompt.

        Args:
            image: BGR image (H, W, 3)
            boxes: List of (x1, y1, x2, y2) bounding boxes

        Returns:
            List of (mask, confidence) tuples
            mask: binary uint8 (H, W), 0 or 255
        """
        if not self.loaded:
            self.load()

        if self._use_real_model and self.predictor is not None:
            return self._segment_sam2(image, boxes)
        else:
            return self._segment_fallback(image, boxes)

    def _segment_sam2(
        self,
        image: np.ndarray,
        boxes: List[Tuple[int, int, int, int]],
    ) -> List[Tuple[np.ndarray, float]]:
        """Real SAM2 inference."""
        results = []

        # Set image (amortized for multiple prompts)
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.predictor.set_image(rgb)

        for box in boxes:
            x1, y1, x2, y2 = box
            input_box = np.array([[x1, y1, x2, y2]])

            masks, scores, _ = self.predictor.predict(
                point_coords=None,
                point_labels=None,
                box=input_box,
                multimask_output=False,
            )

            if masks is not None and len(masks) > 0:
                mask = (masks[0] * 255).astype(np.uint8)
                conf = float(scores[0]) if scores is not None else 0.9
                results.append((mask, conf))
            else:
                # Fallback for this box
                mask = self._bbox_mask(image.shape[:2], box)
                results.append((mask, 0.5))

        self.predictor.reset_image()
        return results

    def _segment_fallback(
        self,
        image: np.ndarray,
        boxes: List[Tuple[int, int, int, int]],
    ) -> List[Tuple[np.ndarray, float]]:
        """
        Fallback segmentation using GrabCut when SAM2 unavailable.
        Uses bbox as initialization for iterative foreground extraction.
        """
        results = []

        for box in boxes:
            x1, y1, x2, y2 = box
            h, w = image.shape[:2]

            # Clamp
            x1 = max(0, min(x1, w - 1))
            y1 = max(0, min(y1, h - 1))
            x2 = max(x1 + 1, min(x2, w))
            y2 = max(y1 + 1, min(y2, h))

            bw = x2 - x1
            bh = y2 - y1

            if bw < 5 or bh < 5:
                mask = self._bbox_mask((h, w), box)
                results.append((mask, 0.4))
                continue

            if CV2_AVAILABLE:
                try:
                    mask_gc = np.zeros((h, w), dtype=np.uint8)
                    bgd_model = np.zeros((1, 65), dtype=np.float64)
                    fgd_model = np.zeros((1, 65), dtype=np.float64)
                    rect = (x1, y1, bw, bh)

                    cv2.grabCut(
                        image, mask_gc, rect,
                        bgd_model, fgd_model,
                        iterCount=3,
                        mode=cv2.GC_INIT_WITH_RECT,
                    )

                    # GrabCut output: 0=bg, 1=fg, 2=probable_bg, 3=probable_fg
                    mask = np.where(
                        (mask_gc == cv2.GC_FGD) | (mask_gc == cv2.GC_PR_FGD),
                        255, 0
                    ).astype(np.uint8)

                    # If GrabCut produced empty mask, use bbox
                    if np.sum(mask > 0) < 50:
                        mask = self._bbox_mask((h, w), box)
                        results.append((mask, 0.5))
                    else:
                        results.append((mask, 0.65))
                    continue

                except cv2.error:
                    pass

            # Pure bbox mask
            mask = self._bbox_mask((h, w), box)
            results.append((mask, 0.4))

        return results

    @staticmethod
    def _bbox_mask(
        shape: Tuple[int, int],
        box: Tuple[int, int, int, int],
    ) -> np.ndarray:
        """Generate a simple rectangular mask from bounding box."""
        h, w = shape
        mask = np.zeros((h, w), dtype=np.uint8)
        x1, y1, x2, y2 = box
        x1 = max(0, int(x1))
        y1 = max(0, int(y1))
        x2 = min(w, int(x2))
        y2 = min(h, int(y2))

        # Slight inset (10%) to approximate object boundary
        inset_x = max(1, int((x2 - x1) * 0.1))
        inset_y = max(1, int((y2 - y1) * 0.1))
        mask[y1 + inset_y:y2 - inset_y, x1 + inset_x:x2 - inset_x] = 255
        return mask


# ═══════════════════════════════════════════════════════════════
# Mask analysis utilities
# ═══════════════════════════════════════════════════════════════
def compute_mask_properties(mask: np.ndarray) -> dict:
    """
    Compute geometric properties of a binary mask.

    Returns:
        major_axis_angle: angle of major axis in degrees
        aspect_ratio:     width/height of minimum bounding rect
        centroid:         (cx, cy) pixel coordinates
        area:             pixel area
        contour:          largest contour points
    """
    if mask is None or np.sum(mask > 0) < 10:
        return {}

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    ) if CV2_AVAILABLE else ([], None)

    if not contours:
        return {}

    # Largest contour
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < 10:
        return {}

    # Minimum bounding rectangle
    rect = cv2.minAreaRect(largest)
    center, (w_rect, h_rect), angle = rect

    # Ensure width > height for consistent major axis
    if w_rect < h_rect:
        w_rect, h_rect = h_rect, w_rect
        angle += 90

    # Moments for centroid
    M = cv2.moments(largest)
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = int(center[0]), int(center[1])

    return {
        'major_axis_angle': float(angle % 180),
        'aspect_ratio': float(w_rect / max(h_rect, 1)),
        'centroid': (cx, cy),
        'area': float(area),
        'min_rect_size': (float(w_rect), float(h_rect)),
        'contour': largest,
    }


# ═══════════════════════════════════════════════════════════════
# ROS2 Node
# ═══════════════════════════════════════════════════════════════
class SAM2Node(Node):
    """
    SAM2 segmentation for ARIA.

    Always-on (0.5GB VRAM for tiny model).
    Runs on every YOLO detection to provide pixel-accurate masks.

    Pipeline:
      1. Receive YOLO detections (bounding boxes)
      2. Run SAM2 with box prompts
      3. Publish binary masks per object
      4. Publish annotated image with colored overlays
    """

    def __init__(self):
        super().__init__('sam2_node')
        self.get_logger().info("═══ ARIA SAM2 Segmentation Node starting ═══")

        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('skip_stationary', True)
        self.declare_parameter('min_mask_area', 100)

        device = self.get_parameter('device').value
        self.skip_stationary = self.get_parameter('skip_stationary').value
        self.min_mask_area = self.get_parameter('min_mask_area').value

        # Model
        self.sam2 = SAM2Model(device=device)
        self.bridge = CvBridge() if CV_BRIDGE else None

        # Data buffers
        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_detections: Optional[Detection2DArray] = None
        self._mask_cache: Dict[int, Tuple[np.ndarray, float]] = {}

        # Performance tracking
        self._frame_count = 0
        self._total_ms = 0.0

        # ── Subscribers ────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=5)

        self.create_subscription(
            Image, '/top_camera/image_raw',
            self._rgb_cb, qos)
        self.create_subscription(
            Detection2DArray, '/detection/objects',
            self._detection_cb, 10)

        # ── Publishers ─────────────────────────────────────
        self.masks_json_pub = self.create_publisher(
            String, '/sam2/masks_json', 10)
        self.masked_img_pub = self.create_publisher(
            Image, '/sam2/image_masked', 10)
        self.status_pub = self.create_publisher(
            String, '/sam2/status', 10)

        # Load model on startup
        self.create_timer(1.0, self._load_model_once)

        self.get_logger().info("SAM2 node ready")

    def _load_model_once(self):
        """Load SAM2 model on first tick."""
        if not self.sam2.loaded:
            ok = self.sam2.load()
            mode = "real SAM2" if self.sam2._use_real_model else "GrabCut fallback"
            self.get_logger().info(
                f"SAM2 model loaded: {mode}")
            status = String()
            status.data = f"loaded:{mode}"
            self.status_pub.publish(status)

    # ── Data callbacks ─────────────────────────────────────
    def _rgb_cb(self, msg: Image):
        if self.bridge:
            try:
                self._latest_rgb = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            except Exception:
                pass

    def _detection_cb(self, msg: Detection2DArray):
        """Process new detections: generate SAM2 masks."""
        self._latest_detections = msg

        if self._latest_rgb is None:
            return

        t_start = time.perf_counter()

        # Extract bounding boxes and object IDs
        boxes = []
        obj_ids = []
        class_names = []

        for det in msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            bw = det.bbox.size_x
            bh = det.bbox.size_y

            x1 = int(cx - bw / 2)
            y1 = int(cy - bh / 2)
            x2 = int(cx + bw / 2)
            y2 = int(cy + bh / 2)
            boxes.append((x1, y1, x2, y2))

            try:
                oid = int(det.id) if det.id else len(obj_ids)
            except (ValueError, AttributeError):
                oid = len(obj_ids)
            obj_ids.append(oid)

            cls = 'unknown'
            if det.results:
                cls = det.results[0].hypothesis.class_id
            class_names.append(cls)

        if not boxes:
            return

        # Run SAM2 segmentation
        mask_results = self.sam2.segment(self._latest_rgb, boxes)

        # Update mask cache and build JSON output
        masks_dict = {}
        confidences = []
        areas = []

        for i, (mask, conf) in enumerate(mask_results):
            oid = obj_ids[i]
            self._mask_cache[oid] = (mask, conf)
            masks_dict[str(oid)] = mask.tolist()
            confidences.append(conf)
            areas.append(float(np.sum(mask > 0)))

        elapsed_ms = (time.perf_counter() - t_start) * 1000
        self._frame_count += 1
        self._total_ms += elapsed_ms

        # Publish masks as JSON (for pose_6d_node and others)
        # Note: for large masks, only publish reduced data
        mask_summary = {}
        for i, (mask, conf) in enumerate(mask_results):
            oid = obj_ids[i]
            props = compute_mask_properties(mask) if CV2_AVAILABLE else {}
            mask_summary[str(oid)] = {
                'confidence': float(conf),
                'area': float(np.sum(mask > 0)),
                'centroid': props.get('centroid', [0, 0]),
                'major_axis_angle': props.get('major_axis_angle', 0),
                'aspect_ratio': props.get('aspect_ratio', 1.0),
                'class_name': class_names[i],
            }

        json_msg = String()
        json_msg.data = json.dumps(mask_summary)
        self.masks_json_pub.publish(json_msg)

        # Publish annotated image
        if CV2_AVAILABLE and self.bridge:
            annotated = self._draw_masks(
                self._latest_rgb, mask_results, obj_ids, class_names)
            try:
                ann_msg = self.bridge.cv2_to_imgmsg(
                    annotated, encoding='bgr8')
                ann_msg.header = msg.header
                self.masked_img_pub.publish(ann_msg)
            except Exception:
                pass

        # Periodic stats
        if self._frame_count % 100 == 0:
            avg = self._total_ms / self._frame_count
            self.get_logger().info(
                f"SAM2 stats ({self._frame_count} frames): "
                f"mean={avg:.1f}ms, {len(boxes)} objects")

    def _draw_masks(
        self,
        image: np.ndarray,
        mask_results: List[Tuple[np.ndarray, float]],
        obj_ids: List[int],
        class_names: List[str],
    ) -> np.ndarray:
        """Draw colored mask overlays on image."""
        overlay = image.copy()

        for i, (mask, conf) in enumerate(mask_results):
            color = SAM2Model.COLORS[i % len(SAM2Model.COLORS)]
            colored = np.zeros_like(overlay)
            colored[:] = color

            # Apply mask overlay with alpha blending
            mask_bool = mask > 0
            overlay[mask_bool] = cv2.addWeighted(
                overlay[mask_bool], 0.6,
                colored[mask_bool], 0.4, 0
            )

            # Draw contour
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay, contours, -1, color, 2)

            # Label
            if contours:
                M = cv2.moments(contours[0])
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    label = f"{class_names[i]} #{obj_ids[i]} ({conf:.0%})"
                    cv2.putText(
                        overlay, label, (cx - 30, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        return overlay


def main(args=None):
    rclpy.init(args=args)
    node = SAM2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sam2.unload()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
