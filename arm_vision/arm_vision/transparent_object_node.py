#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Transparent Object Handler
Corrects depth estimation failures on glass, metallic, and
reflective surfaces.

Problem: Depth-Anything v2 sees through glass → estimates
background depth instead of glass surface depth.

Solution:
  1. Detect transparent/reflective objects automatically
  2. Correct depth using geometric plane fitting from
     surrounding opaque surfaces
  3. Optional: ClearGrasp model for learned correction

Subscribes:
  /top_camera/image_raw
  /depth/image_depth_anything
  /detection/objects
  /sam2/masks_json

Publishes:
  /depth/image_completed        (32FC1 — corrected depth map)
  /perception/transparent_flags (String JSON — per-object flags)
  /perception/depth_quality     (String JSON — per-object confidence)
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
# Transparent object class lists
# ═══════════════════════════════════════════════════════════════
# YOLO COCO classes that are often transparent
TRANSPARENT_CLASSES = {
    'wine glass', 'glass', 'cup', 'bottle', 'vase',
    'bowl',  # glass bowls
}

# Classes that are often reflective/metallic
REFLECTIVE_CLASSES = {
    'knife', 'scissors', 'spoon', 'fork',
}

# Additional keywords in class names
TRANSPARENT_KEYWORDS = [
    'glass', 'transparent', 'clear', 'crystal', 'plastic_wrap',
]


# ═══════════════════════════════════════════════════════════════
# Depth correction algorithms
# ═══════════════════════════════════════════════════════════════
def detect_depth_bimodality(
    depth_patch: np.ndarray,
    threshold: float = 0.08,
) -> Tuple[bool, float]:
    """
    Check if a depth patch has bimodal distribution.

    Transparent objects often show:
      - Some pixels at object distance (edge/frame)
      - Other pixels at background distance (through glass)
    → Bimodal = likely transparent.

    Returns:
        (is_bimodal, gap_meters)
    """
    valid = depth_patch[depth_patch > 0.01].flatten()
    if len(valid) < 20:
        return False, 0.0

    # Sort and find gap
    sorted_depths = np.sort(valid)
    n = len(sorted_depths)

    # Look for largest gap in middle 80%
    start = int(n * 0.1)
    end = int(n * 0.9)
    diffs = np.diff(sorted_depths[start:end])

    if len(diffs) == 0:
        return False, 0.0

    max_gap = np.max(diffs)
    median_depth = np.median(valid)

    # Bimodal if gap > threshold * median depth
    is_bimodal = max_gap > threshold * median_depth
    return is_bimodal, float(max_gap)


def detect_specular_highlights(
    rgb_patch: np.ndarray,
    saturation_threshold: int = 240,
    min_pixel_fraction: float = 0.02,
) -> bool:
    """
    Detect specular highlights indicating metallic/glossy surface.

    Bright, near-white patches on surface → reflective.
    """
    if rgb_patch is None or rgb_patch.size == 0:
        return False

    # Convert to grayscale
    if len(rgb_patch.shape) == 3:
        gray = np.mean(rgb_patch, axis=2)
    else:
        gray = rgb_patch.astype(float)

    # Count bright pixels
    bright_pixels = np.sum(gray > saturation_threshold)
    total_pixels = gray.size

    fraction = bright_pixels / max(total_pixels, 1)
    return fraction > min_pixel_fraction


def correct_depth_plane_fitting(
    depth_map: np.ndarray,
    mask: np.ndarray,
    expansion_px: int = 30,
) -> Optional[np.ndarray]:
    """
    Correct depth within a transparent object mask using
    plane fitting from surrounding opaque surfaces.

    Method:
      1. Expand mask by N pixels to get surrounding ring
      2. Collect depth values in the ring (opaque surface)
      3. Fit a plane z = ax + by + c to ring depths
      4. Fill transparent mask with plane depth

    This assumes the transparent object sits on a surface,
    and the surface plane continues underneath the object.
    """
    if mask is None or depth_map is None:
        return None

    h, w = depth_map.shape[:2]

    # Create surrounding ring: expanded mask minus original mask
    if CV2_AVAILABLE:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (expansion_px * 2 + 1, expansion_px * 2 + 1))
        expanded = cv2.dilate(mask, kernel, iterations=1)
    else:
        expanded = np.copy(mask)
        # Simple expansion via shifting
        for dy in range(-expansion_px, expansion_px + 1):
            for dx in range(-expansion_px, expansion_px + 1):
                if dx * dx + dy * dy <= expansion_px * expansion_px:
                    shifted = np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
                    expanded = np.maximum(expanded, shifted)

    ring = (expanded > 0) & (mask == 0)

    # Get ring pixel positions and depths
    ys, xs = np.where(ring)
    ring_depths = depth_map[ys, xs]

    # Filter valid depths
    valid = ring_depths > 0.01
    if np.sum(valid) < 10:
        return None

    xs_valid = xs[valid].astype(np.float64)
    ys_valid = ys[valid].astype(np.float64)
    ds_valid = ring_depths[valid].astype(np.float64)

    # Fit plane: z = ax + by + c
    # Using least squares: [x, y, 1] @ [a, b, c]^T = z
    A = np.column_stack([xs_valid, ys_valid, np.ones(len(xs_valid))])
    try:
        result = np.linalg.lstsq(A, ds_valid, rcond=None)
        plane_coeffs = result[0]  # [a, b, c]
    except np.linalg.LinAlgError:
        return None

    # Evaluate plane at mask pixels
    mask_ys, mask_xs = np.where(mask > 0)
    if len(mask_xs) == 0:
        return None

    A_mask = np.column_stack([
        mask_xs.astype(np.float64),
        mask_ys.astype(np.float64),
        np.ones(len(mask_xs)),
    ])
    plane_depths = A_mask @ plane_coeffs

    # Clamp to reasonable range
    median_ring = np.median(ds_valid)
    plane_depths = np.clip(
        plane_depths,
        median_ring - 0.10,  # max 10cm below surface
        median_ring + 0.05,  # max 5cm above surface
    )

    # Create corrected depth map
    corrected = depth_map.copy()
    corrected[mask_ys, mask_xs] = plane_depths.astype(np.float32)

    return corrected


# ═══════════════════════════════════════════════════════════════
# ClearGrasp Model Wrapper (optional)
# ═══════════════════════════════════════════════════════════════
class ClearGraspModel:
    """
    Wrapper for ClearGrasp depth completion on transparent objects.

    If ClearGrasp is installed: uses the learned model (~0.6GB).
    Otherwise: falls back to geometric plane fitting.
    """

    def __init__(self, device: str = 'cuda:0'):
        self.device = device
        self.model = None
        self.loaded = False
        self._use_real_model = False

    def load(self) -> bool:
        if self.loaded:
            return True
        try:
            # Attempt ClearGrasp import
            from cleargrasp.api import ClearGraspAPI
            self.model = ClearGraspAPI(device=self.device)
            self.loaded = True
            self._use_real_model = True
            return True
        except (ImportError, Exception):
            self.loaded = True
            self._use_real_model = False
            return True

    def unload(self):
        if self.model is not None:
            del self.model
            self.model = None
        if TORCH_AVAILABLE and torch.cuda.is_available():
            torch.cuda.empty_cache()
        self.loaded = False
        self._use_real_model = False

    def complete_depth(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: np.ndarray,
    ) -> Optional[np.ndarray]:
        """Complete depth for transparent regions."""
        if not self.loaded:
            self.load()

        if self._use_real_model and self.model is not None:
            try:
                return self.model.complete(rgb, depth, mask)
            except Exception:
                pass

        # Geometric fallback
        return correct_depth_plane_fitting(depth, mask)


# ═══════════════════════════════════════════════════════════════
# ROS2 Node
# ═══════════════════════════════════════════════════════════════
class TransparentObjectNode(Node):
    """
    Transparent/reflective object handler for ARIA.

    Loaded on demand when transparent objects are detected.

    Pipeline:
      1. Check each detected object for transparency
      2. If transparent: correct depth using plane fitting
         or ClearGrasp model
      3. Merge corrected depth into main depth map
      4. Publish corrected depth + quality flags
    """

    def __init__(self):
        super().__init__('transparent_object_node')
        self.get_logger().info(
            "═══ ARIA Transparent Object Node starting ═══")

        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('use_cleargrasp', False)
        self.declare_parameter('bimodal_threshold', 0.08)

        device = self.get_parameter('device').value
        use_cg = self.get_parameter('use_cleargrasp').value
        self.bimodal_threshold = self.get_parameter(
            'bimodal_threshold').value

        # ClearGrasp model (loaded on demand)
        self.cleargrasp = ClearGraspModel(device=device)
        if use_cg:
            self.cleargrasp.load()

        self.bridge = CvBridge() if CV_BRIDGE else None

        # Data buffers
        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_detections: Optional[Detection2DArray] = None
        self._mask_data: dict = {}

        # Stats
        self._corrections_applied = 0

        # ── Subscribers ────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=5)

        self.create_subscription(
            Image, '/top_camera/image_raw', self._rgb_cb, qos)
        self.create_subscription(
            Image, '/depth/image_depth_anything', self._depth_cb, qos)
        self.create_subscription(
            Detection2DArray, '/detection/objects',
            self._detection_cb, 10)
        self.create_subscription(
            String, '/sam2/masks_json', self._masks_cb, 10)

        # ── Publishers ─────────────────────────────────────
        self.depth_pub = self.create_publisher(
            Image, '/depth/image_completed', 10)
        self.flags_pub = self.create_publisher(
            String, '/perception/transparent_flags', 10)
        self.quality_pub = self.create_publisher(
            String, '/perception/depth_quality', 10)

        # ── Processing timer ───────────────────────────────
        self.create_timer(0.1, self._tick)  # 10 Hz

        self.get_logger().info("Transparent object node ready")

    # ── Data callbacks ─────────────────────────────────────
    def _rgb_cb(self, msg: Image):
        if self.bridge:
            try:
                self._latest_rgb = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            except Exception:
                pass

    def _depth_cb(self, msg: Image):
        if self.bridge:
            try:
                self._latest_depth = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='32FC1')
            except Exception:
                pass

    def _detection_cb(self, msg: Detection2DArray):
        self._latest_detections = msg

    def _masks_cb(self, msg: String):
        try:
            self._mask_data = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ── Main processing tick ───────────────────────────────
    def _tick(self):
        if self._latest_depth is None or self._latest_detections is None:
            return
        if self._latest_rgb is None:
            return

        h, w = self._latest_depth.shape[:2]
        corrected_depth = self._latest_depth.copy()
        transparent_flags: Dict[str, dict] = {}
        depth_quality: Dict[str, float] = {}

        any_correction = False

        for det in self._latest_detections.detections:
            # Get class name
            cls_name = 'unknown'
            if det.results:
                cls_name = det.results[0].hypothesis.class_id

            # Get object ID
            try:
                obj_id = int(det.id) if det.id else -1
            except (ValueError, AttributeError):
                obj_id = -1

            # Get bounding box region
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            bw = det.bbox.size_x
            bh = det.bbox.size_y
            x1 = max(0, int(cx - bw / 2))
            y1 = max(0, int(cy - bh / 2))
            x2 = min(w, int(cx + bw / 2))
            y2 = min(h, int(cy + bh / 2))

            if x2 <= x1 or y2 <= y1:
                continue

            # ── Check for transparency ─────────────────────
            is_transparent = False
            transparency_reason = ""

            # Method 1: Class-based check
            cls_lower = cls_name.lower()
            if cls_lower in TRANSPARENT_CLASSES:
                is_transparent = True
                transparency_reason = f"class={cls_lower}"
            elif any(kw in cls_lower for kw in TRANSPARENT_KEYWORDS):
                is_transparent = True
                transparency_reason = f"keyword in {cls_lower}"
            elif cls_lower in REFLECTIVE_CLASSES:
                is_transparent = True
                transparency_reason = f"reflective class={cls_lower}"

            # Method 2: Depth bimodality check
            if not is_transparent:
                depth_patch = self._latest_depth[y1:y2, x1:x2]
                is_bimodal, gap = detect_depth_bimodality(
                    depth_patch, self.bimodal_threshold)
                if is_bimodal:
                    is_transparent = True
                    transparency_reason = f"bimodal depth (gap={gap:.3f}m)"

            # Method 3: Specular highlight check
            if not is_transparent:
                rgb_patch = self._latest_rgb[y1:y2, x1:x2]
                if detect_specular_highlights(rgb_patch):
                    is_transparent = True
                    transparency_reason = "specular highlights"

            # Store flags
            transparent_flags[str(obj_id)] = {
                'class': cls_name,
                'is_transparent': is_transparent,
                'reason': transparency_reason,
            }

            # ── Correct depth if transparent ───────────────
            if is_transparent:
                # Create mask for this object
                mask = np.zeros((h, w), dtype=np.uint8)
                mask[y1:y2, x1:x2] = 255

                # Try using SAM2 mask if available
                obj_key = str(obj_id)
                if obj_key in self._mask_data:
                    # SAM2 provides mask metadata; use bbox mask
                    # (full mask data from sam2 node not available via JSON)
                    pass

                # Apply depth correction
                result = correct_depth_plane_fitting(
                    corrected_depth, mask)
                if result is not None:
                    corrected_depth = result
                    any_correction = True
                    self._corrections_applied += 1
                    depth_quality[str(obj_id)] = 0.70  # Corrected
                else:
                    depth_quality[str(obj_id)] = 0.30  # Failed
            else:
                depth_quality[str(obj_id)] = 0.90  # Opaque = high quality

        # ── Publish results ────────────────────────────────
        if any_correction and self.bridge:
            try:
                depth_msg = self.bridge.cv2_to_imgmsg(
                    corrected_depth, encoding='32FC1')
                depth_msg.header.stamp = self.get_clock().now().to_msg()
                depth_msg.header.frame_id = 'top_camera_link'
                self.depth_pub.publish(depth_msg)
            except Exception:
                pass

        # Always publish flags and quality
        flags_msg = String()
        flags_msg.data = json.dumps(transparent_flags)
        self.flags_pub.publish(flags_msg)

        quality_msg = String()
        quality_msg.data = json.dumps(depth_quality)
        self.quality_pub.publish(quality_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TransparentObjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleargrasp.unload()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
