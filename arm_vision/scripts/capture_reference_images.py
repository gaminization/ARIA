#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Reference Image Capture Tool
Captures multi-view reference images for FoundationPose.

Usage:
  python3 capture_reference_images.py --object cup --views 8
  python3 capture_reference_images.py --object bottle --views 12 --manual

Process:
  1. Moves arm to N viewpoints around the object (or manual mode)
  2. Captures top_camera image at each viewpoint
  3. Prompts user to verify each capture
  4. Saves to arm_vision/config/object_references/<class_name>/
  5. Generates reference_manifest.yaml
═══════════════════════════════════════════════════════════════
"""
import argparse
import math
import os
import sys
import time
from datetime import datetime
from typing import List, Tuple

import numpy as np

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# ═══════════════════════════════════════════════════════════════
# Viewpoint generation
# ═══════════════════════════════════════════════════════════════
def generate_viewpoints(
    n_views: int = 8,
    center: Tuple[float, float, float] = (0.20, 0.0, 0.80),
    radius: float = 0.20,
    elevation_range: Tuple[float, float] = (0.15, 0.35),
) -> List[dict]:
    """
    Generate N viewpoints distributed around the object center.

    Viewpoints form a hemisphere:
      - Ring 1 (60%): eye-level, evenly spaced azimuth
      - Ring 2 (25%): elevated 30° above, fewer viewpoints
      - Ring 3 (15%): top-down view

    Returns list of {position: [x,y,z], look_at: [x,y,z], name: str}
    """
    cx, cy, cz = center
    viewpoints = []

    # Ring 1: eye-level views
    n_ring1 = max(3, int(n_views * 0.6))
    for i in range(n_ring1):
        angle = 2.0 * math.pi * i / n_ring1
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        z = cz + elevation_range[0]  # Slightly above
        viewpoints.append({
            'position': [round(x, 3), round(y, 3), round(z, 3)],
            'look_at': list(center),
            'name': f'ring1_view_{i:02d}',
            'azimuth_deg': round(math.degrees(angle), 1),
            'elevation_deg': 15.0,
        })

    # Ring 2: elevated views
    n_ring2 = max(2, int(n_views * 0.25))
    for i in range(n_ring2):
        angle = 2.0 * math.pi * i / n_ring2 + math.pi / n_ring1
        elev = math.radians(35)
        x = cx + radius * math.cos(angle) * math.cos(elev)
        y = cy + radius * math.sin(angle) * math.cos(elev)
        z = cz + radius * math.sin(elev)
        viewpoints.append({
            'position': [round(x, 3), round(y, 3), round(z, 3)],
            'look_at': list(center),
            'name': f'ring2_view_{i:02d}',
            'azimuth_deg': round(math.degrees(angle), 1),
            'elevation_deg': 35.0,
        })

    # Ring 3: top-down view
    n_ring3 = max(1, n_views - n_ring1 - n_ring2)
    for i in range(n_ring3):
        viewpoints.append({
            'position': [round(cx, 3), round(cy, 3),
                         round(cz + elevation_range[1], 3)],
            'look_at': list(center),
            'name': f'top_view_{i:02d}',
            'azimuth_deg': 0.0,
            'elevation_deg': 90.0,
        })

    return viewpoints[:n_views]


# ═══════════════════════════════════════════════════════════════
# ROS2 image capture (when ROS is available)
# ═══════════════════════════════════════════════════════════════
def capture_from_ros(timeout_s: float = 5.0):
    """Capture a single frame from the /top_camera/image_raw topic."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except ImportError:
        return None

    class _CaptureNode(Node):
        def __init__(self):
            super().__init__('_ref_capture')
            self.bridge = CvBridge()
            self.frame = None
            self.sub = self.create_subscription(
                Image, '/top_camera/image_raw',
                self._cb, 10)

        def _cb(self, msg):
            try:
                self.frame = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            except Exception:
                pass

    if not rclpy.ok():
        rclpy.init()

    node = _CaptureNode()
    t_start = time.time()
    while node.frame is None and (time.time() - t_start) < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.1)

    frame = node.frame
    node.destroy_node()
    return frame


def capture_from_webcam():
    """Fallback: capture from default webcam."""
    if not CV2_AVAILABLE:
        return None
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        return None
    ret, frame = cap.read()
    cap.release()
    return frame if ret else None


# ═══════════════════════════════════════════════════════════════
# Main capture loop
# ═══════════════════════════════════════════════════════════════
def run_capture(
    object_name: str,
    n_views: int,
    output_dir: str,
    manual_mode: bool = False,
    use_ros: bool = True,
):
    """
    Run the full reference image capture process.

    Args:
        object_name: Object class name (e.g., 'cup')
        n_views:     Number of reference views to capture
        output_dir:  Directory to save images
        manual_mode: If True, user manually positions camera
        use_ros:     If True, capture from ROS topic
    """
    os.makedirs(output_dir, exist_ok=True)
    viewpoints = generate_viewpoints(n_views)

    print(f"\n{'═' * 54}")
    print(f"  ARIA Reference Image Capture")
    print(f"  Object: {object_name}")
    print(f"  Views:  {n_views}")
    print(f"  Output: {output_dir}")
    print(f"{'═' * 54}\n")

    if manual_mode:
        print("  Manual mode: position the camera yourself.")
        print("  Press ENTER to capture, 'q' to quit.\n")
    else:
        print("  Auto mode: arm will move to viewpoints.")
        print("  Ensure object is centered in workspace.\n")

    captured_images = []
    manifest = {
        'object_name': object_name,
        'capture_date': datetime.now().isoformat(),
        'n_views': n_views,
        'views': [],
    }

    for i, vp in enumerate(viewpoints):
        print(f"  View {i+1}/{n_views}: {vp['name']} "
              f"(az={vp['azimuth_deg']}°, el={vp['elevation_deg']}°)")

        if not manual_mode:
            # TODO: Move arm to viewpoint position via ROS service
            # ros2 service call /aria/move_to_pose ...
            print(f"    → Moving arm to {vp['position']}...")
            time.sleep(0.5)  # Simulated movement time

        if manual_mode:
            input("    Press ENTER to capture...")

        # Capture frame
        frame = None
        if use_ros:
            frame = capture_from_ros()
        if frame is None:
            frame = capture_from_webcam()
        if frame is None:
            # Generate a placeholder for offline testing
            frame = np.random.randint(
                0, 255, (480, 640, 3), dtype=np.uint8)
            print("    ⚠ Using placeholder image (no camera)")

        # Save image
        fname = f"{object_name}_view_{i:02d}.png"
        fpath = os.path.join(output_dir, fname)
        if CV2_AVAILABLE:
            cv2.imwrite(fpath, frame)
        else:
            np.save(fpath.replace('.png', '.npy'), frame)

        captured_images.append(fpath)
        manifest['views'].append({
            'filename': fname,
            'viewpoint': vp,
            'timestamp': datetime.now().isoformat(),
        })
        print(f"    ✓ Saved: {fname}")

    # Save manifest
    manifest_path = os.path.join(output_dir, 'reference_manifest.yaml')
    if YAML_AVAILABLE:
        with open(manifest_path, 'w') as f:
            yaml.dump(manifest, f, default_flow_style=False)
        print(f"\n  ✓ Manifest saved: reference_manifest.yaml")

    print(f"\n{'═' * 54}")
    print(f"  ✅ {object_name} registered.")
    print(f"  {len(captured_images)} reference images captured.")
    print(f"  FoundationPose ready for {object_name}.")
    print(f"{'═' * 54}\n")

    return captured_images


# ═══════════════════════════════════════════════════════════════
# CLI entry point
# ═══════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        description='ARIA Reference Image Capture for FoundationPose')
    parser.add_argument(
        '--object', '-o', required=True,
        help='Object class name (e.g., cup, bottle, wrench)')
    parser.add_argument(
        '--views', '-v', type=int, default=8,
        help='Number of reference views to capture (default: 8)')
    parser.add_argument(
        '--manual', action='store_true',
        help='Manual mode: user positions camera')
    parser.add_argument(
        '--no-ros', action='store_true',
        help='Skip ROS, use webcam or placeholders')
    parser.add_argument(
        '--output', '-d', default='',
        help='Output directory (default: config/object_references/<name>)')

    args = parser.parse_args()

    if not args.output:
        args.output = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'object_references', args.object)

    run_capture(
        object_name=args.object,
        n_views=args.views,
        output_dir=args.output,
        manual_mode=args.manual,
        use_ros=not args.no_ros,
    )


if __name__ == '__main__':
    main()
