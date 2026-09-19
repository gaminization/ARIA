#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Coordinate Transformer
Hybrid approach: primary uses known table geometry,
fallback uses monocular depth estimates.

Diagram:
  Camera (top)
    |
    | ray
    |
  ──●──────────── table (z = TABLE_HEIGHT)
    X,Y found by ray-plane intersection
═══════════════════════════════════════════════════════════════
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


@dataclass
class WorldCoordinate:
    """3D world coordinate with confidence and metadata."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    confidence: float = 0.0
    method: str = "unknown"
    position_uncertainty_m: float = float('inf')


class CoordinateTransformer:
    """
    Transforms pixel coordinates to 3D world coordinates.

    Primary path: ray-plane intersection with known table height.
    Fallback path: monocular depth estimate → 3D projection.
    Fusion: weighted average when both are available.
    """

    # Known table surface height in world frame (optical table surface)
    TABLE_HEIGHT_M = 0.6081

    def __init__(self,
                 camera_matrix: np.ndarray,
                 dist_coeffs: np.ndarray,
                 camera_position: np.ndarray,
                 camera_rotation: np.ndarray):
        """
        Args:
            camera_matrix: 3×3 intrinsic matrix K
                           [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
            dist_coeffs: distortion coefficients [k1, k2, p1, p2, k3]
            camera_position: [x, y, z] camera position in world frame
            camera_rotation: 3×3 rotation matrix (camera → world)
        """
        self.K = camera_matrix
        self.dist = dist_coeffs
        self.cam_pos = camera_position
        self.R_cam_to_world = camera_rotation

    def pixel_to_world(self,
                       px: int, py: int,
                       depth_estimate: Optional[float] = None,
                       object_height: float = 0.0,
                       table_height: Optional[float] = None) -> WorldCoordinate:
        """
        Convert pixel (px, py) to 3D world coordinate.

        Step 1: Undistort pixel → normalized image coordinates
        Step 2: Primary — ray-plane intersection (known Z)
        Step 3: Fallback — monocular depth projection
        Step 4: Fusion (if both available)

        Args:
            px, py: pixel coordinates in the image
            depth_estimate: monocular depth at this pixel (meters, optional)
            object_height: known object height above table (meters)
            table_height: override table height (meters, default TABLE_HEIGHT_M)

        Returns:
            WorldCoordinate with x, y, z, confidence, method
        """
        if table_height is None:
            table_height = self.TABLE_HEIGHT_M

        # ═══════════════════════════════════════════════════
        # STEP 1: Undistort pixel → normalized coordinates
        # ═══════════════════════════════════════════════════
        # OpenCV undistortPoints gives normalized image coords
        # (pixel → camera frame direction, focal-length-independent)
        #
        # Input:  pixel [px, py] in distorted image
        # Output: [px_n, py_n] on normalized image plane (z=1)

        if CV2_AVAILABLE:
            pixel = np.array([[[float(px), float(py)]]], dtype=np.float64)
            undistorted = cv2.undistortPoints(
                pixel, self.K, self.dist
            )
            px_n, py_n = undistorted[0, 0, 0], undistorted[0, 0, 1]
        else:
            # Manual undistortion (no OpenCV)
            fx, fy = self.K[0, 0], self.K[1, 1]
            cx, cy = self.K[0, 2], self.K[1, 2]
            px_n = (px - cx) / fx
            py_n = (py - cy) / fy

        # ═══════════════════════════════════════════════════
        # STEP 2: Primary — known table height (geometric)
        # ═══════════════════════════════════════════════════
        # If object is ON the table, Z_world is known:
        #   Z_world = table_height + object_height
        #
        # Back-project ray from camera through pixel:
        #   ray_dir_cam = [px_n, py_n, 1]  (in camera frame)
        #   ray_dir_world = R_cam @ ray_dir_cam
        #
        # Ray-plane intersection (plane z = Z_world):
        #   ray: P = cam_pos + t * ray_dir_world
        #   plane: P.z = Z_world
        #   → t = (Z_world - cam_pos.z) / ray_dir_world.z
        #   → X_world = cam_pos.x + t * ray_dir_world.x
        #   → Y_world = cam_pos.y + t * ray_dir_world.y

        primary_result = None
        Z_world = table_height + object_height

        # Ray direction in camera frame
        ray_cam = np.array([px_n, py_n, 1.0])

        # Transform to world frame
        ray_world = self.R_cam_to_world @ ray_cam
        ray_world = ray_world / np.linalg.norm(ray_world)

        # Ray-plane intersection
        if abs(ray_world[2]) > 1e-6:
            t = (Z_world - self.cam_pos[2]) / ray_world[2]

            if t > 0:  # Intersection is in front of camera
                X_world = self.cam_pos[0] + t * ray_world[0]
                Y_world = self.cam_pos[1] + t * ray_world[1]

                primary_result = WorldCoordinate(
                    x=float(X_world),
                    y=float(Y_world),
                    z=float(Z_world),
                    confidence=0.95,
                    method="primary_geometric",
                    position_uncertainty_m=0.005,  # ~5mm from geometry
                )

        # ═══════════════════════════════════════════════════
        # STEP 3: Fallback — monocular depth estimate
        # ═══════════════════════════════════════════════════
        # If primary unavailable or depth estimate provided:
        #   3D_cam = [px_n * depth, py_n * depth, depth]
        #   3D_world = R @ 3D_cam + cam_pos

        depth_result = None
        if depth_estimate is not None and depth_estimate > 0.01:
            point_cam = np.array([
                px_n * depth_estimate,
                py_n * depth_estimate,
                depth_estimate
            ])

            point_world = self.R_cam_to_world @ point_cam + self.cam_pos

            depth_result = WorldCoordinate(
                x=float(point_world[0]),
                y=float(point_world[1]),
                z=float(point_world[2]),
                confidence=0.65,
                method="depth_model",
                position_uncertainty_m=0.020,  # ~20mm from monocular
            )

        # ═══════════════════════════════════════════════════
        # STEP 4: Fusion
        # ═══════════════════════════════════════════════════
        # If both available: weighted average by confidence.
        # Higher confidence → more weight.

        if primary_result is not None and depth_result is not None:
            w1 = primary_result.confidence
            w2 = depth_result.confidence
            w_total = w1 + w2

            fused = WorldCoordinate(
                x=(w1 * primary_result.x + w2 * depth_result.x) / w_total,
                y=(w1 * primary_result.y + w2 * depth_result.y) / w_total,
                z=(w1 * primary_result.z + w2 * depth_result.z) / w_total,
                confidence=max(w1, w2),
                method="fused",
                position_uncertainty_m=min(
                    primary_result.position_uncertainty_m,
                    depth_result.position_uncertainty_m
                ),
            )
            return fused

        elif primary_result is not None:
            return primary_result

        elif depth_result is not None:
            return depth_result

        else:
            return WorldCoordinate(
                confidence=0.0,
                method="failed",
                position_uncertainty_m=float('inf'),
            )

    def pixel_to_world_batch(self,
                             pixels: np.ndarray,
                             depths: Optional[np.ndarray] = None,
                             object_height: float = 0.0) -> list:
        """
        Batch transform multiple pixels to world coordinates.

        Args:
            pixels: Nx2 array of (px, py) pixel coordinates
            depths: N array of depth estimates (optional)
            object_height: height above table for all objects

        Returns:
            List of WorldCoordinate
        """
        results = []
        for i in range(len(pixels)):
            depth = depths[i] if depths is not None else None
            result = self.pixel_to_world(
                int(pixels[i, 0]), int(pixels[i, 1]),
                depth_estimate=depth,
                object_height=object_height
            )
            results.append(result)
        return results


def benchmark_coordinate_accuracy(transformer: CoordinateTransformer,
                                  ground_truth_objects: list) -> dict:
    """
    Compare pixel→world estimates vs known ground truth positions.

    Args:
        transformer: configured CoordinateTransformer
        ground_truth_objects: list of dicts with
            'pixel': [px, py], 'world': [x, y, z]

    Returns:
        Dict with xy_error_mm, z_error_mm, method stats
    """
    xy_errors = []
    z_errors = []
    methods_used = {}

    for obj in ground_truth_objects:
        px, py = obj['pixel']
        gt = np.array(obj['world'])

        result = transformer.pixel_to_world(px, py)

        if result.confidence > 0:
            pred = np.array([result.x, result.y, result.z])
            xy_err = np.linalg.norm(pred[:2] - gt[:2]) * 1000  # mm
            z_err = abs(pred[2] - gt[2]) * 1000  # mm
            xy_errors.append(xy_err)
            z_errors.append(z_err)

            methods_used[result.method] = methods_used.get(result.method, 0) + 1

    metrics = {
        'mean_xy_error_mm': np.mean(xy_errors) if xy_errors else float('inf'),
        'mean_z_error_mm': np.mean(z_errors) if z_errors else float('inf'),
        'max_xy_error_mm': max(xy_errors) if xy_errors else float('inf'),
        'max_z_error_mm': max(z_errors) if z_errors else float('inf'),
        'n_objects': len(ground_truth_objects),
        'n_resolved': len(xy_errors),
        'methods_used': methods_used,
    }

    print(f"\nCoordinate Accuracy Benchmark:")
    print(f"  XY error: {metrics['mean_xy_error_mm']:.1f}mm "
          f"(max {metrics['max_xy_error_mm']:.1f}mm)")
    print(f"  Z error:  {metrics['mean_z_error_mm']:.1f}mm "
          f"(max {metrics['max_z_error_mm']:.1f}mm)")
    print(f"  Methods:  {metrics['methods_used']}")

    return metrics


# ═══════════════════════════════════════════════════════════════
# Factory for ARIA top camera
# ═══════════════════════════════════════════════════════════════
def create_top_camera_transformer() -> CoordinateTransformer:
    """
    Create a CoordinateTransformer for the ARIA top camera.
    Uses intrinsics from camera_node and pose from URDF/TF.
    """
    # Intrinsics: Logitech C270
    K = np.array([
        [721.0,   0.0, 640.0],
        [  0.0, 721.0, 360.0],
        [  0.0,   0.0,   1.0],
    ])
    dist = np.array([0.15, -0.08, 0.0, 0.0, 0.0])

    # Camera position: 0.40m forward, 0.80m above table, pointing down
    # From SDF: camera stand at (0.40, 0.0, 0.76+0.80=1.56) looking down
    cam_pos = np.array([0.40, 0.0, 1.56])

    # Camera rotation: pointing straight down (-Z in world = +Z in camera)
    # Camera X = world -X, Camera Y = world -Y, Camera Z = world -Z
    R_cam_to_world = np.array([
        [-1.0,  0.0,  0.0],
        [ 0.0, -1.0,  0.0],
        [ 0.0,  0.0, -1.0],
    ])

    return CoordinateTransformer(K, dist, cam_pos, R_cam_to_world)
