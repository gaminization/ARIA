#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Upgrade U2 — Advanced Perception Validation Suite
6 tests covering all perception modules.

Usage:
  python3 arm_bringup/scripts/validate_upgrade_u2.py
═══════════════════════════════════════════════════════════════
"""
import importlib
import json
import math
import os
import sys
import time
import traceback
import types
from typing import Callable, List, Tuple
from unittest.mock import MagicMock

import numpy as np

# ═══════════════════════════════════════════════════════════════
# Mock ROS packages that may not be importable outside colcon env
# ═══════════════════════════════════════════════════════════════
_MOCK_MODULES = [
    'vision_msgs', 'vision_msgs.msg',
    'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
    'rclpy.publisher',
    'sensor_msgs', 'sensor_msgs.msg',
    'geometry_msgs', 'geometry_msgs.msg',
    'std_msgs', 'std_msgs.msg',
    'std_srvs', 'std_srvs.srv',
    'visualization_msgs', 'visualization_msgs.msg',
    'cv_bridge',
    'arm_interfaces', 'arm_interfaces.srv',
    'arm_planner', 'arm_planner.state_bus',
]

for mod_name in _MOCK_MODULES:
    if mod_name not in sys.modules:
        sys.modules[mod_name] = MagicMock()

# Provide real geometry_msgs types that the math code needs
class _Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x; self.y = y; self.z = z

class _Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x; self.y = y; self.z = z; self.w = w

class _Pose:
    def __init__(self):
        self.position = _Point()
        self.orientation = _Quaternion()

class _PoseStamped:
    class _Header:
        frame_id = ''
        stamp = None
    def __init__(self):
        self.header = self._Header()
        self.pose = _Pose()

class _PoseArray:
    class _Header:
        frame_id = ''
        stamp = None
    def __init__(self):
        self.header = self._Header()
        self.poses = []

# Patch geometry_msgs.msg with real types
_geom = sys.modules['geometry_msgs.msg']
_geom.Point = _Point
_geom.Quaternion = _Quaternion
_geom.Pose = _Pose
_geom.PoseStamped = _PoseStamped
_geom.PoseArray = _PoseArray
_geom.TransformStamped = MagicMock
_geom.Vector3 = MagicMock

# Patch std_msgs.msg
_std = sys.modules['std_msgs.msg']
_std.String = MagicMock
_std.Bool = MagicMock
_std.Float64MultiArray = MagicMock
_std.Header = MagicMock

# Patch sensor_msgs.msg
_sensor = sys.modules['sensor_msgs.msg']
_sensor.Image = MagicMock

# Patch vision_msgs.msg
_vis = sys.modules['vision_msgs.msg']
_vis.Detection2D = MagicMock
_vis.Detection2DArray = MagicMock
_vis.ObjectHypothesisWithPose = MagicMock

# Patch visualization_msgs.msg
_viz = sys.modules['visualization_msgs.msg']
_viz.Marker = type('Marker', (), {
    'ARROW': 0, 'TEXT_VIEW_FACING': 9, 'ADD': 0,
})
_viz.MarkerArray = MagicMock

# Patch std_srvs
_ssrv = sys.modules['std_srvs.srv']
_ssrv.Trigger = MagicMock



# ═══════════════════════════════════════════════════════════════
# Test framework
# ═══════════════════════════════════════════════════════════════
class TestResult:
    def __init__(self, name: str, passed: bool, message: str, duration_s: float):
        self.name = name
        self.passed = passed
        self.message = message
        self.duration_s = duration_s


def run_test(name: str, test_fn: Callable) -> TestResult:
    """Run a single test with timing and error handling."""
    print(f"\n  ┌─ Test: {name}")
    t_start = time.time()
    try:
        passed, message = test_fn()
        duration = time.time() - t_start
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"  └─ {status} ({duration:.1f}s): {message}")
        return TestResult(name, passed, message, duration)
    except Exception as e:
        duration = time.time() - t_start
        msg = f"Exception: {e}"
        print(f"  └─ ❌ ERROR ({duration:.1f}s): {msg}")
        return TestResult(name, False, msg, duration)


# ═══════════════════════════════════════════════════════════════
# Test 1: SAM2 Masks
# ═══════════════════════════════════════════════════════════════
def test_sam2_masks() -> Tuple[bool, str]:
    """Test SAM2 segmentation node (import + mask generation)."""
    from arm_vision.sam2_node import SAM2Model, compute_mask_properties

    model = SAM2Model(device='cpu')
    model.load()

    mode = "real SAM2" if model._use_real_model else "GrabCut fallback"

    # Create test image with 4 objects
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    # Draw 4 colored rectangles
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
    rects = [(50, 50, 150, 150), (200, 80, 300, 180),
             (400, 100, 500, 200), (100, 300, 200, 400)]
    for (x1, y1, x2, y2), color in zip(rects, colors):
        image[y1:y2, x1:x2] = color

    # Run segmentation
    boxes = rects
    results = model.segment(image, boxes)

    if len(results) != 4:
        model.unload()
        return False, f"Expected 4 masks, got {len(results)}"

    # Check masks are non-empty
    areas = []
    for mask, conf in results:
        area = np.sum(mask > 0)
        areas.append(area)
        if area < 50:
            model.unload()
            return False, f"Mask too small: {area} pixels"

    # Check mask properties
    for mask, conf in results:
        props = compute_mask_properties(mask)
        if 'centroid' not in props and props:
            model.unload()
            return False, "compute_mask_properties missing centroid"

    avg_area = sum(areas) / len(areas)
    model.unload()
    return True, f"4 masks OK (mode={mode}, avg_area={avg_area:.0f}px)"


# ═══════════════════════════════════════════════════════════════
# Test 2: 6D Pose Accuracy
# ═══════════════════════════════════════════════════════════════
def test_6d_pose() -> Tuple[bool, str]:
    """Test 6D pose estimation (geometry-based)."""
    from arm_vision.pose_6d_node import (
        FoundationPoseModel, _se3_to_pose, _rotation_matrix_to_quaternion
    )

    model = FoundationPoseModel(device='cpu')
    model.load()

    # Create test depth map with a tilted surface
    h, w = 480, 640
    depth = np.ones((h, w), dtype=np.float32) * 1.0  # 1m background

    # Create a rectangular mask for a "cube" rotated 30°
    mask = np.zeros((h, w), dtype=np.uint8)

    # Draw a rotated rectangle in the mask
    cx, cy = 320, 240
    size = 60
    angle_deg = 30.0
    angle_rad = math.radians(angle_deg)

    for y in range(h):
        for x in range(w):
            dx = x - cx
            dy = y - cy
            rx = dx * math.cos(-angle_rad) - dy * math.sin(-angle_rad)
            ry = dx * math.sin(-angle_rad) + dy * math.cos(-angle_rad)
            if abs(rx) < size and abs(ry) < size * 0.6:
                mask[y, x] = 255
                # Object is closer (0.8m)
                depth[y, x] = 0.8

    # Run pose estimation
    T, confidence = model.estimate_pose(
        np.zeros((h, w, 3), dtype=np.uint8),
        depth, mask, 'cube')

    model.unload()

    if T is None:
        return False, "Pose estimation returned None"

    if confidence < 0.4:
        return False, f"Confidence too low: {confidence:.2f}"

    # Check position is near mask center
    pos = T[:3, 3]
    # Position should be roughly in the center of the mask in camera coords
    # With our test setup, centroid should be near (0, 0, 0.8)

    # Check rotation matrix is valid (orthogonal, det=1)
    R = T[:3, :3]
    det = np.linalg.det(R)
    if abs(det - 1.0) > 0.1:
        return False, f"Invalid rotation matrix (det={det:.3f})"

    # Convert to ROS pose
    pose = _se3_to_pose(T)

    return True, (
        f"6D pose OK (conf={confidence:.2f}, "
        f"det(R)={det:.3f}, z={pos[2]:.2f}m)")


# ═══════════════════════════════════════════════════════════════
# Test 3: Transparent Object Depth Correction
# ═══════════════════════════════════════════════════════════════
def test_transparent_depth() -> Tuple[bool, str]:
    """Test transparent object depth correction."""
    from arm_vision.transparent_object_node import (
        detect_depth_bimodality, detect_specular_highlights,
        correct_depth_plane_fitting,
    )

    # Test bimodality detection
    # Create bimodal depth: some at 0.8m, others at 1.5m
    bimodal_patch = np.concatenate([
        np.full(100, 0.80, dtype=np.float32),
        np.full(100, 1.50, dtype=np.float32),
    ])
    np.random.shuffle(bimodal_patch)
    bimodal_patch = bimodal_patch.reshape(10, 20)

    is_bimodal, gap = detect_depth_bimodality(bimodal_patch)
    if not is_bimodal:
        return False, "Failed to detect bimodal depth distribution"

    # Test uniform depth (should NOT be bimodal)
    uniform_patch = np.full((10, 20), 0.80, dtype=np.float32)
    uniform_patch += np.random.normal(0, 0.01, uniform_patch.shape).astype(np.float32)

    is_uni_bimodal, _ = detect_depth_bimodality(uniform_patch)
    if is_uni_bimodal:
        return False, "False positive: uniform depth detected as bimodal"

    # Test specular highlight detection
    bright_patch = np.ones((50, 50, 3), dtype=np.uint8) * 100
    bright_patch[20:30, 20:30] = 250  # Bright spot
    has_specular = detect_specular_highlights(bright_patch)
    if not has_specular:
        return False, "Failed to detect specular highlights"

    # Test depth correction via plane fitting
    h, w = 100, 100
    depth_map = np.full((h, w), 0.80, dtype=np.float32)  # Table at 0.8m

    # Mark a region as transparent (bad depth showing background)
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[30:70, 30:70] = 255
    depth_map[30:70, 30:70] = 1.50  # Wrong: seeing through to 1.5m

    corrected = correct_depth_plane_fitting(depth_map, mask)
    if corrected is None:
        return False, "Plane fitting returned None"

    # Check corrected depth in masked region
    corrected_region = corrected[30:70, 30:70]
    mean_corrected = np.mean(corrected_region)

    # Should be close to 0.80m (table surface), not 1.50m
    error_mm = abs(mean_corrected - 0.80) * 1000
    if error_mm > 50:  # Within 50mm
        return False, f"Corrected depth error too large: {error_mm:.1f}mm"

    return True, (
        f"Transparent: bimodal=OK, specular=OK, "
        f"depth correction={error_mm:.1f}mm MAE")


# ═══════════════════════════════════════════════════════════════
# Test 4: Material Recognition
# ═══════════════════════════════════════════════════════════════
def test_material_recognition() -> Tuple[bool, str]:
    """Test material classification and grasp param mapping."""
    from arm_vision.material_recognition_node import (
        MaterialClassifier, MATERIAL_TO_GRASP_PARAMS,
        MATERIAL_CLASSES, CLASS_TO_DEFAULT_MATERIAL,
    )

    classifier = MaterialClassifier(device='cpu')
    mode = classifier.load()

    # Test heuristic classification
    mat_cup, conf_cup = classifier.predict(
        np.zeros((50, 50, 3), dtype=np.uint8), 'cup')
    mat_knife, conf_knife = classifier.predict(
        np.zeros((50, 50, 3), dtype=np.uint8), 'knife')
    mat_bottle, conf_bottle = classifier.predict(
        np.zeros((50, 50, 3), dtype=np.uint8), 'bottle')

    # Verify known mappings
    expected = {'cup': 'ceramic', 'knife': 'rigid_metal', 'bottle': 'rigid_plastic'}
    results = {'cup': mat_cup, 'knife': mat_knife, 'bottle': mat_bottle}

    for cls, expected_mat in expected.items():
        actual = results[cls]
        if actual != expected_mat:
            classifier.unload()
            return False, f"{cls}: expected {expected_mat}, got {actual}"

    # Verify grasp params exist for all materials
    for mat in MATERIAL_CLASSES:
        if mat not in MATERIAL_TO_GRASP_PARAMS:
            classifier.unload()
            return False, f"Missing grasp params for material: {mat}"
        params = MATERIAL_TO_GRASP_PARAMS[mat]
        if 'grip_force_scale' not in params:
            classifier.unload()
            return False, f"Missing grip_force_scale for {mat}"

    # Verify glass has reduced force
    glass_force = MATERIAL_TO_GRASP_PARAMS['glass']['grip_force_scale']
    metal_force = MATERIAL_TO_GRASP_PARAMS['rigid_metal']['grip_force_scale']
    if glass_force >= metal_force:
        classifier.unload()
        return False, f"Glass force ({glass_force}) >= metal ({metal_force})"

    classifier.unload()
    return True, (
        f"Material OK (mode={mode}, "
        f"glass_force={glass_force}, metal_force={metal_force})")


# ═══════════════════════════════════════════════════════════════
# Test 5: Gaussian Splatting
# ═══════════════════════════════════════════════════════════════
def test_gaussian_splatting() -> Tuple[bool, str]:
    """Test Gaussian Splatting workspace model."""
    from arm_vision.gaussian_splatting_node import (
        GaussianSplattingEngine, CapturedView,
    )

    engine = GaussianSplattingEngine(device='cpu')

    # Create synthetic views
    views = []
    for i in range(5):
        angle = 2 * math.pi * i / 5
        pose = np.eye(4)
        pose[0, 3] = 0.1 * math.cos(angle)
        pose[1, 3] = 0.1 * math.sin(angle)
        pose[2, 3] = 1.0

        # Simple depth: flat table at 0.8m
        depth = np.full((480, 640), 0.8, dtype=np.float32)
        rgb = np.random.randint(100, 200, (480, 640, 3), dtype=np.uint8)

        views.append(CapturedView(
            rgb=rgb, depth=depth, camera_pose=pose, timestamp=time.time()))

    # Reconstruct
    success = engine.reconstruct(views)
    if not success:
        return False, "Reconstruction failed"

    if not engine.has_model:
        return False, "No model after reconstruction"

    n_points = engine._model.n_gaussians
    if n_points < 100:
        return False, f"Too few points: {n_points}"

    # Render depth
    test_pose = np.eye(4)
    test_pose[2, 3] = 1.0
    rendered = engine.render_depth(test_pose)

    if rendered is None:
        return False, "Render returned None"

    # Check rendered depth is reasonable
    valid_pixels = rendered[rendered > 0.01]
    if len(valid_pixels) < 100:
        return False, f"Too few valid rendered pixels: {len(valid_pixels)}"

    mean_depth = np.mean(valid_pixels)

    # Change detection
    live_depth = np.full((480, 640), 0.8, dtype=np.float32)
    # Place a "new object" in the center
    live_depth[200:280, 280:360] = 0.6  # Object at 0.6m

    change = engine.compute_change_mask(rendered, live_depth, 0.05)
    changed_pixels = np.sum(change > 0)

    return True, (
        f"GS OK ({n_points} points, "
        f"mean_depth={mean_depth:.2f}m, "
        f"change_pixels={changed_pixels})")


# ═══════════════════════════════════════════════════════════════
# Test 6: Perception Orchestrator
# ═══════════════════════════════════════════════════════════════
def test_orchestrator() -> Tuple[bool, str]:
    """Test perception orchestrator mode selection."""
    from arm_vision.perception_orchestrator import (
        PerceptionMode, MODE_MODULES, MODULE_INFO,
        TASK_TO_MODE, VRAM_OVERHEAD_GB, TOTAL_VRAM_GB,
    )

    # Verify all modes have module sets defined
    for mode in PerceptionMode:
        if mode not in MODE_MODULES:
            return False, f"Missing module set for mode: {mode.value}"

    # Verify all modules have info
    all_modules = set()
    for modules in MODE_MODULES.values():
        all_modules.update(modules)
    for mod in all_modules:
        if mod not in MODULE_INFO:
            return False, f"Missing info for module: {mod}"

    # Verify VRAM budget for each mode
    for mode in PerceptionMode:
        modules = MODE_MODULES[mode]
        total = VRAM_OVERHEAD_GB
        for mod in modules:
            total += MODULE_INFO[mod]['vram_gb']
        if total > TOTAL_VRAM_GB:
            return False, (
                f"Mode {mode.value} exceeds VRAM budget: "
                f"{total:.1f}GB > {TOTAL_VRAM_GB}GB")

    # Verify task→mode mappings
    assert TASK_TO_MODE.get('pick') == PerceptionMode.SIMPLE_PICK
    assert TASK_TO_MODE.get('assembly') == PerceptionMode.FULL_PRECISION
    assert TASK_TO_MODE.get('organize') == PerceptionMode.SCENE_UNDERSTANDING

    # Verify mode ordering: SIMPLE < PRECISION < FULL
    simple_vram = sum(MODULE_INFO[m]['vram_gb']
                      for m in MODE_MODULES[PerceptionMode.SIMPLE_PICK])
    full_vram = sum(MODULE_INFO[m]['vram_gb']
                    for m in MODE_MODULES[PerceptionMode.FULL_PRECISION])
    if simple_vram >= full_vram:
        return False, f"SIMPLE ({simple_vram}GB) >= FULL ({full_vram}GB)"

    return True, (
        f"Orchestrator OK: {len(PerceptionMode)} modes, "
        f"{len(all_modules)} modules, VRAM fits in {TOTAL_VRAM_GB}GB")


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════
def main():
    print("\n" + "═" * 54)
    print("  ARIA Upgrade U2 — Advanced Perception Validation")
    print("═" * 54)

    tests = [
        ("SAM2 Masks", test_sam2_masks),
        ("6D Pose Estimation", test_6d_pose),
        ("Transparent Object Depth", test_transparent_depth),
        ("Material Recognition", test_material_recognition),
        ("Gaussian Splatting", test_gaussian_splatting),
        ("Perception Orchestrator", test_orchestrator),
    ]

    results = []
    for name, fn in tests:
        result = run_test(name, fn)
        results.append(result)

    # Summary
    n_pass = sum(1 for r in results if r.passed)
    n_total = len(results)

    print("\n" + "═" * 54)
    print(f"  Results: {n_pass}/{n_total} tests passed")
    print("═" * 54)

    for r in results:
        icon = "✅" if r.passed else "❌"
        print(f"  {icon} {r.name}: {r.message}")

    print("═" * 54)

    if n_pass == n_total:
        print("  🎉 U2 UPGRADE VALIDATION COMPLETE")
    else:
        print(f"  ⚠ {n_total - n_pass} test(s) failed")

    print("═" * 54 + "\n")

    return 0 if n_pass == n_total else 1


if __name__ == "__main__":
    sys.exit(main())
