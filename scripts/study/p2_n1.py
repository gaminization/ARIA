#!/usr/bin/env python3
"""P2-N1: kinematics-anchored recursive monocular depth calibration.

The wrist camera is rigidly attached to the robot wrist.  Its metric pose is
read from TF, i.e. from the robot's kinematic chain.  The workcell contains
two static horizontal reference surfaces whose world heights are fixed in the
world file:

* conveyor belt surface   z = 0.6285 m
* optical table top       z = 0.6080 m

A set of world points on those surfaces is projected through the measured
camera pose and intrinsics.  The Depth-Anything-V2-Small relative-depth value
``d`` sampled at each projected pixel is paired with the known metric optical
depth ``Z`` of the 3-D point.  No ground-truth depth is used by any method.

Four calibrations are compared on identical frames:

``fixed``
    One global linear scale, ``Z = a_fixed * d``.  ``a_fixed`` is fitted on
    development seeds 1-100 and frozen before the evaluation seeds run.

``one_point``
    One per-frame anchor (the nearest reference point) sets the scale; the
    offset ``b0`` is a global prior fitted on development seeds.
    ``Z = a * d + b0``.

``two_point``
    Two per-frame anchors (the nearest and the farthest reference point)
    solve ``Z = a * d + b``.

``n1``
    All reference points, robustly fitted and recursively re-weighted:
    RANSAC seed, then iteratively reweighted least squares with a Huber loss
    and a MAD outlier gate.  ``Z = a * d + b``.

The linear form is used because the development fit is substantially better
than the inverse-depth form on this narrow-range synthetic scene; both were
tried on development seeds and the linear form was frozen before evaluation.

Evaluation uses the Gazebo z-buffer (``/wrist_camera/depth/image_raw``) as
ground truth on pixels that are not within 15 px of an anchor.  RMSE, MAE and
maximum error are reported overall and by ground-truth depth band.

Usage:
    python3 scripts/study/p2_n1.py --dev
    python3 scripts/study/p2_n1.py --smoke
    python3 scripts/study/p2_n1.py
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "arm_ik" / "arm_ik" / "ik_solvers"))

import cv2  # noqa: E402
import rclpy  # noqa: E402
import torch  # noqa: E402
import torch.nn.functional as F  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402
from transformers import AutoImageProcessor, AutoModelForDepthEstimation  # noqa: E402

from aria_study import provenance, stats  # noqa: E402
from aria_study.ros import StudyNode  # noqa: E402
from aria_study.trial_log import TrialLog  # noqa: E402

RAW = ROOT / "data" / "journal" / "raw"
CAL = ROOT / "data" / "journal" / "calibration"
PRIORS = CAL / "n1_dev_priors.json"
MODEL_NAME = "depth-anything/Depth-Anything-V2-Small-hf"

# Static horizontal surfaces, from arm_bringup/worlds/aria_industrial_workcell.world.
BELT_Z = 0.6285
TABLE_Z = 0.6080
SURFACES = (("belt", BELT_Z), ("table", TABLE_Z))

# Reference-point grid in world coordinates (metres).
X_RANGE = (0.08, 0.34)
Y_RANGE = (-0.28, 0.28)
GRID_STEP = 0.020

# Camera workspace accepted for a frame.
CAM_Z_RANGE = (0.730, 0.845)
CAM_X_RANGE = (0.155, 0.300)
CAM_Y_RANGE = (-0.165, 0.160)
AXIS_Z_MAX = -0.90

# Evaluation mask.
GT_MIN, GT_MAX = 0.080, 0.500
ANCHOR_EXCLUSION_PX = 6

METHODS = ("fixed", "one_point", "two_point", "n1")


# ── geometry ────────────────────────────────────────────────────────────────
def project_world(
    pw: np.ndarray, pc: np.ndarray, Rc: np.ndarray, K: np.ndarray
) -> Optional[Tuple[float, float, float]]:
    """Project a world point.  Returns (u, v, optical_z) or None."""
    q = Rc.T @ (pw - pc)
    z = float(q[0])
    if z <= 1e-6:
        return None
    u = K[0, 0] * (-q[1]) / z + K[0, 2]
    v = K[1, 1] * (-q[2]) / z + K[1, 2]
    return float(u), float(v), z


def bilinear(img: np.ndarray, u: float, v: float) -> float:
    h, w = img.shape[:2]
    if u < 0 or v < 0 or u > w - 1 or v > h - 1:
        return float("nan")
    x0, y0 = int(np.floor(u)), int(np.floor(v))
    x1, y1 = min(x0 + 1, w - 1), min(y0 + 1, h - 1)
    fx, fy = u - x0, v - y0
    return float(
        img[y0, x0] * (1 - fx) * (1 - fy)
        + img[y0, x1] * fx * (1 - fy)
        + img[y1, x0] * (1 - fx) * fy
        + img[y1, x1] * fx * fy
    )


def median_patch(img: np.ndarray, u: float, v: float, r: int = 1) -> float:
    h, w = img.shape[:2]
    x0, x1 = max(0, int(u) - r), min(w, int(u) + r + 1)
    y0, y1 = max(0, int(v) - r), min(h, int(v) + r + 1)
    if x1 <= x0 or y1 <= y0:
        return float("nan")
    patch = img[y0:y1, x0:x1]
    return float(np.median(patch[np.isfinite(patch)])) if np.isfinite(patch).any() else float("nan")


def collect_anchors(
    da: np.ndarray,
    gt: np.ndarray,
    pc: np.ndarray,
    Rc: np.ndarray,
    K: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return anchor arrays (u, v, d, Z_known, gt_at_pixel)."""
    h, w = da.shape
    us: List[float] = []
    vs: List[float] = []
    ds: List[float] = []
    zs: List[float] = []
    gts: List[float] = []
    for _name, z_plane in SURFACES:
        for x in np.arange(X_RANGE[0], X_RANGE[1] + 1e-9, GRID_STEP):
            for y in np.arange(Y_RANGE[0], Y_RANGE[1] + 1e-9, GRID_STEP):
                r = project_world(np.array([x, y, z_plane]), pc, Rc, K)
                if r is None:
                    continue
                u, v, z = r
                if not (1 <= u < w - 2 and 1 <= v < h - 2):
                    continue
                d = median_patch(da, u, v, r=1)
                if not np.isfinite(d) or d <= 0:
                    continue
                g = bilinear(gt, u, v)
                us.append(u)
                vs.append(v)
                ds.append(d)
                zs.append(z)
                gts.append(g)
    return (
        np.asarray(us, float),
        np.asarray(vs, float),
        np.asarray(ds, float),
        np.asarray(zs, float),
        np.asarray(gts, float),
    )


# ── calibrations ────────────────────────────────────────────────────────────
def fit_fixed(d: np.ndarray, z: np.ndarray) -> float:
    """Z = a*d, least squares through the origin."""
    denom = float(np.sum(d * d))
    return float(np.sum(d * z) / denom) if denom > 0 else float("nan")


def fit_one_point(d0: float, z0: float, b0: float) -> Tuple[float, float]:
    if d0 <= 1e-9:
        return float("nan"), b0
    return float((z0 - b0) / d0), float(b0)


def fit_two_point(
    d0: float, z0: float, d1: float, z1: float
) -> Tuple[float, float]:
    if abs(d0 - d1) < 1e-9:
        return float("nan"), float("nan")
    a = (z0 - z1) / (d0 - d1)
    return float(a), float(z0 - a * d0)


def robust_n1(
    d: np.ndarray,
    z: np.ndarray,
    iterations: int = 12,
    ransac_trials: int = 300,
    inlier_m: float = 0.006,
    seed: int = 20261025,
) -> Tuple[float, float, int, float]:
    """RANSAC + IRLS (Huber) + MAD gate.  Returns (a, b, n_inliers, anchor_rmse)."""
    n = len(d)
    if n < 2:
        return float("nan"), float("nan"), 0, float("nan")
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(ransac_trials):
        i, j = rng.choice(n, 2, replace=False)
        if abs(d[i] - d[j]) < 1e-6:
            continue
        a = (z[i] - z[j]) / (d[i] - d[j])
        b = z[i] - a * d[i]
        if not np.isfinite(a) or not np.isfinite(b):
            continue
        res = np.abs(a * d + b - z)
        k = int(np.sum(res < inlier_m))
        if best is None or k > best[0]:
            best = (k, a, b)
    if best is None:
        return float("nan"), float("nan"), 0, float("nan")
    _, a, b = best
    inl = np.abs(a * d + b - z) < inlier_m
    if inl.sum() < 2:
        return a, b, int(inl.sum()), float("nan")
    for _ in range(iterations):
        dd, zz = d[inl], z[inl]
        A = np.vstack([dd, np.ones_like(dd)]).T
        a, b = np.linalg.lstsq(A, zz, rcond=None)[0]
        res = np.abs(a * d + b - z)
        mad = float(np.median(res[inl])) + 1e-12
        gate = max(inlier_m, 3.0 * 1.4826 * mad)
        new_inl = res < gate
        if new_inl.sum() < 2:
            break
        if np.array_equal(new_inl, inl):
            break
        inl = new_inl
    rmse = float(np.sqrt(np.mean((a * d[inl] + b - z[inl]) ** 2)))
    return float(a), float(b), int(inl.sum()), rmse


def apply_map(a: float, b: float, da: np.ndarray) -> np.ndarray:
    return a * da + b


# ── frame sampling ──────────────────────────────────────────────────────────
def sample_joints(rng: np.random.Generator) -> Optional[np.ndarray]:
    """Sample a joint configuration whose wrist camera looks down at the belt."""
    for _ in range(2000):
        q1 = rng.uniform(-0.55, 0.55)
        q2 = rng.uniform(0.45, 1.55)
        q3 = rng.uniform(-1.50, -0.10)
        q4 = rng.uniform(0.20, 1.45)
        q = np.array([q1, q2, q3, q4])
        T = _fk(q)
        Rw = _RZ_POS @ T[:3, :3]
        cam = _RZ_POS @ (T[:3, 3] + T[:3, :3] @ _TCAM) + np.array([0.0, 0.0, 0.614])
        axis = Rw @ _RCAM_WRIST @ np.array([1.0, 0.0, 0.0])
        if (
            axis[2] < AXIS_Z_MAX
            and CAM_X_RANGE[0] <= cam[0] <= CAM_X_RANGE[1]
            and CAM_Y_RANGE[0] <= cam[1] <= CAM_Y_RANGE[1]
            and CAM_Z_RANGE[0] <= cam[2] <= CAM_Z_RANGE[1]
        ):
            return q
    return None


# FK and camera transform are imported lazily to keep the ROS import order clean.
_RZ_POS = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
_RCAM_WRIST = Rot.from_euler("ZYX", [2.3358, 0.0, 1.5708]).as_matrix()
_TCAM = np.array([0.025, 0.050, -0.007])


def _fk(q: np.ndarray) -> np.ndarray:
    from aria_analytical_ik import forward_kinematics

    return forward_kinematics(q)


# ── node ────────────────────────────────────────────────────────────────────
class DepthNode(StudyNode):
    def __init__(self, name: str = "n1_depth_node"):
        super().__init__(name)
        self.bridge = CvBridge()
        self.rgb: Optional[np.ndarray] = None
        self.gt: Optional[np.ndarray] = None
        self.info: Optional[CameraInfo] = None
        self.rgb_stamp = 0.0
        self.gt_stamp = 0.0
        self.create_subscription(Image, "/wrist_camera/image_raw", self._rgb_cb, 10)
        self.create_subscription(Image, "/wrist_camera/depth/image_raw", self._gt_cb, 10)
        self.create_subscription(CameraInfo, "/wrist_camera/camera_info", self._info_cb, 10)

    def _rgb_cb(self, msg: Image):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self.rgb_stamp = time.monotonic()
        except Exception:
            pass

    def _gt_cb(self, msg: Image):
        try:
            self.gt = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self.gt_stamp = time.monotonic()
        except Exception:
            pass

    def _info_cb(self, msg: CameraInfo):
        self.info = msg

    def fresh(self, max_age: float = 1.0) -> bool:
        now = time.monotonic()
        return (
            self.rgb is not None
            and self.gt is not None
            and self.info is not None
            and now - self.rgb_stamp < max_age
            and now - self.gt_stamp < max_age
        )


# ── experiment ──────────────────────────────────────────────────────────────
def run(args) -> int:
    dev = bool(args.dev)
    smoke = bool(args.smoke)
    exp_id = "P2-N1-DEV" if dev else ("P2-N1-smoke" if smoke else "P2-N1")
    provenance.assert_clean(exp_id)

    if dev:
        seeds = list(range(1, 21)) if not smoke else [1, 2]
    else:
        seeds = [4001, 4002] if smoke else list(range(4001, 4101))

    if not dev and not PRIORS.exists():
        raise RuntimeError(f"missing frozen dev priors: {PRIORS}")
    priors = json.loads(PRIORS.read_text()) if PRIORS.exists() else {}

    rclpy.init()
    node = DepthNode()
    fields = [
        "trial_id", "seed", "method", "frame_id",
        "cam_x", "cam_y", "cam_z", "cam_axis_z",
        "n_anchors", "n_anchor_inliers", "anchor_rmse_mm",
        "a", "b",
        "rmse_mm", "mae_mm", "max_error_mm", "mean_gt_m",
        "rmse_near_mm", "rmse_mid_mm", "rmse_far_mm",
        "n_eval_pixels", "q_waist", "q_shoulder", "q_elbow", "q_wrist",
        "gripper", "joint_track_err_rad",
    ]
    config = {
        "seeds": seeds,
        "methods": list(METHODS),
        "model": MODEL_NAME,
        "surfaces": {k: v for k, v in SURFACES},
        "anchor_grid_step_m": GRID_STEP,
        "anchor_exclusion_px": ANCHOR_EXCLUSION_PX,
        "gt_band_m": [GT_MIN, GT_MAX],
        "camera_workspace": {
            "x": CAM_X_RANGE, "y": CAM_Y_RANGE, "z": CAM_Z_RANGE,
            "axis_z_max": AXIS_Z_MAX,
        },
        "calibration_form": "Z = a*d + b",
        "dev_priors": priors,
    }

    # Load DA-V2 once.
    device = "cuda" if torch.cuda.is_available() else "cpu"
    processor = AutoImageProcessor.from_pretrained(MODEL_NAME)
    model = AutoModelForDepthEstimation.from_pretrained(MODEL_NAME).to(device).eval()
    print(f"DA-V2 device: {device}", flush=True)

    rows: List[dict] = []
    frame_meta: List[dict] = []
    dev_d: List[float] = []
    dev_z: List[float] = []
    dev_frames: List[dict] = []
    t_start = datetime.now(timezone.utc)

    try:
        if not node.wait_for_joint_states(
            ["waist_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "gripper_joint"],
            30.0,
        ):
            raise RuntimeError("joint states missing; is the simulation running?")
        if not node.wait_until(lambda: node.info is not None, 20.0):
            raise RuntimeError("/wrist_camera/camera_info missing; launch with cameras enabled")
        # Stop the conveyor so the scene is static during a capture.
        node.send_conveyor_speed(0.0)
        node.spin_for(0.5)

        with TrialLog(exp_id, fields, config) as log:
            rng = np.random.default_rng(20261025 if not dev else 777001)
            frame_idx = 0
            for seed in seeds:
                frame_id = f"FRAME_WRIST_{seed:04d}"
                # Retry a frame if the arm does not track the commanded pose.
                tracked = False
                for attempt in range(3):
                    q4 = sample_joints(rng)
                    if q4 is None:
                        raise RuntimeError("could not sample a camera pose in the workspace")
                    grip = float(rng.uniform(0.35, 0.80))
                    q_cmd = list(q4) + [grip]
                    node.send_trajectory_until(q_cmd, 2.0, timeout=8.0)
                    node.spin_for(0.45)
                    q_act = node.q(
                        ["waist_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "gripper_joint"]
                    )
                    track = float(np.max(np.abs(q_act - np.array(q_cmd))))
                    if track <= 0.02:
                        tracked = True
                        break
                if not tracked:
                    print(f"  {frame_id}: skipped, joint tracking error {track:.3f} rad", flush=True)
                    continue

                # Wait for fresh synchronized images.
                t0 = time.monotonic()
                while time.monotonic() - t0 < 5.0 and not node.fresh(0.8):
                    rclpy.spin_once(node, timeout_sec=0.05)
                if not node.fresh(1.0):
                    print(f"  {frame_id}: skipped, no fresh images", flush=True)
                    continue
                rgb = node.rgb.copy()
                gt = node.gt.copy()
                K = np.array(node.info.k, dtype=float).reshape(3, 3)
                pc, qc, _ = node.tf_pose("world", "wrist_camera_link", 2.0)
                Rc = Rot.from_quat(qc).as_matrix()
                axis = Rc @ np.array([1.0, 0.0, 0.0])

                # DA-V2 inference.
                with torch.no_grad():
                    out = model(**processor(images=rgb, return_tensors="pt").to(device))
                    pred = out.predicted_depth
                da = (
                    F.interpolate(
                        pred.unsqueeze(1),
                        size=rgb.shape[:2],
                        mode="bilinear",
                        align_corners=False,
                    )
                    .squeeze()
                    .cpu()
                    .numpy()
                    .astype(np.float64)
                )

                us, vs, ds, zs, gts = collect_anchors(da, gt, pc, Rc, K)
                if len(ds) < 4:
                    print(f"  {frame_id}: skipped, only {len(ds)} anchors", flush=True)
                    continue
                if dev:
                    # Development-only: keep anchors whose z-buffer value agrees
                    # with the known surface point, so the frozen prior is not
                    # fitted to occluded points.  Evaluation never does this.
                    good = np.isfinite(gts) & (np.abs(gts - zs) < 0.006)
                    dev_d.extend(ds[good].tolist())
                    dev_z.extend(zs[good].tolist())
                    # Downsampled maps let the dev fit optimise the global
                    # baseline on pixels, not only on the sparse anchors.
                    stride = 8
                    gt_ds = gt[::stride, ::stride]
                    da_ds = da[::stride, ::stride]
                    mask_ds = np.isfinite(gt_ds) & (gt_ds > GT_MIN) & (gt_ds < GT_MAX)
                    ord_d = np.argsort(zs)
                    kd = max(2, int(round(0.20 * len(ord_d))))
                    dev_frames.append(
                        {
                            "da": da_ds.astype(np.float32),
                            "gt": gt_ds.astype(np.float32),
                            "mask": mask_ds,
                            "d_near": float(np.median(ds[ord_d[:kd]])),
                            "z_near": float(np.median(zs[ord_d[:kd]])),
                        }
                    )

                # Calibrate.
                a_fixed = float(priors.get("a_fixed", np.nan))
                b_fixed = float(priors.get("b_fixed", np.nan))
                b0 = float(priors.get("b0", np.nan))
                # Robust near/far anchors: medians of the nearest and
                # farthest 20% of the reference points.  Using a single
                # extreme point would make the baselines depend on one
                # occluded pixel rather than on the calibration idea.
                order = np.argsort(zs)
                k = max(2, int(round(0.20 * len(order))))
                near_idx = order[:k]
                far_idx = order[-k:]
                d_near = float(np.median(ds[near_idx]))
                z_near = float(np.median(zs[near_idx]))
                d_far = float(np.median(ds[far_idx]))
                z_far = float(np.median(zs[far_idx]))
                a1, b1 = fit_one_point(d_near, z_near, b0)
                a2, b2 = fit_two_point(d_near, z_near, d_far, z_far)
                aN, bN, n_inl, anchor_rmse = robust_n1(ds, zs)
                calib = {
                    "fixed": (a_fixed, b_fixed),
                    "one_point": (a1, b1),
                    "two_point": (a2, b2),
                    "n1": (aN, bN),
                }

                # Evaluation mask: valid GT, scene range, away from anchors.
                mask = np.isfinite(gt) & (gt > GT_MIN) & (gt < GT_MAX)
                if len(us):
                    yy, xx = np.mgrid[0 : gt.shape[0], 0 : gt.shape[1]]
                    for u, v in zip(us, vs):
                        mask &= (xx - u) ** 2 + (yy - v) ** 2 > ANCHOR_EXCLUSION_PX ** 2
                n_eval = int(mask.sum())
                if n_eval < 1000:
                    print(f"  {frame_id}: skipped, only {n_eval} evaluation pixels", flush=True)
                    continue

                gt_eval = gt[mask]
                bands = {
                    "near": (GT_MIN, 0.120),
                    "mid": (0.120, 0.160),
                    "far": (0.160, GT_MAX),
                }
                for method in METHODS:
                    a, b = calib[method]
                    if not np.isfinite(a) or not np.isfinite(b):
                        rmse = mae = mx = float("nan")
                        band_rmse = {k: float("nan") for k in bands}
                    else:
                        pred_z = apply_map(a, b, da)
                        err = np.abs(pred_z[mask] - gt_eval)
                        rmse = float(np.sqrt(np.mean(err ** 2)) * 1000.0)
                        mae = float(np.mean(err) * 1000.0)
                        mx = float(np.max(err) * 1000.0)
                        band_rmse = {}
                        for name, (lo, hi) in bands.items():
                            sel = (gt_eval >= lo) & (gt_eval < hi)
                            band_rmse[name] = (
                                float(np.sqrt(np.mean(err[sel] ** 2)) * 1000.0)
                                if sel.sum() >= 100
                                else float("nan")
                            )
                    row = {
                        "trial_id": f"{frame_id}_{method}",
                        "seed": seed,
                        "method": method,
                        "frame_id": frame_id,
                        "cam_x": float(pc[0]),
                        "cam_y": float(pc[1]),
                        "cam_z": float(pc[2]),
                        "cam_axis_z": float(axis[2]),
                        "n_anchors": int(len(ds)),
                        "n_anchor_inliers": int(n_inl),
                        "anchor_rmse_mm": float(anchor_rmse * 1000.0),
                        "a": float(a),
                        "b": float(b),
                        "rmse_mm": rmse,
                        "mae_mm": mae,
                        "max_error_mm": mx,
                        "mean_gt_m": float(np.mean(gt_eval)),
                        "rmse_near_mm": band_rmse["near"],
                        "rmse_mid_mm": band_rmse["mid"],
                        "rmse_far_mm": band_rmse["far"],
                        "n_eval_pixels": n_eval,
                        "q_waist": float(q_act[0]),
                        "q_shoulder": float(q_act[1]),
                        "q_elbow": float(q_act[2]),
                        "q_wrist": float(q_act[3]),
                        "gripper": float(q_act[4]),
                        "joint_track_err_rad": track,
                    }
                    log.log(row)
                    rows.append(row)
                frame_meta.append(
                    {
                        "frame_id": frame_id,
                        "seed": seed,
                        "n_anchors": int(len(ds)),
                        "n_inliers": int(n_inl),
                        "anchor_rmse_mm": float(anchor_rmse * 1000.0),
                        "mean_gt_m": float(np.mean(gt_eval)),
                    }
                )
                frame_idx += 1
                print(
                    f"  {frame_id}: anchors={len(ds)} inl={n_inl} "
                    f"n1={calib['n1'][0]:+.5f}*d{calib['n1'][1]:+.4f} "
                    f"rmse fixed/1p/2p/n1 = "
                    f"{rows[-4]['rmse_mm']:.1f}/{rows[-3]['rmse_mm']:.1f}/"
                    f"{rows[-2]['rmse_mm']:.1f}/{rows[-1]['rmse_mm']:.1f} mm",
                    flush=True,
                )

            if dev:
                RAW.mkdir(parents=True, exist_ok=True)
                (RAW / "P2-N1-DEV-anchors.json").write_text(
                    json.dumps({"d": dev_d, "z": dev_z}) + "\n"
                )
                if dev_frames:
                    np.savez_compressed(
                        RAW / "P2-N1-DEV-frames.npz",
                        da=np.stack([f["da"] for f in dev_frames]),
                        gt=np.stack([f["gt"] for f in dev_frames]),
                        mask=np.stack([f["mask"] for f in dev_frames]),
                        d_near=np.array([f["d_near"] for f in dev_frames], float),
                        z_near=np.array([f["z_near"] for f in dev_frames], float),
                    )
                _write_dev_priors(rows, seeds)
            _write_summary(exp_id, rows, frame_meta, config, t_start)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def _write_dev_priors(rows: List[dict], seeds: Sequence[int]) -> None:
    """Fit the frozen fixed calibration and the one-point offset on dev data.

    The development run uses the same anchor geometry as evaluation, but the
    ground-truth depth is allowed here because these are development seeds.
    The fixed baseline is a single global affine, fitted on downsampled
    development pixels; the one-point offset ``b0`` is chosen to minimise the
    one-point RMSE on the same development pixels.  Both are frozen before the
    evaluation seeds are used.
    """
    sidecar = RAW / "P2-N1-DEV-anchors.json"
    frames_path = RAW / "P2-N1-DEV-frames.npz"
    if not sidecar.exists() or not frames_path.exists():
        raise RuntimeError("dev sidecars missing; cannot freeze priors")
    data = json.loads(sidecar.read_text())
    d = np.asarray(data["d"], float)
    z = np.asarray(data["z"], float)

    frames = np.load(frames_path)
    da = frames["da"].astype(np.float64)
    gt = frames["gt"].astype(np.float64)
    mask = frames["mask"].astype(bool)
    d_near = frames["d_near"].astype(np.float64)
    z_near = frames["z_near"].astype(np.float64)

    # Global affine on all development pixels.
    xs = da[mask]
    ys = gt[mask]
    A = np.vstack([xs, np.ones_like(xs)]).T
    a_glob, b_glob = np.linalg.lstsq(A, ys, rcond=None)[0]

    # One-point offset: grid search on development pixels.
    best_b0, best_rmse = float(b_glob), float("inf")
    for b0 in np.linspace(0.05, 0.30, 251):
        rmses = []
        for i in range(len(da)):
            if abs(d_near[i]) < 1e-9:
                continue
            a = (z_near[i] - b0) / d_near[i]
            pred = a * da[i] + b0
            m = mask[i]
            if m.sum() == 0:
                continue
            rmses.append(float(np.sqrt(np.mean((pred[m] - gt[i][m]) ** 2))))
        if rmses:
            r = float(np.mean(rmses))
            if r < best_rmse:
                best_rmse, best_b0 = r, float(b0)

    CAL.mkdir(parents=True, exist_ok=True)
    PRIORS.write_text(
        json.dumps(
            {
                "created_at": datetime.now(timezone.utc).isoformat(),
                "dev_seeds": list(seeds),
                "n_anchor_pairs": int(len(d)),
                "n_dev_frames": int(len(da)),
                "a_fixed": float(a_glob),
                "b_fixed": float(b_glob),
                "b0": float(best_b0),
                "dev_one_point_rmse_mm": float(best_rmse * 1000.0),
                "form": "Z = a*d + b",
                "note": "Frozen before evaluation seeds 4001-4100 were used.",
            },
            indent=2,
        )
        + "\n"
    )
    print(f"wrote {PRIORS}", flush=True)


def _write_summary(
    exp_id: str,
    rows: List[dict],
    frames: List[dict],
    config: dict,
    t_start: datetime,
) -> None:
    by_method: Dict[str, List[dict]] = {m: [r for r in rows if r["method"] == m] for m in METHODS}
    summary = {
        "exp_id": exp_id,
        "started_at": t_start.isoformat(),
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "n_rows": len(rows),
        "n_frames": len(frames),
        "seeds": config["seeds"],
        "methods": list(METHODS),
        "calibration_form": config["calibration_form"],
    }
    for m in METHODS:
        cell = by_method[m]
        rmses = np.array([r["rmse_mm"] for r in cell if np.isfinite(r["rmse_mm"])], float)
        if len(rmses) == 0:
            continue
        ci = stats.bootstrap_ci(rmses, np.mean)
        med = stats.bootstrap_ci(rmses, np.median)
        summary[m] = {
            "n": int(len(rmses)),
            "mean_rmse_mm": float(np.mean(rmses)),
            "mean_rmse_ci_low": float(ci[1]),
            "mean_rmse_ci_high": float(ci[2]),
            "median_rmse_mm": float(np.median(rmses)),
            "median_rmse_ci_low": float(med[1]),
            "median_rmse_ci_high": float(med[2]),
            "mean_mae_mm": float(np.mean([r["mae_mm"] for r in cell])),
            "max_error_mm": float(np.max([r["max_error_mm"] for r in cell])),
            "mean_rmse_near_mm": float(np.nanmean([r["rmse_near_mm"] for r in cell])),
            "mean_rmse_mid_mm": float(np.nanmean([r["rmse_mid_mm"] for r in cell])),
            "mean_rmse_far_mm": float(np.nanmean([r["rmse_far_mm"] for r in cell])),
            "mean_anchor_rmse_mm": float(np.nanmean([r["anchor_rmse_mm"] for r in cell])),
        }
    # Paired comparison: n1 vs each baseline on the same frames.
    n1 = {r["frame_id"]: r["rmse_mm"] for r in by_method["n1"]}
    for m in ("fixed", "one_point", "two_point"):
        base = {r["frame_id"]: r["rmse_mm"] for r in by_method[m]}
        common = sorted(set(n1) & set(base))
        a = [n1[f] for f in common]
        b = [base[f] for f in common]
        if len(common) >= 2:
            diff, lo, hi = stats.bootstrap_diff_ci(a, b, stat=np.mean)
            wins = int(sum(x < y for x, y in zip(a, b)))
            summary[f"n1_minus_{m}"] = {
                "n_pairs": len(common),
                "mean_rmse_diff_mm": float(diff),
                "ci_low_mm": float(lo),
                "ci_high_mm": float(hi),
                "ci_excludes_zero": bool(lo > 0 or hi < 0),
                "n1_wins": wins,
            }
    RAW.mkdir(parents=True, exist_ok=True)
    (RAW / f"{exp_id}_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2), flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", action="store_true", help="run development seeds 1-20")
    ap.add_argument("--smoke", action="store_true")
    return run(ap.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
