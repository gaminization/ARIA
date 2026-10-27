#!/usr/bin/env python3
"""P0-DET dataset collection: auto-labelled overhead frames from Gazebo.

Protocol (PREREGISTRATION.md §4, §5, P0-DET):

* 30 scenes: 1-20 train (50 frames each), 21-24 validation (30 each),
  25-30 test (20 each) -> 1,000 train / 120 val / 120 test frames.
* Frames within a scene are randomised; no test-scene frame is used for
  training.  Scene base layouts are seeded and logged.
* Labels are computed from Gazebo ground truth: the eight 40 mm workpieces are
  projected through the calibrated overhead camera and their 3D corners are
  reduced to an axis-aligned 2D box.  No hand labelling and no detection-based
  pseudo-labels are used.

The camera is a static world model (``arm_bringup/worlds/aria_industrial_workcell.world``,
pose ``0.18 0.03 1.35 0 1.5708 0``); it is not in TF, so the extrinsic is the
world-file pose and the intrinsic is read from ``/top_camera/camera_info`` at
run time.

Outputs (all under the git-ignored raw journal):

    data/journal/raw/detector/images/{train,val,test}/*.png
    data/journal/raw/detector/labels/{train,val,test}/*.txt
    data/journal/raw/detector/data.yaml
    data/journal/raw/detector/scenes.json
    data/journal/raw/P0-DET_frames.csv
    data/journal/raw/P0-DET_collect.json
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import cv2  # noqa: E402
import rclpy  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402

from aria_study import kinematics as K  # noqa: E402
from aria_study.provenance import assert_clean, stamp  # noqa: E402
from aria_study.ros import StudyNode  # noqa: E402

OUT = ROOT / "data" / "journal" / "raw" / "detector"
RAW = ROOT / "data" / "journal" / "raw"

# Static overhead camera: world pose from the world file.
CAM_POS = np.array([0.18, 0.03, 1.35])
# Gazebo sensor frame (+X forward, +Y left, +Z up) -> ROS optical frame
# (+X right, +Y down, +Z forward), composed with the model rpy (0, pi/2, 0).
R_WORLD_OPTICAL = np.array([
    [0.0, -1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
])

MODELS = [f"workpiece_{i:02d}" for i in range(1, 9)]
HALF = 0.020
CLASS_ID = 0
CLASS_NAME = "workpiece"

# Visible belt region at workpiece height (u in [0,1280], v in [0,720]).
REGION_X = (0.168, 0.232)
REGION_Y = (0.085, 0.505)
MIN_SEP = 0.058
SPARE_Y = (0.560, 0.615, 0.670)  # parked on the belt, outside the image

SPLITS = {
    "train": (1, 20, 50),
    "val": (21, 24, 30),
    "test": (25, 30, 20),
}

SAFE = [0.0, 0.10, -0.10, 1.55]
GRIP_OPEN = 0.90


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sample_positions(rng: np.random.Generator, n: int,
                     existing: list[np.ndarray] = ()) -> list[np.ndarray]:
    pts: list[np.ndarray] = []
    for _ in range(20000):
        if len(pts) == n:
            return pts
        p = np.array([rng.uniform(*REGION_X), rng.uniform(*REGION_Y)])
        if all(np.linalg.norm(p - q) >= MIN_SEP for q in list(existing) + pts):
            pts.append(p)
    raise RuntimeError(f"could not place {n} objects with {MIN_SEP} m separation")


def project(K: np.ndarray, pts_world: np.ndarray) -> np.ndarray:
    pc = (pts_world - CAM_POS) @ R_WORLD_OPTICAL
    if (pc[:, 2] <= 0.05).any():
        raise RuntimeError("object behind the camera")
    uv = np.empty((len(pc), 2))
    uv[:, 0] = K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2]
    uv[:, 1] = K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]
    return uv


def cube_bbox(K: np.ndarray, centre: np.ndarray, quat: np.ndarray,
              width: int, height: int):
    R = Rot.from_quat(quat).as_matrix()
    corners = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                corners.append(centre + R @ (HALF * np.array([sx, sy, sz])))
    uv = project(K, np.array(corners))
    u0, v0 = uv.min(axis=0)
    u1, v1 = uv.max(axis=0)
    raw_area = max(u1 - u0, 1e-9) * max(v1 - v0, 1e-9)
    cu0, cv0 = max(u0, 0.0), max(v0, 0.0)
    cu1, cv1 = min(u1, float(width)), min(v1, float(height))
    if cu1 <= cu0 or cv1 <= cv0:
        return None
    clipped_area = (cu1 - cu0) * (cv1 - cv0)
    if clipped_area / raw_area < 0.5:
        return None
    if (cu1 - cu0) < 4.0 or (cv1 - cv0) < 4.0:
        return None
    return cu0, cv0, cu1, cv1


def to_yolo(box, width: int, height: int) -> str:
    u0, v0, u1, v1 = box
    xc = 0.5 * (u0 + u1) / width
    yc = 0.5 * (v0 + v1) / height
    bw = (u1 - u0) / width
    bh = (v1 - v0) / height
    return f"{CLASS_ID} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}"


class Collector(StudyNode):
    def __init__(self) -> None:
        super().__init__("p0_det_collect")
        self.frame = None
        self.seq = 0
        self.info = None
        self.create_subscription(Image, "/top_camera/image_raw", self._img_cb, 10)
        self.create_subscription(CameraInfo, "/top_camera/camera_info", self._info_cb, 10)

    def _img_cb(self, msg: Image) -> None:
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        self.frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        self.seq += 1

    def _info_cb(self, msg: CameraInfo) -> None:
        if self.info is None:
            self.info = msg


def build_scene(scene: int) -> dict:
    rng = np.random.default_rng(10_000 + scene)
    n = 6
    pts = sample_positions(rng, n)
    return {
        "scene": scene,
        "seed": 10_000 + scene,
        "base_xy": [[float(x), float(y)] for x, y in pts],
        "n_base": n,
    }


def frame_layout(scene_layout: dict, frame_idx: int, split: str) -> dict:
    rng = np.random.default_rng(scene_layout["seed"] * 1000 + frame_idx + 1)
    pts = [np.array(p, float) for p in scene_layout["base_xy"]]
    # small jitter, preserving the minimum separation
    for _ in range(200):
        jittered = [p + rng.uniform(-0.006, 0.006, size=2) for p in pts]
        jittered = [np.array([np.clip(p[0], *REGION_X), np.clip(p[1], *REGION_Y)])
                    for p in jittered]
        ok = all(np.linalg.norm(jittered[i] - jittered[j]) >= 0.050
                 for i in range(len(jittered)) for j in range(i + 1, len(jittered)))
        if ok:
            pts = jittered
            break
    yaws = rng.uniform(-np.pi, np.pi, len(pts))
    # choose which six of the eight models occupy the six positions
    order = rng.permutation(len(MODELS))
    active = [MODELS[i] for i in order[:len(pts)]]
    spare = [MODELS[i] for i in order[len(pts):]]
    # drop one object in a fraction of frames (5 visible instead of 6)
    keep = len(pts)
    if rng.random() < 0.30:
        keep = len(pts) - 1
    return {"xy": pts, "yaws": yaws, "active": active, "spare": spare, "keep": keep}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=int, default=0, help="debug: cap frames per scene")
    args = ap.parse_args()

    assert_clean("P0-DET collection")
    prov = stamp()

    for split in SPLITS:
        (OUT / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUT / "labels" / split).mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Collector()
    started = time.time()
    rows: list[dict] = []
    scenes: dict[int, dict] = {}
    try:
        node.wait_for_joint_states(K.ALL_JOINTS, timeout=30)
        node.wait_for_models(MODELS, timeout=30)
        for _ in range(100):
            node.spin_for(0.05)
            if node.frame is not None and node.info is not None:
                break
        if node.frame is None or node.info is None:
            raise RuntimeError("camera did not publish")
        Kmat = np.array(node.info.k).reshape(3, 3)
        width, height = node.frame.shape[1], node.frame.shape[0]
        print(f"camera {width}x{height} fx={Kmat[0,0]:.3f} cx={Kmat[0,2]:.3f} cy={Kmat[1,2]:.3f}")

        node.send_conveyor_speed(0.0)
        node.spin_for(0.5)
        node.send_trajectory([float(x) for x in SAFE] + [GRIP_OPEN], 4.0)
        node.spin_for(5.0)
        mid, _ = K.measured_pad_midpoint(node)
        pad_uv = project(Kmat, np.array([mid]))[0]
        print(f"parked pad uv {np.round(pad_uv,1)}")

        for split, (s0, s1, per_scene) in SPLITS.items():
            for scene in range(s0, s1 + 1):
                layout = build_scene(scene)
                scenes[scene] = {**layout, "split": split, "frames": per_scene}
                n_frames = per_scene if not args.limit else min(per_scene, args.limit)
                for fi in range(n_frames):
                    t_frame0 = time.time()
                    fl = frame_layout(layout, fi, split)
                    keep = fl["keep"]
                    placed = list(zip(fl["active"][:keep], fl["xy"][:keep], fl["yaws"][:keep]))
                    # Every model not in the active set is parked off-screen:
                    # the two spares plus any active slot dropped this frame.
                    unused = list(fl["spare"])
                    if keep < len(fl["active"]):
                        unused.append(fl["active"][keep])
                    for name, y in zip(unused, SPARE_Y):
                        node.set_model_pose(name, [0.20, float(y), 0.649], [0, 0, 0, 1])
                    for name, xy, yaw in placed:
                        q = [0.0, 0.0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2))]
                        node.set_model_pose(name, [float(xy[0]), float(xy[1]), 0.649], q)
                    node.spin_for(0.30)
                    poses0 = {n: node.model_pose(n)[0].copy() for n, _, _ in placed}

                    seq0 = node.seq
                    for _ in range(60):
                        node.spin_for(0.02)
                        if node.seq > seq0:
                            break
                    if node.seq == seq0:
                        raise RuntimeError(f"no camera frame for scene {scene} frame {fi}")
                    img = node.frame.copy()

                    labels = []
                    boxes = []
                    drift = 0.0
                    for name, xy, _yaw in placed:
                        c, qv = node.model_pose(name)
                        drift = max(drift, float(np.linalg.norm(c - poses0[name])))
                        box = cube_bbox(Kmat, c, qv, width, height)
                        if box is None:
                            continue
                        # drop objects under the parked arm (should never happen)
                        if (box[0] - 40 <= pad_uv[0] <= box[2] + 40 and
                                box[1] - 40 <= pad_uv[1] <= box[3] + 40):
                            raise RuntimeError(
                                f"arm overlaps {name} in scene {scene} frame {fi}")
                        labels.append(to_yolo(box, width, height))
                        boxes.append([float(v) for v in box])
                    if not labels:
                        raise RuntimeError(f"no labels in scene {scene} frame {fi}")
                    if drift > 0.001:
                        raise RuntimeError(
                            f"pose drift {1000*drift:.2f} mm in scene {scene} frame {fi}")

                    stem = f"{split}_s{scene:02d}_f{fi:04d}"
                    img_path = OUT / "images" / split / f"{stem}.png"
                    lbl_path = OUT / "labels" / split / f"{stem}.txt"
                    cv2.imwrite(str(img_path), img)
                    lbl_path.write_text("\n".join(labels) + "\n")
                    rows.append({
                        "frame_id": stem,
                        "scene": scene,
                        "split": split,
                        "frame_in_scene": fi,
                        "image": str(img_path.relative_to(ROOT)),
                        "label": str(lbl_path.relative_to(ROOT)),
                        "n_objects": len(labels),
                        "seed": layout["seed"],
                        "commit": prov["commit"],
                        "dirty": "false",
                        "image_sha256": sha256_file(img_path),
                        "started_at": t_frame0,
                        "finished_at": time.time(),
                    })
                print(f"scene {scene:02d} ({split}) -> {n_frames} frames, "
                      f"total {len(rows)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    with (OUT / "data.yaml").open("w") as f:
        f.write(f"path: {OUT}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write("test: images/test\n")
        f.write(f"names:\n  0: {CLASS_NAME}\n")
    (OUT / "scenes.json").write_text(json.dumps(scenes, indent=1, sort_keys=True))

    fields = ["frame_id", "scene", "split", "frame_in_scene", "image", "label",
              "n_objects", "seed", "commit", "dirty", "image_sha256",
              "started_at", "finished_at"]
    with (RAW / "P0-DET_frames.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    counts = {s: sum(1 for r in rows if r["split"] == s) for s in SPLITS}
    summary = {
        "exp_id": "P0-DET",
        "phase": "collection",
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(started)),
        "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(time.time())),
        "n_frames": len(rows),
        "frames_by_split": counts,
        "scenes_by_split": {s: [b[0], b[1]] for s, b in SPLITS.items()},
        "class": {"id": CLASS_ID, "name": CLASS_NAME},
        "camera": {
            "model": "overhead_camera_model",
            "pose_world": CAM_POS.tolist(),
            "rpy_world": [0.0, 1.5708, 0.0],
            "intrinsics": Kmat.tolist(),
            "image_size": [width, height],
            "source": "world file pose + /top_camera/camera_info",
        },
        "region_x": REGION_X,
        "region_y": REGION_Y,
        "min_separation_m": MIN_SEP,
        "arm_pose": SAFE + [GRIP_OPEN],
        "commit": prov["commit"],
        "dirty": False,
    }
    (RAW / "P0-DET_collect.json").write_text(json.dumps(summary, indent=1))
    print(f"wrote {len(rows)} frames: {counts}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
