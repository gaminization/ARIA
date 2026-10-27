#!/usr/bin/env python3
"""Verify overhead-camera projection against a rendered frame.

Projects the eight workpiece cubes with the analytic camera extrinsics and
draws the boxes on the captured image so the labelling geometry can be checked
by eye before any dataset is generated.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

import cv2  # noqa: E402
import rclpy  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from aria_study.ros import StudyNode  # noqa: E402

# World pose of the static overhead camera (arm_bringup/worlds/...world:123).
CAM_POS = np.array([0.18, 0.03, 1.35])
# Gazebo sensor frame -> ROS optical frame, composed with rpy = (0, pi/2, 0).
R_WORLD_OPTICAL = np.array([
    [0.0, -1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
])
CUBE_HALF = 0.020

MODELS = [f"workpiece_{i:02d}" for i in range(1, 9)]


def project(K, pts_world):
    pc = (pts_world - CAM_POS) @ R_WORLD_OPTICAL  # (N,3) in optical frame
    z = pc[:, 2]
    uv = np.empty((len(pc), 2))
    uv[:, 0] = K[0, 0] * pc[:, 0] / z + K[0, 2]
    uv[:, 1] = K[1, 1] * pc[:, 1] / z + K[1, 2]
    return uv, z


def main():
    rclpy.init()
    node = StudyNode("cam_verify")
    got = {}
    node.create_subscription(
        CameraInfo, "/top_camera/camera_info",
        lambda m: got.setdefault("info", m), 10)
    node.create_subscription(
        Image, "/top_camera/image_raw",
        lambda m: got.setdefault("img", m), 10)
    try:
        node.wait_for_models(MODELS, timeout=30)
        for _ in range(200):
            node.spin_for(0.05)
            if "info" in got and "img" in got:
                break
        info, img = got["info"], got["img"]
        K = np.array(info.k).reshape(3, 3)
        arr = np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 3)
        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

        print(f"K fx={K[0,0]:.2f} cx={K[0,2]:.2f} cy={K[1,2]:.2f}")
        for name in MODELS:
            c, q = node.model_pose(name)
            from scipy.spatial.transform import Rotation as Rot
            R = Rot.from_quat(q).as_matrix()
            corners = []
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        corners.append(c + R @ (CUBE_HALF * np.array([sx, sy, sz])))
            uv, z = project(K, np.array(corners))
            if (z <= 0).any():
                print(f"{name}: behind camera")
                continue
            u0, v0 = uv.min(axis=0)
            u1, v1 = uv.max(axis=0)
            inside = u1 > 0 and u0 < img.width and v1 > 0 and v0 < img.height
            print(f"{name} c={np.round(1000*c,1)} uv=({u0:.0f},{v0:.0f})-({u1:.0f},{v1:.0f}) "
                  f"inside={inside} depth={z.mean():.3f}")
            if inside:
                cv2.rectangle(arr, (int(u0), int(v0)), (int(u1), int(v1)), (0, 255, 0), 2)
                cv2.putText(arr, name[-2:], (int(u0), int(v0) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imwrite("/tmp/cam_verify.png", arr)
        print("saved /tmp/cam_verify.png")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
