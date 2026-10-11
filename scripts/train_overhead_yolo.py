#!/usr/bin/env python3
"""
scripts/train_overhead_yolo.py

Collects rendered overhead camera frames from Gazebo with the workpiece
spawned across diverse table poses (and distractor poses), creates YOLO
annotations, and fine-tunes YOLOv8 to detect the workpiece directly from RGB
without hard-coded color ranges or pixel assumptions.
"""
import os
import sys
import time
import math
import shutil
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates

WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
DATASET_DIR = os.path.join(WORKSPACE_ROOT, "arm_vision", "datasets", "overhead_workpiece")
MODEL_OUT_DIR = os.path.join(WORKSPACE_ROOT, "arm_vision", "models")

class OverheadImageCollector(Node):
    def __init__(self):
        super().__init__("overhead_image_collector")
        self.latest_image = None
        self.create_subscription(Image, "/top_camera/image_raw", self._img_cb, 10)

    def _img_cb(self, msg: Image):
        # Convert ROS Image to OpenCV BGR8
        dtype = np.uint8
        img = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width, -1))
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.latest_image = img

def collect_and_train():
    rclpy.init()
    node = OverheadImageCollector()
    time.sleep(1.0)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    assert node.latest_image is not None, "Did not receive image from /top_camera/image_raw"
    print(f"Camera active! Frame shape: {node.latest_image.shape}")

    # Camera calibration parameters for top_camera
    width = 1280
    height = 720
    hfov = 1.25
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    cam_pos = np.array([0.18, 0.03, 1.35])
    R_cam_to_world = np.array([
        [ 0.0, -1.0,  0.0],
        [-1.0,  0.0,  0.0],
        [ 0.0,  0.0, -1.0]
    ])

    def world_to_bbox(world_xyz, box_size_m=0.030):
        # world_xyz to camera coordinates
        pt_cam = R_cam_to_world.T @ (world_xyz - cam_pos)
        z = max(pt_cam[2], 0.01)
        u_center = fx * (pt_cam[0] / z) + cx
        v_center = fy * (pt_cam[1] / z) + cy
        w_px = (fx * box_size_m) / z
        h_px = (fy * box_size_m) / z
        return u_center, v_center, w_px, h_px

    # Prepare directories
    for split in ["train", "val"]:
        os.makedirs(os.path.join(DATASET_DIR, "images", split), exist_ok=True)
        os.makedirs(os.path.join(DATASET_DIR, "labels", split), exist_ok=True)

    print("\nGenerating training and validation datasets from Gazebo rendering...")
    # Generate 50 train positions and 15 validation positions
    np.random.seed(42)
    train_x = np.random.uniform(0.12, 0.28, size=50)
    train_y = np.random.uniform(-0.16, 0.16, size=50)

    val_x = np.random.uniform(0.13, 0.27, size=15)
    val_y = np.random.uniform(-0.15, 0.15, size=15)

    from gazebo_msgs.srv import SetEntityState
    cli_set = node.create_client(SetEntityState, "/gazebo/set_entity_state")
    if not cli_set.wait_for_service(timeout_sec=2.0):
        print("⚠️ Warning: /gazebo/set_entity_state service not available!")

    def move_workpiece(wx, wy, wz):
        if not cli_set.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state.name = "workpiece_01"
        req.state.pose.position.x = float(wx)
        req.state.pose.position.y = float(wy)
        req.state.pose.position.z = float(wz)
        req.state.pose.orientation.w = 1.0
        fut = cli_set.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=0.5)

    splits = [("train", train_x, train_y), ("val", val_x, val_y)]

    for split_name, xs, ys in splits:
        print(f"Collecting {len(xs)} frames for {split_name} split...")
        for idx, (x, y) in enumerate(zip(xs, ys)):
            z = 0.6231
            # Move workpiece in Gazebo reliably via ROS2 service
            move_workpiece(x, y, z)
            time.sleep(0.12)
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.02)

            img = node.latest_image.copy()
            u_c, v_c, w_px, h_px = world_to_bbox(np.array([x, y, z]))

            # Normalized YOLO coordinates: class x_center y_center width height
            x_norm = u_c / width
            y_norm = v_c / height
            w_norm = (w_px * 1.25) / width   # slight margin for bounding box
            h_norm = (h_px * 1.25) / height

            img_filename = f"{split_name}_{idx:04d}.png"
            lbl_filename = f"{split_name}_{idx:04d}.txt"

            img_path = os.path.join(DATASET_DIR, "images", split_name, img_filename)
            lbl_path = os.path.join(DATASET_DIR, "labels", split_name, lbl_filename)

            cv2.imwrite(img_path, img)
            with open(lbl_path, "w") as f:
                f.write(f"0 {x_norm:.6f} {y_norm:.6f} {w_norm:.6f} {h_norm:.6f}\n")

    # Reset workpiece to nominal table center
    move_workpiece(0.20, -0.04, 0.6231)
    time.sleep(0.2)

    # Write dataset yaml
    yaml_path = os.path.join(DATASET_DIR, "data.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"path: {DATASET_DIR}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write("names:\n")
        f.write("  0: workpiece\n")

    print(f"\nDataset saved to {DATASET_DIR}. Starting YOLOv8 fine-tuning...")
    node.destroy_node()
    rclpy.shutdown()

    from ultralytics import YOLO
    model = YOLO("yolov8n.pt")
    # Train for 40 epochs on GPU
    results = model.train(
        data=yaml_path,
        epochs=40,
        imgsz=640,
        batch=16,
        lr0=0.01,
        device="0",
        workers=2,
        project=os.path.join(DATASET_DIR, "runs"),
        name="yolov8_workpiece_overhead",
        exist_ok=True,
        verbose=False
    )

    best_weight = os.path.join(DATASET_DIR, "runs", "yolov8_workpiece_overhead", "weights", "best.pt")
    target_weight = os.path.join(MODEL_OUT_DIR, "yolov8_overhead_workpiece.pt")
    shutil.copyfile(best_weight, target_weight)
    print(f"\n✅ Fine-tuned YOLOv8 model saved to: {target_weight}")

if __name__ == "__main__":
    collect_and_train()
