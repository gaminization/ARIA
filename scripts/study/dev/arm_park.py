#!/usr/bin/env python3
"""Park the arm at SAFE and check whether it occludes the belt objects."""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

import rclpy  # noqa: E402
from aria_study.ros import StudyNode  # noqa: E402
from aria_study import kinematics as K  # noqa: E402

SAFE = [0.0, 0.10, -0.10, 1.55]
CAM_POS = np.array([0.18, 0.03, 1.35])
R_WO = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def proj(Km, p):
    pc = (np.asarray(p) - CAM_POS) @ R_WO
    return np.array([Km[0, 0] * pc[0] / pc[2] + Km[0, 2],
                     Km[1, 1] * pc[1] / pc[2] + Km[1, 2]])


def main():
    rclpy.init()
    node = StudyNode("arm_park")
    try:
        node.wait_for_joint_states(K.ALL_JOINTS, timeout=30)
        node.send_conveyor_speed(0.0)
        node.spin_for(0.5)
        node.send_trajectory([float(x) for x in SAFE] + [0.9], 4.0)
        node.spin_for(6.0)
        print("joints", {k: round(v, 3) for k, v in node.joint_positions.items()})
        for link in ("finger_left_1_link", "finger_right_1_link", "wrist_link",
                     "elbow_link", "shoulder_link"):
            try:
                p, q, _ = node.tf_pose("world", link, 2.0)
                print(f"{link:22s} world={np.round(p,4)}")
            except Exception as e:  # noqa: BLE001
                print(link, "no tf", e)
        # pad midpoint
        mid, sep = K.measured_pad_midpoint(node)
        print(f"pad mid world={np.round(mid,4)} sep={1000*sep:.1f} mm")
        Km = np.array([[887.06, 0, 640.5], [0, 887.06, 360.5], [0, 0, 1]])
        print("pad mid uv", np.round(proj(Km, mid), 1))
        for i in range(1, 9):
            c, _ = node.model_pose(f"workpiece_{i:02d}")
            print(f"wp{i:02d} uv {np.round(proj(Km, c),1)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
