#!/usr/bin/env python3
"""Dump measured pad-box geometry at the grasp pose (ground truth from TF)."""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts" / "study" / "dev"))

import rclpy  # noqa: E402
from aria_study.ros import StudyNode  # noqa: E402
from aria_study import kinematics as K  # noqa: E402
from grasp_probe import CUBE, SAFE, axis, send, servo  # noqa: E402


def report(node, g):
    print(f"\n=== g={g:.3f} ===")
    for link in ("finger_left_1_link", "finger_right_1_link"):
        c, corners = K.pad_pose(node, link)
        off, size, rpy = K.PAD_BOXES[link]
        p, q, _ = node.tf_pose("world", link, 2.0)
        Rl = np.array([[0, 0, 0]] * 3, float)
        from scipy.spatial.transform import Rotation as Rot
        Rl = Rot.from_quat(q).as_matrix()
        Rbox = Rl @ Rot.from_euler("ZYX", [rpy[2], rpy[1], rpy[0]]).as_matrix()
        print(f"{link}")
        print(f"  centre mm     {np.round(1000*c, 2)}")
        print(f"  axes (cols)   x={np.round(Rbox[:,0],3)} y={np.round(Rbox[:,1],3)} z={np.round(Rbox[:,2],3)}")
        print(f"  size mm       {np.round(1000*size,1)}")
        print(f"  world z range {1000*corners[:,2].min():.2f} .. {1000*corners[:,2].max():.2f}")
        print(f"  world x range {1000*corners[:,0].min():.2f} .. {1000*corners[:,0].max():.2f}")
        print(f"  world y range {1000*corners[:,1].min():.2f} .. {1000*corners[:,1].max():.2f}")


def main():
    rclpy.init()
    node = StudyNode("pad_geom")
    try:
        node.wait_for_joint_states(K.ALL_JOINTS, timeout=30)
        node.wait_for_models([CUBE], timeout=30)
        node.spin_for(1.0)
        node.send_conveyor_speed(0.0)
        node.spin_for(0.5)
        send(node, SAFE, 0.9, 3.0)
        node.set_model_pose(CUBE, [0.20, 0.12, 0.6488], [0, 0, 0, 1])
        node.spin_for(1.2)
        cube0, _ = node.model_pose(CUBE)
        servo(node, cube0 + np.array([0, 0, 0.06]), 0.9, iters=3)
        yaw = float(np.arctan2(*axis(node)[[1, 0]]))
        node.set_model_pose(CUBE, [cube0[0], cube0[1], 0.6488],
                            [0, 0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2))])
        node.spin_for(1.0)
        cube0, _ = node.model_pose(CUBE)
        print(f"cube centre {np.round(1000*cube0,2)} mm yaw={np.degrees(yaw):.1f} deg")
        for g in (0.9, 0.6, 0.445):
            mid, sep = servo(node, cube0, g, iters=2)
            print(f"\nservo g={g} pad_mid={np.round(1000*mid,2)} sep={1000*sep:.2f}")
            report(node, g)
        # closing axis in world from pad centres at 0.445
        cl, _ = K.pad_pose(node, "finger_left_1_link")
        cr, _ = K.pad_pose(node, "finger_right_1_link")
        d = (cl - cr) / np.linalg.norm(cl - cr)
        print(f"\nclosing axis world {np.round(d,4)}")
        _, corners = K.pad_pose(node, "finger_left_1_link")
        print(f"cube z range {1000*(cube0[2]-0.02):.2f} .. {1000*(cube0[2]+0.02):.2f}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
