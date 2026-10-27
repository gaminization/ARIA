#!/usr/bin/env python3
"""Compute collision-box rpy so both pad faces are vertical and antiparallel."""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts" / "study" / "dev"))

import rclpy  # noqa: E402
from aria_study.ros import StudyNode  # noqa: E402
from aria_study import kinematics as K  # noqa: E402
from grasp_probe import CUBE, SAFE, axis, send, servo  # noqa: E402


def link_rot(node, link):
    _, q, _ = node.tf_pose("world", link, 2.0)
    return Rot.from_quat(q).as_matrix()


def main():
    rclpy.init()
    node = StudyNode("pad_align")
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
        servo(node, cube0 + np.array([0, 0, 0.05]), 0.9, iters=3)
        yaw = float(np.arctan2(*axis(node)[[1, 0]]))
        node.set_model_pose(CUBE, [cube0[0], cube0[1], 0.6488],
                            [0, 0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2))])
        node.spin_for(1.0)
        cube0, _ = node.model_pose(CUBE)
        # settle at the grasp opening without contacting: 1 mm above
        servo(node, cube0 + np.array([0, 0, 0.001]), 0.445, iters=8, tol=0.0002)
        cl, _ = K.pad_pose(node, "finger_left_1_link")
        cr, _ = K.pad_pose(node, "finger_right_1_link")
        d = (cl - cr) / np.linalg.norm(cl - cr)
        d = np.array([d[0], d[1], 0.0])
        d /= np.linalg.norm(d)
        print(f"closing axis {np.round(d,5)}  sep={1000*np.linalg.norm(cl-cr):.3f} mm")

        up = np.array([0.0, 0.0, 1.0])
        for link, z_des in (("finger_left_1_link", -d),
                            ("finger_right_1_link", d)):
            z = z_des / np.linalg.norm(z_des)
            x = up - (up @ z) * z
            x /= np.linalg.norm(x)
            y = np.cross(z, x)
            R_target = np.column_stack([x, y, z])
            R_link = link_rot(node, link)
            R_coll = R_link.T @ R_target
            rpy = Rot.from_matrix(R_coll).as_euler("ZYX")
            # current
            _, _, rpy0 = K.PAD_BOXES[link]
            R0 = Rot.from_euler("ZYX", [rpy0[2], rpy0[1], rpy0[0]]).as_matrix()
            n0 = R_link @ R0[:, 2]
            n1 = R_link @ R_coll[:, 2]
            print(f"{link}")
            print(f"  current rpy {np.round(rpy0,6)}  normal {np.round(n0,4)}")
            print(f"  new     rpy {np.round(rpy,6)}  normal {np.round(n1,4)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
