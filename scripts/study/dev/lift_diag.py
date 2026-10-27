#!/usr/bin/env python3
"""Measure gripper orientation drift during the lift."""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts" / "study" / "dev"))

import rclpy  # noqa: E402
from aria_study.ros import StudyNode  # noqa: E402
from aria_study import kinematics as K  # noqa: E402
from grasp_probe import CUBE, SAFE, axis, ik_pad, send, servo  # noqa: E402


def orient(node, tag):
    from scipy.spatial.transform import Rotation as Rot
    out = {}
    for side, link in (("L", "finger_left_1_link"), ("R", "finger_right_1_link")):
        _, _, rpy = K.PAD_BOXES[link]
        p, q, _ = node.tf_pose("world", link, 2.0)
        R = Rot.from_quat(q).as_matrix() @ Rot.from_euler(
            "ZYX", [rpy[2], rpy[1], rpy[0]]).as_matrix()
        out[side] = (R[:, 2], R[:, 0])  # face normal, vertical axis
    nL, xL = out["L"]
    nR, xR = out["R"]
    print(f"{tag:10s} faceL={np.round(nL,3)} faceR={np.round(nR,3)} "
          f"dot={nL@nR:+.4f} tiltL={np.degrees(np.arcsin(abs(nL[2]))):.2f}deg "
          f"tiltR={np.degrees(np.arcsin(abs(nR[2]))):.2f}deg "
          f"vertL={np.round(xL,3)} vertR={np.round(xR,3)}")


def main():
    rclpy.init()
    node = StudyNode("lift_diag")
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
        servo(node, cube0, 0.9, iters=8, tol=0.0002)
        orient(node, "at 0.9")

        q_prev = node.q(K.ARM_JOINTS)
        ghold = None
        for g in np.arange(0.88, 0.29, -0.005):
            q_cmd, _ = ik_pad(cube0, float(g), seed=q_prev)
            q_prev = q_cmd
            node.send_trajectory([float(x) for x in q_cmd] + [float(g)], 0.7)
            node.spin_for(1.0)
            if max(node.contact_forces()) > 0.4:
                ghold = float(g)
                break
        print(f"contact g={ghold:.3f}")
        orient(node, "contact")

        for dz in np.arange(0.01, 0.061, 0.01):
            q_cmd, _ = ik_pad(cube0 + np.array([0, 0, dz]), ghold, seed=q_prev)
            q_prev = q_cmd
            node.send_trajectory([float(x) for x in q_cmd] + [float(ghold)], 1.2)
            node.spin_for(1.7)
            cp, _ = node.model_pose(CUBE)
            print(f"  dz={1000*dz:.0f} cube_dz={1000*(cp[2]-cube0[2]):.1f} "
                  f"dxy={1000*np.linalg.norm(cp[:2]-cube0[:2]):.1f} "
                  f"F={np.round(node.contact_forces(),2)}")
            orient(node, f"lift {1000*dz:.0f}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
