#!/usr/bin/env python3
"""Dump cube orientation, pad faces and contact points at grasp and after lift."""
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
from grasp_probe import CUBE, SAFE, axis, ik_pad, send, servo  # noqa: E402


def report(node, tag, cube0):
    cp, cq = node.model_pose(CUBE)
    R = Rot.from_quat(cq).as_matrix()
    print(f"{tag}: cube {np.round(1000*cp,2)} rpy={np.round(np.degrees(Rot.from_quat(cq).as_euler('ZYX')),2)}")
    for side, link, sign in (("L", "finger_left_1_link", -1),
                             ("R", "finger_right_1_link", +1)):
        off, size, rpy = K.PAD_BOXES[link]
        p, q, _ = node.tf_pose("world", link, 2.0)
        Rb = Rot.from_quat(q).as_matrix() @ Rot.from_euler(
            "ZYX", [rpy[2], rpy[1], rpy[0]]).as_matrix()
        c = p + Rot.from_quat(q).as_matrix() @ off
        n = Rb[:, 2]
        face = c - 0.5 * size[2] * n
        print(f"  {side} centre={np.round(1000*c,2)} face_pt={np.round(1000*face,2)} "
              f"n={np.round(n,3)} halfsize={np.round(1000*size/2,1)}")
    for side, states in (("L", node.left_contacts), ("R", node.right_contacts)):
        for s in states:
            for i, w in enumerate(s.wrenches):
                f = np.array([w.force.x, w.force.y, w.force.z])
                if np.linalg.norm(f) < 0.05:
                    continue
                pos = s.contact_positions[i] if i < len(s.contact_positions) else None
                nrm = s.contact_normals[i] if i < len(s.contact_normals) else None
                pp = np.round(1000*np.array([pos.x, pos.y, pos.z]), 2) if pos else None
                nn = np.round([nrm.x, nrm.y, nrm.z], 3) if nrm else None
                print(f"  {side} contact F={np.round(f,2)} |F|={np.linalg.norm(f):.2f} "
                      f"pos={pp} n={nn}")


def main():
    rclpy.init()
    node = StudyNode("contact_geom")
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
        print(f"target yaw={np.degrees(yaw):.2f}")
        servo(node, cube0, 0.9, iters=8, tol=0.0002)
        report(node, "at 0.9", cube0)

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
        print(f"\ncontact g={ghold:.3f}")
        report(node, "contact", cube0)
        q_cmd, _ = ik_pad(cube0 + np.array([0, 0, 0.005]), ghold, seed=q_prev)
        node.send_trajectory([float(x) for x in q_cmd] + [float(ghold)], 1.0)
        node.spin_for(2.0)
        print("\nafter 5 mm lift")
        report(node, "lift", cube0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
