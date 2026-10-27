#!/usr/bin/env python3
"""Dump raw pad contact wrenches while the gripper holds the cube."""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

import rclpy  # noqa: E402
from aria_study.ros import StudyNode  # noqa: E402
from aria_study import kinematics as K  # noqa: E402

sys.path.insert(0, str(ROOT / "scripts" / "study" / "dev"))
from grasp_probe import SAFE, CUBE, ik_pad, send, servo, axis  # noqa: E402


def dump(node, tag):
    for side, states in (("L", node.left_contacts), ("R", node.right_contacts)):
        for s in states:
            if CUBE.split("_")[-1] not in s.collision2_name and "collision" not in s.collision2_name:
                continue
            for i, w in enumerate(s.wrenches):
                f = np.array([w.force.x, w.force.y, w.force.z])
                pos = s.contact_positions[i] if i < len(s.contact_positions) else None
                nrm = s.contact_normals[i] if i < len(s.contact_normals) else None
                pp = np.round([pos.x, pos.y, pos.z], 4) if pos else None
                nn = np.round([nrm.x, nrm.y, nrm.z], 3) if nrm else None
                print(f"  {tag} pad{side} F={np.round(f,2)} |F|={np.linalg.norm(f):.2f} "
                      f"pos={pp} n={nn}")


def main():
    rclpy.init()
    node = StudyNode("wrench_diag")
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
        servo(node, cube0, 0.9, iters=3)
        q_prev = node.q(K.ARM_JOINTS)
        for g in np.arange(0.88, 0.29, -0.005):
            q_cmd, _ = ik_pad(cube0, float(g), seed=q_prev)
            q_prev = q_cmd
            node.send_trajectory([float(x) for x in q_cmd] + [float(g)], 0.7)
            node.spin_for(1.1)
            fl, fr = node.contact_forces()
            if max(fl, fr) > 0.4:
                print(f"contact g={g:.3f}")
                dump(node, "contact")
                node.spin_for(2.0)
                print("after 2 s settle:")
                dump(node, "settled")
                print(f"  F={np.round(node.contact_forces(),2)}")
                # Now hold g and lift 10 mm.
                q_cmd, _ = ik_pad(cube0 + np.array([0, 0, 0.010]), float(g), seed=q_prev)
                node.send_trajectory([float(x) for x in q_cmd] + [float(g)], 1.2)
                node.spin_for(2.0)
                print("after 10 mm lift:")
                dump(node, "lifted")
                print(f"  F={np.round(node.contact_forces(),2)} "
                      f"cube={np.round(1000*node.model_pose(CUBE)[0],1)}")
                return
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
