#!/usr/bin/env python3
"""Static hold diagnostic: does the grasp survive without any arm motion?"""
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


def state(node, tag, cube0):
    cl, _ = K.pad_pose(node, "finger_left_1_link")
    cr, _ = K.pad_pose(node, "finger_right_1_link")
    mid = 0.5 * (cl + cr)
    d = (cl - cr) / np.linalg.norm(cl - cr)
    cp, _ = node.model_pose(CUBE)
    gq = node.q(["gripper_joint"])[0]
    print(f"{tag:14s} mid={np.round(1000*mid,1)} axis={np.round(d,3)} "
          f"cube={np.round(1000*cp,1)} dxy={1000*np.linalg.norm(cp[:2]-cube0[:2]):5.1f} "
          f"dz={1000*(cp[2]-cube0[2]):5.1f} F={np.round(node.contact_forces(),2)} g={gq:.4f}")


def main():
    rclpy.init()
    node = StudyNode("hold_diag")
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
        state(node, "at 0.9", cube0)

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
        print(f"first contact g={ghold:.3f}")
        state(node, "contact", cube0)

        print("--- hold 5 s, no motion ---")
        for i in range(10):
            node.spin_for(0.5)
            state(node, f"hold {0.5*(i+1):.1f}s", cube0)

        print("--- lift 5 mm ---")
        q_cmd, _ = ik_pad(cube0 + np.array([0, 0, 0.005]), ghold, seed=q_prev)
        node.send_trajectory([float(x) for x in q_cmd] + [float(ghold)], 1.0)
        for i in range(8):
            node.spin_for(0.5)
            state(node, f"lift {0.5*(i+1):.1f}s", cube0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
