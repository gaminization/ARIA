#!/usr/bin/env python3
"""Identify which collisions produce the reported pad forces."""
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


def dump(node, tag):
    for side, states in (("L", node.left_contacts), ("R", node.right_contacts)):
        for s in states:
            f = np.zeros(3)
            for w in s.wrenches:
                f += np.array([w.force.x, w.force.y, w.force.z])
            print(f"  {tag} {side} {s.collision1_name} <-> {s.collision2_name} "
                  f"|F|={np.linalg.norm(f):.2f} F={np.round(f,2)}")
    print(f"  {tag} total L/R={np.round(node.contact_forces(),2)} "
          f"on_cube={np.round(node.contact_force_on('workpiece'),2)}")


def main():
    rclpy.init()
    node = StudyNode("cube_contact")
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
        # tight centring
        servo(node, cube0, 0.9, iters=8, tol=0.0002)
        mid, sep = K.measured_pad_midpoint(node)
        print(f"centred pad_mid={np.round(1000*mid,2)} cube={np.round(1000*cube0,2)} "
              f"err={np.round(1000*(mid-cube0),2)} sep={1000*sep:.2f}")

        q_prev = node.q(K.ARM_JOINTS)
        ghold = None
        for g in np.arange(0.88, 0.29, -0.005):
            q_cmd, _ = ik_pad(cube0, float(g), seed=q_prev)
            q_prev = q_cmd
            node.send_trajectory([float(x) for x in q_cmd] + [float(g)], 0.7)
            node.spin_for(1.0)
            if node.contact_force_on("workpiece")[0] + node.contact_force_on("workpiece")[1] > 0.4:
                ghold = float(g)
                break
        print(f"first cube contact g={ghold}")
        dump(node, "contact")

        print("--- lift 5 mm ---")
        q_cmd, _ = ik_pad(cube0 + np.array([0, 0, 0.005]), ghold, seed=q_prev)
        node.send_trajectory([float(x) for x in q_cmd] + [float(ghold)], 1.0)
        node.spin_for(2.0)
        dump(node, "lifted")
        cp, _ = node.model_pose(CUBE)
        print(f"cube dz={1000*(cp[2]-cube0[2]):.1f} dxy={1000*np.linalg.norm(cp[:2]-cube0[:2]):.1f}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
