#!/usr/bin/env python3
"""Grasp v2: slow close, low grasp point, re-centre before lift."""
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


def main():
    zoff = float(sys.argv[1]) if len(sys.argv) > 1 else -0.004
    step = float(sys.argv[2]) if len(sys.argv) > 2 else 0.002
    rclpy.init()
    node = StudyNode("grasp_v2")
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
        target = cube0 + np.array([0, 0, zoff])
        mid, sep = servo(node, target, 0.9, iters=8, tol=0.0002)
        print(f"approach pad_mid={np.round(1000*mid,2)} err={np.round(1000*(mid-target),2)}")

        q_prev = node.q(K.ARM_JOINTS)
        ghold = None
        for g in np.arange(0.88, 0.29, -step):
            q_cmd, _ = ik_pad(target, float(g), seed=q_prev)
            q_prev = q_cmd
            node.send_trajectory([float(x) for x in q_cmd] + [float(g)], 1.2)
            node.spin_for(1.5)
            if max(node.contact_forces()) > 0.5:
                ghold = float(g)
                break
        if ghold is None:
            print("RESULT no_contact")
            return
        cp, _ = node.model_pose(CUBE)
        print(f"contact g={ghold:.3f} F={np.round(node.contact_forces(),2)} "
              f"cube={np.round(1000*cp,2)}")

        # re-centre on the current cube pose at the same opening
        mid, sep = servo(node, cp + np.array([0, 0, zoff]), ghold, iters=4, tol=0.0003)
        cp, _ = node.model_pose(CUBE)
        print(f"recentred pad_mid={np.round(1000*mid,2)} cube={np.round(1000*cp,2)} "
              f"F={np.round(node.contact_forces(),2)}")

        print("lift")
        for dz in np.arange(0.005, 0.051, 0.005):
            q_cmd, _ = ik_pad(cp + np.array([0, 0, dz]), ghold, seed=q_prev)
            q_prev = q_cmd
            node.send_trajectory([float(x) for x in q_cmd] + [float(ghold)], 1.2)
            node.spin_for(1.5)
            c2, _ = node.model_pose(CUBE)
            print(f"  dz={1000*dz:4.0f} cube_dz={1000*(c2[2]-cp[2]):5.1f} "
                  f"dxy={1000*np.linalg.norm(c2[:2]-cp[:2]):5.1f} "
                  f"F={np.round(node.contact_forces(),2)}")
        c2, _ = node.model_pose(CUBE)
        print(f"RESULT {'pass' if c2[2]-cp[2] >= 0.05 else 'fail'} "
              f"dz={1000*(c2[2]-cp[2]):.1f} dxy={1000*np.linalg.norm(c2[:2]-cp[:2]):.1f}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
