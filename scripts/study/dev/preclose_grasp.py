#!/usr/bin/env python3
"""Grasp with the jaws pre-closed to the workpiece width before descending.

Closing the jaws *around* the cube applies a torque that rolls it out of the
grip.  This variant descends with the jaws already at ~40.8 mm, so the cube
enters the gap with clearance and only a tiny close is needed to grip.
"""
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


def gap_at(node):
    cl, kl = K.pad_pose(node, "finger_left_1_link")
    cr, kr = K.pad_pose(node, "finger_right_1_link")
    d = (cl - cr) / np.linalg.norm(cl - cr)
    return 1000 * ((cl @ d - max(abs((kl - cl) @ d)))
                   - (cr @ d + max(abs((kr - cr) @ d))))


def main():
    g_open = float(sys.argv[1]) if len(sys.argv) > 1 else 0.45
    g_grip = float(sys.argv[2]) if len(sys.argv) > 2 else 0.442
    rclpy.init()
    node = StudyNode("preclose")
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

        # Align the cube to the closing axis at the *grip* opening.
        servo(node, cube0 + np.array([0, 0, 0.06]), g_open, iters=3)
        yaw = float(np.arctan2(*axis(node)[[1, 0]]))
        node.set_model_pose(CUBE, [cube0[0], cube0[1], 0.6488],
                            [0, 0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2))])
        node.spin_for(1.0)
        cube0, _ = node.model_pose(CUBE)
        print(f"cube {np.round(1000*cube0,1)} yaw={np.degrees(yaw):.1f}")

        for dz in (0.060, 0.040, 0.020, 0.008, 0.0):
            mid, sep = servo(node, cube0 + np.array([0, 0, dz]), g_open, iters=3)
            print(f"  descend dz={1000*dz:.0f} pad={np.round(1000*mid,1)} "
                  f"gap={gap_at(node):.1f} F={np.round(node.contact_forces(),2)}")

        q_prev = node.q(K.ARM_JOINTS)
        for g in np.arange(g_open, g_grip - 1e-9, -0.002):
            q_cmd, _ = ik_pad(cube0, float(g), seed=q_prev)
            q_prev = q_cmd
            node.send_trajectory([float(x) for x in q_cmd] + [float(g)], 0.8)
            node.spin_for(1.2)
            print(f"  close g={g:.3f} gap={gap_at(node):.1f} "
                  f"F={np.round(node.contact_forces(),2)} "
                  f"cube={np.round(1000*node.model_pose(CUBE)[0],1)}")
            if max(node.contact_forces()) > 1.0:
                break

        print("  lift")
        for dz in np.arange(0.01, 0.061, 0.01):
            q_cmd, _ = ik_pad(cube0 + np.array([0, 0, dz]), g_grip, seed=q_prev)
            q_prev = q_cmd
            node.send_trajectory([float(x) for x in q_cmd] + [float(g_grip)], 1.2)
            node.spin_for(1.7)
            cp, _ = node.model_pose(CUBE)
            print(f"    dz={1000*dz:.0f} cube_dz={1000*(cp[2]-cube0[2]):.1f} "
                  f"dxy={1000*np.linalg.norm(cp[:2]-cube0[:2]):.1f} "
                  f"F={np.round(node.contact_forces(),2)}")
        cp, _ = node.model_pose(CUBE)
        print(f"RESULT dz={1000*(cp[2]-cube0[2]):.1f} "
              f"dxy={1000*np.linalg.norm(cp[:2]-cube0[:2]):.1f}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
