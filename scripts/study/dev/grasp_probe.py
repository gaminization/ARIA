#!/usr/bin/env python3
"""Development grasp bring-up for the repaired parallel-jaw gripper.

This is *not* a study run: it does not log trials or compute statistics.  It is
the tool used to establish that the repaired gripper can pick and lift the
40 mm workpiece, and to calibrate the grasp opening.  The N2 study script
imports the same primitives from ``aria_study.kinematics``.

The four-DoF arm cannot satisfy an arbitrary wrist pose, so this tool solves
the inverse kinematics for the *pad midpoint* directly (position first, with a
soft "fingers point down" term).  All arm motion is commanded from that solve;
the measured pad midpoint from TF is used only to correct the target.

Usage:
    grasp_probe.py [--x X] [--y Y] [--zoff Z] [--hold-g G]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

import rclpy  # noqa: E402
from aria_study.ros import StudyNode  # noqa: E402
from aria_study import kinematics as K  # noqa: E402
from aria_analytical_ik import forward_kinematics as fk, JOINT_LIMITS  # noqa: E402

CUBE = "workpiece_01"
SAFE = [0.0, 0.10, -0.10, 1.55]
NEIGHBOURS = (("workpiece_02", 0.45), ("workpiece_03", 0.55), ("workpiece_04", 0.65))


def pad_mid_fk(q, g):
    T = fk(np.asarray(q, float))
    return T[:3, 3] + T[:3, :3] @ K.pad_offset_wrist(g), T[:3, :3]


def ik_pad(target_world, g, seed=None):
    """Least-squares IK on the pad midpoint; returns (q, residual_m)."""
    t = K.world_to_base(target_world)

    def res(q):
        m, R = pad_mid_fk(q, g)
        r = list((m - t) * 1000.0)
        down = R @ np.array([0.0, 0.0, 1.0])
        r.extend(list(0.05 * (down - np.array([0.0, 0.0, -1.0]))))
        if seed is not None:
            r.extend(list(0.3 * (np.asarray(q) - np.asarray(seed))))
        return np.array(r)

    seeds = ([np.asarray(seed, float)] if seed is not None else []) + [
        [0, 1.2, -0.6, 1.2],
        [0, 0.9, -1.0, 0.6],
    ]
    best = None
    for s in seeds:
        s = np.clip(np.asarray(s, float), JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        r = least_squares(
            res, s, bounds=(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]),
            xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=4000,
        )
        m, _ = pad_mid_fk(r.x, g)
        e = float(np.linalg.norm(m - t))
        if best is None or e < best[0]:
            best = (e, r.x)
    return best[1], best[0]


def send(node, q_arm, g, duration=2.0):
    node.send_trajectory([float(x) for x in q_arm] + [float(g)], duration)
    node.spin_for(duration + 0.4)


def servo(node, target, g, iters=4, tol=0.0015):
    target = np.asarray(target, float)
    tgt = target.copy()
    for _ in range(iters):
        q_sol, _ = ik_pad(tgt, g, seed=node.q(K.ARM_JOINTS))
        send(node, q_sol, g)
        mid, sep = K.measured_pad_midpoint(node)
        err = target - mid
        if np.linalg.norm(err) < tol:
            break
        tgt = tgt + err
    return mid, sep


def axis(node):
    cl, _ = K.pad_pose(node, "finger_left_1_link")
    cr, _ = K.pad_pose(node, "finger_right_1_link")
    return (cl - cr) / np.linalg.norm(cl - cr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=0.20)
    ap.add_argument("--y", type=float, default=0.12)
    ap.add_argument("--zoff", type=float, default=0.0, help="pad midpoint above cube centre (m)")
    ap.add_argument("--hold-g", type=float, default=None,
                    help="skip the close sweep and hold this opening")
    ap.add_argument("--lift", type=float, default=0.06)
    args = ap.parse_args()

    rclpy.init()
    node = StudyNode("grasp_probe")
    try:
        node.wait_for_joint_states(K.ALL_JOINTS, timeout=30)
        node.wait_for_models([CUBE], timeout=30)
        node.spin_for(1.0)
        node.send_conveyor_speed(0.0)
        node.spin_for(0.5)

        send(node, SAFE, 0.9, 3.0)
        for name, yy in NEIGHBOURS:
            node.set_model_pose(name, [0.20, yy, 0.6488], [0, 0, 0, 1])
        node.set_model_pose(CUBE, [args.x, args.y, 0.6488], [0, 0, 0, 1])
        node.spin_for(1.2)
        cube0, _ = node.model_pose(CUBE)
        print(f"cube {np.round(1000*cube0,1)} mm")

        servo(node, cube0 + np.array([0, 0, 0.05]), 0.9, iters=3)
        yaw = float(np.arctan2(*axis(node)[[1, 0]]))
        node.set_model_pose(
            CUBE, [cube0[0], cube0[1], 0.6488],
            [0, 0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2))],
        )
        node.spin_for(1.0)
        cube0, _ = node.model_pose(CUBE)
        mid, sep = servo(node, cube0 + np.array([0, 0, args.zoff]), 0.9, iters=3)
        print(f"grasp pose pad_mid={np.round(1000*mid,1)} sep={sep*1000:.1f} yaw={np.degrees(yaw):.1f}")

        if args.hold_g is not None:
            ghold = args.hold_g
            q_prev = node.q(K.ARM_JOINTS)
            send(node, q_prev, ghold, 2.0)
        else:
            q_prev = node.q(K.ARM_JOINTS)
            hold = cube0 + np.array([0, 0, args.zoff])
            ghold = None
            for g in np.arange(0.88, 0.29, -0.005):
                q_cmd, _ = ik_pad(hold, float(g), seed=q_prev)
                q_prev = q_cmd
                node.send_trajectory([float(x) for x in q_cmd] + [float(g)], 0.7)
                node.spin_for(1.1)
                fl, fr = node.contact_forces()
                if max(fl, fr) > 0.4:
                    ghold = float(g)
                    break
            if ghold is None:
                print("RESULT no_contact")
                return
            cp, _ = node.model_pose(CUBE)
            print(f"first contact g={ghold:.3f} F={np.round(node.contact_forces(),2)} "
                  f"cube_moved={np.round(1000*(cp-cube0),1)} mm")

        lifted = 0.0
        for dz in np.arange(0.01, args.lift + 1e-9, 0.01):
            q_cmd, _ = ik_pad(cube0 + np.array([0, 0, dz]), ghold, seed=q_prev)
            q_prev = q_cmd
            node.send_trajectory([float(x) for x in q_cmd] + [float(ghold)], 1.2)
            node.spin_for(1.7)
            cp, _ = node.model_pose(CUBE)
            lifted = float(cp[2] - cube0[2])
            mid2, sep2 = K.measured_pad_midpoint(node)
            pl, pr = node.contact_partners()
            cl, kl = K.pad_pose(node, "finger_left_1_link")
            cr, kr = K.pad_pose(node, "finger_right_1_link")
            print(f"  lift dz={1000*dz:.0f} pad={np.round(1000*mid2,1)} "
                  f"padLz={1000*kl[:,2].min():.0f}..{1000*kl[:,2].max():.0f} "
                  f"padRz={1000*kr[:,2].min():.0f}..{1000*kr[:,2].max():.0f} "
                  f"cube={np.round(1000*cp,1)} cube_dz={1000*lifted:.1f} "
                  f"F={np.round(node.contact_forces(),2)} "
                  f"p={sorted(set(x.split('::')[-1] for x in pl+pr))}")
            if lifted < 0.01 and dz > 0.02:
                break
        node.spin_for(2.0)
        cp, _ = node.model_pose(CUBE)
        dz = float(cp[2] - cube0[2])
        dxy = float(np.linalg.norm(cp[:2] - cube0[:2]))
        ok = dz >= 0.050
        print(f"RESULT {'pass' if ok else 'fail'} dz={1000*dz:.1f}mm dxy={1000*dxy:.1f}mm "
              f"F={np.round(node.contact_forces(),2)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
