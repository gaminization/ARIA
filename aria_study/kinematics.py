"""URDF-faithful kinematics and measured gripper geometry for the ARIA study.

The arm has four actuated joints (waist, shoulder, elbow, wrist_pitch).  The
gripper pads are *not* at the wrist origin: at the closed configuration the
pad midpoint sits about 35 mm behind and 66 mm along the wrist y axis.  The
offsets below were measured from Gazebo TF (robot_state_publisher) across a
gripper sweep; see ``data/journal/calibration``.

Nothing here models an outcome.  FK is the URDF chain; IK is a numerical
solve whose residual is reported; pad poses are read from TF at run time.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares

_IK_DIR = Path(__file__).resolve().parents[1] / "arm_ik" / "arm_ik" / "ik_solvers"
if str(_IK_DIR) not in sys.path:
    sys.path.insert(0, str(_IK_DIR))

from aria_analytical_ik import JOINT_LIMITS, forward_kinematics  # noqa: E402

ARM_JOINTS = ["waist_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint"]
ALL_JOINTS = ARM_JOINTS + ["gripper_joint"]

# world -> base_link: base is at (0, 0, 0.614) with yaw +90 deg.
BASE_Z = 0.614
_RZ_POS = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
_RZ_NEG = _RZ_POS.T

# Pad collision boxes in the finger-link frames (from aria_arm.urdf.xacro).
# The third element is the collision-box rpy.  The orientations were corrected
# so that at the grasp opening (gripper_joint ~= 0.30 rad) the two pad faces
# are opposed: the left normal points at the right pad and the right normal
# points at the left pad (dot = -1.000).  The original boxes were rotated 90
# degrees about the finger axis, so the pads approached the object edge-on.
PAD_BOXES = {
    "finger_left_1_link": (
        np.array([0.0012, 0.038, -0.003]),
        np.array([0.022, 0.016, 0.016]),
        (2.704300, -1.566512, 0.0),
    ),
    "finger_right_1_link": (
        np.array([-0.0012, -0.038, 0.001]),
        np.array([0.022, 0.016, 0.016]),
        (-2.678970, 1.566213, 0.0),
    ),
}

# Measured pad-midpoint offset in the wrist frame vs actual gripper angle.
_G_CAL = np.array([0.0, 0.5, 0.7, 1.0])
_OFF_CAL = np.array(
    [
        [-0.0356, 0.0664, -0.0060],
        [-0.0341, 0.0629, -0.0060],
        [-0.0331, 0.0594, -0.0065],
        [-0.0307, 0.0515, -0.0069],
    ]
)

# A top grasp: fingers point down, separation along world y, wrist x along world x.
R_TOP_W = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
R_TOP_B = _RZ_NEG @ R_TOP_W


def world_to_base(p: Sequence[float]) -> np.ndarray:
    x, y, z = (float(v) for v in p)
    return np.array([y, -x, z - BASE_Z])


def base_to_world(p: Sequence[float]) -> np.ndarray:
    x, y, z = (float(v) for v in p)
    return np.array([-y, x, z + BASE_Z])


def rot_base_to_world(R: np.ndarray) -> np.ndarray:
    return _RZ_POS @ R


def pad_offset_wrist(gripper: float) -> np.ndarray:
    """Pad-midpoint offset expressed in the wrist frame (metres)."""
    g = float(np.clip(gripper, _G_CAL[0], _G_CAL[-1]))
    return np.array([np.interp(g, _G_CAL, _OFF_CAL[:, i]) for i in range(3)])


def pad_offset_world(gripper: float, R_wrist_world: np.ndarray = R_TOP_W) -> np.ndarray:
    return R_wrist_world @ pad_offset_wrist(gripper)


def grasp_to_wrist(grasp_world: Sequence[float], gripper: float,
                   R_wrist_world: np.ndarray = R_TOP_W) -> np.ndarray:
    return np.asarray(grasp_world, float) - pad_offset_world(gripper, R_wrist_world)


def ik_pose(
    wrist_world: Sequence[float],
    R_wrist_world: np.ndarray = R_TOP_W,
    seed: Optional[Sequence[float]] = None,
    reg: float = 1e-3,
) -> Tuple[np.ndarray, float, float]:
    """Solve the 4-DoF arm for a wrist pose.  Returns (q, pos_err_m, ori_err).

    ``reg`` keeps the solution on the branch nearest ``seed``; without it the
    solver can jump branches and sweep the pads through the scene.
    """
    p = world_to_base(wrist_world)
    R_target = _RZ_NEG @ R_wrist_world

    def residual(q):
        T = forward_kinematics(q)
        rp = (T[:3, 3] - p) * 1000.0
        R = T[:3, :3]
        ro = []
        for i in range(3):
            ro.extend(np.cross(R[:, i], R_target[:, i]))
        r = np.array(list(rp) + list(ro))
        if seed is not None and reg > 0:
            r = np.concatenate([r, reg * (np.asarray(q) - np.asarray(seed))])
        return r

    seeds = []
    if seed is not None:
        seeds.append(np.asarray(seed, float))
    seeds += [
        np.array([0.0, 1.4, -0.9, 0.7]),
        np.array([0.05, 1.5, -0.2, 1.4]),
        np.array([0.0, 1.2, -0.5, 1.0]),
        np.array([0.0, 0.9, -1.0, 0.5]),
    ]
    best = None
    for s in seeds:
        s = np.clip(np.asarray(s, float), JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        r = least_squares(
            residual, s, bounds=(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]),
            xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=4000,
        )
        # Compare on pose error only, ignoring the regularisation term.
        T = forward_kinematics(r.x)
        pe = float(np.linalg.norm(T[:3, 3] - p))
        oe = 0.0
        for i in range(3):
            oe += float(np.linalg.norm(np.cross(T[:3, i], R_target[:, i])))
        if best is None or pe + 0.05 * oe < best[0]:
            best = (pe + 0.05 * oe, r.x, pe, oe)
    return best[1], best[2], best[3]


def pad_pose(node, link: str) -> Tuple[np.ndarray, np.ndarray]:
    """Measured pad-box centre and 8 corners in world coordinates."""
    from scipy.spatial.transform import Rotation as Rot

    off, size, rpy = PAD_BOXES[link]
    p, q, _ = node.tf_pose("world", link, 2.0)
    Rl = Rot.from_quat(q).as_matrix()
    centre = p + Rl @ off
    R = Rl @ Rot.from_euler("ZYX", [rpy[2], rpy[1], rpy[0]]).as_matrix()
    corners = np.array(
        [
            centre + R @ (size * np.array([sx, sy, sz]))
            for sx in (-0.5, 0.5)
            for sy in (-0.5, 0.5)
            for sz in (-0.5, 0.5)
        ]
    )
    return centre, corners


def measured_pad_midpoint(node) -> Tuple[np.ndarray, float]:
    """Measured pad midpoint and centre-to-centre separation from TF."""
    cl, _ = pad_pose(node, "finger_left_1_link")
    cr, _ = pad_pose(node, "finger_right_1_link")
    return 0.5 * (cl + cr), float(np.linalg.norm(cl - cr))
