#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ikpy-based IK Solver for the ARIA 5-DoF Arm
Uses the ikpy library to build a kinematic chain from DH params
and solve via numerical optimization (L-BFGS-B).
═══════════════════════════════════════════════════════════════
"""
import math
import time
from typing import Optional

import numpy as np

try:
    import ikpy.chain
    import ikpy.link
    IKPY_AVAILABLE = True
except ImportError:
    IKPY_AVAILABLE = False

from arm_ik.ik_solvers.aria_analytical_ik import (
    IKResult, SolverConfig, JOINT_LIMITS, forward_kinematics
)


def _build_ikpy_chain() -> 'ikpy.chain.Chain':
    """
    Build an ikpy Chain from ARIA DH parameters.

    ikpy uses a modified DH convention internally, so we specify
    links with translation and rotation parameters matching our
    kinematic structure.
    """
    links = [
        # Base (fixed, origin to shoulder)
        ikpy.link.OriginLink(),

        # Joint 1: Waist — Z rotation, 0.105m up
        ikpy.link.URDFLink(
            name="waist",
            origin_translation=[0, 0, 0.105],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],  # Z-axis
            bounds=(JOINT_LIMITS[0, 0], JOINT_LIMITS[0, 1]),
        ),

        # Joint 2: Shoulder — Y rotation
        ikpy.link.URDFLink(
            name="shoulder",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],  # Y-axis
            bounds=(JOINT_LIMITS[1, 0], JOINT_LIMITS[1, 1]),
        ),

        # Joint 3: Elbow — Y rotation, 0.145m along previous link
        ikpy.link.URDFLink(
            name="elbow",
            origin_translation=[0, 0, 0.145],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],  # Y-axis
            bounds=(JOINT_LIMITS[2, 0], JOINT_LIMITS[2, 1]),
        ),

        # Joint 4: Wrist pitch — Y rotation, 0.115m along forearm
        ikpy.link.URDFLink(
            name="wrist_pitch",
            origin_translation=[0, 0, 0.115],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],  # Y-axis
            bounds=(JOINT_LIMITS[3, 0], JOINT_LIMITS[3, 1]),
        ),

        # Joint 5: Wrist roll — X rotation, 0.055m along wrist
        ikpy.link.URDFLink(
            name="wrist_roll",
            origin_translation=[0, 0, 0.055],
            origin_orientation=[0, 0, 0],
            rotation=[1, 0, 0],  # X-axis
            bounds=(JOINT_LIMITS[4, 0], JOINT_LIMITS[4, 1]),
        ),

        # Tool frame (fixed, 0.040m gripper offset)
        ikpy.link.URDFLink(
            name="tool",
            origin_translation=[0, 0, 0.040],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0],  # Fixed
        ),
    ]

    return ikpy.chain.Chain(
        name="aria_arm",
        links=links,
        active_links_mask=[False, True, True, True, True, True, False],
    )


# Module-level chain (built once)
_chain: Optional['ikpy.chain.Chain'] = None


def _get_chain() -> 'ikpy.chain.Chain':
    """Lazy initialization of the ikpy chain."""
    global _chain
    if _chain is None:
        _chain = _build_ikpy_chain()
    return _chain


def solve_ikpy(target_position: np.ndarray,
               target_orientation: Optional[np.ndarray] = None,
               initial_position: Optional[np.ndarray] = None,
               config: Optional[SolverConfig] = None) -> IKResult:
    """
    Solve IK using the ikpy library.

    Args:
        target_position: [x, y, z] target in base_link frame (meters)
        target_orientation: 3×3 rotation matrix (optional, ikpy can solve
                           position-only if None)
        initial_position: initial joint angles [5] for warm start
        config: solver configuration

    Returns:
        IKResult
    """
    if not IKPY_AVAILABLE:
        return IKResult(
            success=False,
            solver_name="ikpy",
            message="ikpy library not installed"
        )

    if config is None:
        config = SolverConfig()

    t_start = time.perf_counter()
    chain = _get_chain()

    # Build initial position vector (ikpy includes fixed links)
    # Format: [fixed, j1, j2, j3, j4, j5, fixed] = 7 values
    if initial_position is not None:
        init_pos = [0.0] + list(initial_position) + [0.0]
    elif config.current_joints is not None:
        init_pos = [0.0] + list(config.current_joints) + [0.0]
    else:
        init_pos = [0.0, 0.0, 1.0, 0.5, 0.0, 0.0, 0.0]

    try:
        # Build target frame (4×4 homogeneous)
        target_frame = np.eye(4)
        target_frame[:3, 3] = target_position

        if target_orientation is not None:
            target_frame[:3, :3] = target_orientation

        # Solve with ikpy's inverse_kinematics
        # orientation_mode: "all" uses full orientation, "X" ignores some
        if target_orientation is not None:
            ik_result = chain.inverse_kinematics_frame(
                target=target_frame,
                initial_position=init_pos,
                orientation_mode="all",
            )
        else:
            ik_result = chain.inverse_kinematics(
                target_position=target_position,
                initial_position=init_pos,
                orientation_mode=None,
            )

        # Extract active joint angles (skip fixed links)
        joint_angles = np.array(ik_result[1:6])

        # Clamp to joint limits
        for j in range(5):
            joint_angles[j] = np.clip(
                joint_angles[j], JOINT_LIMITS[j, 0], JOINT_LIMITS[j, 1]
            )

        # Verify with our FK
        T = forward_kinematics(joint_angles)
        fk_pos = T[:3, 3]
        pos_error = np.linalg.norm(fk_pos - target_position)

        solve_time = (time.perf_counter() - t_start) * 1000

        success = pos_error < config.tolerance_m * 10  # 10mm default

        return IKResult(
            success=success,
            joint_angles=joint_angles,
            position_error_m=pos_error,
            solve_time_ms=solve_time,
            solver_name="ikpy",
            message=f"{'Solved' if success else 'High error'}: {pos_error*1000:.1f}mm",
        )

    except Exception as e:
        solve_time = (time.perf_counter() - t_start) * 1000
        return IKResult(
            success=False,
            solve_time_ms=solve_time,
            solver_name="ikpy",
            message=f"ikpy exception: {str(e)}",
        )


if __name__ == "__main__":
    print("═══ ikpy IK Solver Self-Test ═══")

    if not IKPY_AVAILABLE:
        print("❌ ikpy not installed. Run: pip install ikpy")
    else:
        target = np.array([0.15, 0.05, 0.30])
        result = solve_ikpy(target)
        print(f"Target: {target}")
        print(f"Success: {result.success}")
        print(f"Joints (deg): {np.degrees(result.joint_angles)}")
        print(f"Error: {result.position_error_m*1000:.2f} mm")
        print(f"Time: {result.solve_time_ms:.2f} ms")
