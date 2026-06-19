#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
Robotics Toolbox IK Solver for the ARIA 5-DoF Arm
Uses roboticstoolbox-python (Peter Corke's library).
Provides: Levenberg-Marquardt (LM) and Gauss-Newton (GN) solvers.
═══════════════════════════════════════════════════════════════
"""
import math
import time
from typing import Optional

import numpy as np

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
    RTB_AVAILABLE = True
except ImportError:
    RTB_AVAILABLE = False

from arm_ik.ik_solvers.aria_analytical_ik import (
    IKResult, SolverConfig, JOINT_LIMITS, forward_kinematics
)


def _build_rtb_robot() -> 'rtb.DHRobot':
    """
    Build a DHRobot from ARIA DH parameters using standard DH convention.

    DH Parameters:
      Joint  | θ      | d      | a      | α
      -------|--------|--------|--------|--------
      1      | θ1     | 0.105  | 0      | π/2
      2      | θ2     | 0      | 0.145  | 0
      3      | θ3     | 0      | 0.115  | 0
      4      | θ4     | 0      | 0.055  | π/2
      5      | θ5     | 0.040  | 0      | 0
    """
    links = [
        # Joint 1: Waist (d=0.105 includes base + bracket height)
        rtb.RevoluteDH(
            d=0.105, a=0, alpha=math.pi / 2,
            qlim=JOINT_LIMITS[0],
        ),
        # Joint 2: Shoulder
        rtb.RevoluteDH(
            d=0, a=0.145, alpha=0,
            qlim=JOINT_LIMITS[1],
        ),
        # Joint 3: Elbow
        rtb.RevoluteDH(
            d=0, a=0.115, alpha=0,
            qlim=JOINT_LIMITS[2],
        ),
        # Joint 4: Wrist pitch
        rtb.RevoluteDH(
            d=0, a=0.055, alpha=math.pi / 2,
            qlim=JOINT_LIMITS[3],
        ),
        # Joint 5: Wrist roll
        rtb.RevoluteDH(
            d=0.040, a=0, alpha=0,
            qlim=JOINT_LIMITS[4],
        ),
    ]

    robot = rtb.DHRobot(links, name="aria_arm")
    return robot


# Module-level robot (built once)
_robot: Optional['rtb.DHRobot'] = None


def _get_robot() -> 'rtb.DHRobot':
    """Lazy initialization of the RTB robot model."""
    global _robot
    if _robot is None:
        _robot = _build_rtb_robot()
    return _robot


def solve_rtb(target_position: np.ndarray,
              target_orientation: Optional[np.ndarray] = None,
              method: str = 'LM',
              initial_joints: Optional[np.ndarray] = None,
              config: Optional[SolverConfig] = None) -> IKResult:
    """
    Solve IK using Robotics Toolbox.

    Args:
        target_position: [x, y, z] target in base_link frame
        target_orientation: 3×3 rotation matrix (optional)
        method: 'LM' (Levenberg-Marquardt) or 'GN' (Gauss-Newton)
        initial_joints: warm start joint angles [5]
        config: solver configuration

    Returns:
        IKResult

    Methods:
        LM: Levenberg-Marquardt — robust, good convergence basin
        GN: Gauss-Newton — faster near solution, less robust far away
    """
    if not RTB_AVAILABLE:
        return IKResult(
            success=False,
            solver_name=f"rtb_{method}",
            message="roboticstoolbox not installed"
        )

    if config is None:
        config = SolverConfig()

    t_start = time.perf_counter()
    robot = _get_robot()

    # Build target SE3 pose
    if target_orientation is not None:
        T_target = SE3.Rt(target_orientation, target_position)
    else:
        # Position-only: use identity rotation
        T_target = SE3(target_position)

    # Initial guess
    if initial_joints is not None:
        q0 = initial_joints
    elif config.current_joints is not None:
        q0 = config.current_joints
    else:
        q0 = np.array([0.0, 1.0, 0.5, 0.0, 0.0])

    try:
        # Select solver method
        if method.upper() == 'LM':
            sol = robot.ikine_LM(
                T_target,
                q0=q0,
                ilimit=config.max_iterations,
                slimit=100,
                tol=config.tolerance_m,
                joint_limits=True,
                mask=[1, 1, 1, 1, 1, 0] if target_orientation is None
                     else [1, 1, 1, 1, 1, 1],
            )
        elif method.upper() == 'GN':
            sol = robot.ikine_GN(
                T_target,
                q0=q0,
                ilimit=config.max_iterations,
                slimit=100,
                tol=config.tolerance_m,
                joint_limits=True,
                pinv=True,
                mask=[1, 1, 1, 1, 1, 0] if target_orientation is None
                     else [1, 1, 1, 1, 1, 1],
            )
        else:
            return IKResult(
                success=False,
                solver_name=f"rtb_{method}",
                message=f"Unknown method: {method}"
            )

        joint_angles = np.array(sol.q)

        # Clamp to limits
        for j in range(5):
            joint_angles[j] = np.clip(
                joint_angles[j], JOINT_LIMITS[j, 0], JOINT_LIMITS[j, 1]
            )

        # Verify with our FK
        T = forward_kinematics(joint_angles)
        fk_pos = T[:3, 3]
        pos_error = np.linalg.norm(fk_pos - target_position)

        solve_time = (time.perf_counter() - t_start) * 1000

        success = sol.success and pos_error < config.tolerance_m * 10

        return IKResult(
            success=success,
            joint_angles=joint_angles,
            position_error_m=pos_error,
            solve_time_ms=solve_time,
            solver_name=f"rtb_{method}",
            message=f"{'Solved' if success else 'Failed'}: "
                    f"err={pos_error*1000:.1f}mm, "
                    f"iters={sol.iterations}",
        )

    except Exception as e:
        solve_time = (time.perf_counter() - t_start) * 1000
        return IKResult(
            success=False,
            solve_time_ms=solve_time,
            solver_name=f"rtb_{method}",
            message=f"RTB {method} exception: {str(e)}",
        )


if __name__ == "__main__":
    print("═══ Robotics Toolbox IK Self-Test ═══")

    if not RTB_AVAILABLE:
        print("❌ roboticstoolbox not installed. Run: pip install roboticstoolbox-python")
    else:
        target = np.array([0.15, 0.05, 0.30])

        for method in ['LM', 'GN']:
            result = solve_rtb(target, method=method)
            print(f"\n{method}:")
            print(f"  Success: {result.success}")
            print(f"  Joints (deg): {np.degrees(result.joint_angles)}")
            print(f"  Error: {result.position_error_m*1000:.2f} mm")
            print(f"  Time: {result.solve_time_ms:.2f} ms")
            print(f"  Message: {result.message}")
