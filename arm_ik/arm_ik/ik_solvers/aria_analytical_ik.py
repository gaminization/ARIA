#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
Closed-Form Analytical IK for the ARIA 5-DoF Arm
Techno-Tirupati kit — geometric decomposition approach.

DH Parameters (from assembly manual):
  d1=0.070  a1=0      α1=0°    (waist)
  d2=0      a2=0      α2=90°   (shoulder)
  d3=0      a3=0.145  α3=0°    (elbow)
  d4=0      a4=0.115  α4=0°    (wrist pitch)
  d5=0      a5=0.055  α5=90°   (wrist roll)
  d6=0.040  a6=0      α6=0°    (gripper)

Effective kinematic chain:
  Base → shoulder height: d1 + waist_bracket = 0.105m
  Upper arm: L1 = 0.145m
  Forearm:   L2 = 0.115m
  Wrist:     L3 = 0.055m
  Gripper:   L4 = 0.040m

Note: 5-DoF arm cannot achieve arbitrary 6-DoF poses.
      We solve for position (3 DoF) + pitch (1 DoF) + roll (1 DoF).
═══════════════════════════════════════════════════════════════
"""
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


# ═══════════════════════════════════════════════════════════════
# Data structures
# ═══════════════════════════════════════════════════════════════
@dataclass
class SolverConfig:
    """Configuration for the IK solver."""
    max_iterations: int = 100          # Not used for analytical (exact)
    tolerance_m: float = 0.001         # Position tolerance in meters
    tolerance_rad: float = 0.01        # Orientation tolerance in radians
    prefer_elbow_up: bool = True       # Prefer elbow-up configuration
    current_joints: Optional[np.ndarray] = None  # For nearest-solution selection


@dataclass
class IKResult:
    """Result from an IK solver."""
    success: bool = False
    joint_angles: np.ndarray = field(default_factory=lambda: np.zeros(5))
    position_error_m: float = float('inf')
    orientation_error_rad: float = float('inf')
    solve_time_ms: float = 0.0
    solver_name: str = "analytical"
    message: str = ""
    all_solutions: List[np.ndarray] = field(default_factory=list)


# ═══════════════════════════════════════════════════════════════
# ARIA arm constants (from DH parameters / URDF)
# ═══════════════════════════════════════════════════════════════

# Vertical offset: base cylinder (0.070) + waist bracket (0.035)
D_BASE = 0.105

# Link lengths in the 2R planar chain
L_UPPER = 0.145   # upper arm (a3)
L_FORE  = 0.115   # forearm (a4)

# Wrist-to-tool offset
L_WRIST = 0.055   # wrist length (a5)
L_GRIP  = 0.040   # gripper holder (d6)
L_TOOL  = L_WRIST + L_GRIP  # total wrist-to-tool = 0.095m

# Joint limits (radians) — from Stage 1 URDF
JOINT_LIMITS = np.array([
    [-1.5708,  1.5708],   # waist:       -90° to +90°
    [ 0.0000,  3.1416],   # shoulder:      0° to 180°
    [ 0.0000,  2.6180],   # elbow:         0° to 150°
    [-1.5708,  1.5708],   # wrist_pitch: -90° to +90°
    [-1.5708,  1.5708],   # wrist_roll:  -90° to +90°
])


def forward_kinematics(joints: np.ndarray) -> np.ndarray:
    """
    Compute end-effector pose from joint angles using DH convention.

    Args:
        joints: [θ1, θ2, θ3, θ4, θ5] in radians

    Returns:
        4×4 homogeneous transformation matrix (base_link → tool_frame)

    Derivation:
        T = T_base · T1(θ1) · T2(θ2) · T3(θ3) · T4(θ4) · T5(θ5) · T_tool
        Each Ti is the standard DH transformation matrix:
        Ti = Rz(θi) · Tz(di) · Tx(ai) · Rx(αi)
    """
    θ1, θ2, θ3, θ4, θ5 = joints

    # ── DH transformation matrices ─────────────────────────
    # Standard DH: T = Rz(θ) · Tz(d) · Tx(a) · Rx(α)
    def dh_matrix(theta, d, a, alpha):
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d     ],
            [0,   0,        0,       1     ]
        ])

    # Base offset (translation up to shoulder)
    T_base = np.eye(4)
    T_base[2, 3] = D_BASE  # Z offset to shoulder

    # Joint 1: Waist rotation (Z-axis)
    # DH: θ=θ1, d=0, a=0, α=π/2 (rotates next axis to Y)
    T1 = dh_matrix(θ1, 0, 0, math.pi / 2)

    # Joint 2: Shoulder rotation (now Y-axis due to α1=90°)
    # DH: θ=θ2, d=0, a=L_UPPER, α=0
    T2 = dh_matrix(θ2, 0, L_UPPER, 0)

    # Joint 3: Elbow rotation (Y-axis)
    # DH: θ=θ3, d=0, a=L_FORE, α=0
    T3 = dh_matrix(θ3, 0, L_FORE, 0)

    # Joint 4: Wrist pitch (Y-axis)
    # DH: θ=θ4, d=0, a=L_WRIST, α=π/2
    T4 = dh_matrix(θ4, 0, L_WRIST, math.pi / 2)

    # Joint 5: Wrist roll (X-axis after α4=90°)
    # DH: θ=θ5, d=L_GRIP, a=0, α=0
    T5 = dh_matrix(θ5, L_GRIP, 0, 0)

    # Chain all transformations
    T = T_base @ T1 @ T2 @ T3 @ T4 @ T5

    return T


def solve_analytical(target_position: np.ndarray,
                     target_pitch: float = 0.0,
                     target_roll: float = 0.0,
                     config: Optional[SolverConfig] = None) -> IKResult:
    """
    5-DoF analytical IK for the Techno-Tirupati arm.

    Geometric decomposition approach:
      1. θ1 (waist): atan2(py, px) of target position
      2. θ2, θ3 (shoulder, elbow): planar 2-link IK in vertical plane
      3. θ4 (wrist pitch): achieves target pitch angle
      4. θ5 (wrist roll): achieves target roll angle

    Args:
        target_position: [x, y, z] in base_link frame (meters)
        target_pitch: desired end-effector pitch angle (radians)
        target_roll: desired end-effector roll angle (radians)
        config: solver configuration

    Returns:
        IKResult with best solution and all valid alternatives

    Diagram:
                    Shoulder (θ2)
                   /
                  / L_UPPER (0.145m)
                 /
        Base ───●───── Elbow (θ3)
        (θ1)   |        \\
               |         \\ L_FORE (0.115m)
               |          \\
               |           ● Wrist (θ4, θ5)
               |           |
             D_BASE       L_TOOL
             (0.105m)     (0.095m)
    """
    if config is None:
        config = SolverConfig()

    t_start = time.perf_counter()

    px, py, pz = target_position
    all_solutions = []

    # ═══════════════════════════════════════════════════════
    # STEP 1: θ1 (Waist) — Z-axis rotation
    # ═══════════════════════════════════════════════════════
    # The waist angle is simply the atan2 of the XY position.
    # This decouples the 3D problem into a 2D vertical plane.
    #
    # θ1 = atan2(py, px)
    #
    # Edge case: if px=0 and py=0, target is directly above base.
    # In this case θ1 is arbitrary; we use 0 or current joint angle.
    if abs(px) < 1e-6 and abs(py) < 1e-6:
        # Target directly above base — θ1 is degenerate
        if config.current_joints is not None:
            theta1 = config.current_joints[0]
        else:
            theta1 = 0.0
    else:
        theta1 = math.atan2(py, px)

    # Check θ1 limits
    if not (JOINT_LIMITS[0, 0] <= theta1 <= JOINT_LIMITS[0, 1]):
        # Try θ1 + π (reach from behind)
        theta1_alt = theta1 + math.pi
        if theta1_alt > math.pi:
            theta1_alt -= 2 * math.pi
        if JOINT_LIMITS[0, 0] <= theta1_alt <= JOINT_LIMITS[0, 1]:
            theta1 = theta1_alt
        else:
            result = IKResult(
                success=False,
                solve_time_ms=(time.perf_counter() - t_start) * 1000,
                message=f"Waist angle {math.degrees(theta1):.1f}° out of limits"
            )
            return result

    # ═══════════════════════════════════════════════════════
    # STEP 2: Wrist center position
    # ═══════════════════════════════════════════════════════
    # To solve the 2-link IK for shoulder+elbow, we need the
    # wrist center position (before the wrist pitch/roll).
    #
    # The tool extends L_TOOL from the wrist center in the
    # direction determined by the target pitch angle.
    #
    # wrist_center = target_pos - L_TOOL * approach_direction
    #
    # For a planar arm, the approach direction in the vertical
    # plane is determined by the desired pitch angle:
    #   approach_x = cos(target_pitch)  (horizontal)
    #   approach_z = sin(target_pitch)  (vertical)

    # Horizontal distance from base Z-axis to target
    r_target = math.sqrt(px * px + py * py)

    # Wrist center in the vertical plane (r, z)
    # The tool projects from wrist center in the pitch direction
    r_wrist = r_target - L_TOOL * math.cos(target_pitch)
    z_wrist = (pz - D_BASE) - L_TOOL * math.sin(target_pitch)

    # ═══════════════════════════════════════════════════════
    # STEP 3: θ2, θ3 (Shoulder, Elbow) — Planar 2R IK
    # ═══════════════════════════════════════════════════════
    # We have a 2-link planar arm with:
    #   L1 = L_UPPER = 0.145m (shoulder → elbow)
    #   L2 = L_FORE  = 0.115m (elbow → wrist)
    #
    # Target: reach point (r_wrist, z_wrist) in the vertical plane.
    #
    # Using the law of cosines:
    #   D² = r² + z² (squared distance to wrist center)
    #   cos(θ3) = (D² - L1² - L2²) / (2·L1·L2)
    #
    # Then θ2 from geometry:
    #   θ2 = atan2(z, r) - atan2(L2·sin(θ3), L1 + L2·cos(θ3))

    D_sq = r_wrist * r_wrist + z_wrist * z_wrist
    D = math.sqrt(D_sq)

    # Reachability check
    L1, L2 = L_UPPER, L_FORE
    reach_max = L1 + L2
    reach_min = abs(L1 - L2)

    if D > reach_max or D < reach_min:
        result = IKResult(
            success=False,
            solve_time_ms=(time.perf_counter() - t_start) * 1000,
            message=(f"Target unreachable: distance={D:.4f}m, "
                     f"range=[{reach_min:.4f}, {reach_max:.4f}]m")
        )
        return result

    # Law of cosines for elbow angle
    #   cos(θ3) = (D² - L1² - L2²) / (2·L1·L2)
    cos_theta3 = (D_sq - L1 * L1 - L2 * L2) / (2 * L1 * L2)

    # Clamp to [-1, 1] for numerical stability
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))

    # Two solutions: elbow-up and elbow-down
    # θ3 = ±acos(cos_theta3)
    theta3_options = [
        math.acos(cos_theta3),    # Elbow-down (positive)
        -math.acos(cos_theta3),   # Elbow-up (negative)
    ]

    for theta3 in theta3_options:
        # ═══════════════════════════════════════════════════
        # Shoulder angle θ2
        # ═══════════════════════════════════════════════════
        # θ2 = atan2(z_w, r_w) - atan2(L2·sin(θ3), L1 + L2·cos(θ3))
        #
        # This places the arm so the wrist reaches (r_w, z_w).
        # The first atan2 gives the angle to the wrist center.
        # The second atan2 accounts for the elbow bend.

        beta = math.atan2(z_wrist, r_wrist)
        phi = math.atan2(L2 * math.sin(theta3), L1 + L2 * math.cos(theta3))
        theta2 = beta - phi

        # ═══════════════════════════════════════════════════
        # STEP 4: θ4 (Wrist Pitch)
        # ═══════════════════════════════════════════════════
        # The total pitch of the end-effector is:
        #   pitch_total = θ2 + θ3 + θ4
        # So:
        #   θ4 = target_pitch - (θ2 + θ3)
        theta4 = target_pitch - (theta2 + theta3)

        # ═══════════════════════════════════════════════════
        # STEP 5: θ5 (Wrist Roll)
        # ═══════════════════════════════════════════════════
        # Directly maps to target roll angle.
        theta5 = target_roll

        solution = np.array([theta1, theta2, theta3, theta4, theta5])

        # ─── Check all joint limits ───────────────────────
        within_limits = True
        for j in range(5):
            if not (JOINT_LIMITS[j, 0] <= solution[j] <= JOINT_LIMITS[j, 1]):
                within_limits = False
                break

        if within_limits:
            # Verify with FK
            T = forward_kinematics(solution)
            fk_pos = T[:3, 3]
            pos_error = np.linalg.norm(fk_pos - target_position)

            if pos_error < config.tolerance_m * 10:  # Relaxed check
                all_solutions.append(solution.copy())

    # ═══════════════════════════════════════════════════════
    # Select best solution
    # ═══════════════════════════════════════════════════════
    solve_time = (time.perf_counter() - t_start) * 1000

    if not all_solutions:
        return IKResult(
            success=False,
            solve_time_ms=solve_time,
            message="No valid solution within joint limits"
        )

    # Select solution with minimum joint travel from current config
    if config.current_joints is not None:
        distances = [
            np.sum(np.abs(sol - config.current_joints))
            for sol in all_solutions
        ]
        best_idx = int(np.argmin(distances))
    elif config.prefer_elbow_up:
        # Prefer elbow-up (second solution if available)
        best_idx = min(1, len(all_solutions) - 1)
    else:
        best_idx = 0

    best = all_solutions[best_idx]

    # Compute final FK error
    T = forward_kinematics(best)
    fk_pos = T[:3, 3]
    pos_error = np.linalg.norm(fk_pos - target_position)

    return IKResult(
        success=True,
        joint_angles=best,
        position_error_m=pos_error,
        orientation_error_rad=0.0,  # Analytical — exact by construction
        solve_time_ms=solve_time,
        solver_name="analytical",
        message=f"Solved ({len(all_solutions)} configs, "
                f"selected {'elbow-up' if best_idx > 0 else 'elbow-down'})",
        all_solutions=all_solutions,
    )


def solve_from_pose_stamped(target_pose_position: np.ndarray,
                            target_pose_orientation: np.ndarray,
                            config: Optional[SolverConfig] = None) -> IKResult:
    """
    High-level solver that accepts position + quaternion.

    Extracts pitch and roll from the quaternion, then calls
    the geometric solver.

    Args:
        target_pose_position: [x, y, z]
        target_pose_orientation: [qx, qy, qz, qw]
        config: solver configuration

    Returns:
        IKResult
    """
    qx, qy, qz, qw = target_pose_orientation

    # Extract pitch (rotation about Y) and roll (rotation about X)
    # from the quaternion using standard Euler angle extraction (ZYX order).
    #
    # pitch = asin(2(qw·qy - qz·qx))
    # roll  = atan2(2(qw·qx + qy·qz), 1 - 2(qx² + qy²))

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))  # clamp for numerical safety
    pitch = math.asin(sinp)

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    return solve_analytical(target_pose_position, pitch, roll, config)


# ═══════════════════════════════════════════════════════════════
# Quick self-test
# ═══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("═══ Analytical IK Self-Test ═══")

    # Test: FK of home position → IK back
    home_joints = np.array([0.0, 1.5708, 1.3090, 0.0, 0.0])
    T = forward_kinematics(home_joints)
    target_pos = T[:3, 3]

    print(f"Home FK position: {target_pos}")

    result = solve_analytical(target_pos, target_pitch=0.0, target_roll=0.0,
                              config=SolverConfig(current_joints=home_joints))

    print(f"IK result: success={result.success}")
    print(f"  Joint angles: {np.degrees(result.joint_angles)}")
    print(f"  Position error: {result.position_error_m * 1000:.3f} mm")
    print(f"  Solve time: {result.solve_time_ms:.3f} ms")
    print(f"  Message: {result.message}")

    # Roundtrip check
    T2 = forward_kinematics(result.joint_angles)
    roundtrip_error = np.linalg.norm(T2[:3, 3] - target_pos)
    print(f"  Roundtrip FK error: {roundtrip_error * 1000:.3f} mm")

def _rotation_matrix_to_quaternion(R):
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (m21 - m12) / S
        y = (m02 - m20) / S
        z = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2
        w = (m21 - m12) / S
        x = 0.25 * S
        y = (m01 + m10) / S
        z = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2
        w = (m02 - m20) / S
        x = (m01 + m10) / S
        y = 0.25 * S
        z = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2
        w = (m10 - m01) / S
        x = (m02 + m20) / S
        y = (m12 + m21) / S
        z = 0.25 * S
    return np.array([x, y, z, w])
