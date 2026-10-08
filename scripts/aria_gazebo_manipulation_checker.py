#!/usr/bin/env python3
"""
scripts/aria_gazebo_manipulation_checker.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PROJECT ARIA — Manipulation Success Checker (Gazebo Physical Ground-Truth)

Enforces STRICT, empirical verification of manipulation tasks using ONLY
physical Gazebo measurements:
  - End-Effector (EE) pose from tf2_ros lookup_transform('base_link', 'wrist_link')
    (NEVER from forward_kinematics of arm_ik or joint angles).
  - Object poses from Gazebo /gazebo/model_states.
  - Pick & Lift: Object lifted >= 50 mm (0.050 m) AND continuously held >= 2.0 s.
  - Tray Placement: Object inside pocket within <= 5 mm (0.005 m) planar radius.
  - Stacking: Top object xy offset <= 3 mm (0.003 m) from base, z correct height,
              not toppled (|roll| < 15°, |pitch| < 15°).
  - Drawer / Slide: Linear displacement >= 30 mm (0.030 m).
  - Pivot Reorientation: Measured yaw change within <= 15° (0.2618 rad) of commanded.
  - Conveyor Sort: Object inside correct bin bounds, not tipped (|roll, pitch| < 20°).
  - Reach & Touch / Visual Alignment: TF EE position error <= 10 mm (0.010 m).

All thresholds are statically defined and stated before trial execution.
Gripper physical friction contact vs attach shortcut are tracked and reported separately.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


# ═════════════════════════════════════════════════════════════════════════════
# PRE-FLIGHT AUTHORITATIVE THRESHOLDS
# ═════════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class ManipulationThresholds:
    """Authoritative thresholds stated before running any benchmark."""
    # Pick & Lift
    PICK_MIN_LIFT_M: float = 0.050          # Object lifted >= 50 mm above initial table z
    PICK_MIN_HOLD_DURATION_S: float = 2.00  # Held lifted for >= 2.0 seconds

    # Tray Placement
    TRAY_MAX_PLANAR_OFFSET_M: float = 0.005 # Object inside pocket within <= 5 mm

    # Stacking
    STACK_MAX_XY_OFFSET_M: float = 0.003    # XY offset <= 3 mm relative to base object
    STACK_MIN_Z_DELTA_M: float = 0.025      # Top object sitting on base object (h=0.03m)
    STACK_MAX_Z_DELTA_M: float = 0.038      # Must not float or penetrate
    STACK_MAX_TILT_RAD: float = 0.2618      # Not toppled (|roll|, |pitch| < 15°)

    # Drawer / Slide
    DRAWER_MIN_DISPLACEMENT_M: float = 0.030 # Linear displacement >= 30 mm

    # Pivot Reorientation
    PIVOT_MAX_YAW_ERROR_RAD: float = 0.2618  # |measured_yaw_delta - commanded_yaw| <= 15° (0.2618 rad)

    # Conveyor Sort
    CONVEYOR_MAX_TILT_RAD: float = 0.3491   # Not tipped (|roll|, |pitch| < 20°)

    # Kinematic Reach & Visual Servoing
    EE_MAX_POSITION_ERROR_M: float = 0.010  # TF EE position error <= 10 mm

    @classmethod
    def print_declared_thresholds(cls):
        """State all authoritative thresholds to stdout BEFORE running."""
        print("═" * 78)
        print("PROJECT ARIA: AUTHORITATIVE PHYSICAL SUCCESS CHECKER THRESHOLDS")
        print("Enforced strictly via Gazebo ODE ground truth & tf2_ros reads:")
        print(f"  • EE Pose Measurement:     tf2_ros lookup (base_link -> wrist_link)")
        print(f"  • Object Pose Source:       Gazebo /gazebo/model_states")
        print(f"  • Pick & Lift:              Lifted Δz >= {cls.PICK_MIN_LIFT_M*1000:.1f} mm AND held >= {cls.PICK_MIN_HOLD_DURATION_S:.1f} s")
        print(f"  • Tray Placement:           Pocket planar offset <= {cls.TRAY_MAX_PLANAR_OFFSET_M*1000:.1f} mm")
        print(f"  • Stacking:                 XY offset <= {cls.STACK_MAX_XY_OFFSET_M*1000:.1f} mm, z correct, tilt < {math.degrees(cls.STACK_MAX_TILT_RAD):.1f}°")
        print(f"  • Drawer / Slide:           Displacement >= {cls.DRAWER_MIN_DISPLACEMENT_M*1000:.1f} mm")
        print(f"  • Pivot Reorientation:      |Δyaw_meas - Δyaw_cmd| <= {math.degrees(cls.PIVOT_MAX_YAW_ERROR_RAD):.1f}°")
        print(f"  • Conveyor Sort:            In target bin, not tipped (tilt < {math.degrees(cls.CONVEYOR_MAX_TILT_RAD):.1f}°)")
        print(f"  • Reach & Alignment:        TF EE position error <= {cls.EE_MAX_POSITION_ERROR_M*1000:.1f} mm")
        print(f"  • Gripper Policy:           Physical contact friction vs attach reported separately")
        print("═" * 78 + "\n")


THRESHOLDS = ManipulationThresholds()


# ═════════════════════════════════════════════════════════════════════════════
# PHYSICAL SUCCESS EVALUATION RESULTS
# ═════════════════════════════════════════════════════════════════════════════
@dataclass
class EvaluationResult:
    """Result of a task execution evaluated strictly from Gazebo ODE / TF."""
    success: bool
    outcome: str                           # "SUCCESS" or "FAILURE"
    failure_category: str                  # "NONE", "B_GRASP_SLIP", "C_IK_FAILURE", "D_TOLERANCE_EXCEEDED", etc.
    metric_value: float                    # Primary metric (e.g. lift height, planar error mm, yaw error deg)
    metric_threshold: float                # Declared threshold for comparison
    tf_ee_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tf_ee_err_mm: float = 0.0
    object_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    object_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    physical_contact_only_held: bool = False
    attach_assisted: bool = False
    details: str = ""


class GazeboManipulationSuccessChecker:
    """
    Physical Ground-Truth Checker.
    All evaluations accept ONLY measured TF transforms and Gazebo ModelStates.
    """

    @staticmethod
    def check_reach_touch(tf_ee_pos: np.ndarray, target_pos: np.ndarray) -> EvaluationResult:
        """Task 01: Reach & Touch Object via TF EE lookup."""
        err_m = float(np.linalg.norm(tf_ee_pos - target_pos))
        err_mm = err_m * 1000.0
        succ = (err_m <= THRESHOLDS.EE_MAX_POSITION_ERROR_M)
        return EvaluationResult(
            success=succ,
            outcome="SUCCESS" if succ else "FAILURE",
            failure_category="NONE" if succ else "C_IK_FAILURE",
            metric_value=err_mm,
            metric_threshold=THRESHOLDS.EE_MAX_POSITION_ERROR_M * 1000.0,
            tf_ee_pos=tf_ee_pos,
            tf_ee_err_mm=err_mm,
            details=f"TF EE error={err_mm:.2f} mm (limit={THRESHOLDS.EE_MAX_POSITION_ERROR_M*1000:.1f} mm)"
        )

    @staticmethod
    def check_pick_and_lift(initial_obj_z: float,
                            lifted_z_samples: List[Tuple[float, float]], # (timestamp, z_pos)
                            tf_ee_pos: np.ndarray,
                            physical_contact_only: bool,
                            attach_used: bool) -> EvaluationResult:
        """
        Task 02: Pick & Lift Workpiece.
        Condition: Lifted >= 50 mm and held >= 2.0 s.
        """
        if not lifted_z_samples:
            return EvaluationResult(
                success=False,
                outcome="FAILURE",
                failure_category="B_GRASP_SLIP",
                metric_value=0.0,
                metric_threshold=THRESHOLDS.PICK_MIN_LIFT_M * 1000.0,
                tf_ee_pos=tf_ee_pos,
                physical_contact_only_held=False,
                attach_assisted=attach_used,
                details="No Gazebo object samples captured during lift"
            )

        max_lift_m = max(s[1] - initial_obj_z for s in lifted_z_samples)
        min_required_z = initial_obj_z + THRESHOLDS.PICK_MIN_LIFT_M

        # Calculate continuous hold duration above min_required_z
        held_duration_s = 0.0
        current_streak_start: Optional[float] = None
        max_held_duration_s = 0.0

        for t, z in lifted_z_samples:
            if z >= min_required_z:
                if current_streak_start is None:
                    current_streak_start = t
                streak = t - current_streak_start
                if streak > max_held_duration_s:
                    max_held_duration_s = streak
            else:
                current_streak_start = None

        held_ok = (max_held_duration_s >= THRESHOLDS.PICK_MIN_HOLD_DURATION_S)
        succ = (max_lift_m >= THRESHOLDS.PICK_MIN_LIFT_M) and held_ok
        phys_held = succ and physical_contact_only and (not attach_used)

        return EvaluationResult(
            success=succ,
            outcome="SUCCESS" if succ else "FAILURE",
            failure_category="NONE" if succ else "B_GRASP_SLIP",
            metric_value=max_lift_m * 1000.0,
            metric_threshold=THRESHOLDS.PICK_MIN_LIFT_M * 1000.0,
            tf_ee_pos=tf_ee_pos,
            physical_contact_only_held=phys_held,
            attach_assisted=attach_used,
            details=f"Lifted {max_lift_m*1000:.1f} mm (limit={THRESHOLDS.PICK_MIN_LIFT_M*1000:.1f} mm), held {max_held_duration_s:.2f}s (req={THRESHOLDS.PICK_MIN_HOLD_DURATION_S:.1f}s)"
        )

    @staticmethod
    def check_tray_placement(obj_final_pos: np.ndarray,
                             pocket_center_xy: Tuple[float, float],
                             tf_ee_pos: np.ndarray,
                             attach_used: bool = False) -> EvaluationResult:
        """
        Task 04: Precision Placement into Tray.
        Condition: Object inside pocket within <= 5 mm planar radius.
        """
        dx = obj_final_pos[0] - pocket_center_xy[0]
        dy = obj_final_pos[1] - pocket_center_xy[1]
        dist_m = math.sqrt(dx * dx + dy * dy)
        dist_mm = dist_m * 1000.0

        succ = (dist_m <= THRESHOLDS.TRAY_MAX_PLANAR_OFFSET_M)
        return EvaluationResult(
            success=succ,
            outcome="SUCCESS" if succ else "FAILURE",
            failure_category="NONE" if succ else "D_TOLERANCE_EXCEEDED",
            metric_value=dist_mm,
            metric_threshold=THRESHOLDS.TRAY_MAX_PLANAR_OFFSET_M * 1000.0,
            tf_ee_pos=tf_ee_pos,
            object_pos=obj_final_pos,
            attach_assisted=attach_used,
            details=f"Pocket xy offset={dist_mm:.2f} mm (limit={THRESHOLDS.TRAY_MAX_PLANAR_OFFSET_M*1000:.1f} mm)"
        )

    @staticmethod
    def check_stacking(top_obj_pos: np.ndarray,
                       top_obj_rpy: np.ndarray,
                       base_obj_pos: np.ndarray,
                       tf_ee_pos: np.ndarray) -> EvaluationResult:
        """
        Task 09: Multi-Object Stacking.
        Condition: xy offset <= 3 mm, z correct, not toppled (|roll, pitch| < 15°).
        """
        d_xy_m = math.sqrt((top_obj_pos[0] - base_obj_pos[0])**2 + (top_obj_pos[1] - base_obj_pos[1])**2)
        d_xy_mm = d_xy_m * 1000.0
        dz_m = top_obj_pos[2] - base_obj_pos[2]

        xy_ok = (d_xy_m <= THRESHOLDS.STACK_MAX_XY_OFFSET_M)
        z_ok = (THRESHOLDS.STACK_MIN_Z_DELTA_M <= dz_m <= THRESHOLDS.STACK_MAX_Z_DELTA_M)
        tilt_ok = (abs(top_obj_rpy[0]) < THRESHOLDS.STACK_MAX_TILT_RAD and
                   abs(top_obj_rpy[1]) < THRESHOLDS.STACK_MAX_TILT_RAD)

        succ = xy_ok and z_ok and tilt_ok
        fail_cat = "NONE" if succ else ("B_GRASP_SLIP" if not tilt_ok else "D_TOLERANCE_EXCEEDED")

        return EvaluationResult(
            success=succ,
            outcome="SUCCESS" if succ else "FAILURE",
            failure_category=fail_cat,
            metric_value=d_xy_mm,
            metric_threshold=THRESHOLDS.STACK_MAX_XY_OFFSET_M * 1000.0,
            tf_ee_pos=tf_ee_pos,
            object_pos=top_obj_pos,
            object_rpy=top_obj_rpy,
            details=f"Stack xy offset={d_xy_mm:.2f} mm (lim={THRESHOLDS.STACK_MAX_XY_OFFSET_M*1000:.1f} mm), Δz={dz_m*1000:.1f} mm, roll={math.degrees(top_obj_rpy[0]):.1f}°, pitch={math.degrees(top_obj_rpy[1]):.1f}°"
        )

    @staticmethod
    def check_drawer(initial_pos: np.ndarray,
                     final_pos: np.ndarray,
                     tf_ee_pos: np.ndarray) -> EvaluationResult:
        """
        Task 08: Articulated Drawer/Slide Interaction.
        Condition: Linear displacement >= 30 mm.
        """
        disp_m = float(np.linalg.norm(final_pos[:2] - initial_pos[:2]))
        disp_mm = disp_m * 1000.0
        succ = (disp_m >= THRESHOLDS.DRAWER_MIN_DISPLACEMENT_M)

        return EvaluationResult(
            success=succ,
            outcome="SUCCESS" if succ else "FAILURE",
            failure_category="NONE" if succ else "D_TOLERANCE_EXCEEDED",
            metric_value=disp_mm,
            metric_threshold=THRESHOLDS.DRAWER_MIN_DISPLACEMENT_M * 1000.0,
            tf_ee_pos=tf_ee_pos,
            details=f"Drawer displacement={disp_mm:.1f} mm (req={THRESHOLDS.DRAWER_MIN_DISPLACEMENT_M*1000:.1f} mm)"
        )

    @staticmethod
    def check_pivot(initial_yaw_rad: float,
                    final_yaw_rad: float,
                    commanded_yaw_delta_rad: float,
                    tf_ee_pos: np.ndarray) -> EvaluationResult:
        """
        Task 07: Multi-Axis In-Hand Reorientation / Pivot.
        Condition: Measured yaw change within <= 15° (0.2618 rad) of commanded.
        """
        meas_delta = math.atan2(math.sin(final_yaw_rad - initial_yaw_rad),
                                math.cos(final_yaw_rad - initial_yaw_rad))
        yaw_err_rad = abs(meas_delta - commanded_yaw_delta_rad)
        yaw_err_deg = math.degrees(yaw_err_rad)
        succ = (yaw_err_rad <= THRESHOLDS.PIVOT_MAX_YAW_ERROR_RAD)

        return EvaluationResult(
            success=succ,
            outcome="SUCCESS" if succ else "FAILURE",
            failure_category="NONE" if succ else "D_TOLERANCE_EXCEEDED",
            metric_value=yaw_err_deg,
            metric_threshold=math.degrees(THRESHOLDS.PIVOT_MAX_YAW_ERROR_RAD),
            tf_ee_pos=tf_ee_pos,
            details=f"Measured Δyaw={math.degrees(meas_delta):.1f}°, cmd={math.degrees(commanded_yaw_delta_rad):.1f}°, err={yaw_err_deg:.1f}° (limit={math.degrees(THRESHOLDS.PIVOT_MAX_YAW_ERROR_RAD):.1f}°)"
        )

    @staticmethod
    def check_conveyor_sort(final_obj_pos: np.ndarray,
                            final_obj_rpy: np.ndarray,
                            target_bin_bounds: Tuple[float, float, float, float], # xmin, xmax, ymin, ymax
                            tf_ee_pos: np.ndarray) -> EvaluationResult:
        """
        Task 03: Conveyor Dynamic Rendezvous / Sort.
        Condition: Object in correct bin, not tipped (|roll, pitch| < 20°).
        """
        xmin, xmax, ymin, ymax = target_bin_bounds
        in_bin = (xmin <= final_obj_pos[0] <= xmax) and (ymin <= final_obj_pos[1] <= ymax)
        tilt_ok = (abs(final_obj_rpy[0]) < THRESHOLDS.CONVEYOR_MAX_TILT_RAD and
                   abs(final_obj_rpy[1]) < THRESHOLDS.CONVEYOR_MAX_TILT_RAD)
        succ = in_bin and tilt_ok
        fail_cat = "NONE" if succ else ("B_GRASP_SLIP" if not tilt_ok else "D_TOLERANCE_EXCEEDED")

        return EvaluationResult(
            success=succ,
            outcome="SUCCESS" if succ else "FAILURE",
            failure_category=fail_cat,
            metric_value=1.0 if succ else 0.0,
            metric_threshold=1.0,
            tf_ee_pos=tf_ee_pos,
            object_pos=final_obj_pos,
            object_rpy=final_obj_rpy,
            details=f"In bin: {in_bin} (bounds X[{xmin:.2f},{xmax:.2f}], Y[{ymin:.2f},{ymax:.2f}]), roll={math.degrees(final_obj_rpy[0]):.1f}°, pitch={math.degrees(final_obj_rpy[1]):.1f}°"
        )
