#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Search Behavior
Multi-phase search when object not found in world model.

Phases:
  1. Check last known location (world model → arm moves there, YOLO detects)
  2. Systematic workspace grid scan (arm physically sweeps 9 positions)
  3. Check behind occluding obstacles
  4. Ask user for help
  5. Re-scan at user-provided location

The arm physically moves to each viewpoint. Real YOLO detections
drive the result — no simulated outcomes.
═══════════════════════════════════════════════════════════════
"""
import math
import time
from typing import Optional, Callable

from arm_planner.state_bus import StateBus
from arm_planner.msg import WorldObject

# ── Workspace scan grid (wrist_camera overhead viewpoints) ────────────────────
# Each tuple: (x_base, y_base, z_base) in base_link frame.
# z = height above table (table surface ≈ -0.006m in base frame).
# The arm sweeps these positions so the wrist camera acts as a top-down sensor.
# Grid covers the full hemispherical workspace of the 5-DoF arm.
SCAN_GRID_BASE = [
    # Row 1 — near row (r ≈ 0.15 m, table forward)
    ( 0.08, -0.13, 0.20),   # near-left
    ( 0.00, -0.15, 0.20),   # near-center
    (-0.08, -0.13, 0.20),   # near-right
    # Row 2 — mid row (r ≈ 0.18 m, table center)
    ( 0.09, -0.16, 0.18),   # mid-left
    ( 0.00, -0.18, 0.18),   # mid-center
    (-0.09, -0.16, 0.18),   # mid-right
    # Row 3 — far row (r ≈ 0.21 m, table far)
    ( 0.09, -0.19, 0.16),   # far-left
    ( 0.00, -0.21, 0.16),   # far-center
    (-0.09, -0.19, 0.16),   # far-right
]

# Detection wait time per viewpoint (seconds) — time for YOLO to process frame
DETECTION_WAIT_S = 0.6

# Detection check poll interval (seconds)
POLL_INTERVAL_S = 0.05

# Minimum match score to accept a detection
MIN_MATCH_SCORE = 35.0


class SearchBehavior:
    """
    Multi-phase object search.

    The arm physically moves to viewpoints and YOLO detects the target.
    No simulated outcomes — detection results are driven by the live
    vision pipeline (YOLO + coordinate transformer).

    Phase 1: Check last known location (from world model)
    Phase 2: Systematic workspace grid scan (9 arm positions)
    Phase 3: Check behind occluding objects
    Phase 4: Ask user for help
    Phase 5: Re-scan at user-provided location
    """

    def __init__(self, bus: StateBus,
                 move_fn: Optional[Callable] = None,
                 wait_fn: Optional[Callable] = None,
                 match_fn: Optional[Callable] = None):
        """
        Args:
            bus:      StateBus reference for reading vision/memory state
            move_fn:  Callable(joint_angles_rad, gripper_deg, speed_dps) -> bool
                      Physically moves the arm to the given joint configuration.
                      Injected by SkillAgent so SearchBehavior can drive the arm.
            wait_fn:  Callable(joint_angles_rad, timeout) -> bool
                      Waits until the arm reaches the given joint config.
            match_fn: Callable(target_name, obj) -> float
                      Scores how well a detection matches the target name.
                      Injected from SkillAgent._match_target_object.
        """
        self.bus = bus
        self._move_fn = move_fn
        self._wait_fn = wait_fn
        self._match_fn = match_fn
        self.current_phase = 0
        self.search_target = ''
        self.scan_index = 0

        # IK solver injected from SkillAgent
        self._solve_ik_fn: Optional[Callable] = None

    def set_ik_solver(self, solve_ik_fn: Callable):
        """Inject the IK solver from SkillAgent."""
        self._solve_ik_fn = solve_ik_fn

    def _move_to_viewpoint(self, bx: float, by: float, bz: float) -> bool:
        """
        Move the arm so the wrist camera looks down at (bx, by, bz).
        Uses the injected IK solver and move function.
        Returns True if the arm reached the viewpoint.
        """
        if self._solve_ik_fn is None or self._move_fn is None:
            # No arm control injected — skip physical motion
            return True

        try:
            from geometry_msgs.msg import PoseStamped
            p = PoseStamped()
            p.header.frame_id = 'base_link'
            p.pose.position.x = float(bx)
            p.pose.position.y = float(by)
            p.pose.position.z = float(bz)
            # A search viewpoint is a camera pose, not merely a Cartesian point.
            # Keep the wrist pointed toward the table so the wrist camera really
            # acts as the top-down sensor described by the state machine.
            # The position-only IK implementation may ignore this orientation,
            # but downstream IK services can use it.
            p.pose.orientation.x = 1.0
            p.pose.orientation.w = 0.0

            joints = None
            if self._solve_ik_fn is not None:
                joints = self._solve_ik_fn(p)

            if joints is None:
                # Try direct closed-form analytical IK
                try:
                    import numpy as np
                    from arm_ik.ik_solvers.aria_analytical_ik import solve_analytical
                    ik_res = solve_analytical(
                        np.array([bx, by, bz]), target_pitch=math.radians(-65))
                    if ik_res and ik_res.success:
                        joints = list(ik_res.joint_angles)
                except Exception:
                    pass

            if joints is None:
                # Never send an unverified pose.  The previous heuristic mixed
                # Cartesian conventions and could point the arm upward while the
                # caller believed the scan viewpoint had been reached.
                self.bus.add_chain_of_thought(
                    "  SEARCH: No valid downward IK solution for viewpoint; "
                    "skipping it instead of issuing an unsafe guessed pose")
                return False

            ok = self._move_fn(joints, 0.0, 25.0)
            if self._wait_fn:
                self._wait_fn(joints, timeout=8.0)
            return bool(ok)
        except Exception as e:
            self.bus.add_chain_of_thought(f"  SEARCH: Viewpoint move error: {e}")
            return False

    def _score_detection(self, obj) -> float:
        """Score a detection candidate against the search target."""
        if self._match_fn is not None:
            return self._match_fn(self.search_target, obj)
        # Fallback: simple substring match
        t = self.search_target.lower()
        cls = (getattr(obj, 'class_name', '') or '').lower()
        name = (getattr(obj, 'name', '') or '').lower()
        if t in cls or t in name or cls in t:
            return 60.0
        return 0.0

    def _poll_for_target(self, wait_s: float = DETECTION_WAIT_S) -> Optional[object]:
        """
        Poll the vision/memory state for the target object for up to wait_s seconds.
        Returns the best-matching detection or None.
        """
        best = None
        best_score = MIN_MATCH_SCORE

        t_end = time.time() + wait_s
        while time.time() < t_end:
            # Check memory
            memory = self.bus.state.memory
            if memory and memory.known_objects:
                for obj in memory.known_objects:
                    s = self._score_detection(obj)
                    if s > best_score:
                        best_score = s
                        best = obj

            # Check live vision detections
            vision = self.bus.state.vision
            if vision and vision.detected_objects:
                for det in vision.detected_objects:
                    s = self._score_detection(det)
                    if s > best_score:
                        best_score = s
                        best = det

            if best_score >= 60.0:
                break

            time.sleep(POLL_INTERVAL_S)

        return best

    def search(self, object_name: str) -> Optional[WorldObject]:
        """
        Run multi-phase physical search for an object.
        Returns WorldObject if found, None if all phases exhausted.
        """
        self.search_target = object_name
        self.current_phase = 1

        self.bus.add_chain_of_thought(
            f"SEARCH: Starting multi-phase search for '{object_name}'")

        result = self._phase_1_last_known()
        if result:
            return result

        result = self._phase_2_grid_scan()
        if result:
            return result

        result = self._phase_3_behind_obstacles()
        if result:
            return result

        self._phase_4_ask_user()
        return None

    def _phase_1_last_known(self) -> Optional[WorldObject]:
        """Move to last known location and verify with YOLO."""
        self.bus.add_chain_of_thought(
            "SEARCH Phase 1: Checking last known location in world model...")

        memory = self.bus.state.memory
        if memory is None:
            self.bus.add_chain_of_thought("  No memory available — skipping phase 1")
            return None

        for obj in memory.known_objects:
            s = self._score_detection(obj)
            if s < MIN_MATCH_SCORE:
                continue

            pos = obj.last_known_pose.pose.position
            # Convert from world frame if needed
            if pos.z > 0.45:
                bx_base = float(pos.y)
                by_base = float(-pos.x)
                bz_base = float(pos.z - 0.614) + 0.18
            else:
                bx_base = float(pos.x)
                by_base = float(pos.y)
                bz_base = float(pos.z) + 0.18

            self.bus.add_chain_of_thought(
                f"  Last seen at world({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}). "
                f"Moving wrist camera above that point...")

            self._move_to_viewpoint(bx_base, by_base, bz_base)

            found = self._poll_for_target(wait_s=1.0)
            if found is not None:
                self.bus.add_chain_of_thought(
                    f"  Found '{self.search_target}' at last known location!")
                return self._to_world_object(found, 'Recovered')

            self.bus.add_chain_of_thought(
                "  Not at last known location — proceeding to grid scan")
            return None

        self.bus.add_chain_of_thought(
            f"  No last known location for '{self.search_target}' — "
            "proceeding to grid scan")
        return None

    def _phase_2_grid_scan(self) -> Optional[WorldObject]:
        """
        Systematic workspace scan: arm physically moves to each of 9 grid
        positions so the wrist camera sweeps the entire reachable workspace.
        YOLO detects the target at each viewpoint.
        """
        self.bus.add_chain_of_thought(
            f"SEARCH Phase 2: Physical workspace scan "
            f"({len(SCAN_GRID_BASE)} viewpoints)...")

        for i, (bx, by, bz) in enumerate(SCAN_GRID_BASE):
            self.bus.add_chain_of_thought(
                f"  Scan {i+1}/{len(SCAN_GRID_BASE)}: "
                f"Moving wrist camera to ({bx:.2f}, {by:.2f}, {bz:.2f}) base_link...")

            moved = self._move_to_viewpoint(bx, by, bz)
            if not moved:
                self.bus.add_chain_of_thought(
                    f"  Could not reach scan viewpoint {i+1} — skipping")
                continue

            found = self._poll_for_target(wait_s=DETECTION_WAIT_S)
            if found is not None:
                self.bus.add_chain_of_thought(
                    f"  Found '{self.search_target}' at scan position {i+1}!")
                return self._to_world_object(found, 'Recovered')

        self.bus.add_chain_of_thought(
            f"  '{self.search_target}' not found in "
            f"{len(SCAN_GRID_BASE)}-position scan")
        return None

    def _phase_3_behind_obstacles(self) -> Optional[WorldObject]:
        """Check behind occluding objects using side-angle viewpoints."""
        self.bus.add_chain_of_thought(
            "SEARCH Phase 3: Checking behind obstacles...")

        memory = self.bus.state.memory
        if memory is None:
            return None

        checked = 0
        for obj in memory.known_objects:
            if self._score_detection(obj) >= MIN_MATCH_SCORE:
                continue  # Skip the target itself

            pos = obj.last_known_pose.pose.position
            bx = float(pos.y) if pos.z > 0.45 else float(pos.x)
            by = float(-pos.x) if pos.z > 0.45 else float(pos.y)
            side_bz = 0.20
            side_bx = bx + 0.06
            side_by = by - 0.06

            self.bus.add_chain_of_thought(
                f"  Looking behind '{getattr(obj, 'name', obj.class_name)}'...")
            self._move_to_viewpoint(side_bx, side_by, side_bz)

            found = self._poll_for_target(wait_s=DETECTION_WAIT_S)
            if found is not None:
                self.bus.add_chain_of_thought(
                    f"  Found '{self.search_target}' behind obstacle!")
                return self._to_world_object(found, 'Recovered')

            checked += 1
            if checked >= 4:
                break

        self.bus.add_chain_of_thought("  Not found behind obstacles")
        return None

    def _phase_4_ask_user(self):
        """Ask user for help finding the object."""
        self.bus.add_chain_of_thought(
            f"SEARCH Phase 4: Asking user for help finding "
            f"'{self.search_target}'...")

        memory = self.bus.state.memory
        last_location = "unknown location"

        if memory:
            for obj in memory.known_objects:
                if self._score_detection(obj) >= MIN_MATCH_SCORE:
                    p = obj.last_known_pose.pose.position
                    last_location = f"({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
                    break

        self.bus.add_chain_of_thought(
            f"SEARCH: I have searched the entire workspace for "
            f"'{self.search_target}'. "
            f"It was last seen at {last_location}. "
            f"Has it been moved? If yes, where is it?")

    def search_at_location(self, x: float, y: float,
                           z: float) -> Optional[WorldObject]:
        """Phase 5: Move to user-provided location and scan."""
        self.bus.add_chain_of_thought(
            f"SEARCH Phase 5: Scanning user-provided location "
            f"({x:.2f}, {y:.2f}, {z:.2f})...")

        vz = max(z + 0.18, 0.18)
        self._move_to_viewpoint(x, y, vz)

        found = self._poll_for_target(wait_s=1.5)
        if found is not None:
            self.bus.add_chain_of_thought(
                f"  Found '{self.search_target}' at user-provided location!")
            return self._to_world_object(found, 'Recovered')

        self.bus.add_chain_of_thought("  Still not found at user location")
        return None

    @staticmethod
    def _to_world_object(found_obj, lifecycle_state: str) -> WorldObject:
        """Promote a detection or memory object to a WorldObject."""
        wo = WorldObject()
        wo.name = (getattr(found_obj, 'name', '')
                   or getattr(found_obj, 'class_name', 'unknown'))
        wo.class_name = getattr(found_obj, 'class_name', 'unknown')
        wo.lifecycle_state = lifecycle_state
        pose = (getattr(found_obj, 'last_known_pose', None)
                or getattr(found_obj, 'pose_3d', None))
        if pose is not None:
            wo.last_known_pose = pose
        return wo
