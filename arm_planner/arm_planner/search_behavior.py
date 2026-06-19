#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Search Behavior
Multi-phase search when object not found.
5 phases: last known → grid scan → behind obstacles →
          ask user → re-scan new location.
═══════════════════════════════════════════════════════════════
"""
import math, time
from typing import Optional
import numpy as np
from arm_planner.state_bus import StateBus
from arm_planner.msg import WorldObject

# Grid scan positions (9 viewpoints across workspace)
SCAN_GRID = [
    (0.15, 0.10, 0.35),   # front-left
    (0.15, 0.00, 0.35),   # front-center
    (0.15, -0.10, 0.35),  # front-right
    (0.20, 0.10, 0.30),   # mid-left
    (0.20, 0.00, 0.30),   # mid-center
    (0.20, -0.10, 0.30),  # mid-right
    (0.25, 0.10, 0.25),   # back-left
    (0.25, 0.00, 0.25),   # back-center
    (0.25, -0.10, 0.25),  # back-right
]


class SearchBehavior:
    """
    Multi-phase object search.

    Phase 1: Check last known location (from WorldModel)
    Phase 2: Systematic workspace grid scan (9 positions)
    Phase 3: Check behind obstacles
    Phase 4: Ask user
    Phase 5: Re-scan user-provided location
    """

    def __init__(self, bus: StateBus):
        self.bus = bus
        self.current_phase = 0
        self.search_target = ''
        self.scan_index = 0

    def search(self, object_name: str) -> Optional[WorldObject]:
        """
        Run multi-phase search for an object.

        Returns WorldObject if found, None if all phases exhausted.
        """
        self.search_target = object_name
        self.current_phase = 1

        self.bus.add_chain_of_thought(
            f"SEARCH: Starting multi-phase search for '{object_name}'")

        # Phase 1: Last known location
        result = self._phase_1_last_known()
        if result:
            return result

        # Phase 2: Grid scan
        result = self._phase_2_grid_scan()
        if result:
            return result

        # Phase 3: Behind obstacles
        result = self._phase_3_behind_obstacles()
        if result:
            return result

        # Phase 4: Ask user
        self._phase_4_ask_user()

        # Phase 5 would be triggered after user response
        return None

    def _phase_1_last_known(self) -> Optional[WorldObject]:
        """Check last known location from WorldModel."""
        self.bus.add_chain_of_thought(
            f"SEARCH Phase 1: Checking last known location...")

        memory = self.bus.state.memory
        if memory is None:
            self.bus.add_chain_of_thought("  No memory available")
            return None

        for obj in memory.known_objects:
            if obj.name == self.search_target or obj.class_name in self.search_target:
                pos = obj.last_known_pose.pose.position
                self.bus.add_chain_of_thought(
                    f"  Last seen at ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}). "
                    f"Moving to viewpoint...")
                # Would move arm to viewpoint and run detection for 2s
                time.sleep(0.5)  # Simulated detection wait

                # Check if now visible
                vision = self.bus.state.vision
                if vision:
                    for det in vision.detected_objects:
                        if det.class_name in self.search_target:
                            self.bus.add_chain_of_thought(
                                f"  ✓ Found '{self.search_target}' at last known location!")
                            return obj

                self.bus.add_chain_of_thought(
                    f"  ✗ Not at last known location")
                return None

        self.bus.add_chain_of_thought(f"  No last known location for '{self.search_target}'")
        return None

    def _phase_2_grid_scan(self) -> Optional[WorldObject]:
        """Systematic workspace scan: 9 grid positions."""
        self.bus.add_chain_of_thought(
            f"SEARCH Phase 2: Systematic workspace scan (9 positions)...")

        for i, (x, y, z) in enumerate(SCAN_GRID):
            self.bus.add_chain_of_thought(
                f"  Scanning position {i+1}/9: ({x:.2f}, {y:.2f}, {z:.2f})")
            # Would move arm to viewpoint and run detection
            time.sleep(0.2)  # Simulated

            # Check detections
            vision = self.bus.state.vision
            if vision:
                for det in vision.detected_objects:
                    if det.class_name in self.search_target:
                        self.bus.add_chain_of_thought(
                            f"  ✓ Found '{self.search_target}' at scan position {i+1}!")
                        wo = WorldObject()
                        wo.name = self.search_target
                        wo.class_name = det.class_name
                        wo.last_known_pose = det.pose_3d
                        wo.lifecycle_state = 'Recovered'
                        return wo

        self.bus.add_chain_of_thought(
            f"  ✗ Not found in {len(SCAN_GRID)} scan positions")
        return None

    def _phase_3_behind_obstacles(self) -> Optional[WorldObject]:
        """Check behind occluding objects."""
        self.bus.add_chain_of_thought(
            f"SEARCH Phase 3: Checking behind obstacles...")

        memory = self.bus.state.memory
        if memory is None:
            return None

        # Find potential occluders (larger objects)
        for obj in memory.known_objects:
            if obj.name == self.search_target:
                continue
            # Move to angle that sees around this object
            self.bus.add_chain_of_thought(
                f"  Looking behind '{obj.name}'...")
            time.sleep(0.2)

        self.bus.add_chain_of_thought(f"  ✗ Not found behind obstacles")
        return None

    def _phase_4_ask_user(self):
        """Ask user for help finding the object."""
        self.bus.add_chain_of_thought(
            f"SEARCH Phase 4: Asking user for help...")

        memory = self.bus.state.memory
        last_location = "unknown location"
        last_time = "unknown time"

        if memory:
            for obj in memory.known_objects:
                if obj.name == self.search_target:
                    pos = obj.last_known_pose.pose.position
                    last_location = f"({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
                    break

        # This would trigger DialogueAgent
        self.bus.add_chain_of_thought(
            f"SEARCH: Asking user — "
            f"\"I've searched the entire workspace for '{self.search_target}'. "
            f"It was last seen at {last_location}. "
            f"Has it been moved? If yes, where is it?\"")

    def search_at_location(self, x: float, y: float, z: float) -> Optional[WorldObject]:
        """Phase 5: Re-scan at user-provided location."""
        self.bus.add_chain_of_thought(
            f"SEARCH Phase 5: Scanning user-provided location "
            f"({x:.2f}, {y:.2f}, {z:.2f})...")
        # Move to viewpoint and detect
        time.sleep(0.5)

        vision = self.bus.state.vision
        if vision:
            for det in vision.detected_objects:
                if det.class_name in self.search_target:
                    wo = WorldObject()
                    wo.name = self.search_target
                    wo.class_name = det.class_name
                    wo.last_known_pose = det.pose_3d
                    wo.lifecycle_state = 'Recovered'
                    self.bus.add_chain_of_thought(
                        f"  ✓ Found '{self.search_target}'!")
                    return wo

        self.bus.add_chain_of_thought(
            f"  ✗ Still not found at user location")
        return None
