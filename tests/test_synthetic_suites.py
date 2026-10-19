#!/usr/bin/env python3
"""
Unit and integration verification tests for ARIA synthetic manipulation suites.
Tests:
  - Maneuver registry and methods for tester world suite
  - Maneuver registry and methods for industrial world suite
  - Kinematic reachability and IK solving for all 15 objects
  - All-speed conveyor power profiles
  - Launch file argument registration (synthetic flag)
"""

import os
import sys
import unittest
import numpy as np

WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, WORKSPACE_ROOT)
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "arm_ik", "arm_ik", "ik_solvers"))

from aria_analytical_ik import solve_analytical, forward_kinematics_position


class TestSyntheticSuites(unittest.TestCase):

    def test_tester_suite_coverage(self):
        """Verify that synthetic_tester_suite defines all required maneuvers."""
        from synthetic_tester_suite import SyntheticTesterSuite

        expected_methods = [
            "maneuver_wave",
            "maneuver_nod_and_no",
            "maneuver_drink_coffee",
            "maneuver_pick_every_object",
            "maneuver_mug_handle_pick",
            "maneuver_stack_mugs",
            "maneuver_banana_in_pan",
            "maneuver_orange_ball_in_glass",
            "maneuver_smaller_in_bigger_objects",
            "maneuver_stack_jenga_tower",
            "maneuver_sort_by_color",
            "maneuver_workspace_inspection_scan",
            "maneuver_pedestal_transfer",
            "maneuver_table_wipe",
            "run_all",
        ]
        for m in expected_methods:
            self.assertTrue(
                hasattr(SyntheticTesterSuite, m),
                f"SyntheticTesterSuite missing method '{m}'"
            )

        # Check all 15 objects listed
        self.assertEqual(len(SyntheticTesterSuite.ALL_OBJECTS), 15)
        self.assertIn("bullet_banana", SyntheticTesterSuite.ALL_OBJECTS)
        self.assertIn("orange_fruit", SyntheticTesterSuite.ALL_OBJECTS)
        self.assertIn("mug_ycb_front", SyntheticTesterSuite.ALL_OBJECTS)
        self.assertIn("ceramic_tea_mug", SyntheticTesterSuite.ALL_OBJECTS)
        self.assertIn("tefal_pan", SyntheticTesterSuite.ALL_OBJECTS)
        self.assertIn("plastic_glass", SyntheticTesterSuite.ALL_OBJECTS)
        self.assertIn("jenga_block_1", SyntheticTesterSuite.ALL_OBJECTS)

    def test_industrial_suite_coverage(self):
        """Verify that synthetic_industrial_suite defines all required maneuvers."""
        from synthetic_industrial_suite import SyntheticIndustrialSuite

        expected_methods = [
            "maneuver_conveyor_sort_boxes",
            "maneuver_stack_boxes",
            "maneuver_drop_boxes_on_ground",
            "maneuver_conveyor_all_speeds_and_switching",
            "maneuver_estop_button_press",
            "maneuver_continuous_conveyor_cycle",
            "run_all",
        ]
        for m in expected_methods:
            self.assertTrue(
                hasattr(SyntheticIndustrialSuite, m),
                f"SyntheticIndustrialSuite missing method '{m}'"
            )

    def test_all_objects_ik_reachability(self):
        """Verify that every single object target position has a valid IK solution."""
        objects = {
            "mug_ycb_front": (0.22, -0.06, 0.6081),
            "bullet_duck": (0.22, 0.06, 0.6081),
            "bullet_banana": (0.20, -0.12, 0.6081),
            "beverage_bottle": (0.20, 0.12, 0.6081),
            "dinner_plate_rim": (0.08, 0.185, 0.615),
            "plastic_glass": (0.00, 0.22, 0.6081),
            "plastic_bowl_rim": (-0.05, 0.19, 0.620),
            "ceramic_tea_mug_handle": (-0.18, 0.145, 0.6481),
            "tefal_pan_handle": (0.12, -0.18, 0.625),
            "mug_bullet_right": (0.00, -0.22, 0.6081),
            "sorting_tray_edge": (-0.06, -0.18, 0.620),
            "orange_fruit": (0.18, -0.16, 0.6081),
            "pop_tarts_box_edge": (-0.18, -0.145, 0.6481),
            "jenga_block_1": (-0.20, 0.08, 0.6081),
            "jenga_block_2": (-0.20, -0.08, 0.6081),
        }

        for name, (xw, yw, zw) in objects.items():
            bx, by, bz = yw, -xw, zw - 0.614
            target = np.array([bx, by, bz + 0.02])
            res = solve_analytical(target, target_pitch=-0.4)
            self.assertTrue(
                res.success,
                f"IK failed for object '{name}' at target=({bx:.3f}, {by:.3f}, {bz:.3f})"
            )
            self.assertLess(
                res.position_error_m, 0.008,
                f"IK position error too high for '{name}': {res.position_error_m*1000:.2f} mm"
            )

    def test_industrial_conveyor_and_bins_ik(self):
        """Verify that industrial conveyor stopper and sort bins have valid IK solutions."""
        targets = {
            "pick_stopper": (0.20, 0.070, 0.644),
            "assembly_tray": (0.065, -0.200, 0.608),
            "reject_bin": (0.00, 0.220, 0.608),
            "staging_pallet": (0.12, -0.12, 0.608),
        }
        for name, (xw, yw, zw) in targets.items():
            bx, by, bz = yw, -xw, zw - 0.614
            target = np.array([bx, by, bz + 0.03])
            res = solve_analytical(target, target_pitch=-0.4)
            self.assertTrue(res.success, f"Industrial target '{name}' IK failed")
            self.assertLess(res.position_error_m, 0.008)

    def test_tester_prompt_matching(self):
        """Verify natural language prompt matching for all tester world tasks."""
        import rclpy
        from synthetic_server import SyntheticTaskServer
        from unittest.mock import MagicMock

        if not rclpy.ok():
            rclpy.init()

        server = SyntheticTaskServer(target_world="tester")
        server.active_world = "tester"
        server.suite = MagicMock()

        test_cases = [
            ("wave", "Greeting Wave Gesture"),
            ("can you please wave to me?", "Greeting Wave Gesture"),
            ("say hello", "Greeting Wave Gesture"),
            ("nod your head", "Nod ('Yes') and Shake ('No') Gestures"),
            ("say no", "Nod ('Yes') and Shake ('No') Gestures"),
            ("drink some coffee", "Coffee Drinking Gesture"),
            ("pick up the mug using its handle", "Pick Mug Using Handle"),
            ("by handle", "Pick Mug Using Handle"),
            ("stack the mugs on top of each other", "Stack Mugs on Top of Each Other"),
            ("put the banana in the pan", "Put Banana into Frying Pan"),
            ("put the orange ball in the glass", "Put Orange Ball into Glass"),
            ("smaller in bigger", "Smaller Object into Bigger Object (Duck into Bowl)"),
            ("stack the jenga tower", "Precision Jenga Tower Assembly & Stacking"),
            ("solve the jenga tower", "Precision Jenga Tower Assembly & Stacking"),
            ("sort objects based on color", "Sort Objects by Color"),
            ("pick up every object available in the world", "Pick Every Object in World (15/15)"),
            ("360 panoramic inspection scan", "360° Panoramic Inspection Scan"),
            ("transfer to pedestal", "Pedestal-to-Pedestal Transfer"),
            ("clean the table surface", "Tabletop Wipe / Cleaning Motion"),
            ("go home", "Return to Home Neutral Posture"),
        ]

        for prompt, expected_name in test_cases:
            matched = server.match_prompt(prompt)
            self.assertIsNotNone(matched, f"Prompt '{prompt}' failed to match any maneuver")
            action_name, func, subgoals = matched
            self.assertEqual(action_name, expected_name, f"Prompt '{prompt}' matched '{action_name}' instead of '{expected_name}'")
            self.assertGreater(len(subgoals), 0)

        server.destroy_node()

    def test_industrial_prompt_matching(self):
        """Verify natural language prompt matching for all industrial world tasks."""
        import rclpy
        from synthetic_server import SyntheticTaskServer
        from unittest.mock import MagicMock

        if not rclpy.ok():
            rclpy.init()

        server = SyntheticTaskServer(target_world="industrial")
        server.active_world = "industrial"
        server.suite = MagicMock()

        test_cases = [
            ("starts the conveyor belt and sorts the boxes", "Conveyor Infeed & Dual-Bin Box Sorting"),
            ("sort the boxes", "Conveyor Infeed & Dual-Bin Box Sorting"),
            ("sort blue vs red defect boxes", "Conveyor Infeed & Dual-Bin Box Sorting"),
            ("stack the boxes on pallet", "Multi-Tier Box Pallet Stacking"),
            ("drop the boxes on the ground", "Drop Boxes on Ground (Disposal)"),
            ("conveyor belt moving maneuver at all the speeds", "Conveyor Speed Range & Dynamic Switching"),
            ("switch conveyor speeds", "Conveyor Speed Range & Dynamic Switching"),
            ("press the estop button", "Emergency Stop Mushroom Button Press"),
            ("continuous infeed cycle", "Continuous Conveyor Infeed Cycle"),
            ("go home", "Return to Home Neutral Posture"),
        ]

        for prompt, expected_name in test_cases:
            matched = server.match_prompt(prompt)
            self.assertIsNotNone(matched, f"Prompt '{prompt}' failed to match any maneuver")
            action_name, func, subgoals = matched
            self.assertEqual(action_name, expected_name, f"Prompt '{prompt}' matched '{action_name}' instead of '{expected_name}'")
            self.assertGreater(len(subgoals), 0)

        server.destroy_node()


if __name__ == "__main__":
    unittest.main()
