"""Unit tests for in-hand manipulation wiring and helpers.

Covers the geometry helper used to verify a 180° flip and that the extended
in-hand skills can be rebound from placeholders to a real executor.
"""
import math
import unittest

from arm_agents.in_hand_manipulation import InHandManipulation
from arm_planner.skill_manager import SkillManager


class AngleDiffTests(unittest.TestCase):
    def test_small_difference(self):
        self.assertAlmostEqual(InHandManipulation._angle_diff_deg(30.0, 10.0), 20.0)

    def test_wraps_across_180(self):
        # 170 -> -170 is a 20° change, not 340°.
        self.assertAlmostEqual(
            InHandManipulation._angle_diff_deg(170.0, -170.0), -20.0)

    def test_full_flip_detected(self):
        # Sweeping -90 -> +90 is a 180° flip.
        self.assertAlmostEqual(
            abs(InHandManipulation._angle_diff_deg(90.0, -90.0)), 180.0)


class _FakeExecutor:
    def __init__(self, result=True):
        self.result = result
        self.calls = []

    def rotate_in_hand(self, angle_deg, axis='y'):
        self.calls.append(('rotate', angle_deg, axis))
        return self.result

    def reposition_grip(self, offset_mm, direction):
        self.calls.append(('reposition', offset_mm, direction))
        return self.result

    def flip_object(self):
        self.calls.append(('flip',))
        return self.result

    def slide_to_tip(self, target_extension_mm):
        self.calls.append(('slide', target_extension_mm))
        return self.result


class SkillManagerBindingTests(unittest.TestCase):
    def test_placeholders_fail_before_binding(self):
        sm = SkillManager()
        result = sm.execute_skill('flip')
        self.assertFalse(result.success)

    def test_binding_replaces_placeholders_with_real_callbacks(self):
        sm = SkillManager()
        executor = _FakeExecutor(result=True)
        bound = sm.register_in_hand_skills(executor)

        self.assertCountEqual(
            bound, ['rotate_in_hand', 'reposition_grip', 'flip', 'slide_to_tip'])

        flip = sm.execute_skill('flip')
        self.assertTrue(flip.success)
        slide = sm.execute_skill('slide_to_tip', target_extension_mm=12.0)
        self.assertTrue(slide.success)

        self.assertIn(('flip',), executor.calls)
        self.assertIn(('slide', 12.0), executor.calls)

    def test_binding_reports_executor_failure_honestly(self):
        sm = SkillManager()
        executor = _FakeExecutor(result=False)
        sm.register_in_hand_skills(executor)

        self.assertFalse(sm.execute_skill('reposition_grip',
                                          offset_mm=4.0,
                                          direction='forward').success)


if __name__ == '__main__':
    unittest.main()
