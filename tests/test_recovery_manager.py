#!/usr/bin/env python3
"""
ARIA Recovery Manager — Unit Tests

Verifies that the LLM-unavailable recovery fallback picks the cheapest
corrective action that can actually resolve the failure (a ``locate``
when a target is named, a workspace scan otherwise), that it retries the
failed action, and that it never invents success.
"""
import unittest

from arm_planner.msg import Action, TaskState
from arm_planner.recovery_manager import build_recovery_actions


def make_task(target_object: str = '') -> TaskState:
    task = TaskState()
    task.task_id = 'task_1'
    task.current_command = 'pick up the banana'
    task.current_goal = 'pick banana'
    action = Action()
    action.action_type = 'pick'
    action.target_object = target_object
    action.confidence = 0.4
    action.status = 'FAILED'
    task.action_queue = [action]
    return task


class RecoveryFallbackTests(unittest.TestCase):
    def test_named_target_recovers_with_locate_not_scan(self):
        """A failed pick of a known object should refresh its pose via locate."""
        task = make_task(target_object='banana')
        actions = build_recovery_actions(task, 0, None)
        self.assertIsNotNone(actions)
        self.assertEqual(actions[0].action_type, 'locate')
        self.assertEqual(actions[0].target_object, 'banana')
        # The failed action is retried after the corrective step.
        self.assertEqual(actions[-1].action_type, 'pick')
        self.assertEqual(actions[-1].status, 'PENDING')

    def test_untargeted_failure_recovers_with_scan(self):
        task = make_task(target_object='')
        actions = build_recovery_actions(task, 0, None)
        self.assertIsNotNone(actions)
        self.assertEqual(actions[0].action_type, 'scan_workspace')

    def test_give_up_halts_the_task(self):
        task = make_task(target_object='banana')
        self.assertIsNone(build_recovery_actions(task, 0, {'give_up': True}))

    def test_llm_steps_are_used_when_available(self):
        task = make_task(target_object='banana')
        plan = {
            'diagnosis': 'stale world model',
            'steps': [{'skill': 'scan_workspace'}],
            'retry_failed_action': True,
        }
        actions = build_recovery_actions(task, 0, plan)
        self.assertIsNotNone(actions)
        self.assertEqual(actions[0].action_type, 'scan_workspace')
        self.assertEqual(actions[-1].action_type, 'pick')

    def test_retry_can_be_suppressed(self):
        task = make_task(target_object='banana')
        plan = {
            'steps': [{'skill': 'scan_workspace'}],
            'retry_failed_action': False,
        }
        actions = build_recovery_actions(task, 0, plan)
        self.assertEqual([a.action_type for a in actions], ['scan_workspace'])


if __name__ == '__main__':
    unittest.main()
