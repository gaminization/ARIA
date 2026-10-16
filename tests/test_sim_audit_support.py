import unittest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from aria_sim_audit_support import evaluate_pick_hold, validate_task_trace


class PickEvidenceTests(unittest.TestCase):
    def test_requires_fifty_millimetres_and_two_continuous_seconds(self):
        samples = [(i * 0.1, 0.60) for i in range(26)]
        result = evaluate_pick_hold(0.54, samples)
        self.assertTrue(result.passed)
        self.assertAlmostEqual(result.max_lift_m, 0.06)
        self.assertAlmostEqual(result.continuous_hold_s, 2.5)

    def test_does_not_count_a_drop_or_missing_sample_as_continuous_hold(self):
        dropped = [(0.0, 0.60), (1.0, 0.60), (1.1, 0.54), (1.2, 0.60), (2.1, 0.60)]
        missing = [(0.0, 0.60), (0.1, 0.60), (0.5, 0.60), (2.1, 0.60)]
        self.assertFalse(evaluate_pick_hold(0.54, dropped).passed)
        self.assertFalse(evaluate_pick_hold(0.54, missing).passed)


class TaskTraceTests(unittest.TestCase):
    def test_accepts_ordered_task_and_action_completion(self):
        snapshots = [
            ("PLANNING", ("PENDING", "PENDING")),
            ("EXECUTING", ("EXECUTING", "PENDING")),
            ("EXECUTING", ("COMPLETE", "EXECUTING")),
            ("COMPLETE", ("COMPLETE", "COMPLETE")),
        ]
        self.assertTrue(validate_task_trace(snapshots)[0])

    def test_rejects_out_of_order_and_incomplete_tasks(self):
        out_of_order = [
            ("PLANNING", ("PENDING", "PENDING")),
            ("EXECUTING", ("PENDING", "EXECUTING")),
            ("COMPLETE", ("COMPLETE", "COMPLETE")),
        ]
        incomplete = [
            ("PLANNING", ("PENDING",)),
            ("EXECUTING", ("EXECUTING",)),
            ("COMPLETE", ("EXECUTING",)),
        ]
        self.assertFalse(validate_task_trace(out_of_order)[0])
        self.assertFalse(validate_task_trace(incomplete)[0])


if __name__ == "__main__":
    unittest.main()
