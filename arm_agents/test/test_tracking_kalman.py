"""Unit tests for the tracking Kalman filter and moving-target intercept logic.

These are pure-Python (no ROS graph) checks that the 2D Kalman filter and
MovingTargetHandler math behave correctly, since TrackingAgent depends on them
to predict trajectories and compute rolling-object intercepts.
"""
import unittest

import numpy as np

from arm_agents.moving_target_handler import KalmanFilter2D, MovingTargetHandler


class _FakeBus:
    """Minimal StateBus stand-in capturing chain-of-thought notes."""

    def __init__(self):
        self.notes = []

    def add_chain_of_thought(self, text):
        self.notes.append(text)


class KalmanFilterTests(unittest.TestCase):
    def test_first_measurement_initializes_position(self):
        kf = KalmanFilter2D(dt=0.1)
        kf.update(np.array([1.0, 2.0]))
        self.assertTrue(kf.initialized)
        np.testing.assert_allclose(kf.position, [1.0, 2.0])
        np.testing.assert_allclose(kf.velocity, [0.0, 0.0])

    def test_filter_tracks_constant_velocity(self):
        kf = KalmanFilter2D(dt=0.1)
        # Object moving at 1.0 m/s in x, 0.5 m/s in y.
        for i in range(30):
            kf.update(np.array([1.0 * i * 0.1, 0.5 * i * 0.1]))
        self.assertAlmostEqual(kf.velocity[0], 1.0, delta=0.1)
        self.assertAlmostEqual(kf.velocity[1], 0.5, delta=0.1)

    def test_filter_smooths_measurement_noise(self):
        rng = np.random.default_rng(0)
        kf = KalmanFilter2D(dt=0.1)
        true_pos = np.array([0.0, 0.0])
        for _ in range(50):
            true_pos += np.array([0.0, 0.0])
            noisy = true_pos + rng.normal(0, 0.01, size=2)
            kf.update(noisy)
        # Filtered estimate should be within the measurement-noise band of the
        # (static) truth, i.e. smoothing actually suppresses the noise.
        self.assertLess(np.linalg.norm(kf.position - true_pos), 0.02)


class MovingTargetHandlerTests(unittest.TestCase):
    def test_stationary_object_yields_current_position_intercept(self):
        bus = _FakeBus()
        handler = MovingTargetHandler(bus, arm_speed_mps=0.15)
        for _ in range(5):
            handler.update(7, np.array([0.2, 0.1, 0.05]))
        intercept = handler.get_intercept(
            7, arm_position=np.array([0.0, 0.0, 0.3]))
        self.assertTrue(intercept["feasible"])
        np.testing.assert_allclose(
            intercept["intercept_position"], [0.2, 0.1, 0.05], atol=1e-6)

    def test_intercept_fails_without_tracking_data(self):
        handler = MovingTargetHandler(_FakeBus())
        intercept = handler.get_intercept(999)
        self.assertFalse(intercept["feasible"])
        self.assertIn("reason", intercept)

    def test_moving_object_gets_feasible_intercept_ahead(self):
        bus = _FakeBus()
        handler = MovingTargetHandler(bus, arm_speed_mps=0.5)
        # Object rolls along +x at ~0.2 m/s (0.033 s nominal sample period).
        for i in range(20):
            handler.update(3, np.array([0.3 + 0.2 * 0.033 * i, 0.0, 0.02]))
        self.assertGreater(handler.get_speed(3), 0.01)
        self.assertTrue(handler.is_moving(3))
        intercept = handler.get_intercept(
            3, arm_position=np.array([0.0, 0.0, 0.3]))
        # Arm should catch the object ahead of its current x position.
        self.assertTrue(intercept["feasible"])
        self.assertGreater(intercept["intercept_position"][0], 0.3)
        self.assertTrue(bus.notes)

    def test_very_fast_object_is_reported_infeasible(self):
        bus = _FakeBus()
        handler = MovingTargetHandler(bus, arm_speed_mps=0.02)
        for i in range(20):
            handler.update(4, np.array([5.0 * i, 0.0, 0.02]))
        intercept = handler.get_intercept(
            4, arm_position=np.array([0.0, 0.0, 0.3]))
        self.assertFalse(intercept["feasible"])

    def test_predict_position_extrapolates_in_time(self):
        handler = MovingTargetHandler(_FakeBus())
        for i in range(10):
            handler.update(5, np.array([0.1 * i, 0.0, 0.15]))
        predicted = handler.predict_position(5, dt_s=1.0)
        self.assertAlmostEqual(predicted[2], 0.15)
        self.assertGreater(predicted[0], 0.9)


if __name__ == "__main__":
    unittest.main()
