#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Moving Target Handler
Extends TrackingAgent for dynamic targets.
Kalman filter + intercept prediction + continuous replanning.
═══════════════════════════════════════════════════════════════
"""
import math, time
from collections import deque
import numpy as np
from arm_planner.state_bus import StateBus

class KalmanFilter2D:
    """Simple 2D Kalman filter for position + velocity tracking."""

    def __init__(self, dt: float = 0.033):
        self.dt = dt
        # State: [x, y, vx, vy]
        self.x = np.zeros(4)
        # State transition
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        # Measurement matrix (we observe x, y)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ])
        # Covariance
        self.P = np.eye(4) * 1.0
        # Process noise
        q = 0.1
        self.Q = np.array([
            [dt**4/4, 0, dt**3/2, 0],
            [0, dt**4/4, 0, dt**3/2],
            [dt**3/2, 0, dt**2, 0],
            [0, dt**3/2, 0, dt**2],
        ]) * q
        # Measurement noise
        self.R = np.eye(2) * 0.01  # 1cm noise
        self.initialized = False

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z: np.ndarray):
        """Update with measurement z = [x, y]."""
        if not self.initialized:
            self.x[:2] = z
            self.initialized = True
            return
        self.predict()
        y = z - self.H @ self.x  # Innovation
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    @property
    def position(self) -> np.ndarray:
        return self.x[:2]

    @property
    def velocity(self) -> np.ndarray:
        return self.x[2:4]


class MovingTargetHandler:
    """
    Handles dynamic (moving) targets for grasping.

    Velocity estimation from position history.
    Kalman filter for smoothing + prediction.
    Intercept prediction: where can the arm catch the object?

    Usage:
      handler = MovingTargetHandler(bus, arm_speed_mps=0.15)
      handler.update(tracking_id, position_xyz)
      intercept = handler.get_intercept(tracking_id)
    """

    def __init__(self, bus: StateBus, arm_speed_mps: float = 0.15):
        self.bus = bus
        self.arm_speed = arm_speed_mps  # arm end-effector max speed
        self.filters: dict = {}  # tracking_id → KalmanFilter2D
        self.z_positions: dict = {}  # tracking_id → float (Z is usually constant)
        self.position_histories: dict = {}

    def update(self, tracking_id: int, position: np.ndarray):
        """Update with a new observation."""
        if tracking_id not in self.filters:
            self.filters[tracking_id] = KalmanFilter2D()
            self.position_histories[tracking_id] = deque(maxlen=30)

        self.filters[tracking_id].update(position[:2])
        self.z_positions[tracking_id] = position[2] if len(position) > 2 else 0.0
        self.position_histories[tracking_id].append(
            (time.time(), position.copy()))

    def get_velocity(self, tracking_id: int) -> np.ndarray:
        """Get estimated velocity [vx, vy] in m/s."""
        kf = self.filters.get(tracking_id)
        if kf is None:
            return np.zeros(2)
        return kf.velocity

    def get_speed(self, tracking_id: int) -> float:
        """Get speed magnitude in m/s."""
        return float(np.linalg.norm(self.get_velocity(tracking_id)))

    def is_moving(self, tracking_id: int, threshold_mps: float = 0.01) -> bool:
        """Check if object is moving."""
        return self.get_speed(tracking_id) > threshold_mps

    def get_intercept(self, tracking_id: int,
                      arm_position: np.ndarray = None) -> dict:
        """
        Compute intercept position where arm can catch the object.

        Solves: arm_travel_time(arm_pos → intercept) = object_travel_time(obj_pos → intercept)

        For constant velocity objects:
          intercept_time = distance_to_object / (arm_speed - object_speed * cos(approach_angle))

        Returns:
          {
            'intercept_position': [x, y, z],
            'intercept_time_s': float,
            'feasible': bool,
            'object_speed_mps': float,
          }
        """
        kf = self.filters.get(tracking_id)
        if kf is None or not kf.initialized:
            return {'feasible': False, 'reason': 'No tracking data'}

        obj_pos = np.array([kf.position[0], kf.position[1],
                            self.z_positions.get(tracking_id, 0.0)])
        obj_vel_2d = kf.velocity
        obj_speed = float(np.linalg.norm(obj_vel_2d))

        if arm_position is None:
            arm_position = np.zeros(3)

        # If object is stationary, intercept is current position
        if obj_speed < 0.005:
            return {
                'intercept_position': obj_pos.tolist(),
                'intercept_time_s': float(
                    np.linalg.norm(arm_position - obj_pos) / max(self.arm_speed, 0.01)),
                'feasible': True,
                'object_speed_mps': obj_speed,
            }

        # Iterative intercept computation
        # Start with simple linear prediction
        best_time = None
        best_pos = None

        for dt_s in np.linspace(0.1, 5.0, 50):
            # Where will object be at time dt?
            predicted_obj = obj_pos.copy()
            predicted_obj[0] += obj_vel_2d[0] * dt_s
            predicted_obj[1] += obj_vel_2d[1] * dt_s

            # How long for arm to reach that position?
            arm_dist = float(np.linalg.norm(arm_position - predicted_obj))
            arm_time = arm_dist / max(self.arm_speed, 0.01)

            # The arm arrives at the right time?
            if abs(arm_time - dt_s) < 0.2:  # within 200ms
                best_time = dt_s
                best_pos = predicted_obj
                break

        if best_pos is not None:
            self.bus.add_chain_of_thought(
                f"INTERCEPT: Object {tracking_id} moving at "
                f"{obj_speed*100:.1f}cm/s. Intercept at "
                f"({best_pos[0]:.3f}, {best_pos[1]:.3f}) in {best_time:.2f}s")
            return {
                'intercept_position': best_pos.tolist(),
                'intercept_time_s': best_time,
                'feasible': True,
                'object_speed_mps': obj_speed,
            }
        else:
            self.bus.add_chain_of_thought(
                f"INTERCEPT: Object {tracking_id} too fast "
                f"({obj_speed*100:.1f}cm/s) — no feasible intercept")
            return {
                'feasible': False,
                'reason': 'Object too fast for arm',
                'object_speed_mps': obj_speed,
            }

    def predict_position(self, tracking_id: int, dt_s: float) -> np.ndarray:
        """Predict position dt seconds in the future."""
        kf = self.filters.get(tracking_id)
        if kf is None:
            return np.zeros(3)
        pos = kf.position + kf.velocity * dt_s
        z = self.z_positions.get(tracking_id, 0.0)
        return np.array([pos[0], pos[1], z])
