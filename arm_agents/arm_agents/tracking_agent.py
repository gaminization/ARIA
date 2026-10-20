#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Tracking Agent — LifecycleNode
Kalman filter tracking, trajectory prediction, LOST detection.

Wires MovingTargetHandler (2D KalmanFilter2D) into the rolling-object
pipeline: per-object velocity estimation, constant-velocity prediction
and intercept-point computation for moving/rolling workpieces.

Subscribes:
  /aria/state/vision   (via StateBus) — detections with tracking ids

Publishes:
  /tracking/predicted_viz  (MarkerArray — velocity arrows)
  /tracking/objects        (String JSON — tracked objects + intercepts)
  /tracking/status         (String — node status)
═══════════════════════════════════════════════════════════════
"""
import json
import math
import time
from collections import deque
from typing import Dict, List, Optional

import numpy as np

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

from arm_planner.msg import VisionState, ObjectDetection
from arm_planner.state_bus import StateBus

try:
    from arm_agents.moving_target_handler import (
        MovingTargetHandler, KalmanFilter2D,
    )
except ImportError:
    from moving_target_handler import (
        MovingTargetHandler, KalmanFilter2D,
    )

LOST_THRESHOLD_FRAMES = 15   # Mark as LOST after N frames without detection
PREDICTION_HORIZON_S = 0.5   # Predict 0.5s into the future
MOVING_SPEED_THRESHOLD_MPS = 0.01  # > 1 cm/s counts as moving/rolling


class TrackedObject:
    """Per-object tracking state using KalmanFilter2D from MovingTargetHandler.

    Uses a 2D image-plane Kalman filter for velocity estimation and
    prediction, with separate 3D depth information from the depth camera.
    """

    def __init__(self, tracking_id: int, class_name: str):
        self.tracking_id = tracking_id
        self.class_name = class_name
        self.kf = KalmanFilter2D(dt=0.033)  # 30Hz matching tracking timer
        self.initialized = False
        self.last_measured_position = np.zeros(2)  # (x, y) in pixel coords
        self.frames_since_seen = 0
        self.lifecycle_state = 'Tracked'
        # 3D position from depth camera (meters, separate from Kalman filter)
        self.position_3d = np.zeros(3)
        self.last_depth_timestamp = 0.0
        # Rolling object velocity in 3D space
        self.velocity_3d = np.zeros(3)

    def update(self, position_2d: np.ndarray, position_3d: np.ndarray = None,
               timestamp: float = None, velocity_3d: np.ndarray = None):
        """Update tracking with 2D image position and optional 3D depth position.

        Args:
            position_2d: (x, y) in pixel coordinates from detection
            position_3d: (x, y, z) in meters from depth camera (optional)
            timestamp: ROS time stamp (optional)
            velocity_3d: (vx, vy, vz) in m/s from detection (optional)
        """
        # Update Kalman filter with 2D measurement
        measurement = np.array(position_2d, dtype=np.float64)
        self.kf.update(measurement)
        self.initialized = True
        self.last_measured_position = np.asarray(position_2d, dtype=float).copy()

        # Store 3D position if provided
        if position_3d is not None:
            self.position_3d = np.array(position_3d, dtype=np.float64)
        if timestamp is not None:
            self.last_depth_timestamp = timestamp
        if velocity_3d is not None:
            self.velocity_3d = np.array(velocity_3d, dtype=np.float64)

        # Update lifecycle state
        self.lifecycle_state = 'Tracked'
        self.frames_since_seen = 0

    def mark_unseen(self):
        """Mark object as unseen for one frame."""
        self.frames_since_seen += 1
        if self.frames_since_seen > LOST_THRESHOLD_FRAMES:
            self.lifecycle_state = 'Lost'

    @property
    def estimated_position_2d(self) -> np.ndarray:
        """Get the Kalman-filtered 2D position estimate."""
        if not self.initialized:
            return self.last_measured_position
        return self.kf.position.copy()

    @property
    def estimated_velocity_2d(self) -> np.ndarray:
        """Get the Kalman-filtered 2D velocity estimate."""
        if not self.initialized:
            return np.zeros(2)
        return self.kf.velocity.copy()

    def predicted_position_2d(self, horizon_s: float = PREDICTION_HORIZON_S
                              ) -> np.ndarray:
        """Constant-velocity prediction of the 2D position."""
        return self.estimated_position_2d + self.estimated_velocity_2d * horizon_s


class MovingTargetTracker:
    """Wraps MovingTargetHandler to manage per-object tracking.

    Handles ID assignment, spatial gating, and LOST detection.
    """

    def __init__(self):
        self.tracked: dict = {}  # tracking_id -> TrackedObject
        self.lost: dict = {}     # tracking_id -> TrackedObject
        self.next_id = 0
        self.seen_tids: set = set()
        self.frame_count = 0

    def update(self, detections, timestamp):
        """Update tracker with new detections from VisionState.

        Args:
            detections: list of ObjectDetection from VisionState
            timestamp: current ROS time
        """
        self.seen_tids = set()
        self.frame_count += 1

        for det in detections:
            tid = det.tracking_id
            # Extract 2D position from the detection's bbox fields
            # (VisionState/ObjectDetection exposes bbox_x/y/w/h, not a
            # Detection2D bbox message).
            cx = float(det.bbox_x) + float(det.bbox_w) * 0.5
            cy = float(det.bbox_y) + float(det.bbox_h) * 0.5
            position_2d = np.array([cx, cy])

            # Extract 3D position if available
            pos_3d_np = np.zeros(3)
            if det.pose_3d and det.pose_3d.pose.position.x != 0:
                pos_3d_np = np.array([
                    det.pose_3d.pose.position.x,
                    det.pose_3d.pose.position.y,
                    det.pose_3d.pose.position.z,
                ])

            # Spatial gating: match to existing tracked object if within threshold
            matched_tid = None
            if tid >= 0 and tid in self.tracked:
                matched_tid = tid
            else:
                # Try to match to existing tracked object if within 8cm in 2D
                if np.linalg.norm(position_2d) > 0:  # non-zero position
                    for exist_tid, exist_obj in self.tracked.items():
                        kf_pos = exist_obj.estimated_position_2d
                        dist = float(np.linalg.norm(kf_pos - position_2d))
                        if dist < 80:  # 8cm in pixel coords at typical resolution
                            matched_tid = exist_tid
                            break

            if matched_tid is None:
                # New object
                new_id = tid if tid >= 0 else self.next_id
                self.tracked[new_id] = TrackedObject(new_id, det.class_name)
                matched_tid = new_id
                self.next_id = max(self.next_id, new_id + 1)

            # Update the matched/tracked object
            self.tracked[matched_tid].update(
                position_2d=position_2d,
                position_3d=pos_3d_np if np.linalg.norm(pos_3d_np) > 0 else None,
                timestamp=timestamp,
                velocity_3d=np.array([
                    getattr(det, 'velocity_x', 0.0),
                    getattr(det, 'velocity_y', 0.0),
                    getattr(det, 'velocity_z', 0.0),
                ]),
            )
            self.seen_tids.add(matched_tid)

        # Mark previously tracked objects as unseen
        for tid in list(self.tracked.keys()):
            if tid not in self.seen_tids:
                self.tracked[tid].mark_unseen()
                # Move to lost after threshold
                if self.tracked[tid].frames_since_seen > LOST_THRESHOLD_FRAMES:
                    self.lost[tid] = self.tracked.pop(tid)

    def tick(self):
        """Call at 30Hz — marks objects as LOST if unseen too long."""
        for tid in list(self.tracked.keys()):
            if self.tracked[tid].frames_since_seen > LOST_THRESHOLD_FRAMES:
                self.lost[tid] = self.tracked.pop(tid)

    def get_publish_data(self):
        """Get data for visualization publishers."""
        viz_data = []
        for tid, obj in self.tracked.items():
            if obj.lifecycle_state == 'Lost':
                continue
            kf_pos = obj.estimated_position_2d
            kf_vel = obj.estimated_velocity_2d
            viz_data.append({
                'id': tid,
                'class': obj.class_name,
                'kf_position_2d': kf_pos,
                'kf_velocity_2d': kf_vel,
                'position_3d': obj.position_3d,
                'velocity_3d': obj.velocity_3d,
                'frames_seen': obj.frames_since_seen,
            })
        return viz_data


class TrackingAgent(LifecycleNode):
    """Maintains consistent object IDs, predicts trajectories, detects LOST.

    Also drives the rolling-object pipeline: for any tracked object whose
    estimated speed exceeds MOVING_SPEED_THRESHOLD_MPS, MovingTargetHandler
    computes an intercept point that the planner/control layer can target.
    """

    def __init__(self):
        super().__init__('tracking_agent')
        self.bus = StateBus(self)
        self.tracker = MovingTargetTracker()
        # 3D intercept predictor (constant-velocity model on world positions).
        # Arm speed is a conservative nominal cap used only for feasibility.
        self.moving_handler = MovingTargetHandler(
            self.bus, arm_speed_mps=0.15)
        self._latest_detections = []

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("TrackingAgent: CONFIGURING")
        self.bus.on_change('vision', self._on_vision)
        self.viz_pub = self.create_publisher(
            MarkerArray, '/tracking/predicted_viz', 10)
        self.objects_pub = self.create_publisher(
            String, '/tracking/objects', 10)
        self.status_pub = self.create_publisher(
            String, '/tracking/status', 10)
        self.create_timer(1.0 / 30.0, self._tick)  # 30Hz
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("TrackingAgent: ACTIVATED")
        self.status_pub.publish(String(data="active"))
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.status_pub.publish(String(data="inactive"))
        return TransitionCallbackReturn.SUCCESS

    def _on_vision(self, msg: VisionState):
        """Update tracking from VisionState detections with spatial gating."""
        now = time.time()
        self._latest_detections = list(msg.detected_objects)

        # 1. 2D image-plane association / prediction (pixel gating).
        self.tracker.update(self._latest_detections, now)

        # 2. 3D velocity/intercept model for objects with a valid world pose.
        for det in self._latest_detections:
            tracking_id = det.tracking_id if det.tracking_id >= 0 else None
            if tracking_id is None:
                continue
            p = det.pose_3d.pose.position if det.pose_3d else None
            if p is None or (p.x == 0.0 and p.y == 0.0):
                continue
            self.moving_handler.update(
                tracking_id, np.array([p.x, p.y, p.z], dtype=float))

    def _tick(self):
        """Publish prediction visualization + rolling-object intercepts."""
        self.tracker.tick()
        markers = MarkerArray()

        for i, (tid, obj) in enumerate(self.tracker.tracked.items()):
            if obj.lifecycle_state == 'Lost':
                continue
            velocity_2d = obj.estimated_velocity_2d
            speed_px = float(np.linalg.norm(velocity_2d))
            if speed_px < 0.5:
                continue  # Stationary — no prediction arrow

            # Arrow from measured position to constant-velocity prediction.
            start_2d = obj.estimated_position_2d
            end_2d = obj.predicted_position_2d()
            m = Marker()
            m.header.frame_id = 'top_camera_link'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'tracking_predictions'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.points = [
                Point(x=float(start_2d[0]), y=float(start_2d[1]), z=0.0),
                Point(x=float(end_2d[0]), y=float(end_2d[1]), z=0.0),
            ]
            m.scale.x = 2.0
            m.scale.y = 4.0
            m.scale.z = 4.0
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.8
            m.lifetime.sec = 1
            markers.markers.append(m)

        # 3D world-frame intercept arrows for rolling/moving objects.
        tracked_payload = []
        for i, (tid, obj) in enumerate(self.tracker.tracked.items()):
            if obj.lifecycle_state == 'Lost':
                continue
            entry = {
                'id': int(tid),
                'class': obj.class_name,
                'position_3d': [float(v) for v in obj.position_3d],
                'velocity_3d': [float(v) for v in obj.velocity_3d],
                'velocity_2d': [float(v) for v in obj.estimated_velocity_2d],
                'lifecycle_state': obj.lifecycle_state,
            }
            if np.linalg.norm(obj.position_3d) > 0:
                speed = float(self.moving_handler.get_speed(tid))
                if speed > MOVING_SPEED_THRESHOLD_MPS:
                    intercept = self.moving_handler.get_intercept(
                        tid, arm_position=np.array([0.0, 0.0, 0.61]))
                    entry['speed_mps'] = speed
                    entry['intercept'] = intercept
                    if intercept.get('feasible'):
                        pos = intercept['intercept_position']
                        m = Marker()
                        m.header.frame_id = 'world'
                        m.header.stamp = self.get_clock().now().to_msg()
                        m.ns = 'tracking_intercepts'
                        m.id = 1000 + int(tid)
                        m.type = Marker.ARROW
                        m.action = Marker.ADD
                        m.points = [
                            Point(x=float(obj.position_3d[0]),
                                  y=float(obj.position_3d[1]),
                                  z=float(obj.position_3d[2])),
                            Point(x=float(pos[0]), y=float(pos[1]),
                                  z=float(pos[2])),
                        ]
                        m.scale.x = 0.004
                        m.scale.y = 0.012
                        m.scale.z = 0.012
                        m.color.r, m.color.g, m.color.b, m.color.a = (
                            0.1, 0.9, 0.1, 0.9)
                        m.lifetime.sec = 1
                        markers.markers.append(m)
            tracked_payload.append(entry)

        if markers.markers:
            self.viz_pub.publish(markers)

        msg = String()
        msg.data = json.dumps({
            'n_tracked': len(self.tracker.tracked),
            'n_lost': len(self.tracker.lost),
            'objects': tracked_payload,
        })
        self.objects_pub.publish(msg)

        status = String()
        status.data = (
            f"tracked={len(self.tracker.tracked)} "
            f"lost={len(self.tracker.lost)}")
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingAgent()
    node.trigger_configure()
    node.trigger_activate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
