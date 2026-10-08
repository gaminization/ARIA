#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Tracking Agent — LifecycleNode
Kalman filter tracking, trajectory prediction, LOST detection.
═══════════════════════════════════════════════════════════════
"""
import math, time
from collections import deque
from typing import Dict
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from arm_planner.msg import VisionState, ObjectDetection
from arm_planner.state_bus import StateBus

LOST_THRESHOLD_FRAMES = 15  # Mark as LOST after N frames without detection
PREDICTION_HORIZON_S = 0.5   # Predict 0.5s into the future

class TrackedObject:
    """Per-object tracking state with Kalman-like prediction."""
    def __init__(self, tracking_id: int, class_name: str):
        self.tracking_id = tracking_id
        self.class_name = class_name
        self.position_history: deque = deque(maxlen=20)
        self.timestamps: deque = deque(maxlen=20)
        self.velocity = np.zeros(3)
        self.predicted_position = np.zeros(3)
        self.frames_since_seen = 0
        self.lifecycle_state = 'Tracked'

    def update(self, position: np.ndarray, timestamp: float):
        self.position_history.append(position.copy())
        self.timestamps.append(timestamp)
        self.frames_since_seen = 0
        self.lifecycle_state = 'Tracked'
        # Velocity from 5-frame difference
        if len(self.position_history) >= 6:
            dt = self.timestamps[-1] - self.timestamps[-6]
            if dt > 0.01:
                self.velocity = (self.position_history[-1] - self.position_history[-6]) / dt
        # Predict future position
        self.predicted_position = position + self.velocity * PREDICTION_HORIZON_S

    def mark_unseen(self):
        self.frames_since_seen += 1
        if self.frames_since_seen > LOST_THRESHOLD_FRAMES:
            self.lifecycle_state = 'Lost'

class TrackingAgent(LifecycleNode):
    """Maintains consistent object IDs, predicts trajectories, detects LOST."""

    def __init__(self):
        super().__init__('tracking_agent')
        self.bus = StateBus(self)
        self.tracked: Dict[int, TrackedObject] = {}

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("TrackingAgent: CONFIGURING")
        self.bus.on_change('vision', self._on_vision)
        self.viz_pub = self.create_publisher(
            MarkerArray, '/tracking/predicted_viz', 10)
        self.create_timer(0.033, self._tick)  # 30Hz
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("TrackingAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_vision(self, msg: VisionState):
        """Update tracking from VisionState detections with spatial gating."""
        now = time.time()
        seen_tids = set()

        for det in msg.detected_objects:
            tid = det.tracking_id
            pos = np.array([
                det.pose_3d.pose.position.x,
                det.pose_3d.pose.position.y,
                det.pose_3d.pose.position.z,
            ]) if det.pose_3d.pose.position.x != 0 else np.zeros(3)

            matched_tid = None
            if tid >= 0 and tid in self.tracked:
                matched_tid = tid
            else:
                # Spatial gating: match to existing tracked object if within 8cm
                if np.linalg.norm(pos) > 0.05:
                    for exist_tid, exist_obj in self.tracked.items():
                        if len(exist_obj.position_history) > 0:
                            dist = float(np.linalg.norm(exist_obj.position_history[-1] - pos))
                            if dist < 0.08:
                                matched_tid = exist_tid
                                break

            if matched_tid is None:
                new_id = tid if tid >= 0 else len(self.tracked) + 1
                self.tracked[new_id] = TrackedObject(new_id, det.class_name)
                matched_tid = new_id
                self.get_logger().info(f"TRACKING: Registered object ID={new_id} ({det.class_name}) at {pos}")

            seen_tids.add(matched_tid)
            self.tracked[matched_tid].update(pos, now)

        # Mark unseen objects
        for tid, obj in self.tracked.items():
            if tid not in seen_tids:
                obj.mark_unseen()

    def _tick(self):
        """Publish prediction visualization."""
        markers = MarkerArray()
        for i, (tid, obj) in enumerate(self.tracked.items()):
            if obj.lifecycle_state == 'Lost':
                continue
            speed = float(np.linalg.norm(obj.velocity))
            if speed < 0.005:
                continue  # Stationary — no prediction arrow
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'tracking_predictions'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            start = Point()
            if len(obj.position_history) > 0:
                p = obj.position_history[-1]
                start.x, start.y, start.z = float(p[0]), float(p[1]), float(p[2])
            end = Point()
            end.x = float(obj.predicted_position[0])
            end.y = float(obj.predicted_position[1])
            end.z = float(obj.predicted_position[2])
            m.points = [start, end]
            m.scale.x = 0.005
            m.scale.y = 0.01
            m.scale.z = 0.01
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.8
            m.lifetime.sec = 1
            markers.markers.append(m)
        self.viz_pub.publish(markers)

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
