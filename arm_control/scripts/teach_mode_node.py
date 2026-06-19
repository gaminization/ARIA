#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Teach Mode Node
# Full teach-by-demonstration workflow.
#
# User flow:
#   aria teach start      → relax arm, start recording
#   [user moves arm]
#   aria teach waypoint   → record current position
#   aria teach stop       → re-engage, save trajectory
#   aria teach replay     → execute recorded trajectory
#   aria teach save <n>   → save as named skill
#
# Usage:
#   ros2 run arm_control teach_mode_node
# ═══════════════════════════════════════════════════════════════
import os
import sys
import time
import math
import json
import yaml
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class TeachModeNode(Node):
    """Manages teach-by-demonstration recording and replay."""

    JOINT_NAMES = [
        "waist_joint", "shoulder_joint", "elbow_joint",
        "wrist_pitch_joint", "wrist_roll_joint", "gripper_joint",
    ]

    def __init__(self):
        super().__init__("teach_mode_node")
        self.get_logger().info("═══ ARIA Teach Mode Node Starting ═══")

        # ── Parameters ─────────────────────────────────────
        self.declare_parameter("trajectory_dir",
                               os.path.expanduser("~/aria_trajectories"))
        self._traj_dir = self.get_parameter("trajectory_dir").value
        os.makedirs(self._traj_dir, exist_ok=True)

        # ── State ──────────────────────────────────────────
        self._active = False
        self._recording = False
        self._current_positions_deg = [90.0] * 6
        self._waypoints = []
        self._continuous_trajectory = []
        self._start_time = 0.0

        # ── QoS ────────────────────────────────────────────
        qos_reliable = QoSProfile(depth=10)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=5
        )

        # ── Subscribers ────────────────────────────────────
        self._sub_servo_states = self.create_subscription(
            JointState, "/aria/servo_states",
            self._servo_states_cb, qos_sensor
        )

        # ── Publishers ─────────────────────────────────────
        self._pub_teach_mode = self.create_publisher(
            Bool, "/aria/teach_mode", qos_reliable
        )
        self._pub_status = self.create_publisher(
            String, "/aria/teach_status", qos_reliable
        )
        self._pub_joint_cmds = self.create_publisher(
            JointState, "/aria/servo_commands", qos_reliable
        )

        # ── Services ───────────────────────────────────────
        self._srv_start = self.create_service(
            Trigger, "/aria/teach/start", self._start_cb
        )
        self._srv_waypoint = self.create_service(
            Trigger, "/aria/teach/waypoint", self._waypoint_cb
        )
        self._srv_stop = self.create_service(
            Trigger, "/aria/teach/stop", self._stop_cb
        )
        self._srv_replay = self.create_service(
            Trigger, "/aria/teach/replay", self._replay_cb
        )
        self._srv_save = self.create_service(
            Trigger, "/aria/teach/save", self._save_cb
        )

        # ── Status display timer (10Hz) ────────────────────
        self._display_timer = self.create_timer(0.1, self._display_status)

        self.get_logger().info(
            f"Trajectory directory: {self._traj_dir}"
        )

    # ═══════════════════════════════════════════════════════════
    # Servo state callback
    # ═══════════════════════════════════════════════════════════
    def _servo_states_cb(self, msg: JointState):
        """Track current servo positions from ESP32 ADC feedback."""
        for i in range(min(len(msg.position), 6)):
            self._current_positions_deg[i] = msg.position[i]

        # Continuous recording during active teach
        if self._active and self._recording:
            self._continuous_trajectory.append({
                "t": time.time() - self._start_time,
                "pos": list(self._current_positions_deg),
            })

    # ═══════════════════════════════════════════════════════════
    # Service: Start teach mode
    # ═══════════════════════════════════════════════════════════
    def _start_cb(self, request, response):
        if self._active:
            response.success = False
            response.message = "Teach mode already active."
            return response

        self._active = True
        self._recording = True
        self._waypoints = []
        self._continuous_trajectory = []
        self._start_time = time.time()

        # Tell ESP32 to relax servos
        teach_msg = Bool()
        teach_msg.data = True
        self._pub_teach_mode.publish(teach_msg)

        response.success = True
        response.message = (
            "Teach mode STARTED. Arm is RELAXED.\n"
            "Move arm to desired positions and call /aria/teach/waypoint.\n"
            "Call /aria/teach/stop when done."
        )
        self.get_logger().info("TEACH MODE STARTED — servos relaxed")
        return response

    # ═══════════════════════════════════════════════════════════
    # Service: Record waypoint
    # ═══════════════════════════════════════════════════════════
    def _waypoint_cb(self, request, response):
        if not self._active:
            response.success = False
            response.message = "Teach mode not active. Call /aria/teach/start first."
            return response

        wp = {
            "index": len(self._waypoints),
            "timestamp": time.time() - self._start_time,
            "positions_deg": list(self._current_positions_deg),
        }
        self._waypoints.append(wp)

        pos_str = ", ".join(f"{p:.1f}°" for p in self._current_positions_deg)
        response.success = True
        response.message = (
            f"Waypoint {wp['index']} recorded: [{pos_str}]"
        )
        self.get_logger().info(response.message)
        return response

    # ═══════════════════════════════════════════════════════════
    # Service: Stop teach mode
    # ═══════════════════════════════════════════════════════════
    def _stop_cb(self, request, response):
        if not self._active:
            response.success = False
            response.message = "Teach mode not active."
            return response

        self._active = False
        self._recording = False
        duration = time.time() - self._start_time

        # Re-engage servos at current position
        teach_msg = Bool()
        teach_msg.data = False
        self._pub_teach_mode.publish(teach_msg)

        # Auto-save trajectory
        filename = time.strftime("teach_%Y%m%d_%H%M%S")
        self._save_trajectory(filename)

        response.success = True
        response.message = (
            f"Teach mode STOPPED. Servos re-engaged.\n"
            f"Recorded: {len(self._waypoints)} waypoints, "
            f"{len(self._continuous_trajectory)} samples, "
            f"{duration:.1f}s duration.\n"
            f"Saved: {filename}"
        )
        self.get_logger().info(response.message)
        return response

    # ═══════════════════════════════════════════════════════════
    # Service: Replay last trajectory
    # ═══════════════════════════════════════════════════════════
    def _replay_cb(self, request, response):
        if self._active:
            response.success = False
            response.message = "Cannot replay while teach mode is active."
            return response

        if not self._waypoints:
            response.success = False
            response.message = "No waypoints to replay. Record first."
            return response

        self.get_logger().info(
            f"Replaying {len(self._waypoints)} waypoints..."
        )

        # Execute waypoints with interpolation
        thread = threading.Thread(
            target=self._execute_waypoints, daemon=True
        )
        thread.start()

        response.success = True
        response.message = (
            f"Replaying {len(self._waypoints)} waypoints in background."
        )
        return response

    def _execute_waypoints(self):
        """Execute waypoints by publishing servo commands."""
        for i, wp in enumerate(self._waypoints):
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = list(self.JOINT_NAMES)
            cmd.position = [float(p) for p in wp["positions_deg"]]

            self._pub_joint_cmds.publish(cmd)
            self.get_logger().info(
                f"Executing waypoint {i}/{len(self._waypoints)}"
            )

            # Wait between waypoints (quintic-style timing)
            if i < len(self._waypoints) - 1:
                next_t = self._waypoints[i + 1]["timestamp"]
                dt = max(next_t - wp["timestamp"], 0.5)
                time.sleep(dt)

        self.get_logger().info("Replay complete.")

    # ═══════════════════════════════════════════════════════════
    # Service: Save as named skill
    # ═══════════════════════════════════════════════════════════
    def _save_cb(self, request, response):
        if not self._waypoints:
            response.success = False
            response.message = "No trajectory to save."
            return response

        # Use a default name based on timestamp
        name = time.strftime("skill_%Y%m%d_%H%M%S")
        self._save_trajectory(name)

        response.success = True
        response.message = f"Trajectory saved as skill: {name}"
        self.get_logger().info(response.message)
        return response

    # ═══════════════════════════════════════════════════════════
    # File I/O
    # ═══════════════════════════════════════════════════════════
    def _save_trajectory(self, name: str):
        """Save trajectory data (waypoints + continuous) to files."""
        base_path = os.path.join(self._traj_dir, name)
        os.makedirs(base_path, exist_ok=True)

        # Waypoints YAML
        wp_path = os.path.join(base_path, "waypoints.yaml")
        with open(wp_path, "w") as f:
            yaml.dump({
                "name": name,
                "joint_names": self.JOINT_NAMES,
                "num_waypoints": len(self._waypoints),
                "duration_s": self._waypoints[-1]["timestamp"]
                    if self._waypoints else 0.0,
                "waypoints": self._waypoints,
            }, f, default_flow_style=False)

        # Continuous trajectory JSON (for ML training)
        cont_path = os.path.join(base_path, "continuous.json")
        with open(cont_path, "w") as f:
            json.dump({
                "joint_names": self.JOINT_NAMES,
                "num_samples": len(self._continuous_trajectory),
                "samples": self._continuous_trajectory,
            }, f, indent=2)

        self.get_logger().info(
            f"Saved trajectory '{name}': "
            f"{len(self._waypoints)} waypoints, "
            f"{len(self._continuous_trajectory)} samples"
        )

    def _load_trajectory(self, name: str) -> bool:
        """Load trajectory from file."""
        wp_path = os.path.join(self._traj_dir, name, "waypoints.yaml")
        if not os.path.exists(wp_path):
            self.get_logger().error(f"Trajectory not found: {wp_path}")
            return False

        with open(wp_path, "r") as f:
            data = yaml.safe_load(f)

        self._waypoints = data.get("waypoints", [])
        self.get_logger().info(
            f"Loaded trajectory '{name}': {len(self._waypoints)} waypoints"
        )
        return True

    # ═══════════════════════════════════════════════════════════
    # Real-time terminal display
    # ═══════════════════════════════════════════════════════════
    def _display_status(self):
        """Publish teach mode status for dashboard/terminal."""
        if not self._active:
            return

        msg = String()
        lines = [
            "══════════════════════════════════════",
            " ARIA TEACH MODE — Arm is RELAXED",
            "══════════════════════════════════════",
        ]

        joint_short = ["Waist", "Shoulder", "Elbow",
                       "Wr.Pitch", "Wr.Roll", "Gripper"]
        for i, name in enumerate(joint_short):
            lines.append(
                f" J{i+1} {name:12s}: [{self._current_positions_deg[i]:6.1f}°] (ADC)"
            )

        elapsed = time.time() - self._start_time if self._start_time else 0
        lines.extend([
            "──────────────────────────────────────",
            f" Waypoints recorded: {len(self._waypoints)}",
            f" Recording: [●] {'ACTIVE' if self._recording else 'PAUSED'}",
            f" Elapsed: {elapsed:.1f}s",
            " Sim: FOLLOWING real arm in real-time",
            "══════════════════════════════════════",
        ])

        msg.data = "\n".join(lines)
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeachModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
