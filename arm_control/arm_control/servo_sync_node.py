#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Servo Sync Node
# THE BRIDGE between simulation and real hardware.
#
# Sync modes:
#   SIM_TO_REAL  — simulation joint_states drive real servos
#   REAL_TO_SIM  — ESP32 ADC readings drive simulation
#   MIRROR       — both directions synchronized
#   DISABLED     — no sync, manual control only
#
# Usage:
#   ros2 run arm_control servo_sync_node
# ═══════════════════════════════════════════════════════════════
import math
import time
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class ServoSyncNode(Node):
    """Bidirectional bridge between ROS2 simulation and ESP32 hardware."""

    SYNC_MODES = {
        "SIM_TO_REAL":  "Normal: sim drives hardware",
        "REAL_TO_SIM":  "Teach: hardware drives sim",
        "MIRROR":       "Both: commands and feedback synchronized",
        "DISABLED":     "No sync, manual control only",
    }

    JOINT_NAMES = [
        "waist_joint", "shoulder_joint", "elbow_joint",
        "wrist_pitch_joint", "wrist_roll_joint", "gripper_joint",
    ]

    def __init__(self):
        super().__init__("servo_sync_node")
        self.get_logger().info("═══ ARIA Servo Sync Node Starting ═══")

        # ── Parameters ─────────────────────────────────────
        self.declare_parameter("sync_mode", "SIM_TO_REAL")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("record_teach", False)

        self._sync_mode = self.get_parameter("sync_mode").value
        cal_file = self.get_parameter("calibration_file").value

        # ── Calibration offsets (degrees) ──────────────────
        # Offset = sim_angle - real_servo_angle per joint
        self._offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if cal_file and os.path.exists(cal_file):
            self._load_calibration(cal_file)

        # ── State ──────────────────────────────────────────
        self._sim_positions_rad = [0.0] * 6
        self._real_positions_deg = [0.0] * 6
        self._esp32_connected = False
        self._last_heartbeat = 0.0
        self._teach_trajectory = []
        self._record_teach = self.get_parameter("record_teach").value

        # ── QoS ────────────────────────────────────────────
        qos_reliable = QoSProfile(depth=10)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=5
        )

        # ── Subscribers ────────────────────────────────────

        # From simulation (ros2_control joint states)
        self._sub_sim_joints = self.create_subscription(
            JointState, "/joint_states",
            self._sim_joint_cb, qos_reliable
        )

        # From ESP32 (real servo feedback)
        self._sub_real_joints = self.create_subscription(
            JointState, "/aria/servo_states",
            self._real_joint_cb, qos_sensor
        )

        # ESP32 heartbeat
        from std_msgs.msg import Header
        self._sub_heartbeat = self.create_subscription(
            Header, "/aria/heartbeat",
            self._heartbeat_cb, qos_sensor
        )

        # ── Publishers ─────────────────────────────────────

        # To ESP32 (servo commands in degrees)
        self._pub_servo_cmds = self.create_publisher(
            JointState, "/aria/servo_commands", qos_reliable
        )

        # To simulation (override joint states from real arm)
        self._pub_sim_override = self.create_publisher(
            JointState, "/joint_states_override", qos_reliable
        )

        # Teach mode toggle to ESP32
        self._pub_teach_mode = self.create_publisher(
            Bool, "/aria/teach_mode", qos_reliable
        )

        # Sync status for dashboard
        self._pub_sync_status = self.create_publisher(
            String, "/aria/sync_status", qos_reliable
        )

        # ── Services ───────────────────────────────────────

        self._srv_set_mode = self.create_service(
            Trigger, "/aria/sync_mode",
            self._set_sync_mode_cb
        )

        self._srv_calibrate = self.create_service(
            Trigger, "/aria/calibrate_offsets",
            self._calibrate_offsets_cb
        )

        self._srv_verify = self.create_service(
            Trigger, "/aria/verify_sync",
            self._verify_sync_cb
        )

        # ── Timer: publish sync status at 2Hz ──────────────
        self._status_timer = self.create_timer(0.5, self._publish_status)

        # ── Timer: check ESP32 heartbeat at 2Hz ────────────
        self._heartbeat_timer = self.create_timer(0.5, self._check_heartbeat)

        self.get_logger().info(f"Sync mode: {self._sync_mode}")
        self.get_logger().info(f"Calibration offsets: {self._offsets}")

    # ═══════════════════════════════════════════════════════════
    # SIM → REAL callback
    # ═══════════════════════════════════════════════════════════
    def _sim_joint_cb(self, msg: JointState):
        """Receive sim joint states. Forward to ESP32 if SIM_TO_REAL."""
        # Store sim positions
        for i, name in enumerate(msg.name):
            if name in self.JOINT_NAMES:
                idx = self.JOINT_NAMES.index(name)
                if idx < len(msg.position):
                    self._sim_positions_rad[idx] = msg.position[idx]

        if self._sync_mode in ("SIM_TO_REAL", "MIRROR"):
            self._forward_to_real()

    def _forward_to_real(self):
        """Convert sim radians to servo degrees and send to ESP32."""
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(self.JOINT_NAMES)
        cmd.position = []

        for i in range(6):
            # Convert radians to degrees
            sim_deg = math.degrees(self._sim_positions_rad[i])
            # Apply calibration offset
            servo_deg = sim_deg + self._offsets[i]
            # Clamp to safe range
            servo_deg = max(0.0, min(180.0, servo_deg))
            cmd.position.append(servo_deg)

        self._pub_servo_cmds.publish(cmd)

    # ═══════════════════════════════════════════════════════════
    # REAL → SIM callback
    # ═══════════════════════════════════════════════════════════
    def _real_joint_cb(self, msg: JointState):
        """Receive ESP32 servo states. Forward to sim if REAL_TO_SIM."""
        # Store real positions
        for i in range(min(len(msg.position), 6)):
            self._real_positions_deg[i] = msg.position[i]

        if self._sync_mode in ("REAL_TO_SIM", "MIRROR"):
            self._forward_to_sim()

        # Record for teach mode
        if self._record_teach and self._sync_mode == "REAL_TO_SIM":
            self._teach_trajectory.append({
                "timestamp": time.time(),
                "positions_deg": list(self._real_positions_deg),
            })

    def _forward_to_sim(self):
        """Convert real servo degrees to sim radians and update sim."""
        override = JointState()
        override.header.stamp = self.get_clock().now().to_msg()
        override.name = list(self.JOINT_NAMES)
        override.position = []

        for i in range(6):
            # Remove calibration offset
            sim_deg = self._real_positions_deg[i] - self._offsets[i]
            # Convert to radians
            sim_rad = math.radians(sim_deg)
            override.position.append(sim_rad)

        self._pub_sim_override.publish(override)

    # ═══════════════════════════════════════════════════════════
    # Heartbeat monitoring
    # ═══════════════════════════════════════════════════════════
    def _heartbeat_cb(self, msg):
        """Track ESP32 heartbeat."""
        self._last_heartbeat = time.time()
        if not self._esp32_connected:
            self._esp32_connected = True
            self.get_logger().info("ESP32 heartbeat detected — connected!")

    def _check_heartbeat(self):
        """Check if ESP32 is still alive."""
        if self._esp32_connected and (time.time() - self._last_heartbeat) > 3.0:
            self._esp32_connected = False
            self.get_logger().warn("ESP32 heartbeat LOST — connection timeout!")

    # ═══════════════════════════════════════════════════════════
    # Service: Set sync mode
    # ═══════════════════════════════════════════════════════════
    def _set_sync_mode_cb(self, request, response):
        """Switch sync mode. Expects mode name in trigger message."""
        # For Trigger service, we use a simple toggle or check parameter
        prev_mode = self._sync_mode

        # Cycle through modes: SIM_TO_REAL → REAL_TO_SIM → MIRROR → DISABLED
        mode_order = ["SIM_TO_REAL", "REAL_TO_SIM", "MIRROR", "DISABLED"]
        try:
            idx = mode_order.index(prev_mode)
            new_mode = mode_order[(idx + 1) % len(mode_order)]
        except ValueError:
            new_mode = "SIM_TO_REAL"

        self._sync_mode = new_mode

        # Handle teach mode transitions
        teach_msg = Bool()
        if new_mode == "REAL_TO_SIM":
            teach_msg.data = True
            self._pub_teach_mode.publish(teach_msg)
            self.get_logger().info(
                "Teach mode activated. Arm torque will relax. "
                "Move arm to desired position."
            )
        elif prev_mode == "REAL_TO_SIM":
            teach_msg.data = False
            self._pub_teach_mode.publish(teach_msg)
            self.get_logger().info(
                "Teach mode deactivated. Servos re-engaged at current position."
            )

        response.success = True
        response.message = f"Sync: {prev_mode} → {new_mode}"
        self.get_logger().info(response.message)
        return response

    # ═══════════════════════════════════════════════════════════
    # Service: Calibrate offsets
    # ═══════════════════════════════════════════════════════════
    def _calibrate_offsets_cb(self, request, response):
        """
        Compute per-joint calibration offsets.
        Requires arm to be in SIM_TO_REAL mode with ESP32 connected.
        """
        if not self._esp32_connected:
            response.success = False
            response.message = "ESP32 not connected. Cannot calibrate."
            return response

        self.get_logger().info("Starting calibration (5 pose samples)...")

        # Collect offset samples at current position
        samples = []
        for _ in range(5):
            offsets = []
            for i in range(6):
                sim_deg = math.degrees(self._sim_positions_rad[i])
                real_deg = self._real_positions_deg[i]
                offsets.append(sim_deg - real_deg)
            samples.append(offsets)
            time.sleep(0.2)

        # Average offsets
        for i in range(6):
            vals = [s[i] for s in samples]
            self._offsets[i] = sum(vals) / len(vals)

        # Save to file
        cal_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "config", "servo_calibration.yaml"
        )
        self._save_calibration(cal_path)

        offset_str = ", ".join(f"{o:.2f}°" for o in self._offsets)
        response.success = True
        response.message = f"Calibration complete. Offsets: [{offset_str}]"
        self.get_logger().info(response.message)
        return response

    # ═══════════════════════════════════════════════════════════
    # Service: Verify sync
    # ═══════════════════════════════════════════════════════════
    def _verify_sync_cb(self, request, response):
        """Verify sim and real arm are synchronized within 3°."""
        if not self._esp32_connected:
            response.success = False
            response.message = "ESP32 not connected."
            return response

        max_error = 0.0
        errors = []
        for i in range(6):
            sim_deg = math.degrees(self._sim_positions_rad[i])
            real_deg = self._real_positions_deg[i] + self._offsets[i]
            err = abs(sim_deg - real_deg)
            errors.append(err)
            max_error = max(max_error, err)

        synced = max_error < 3.0
        err_str = ", ".join(f"{e:.1f}°" for e in errors)
        response.success = synced
        response.message = (
            f"{'SYNCED' if synced else 'OUT OF SYNC'} — "
            f"Max error: {max_error:.1f}° — Per-joint: [{err_str}]"
        )
        self.get_logger().info(response.message)
        return response

    # ═══════════════════════════════════════════════════════════
    # Status publishing
    # ═══════════════════════════════════════════════════════════
    def _publish_status(self):
        """Publish sync status for dashboard."""
        msg = String()
        msg.data = (
            f"mode={self._sync_mode} "
            f"esp32={'OK' if self._esp32_connected else 'DISCONNECTED'} "
            f"offsets={[round(o, 1) for o in self._offsets]}"
        )
        self._pub_sync_status.publish(msg)

    # ═══════════════════════════════════════════════════════════
    # Calibration file I/O
    # ═══════════════════════════════════════════════════════════
    def _load_calibration(self, path: str):
        """Load calibration offsets from YAML."""
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)
            if data and "offsets_deg" in data:
                self._offsets = data["offsets_deg"][:6]
                self.get_logger().info(f"Loaded calibration from {path}")
        except Exception as e:
            self.get_logger().warn(f"Could not load calibration: {e}")

    def _save_calibration(self, path: str):
        """Save calibration offsets to YAML."""
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            data = {
                "offsets_deg": self._offsets,
                "calibrated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "joint_names": self.JOINT_NAMES,
            }
            with open(path, "w") as f:
                yaml.dump(data, f, default_flow_style=False)
            self.get_logger().info(f"Calibration saved to {path}")
        except Exception as e:
            self.get_logger().error(f"Could not save calibration: {e}")

    # ═══════════════════════════════════════════════════════════
    # Teach trajectory access (used by TeachModeNode)
    # ═══════════════════════════════════════════════════════════
    def get_teach_trajectory(self):
        """Return and clear the recorded teach trajectory."""
        traj = list(self._teach_trajectory)
        self._teach_trajectory.clear()
        return traj


def main(args=None):
    rclpy.init(args=args)
    node = ServoSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
