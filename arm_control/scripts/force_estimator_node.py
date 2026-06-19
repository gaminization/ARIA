#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Force Estimator Node
Estimates contact force WITHOUT a force sensor.
Uses servo position error as proxy for torque.
═══════════════════════════════════════════════════════════════
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory


class ForceEstimatorNode(Node):
    """
    Sensorless force estimation.

    Method:
      expected_pos = last commanded joint angle
      actual_pos   = /joint_states reading
      position_error = expected - actual

      Under load, servo lags behind command.
      Lag magnitude correlates with torque:
        estimated_torque = K_spring × position_error

    K_spring values (from servo stall specifications):
      MG995: K = 0.5 N·m/rad  (stall torque ~1.0 N·m)
      SG90:  K = 0.15 N·m/rad (stall torque ~0.18 N·m)

    Note: This is APPROXIMATE. Values improve with calibration
    using known weights.
    """

    JOINT_NAMES = [
        "waist_joint", "shoulder_joint", "elbow_joint",
        "wrist_pitch_joint", "wrist_roll_joint"
    ]

    # Spring constants per joint (N·m/rad)
    K_SPRING = [
        0.5,   # J1 waist (MG995)
        0.5,   # J2 shoulder (MG995)
        0.5,   # J3 elbow (MG995)
        0.15,  # J4 wrist pitch (SG90)
        0.15,  # J5 wrist roll (SG90)
    ]

    # Gripper spring constant (N/rad)
    K_GRIPPER = 2.0  # Approximate: converts angle error to force

    # Slip detection threshold (N)
    SLIP_THRESHOLD_N = 0.05

    def __init__(self):
        super().__init__('force_estimator_node')
        self.get_logger().info("═══ ARIA Force Estimator Node ═══")

        # State
        self.actual_positions = np.zeros(5)
        self.commanded_positions = np.zeros(5)
        self.gripper_actual = 0.0
        self.gripper_commanded = 0.0
        self.prev_gripper_force = 0.0

        # EMA filter coefficient for smooth estimates
        self.alpha = 0.3  # Lower = smoother

        # Running estimates
        self.torque_estimates = np.zeros(5)
        self.gripper_force_estimate = 0.0

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 50  # High rate for accurate tracking
        )

        # Subscribe to commanded trajectory
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self._trajectory_cb, 10
        )

        # Publishers
        self.torque_pub = self.create_publisher(
            Float64MultiArray, '/force/joint_torques', 10)
        self.force_pub = self.create_publisher(
            Float64, '/force/gripper_force', 10)
        self.slip_pub = self.create_publisher(
            Float64, '/force/slip_indicator', 10)

        # Update at 50Hz for responsive force estimation
        self.timer = self.create_timer(0.02, self._estimate_forces)

        self.get_logger().info("Force estimator ready")

    def _joint_state_cb(self, msg: JointState):
        """Update actual joint positions."""
        for i, name in enumerate(self.JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self.actual_positions[i] = msg.position[idx]

        if "gripper_joint" in msg.name:
            idx = msg.name.index("gripper_joint")
            self.gripper_actual = msg.position[idx]

    def _trajectory_cb(self, msg: JointTrajectory):
        """Update commanded positions from trajectory."""
        if not msg.points:
            return

        # Use the last point as the target
        last_point = msg.points[-1]

        for i, name in enumerate(self.JOINT_NAMES):
            if name in msg.joint_names:
                j_idx = msg.joint_names.index(name)
                if j_idx < len(last_point.positions):
                    self.commanded_positions[i] = last_point.positions[j_idx]

    def _estimate_forces(self):
        """Estimate joint torques and gripper force."""
        # ── Joint torque estimation ────────────────────────
        for i in range(5):
            # Position error (commanded - actual)
            error = self.commanded_positions[i] - self.actual_positions[i]

            # Estimated torque = K × error
            raw_torque = self.K_SPRING[i] * error

            # EMA filter for smooth estimates
            self.torque_estimates[i] = (
                self.alpha * raw_torque +
                (1 - self.alpha) * self.torque_estimates[i]
            )

        # Publish joint torques
        torque_msg = Float64MultiArray()
        torque_msg.data = self.torque_estimates.tolist()
        self.torque_pub.publish(torque_msg)

        # ── Gripper force estimation ──────────────────────
        gripper_error = self.gripper_commanded - self.gripper_actual
        raw_force = self.K_GRIPPER * abs(gripper_error)

        self.gripper_force_estimate = (
            self.alpha * raw_force +
            (1 - self.alpha) * self.gripper_force_estimate
        )

        force_msg = Float64()
        force_msg.data = self.gripper_force_estimate
        self.force_pub.publish(force_msg)

        # ── Slip detection ─────────────────────────────────
        # Sudden drop in gripper force indicates slip
        force_delta = self.prev_gripper_force - self.gripper_force_estimate
        self.prev_gripper_force = self.gripper_force_estimate

        slip_msg = Float64()
        slip_msg.data = max(0.0, force_delta)  # Only positive = drop
        self.slip_pub.publish(slip_msg)

        if force_delta > self.SLIP_THRESHOLD_N:
            self.get_logger().warn(
                f"⚠ Possible slip detected! Force drop: {force_delta:.3f}N"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ForceEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
