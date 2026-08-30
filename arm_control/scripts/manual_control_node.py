#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Manual Control Node
# Primary manual control interface for the ARIA 5-DoF arm.
# All control methods publish to /joint_trajectory_controller.
# Safety: soft limits (1° margin), max speed 90°/s, e-stop.
# ═══════════════════════════════════════════════════════════════
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand
from std_srvs.srv import Trigger
from arm_interfaces.srv import SetJoint, SetAllJoints, GoNamedPose
from builtin_interfaces.msg import Duration


class ManualControlNode(Node):
    """
    Primary manual control interface for ARIA.
    All control methods publish to /joint_trajectory_controller.
    Safety: soft limits (1° margin), max speed 90°/s, e-stop.
    """

    # ── Constants ──────────────────────────────────────────────
    JOINT_NAMES = [
        "waist_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist_pitch_joint",
        "gripper_joint",
    ]

    ARM_JOINT_NAMES = [
        "waist_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist_pitch_joint",
    ]

    # Home = straight upright reference pose (matching CAD zero and reference positions.png)
    HOME_ANGLES_DEG = [0.0, 0.0, 0.0, 0.0, 0.0]

    # Joint limits in degrees [min, max] (matching -1.571 to +1.571 rad)
    JOINT_LIMITS_DEG = [
        [-180.0, 180.0],  # waist
        [-90.0,  90.0],   # shoulder
        [-90.0,  90.0],   # elbow
        [-90.0,  90.0],   # wrist_pitch
        [  0.0,  60.0],   # gripper (0=closed, 45=open)
    ]

    SOFT_LIMIT_MARGIN_DEG = 1.0  # degrees before hard limit
    MAX_SPEED_DEG_PER_S = 90.0   # max joint speed

    # Named poses: [waist, shoulder, elbow, wrist_pitch, gripper]
    NAMED_POSES = {
        "home":    [  0.0,   0.0,   0.0,   0.0,  0.0],
        "ready":   [  0.0,  35.0, -55.0,  20.0, 20.0],
        "reach":   [  0.0,  48.0, -70.0,  22.0, 25.0],
        "inspect": [  0.0,  20.0, -30.0,  10.0, 20.0],
        "folded":  [  0.0, -30.0,  60.0, -30.0,  0.0],
    }

    def __init__(self):
        super().__init__("manual_control_node")
        self.get_logger().info("═══ ARIA Manual Control Node starting ═══")

        self.callback_group = ReentrantCallbackGroup()

        # ── State ──────────────────────────────────────────────
        self.current_positions_rad = [0.0] * 5
        self.target_positions_rad = [0.0] * 5
        self.estop_active = False
        self.motion_complete = True

        # ── Subscriber: joint states ───────────────────────────
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states",
            self._joint_state_cb, 10
        )

        # ── Publisher: trajectory commands ─────────────────────
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        # ── Action client: gripper ─────────────────────────────
        self.gripper_client = ActionClient(
            self, GripperCommand,
            "/gripper_action_controller/gripper_cmd",
            callback_group=self.callback_group
        )

        # ── Subscriber: streaming control ──────────────────────
        self.stream_sub = self.create_subscription(
            JointState, "/aria/joint_stream",
            self._stream_cb, 50
        )

        # ── Services ───────────────────────────────────────────
        self.create_service(
            SetJoint, "/aria/set_joint",
            self._set_joint_cb,
            callback_group=self.callback_group
        )
        self.create_service(
            SetAllJoints, "/aria/set_all_joints",
            self._set_all_joints_cb,
            callback_group=self.callback_group
        )
        self.create_service(
            GoNamedPose, "/aria/go_named_pose",
            self._go_named_pose_cb,
            callback_group=self.callback_group
        )
        self.create_service(
            Trigger, "/aria/open_gripper",
            self._open_gripper_cb,
            callback_group=self.callback_group
        )
        self.create_service(
            Trigger, "/aria/close_gripper",
            self._close_gripper_cb,
            callback_group=self.callback_group
        )
        self.create_service(
            Trigger, "/aria/estop",
            self._estop_cb,
            callback_group=self.callback_group
        )
        self.create_service(
            Trigger, "/aria/release_estop",
            self._release_estop_cb,
            callback_group=self.callback_group
        )

        # ── Status publisher at 10Hz ───────────────────────────
        self.status_pub = self.create_publisher(
            JointState, "/aria/manual_status", 10
        )
        self.status_timer = self.create_timer(0.1, self._publish_status)

        # ── Auto-home on startup ────────────────────────────────
        self._startup_timer = self.create_timer(1.5, self._initial_home_callback)

        self.get_logger().info("═══ ARIA Manual Control Node ready ═══")
        self.get_logger().info("Services: /aria/set_joint, /aria/set_all_joints, "
                               "/aria/go_named_pose, /aria/open_gripper, "
                               "/aria/close_gripper, /aria/estop, /aria/release_estop")

    def _initial_home_callback(self):
        """Move to home pose on initial node start."""
        if hasattr(self, '_startup_timer') and self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None
        self.get_logger().info("Executing automatic initial home pose...")
        self._send_arm_command(self.HOME_ANGLES_DEG, speed_deg_per_s=30.0)

    # ═══════════════════════════════════════════════════════════
    # SAFETY CHECKS — run on EVERY command
    # ═══════════════════════════════════════════════════════════
    def _check_estop(self):
        """Return True if e-stop is active (commands should be rejected)."""
        return self.estop_active

    def _check_nan(self, value):
        """Reject NaN/Inf values."""
        return math.isnan(value) or math.isinf(value)

    def _apply_soft_limits(self, joint_idx, angle_deg):
        """
        Apply soft limits (1° margin from hard limits).
        Returns clamped angle in degrees and whether it was clamped.
        """
        limits = self.JOINT_LIMITS_DEG[joint_idx]
        soft_min = limits[0] + self.SOFT_LIMIT_MARGIN_DEG
        soft_max = limits[1] - self.SOFT_LIMIT_MARGIN_DEG
        clamped = max(soft_min, min(soft_max, angle_deg))
        was_clamped = (clamped != angle_deg)

        if was_clamped:
            self.get_logger().warn(
                f"Joint {self.JOINT_NAMES[joint_idx]}: angle {angle_deg:.1f}° "
                f"clamped to soft limit {clamped:.1f}°"
            )

        return clamped, was_clamped

    def _cap_speed(self, speed_deg_per_s):
        """Cap speed at maximum allowed."""
        if speed_deg_per_s <= 0:
            return 30.0  # default speed
        return min(speed_deg_per_s, self.MAX_SPEED_DEG_PER_S)

    @staticmethod
    def _deg_to_rad(deg):
        return deg * math.pi / 180.0

    @staticmethod
    def _rad_to_deg(rad):
        return rad * 180.0 / math.pi

    # ═══════════════════════════════════════════════════════════
    # JOINT STATE CALLBACK
    # ═══════════════════════════════════════════════════════════
    def _joint_state_cb(self, msg: JointState):
        """Update current joint positions from /joint_states."""
        for i, name in enumerate(self.JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_positions_rad[i] = msg.position[idx]

    # ═══════════════════════════════════════════════════════════
    # STREAMING CONTROL CALLBACK
    # ═══════════════════════════════════════════════════════════
    def _stream_cb(self, msg: JointState):
        """Process streaming joint commands at up to 50Hz."""
        if self._check_estop():
            return  # Silently ignore during e-stop

        n_joints = len(self.JOINT_NAMES)
        if len(msg.position) >= n_joints:
            angles_deg = [self._rad_to_deg(p) for p in msg.position[:n_joints]]
            self._send_arm_command(angles_deg, speed_deg_per_s=60.0)

    # ═══════════════════════════════════════════════════════════
    # COMMAND EXECUTION
    # ═══════════════════════════════════════════════════════════
    def _send_arm_command(self, angles_deg, speed_deg_per_s=30.0):
        """
        Send position command to the arm joints via JointTrajectory.
        angles_deg: list of target angles in degrees
        speed_deg_per_s: max speed per joint
        """
        speed = self._cap_speed(speed_deg_per_s)
        n_joints = len(self.JOINT_NAMES)

        # Apply safety checks and soft limits to all joints
        safe_angles_deg = []
        for i in range(min(len(angles_deg), n_joints)):
            if self._check_nan(angles_deg[i]):
                self.get_logger().error(
                    f"NaN/Inf rejected for joint {self.JOINT_NAMES[i]}")
                return False
            clamped, _ = self._apply_soft_limits(i, angles_deg[i])
            safe_angles_deg.append(clamped)

        # Pad with current positions if fewer angles provided
        while len(safe_angles_deg) < n_joints:
            idx = len(safe_angles_deg)
            safe_angles_deg.append(self._rad_to_deg(self.current_positions_rad[idx]))

        # Compute duration from max angular displacement
        max_displacement_deg = 0.0
        for i in range(n_joints):
            current_deg = self._rad_to_deg(self.current_positions_rad[i])
            displacement = abs(safe_angles_deg[i] - current_deg)
            max_displacement_deg = max(max_displacement_deg, displacement)

        duration_s = max(max_displacement_deg / speed, 0.1)

        # Build and publish trajectory for ALL 6 joints (including gripper)
        # The joint_trajectory_controller is configured with all 6 joints
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [self._deg_to_rad(a) for a in safe_angles_deg]
        point.time_from_start = Duration(
            sec=int(duration_s),
            nanosec=int((duration_s % 1.0) * 1e9)
        )
        traj.points.append(point)

        self.traj_pub.publish(traj)

        # Update target state
        self.target_positions_rad = [self._deg_to_rad(a) for a in safe_angles_deg]
        self.motion_complete = False

        return True

    def _send_gripper_command(self, position_rad):
        """Send gripper command via JointTrajectory (gripper is in JTC)."""
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES

        # Keep current arm positions, only change gripper
        point = JointTrajectoryPoint()
        positions = list(self.current_positions_rad)
        gripper_idx = self.JOINT_NAMES.index("gripper_joint") if "gripper_joint" in self.JOINT_NAMES else 4
        if gripper_idx < len(positions):
            positions[gripper_idx] = position_rad  # Update gripper
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5s
        traj.points.append(point)

        self.traj_pub.publish(traj)

    # ═══════════════════════════════════════════════════════════
    # SERVICE CALLBACKS
    # ═══════════════════════════════════════════════════════════
    def _set_joint_cb(self, request, response):
        """Service: /aria/set_joint — command a single joint."""
        # E-stop check
        if self._check_estop():
            response.success = False
            response.message = "E-STOP active — command rejected"
            response.actual_angle_deg = 0.0
            return response

        # Find joint index
        joint_name = request.joint_name
        # Allow short names (e.g., "waist") or full names (e.g., "waist_joint")
        if not joint_name.endswith("_joint"):
            joint_name += "_joint"

        if joint_name not in self.JOINT_NAMES:
            response.success = False
            response.message = f"Unknown joint: {request.joint_name}"
            response.actual_angle_deg = 0.0
            return response

        joint_idx = self.JOINT_NAMES.index(joint_name)

        # NaN check
        if self._check_nan(request.angle_deg):
            response.success = False
            response.message = "NaN/Inf angle rejected"
            response.actual_angle_deg = 0.0
            return response

        # Apply soft limits
        clamped_deg, was_clamped = self._apply_soft_limits(
            joint_idx, request.angle_deg)

        # Build target: keep current positions, update one joint
        target_deg = [self._rad_to_deg(p) for p in self.current_positions_rad]
        target_deg[joint_idx] = clamped_deg

        speed = self._cap_speed(request.speed_deg_per_s)
        success = self._send_arm_command(target_deg, speed)

        response.success = success
        response.actual_angle_deg = clamped_deg
        if was_clamped:
            response.message = f"Clamped to soft limit: {clamped_deg:.1f}°"
        else:
            response.message = f"Moving {request.joint_name} to {clamped_deg:.1f}°"

        self.get_logger().info(response.message)
        return response

    def _set_all_joints_cb(self, request, response):
        """Service: /aria/set_all_joints — command all 6 joints."""
        if self._check_estop():
            response.success = False
            response.expected_duration_s = 0.0
            return response

        n_joints = len(self.JOINT_NAMES)
        if len(request.angles_deg) != n_joints:
            response.success = False
            response.expected_duration_s = 0.0
            self.get_logger().error(
                f"Expected {n_joints} angles, got {len(request.angles_deg)}")
            return response

        angles_deg = list(request.angles_deg)
        speed = self._cap_speed(request.speed_deg_per_s)
        success = self._send_arm_command(angles_deg, speed)

        # Compute expected duration
        max_disp = 0.0
        for i in range(n_joints):
            current_deg = self._rad_to_deg(self.current_positions_rad[i])
            disp = abs(angles_deg[i] - current_deg)
            max_disp = max(max_disp, disp)

        response.success = success
        response.expected_duration_s = max_disp / speed if speed > 0 else 0.0

        self.get_logger().info(
            f"Moving all joints, ETA: {response.expected_duration_s:.1f}s")
        return response

    def _go_named_pose_cb(self, request, response):
        """Service: /aria/go_named_pose — move to a named pose."""
        if self._check_estop():
            response.success = False
            response.message = "E-STOP active — command rejected"
            return response

        pose_name = request.pose_name.lower()
        if pose_name not in self.NAMED_POSES:
            response.success = False
            response.message = (
                f"Unknown pose '{pose_name}'. "
                f"Available: {list(self.NAMED_POSES.keys())}"
            )
            return response

        angles_deg = self.NAMED_POSES[pose_name]
        success = self._send_arm_command(angles_deg, speed_deg_per_s=30.0)

        response.success = success
        response.message = f"Moving to '{pose_name}' pose"
        self.get_logger().info(response.message)
        return response

    def _open_gripper_cb(self, request, response):
        """Service: /aria/open_gripper."""
        if self._check_estop():
            response.success = False
            response.message = "E-STOP active"
            return response

        self._send_gripper_command(self._deg_to_rad(44.0))  # Near full open
        response.success = True
        response.message = "Opening gripper"
        self.get_logger().info("Opening gripper to 44°")
        return response

    def _close_gripper_cb(self, request, response):
        """Service: /aria/close_gripper."""
        if self._check_estop():
            response.success = False
            response.message = "E-STOP active"
            return response

        self._send_gripper_command(self._deg_to_rad(1.0))  # Near closed
        response.success = True
        response.message = "Closing gripper"
        self.get_logger().info("Closing gripper to 1°")
        return response

    def _estop_cb(self, request, response):
        """Service: /aria/estop — IMMEDIATE stop, hold position."""
        self.estop_active = True
        self.motion_complete = True

        # Send current position as target to hold
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = list(self.current_positions_rad)
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1s
        traj.points.append(point)
        self.traj_pub.publish(traj)

        response.success = True
        response.message = "E-STOP ACTIVATED — all motion halted"
        self.get_logger().warn("🔴 E-STOP ACTIVATED")
        return response

    def _release_estop_cb(self, request, response):
        """Service: /aria/release_estop."""
        self.estop_active = False
        response.success = True
        response.message = "E-stop released — commands accepted"
        self.get_logger().info("🟢 E-stop released")
        return response

    # ═══════════════════════════════════════════════════════════
    # STATUS PUBLISHER (10Hz)
    # ═══════════════════════════════════════════════════════════
    def _publish_status(self):
        """Publish manual control status at 10Hz."""
        n_joints = len(self.JOINT_NAMES)
        # Check if motion is complete (all joints within 2° of target)
        all_within = True
        for i in range(n_joints):
            current_deg = self._rad_to_deg(self.current_positions_rad[i])
            target_deg = self._rad_to_deg(self.target_positions_rad[i])
            if abs(current_deg - target_deg) > 2.0:
                all_within = False
                break
        self.motion_complete = all_within

        # Check joints within limits
        joints_ok = True
        for i in range(n_joints):
            current_deg = self._rad_to_deg(self.current_positions_rad[i])
            limits = self.JOINT_LIMITS_DEG[i]
            if current_deg < limits[0] or current_deg > limits[1]:
                joints_ok = False
                break

        # Publish as JointState with custom fields in effort
        status = JointState()
        status.header.stamp = self.get_clock().now().to_msg()
        status.name = self.JOINT_NAMES

        # Current angles in radians
        status.position = list(self.current_positions_rad)
        # Target angles in radians
        status.velocity = list(self.target_positions_rad)
        # Pack status flags into effort: [motion_complete, estop_active, joints_ok, 0, 0, 0]
        status.effort = [
            1.0 if self.motion_complete else 0.0,
            1.0 if self.estop_active else 0.0,
            1.0 if joints_ok else 0.0,
            0.0, 0.0, 0.0
        ]

        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Manual control node shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
