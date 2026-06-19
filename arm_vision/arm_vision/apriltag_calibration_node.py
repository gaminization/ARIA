#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA AprilTag Calibration Node
Detects AprilTag at known world position, computes FK errors,
and fits calibration offsets via least-squares.
═══════════════════════════════════════════════════════════════
"""
import math
import os

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

from tf2_ros import Buffer, TransformListener

from arm_ik.ik_solvers.aria_analytical_ik import forward_kinematics


class AprilTagCalibrationNode(Node):
    """
    Camera-to-arm calibration using AprilTag.

    Process:
      1. Detect AprilTag at known world position
      2. Move arm to calibration poses
      3. At each pose: compare FK vs visual measurement
      4. Fit calibration offsets via least squares
    """

    JOINT_NAMES = [
        "waist_joint", "shoulder_joint", "elbow_joint",
        "wrist_pitch_joint", "wrist_roll_joint"
    ]

    # Calibration poses: arm configurations that give good tag visibility
    CALIBRATION_POSES_RAD = [
        [0.0, 1.5708, 1.3090, 0.0, 0.0],       # home
        [0.3, 1.2, 1.5, 0.0, 0.0],              # left-forward
        [-0.3, 1.2, 1.5, 0.0, 0.0],             # right-forward
        [0.0, 0.8, 2.0, -0.3, 0.0],             # extended-down
        [0.0, 1.8, 0.5, 0.3, 0.0],              # tucked-up
    ]

    # Known AprilTag position in world frame (from SDF)
    TAG_WORLD_POSITION = np.array([0.35, 0.20, 0.76])

    def __init__(self):
        super().__init__('apriltag_calibration_node')
        self.get_logger().info("═══ ARIA AprilTag Calibration Node ═══")

        # State
        self.current_joints = np.zeros(5)
        self.calibration_offsets = np.zeros(5)
        self.calibration_valid = False

        # Config path
        self.declare_parameter('offsets_path', '')
        path = self.get_parameter('offsets_path').value
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg = get_package_share_directory('arm_vision')
                path = os.path.join(pkg, 'config', 'calibration_offsets.yaml')
            except Exception:
                path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'config', 'calibration_offsets.yaml'
                )
        self.offsets_path = path

        # Load existing offsets if available
        if os.path.exists(self.offsets_path):
            self._load_offsets()

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Joint state subscriber
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10
        )

        # Services
        self.create_service(
            Trigger, '/aria/calibrate', self._calibrate_cb)
        self.create_service(
            Trigger, '/aria/verify_calibration', self._verify_cb)

        self.get_logger().info("Calibration node ready")

    def _joint_state_cb(self, msg: JointState):
        """Update current joint positions."""
        for i, name in enumerate(self.JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joints[i] = msg.position[idx]

    def _load_offsets(self):
        """Load calibration offsets from file."""
        try:
            with open(self.offsets_path, 'r') as f:
                data = yaml.safe_load(f)
            self.calibration_offsets = np.array(data.get('offsets', [0]*5))
            self.calibration_valid = data.get('valid', False)
            self.get_logger().info(
                f"Loaded calibration offsets: {self.calibration_offsets}"
            )
        except Exception as e:
            self.get_logger().warn(f"Could not load offsets: {e}")

    def _save_offsets(self, offsets: np.ndarray, errors_before: list,
                      errors_after: list):
        """Save calibration offsets to file."""
        data = {
            'offsets': offsets.tolist(),
            'valid': True,
            'mean_error_before_mm': float(np.mean(errors_before) * 1000),
            'mean_error_after_mm': float(np.mean(errors_after) * 1000),
            'per_joint_offsets_deg': [float(o * 180 / math.pi) for o in offsets],
        }
        os.makedirs(os.path.dirname(self.offsets_path), exist_ok=True)
        with open(self.offsets_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        self.get_logger().info(f"Saved offsets to {self.offsets_path}")

    def _get_visual_end_effector_position(self) -> np.ndarray:
        """
        Get visual end-effector position via AprilTag detection.
        In sim: use TF lookup for tool_frame.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'world', 'tool_frame', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pos = transform.transform.translation
            return np.array([pos.x, pos.y, pos.z])
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def _calibrate_cb(self, request, response):
        """
        Service: /aria/calibrate

        Moves arm to calibration poses, measures FK vs visual error,
        fits least-squares calibration offsets.
        """
        self.get_logger().info("Starting calibration...")

        measurements = []  # (joints, fk_pos, visual_pos)

        for i, pose in enumerate(self.CALIBRATION_POSES_RAD):
            # Note: In a full implementation, we'd command the arm
            # to each pose and wait. Here we collect data from
            # current position (assumes manual positioning or
            # service calls to manual_control_node).
            joints = np.array(pose)

            # FK prediction
            T = forward_kinematics(joints)
            fk_pos = T[:3, 3]

            # Visual measurement
            visual_pos = self._get_visual_end_effector_position()

            if visual_pos is not None:
                measurements.append((joints, fk_pos, visual_pos))
                error = np.linalg.norm(fk_pos - visual_pos)
                self.get_logger().info(
                    f"  Pose {i+1}: FK error = {error*1000:.1f}mm"
                )

        if len(measurements) < 3:
            response.success = False
            response.message = (
                f"Only {len(measurements)} measurements collected (need ≥3)"
            )
            return response

        # Compute errors before calibration
        errors_before = [
            np.linalg.norm(m[1] - m[2]) for m in measurements
        ]

        # Fit calibration offsets via least squares
        # We seek offsets δ such that FK(joints + δ) ≈ visual_pos
        # Linearize: FK(q + δ) ≈ FK(q) + J·δ
        # → minimize ||J·δ - (visual - FK)||²

        from scipy.optimize import minimize

        def objective(offsets):
            total_error = 0.0
            for joints, fk_pos, visual_pos in measurements:
                T = forward_kinematics(joints + offsets)
                pred_pos = T[:3, 3]
                total_error += np.sum((pred_pos - visual_pos) ** 2)
            return total_error

        result = minimize(
            objective,
            x0=np.zeros(5),
            method='Nelder-Mead',
            options={'maxiter': 1000, 'xatol': 1e-6},
        )

        offsets = result.x

        # Compute errors after calibration
        errors_after = []
        for joints, fk_pos, visual_pos in measurements:
            T = forward_kinematics(joints + offsets)
            pred_pos = T[:3, 3]
            errors_after.append(np.linalg.norm(pred_pos - visual_pos))

        self.calibration_offsets = offsets
        self.calibration_valid = True
        self._save_offsets(offsets, errors_before, errors_after)

        mean_before = np.mean(errors_before) * 1000
        mean_after = np.mean(errors_after) * 1000

        response.success = True
        response.message = (
            f"Calibration complete. "
            f"Mean FK error before: {mean_before:.1f}mm, "
            f"after: {mean_after:.1f}mm"
        )
        self.get_logger().info(response.message)
        return response

    def _verify_cb(self, request, response):
        """Service: /aria/verify_calibration — check drift."""
        visual_pos = self._get_visual_end_effector_position()
        if visual_pos is None:
            response.success = False
            response.message = "Cannot get visual measurement"
            return response

        T = forward_kinematics(self.current_joints + self.calibration_offsets)
        fk_pos = T[:3, 3]
        drift = np.linalg.norm(fk_pos - visual_pos) * 1000  # mm

        if drift < 3.0:
            response.success = True
            response.message = f"Calibration OK. Drift: {drift:.1f}mm"
        else:
            response.success = False
            response.message = (
                f"Drift: {drift:.1f}mm (>3mm). Recalibration recommended."
            )

        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
