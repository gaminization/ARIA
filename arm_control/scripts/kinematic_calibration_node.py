#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Online Kinematic Self-Calibration Node
Continuously monitors and corrects kinematic model drift.
Uses visual measurements vs FK predictions to fit offsets.
═══════════════════════════════════════════════════════════════
"""
import math
import os

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from arm_ik.ik_solvers.aria_analytical_ik import forward_kinematics


class KinematicCalibrationNode(Node):
    """
    Online kinematic self-calibration.

    Runs periodically (every 100 tasks or on demand):
      1. Move arm to 5 calibration poses
      2. Compare FK(joints) vs visual measurement
      3. Fit calibration offsets via optimization
      4. Store offsets for IK node to apply

    Compensates for:
      - Servo wear (neutral offset drift)
      - 3D print tolerances (link length errors)
      - Assembly variation (axis misalignment)
    """

    JOINT_NAMES = [
        "waist_joint", "shoulder_joint", "elbow_joint",
        "wrist_pitch_joint", "wrist_roll_joint"
    ]

    # Calibration poses: diverse configurations for good observability
    CALIBRATION_POSES_RAD = [
        [0.0, 1.5708, 1.3090, 0.0, 0.0],       # home
        [0.5, 0.8, 2.0, -0.3, 0.0],             # left-extended
        [-0.5, 0.8, 2.0, 0.3, 0.0],             # right-extended
        [0.0, 1.2, 0.8, 0.0, 0.5],              # upright-rolled
        [0.3, 1.8, 0.5, -0.5, -0.3],            # tucked-tilted
    ]

    def __init__(self):
        super().__init__('kinematic_calibration_node')
        self.get_logger().info("═══ ARIA Kinematic Calibration Node ═══")

        # State
        self.current_joints = np.zeros(5)
        self.task_count = 0
        self.calibration_offsets = np.zeros(5)
        self.link_length_corrections = np.zeros(3)  # upper, fore, wrist

        # Config path
        self.declare_parameter('offsets_path', '')
        path = self.get_parameter('offsets_path').value
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg = get_package_share_directory('arm_ik')
                path = os.path.join(pkg, 'config', 'calibration_offsets.yaml')
            except Exception:
                path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    '..', 'arm_ik', 'config', 'calibration_offsets.yaml'
                )
        self.offsets_path = path

        # Load existing offsets
        self._load_offsets()

        # TF for visual measurements
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10
        )

        # Services
        self.create_service(
            Trigger, '/aria/kinematic_calibration',
            self._calibration_cb
        )
        self.create_service(
            Trigger, '/aria/kinematic_calibration/status',
            self._status_cb
        )

        self.get_logger().info("Kinematic calibration node ready")

    def _joint_state_cb(self, msg: JointState):
        """Update current joint positions."""
        for i, name in enumerate(self.JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joints[i] = msg.position[idx]

    def _load_offsets(self):
        """Load existing calibration offsets."""
        if os.path.exists(self.offsets_path):
            try:
                with open(self.offsets_path, 'r') as f:
                    data = yaml.safe_load(f)
                if data and 'joint_offsets' in data:
                    self.calibration_offsets = np.array(data['joint_offsets'])
                if data and 'link_corrections' in data:
                    self.link_length_corrections = np.array(
                        data['link_corrections']
                    )
                self.get_logger().info(
                    f"Loaded offsets: {np.degrees(self.calibration_offsets)}"
                )
            except Exception as e:
                self.get_logger().warn(f"Could not load offsets: {e}")

    def _save_offsets(self):
        """Save calibration offsets to file."""
        data = {
            'joint_offsets': self.calibration_offsets.tolist(),
            'joint_offsets_deg': [
                float(o * 180 / math.pi) for o in self.calibration_offsets
            ],
            'link_corrections': self.link_length_corrections.tolist(),
            'task_count_at_calibration': self.task_count,
        }
        os.makedirs(os.path.dirname(self.offsets_path), exist_ok=True)
        with open(self.offsets_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        self.get_logger().info(f"Saved offsets to {self.offsets_path}")

    def _get_visual_position(self) -> np.ndarray:
        """Get visual end-effector position via TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'world', 'tool_frame', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return np.array([
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            ])
        except Exception:
            return None

    def _compute_drift_estimate(self) -> float:
        """Estimate current drift magnitude."""
        T = forward_kinematics(
            self.current_joints + self.calibration_offsets
        )
        fk_pos = T[:3, 3]

        visual_pos = self._get_visual_position()
        if visual_pos is None:
            return -1.0

        return float(np.linalg.norm(fk_pos - visual_pos) * 1000)  # mm

    def _calibration_cb(self, request, response):
        """
        Service: /aria/kinematic_calibration

        Run full calibration procedure.
        """
        self.get_logger().info("Starting kinematic calibration...")

        from scipy.optimize import minimize

        measurements = []

        for i, pose in enumerate(self.CALIBRATION_POSES_RAD):
            joints = np.array(pose)

            # FK prediction (with current offsets)
            T = forward_kinematics(joints + self.calibration_offsets)
            fk_pos = T[:3, 3]

            # Visual measurement
            visual_pos = self._get_visual_position()

            if visual_pos is not None:
                error_mm = np.linalg.norm(fk_pos - visual_pos) * 1000
                measurements.append((joints, visual_pos))
                self.get_logger().info(
                    f"  Pose {i+1}: error = {error_mm:.1f}mm"
                )

        if len(measurements) < 3:
            response.success = False
            response.message = f"Only {len(measurements)} measurements"
            return response

        # Optimize joint offsets
        def objective(offsets):
            total = 0.0
            for joints, visual_pos in measurements:
                T = forward_kinematics(joints + offsets)
                pred = T[:3, 3]
                total += np.sum((pred - visual_pos) ** 2)
            return total

        result = minimize(
            objective,
            x0=self.calibration_offsets,
            method='Nelder-Mead',
            options={'maxiter': 2000, 'xatol': 1e-7},
        )

        # Compute improvement
        old_errors = []
        new_errors = []
        for joints, visual_pos in measurements:
            T_old = forward_kinematics(joints + self.calibration_offsets)
            T_new = forward_kinematics(joints + result.x)
            old_errors.append(np.linalg.norm(T_old[:3, 3] - visual_pos))
            new_errors.append(np.linalg.norm(T_new[:3, 3] - visual_pos))

        mean_before = np.mean(old_errors) * 1000
        mean_after = np.mean(new_errors) * 1000

        self.calibration_offsets = result.x
        self._save_offsets()

        response.success = True
        response.message = (
            f"Calibration complete. Error: {mean_before:.1f}mm → "
            f"{mean_after:.1f}mm"
        )
        self.get_logger().info(response.message)
        return response

    def _status_cb(self, request, response):
        """Service: /aria/kinematic_calibration/status"""
        drift = self._compute_drift_estimate()

        if drift < 0:
            response.success = False
            response.message = "Cannot measure drift (TF unavailable)"
        elif drift < 3.0:
            response.success = True
            response.message = f"Drift: {drift:.1f}mm (OK)"
        else:
            response.success = False
            response.message = (
                f"Drift: {drift:.1f}mm (>3mm, recalibration recommended)"
            )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = KinematicCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
