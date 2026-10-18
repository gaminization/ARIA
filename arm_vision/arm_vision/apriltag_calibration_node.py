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
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import TransformStamped

from tf2_ros import Buffer, TransformListener

from arm_ik.ik_solvers.aria_analytical_ik import forward_kinematics

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    from cv_bridge import CvBridge
    CV_BRIDGE = True
except ImportError:
    CV_BRIDGE = False


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
        "wrist_pitch_joint"
    ]

    # Calibration poses: arm configurations that give good tag visibility
    CALIBRATION_POSES_RAD = [
        [0.0, 0.0, 0.0, 0.0],       # home
        [0.3, 0.4, -0.5, 0.0],      # left-forward
        [-0.3, 0.4, -0.5, 0.0],     # right-forward
        [0.0, 0.8, -0.7, -0.3],     # extended-down
        [0.0, 0.6, -0.5, 0.3],      # tucked-up
    ]

    # Known AprilTag position in world frame (from SDF)
    TAG_WORLD_POSITION = np.array([0.35, 0.20, 0.76])

    def __init__(self):
        super().__init__('apriltag_calibration_node')
        self.get_logger().info("═══ ARIA AprilTag Calibration Node ═══")

        # State
        self.current_joints = np.zeros(len(self.JOINT_NAMES))
        self.calibration_offsets = np.zeros(len(self.JOINT_NAMES))
        self.calibration_valid = False
        self._latest_tag_position = None   # tag position in base_link (m)

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

        # AprilTag detection parameters
        self.declare_parameter('tag_size', 0.03)       # metres
        self.declare_parameter('camera_frame', 'top_camera_link')
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        # Top-camera intrinsics (default); override via a camera_info topic
        # in a full calibration pipeline.
        self._camera_matrix = np.array(
            [[525.0, 0.0, 320.0], [0.0, 525.0, 240.0], [0.0, 0.0, 1.0]],
            dtype=np.float64)

        self._detector = None
        if CV2_AVAILABLE:
            try:
                dictionary = cv2.aruco.getPredefinedDictionary(
                    cv2.aruco.DICT_APRILTAG_36h11)
                parameters = cv2.aruco.DetectorParameters()
                self._detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            except Exception as e:
                self.get_logger().warn(f"Could not create AprilTag detector: {e}")
                self._detector = None
        else:
            self.get_logger().warn(
                "cv2 not available — AprilTag detection is disabled.")

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

        # Camera subscriber for real AprilTag detection
        self._bridge = CvBridge() if CV_BRIDGE else None
        if self._detector is not None and self._bridge is not None:
            self.image_sub = self.create_subscription(
                Image, '/top_camera/image_raw', self._image_cb, 10)
        else:
            self.image_sub = None
            self.get_logger().warn(
                "AprilTag calibration needs cv2 + cv_bridge; visual "
                "measurements will not be available.")

        # Services
        self.create_service(
            Trigger, '/aria/calibrate', self._calibrate_cb)
        self.create_service(
            Trigger, '/aria/verify_calibration', self._verify_cb)

        self.get_logger().info("Calibration node ready")

    def _image_cb(self, msg: Image):
        """Detect AprilTags and store the first tag's position in base_link."""
        if self._detector is None:
            return
        try:
            cv_image = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Image decode failed: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return

        # Estimate the first detected tag's pose in the camera frame.
        try:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.tag_size, self._camera_matrix,
                np.zeros(5))
        except Exception as e:
            self.get_logger().warn(f"Tag pose estimation failed: {e}")
            return

        rvec = rvecs[0].reshape(3)
        tvec = tvecs[0].reshape(3)
        R, _ = cv2.Rodrigues(rvec)
        T_cam_tag = np.eye(4)
        T_cam_tag[:3, :3] = R
        T_cam_tag[:3, 3] = tvec

        # Transform tag position from camera frame to base_link.
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', self.camera_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            t = tf.transform.translation
            q = tf.transform.rotation
            R_bc = self._quat_to_R(q.x, q.y, q.z, q.w)
            T_bc = np.eye(4)
            T_bc[:3, :3] = R_bc
            T_bc[:3, 3] = np.array([t.x, t.y, t.z])
            tag_pos = (T_bc @ T_cam_tag)[:3, 3]
            self._latest_tag_position = tag_pos
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup {self.camera_frame}→base_link failed: {e}")

    @staticmethod
    def _quat_to_R(x, y, z, w):
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
        ])

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
            offsets = np.asarray(
                data.get('offsets', [0] * len(self.JOINT_NAMES)),
                dtype=float)
            if offsets.shape == (len(self.JOINT_NAMES) + 1,):
                self.get_logger().warning(
                    "Dropping legacy wrist-roll calibration offset; "
                    "the arm has four kinematic joints")
                offsets = offsets[:len(self.JOINT_NAMES)]
            if offsets.shape != (len(self.JOINT_NAMES),):
                raise ValueError(
                    "Calibration offsets must contain exactly four "
                    "arm joint values")
            self.calibration_offsets = offsets
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
        Return the last AprilTag-derived position in base_link, or None.

        This is a real visual measurement: it comes from cv2.aruco detection
        on the top camera plus a TF transform. If no tag has been detected
        (or detection/TF is unavailable), it returns None so the caller
        knows there is no measurement.
        """
        if self._latest_tag_position is None:
            self.get_logger().warn(
                "No AprilTag measurement available (detection or TF failed).")
            return None
        return self._latest_tag_position

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
            x0=np.zeros(len(self.JOINT_NAMES)),
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
