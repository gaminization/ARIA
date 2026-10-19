#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Camera Node
Manages both cameras (top + wrist) for sim and real modes.
Re-publishes with standardized topics and camera_info.
═══════════════════════════════════════════════════════════════
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool

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


class CameraNode(Node):
    """
    Camera manager for ARIA. Handles sim and hardware identically.

    Sim mode:  subscribes to Gazebo camera topics, re-publishes
    Real mode: opens OpenCV VideoCapture (Stage 4)

    Camera intrinsics:
      C270:     fx=721, fy=721, cx=640, cy=360 (1280×720)
      ESP32:    fx=455, fy=455, cx=320, cy=240 (640×480)
    """

    # ── Camera intrinsics ──────────────────────────────────
    # Logitech C270 (1280×720, 60° HFOV)
    C270_INTRINSICS = {
        'width': 1280, 'height': 720,
        'fx': 721.0, 'fy': 721.0,
        'cx': 640.0, 'cy': 360.0,
        'k1': 0.15, 'k2': -0.08, 'p1': 0.0, 'p2': 0.0, 'k3': 0.0,
    }

    # ESP32-CAM OV2640 (640×480, 66° HFOV)
    ESP32_INTRINSICS = {
        'width': 640, 'height': 480,
        'fx': 455.0, 'fy': 455.0,
        'cx': 320.0, 'cy': 240.0,
        'k1': 0.25, 'k2': -0.12, 'p1': 0.0, 'p2': 0.0, 'k3': 0.0,
    }

    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("═══ ARIA Camera Node starting ═══")

        self.declare_parameter('mode', 'sim')
        self.mode = self.get_parameter('mode').value

        # Device parameters are only used in real/hardware mode. Declare them
        # with defaults up front so they can be read (and overridden) in every
        # mode without raising ParameterNotDeclaredException. In sim mode they
        # are simply unused. This keeps the sim relay able to start even when
        # no hardware parameters are supplied.
        for _param, _default in (
            ('camera_id', -1),
            ('top_device', 0),
            ('wrist_url', 'http://192.168.4.1:81/stream'),
        ):
            if not self.has_parameter(_param):
                self.declare_parameter(_param, _default)

        # Support camera_id parameter from hardware launch (maps to top_device/wrist_url)
        # Hardware launch convention:
        #   camera_id = 0 → top camera, use device index 0
        #   camera_id = 1 → wrist camera, use default ESP32 URL (not a device index)
        #   camera_id absent (-1) → use original top_device/wrist_url parameters
        camera_id = self.get_parameter('camera_id').value
        if camera_id == 0:
            # Use camera_id as top device index, keep default wrist URL
            self.get_logger().info("Using camera_id=0 as top_device index")
            self._top_device = camera_id
            self._wrist_url = self.get_parameter('wrist_url').value
        elif camera_id == 1:
            # camera_id=1 for wrist camera: use original defaults (ESP32 URL)
            # This preserves the launch file intent: "Or ESP32-CAM HTTP stream URL"
            self.get_logger().info("Using camera_id=1: keeping default camera parameters")
            self._top_device = self.get_parameter('top_device').value
            self._wrist_url = self.get_parameter('wrist_url').value
        else:
            # No/unknown camera_id: fall back to original parameters
            self._top_device = self.get_parameter('top_device').value
            self._wrist_url = self.get_parameter('wrist_url').value

        # Timestamp tracking for synchronization
        self.top_camera_last_stamp = None
        self.wrist_camera_last_stamp = None
        self.sync_threshold_ns = 33_000_000  # 33ms

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # ── Sim mode: relay Gazebo topics ──────────────────
        if self.mode == 'sim':
            self.top_sub = self.create_subscription(
                Image, '/top_camera/image_raw',
                self._top_camera_cb, qos_sensor
            )
            self.wrist_sub = self.create_subscription(
                Image, '/wrist_camera/image_raw',
                self._wrist_camera_cb, qos_sensor
            )
        else:
            # Real mode: capture from real devices.
            # Top camera: local UVC device (Logitech C270).
            # Wrist camera: ESP32-CAM MJPEG stream over WiFi.
            self._top_device = int(self._top_device)
            self._wrist_url = str(self._wrist_url)

            self._top_cap = None
            self._wrist_cap = None
            if CV2_AVAILABLE:
                self._top_cap = cv2.VideoCapture(self._top_device)
                if not self._top_cap.isOpened():
                    self.get_logger().error(
                        f"Could not open top camera device {self._top_device}")
                    self._top_cap = None
                self._wrist_cap = cv2.VideoCapture(self._wrist_url)
                if not self._wrist_cap.isOpened():
                    self.get_logger().error(
                        f"Could not open wrist camera stream {self._wrist_url}")
                    self._wrist_cap = None
                if self._top_cap or self._wrist_cap:
                    self.create_timer(1.0 / 15.0, self._capture_loop)
            else:
                self.get_logger().error(
                    "cv2 not available — real camera mode cannot capture.")

        # ── Publishers ─────────────────────────────────────
        self.top_pub = self.create_publisher(
            Image, '/aria/top_camera/image_raw', 10)
        self.top_info_pub = self.create_publisher(
            CameraInfo, '/aria/top_camera/camera_info', 10)

        self.wrist_pub = self.create_publisher(
            Image, '/aria/wrist_camera/image_raw', 10)
        self.wrist_info_pub = self.create_publisher(
            CameraInfo, '/aria/wrist_camera/camera_info', 10)

        self.sync_pub = self.create_publisher(
            Bool, '/aria/cameras/synchronized', 10)

        # Build CameraInfo messages
        self.top_info = self._build_camera_info(
            self.C270_INTRINSICS, 'top_camera_link')
        self.wrist_info = self._build_camera_info(
            self.ESP32_INTRINSICS, 'wrist_camera_link')

        # Sync check timer at 30Hz
        self.create_timer(1.0 / 30.0, self._check_sync)

        self.get_logger().info(f"Camera node ready (mode={self.mode})")

    def _build_camera_info(self, intrinsics: dict,
                           frame_id: str) -> CameraInfo:
        """Build CameraInfo message from intrinsic parameters."""
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width = intrinsics['width']
        info.height = intrinsics['height']
        info.distortion_model = 'plumb_bob'

        # Distortion coefficients [k1, k2, p1, p2, k3]
        info.d = [
            intrinsics['k1'], intrinsics['k2'],
            intrinsics['p1'], intrinsics['p2'],
            intrinsics['k3'],
        ]

        # Camera matrix K (3×3)
        fx, fy = intrinsics['fx'], intrinsics['fy']
        cx, cy = intrinsics['cx'], intrinsics['cy']
        info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]

        # Rectification matrix (identity for monocular)
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Projection matrix P (3×4)
        info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        return info

    def _top_camera_cb(self, msg: Image):
        """Relay top camera image with camera_info."""
        self.top_camera_last_stamp = msg.header.stamp

        # Re-publish with ARIA namespace
        self.top_pub.publish(msg)

        # Publish camera info with matching timestamp
        info = CameraInfo()
        info.header = msg.header
        info.header.frame_id = 'top_camera_link'
        info.width = self.top_info.width
        info.height = self.top_info.height
        info.distortion_model = self.top_info.distortion_model
        info.d = list(self.top_info.d)
        info.k = list(self.top_info.k)
        info.r = list(self.top_info.r)
        info.p = list(self.top_info.p)
        self.top_info_pub.publish(info)

    def _wrist_camera_cb(self, msg: Image):
        """Relay wrist camera image with camera_info."""
        self.wrist_camera_last_stamp = msg.header.stamp

        self.wrist_pub.publish(msg)

        info = CameraInfo()
        info.header = msg.header
        info.header.frame_id = 'wrist_camera_link'
        info.width = self.wrist_info.width
        info.height = self.wrist_info.height
        info.distortion_model = self.wrist_info.distortion_model
        info.d = list(self.wrist_info.d)
        info.k = list(self.wrist_info.k)
        info.r = list(self.wrist_info.r)
        info.p = list(self.wrist_info.p)
        self.wrist_info_pub.publish(info)

    def _capture_loop(self):
        """Real mode: grab frames from the devices and publish them."""
        if CV_BRIDGE:
            bridge = CvBridge()
        else:
            bridge = None

        if self._top_cap is not None:
            ret, frame = self._top_cap.read()
            if ret and bridge is not None:
                msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'top_camera_link'
                self.top_pub.publish(msg)
                self.top_camera_last_stamp = msg.header.stamp

        if self._wrist_cap is not None:
            ret, frame = self._wrist_cap.read()
            if ret and bridge is not None:
                msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'wrist_camera_link'
                self.wrist_pub.publish(msg)
                self.wrist_camera_last_stamp = msg.header.stamp

    def _check_sync(self):
        """Check if both cameras are temporally synchronized."""
        synced = False

        if (self.top_camera_last_stamp is not None and
                self.wrist_camera_last_stamp is not None):
            top_ns = (self.top_camera_last_stamp.sec * 1_000_000_000 +
                      self.top_camera_last_stamp.nanosec)
            wrist_ns = (self.wrist_camera_last_stamp.sec * 1_000_000_000 +
                        self.wrist_camera_last_stamp.nanosec)
            delta = abs(top_ns - wrist_ns)
            synced = delta < self.sync_threshold_ns

        msg = Bool()
        msg.data = synced
        self.sync_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
