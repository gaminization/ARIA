#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Visual Servoing Node
Position-based visual servoing for final approach.
Uses wrist camera for fine alignment, top camera for oversight.
═══════════════════════════════════════════════════════════════
"""
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class VisualServoNode(Node):
    """
    Position-based visual servoing for ARIA.

    Activates when gripper is within 8cm of target.
    Uses wrist camera to center object in field of view.

    Control loop (30Hz):
      1. Detect object in wrist camera
      2. Compute pixel error from image center
      3. Convert to end-effector velocity correction
      4. Apply correction via trajectory commands
    """

    # Activation distance (meters)
    ACTIVATION_DISTANCE_M = 0.08

    # Target pixel position (center of wrist camera image)
    TARGET_PX = 320  # ESP32-CAM: 640×480
    TARGET_PY = 240

    # Convergence threshold (pixels)
    CONVERGENCE_PX = 5

    def __init__(self):
        super().__init__('visual_servo_node')
        self.get_logger().info("═══ ARIA Visual Servo Node ═══")

        # PID gains
        self.declare_parameter('kp_xy', 0.001)
        self.declare_parameter('kd_xy', 0.0001)
        self.declare_parameter('max_correction_mps', 0.02)

        self.kp = self.get_parameter('kp_xy').value
        self.kd = self.get_parameter('kd_xy').value
        self.max_correction = self.get_parameter('max_correction_mps').value

        # State
        self.active = False
        self.pixel_error_x = 0.0
        self.pixel_error_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.object_detected = False

        self.bridge = CvBridge() if CV_AVAILABLE else None

        # Subscribers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=5
        )
        self.wrist_sub = self.create_subscription(
            Image, '/wrist_camera/image_raw',
            self._wrist_image_cb, qos
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10
        )

        # Publishers
        self.active_pub = self.create_publisher(
            Bool, '/visual_servo/active', 10)
        self.error_pub = self.create_publisher(
            Point, '/visual_servo/pixel_error', 10)
        self.correction_pub = self.create_publisher(
            Twist, '/visual_servo/correction', 10)

        # Control timer at 30Hz
        self.control_timer = self.create_timer(1.0 / 30.0, self._control_loop)

        self.get_logger().info("Visual servo node ready")

    def _joint_state_cb(self, msg: JointState):
        """Track joint states for distance estimation."""
        pass  # Joint tracking handled by grasp executor

    def _wrist_image_cb(self, msg: Image):
        """Process wrist camera image for object detection."""
        if not self.active or not CV_AVAILABLE or self.bridge is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        # Simple color-based detection for Stage 2
        # (Full YOLO detection on wrist camera would be Stage 3)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Detect colored objects (broad range)
        # Red range
        mask1 = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (160, 80, 80), (180, 255, 255))
        # Green range
        mask3 = cv2.inRange(hsv, (35, 80, 80), (85, 255, 255))
        # Blue range
        mask4 = cv2.inRange(hsv, (95, 80, 80), (130, 255, 255))
        # Yellow range
        mask5 = cv2.inRange(hsv, (20, 80, 80), (35, 255, 255))

        mask = mask1 | mask2 | mask3 | mask4 | mask5

        # Find largest contour
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 100:  # Minimum area threshold
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    self.pixel_error_x = float(cx - self.TARGET_PX)
                    self.pixel_error_y = float(cy - self.TARGET_PY)
                    self.object_detected = True
                    return

        self.object_detected = False

    def _control_loop(self):
        """PD control loop at 30Hz."""
        # Publish active state
        active_msg = Bool()
        active_msg.data = self.active
        self.active_pub.publish(active_msg)

        if not self.active or not self.object_detected:
            return

        # PD control
        # Error derivative (pixel/frame)
        d_error_x = self.pixel_error_x - self.prev_error_x
        d_error_y = self.pixel_error_y - self.prev_error_y

        # Correction velocities (m/s)
        vx = -(self.kp * self.pixel_error_x + self.kd * d_error_x)
        vy = -(self.kp * self.pixel_error_y + self.kd * d_error_y)

        # Clamp corrections
        vx = max(-self.max_correction, min(self.max_correction, vx))
        vy = max(-self.max_correction, min(self.max_correction, vy))

        self.prev_error_x = self.pixel_error_x
        self.prev_error_y = self.pixel_error_y

        # Publish error
        error_msg = Point()
        error_msg.x = self.pixel_error_x
        error_msg.y = self.pixel_error_y
        error_msg.z = 0.0
        self.error_pub.publish(error_msg)

        # Publish correction
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = 0.0
        self.correction_pub.publish(twist)

        # Check convergence
        error_mag = math.sqrt(
            self.pixel_error_x**2 + self.pixel_error_y**2
        )
        if error_mag < self.CONVERGENCE_PX:
            self.get_logger().info(
                f"Visual servo converged: error={error_mag:.1f}px"
            )

    def activate(self):
        """Activate visual servoing."""
        self.active = True
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.get_logger().info("Visual servoing ACTIVATED")

    def deactivate(self):
        """Deactivate visual servoing."""
        self.active = False
        self.get_logger().info("Visual servoing DEACTIVATED")


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
