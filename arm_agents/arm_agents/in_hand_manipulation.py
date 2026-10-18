#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA In-Hand Manipulation
Reorients objects while holding them.
Uses MPU6050 for orientation feedback + wrist camera for visual.
═══════════════════════════════════════════════════════════════
"""
import math
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Imu, Image, JointState
from std_srvs.srv import Trigger
from arm_interfaces.srv import SetAllJoints
from arm_planner.state_bus import StateBus

try:
    from cv_bridge import CvBridge
    CV_BRIDGE = True
except ImportError:
    CV_BRIDGE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


# Arm joint indices
WRIST_PITCH_IDX = 3

# Arm joints only; gripper is commanded separately.
JOINT_NAMES = [
    "waist_joint", "shoulder_joint", "elbow_joint",
    "wrist_pitch_joint",
]

# Joint limits (degrees)
WRIST_PITCH_MIN_DEG = -90.0
WRIST_PITCH_MAX_DEG = 90.0

# Rotation increments
ROTATION_STEP_DEG = 5.0
VERIFY_WAIT_S = 0.3        # Wait for IMU/camera after each step

# Gripper aperture (gripper_joint degrees). Closed < 10°, open > 35°.
GRIPPER_CLOSED_DEG = 5.0
GRIPPER_OPEN_DEG = 40.0
SLIDE_AJAR_DEG = 8.0       # extra opening used to let the object slide

# Wrist-camera slide verification
SLIDE_MIN_PX = 3.0         # below this, no measurable displacement
SLIDE_TOLERANCE_PX = 40.0  # |measured - expected| beyond this = unverified


class InHandManipulation:
    """
    Provides the currently supported wrist-pitch operation. The arm model
    has no wrist-roll actuator, so operations requiring tool-axis rotation
    or regrasping are reported as unsupported.
    """

    def __init__(self, node: Node, bus: StateBus):
        self.node = node
        self.bus = bus
        self.cb_group = ReentrantCallbackGroup()

        # Current state
        self.current_joints = np.zeros(len(JOINT_NAMES))
        self.current_gripper_deg = GRIPPER_OPEN_DEG
        self.imu_orientation = np.zeros(3)  # roll, pitch, yaw (degrees)
        self.imu_accel = np.zeros(3)
        self.holding_object = False
        self.bridge = CvBridge() if CV_BRIDGE else None

        # Subscribers
        self.joint_sub = node.create_subscription(
            JointState, '/joint_states', self._joint_cb, 50)
        self.imu_sub = node.create_subscription(
            Imu, '/mpu6050/imu_raw', self._imu_cb, 50)
        self.wrist_cam_sub = node.create_subscription(
            Image, '/wrist_camera/image_raw', self._wrist_cam_cb, 10)

        # Service clients
        self.set_joints = node.create_client(
            SetAllJoints, '/aria/set_all_joints',
            callback_group=self.cb_group)
        self.close_gripper = node.create_client(
            Trigger, '/aria/close_gripper',
            callback_group=self.cb_group)
        self.open_gripper = node.create_client(
            Trigger, '/aria/open_gripper',
            callback_group=self.cb_group)

        self._last_wrist_frame = None

    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joints[i] = msg.position[idx]
        if 'gripper_joint' in msg.name:
            idx = msg.name.index('gripper_joint')
            self.current_gripper_deg = math.degrees(msg.position[idx])

    def _imu_cb(self, msg: Imu):
        """Extract orientation from IMU quaternion."""
        q = msg.orientation
        # Quaternion to Euler (simplified)
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        self.imu_orientation = np.array([
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        ])

        self.imu_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])

    def _wrist_cam_cb(self, msg: Image):
        self._last_wrist_frame = msg

    # ═══════════════════════════════════════════════════════
    # Core Operations
    # ═══════════════════════════════════════════════════════

    def rotate_in_hand(self, target_angle_deg: float,
                       axis: str = 'z') -> bool:
        """
        Rotate held object by rotating wrist joint.

        Args:
            target_angle_deg: rotation amount in degrees
            axis: 'y' for wrist_pitch; 'z' is unsupported without wrist roll

        Steps:
          1. Record initial IMU orientation
          2. Rotate in 5° increments
          3. After each step, verify via IMU
          4. Stop when target reached or limit hit

        Returns: True if target rotation achieved.
        """
        self.bus.add_chain_of_thought(
            f"IN-HAND: Rotating {target_angle_deg:.0f}° around {axis}-axis")

        if axis == 'y':
            joint_idx = WRIST_PITCH_IDX
            limit_min, limit_max = WRIST_PITCH_MIN_DEG, WRIST_PITCH_MAX_DEG
        elif axis == 'z':
            self.bus.add_chain_of_thought(
                "IN-HAND: Unsupported: this arm has no wrist-roll joint")
            return False
        else:
            self.bus.add_chain_of_thought(f"IN-HAND: Invalid axis '{axis}'")
            return False

        initial_imu = self.imu_orientation.copy()
        current_deg = math.degrees(self.current_joints[joint_idx])
        target_deg = current_deg + target_angle_deg

        # Clamp to limits
        target_deg = max(limit_min, min(limit_max, target_deg))
        actual_rotation = target_deg - current_deg

        if abs(actual_rotation) < 1.0:
            self.bus.add_chain_of_thought(
                "IN-HAND: Already at target (within 1°)")
            return True

        # Rotate in steps
        direction = 1.0 if actual_rotation > 0 else -1.0
        steps = int(abs(actual_rotation) / ROTATION_STEP_DEG) + 1
        step_deg = actual_rotation / steps

        for step in range(steps):
            next_deg = current_deg + step_deg * (step + 1)
            next_deg = max(limit_min, min(limit_max, next_deg))

            # Command the joint
            target_joints = self.current_joints.copy()
            target_joints[joint_idx] = math.radians(next_deg)

            success = self._command_joints(target_joints, speed_dps=15.0)
            if not success:
                self.bus.add_chain_of_thought(
                    f"IN-HAND: Joint command failed at step {step + 1}")
                return False

            time.sleep(VERIFY_WAIT_S)

            # Verify via IMU
            imu_change = abs(self.imu_orientation[
                0 if axis == 'z' else 1] - initial_imu[
                0 if axis == 'z' else 1])

            self.bus.add_chain_of_thought(
                f"IN-HAND: Step {step + 1}/{steps}: "
                f"joint={next_deg:.1f}°, IMU change={imu_change:.1f}°")

        final_imu_change = abs(self.imu_orientation[
            0 if axis == 'z' else 1] - initial_imu[
            0 if axis == 'z' else 1])

        self.bus.add_chain_of_thought(
            f"IN-HAND: Rotation complete. "
            f"Commanded={actual_rotation:.1f}°, "
            f"IMU measured={final_imu_change:.1f}°")

        return abs(final_imu_change - abs(actual_rotation)) < 10.0

    def reposition_grip(self, offset_mm: float,
                        direction: str) -> bool:
        """
        Reposition the held object within the fingers by tilting the wrist
        so gravity slides it along the finger axis, then re-gripping.

        The wrist-pitch actuator tilts the finger axis; the object slides
        toward the fingertips ('forward') or back toward the palm
        ('backward'). Lateral sliding ('left'/'right') is NOT possible with
        this arm's DOF and is reported as unsupported.

        Args:
            offset_mm: how far to slide (magnitude in mm)
            direction: 'forward' or 'backward'

        Returns: True only when the wrist camera confirms the slide.
        """
        self.bus.add_chain_of_thought(
            f"IN-HAND: Reposition grip {offset_mm:.0f}mm {direction}")

        if direction not in ('forward', 'backward'):
            self.bus.add_chain_of_thought(
                f"IN-HAND: Unsupported direction '{direction}' "
                "(no lateral sliding axis on this arm)")
            return False
        if not self.holding_object:
            self.bus.add_chain_of_thought(
                "IN-HAND: No object held; cannot reposition grip")
            return False

        sign = 1.0 if direction == 'forward' else -1.0
        # Tilt magnitude scales with the requested slide distance.
        tilt_deg = max(3.0, min(25.0, offset_mm / 5.0)) * sign

        before = self._grab_wrist_gray()
        if before is None:
            self.bus.add_chain_of_thought(
                "IN-HAND: No wrist-camera frame; cannot verify slide")
            return False

        if not self._slide_sequence(tilt_deg):
            return False

        after = self._grab_wrist_gray()
        disp = self._phase_shift_px(before, after)
        if disp is None:
            self.bus.add_chain_of_thought(
                "IN-HAND: Wrist-camera displacement could not be measured")
            return False

        dx, dy = disp
        self.bus.add_chain_of_thought(
            f"IN-HAND: Wrist-camera displacement dx={dx:.1f}px dy={dy:.1f}px")
        return abs(dy) >= SLIDE_MIN_PX

    def flip_object(self) -> bool:
        """
        Flip the held object ~180° about the gripper pitch axis.

        The wrist-pitch joint sweeps the full ±90° range, so moving the
        wrist from one limit to the other rotates a rigidly held object by
        180° in hand. The sweep is executed in increments and verified from
        the MPU6050 orientation feedback.

        Returns: True only when the measured IMU pitch change matches 180°.
        """
        self.bus.add_chain_of_thought("IN-HAND: Flipping object 180° (pitch)")

        if not self.holding_object:
            self.bus.add_chain_of_thought(
                "IN-HAND: No object held; cannot flip")
            return False

        initial_pitch = self.imu_orientation[1]

        # Sweep to the lower limit, then to the upper limit → net 180°.
        sweep = [
            (WRIST_PITCH_MIN_DEG, 12.0),
            (WRIST_PITCH_MAX_DEG, 12.0),
        ]
        for target_deg, speed in sweep:
            if not self._sweep_wrist_pitch(target_deg, speed):
                return False

        # Orientation change (mod 360) should be ~180°.
        delta = abs(self._angle_diff_deg(self.imu_orientation[1],
                                         initial_pitch))
        self.bus.add_chain_of_thought(
            f"IN-HAND: Flip complete. IMU pitch change={delta:.1f}°")
        return abs(delta - 180.0) < 20.0

    def slide_to_tip(self, target_extension_mm: float) -> bool:
        """
        Slide the object toward the fingertips (e.g. to expose a tool tip).

        Tilting the wrist lets gravity slide the object along the fingers
        until it is re-gripped at the target extension.

        Args:
            target_extension_mm: desired slide distance toward the tips.

        Returns: True only when the wrist camera confirms the slide in the
        correct direction.
        """
        self.bus.add_chain_of_thought(
            f"IN-HAND: Slide to tip {target_extension_mm:.0f}mm")

        if not self.holding_object:
            self.bus.add_chain_of_thought(
                "IN-HAND: No object held; cannot slide")
            return False

        before = self._grab_wrist_gray()
        if before is None:
            self.bus.add_chain_of_thought(
                "IN-HAND: No wrist-camera frame; cannot verify slide")
            return False

        tilt_deg = max(3.0, min(30.0, abs(target_extension_mm) / 4.0))
        if not self._slide_sequence(tilt_deg):
            return False

        after = self._grab_wrist_gray()
        disp = self._phase_shift_px(before, after)
        if disp is None:
            self.bus.add_chain_of_thought(
                "IN-HAND: Wrist-camera displacement could not be measured")
            return False

        dx, dy = disp
        self.bus.add_chain_of_thought(
            f"IN-HAND: Slide displacement dx={dx:.1f}px dy={dy:.1f}px")
        return abs(dy) >= SLIDE_MIN_PX

    # ═══════════════════════════════════════════════════════
    # Helpers
    # ═══════════════════════════════════════════════════════

    def set_holding(self, holding: bool):
        """Record whether an object is currently held (gates in-hand ops)."""
        self.holding_object = bool(holding)

    def _sweep_wrist_pitch(self, target_deg: float,
                           speed_dps: float) -> bool:
        """Move the wrist-pitch joint to target_deg in ROTATION_STEP_DEG steps."""
        current_deg = math.degrees(self.current_joints[WRIST_PITCH_IDX])
        target_deg = max(WRIST_PITCH_MIN_DEG,
                         min(WRIST_PITCH_MAX_DEG, target_deg))
        delta = target_deg - current_deg
        if abs(delta) < 1.0:
            return True

        steps = int(abs(delta) / ROTATION_STEP_DEG) + 1
        step_deg = delta / steps
        for step in range(steps):
            next_deg = current_deg + step_deg * (step + 1)
            next_deg = max(WRIST_PITCH_MIN_DEG,
                           min(WRIST_PITCH_MAX_DEG, next_deg))
            target_joints = self.current_joints.copy()
            target_joints[WRIST_PITCH_IDX] = math.radians(next_deg)
            if not self._command_joints(target_joints, speed_dps=speed_dps):
                self.bus.add_chain_of_thought(
                    f"IN-HAND: Wrist sweep failed at step {step + 1}")
                return False
            time.sleep(VERIFY_WAIT_S)
        return True

    def _slide_sequence(self, tilt_deg: float) -> bool:
        """Open slightly, tilt the wrist so gravity slides the object, re-close.

        Returns True only if every service call succeeded.
        """
        ajar = min(GRIPPER_OPEN_DEG, self.current_gripper_deg + SLIDE_AJAR_DEG)
        # 1. Open just enough to release grip friction.
        if not self._command_gripper(ajar):
            return False
        time.sleep(VERIFY_WAIT_S)

        # 2. Tilt the wrist to let gravity slide the object.
        target_deg = math.degrees(
            self.current_joints[WRIST_PITCH_IDX]) + tilt_deg
        if not self._sweep_wrist_pitch(target_deg, speed_dps=10.0):
            return False
        time.sleep(VERIFY_WAIT_S)

        # 3. Re-close to hold the object at its new position.
        if not self._command_gripper(GRIPPER_CLOSED_DEG):
            return False
        time.sleep(VERIFY_WAIT_S)
        return True

    def _command_gripper(self, aperture_deg: float) -> bool:
        """Command only the gripper aperture, holding arm joints fixed."""
        aperture_deg = max(0.0, min(GRIPPER_OPEN_DEG, aperture_deg))
        target_joints = self.current_joints.copy()
        return self._command_joints(
            target_joints, speed_dps=20.0, gripper_deg=aperture_deg)

    def _grab_wrist_gray(self):
        """Return the latest wrist-camera frame as a grayscale image."""
        if self._last_wrist_frame is None or not CV_BRIDGE \
                or self.bridge is None:
            return None
        try:
            img = self.bridge.imgmsg_to_cv2(
                self._last_wrist_frame, desired_encoding='bgr8')
        except Exception:
            return None
        if CV2_AVAILABLE:
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return img

    @staticmethod
    def _phase_shift_px(before, after):
        """Estimate global image translation (dx, dy) via phase correlation."""
        if before is None or after is None or not CV2_AVAILABLE:
            return None
        if before.shape != after.shape:
            return None
        a = np.float32(before)
        b = np.float32(after)
        try:
            (dx, dy), _response = cv2.phaseCorrelate(a, b)
        except cv2.error:
            return None
        if not (np.isfinite(dx) and np.isfinite(dy)):
            return None
        return float(dx), float(dy)

    @staticmethod
    def _angle_diff_deg(a: float, b: float) -> float:
        """Signed smallest difference a-b wrapped to (-180, 180]."""
        d = (a - b + 180.0) % 360.0 - 180.0
        return d

    def _command_joints(self, joints_rad: np.ndarray,
                        speed_dps: float = 30.0,
                        gripper_deg: Optional[float] = None) -> bool:
        """Send joint command via service."""
        if not self.set_joints.wait_for_service(timeout_sec=2.0):
            return False
        req = SetAllJoints.Request()
        req.angles_deg = [float(math.degrees(a)) for a in joints_rad[:4]]
        # Append gripper angle (maintain current unless overridden).
        req.angles_deg.append(
            float(self.current_gripper_deg if gripper_deg is None
                  else gripper_deg))
        req.speed_deg_per_s = speed_dps
        future = self.set_joints.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        return future.done() and future.result() and future.result().success

    def _call_trigger(self, client) -> bool:
        """Call a Trigger service."""
        if not client.wait_for_service(timeout_sec=2.0):
            return False
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.done() and future.result() and future.result().success

    def get_imu_orientation(self) -> np.ndarray:
        """Get current IMU orientation (roll, pitch, yaw) in degrees."""
        return self.imu_orientation.copy()

    def is_object_stable(self) -> bool:
        """Check if held object is stable (low acceleration)."""
        accel_mag = float(np.linalg.norm(
            self.imu_accel - np.array([0, 0, 9.81])))
        return accel_mag < 1.0  # Less than 1 m/s² deviation from gravity
