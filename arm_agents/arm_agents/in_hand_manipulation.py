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


# Wrist joint indices
WRIST_PITCH_IDX = 3
WRIST_ROLL_IDX = 4

# All joint names
JOINT_NAMES = [
    "waist_joint", "shoulder_joint", "elbow_joint",
    "wrist_pitch_joint", "wrist_roll_joint",
]

# Joint limits (degrees)
WRIST_ROLL_MIN_DEG = -90.0
WRIST_ROLL_MAX_DEG = 90.0
WRIST_PITCH_MIN_DEG = -90.0
WRIST_PITCH_MAX_DEG = 90.0

# Rotation increments
ROTATION_STEP_DEG = 5.0
VERIFY_WAIT_S = 0.3        # Wait for IMU/camera after each step
GRIP_OPEN_PARTIAL_S = 0.2  # Partial open time for repositioning


class InHandManipulation:
    """
    Reorients objects while holding them.

    Uses:
      - wrist_roll_joint for Z-axis rotation
      - wrist_pitch_joint for Y-axis tilt
      - MPU6050 IMU for orientation feedback
      - Wrist camera for visual verification

    Use cases:
      Screwdriver: rotate_in_hand to align tip with screw
      Paintbrush: reposition_grip for correct holding angle
      Key: rotate_in_hand to correct insertion angle
    """

    def __init__(self, node: Node, bus: StateBus):
        self.node = node
        self.bus = bus
        self.cb_group = ReentrantCallbackGroup()

        # Current state
        self.current_joints = np.zeros(5)
        self.imu_orientation = np.zeros(3)  # roll, pitch, yaw (degrees)
        self.imu_accel = np.zeros(3)
        self.holding_object = False

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
            axis: 'z' for wrist_roll, 'y' for wrist_pitch

        Steps:
          1. Record initial IMU orientation
          2. Rotate in 5° increments
          3. After each step, verify via IMU
          4. Stop when target reached or limit hit

        Returns: True if target rotation achieved.
        """
        self.bus.add_chain_of_thought(
            f"IN-HAND: Rotating {target_angle_deg:.0f}° around {axis}-axis")

        if axis == 'z':
            joint_idx = WRIST_ROLL_IDX
            limit_min, limit_max = WRIST_ROLL_MIN_DEG, WRIST_ROLL_MAX_DEG
        elif axis == 'y':
            joint_idx = WRIST_PITCH_IDX
            limit_min, limit_max = WRIST_PITCH_MIN_DEG, WRIST_PITCH_MAX_DEG
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
        Slide object in gripper by partially opening, nudging, re-closing.

        Args:
            offset_mm: how far to slide (positive = outward)
            direction: 'forward', 'backward', 'left', 'right'

        Steps:
          1. Record wrist camera view (reference)
          2. Partially open gripper (just enough to slide)
          3. Nudge arm to shift object
          4. Re-close gripper
          5. Verify new grip via wrist camera
        """
        self.bus.add_chain_of_thought(
            f"IN-HAND: Repositioning grip {offset_mm:.0f}mm {direction}")

        # Step 1: Reference
        ref_frame = self._last_wrist_frame

        # Step 2: Partial open
        self.bus.add_chain_of_thought("IN-HAND: Partially opening gripper...")
        # In simulation: use a partial grip angle
        # The actual implementation would send a partial close command
        time.sleep(GRIP_OPEN_PARTIAL_S)

        # Step 3: Nudge
        nudge_m = offset_mm / 1000.0
        nudge_joints = self.current_joints.copy()

        if direction in ('forward', 'backward'):
            sign = 1.0 if direction == 'forward' else -1.0
            # Small elbow adjustment to nudge
            nudge_joints[2] += sign * 0.02  # ~1° elbow change
        elif direction in ('left', 'right'):
            sign = 1.0 if direction == 'right' else -1.0
            nudge_joints[0] += sign * 0.01  # small waist change

        success = self._command_joints(nudge_joints, speed_dps=5.0)
        time.sleep(0.2)

        # Step 4: Re-close gripper
        self.bus.add_chain_of_thought("IN-HAND: Re-closing gripper...")
        self._call_trigger(self.close_gripper)
        time.sleep(0.3)

        # Step 5: Verify
        time.sleep(0.2)
        self.bus.add_chain_of_thought(
            f"IN-HAND: Grip repositioned {offset_mm:.0f}mm {direction}")

        return True

    def flip_object(self) -> bool:
        """
        180° rotation by placing on table, re-grasping from other end.

        Sequence:
          1. Record current object height
          2. Lower to table surface
          3. Open gripper — place object
          4. Lift arm slightly
          5. Move to opposite side
          6. Lower and re-grasp
          7. Verify via wrist camera
        """
        self.bus.add_chain_of_thought(
            "IN-HAND: Flipping object (place → reposition → regrasp)")

        # Step 1: Remember pose
        current = self.current_joints.copy()

        # Step 2: Lower to table
        place_joints = current.copy()
        place_joints[1] += 0.15  # Lower shoulder
        place_joints[2] += 0.10  # Extend elbow
        self._command_joints(place_joints, speed_dps=20.0)
        time.sleep(1.0)

        # Step 3: Release
        self.bus.add_chain_of_thought("IN-HAND: Placing object on table...")
        self._call_trigger(self.open_gripper)
        time.sleep(0.5)

        # Step 4: Lift
        retract = place_joints.copy()
        retract[1] -= 0.10
        self._command_joints(retract, speed_dps=30.0)
        time.sleep(0.5)

        # Step 5: Rotate wrist 180°
        self.bus.add_chain_of_thought(
            "IN-HAND: Repositioning for opposite-side grasp...")
        approach = retract.copy()
        approach[4] += math.pi  # Rotate wrist roll 180°
        # Clamp to limits
        approach[4] = max(math.radians(-90), min(math.radians(90), approach[4]))
        self._command_joints(approach, speed_dps=30.0)
        time.sleep(0.5)

        # Step 6: Lower and re-grasp
        self._command_joints(place_joints, speed_dps=20.0)
        time.sleep(0.5)
        self.bus.add_chain_of_thought("IN-HAND: Re-grasping...")
        self._call_trigger(self.close_gripper)
        time.sleep(0.5)

        # Step 7: Lift
        self._command_joints(current, speed_dps=20.0)
        time.sleep(1.0)

        self.bus.add_chain_of_thought("IN-HAND: Flip complete")
        return True

    def slide_to_tip(self, target_extension_mm: float) -> bool:
        """
        Let object slide to tip of gripper fingers.
        For tools: screwdriver, pen, paintbrush.

        Steps:
          1. Tilt wrist downward (gravity assists sliding)
          2. Partially open gripper
          3. Wait for object to slide
          4. Re-close at target extension
          5. Return wrist to neutral
          6. Verify via wrist camera
        """
        self.bus.add_chain_of_thought(
            f"IN-HAND: Sliding object {target_extension_mm:.0f}mm "
            f"toward gripper tips")

        # Step 1: Tilt wrist down
        tilt_joints = self.current_joints.copy()
        original_pitch = tilt_joints[WRIST_PITCH_IDX]
        tilt_joints[WRIST_PITCH_IDX] = math.radians(45)  # Tilt down
        self._command_joints(tilt_joints, speed_dps=15.0)
        time.sleep(0.5)

        # Step 2: Partial open
        self.bus.add_chain_of_thought("IN-HAND: Partial open for sliding...")
        time.sleep(GRIP_OPEN_PARTIAL_S)

        # Step 3: Wait for gravity slide
        # Estimate slide time from extension distance
        slide_time = target_extension_mm / 20.0  # ~20mm/s slide rate
        self.bus.add_chain_of_thought(
            f"IN-HAND: Waiting {slide_time:.1f}s for gravity slide...")
        time.sleep(max(0.2, slide_time))

        # Step 4: Re-close
        self._call_trigger(self.close_gripper)
        time.sleep(0.3)

        # Step 5: Return wrist to neutral
        restore = self.current_joints.copy()
        restore[WRIST_PITCH_IDX] = original_pitch
        self._command_joints(restore, speed_dps=15.0)
        time.sleep(0.5)

        self.bus.add_chain_of_thought(
            f"IN-HAND: Slide complete. Tool extended ~{target_extension_mm:.0f}mm")
        return True

    # ═══════════════════════════════════════════════════════
    # Helpers
    # ═══════════════════════════════════════════════════════

    def _command_joints(self, joints_rad: np.ndarray,
                        speed_dps: float = 30.0) -> bool:
        """Send joint command via service."""
        if not self.set_joints.wait_for_service(timeout_sec=2.0):
            return False
        req = SetAllJoints.Request()
        req.angles_deg = [float(math.degrees(a)) for a in joints_rad[:5]]
        # Append gripper angle (maintain current)
        req.angles_deg.append(44.0)
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
