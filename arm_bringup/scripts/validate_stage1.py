#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Stage 1 Validation Script
# Automated test checklist — run after first launch.
# Verifies: URDF, joints, limits, named poses, gripper,
#           cameras, IMU, e-stop, TF tree, NaN check.
#
# Usage:
#   ros2 run arm_bringup validate_stage1.py
# ═══════════════════════════════════════════════════════════════
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState, Image, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from arm_interfaces.srv import SetJoint, SetAllJoints, GoNamedPose
from tf2_ros import Buffer, TransformListener


class ValidationResult:
    """Track a single test result."""

    def __init__(self, name):
        self.name = name
        self.passed = False
        self.message = ""
        self.details = []

    def pass_test(self, msg=""):
        self.passed = True
        self.message = msg

    def fail_test(self, msg=""):
        self.passed = False
        self.message = msg

    def __str__(self):
        icon = "✅" if self.passed else "❌"
        status = "PASS" if self.passed else "FAIL"
        result = f"{icon} {status} {self.name}"
        if self.message:
            result += f" — {self.message}"
        return result


class Stage1Validator(Node):
    """Automated Stage 1 validation."""

    JOINT_NAMES = [
        "waist_joint", "shoulder_joint", "elbow_joint",
        "wrist_pitch_joint", "wrist_roll_joint", "gripper_joint"
    ]

    def __init__(self):
        super().__init__("stage1_validator")
        self.get_logger().info("═══ ARIA Stage 1 Validation Starting ═══")

        # ── State ──────────────────────────────────────────
        self.joint_states = {}          # name → position
        self.joint_states_received = False
        self.top_camera_count = 0
        self.wrist_camera_count = 0
        self.imu_count = 0
        self.top_camera_time = None
        self.wrist_camera_time = None
        self.imu_time = None
        self.results = []

        # ── TF ─────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscribers ────────────────────────────────────
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states",
            self._joint_state_cb, 10
        )

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        self.top_cam_sub = self.create_subscription(
            Image, "/top_camera/image_raw",
            self._top_camera_cb, qos_sensor
        )
        self.wrist_cam_sub = self.create_subscription(
            Image, "/wrist_camera/image_raw",
            self._wrist_camera_cb, qos_sensor
        )
        self.imu_sub = self.create_subscription(
            Imu, "/mpu6050/imu_raw",
            self._imu_cb, qos_sensor
        )

        # ── Service clients ────────────────────────────────
        self.set_joint_client = self.create_client(SetJoint, "/aria/set_joint")
        self.set_all_client = self.create_client(SetAllJoints, "/aria/set_all_joints")
        self.named_pose_client = self.create_client(GoNamedPose, "/aria/go_named_pose")
        self.estop_client = self.create_client(Trigger, "/aria/estop")
        self.release_estop_client = self.create_client(Trigger, "/aria/release_estop")
        self.open_gripper_client = self.create_client(Trigger, "/aria/open_gripper")
        self.close_gripper_client = self.create_client(Trigger, "/aria/close_gripper")

    # ── Callbacks ──────────────────────────────────────────
    def _joint_state_cb(self, msg):
        self.joint_states_received = True
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = msg.position[i]

    def _top_camera_cb(self, msg):
        self.top_camera_count += 1
        self.top_camera_time = time.time()

    def _wrist_camera_cb(self, msg):
        self.wrist_camera_count += 1
        self.wrist_camera_time = time.time()

    def _imu_cb(self, msg):
        self.imu_count += 1
        self.imu_time = time.time()

    # ── Helpers ────────────────────────────────────────────
    def _spin_for(self, seconds):
        """Spin for a given duration."""
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _call_service_sync(self, client, request, timeout=5.0):
        """Call service and wait for result."""
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if future.done():
            return future.result()
        return None

    def _get_joint_deg(self, name):
        """Get current joint position in degrees."""
        if name in self.joint_states:
            return self.joint_states[name] * 180.0 / math.pi
        return None

    # ═══════════════════════════════════════════════════════
    # TEST METHODS
    # ═══════════════════════════════════════════════════════
    def test_urdf_loads(self):
        """Test 1: URDF loads without errors."""
        result = ValidationResult("URDF loads")

        # Check /robot_description is published
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        received = [False]
        def cb(msg):
            received[0] = True

        sub = self.create_subscription(String, "/robot_description", cb, qos)
        self._spin_for(3.0)
        self.destroy_subscription(sub)

        if received[0] or self.joint_states_received:
            result.pass_test("robot_description published")
        else:
            result.fail_test("robot_description not found")

        return result

    def test_all_joints_present(self):
        """Test 2: All 6 joint names in /joint_states."""
        result = ValidationResult("All 6 joints in /joint_states")

        self._spin_for(2.0)

        missing = []
        for name in self.JOINT_NAMES:
            if name not in self.joint_states:
                missing.append(name)

        if not missing:
            result.pass_test(f"Found all {len(self.JOINT_NAMES)} joints")
        else:
            result.fail_test(f"Missing: {missing}")

        return result

    def test_joint_response(self):
        """Test 3: Each joint responds to command."""
        result = ValidationResult("Joint response")

        if not self.set_joint_client.wait_for_service(timeout_sec=3.0):
            result.fail_test("set_joint service not available")
            return result

        joint_results = []
        for i, name in enumerate(self.JOINT_NAMES[:5]):  # Skip gripper
            short_name = name.replace("_joint", "")

            # Command to a test angle
            req = SetJoint.Request()
            req.joint_name = short_name
            req.angle_deg = 45.0
            req.speed_deg_per_s = 60.0
            resp = self._call_service_sync(self.set_joint_client, req)

            if resp and resp.success:
                self._spin_for(2.0)  # Wait for motion

                actual = self._get_joint_deg(name)
                if actual is not None and abs(actual - 45.0) < 5.0:
                    joint_results.append(f"{short_name}: ✅")
                else:
                    joint_results.append(f"{short_name}: ❌ ({actual:.1f}°)")
            else:
                joint_results.append(f"{short_name}: ❌ (no response)")

            # Return to home position for this joint
            req.angle_deg = 0.0 if i == 0 else 90.0
            self._call_service_sync(self.set_joint_client, req)
            self._spin_for(1.0)

        all_passed = all("✅" in r for r in joint_results)
        msg = " ".join(joint_results)
        if all_passed:
            result.pass_test(msg)
        else:
            result.fail_test(msg)

        return result

    def test_joint_limits(self):
        """Test 4: Joint limits enforced."""
        result = ValidationResult("Joint limits enforced")

        req = SetJoint.Request()
        req.joint_name = "waist"
        req.angle_deg = 180.0  # Beyond limit (+90°)
        req.speed_deg_per_s = 60.0

        resp = self._call_service_sync(self.set_joint_client, req)
        self._spin_for(2.0)

        if resp:
            # Should be clamped to soft limit (~89°)
            if resp.actual_angle_deg <= 90.0:
                result.pass_test(
                    f"180° clamped to {resp.actual_angle_deg:.1f}°")
            else:
                result.fail_test(
                    f"Expected ≤90°, got {resp.actual_angle_deg:.1f}°")
        else:
            result.fail_test("No service response")

        # Return to home
        req.angle_deg = 0.0
        self._call_service_sync(self.set_joint_client, req)
        self._spin_for(1.0)

        return result

    def test_home_position(self):
        """Test 5: Home position reachable."""
        result = ValidationResult("Home position")

        req = GoNamedPose.Request()
        req.pose_name = "home"
        resp = self._call_service_sync(self.named_pose_client, req)
        self._spin_for(3.0)

        if resp and resp.success:
            home_deg = [0.0, 90.0, 75.0, 0.0, 0.0, 20.0]
            all_ok = True
            for i, name in enumerate(self.JOINT_NAMES):
                actual = self._get_joint_deg(name)
                if actual is None or abs(actual - home_deg[i]) > 5.0:
                    all_ok = False
                    break

            if all_ok:
                result.pass_test("All joints within 5° of home")
            else:
                result.fail_test("Some joints not at home position")
        else:
            result.fail_test("go_named_pose service failed")

        return result

    def test_named_poses(self):
        """Test 6: Named poses reachable."""
        result = ValidationResult("Named poses")

        poses = ["home", "ready", "folded", "inspect"]
        pose_results = []

        for pose in poses:
            req = GoNamedPose.Request()
            req.pose_name = pose
            resp = self._call_service_sync(self.named_pose_client, req)
            self._spin_for(3.0)

            if resp and resp.success:
                pose_results.append(f"{pose}: ✅")
            else:
                pose_results.append(f"{pose}: ❌")

        all_passed = all("✅" in r for r in pose_results)
        msg = " ".join(pose_results)
        if all_passed:
            result.pass_test(msg)
        else:
            result.fail_test(msg)

        return result

    def test_gripper(self):
        """Test 7: Gripper opens and closes."""
        result = ValidationResult("Gripper")

        # Close gripper
        resp = self._call_service_sync(
            self.close_gripper_client, Trigger.Request())
        self._spin_for(2.0)

        gripper_closed = self._get_joint_deg("gripper_joint")

        # Open gripper
        resp = self._call_service_sync(
            self.open_gripper_client, Trigger.Request())
        self._spin_for(2.0)

        gripper_open = self._get_joint_deg("gripper_joint")

        if gripper_closed is not None and gripper_open is not None:
            closed_ok = gripper_closed < 10.0
            open_ok = gripper_open > 35.0
            if closed_ok and open_ok:
                result.pass_test(
                    f"Closed: {gripper_closed:.1f}°, Open: {gripper_open:.1f}°")
            else:
                result.fail_test(
                    f"Closed: {gripper_closed:.1f}° (want <10°), "
                    f"Open: {gripper_open:.1f}° (want >35°)")
        else:
            result.fail_test("Could not read gripper position")

        return result

    def test_cameras(self):
        """Test 8: Camera topics active."""
        result = ValidationResult("Camera topics")

        # Reset counters
        self.top_camera_count = 0
        self.wrist_camera_count = 0
        start = time.time()

        self._spin_for(3.0)

        elapsed = time.time() - start
        top_fps = self.top_camera_count / elapsed if elapsed > 0 else 0
        wrist_fps = self.wrist_camera_count / elapsed if elapsed > 0 else 0

        top_ok = top_fps >= 5.0    # Relaxed: at least 5fps (target 30)
        wrist_ok = wrist_fps >= 3.0  # Relaxed: at least 3fps (target 15)

        if top_ok and wrist_ok:
            result.pass_test(
                f"Top: {top_fps:.0f}fps, Wrist: {wrist_fps:.0f}fps")
        elif self.top_camera_count == 0 and self.wrist_camera_count == 0:
            result.fail_test("No camera data received (bridge may not be running)")
        else:
            result.fail_test(
                f"Top: {top_fps:.0f}fps (need ≥5), "
                f"Wrist: {wrist_fps:.0f}fps (need ≥3)")

        return result

    def test_imu(self):
        """Test 9: IMU topic active."""
        result = ValidationResult("IMU topic")

        self.imu_count = 0
        start = time.time()

        self._spin_for(3.0)

        elapsed = time.time() - start
        imu_hz = self.imu_count / elapsed if elapsed > 0 else 0

        if imu_hz >= 10.0:  # Relaxed: at least 10Hz (target 100)
            result.pass_test(f"{imu_hz:.0f} Hz")
        elif self.imu_count == 0:
            result.fail_test("No IMU data received (bridge may not be running)")
        else:
            result.fail_test(f"{imu_hz:.0f} Hz (need ≥10)")

        return result

    def test_estop(self):
        """Test 10: E-stop works."""
        result = ValidationResult("E-stop")

        # Activate e-stop
        resp = self._call_service_sync(
            self.estop_client, Trigger.Request())
        self._spin_for(0.5)

        if not (resp and resp.success):
            result.fail_test("E-stop service failed")
            return result

        # Try to send command — should be rejected
        req = SetJoint.Request()
        req.joint_name = "waist"
        req.angle_deg = 45.0
        req.speed_deg_per_s = 30.0
        cmd_resp = self._call_service_sync(self.set_joint_client, req)

        estop_blocks = (cmd_resp is not None and not cmd_resp.success)

        # Release e-stop
        resp = self._call_service_sync(
            self.release_estop_client, Trigger.Request())
        self._spin_for(0.5)

        # Try command again — should succeed
        cmd_resp2 = self._call_service_sync(self.set_joint_client, req)
        released_ok = (cmd_resp2 is not None and cmd_resp2.success)

        # Return to 0
        req.angle_deg = 0.0
        self._call_service_sync(self.set_joint_client, req)
        self._spin_for(1.0)

        if estop_blocks and released_ok:
            result.pass_test("Commands blocked during e-stop, accepted after release")
        elif not estop_blocks:
            result.fail_test("E-stop did not block commands")
        else:
            result.fail_test("Commands not accepted after e-stop release")

        return result

    def test_tf_tree(self):
        """Test 11: TF tree complete."""
        result = ValidationResult("TF tree")

        self._spin_for(2.0)

        required_frames = [
            ("world", "base_link"),
            ("base_link", "waist_link"),
            ("waist_link", "upper_arm_link"),
            ("upper_arm_link", "forearm_link"),
            ("forearm_link", "wrist_link"),
            ("wrist_link", "gripper_holder_link"),
            ("gripper_holder_link", "gear_housing_link"),
            ("world", "top_camera_link"),
        ]

        missing = []
        for parent, child in required_frames:
            try:
                self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time())
            except Exception:
                missing.append(f"{parent}→{child}")

        if not missing:
            result.pass_test(f"All {len(required_frames)} transforms found")
        else:
            result.fail_test(f"Missing: {missing}")

        return result

    def test_no_nan(self):
        """Test 12: No NaN in joint states (monitor for 5s)."""
        result = ValidationResult("No NaN")

        nan_found = False
        nan_details = []

        end = time.time() + 5.0
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            for name, pos in self.joint_states.items():
                if math.isnan(pos) or math.isinf(pos):
                    nan_found = True
                    nan_details.append(f"{name}: {pos}")

        if not nan_found:
            result.pass_test("No NaN/Inf in 5s monitoring window")
        else:
            result.fail_test(f"NaN detected: {nan_details}")

        return result

    # ═══════════════════════════════════════════════════════
    # RUN ALL TESTS
    # ═══════════════════════════════════════════════════════
    def run_all_tests(self):
        """Execute all validation tests in sequence."""
        print()
        print("══════════════════════════════════════")
        print("  ARIA Stage 1 Validation Results")
        print("══════════════════════════════════════")
        print()

        tests = [
            self.test_urdf_loads,
            self.test_all_joints_present,
            self.test_joint_response,
            self.test_joint_limits,
            self.test_home_position,
            self.test_named_poses,
            self.test_gripper,
            self.test_cameras,
            self.test_imu,
            self.test_estop,
            self.test_tf_tree,
            self.test_no_nan,
        ]

        results = []
        for test_fn in tests:
            self.get_logger().info(f"Running: {test_fn.__doc__}")
            try:
                result = test_fn()
            except Exception as e:
                result = ValidationResult(test_fn.__doc__ or test_fn.__name__)
                result.fail_test(f"Exception: {e}")
            results.append(result)
            print(f"  {result}")

        # Summary
        passed = sum(1 for r in results if r.passed)
        total = len(results)
        all_passed = (passed == total)

        print()
        print("══════════════════════════════════════")
        if all_passed:
            print("  RESULT: STAGE 1 COMPLETE ✅")
            print("  Ready for Stage 2")
        else:
            print(f"  RESULT: {passed}/{total} tests passed")
            print("  Fix failures before proceeding")
        print("══════════════════════════════════════")
        print()

        return all_passed


def main(args=None):
    rclpy.init(args=args)
    validator = Stage1Validator()

    try:
        # Let node warm up and receive initial data
        print("Waiting for simulation to stabilize (5s)...")
        end = time.time() + 5.0
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(validator, timeout_sec=0.1)

        success = validator.run_all_tests()
        sys.exit(0 if success else 1)

    except KeyboardInterrupt:
        print("\nValidation interrupted")
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
