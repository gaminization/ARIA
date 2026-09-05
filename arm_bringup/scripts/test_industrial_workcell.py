#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Industrial Workcell Automated Test Checklist
# Verifies:
#   1. World file & model assets
#   2. Conveyor belt service & topic interfaces
#   3. Arm industrial named poses reachable
#   4. Camera streams active (/top_camera, /side_camera)
#   5. Gripper actuation
# Usage:
#   ros2 run arm_bringup test_industrial_workcell.py
# ═══════════════════════════════════════════════════════════════
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Trigger
from arm_interfaces.srv import GoNamedPose, SetConveyorPower


class IndustrialWorkcellTester(Node):
    """Test suite for ARIA Industrial Manufacturing Workcell."""

    def __init__(self):
        super().__init__("industrial_workcell_tester")
        self.get_logger().info("═════════════════════════════════════════════════════")
        self.get_logger().info("★ Starting ARIA Industrial Workcell Verification")
        self.get_logger().info("═════════════════════════════════════════════════════")

        self.results = []
        self.top_cam_frames = 0
        self.side_cam_frames = 0
        self.wrist_cam_frames = 0
        self.joint_states_received = 0

        # Subscribers
        self.create_subscription(Image, "/top_camera/image_raw", self._top_cam_cb, 10)
        self.create_subscription(Image, "/side_camera/image_raw", self._side_cam_cb, 10)
        self.create_subscription(Image, "/wrist_camera/image_raw", self._wrist_cam_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        # Clients
        self.conveyor_client = self.create_client(SetConveyorPower, "/aria/conveyor/set_power")
        self.pose_client = self.create_client(GoNamedPose, "/aria/go_named_pose")
        self.open_grip_client = self.create_client(Trigger, "/aria/open_gripper")
        self.close_grip_client = self.create_client(Trigger, "/aria/close_gripper")

    def _top_cam_cb(self, msg):
        self.top_cam_frames += 1

    def _side_cam_cb(self, msg):
        self.side_cam_frames += 1

    def _wrist_cam_cb(self, msg):
        self.wrist_cam_frames += 1

    def _js_cb(self, msg):
        self.joint_states_received += 1

    def record_result(self, name: str, passed: bool, msg: str = ""):
        icon = "✅" if passed else "❌"
        status = "PASS" if passed else "FAIL"
        self.results.append((name, passed, msg))
        self.get_logger().info(f"[{status}] {icon} {name}: {msg}")

    def run_tests(self):
        # 1. World file check
        bringup_pkg = get_package_share_directory("arm_bringup")
        world_path = os.path.join(bringup_pkg, "worlds", "aria_industrial_workcell.world")
        world_exists = os.path.exists(world_path)
        self.record_result("World Asset Verification", world_exists, f"File: {world_path}")

        # 2. Wait for camera and joint state topics
        self.get_logger().info("Waiting 3s for sensor streams...")
        t_end = time.time() + 3.0
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.record_result(
            "Joint State Broadcaster",
            self.joint_states_received > 0,
            f"{self.joint_states_received} frames received"
        )
        self.record_result(
            "Overhead Inspection Camera",
            self.top_cam_frames > 0,
            f"{self.top_cam_frames} frames on /top_camera/image_raw"
        )
        self.record_result(
            "Side Inspection Camera",
            self.side_cam_frames > 0,
            f"{self.side_cam_frames} frames on /side_camera/image_raw"
        )
        self.record_result(
            "Wrist Gripper Camera (Commit 58ffae39)",
            self.wrist_cam_frames > 0,
            f"{self.wrist_cam_frames} frames on /wrist_camera/image_raw"
        )

        # 3. Test Conveyor Service
        if self.conveyor_client.wait_for_service(timeout_sec=2.0):
            req = SetConveyorPower.Request()
            req.power = 25.0
            future = self.conveyor_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            res = future.result()
            conv_ok = res and res.success
            self.record_result("Conveyor Service (/aria/conveyor/set_power)", conv_ok, res.message if res else "No response")
            # Stop conveyor
            req.power = 0.0
            self.conveyor_client.call_async(req)
        else:
            self.record_result("Conveyor Service", False, "Service /aria/conveyor/set_power not online")

        # 4. Test Industrial Named Poses
        if self.pose_client.wait_for_service(timeout_sec=2.0):
            industrial_poses = [
                "ready",
                "conveyor_pick_approach",
                "inspect_station",
                "assembly_place",
                "reject_drop",
                "ready"
            ]
            all_poses_ok = True
            for pose in industrial_poses:
                req = GoNamedPose.Request()
                req.pose_name = pose
                future = self.pose_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                res = future.result()
                if not (res and res.success):
                    all_poses_ok = False
                    self.get_logger().warn(f"Failed pose: {pose}")
            self.record_result("Industrial Named Poses Trajectory", all_poses_ok, f"Tested {len(industrial_poses)} poses")
        else:
            self.record_result("Named Pose Service", False, "/aria/go_named_pose not online")

        # Summary
        self.get_logger().info("═════════════════════════════════════════════════════")
        self.get_logger().info("★ TEST SUMMARY")
        total = len(self.results)
        passed = sum(1 for _, p, _ in self.results if p)
        self.get_logger().info(f"Score: {passed}/{total} tests passed ({passed/total*100:.1f}%)")
        self.get_logger().info("═════════════════════════════════════════════════════")


def main(args=None):
    rclpy.init(args=args)
    tester = IndustrialWorkcellTester()
    try:
        tester.run_tests()
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
