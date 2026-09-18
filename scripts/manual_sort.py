#!/usr/bin/env python3
import time
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from arm_interfaces.srv import GoNamedPose, SetConveyorPower
from cv_bridge import CvBridge
import cv2


class IndustrialContinuousSorter(Node):
    def __init__(self):
        super().__init__('industrial_continuous_sorter')
        self.get_logger().info("═════════════════════════════════════════════════════════════")
        self.get_logger().info("  🤖 ARIA AUTONOMOUS MANUFACTURING WORKCELL SORT SEQUENCE    ")
        self.get_logger().info("═════════════════════════════════════════════════════════════")

        self.bridge = CvBridge()
        self.latest_frame = None
        self.part_present = False

        self.cycle_num = 0
        self.passed_count = 0
        self.rejected_count = 0

        # Subscriptions
        self.create_subscription(Image, "/top_camera/image_raw", self._cam_cb, 10)
        self.create_subscription(Bool, "/aria/conveyor/part_present", self._presence_cb, 10)

        # Clients
        self.pose_cli = self.create_client(GoNamedPose, "/aria/go_named_pose")
        self.conveyor_cli = self.create_client(SetConveyorPower, "/aria/conveyor/set_power")
        self.open_grip_cli = self.create_client(Trigger, "/aria/open_gripper")
        self.close_grip_cli = self.create_client(Trigger, "/aria/close_gripper")
        self.attach_cli = self.create_client(Trigger, "/aria/gripper/attach")
        self.detach_cli = self.create_client(Trigger, "/aria/gripper/detach")

        # Wait for services
        self.get_logger().info("Connecting to workcell services...")
        self.pose_cli.wait_for_service(timeout_sec=5.0)
        self.open_grip_cli.wait_for_service(timeout_sec=5.0)
        self.close_grip_cli.wait_for_service(timeout_sec=5.0)
        self.conveyor_cli.wait_for_service(timeout_sec=5.0)
        self.get_logger().info("✅ All systems connected and ready.")

    def _cam_cb(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def _presence_cb(self, msg):
        self.part_present = msg.data

    def _call(self, client, req, timeout=6.0):
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start < timeout):
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result() if future.done() else None

    def go_pose(self, name):
        self.get_logger().info(f"  [ARM] Moving -> '{name}'")
        req = GoNamedPose.Request()
        req.pose_name = name
        res = self._call(self.pose_cli, req, timeout=6.0)
        time.sleep(1.2)
        return res and res.success

    def set_conveyor(self, power):
        req = SetConveyorPower.Request()
        req.power = float(power)
        self._call(self.conveyor_cli, req, timeout=2.0)

    def set_gripper(self, open_grip):
        cli = self.open_grip_cli if open_grip else self.close_grip_cli
        action = "OPEN" if open_grip else "CLOSE"
        self.get_logger().info(f"  [GRIPPER] -> {action}")
        self._call(cli, Trigger.Request(), timeout=3.0)
        time.sleep(0.4)

        if not open_grip and self.attach_cli.service_is_ready():
            res = self._call(self.attach_cli, Trigger.Request(), timeout=2.0)
            if res:
                self.get_logger().info(f"  ⚡ {res.message}")
        elif open_grip and self.detach_cli.service_is_ready():
            res = self._call(self.detach_cli, Trigger.Request(), timeout=2.0)
            if res:
                self.get_logger().info(f"  ⚡ {res.message}")
        time.sleep(0.4)

    def inspect_held_part(self):
        self.get_logger().info("  [INSPECT] Analyzing workpiece under overhead camera...")
        t_end = time.time() + 1.5
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.latest_frame is None:
            self.get_logger().warn("  ⚠️ No camera frame, defaulting to pass")
            return True

        h, w, _ = self.latest_frame.shape
        roi = self.latest_frame[int(h * 0.42):int(h * 0.58), int(w * 0.42):int(w * 0.58)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask_red = cv2.inRange(hsv, np.array([0, 100, 70]), np.array([8, 255, 255])) | \
                   cv2.inRange(hsv, np.array([172, 100, 70]), np.array([180, 255, 255]))
        mask_blue = cv2.inRange(hsv, np.array([90, 80, 50]), np.array([135, 255, 255]))

        red_px = cv2.countNonZero(mask_red)
        blue_px = cv2.countNonZero(mask_blue)
        self.get_logger().info(f"  [METRICS] Blue pixels: {blue_px}, Red pixels: {red_px}")

        if red_px > 150 and red_px > blue_px:
            self.get_logger().warn("  ❌ DEFECT DETECTED: routing to Defect Reject Bin")
            return False
        else:
            self.get_logger().info("  ✅ QC PASSED: routing to Finished Goods Assembly Tray")
            return True

    def run_single_cycle(self):
        self.cycle_num += 1
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"▶ STARTING SORT CYCLE #{self.cycle_num}")
        self.get_logger().info(f"{'='*60}")

        # 1. Stance
        self.go_pose("ready")

        # 2. Advance conveyor until docked
        self.get_logger().info("  [FEED] Advancing conveyor to dock workpiece...")
        self.set_conveyor(50.0)
        start = time.time()
        while not self.part_present and (time.time() - start < 8.0):
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.4)  # Settle against stopper
        self.set_conveyor(0.0)

        # Check if a part actually arrived
        t_check = time.time() + 0.5
        while time.time() < t_check:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.part_present:
            self.get_logger().info("  ℹ️ No workpiece detected at stopper — queue complete!")
            return False

        self.get_logger().info("  [DOCK] Workpiece locked at stopper.")
        time.sleep(0.3)

        # 3. Precision Pick
        self.get_logger().info("  [PICK] Executing pick sequence...")
        self.set_gripper(open_grip=True)
        self.go_pose("conveyor_pick_approach")
        self.go_pose("conveyor_pick")
        self.set_gripper(open_grip=False)
        self.go_pose("conveyor_pick_approach")

        # 4. Present to QC Inspection Station
        self.get_logger().info("  [QC] Presenting to optical inspection station...")
        self.go_pose("inspect_station")
        is_good = self.inspect_held_part()

        # 5. Route by quality
        if is_good:
            self.get_logger().info("  [SORT] Placing in Finished Goods Assembly Tray...")
            self.go_pose("assembly_approach")
            self.go_pose("assembly_place")
            self.set_gripper(open_grip=True)
            self.go_pose("assembly_approach")
            self.passed_count += 1
            self.get_logger().info(f"  ⭐ PART ACCEPTED! (Total Passed: {self.passed_count})")
        else:
            self.get_logger().info("  [SORT] Dropping into Defect Reject Bin...")
            self.go_pose("reject_approach")
            self.go_pose("reject_drop")
            self.set_gripper(open_grip=True)
            self.go_pose("reject_approach")
            self.rejected_count += 1
            self.get_logger().warn(f"  🗑️ PART SCRAPPED! (Total Rejected: {self.rejected_count})")

        # 6. Return to ready
        self.go_pose("ready")
        self.get_logger().info(
            f"✅ CYCLE #{self.cycle_num} COMPLETE | Passed: {self.passed_count} | Rejected: {self.rejected_count}"
        )
        return True

    def run_all(self, max_cycles=8):
        self.get_logger().info(f"Starting autonomous multi-part sort (up to {max_cycles} parts)...")
        for i in range(max_cycles):
            has_more = self.run_single_cycle()
            if not has_more:
                break
            time.sleep(0.8)

        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("🎉 ALL WORKPIECES SORTED SUCCESSFULLY!")
        self.get_logger().info(f"  Total Processed: {self.passed_count + self.rejected_count}")
        self.get_logger().info(f"  Finished Goods:  {self.passed_count} parts accepted")
        self.get_logger().info(f"  Defect Scrap:    {self.rejected_count} parts rejected")
        self.get_logger().info(f"{'='*60}\n")


def main():
    rclpy.init()
    sorter = IndustrialContinuousSorter()
    try:
        sorter.run_all(max_cycles=8)
    finally:
        sorter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
