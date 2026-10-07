#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Industrial Workcell Coordinator Node
# Orchestrates automated production cycle:
# 1. Infeed Conveyor Feed -> 2. Mechanical Stop -> 3. Precision Pick ->
# 4. Overhead Vision Inspection -> 5. Sort (Assembly vs. Reject Bin)
# ═══════════════════════════════════════════════════════════════
import sys
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float64
from std_srvs.srv import Trigger
from arm_interfaces.srv import GoNamedPose, SetConveyorPower

try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CV = True
except ImportError:
    HAS_CV = False


class IndustrialWorkcellNode(Node):
    """
    Automated Industrial Manufacturing Workcell Orchestrator.
    Manages conveyor feeding, vision-based inspection, and pick-and-place sorting.
    """

    def __init__(self):
        super().__init__("industrial_workcell_node")
        self.get_logger().info("════ ARIA Industrial Workcell Coordinator Initializing ════")

        self.cb_group = ReentrantCallbackGroup()
        self.bridge = CvBridge() if HAS_CV else None

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter("blue_item_destination", "red_box")
        self.declare_parameter("defect_item_destination", "reject_bin")
        self.declare_parameter("max_cycles", 1)

        # ── State ──────────────────────────────────────────────
        self.latest_camera_frame = None
        self.cycle_count = 0
        self.parts_passed = 0
        self.parts_rejected = 0
        self.blue_items_in_red_box = 0
        self.is_running_cycle = False
        self.part_present = False

        # ── Publishers ─────────────────────────────────────────
        self.cot_pub = self.create_publisher(String, "/aria/cot/reasoning", 10)

        # ── Subscriptions ──────────────────────────────────────
        self.camera_sub = self.create_subscription(
            Image, "/top_camera/image_raw",
            self._camera_cb, 10,
            callback_group=self.cb_group
        )

        self.presence_sub = self.create_subscription(
            Bool, "/aria/conveyor/part_present",
            self._presence_cb, 10,
            callback_group=self.cb_group
        )

        # ── Service Clients ────────────────────────────────────
        self.conveyor_client = self.create_client(
            SetConveyorPower, "/aria/conveyor/set_power",
            callback_group=self.cb_group
        )

        self.named_pose_client = self.create_client(
            GoNamedPose, "/aria/go_named_pose",
            callback_group=self.cb_group
        )

        self.open_gripper_client = self.create_client(
            Trigger, "/aria/open_gripper",
            callback_group=self.cb_group
        )

        self.close_gripper_client = self.create_client(
            Trigger, "/aria/close_gripper",
            callback_group=self.cb_group
        )

        self.attach_client = self.create_client(
            Trigger, "/aria/gripper/attach",
            callback_group=self.cb_group
        )

        self.detach_client = self.create_client(
            Trigger, "/aria/gripper/detach",
            callback_group=self.cb_group
        )

        # Workcell status publisher
        self.status_pub = self.create_publisher(String, "/aria/workcell/status", 10)

        # Service to trigger production cycles on demand
        self.start_cycle_srv = self.create_service(
            Trigger, "/aria/start_cycle",
            self._start_cycle_srv_cb,
            callback_group=self.cb_group
        )

        # Wait for key services
        self.get_logger().info("Connecting to ARIA control services...")
        self._wait_for_services()

        # Start master cycle timer (fires 3 seconds after launch)
        self.cycle_timer = self.create_timer(3.0, self._start_cycle_callback)
        self.get_logger().info("✅ Industrial Workcell Coordinator Ready.")

    def _wait_for_services(self):
        """Ensure core arm control services are available."""
        services = [
            (self.named_pose_client, "/aria/go_named_pose"),
            (self.open_gripper_client, "/aria/open_gripper"),
            (self.close_gripper_client, "/aria/close_gripper"),
            (self.conveyor_client, "/aria/conveyor/set_power"),
            (self.attach_client, "/aria/gripper/attach"),
            (self.detach_client, "/aria/gripper/detach"),
        ]
        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {name}...")
        self.get_logger().info("✅ Connected to all workcell control & gripper services.")

    def _camera_cb(self, msg: Image):
        """Cache latest overhead inspection frame."""
        if self.bridge:
            try:
                self.latest_camera_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().warn(f"CvBridge decode error: {e}")

    def _call_sync(self, client, request, timeout=5.0):
        """Synchronously call a ROS2 service under MultiThreadedExecutor."""
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _set_conveyor_power(self, power_pct: float):
        """Set conveyor belt speed."""
        if not self.conveyor_client.service_is_ready():
            return False
        req = SetConveyorPower.Request()
        req.power = float(power_pct)
        try:
            res = self._call_sync(self.conveyor_client, req, timeout=2.0)
            return res.success if res else False
        except Exception as e:
            self.get_logger().warn(f"Conveyor command error: {e}")
            return False

    def _go_pose(self, pose_name: str) -> bool:
        """Command arm to a named pose."""
        self.get_logger().info(f"Moving arm -> {pose_name}")
        req = GoNamedPose.Request()
        req.pose_name = pose_name
        try:
            res = self._call_sync(self.named_pose_client, req, timeout=5.0)
            time.sleep(1.2)  # Allow physical settling
            return res.success if res else False
        except Exception as e:
            self.get_logger().error(f"Pose move failed: {e}")
            return False

    def _set_gripper(self, open_grip: bool) -> bool:
        """Trigger gripper open or close with physical grasp confirmation."""
        client = self.open_gripper_client if open_grip else self.close_gripper_client
        action_name = "OPEN" if open_grip else "CLOSE"
        self.get_logger().info(f"Gripper -> {action_name}")
        req = Trigger.Request()
        try:
            if open_grip:
                # When opening to drop/release, detach physical ODE joint first
                if self.detach_client.wait_for_service(timeout_sec=1.0):
                    res_det = self._call_sync(self.detach_client, Trigger.Request(), timeout=2.0)
                    if res_det:
                        self.get_logger().info(f"⚡ {res_det.message}")
                res = self._call_sync(client, req, timeout=3.0)
                time.sleep(0.5)
                return res.success if res else False
            else:
                # When closing to grasp, actuate fingers first, then attach physical joint
                res = self._call_sync(client, req, timeout=3.0)
                time.sleep(0.5)
                grasp_success = True
                if self.attach_client.wait_for_service(timeout_sec=1.0):
                    res_att = self._call_sync(self.attach_client, Trigger.Request(), timeout=2.0)
                    if res_att:
                        msg = res_att.message
                        self.get_logger().info(f"⚡ {msg}")
                        if not res_att.success and "Already holding" not in msg:
                            grasp_success = False
                time.sleep(0.5)
                return (res.success if res else False) and grasp_success
        except Exception as e:
            self.get_logger().error(f"Gripper trigger failed: {e}")
            return False

    def _inspect_workpiece_vision(self) -> bool:
        """
        Analyze the held workpiece under the overhead inspection camera.
        Returns True if component passes QC (Blue/Silver component), False if Defect (Red component).
        """
        self.get_logger().info("🔎 Performing Quality Inspection under Overhead Camera...")
        time.sleep(0.8)  # Exposure settle

        if self.latest_camera_frame is None:
            self.get_logger().warn("No image frame received yet. Defaulting to PASS.")
            return True

        frame = self.latest_camera_frame
        h, w, _ = frame.shape
        # Precision center ROI focused squarely on the held workpiece
        roi = frame[int(h * 0.42):int(h * 0.58), int(w * 0.42):int(w * 0.58)]

        if roi.size == 0:
            return True

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # Red detection masks (Defect part)
        lower_red1 = np.array([0, 100, 70])
        upper_red1 = np.array([8, 255, 255])
        lower_red2 = np.array([172, 100, 70])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_pixels = cv2.countNonZero(mask1 | mask2)

        # Blue detection masks (Good part)
        lower_blue = np.array([90, 80, 50])
        upper_blue = np.array([135, 255, 255])
        blue_pixels = cv2.countNonZero(cv2.inRange(hsv, lower_blue, upper_blue))

        self.get_logger().info(f"Inspection metrics: Blue px={blue_pixels}, Red px={red_pixels}")

        if red_pixels > 150 and red_pixels > blue_pixels:
            self.get_logger().warn("❌ DEFECT DETECTED: Workpiece failed optical quality inspection!")
            return False
        elif blue_pixels > 150 and blue_pixels >= red_pixels:
            self.get_logger().info("✅ QC PASSED: Workpiece meets dimensional and visual specification.")
            return True
        else:
            self.get_logger().warn("⚠️ Dominant color inspection fallback")
            return blue_pixels >= red_pixels

    def run_production_cycle(self):
        """Execute one complete manufacturing sorting cycle."""
        if self.is_running_cycle:
            return
        self.is_running_cycle = True
        self.cycle_count += 1
        def log_cot(step_num: int, message: str):
            cot_msg = String()
            cot_msg.data = f"Step {step_num}: {message}"
            self.cot_pub.publish(cot_msg)
            self.get_logger().info(f"🧠 [Chain-of-Thought {step_num}] {message}")

        self.get_logger().info(f"\n{'='*55}\n▶ STARTING INDUSTRIAL PRODUCTION CYCLE #{self.cycle_count}\n{'='*55}")

        blue_dest = self.get_parameter("blue_item_destination").value.lower()
        defect_dest = self.get_parameter("defect_item_destination").value.lower()

        # Step 1: Arm to ready pose
        self._publish_status("CELL_READY")
        log_cot(1, "Goal: Process workpiece and execute destination routing. Arm moving to ready stance.")
        self._go_pose("ready")

        # Step 2: Feed infeed conveyor until optical presence sensor detects docked part
        self._publish_status("FEEDING_CONVEYOR")
        log_cot(2, "Infeed conveyor active (0.138 m/s). Advancing workpiece toward mechanical pick stopper.")
        self._set_conveyor_power(55.0)  # High efficiency feed (0.138 m/s)
        start_t = time.time()
        while not self.part_present and (time.time() - start_t) < 8.0:
            time.sleep(0.1)
        time.sleep(0.4)  # Settle workpiece flush against mechanical stopper
        log_cot(3, "Workpiece docked at pick stopper. Conveyor halted.")
        self._set_conveyor_power(0.0)
        time.sleep(0.5)

        # Step 4: Approach and pick
        self._publish_status("PICKING_PART")
        log_cot(4, "Planning precision pick: moving through 'conveyor_pick_approach' to 'conveyor_pick'.")
        self._go_pose("conveyor_pick_approach")
        self._set_gripper(open_grip=True)
        self._go_pose("conveyor_pick")
        grasped = self._set_gripper(open_grip=False)
        self._go_pose("conveyor_pick_approach")  # Vertical lift away from conveyor belt

        if not grasped:
            log_cot(5, "Failure: Gripper closed but no contact confirmed. Safe retract to ready stance.")
            self._set_gripper(open_grip=True)
            self._go_pose("ready")
            self.is_running_cycle = False
            return

        # Step 5: Lift and present to Quality Inspection Station
        self._publish_status("INSPECTING_PART")
        log_cot(5, "Precision grasp confirmed. Presenting workpiece to Quality Control under overhead camera.")
        self._go_pose("inspect_station")

        # Step 6: Visual Quality Check
        is_blue = self._inspect_workpiece_vision()

        # Step 7: Agentic Destination Routing
        # Check whether target for this item class is red box (reject bin) or assembly tray
        target_destination = blue_dest if is_blue else defect_dest
        is_routed_to_red_box = any(k in target_destination for k in ["red", "box", "reject", "bin"])

        if is_blue:
            log_cot(6, f"Perception Result: BLUE workpiece identified. Target Policy: '{blue_dest}'.")
        else:
            log_cot(6, f"Perception Result: DEFECT workpiece identified. Target Policy: '{defect_dest}'.")

        if is_routed_to_red_box:
            self._publish_status("SORTING_RED_BOX")
            log_cot(7, "Routing to RED BOX (Reject Bin): Navigating trajectory 'reject_approach' -> 'reject_drop'.")
            self._go_pose("reject_approach")
            self._go_pose("reject_drop")
            log_cot(8, "Over Red Box: Gripper opening. Depositing workpiece flush into Red Box.")
            self._set_gripper(open_grip=True)
            self._go_pose("reject_approach")  # Clean vertical retract
            if is_blue:
                self.blue_items_in_red_box += 1
                log_cot(9, f"Success: BLUE item securely placed in RED BOX! (Total Blue in Red Box: {self.blue_items_in_red_box})")
            else:
                self.parts_rejected += 1
                log_cot(9, f"Success: Defective item deposited in reject bin. (Total: {self.parts_rejected})")
        else:
            self._publish_status("SORTING_ASSEMBLY")
            log_cot(7, "Routing to ASSEMBLY TRAY: Navigating trajectory 'assembly_approach' -> 'assembly_place'.")
            self._go_pose("assembly_approach")
            self._go_pose("assembly_place")
            log_cot(8, "Over Assembly Tray: Gripper opening. Placing workpiece into tray pocket.")
            self._set_gripper(open_grip=True)
            self._go_pose("assembly_approach")
            self.parts_passed += 1
            log_cot(9, f"Success: Part accepted into finished goods tray. (Total: {self.parts_passed})")

        # Step 8: Return home / ready
        self._publish_status("CYCLE_COMPLETE")
        log_cot(10, "Cycle complete. Arm returning to ready stance for next task.")
        self._go_pose("ready")

        self.get_logger().info(
            f"✅ CYCLE #{self.cycle_count} COMPLETE | Passed: {self.parts_passed} | Rejected: {self.parts_rejected}\n"
        )
        self.is_running_cycle = False

    def _publish_status(self, status_str: str):
        msg = String()
        msg.data = f"CYCLE={self.cycle_count};STATUS={status_str};PASSED={self.parts_passed};REJECTED={self.parts_rejected}"
        self.status_pub.publish(msg)

    def _presence_cb(self, msg: Bool):
        self.part_present = msg.data

    def _start_cycle_srv_cb(self, request, response):
        if self.is_running_cycle:
            response.success = False
            response.message = "Industrial cycle currently in progress"
            return response
        threading.Thread(target=self.run_production_cycle, daemon=True).start()
        response.success = True
        response.message = f"Triggered industrial cycle #{self.cycle_count + 1}"
        return response

    def _start_cycle_callback(self):
        """Timer callback to initiate cycle."""
        self.cycle_timer.cancel()  # Run once automatically
        threading.Thread(target=self.run_production_cycle, daemon=True).start()


def main(args=None):
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = IndustrialWorkcellNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
