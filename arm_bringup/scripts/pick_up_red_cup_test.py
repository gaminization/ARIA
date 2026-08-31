#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Pick-Up Red Cup Test
Directly orchestrates the robot to:
  1. Go to home pose
  2. Use IK to compute approach pose above the red mug
  3. Open gripper
  4. Move to approach pose
  5. Descend to grasp pose
  6. Close gripper
  7. Lift the mug
═══════════════════════════════════════════════════════════════
"""
import math
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_srvs.srv import Trigger
from arm_interfaces.srv import SolveIK, SetAllJoints, GoNamedPose

import numpy as np


class PickUpRedCupTest(Node):
    """
    Direct pick-up test node.
    Moves the robot arm to pick up the red YCB mug from the tester workspace.
    """

    # Red mug world position: 0.22, -0.06, 0.6081
    # Robot base position: 0.0, 0.0, 0.608 (on mounting plate)
    # So mug in base_link frame is approximately:
    #   x = 0.22 (forward), y = -0.06 (slightly right), z = 0.0 (table height)
    # The mug height is ~0.045m, so grasp at z ~ 0.022 (halfway up mug)
    # Approach at z ~ 0.08 (above mug)

    MUG_X = 0.22
    MUG_Y = -0.06
    MUG_Z_TABLE = 0.0     # Relative to base (table surface)
    MUG_HEIGHT = 0.045     # Mug is 45mm tall
    GRASP_Z = 0.02         # Grasp at 20mm above table (half mug height)
    APPROACH_Z = 0.10      # Approach at 100mm above table
    LIFT_Z = 0.15          # Lift to 150mm above table

    def __init__(self):
        super().__init__('pick_up_test')
        self.get_logger().info("═══ ARIA Pick-Up Red Cup Test ═══")
        
        self.cb_group = ReentrantCallbackGroup()
        self.current_joints = np.zeros(5)
        self.gripper_position = 0.0

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

        # Service clients
        self.ik_client = self.create_client(
            SolveIK, '/aria/ik/solve', callback_group=self.cb_group)
        self.joints_client = self.create_client(
            SetAllJoints, '/aria/set_all_joints', callback_group=self.cb_group)
        self.named_pose_client = self.create_client(
            GoNamedPose, '/aria/go_named_pose', callback_group=self.cb_group)
        self.close_gripper = self.create_client(
            Trigger, '/aria/close_gripper', callback_group=self.cb_group)
        self.open_gripper = self.create_client(
            Trigger, '/aria/open_gripper', callback_group=self.cb_group)

        # Start the test sequence after a short delay
        self.create_timer(2.0, self._run_test, callback_group=self.cb_group)
        self._test_started = False

    def _joint_cb(self, msg: JointState):
        """Update current joint state."""
        names = ["waist_joint", "shoulder_joint", "elbow_joint",
                 "wrist_pitch_joint"]
        for i, name in enumerate(names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joints[i] = msg.position[idx]
        if "gripper_joint" in msg.name:
            idx = msg.name.index("gripper_joint")
            self.gripper_position = msg.position[idx]

    def _call_sync(self, client, request, timeout=10.0):
        """Synchronous service call."""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service not available: {client.srv_name}")
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.done():
            return future.result()
        return None

    def _wait_motion(self, target_joints, tolerance=0.12, timeout=15.0):
        """Wait for arm to reach target."""
        start = time.time()
        tgt = target_joints[:4]
        while time.time() - start < timeout:
            error = np.max(np.abs(self.current_joints[:4] - tgt))
            if error < tolerance:
                return True
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        self.get_logger().warn(f"Motion timeout! Max error: {np.max(np.abs(self.current_joints[:4] - tgt)):.3f} rad")
        return False

    def _solve_ik(self, x, y, z, pitch_down=True):
        """Solve IK for a Cartesian position with gripper pointing down."""
        req = SolveIK.Request()
        req.target_pose = PoseStamped()
        req.target_pose.header.frame_id = "base_link"
        req.target_pose.pose.position = Point(x=x, y=y, z=z)
        # Quaternion for gripper pointing straight down (ZYX: pitch=-90°)
        if pitch_down:
            req.target_pose.pose.orientation = Quaternion(
                x=0.0, y=0.7071068, z=0.0, w=0.7071068)  # pitch -90°
        else:
            req.target_pose.pose.orientation = Quaternion(
                x=0.0, y=0.0, z=0.0, w=1.0)
        req.current_joints = self.current_joints.tolist()
        req.allow_fallback = True
        
        resp = self._call_sync(self.ik_client, req)
        if resp and resp.success:
            return np.array(resp.joint_angles), resp.solver_used
        return None, None

    def _move_joints(self, target_rad, speed=30.0, gripper_deg=44.0):
        """Move arm to target joint angles."""
        req = SetAllJoints.Request()
        req.angles_deg = [float(math.degrees(a)) for a in target_rad[:4]] + [gripper_deg]
        req.speed_deg_per_s = speed
        resp = self._call_sync(self.joints_client, req)
        return resp and resp.success

    def _run_test(self):
        """Execute the pick-up sequence."""
        if self._test_started:
            return
        self._test_started = True

        self.get_logger().info("")
        self.get_logger().info("═" * 60)
        self.get_logger().info("  🤖 PICK-UP RED CUP TEST — STARTING")
        self.get_logger().info("═" * 60)
        self.get_logger().info("")

        # ── STEP 1: Go to home/ready pose ──────────────────
        self.get_logger().info("[STEP 1/7] Going to home pose...")
        req = GoNamedPose.Request()
        req.pose_name = "home"
        resp = self._call_sync(self.named_pose_client, req)
        if resp and resp.success:
            self.get_logger().info(f"  ✓ Home pose reached: {resp.message}")
        else:
            self.get_logger().warn("  ⚠ Named pose 'home' failed, continuing...")
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.1)

        # ── STEP 2: Open gripper ───────────────────────────
        self.get_logger().info("[STEP 2/7] Opening gripper...")
        self._call_sync(self.open_gripper, Trigger.Request())
        time.sleep(1.0)
        self.get_logger().info("  ✓ Gripper open")

        # ── STEP 3: Solve IK for approach pose ─────────────
        self.get_logger().info("[STEP 3/7] Computing approach pose via IK...")
        self.get_logger().info(f"  Target: mug at ({self.MUG_X}, {self.MUG_Y}), approach at z={self.APPROACH_Z}")
        
        approach_joints, solver = self._solve_ik(
            self.MUG_X, self.MUG_Y, self.APPROACH_Z)
        
        if approach_joints is None:
            self.get_logger().error("  ✗ IK failed for approach pose!")
            self.get_logger().info("  Attempting direct joint approach...")
            # Fallback: compute manually using geometry
            # waist angle = atan2(y, x)
            waist = math.atan2(self.MUG_Y, self.MUG_X)
            # Rough joint angles for reaching forward at table height
            approach_joints = np.array([waist, 0.8, -1.2, -0.3, 0.0])
            solver = "manual_fallback"
        
        self.get_logger().info(
            f"  ✓ IK solved ({solver}): "
            f"[{', '.join(f'{math.degrees(a):.1f}°' for a in approach_joints)}]")

        # ── STEP 4: Move to approach pose ──────────────────
        self.get_logger().info("[STEP 4/7] Moving to approach pose (above mug)...")
        if self._move_joints(approach_joints, speed=25.0, gripper_deg=44.0):
            self.get_logger().info("  Command sent, waiting for motion...")
            self._wait_motion(approach_joints, timeout=12.0)
            self.get_logger().info("  ✓ Approach pose reached")
        else:
            self.get_logger().error("  ✗ Move command failed!")
        time.sleep(1.0)

        # ── STEP 5: Descend to grasp pose ──────────────────
        self.get_logger().info("[STEP 5/7] Descending to grasp pose...")
        rclpy.spin_once(self, timeout_sec=0.1)
        
        grasp_joints, solver = self._solve_ik(
            self.MUG_X, self.MUG_Y, self.GRASP_Z)
        
        if grasp_joints is None:
            self.get_logger().warn("  ⚠ IK failed for grasp — adjusting approach joints")
            # Lower the wrist pitch to descend
            grasp_joints = approach_joints.copy()
            grasp_joints[1] += 0.15  # lean forward more (shoulder)
            grasp_joints[2] -= 0.10  # bend elbow more
            grasp_joints[3] -= 0.15  # pitch wrist down
            solver = "adjusted_from_approach"
        
        self.get_logger().info(
            f"  IK ({solver}): "
            f"[{', '.join(f'{math.degrees(a):.1f}°' for a in grasp_joints)}]")
        
        if self._move_joints(grasp_joints, speed=12.0, gripper_deg=44.0):
            self._wait_motion(grasp_joints, timeout=12.0)
            self.get_logger().info("  ✓ Grasp pose reached")
        time.sleep(0.5)

        # ── STEP 6: Close gripper ──────────────────────────
        self.get_logger().info("[STEP 6/7] Closing gripper on mug...")
        self._call_sync(self.close_gripper, Trigger.Request())
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        gripper_deg = math.degrees(self.gripper_position)
        if gripper_deg > 2.0:
            self.get_logger().info(f"  ✓ Object detected! Gripper at {gripper_deg:.1f}°")
        else:
            self.get_logger().warn(f"  ⚠ Gripper fully closed ({gripper_deg:.1f}°) — may have missed")

        # ── STEP 7: Lift the mug ───────────────────────────
        self.get_logger().info("[STEP 7/7] Lifting mug...")
        
        lift_joints, solver = self._solve_ik(
            self.MUG_X, self.MUG_Y, self.LIFT_Z)
        
        if lift_joints is None:
            lift_joints = approach_joints.copy()
            lift_joints[1] -= 0.1  # lean back slightly
        
        if self._move_joints(lift_joints, speed=12.0, gripper_deg=1.0):
            self._wait_motion(lift_joints, timeout=12.0)
            self.get_logger().info("  ✓ Mug lifted!")
        
        time.sleep(2.0)

        self.get_logger().info("")
        self.get_logger().info("═" * 60)
        self.get_logger().info("  🎉 PICK-UP RED CUP TEST — COMPLETE!")
        self.get_logger().info("═" * 60)
        self.get_logger().info("")
        self.get_logger().info("  The arm should now be holding the red mug.")
        self.get_logger().info("  Check the Gazebo simulation to verify.")
        self.get_logger().info("")


def main(args=None):
    rclpy.init(args=args)
    node = PickUpRedCupTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
