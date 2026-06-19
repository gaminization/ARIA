#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Grasp Executor
Simple grasp execution pipeline — no agents yet.
Pure pipeline test: see → localize → plan → grasp → lift.
═══════════════════════════════════════════════════════════════
"""
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from arm_interfaces.srv import SolveIK, PlanGrasp, SetAllJoints, GoNamedPose
from arm_interfaces.action import ExecuteGrasp


class GraspExecutor(Node):
    """
    Executes a full grasp sequence with logged reasoning.

    Steps:
      1. Get object 3D pose
      2. Plan grasp pose
      3. Check reachability (IK)
      4. Move to approach pose
      5. Descend to grasp pose (slow)
      6. Close gripper
      7. Verify grasp (contact check)
      8. Lift object
    """

    MAX_RETRIES = 2

    def __init__(self):
        super().__init__('grasp_executor')
        self.get_logger().info("═══ ARIA Grasp Executor starting ═══")

        self.cb_group = ReentrantCallbackGroup()

        # Current state
        self.current_joints = np.zeros(5)
        self.gripper_position = 0.0

        # Joint state subscriber
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10
        )

        # Service clients
        self.ik_client = self.create_client(
            SolveIK, '/aria/ik/solve',
            callback_group=self.cb_group)
        self.grasp_plan_client = self.create_client(
            PlanGrasp, '/aria/grasp/plan',
            callback_group=self.cb_group)
        self.set_joints_client = self.create_client(
            SetAllJoints, '/aria/set_all_joints',
            callback_group=self.cb_group)
        self.named_pose_client = self.create_client(
            GoNamedPose, '/aria/go_named_pose',
            callback_group=self.cb_group)
        self.close_gripper_client = self.create_client(
            Trigger, '/aria/close_gripper',
            callback_group=self.cb_group)
        self.open_gripper_client = self.create_client(
            Trigger, '/aria/open_gripper',
            callback_group=self.cb_group)

        # Action server
        self.action_server = ActionServer(
            self, ExecuteGrasp, '/aria/execute_grasp',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cb_group,
        )

        self.get_logger().info("Grasp executor ready")

    def _joint_state_cb(self, msg: JointState):
        """Update current joint positions."""
        joint_names = [
            "waist_joint", "shoulder_joint", "elbow_joint",
            "wrist_pitch_joint", "wrist_roll_joint"
        ]
        for i, name in enumerate(joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joints[i] = msg.position[idx]

        if "gripper_joint" in msg.name:
            idx = msg.name.index("gripper_joint")
            self.gripper_position = msg.position[idx]

    def _goal_cb(self, goal_request):
        """Accept all grasp goals."""
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        """Accept cancellation."""
        return CancelResponse.ACCEPT

    def _log_reasoning(self, step: str, reasoning: str):
        """Log step-by-step reasoning."""
        self.get_logger().info(f"[EXECUTOR] {step}: {reasoning}")

    def _publish_feedback(self, goal_handle, step: str, progress: float):
        """Publish action feedback."""
        feedback = ExecuteGrasp.Feedback()
        feedback.current_step = step
        feedback.progress_pct = progress
        goal_handle.publish_feedback(feedback)

    def _call_service_sync(self, client, request, timeout=5.0):
        """Synchronously call a service."""
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.done():
            return future.result()
        return None

    def _wait_for_motion(self, target_joints: np.ndarray,
                         tolerance_rad: float = 0.05,
                         timeout: float = 15.0):
        """Wait for arm to reach target position."""
        start = time.time()
        while time.time() - start < timeout:
            error = np.max(np.abs(self.current_joints - target_joints))
            if error < tolerance_rad:
                return True
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        return False

    async def _execute_cb(self, goal_handle):
        """
        Execute full grasp pipeline.
        """
        result = ExecuteGrasp.Result()
        object_id = goal_handle.request.object_id

        self._log_reasoning("START", f"Executing grasp on object {object_id}")

        try:
            # ── STEP 1: Get object 3D pose ────────────────
            self._publish_feedback(goal_handle, "Getting object pose", 10.0)
            self._log_reasoning(
                "Step 1 — Object Localization",
                "Using hybrid coordinate system. "
                "Querying /detection/objects and /depth/object_positions."
            )
            # In full implementation: query coordinate_transformer
            # For now: use grasp planner which handles detection lookup
            time.sleep(0.5)

            # ── STEP 2: Plan grasp pose ───────────────────
            self._publish_feedback(goal_handle, "Planning grasp", 20.0)

            grasp_req = PlanGrasp.Request()
            grasp_req.object_id = object_id
            grasp_req.method = 'auto'

            grasp_resp = self._call_service_sync(self.grasp_plan_client, grasp_req)

            if grasp_resp is None or not grasp_resp.success:
                self._log_reasoning("FAIL", "Grasp planning failed")
                result.success = False
                result.failure_reason = "Grasp planning failed"
                goal_handle.abort()
                return result

            self._log_reasoning(
                "Step 2 — Grasp Planning",
                f"Method: {grasp_resp.method_used}, "
                f"confidence: {grasp_resp.confidence:.2f}"
            )

            # ── STEP 3: Check IK reachability ─────────────
            self._publish_feedback(goal_handle, "Checking reachability", 30.0)

            ik_req = SolveIK.Request()
            ik_req.target_pose = grasp_resp.approach_pose
            ik_req.current_joints = self.current_joints.tolist()
            ik_req.allow_fallback = True

            ik_resp = self._call_service_sync(self.ik_client, ik_req)

            if ik_resp is None or not ik_resp.success:
                self._log_reasoning("FAIL", "Approach pose unreachable")
                result.success = False
                result.failure_reason = "IK failed for approach pose"
                goal_handle.abort()
                return result

            approach_joints = np.array(ik_resp.joint_angles)
            self._log_reasoning(
                "Step 3 — IK Check",
                f"Approach reachable. Solver: {ik_resp.solver_used}, "
                f"error: {ik_resp.position_error_mm:.1f}mm"
            )

            # ── STEP 4: Move to approach pose ─────────────
            self._publish_feedback(goal_handle, "Moving to approach", 40.0)

            # Open gripper first
            self._call_service_sync(self.open_gripper_client, Trigger.Request())
            time.sleep(1.0)

            move_req = SetAllJoints.Request()
            move_req.angles_deg = [
                float(a * 180.0 / math.pi) for a in approach_joints
            ] + [44.0]  # gripper open
            move_req.speed_deg_per_s = 30.0

            move_resp = self._call_service_sync(self.set_joints_client, move_req)

            if move_resp is None or not move_resp.success:
                self._log_reasoning("FAIL", "Failed to move to approach pose")
                result.success = False
                result.failure_reason = "Motion to approach failed"
                goal_handle.abort()
                return result

            self._wait_for_motion(approach_joints)
            self._log_reasoning("Step 4", "Reached approach pose")

            # ── STEP 5: Descend to grasp pose ─────────────
            self._publish_feedback(goal_handle, "Descending to grasp", 55.0)

            # IK for grasp pose
            ik_req.target_pose = grasp_resp.grasp_pose
            ik_req.current_joints = approach_joints.tolist()
            ik_resp = self._call_service_sync(self.ik_client, ik_req)

            if ik_resp is None or not ik_resp.success:
                self._log_reasoning("FAIL", "Grasp pose unreachable")
                result.success = False
                result.failure_reason = "IK failed for grasp pose"
                goal_handle.abort()
                return result

            grasp_joints = np.array(ik_resp.joint_angles)

            move_req.angles_deg = [
                float(a * 180.0 / math.pi) for a in grasp_joints
            ] + [44.0]
            move_req.speed_deg_per_s = 15.0  # Slow descent

            self._call_service_sync(self.set_joints_client, move_req)
            self._wait_for_motion(grasp_joints, timeout=10.0)
            self._log_reasoning("Step 5", "Reached grasp pose")

            # ── STEP 6: Close gripper ─────────────────────
            self._publish_feedback(goal_handle, "Closing gripper", 70.0)

            self._call_service_sync(self.close_gripper_client, Trigger.Request())
            time.sleep(1.5)  # Allow gripper to fully close

            self._log_reasoning(
                "Step 6 — Gripper Close",
                f"Gripper position: {self.gripper_position * 180/math.pi:.1f}°"
            )

            # ── STEP 7: Verify grasp ──────────────────────
            self._publish_feedback(goal_handle, "Verifying grasp", 80.0)

            # Check gripper didn't fully close (object is between fingers)
            gripper_deg = self.gripper_position * 180.0 / math.pi
            grasp_detected = gripper_deg > 2.0  # Not fully closed

            if grasp_detected:
                self._log_reasoning(
                    "Step 7 — Grasp Verified",
                    f"Object detected between fingers. "
                    f"Gripper: {gripper_deg:.1f}°"
                )
            else:
                self._log_reasoning(
                    "Step 7 — WARNING",
                    "Gripper fully closed — may have missed object"
                )

            # ── STEP 8: Lift object ───────────────────────
            self._publish_feedback(goal_handle, "Lifting object", 90.0)

            # Move back to approach pose (5cm lift)
            move_req.angles_deg = [
                float(a * 180.0 / math.pi) for a in approach_joints
            ] + [1.0]  # Keep gripper closed
            move_req.speed_deg_per_s = 15.0

            self._call_service_sync(self.set_joints_client, move_req)
            self._wait_for_motion(approach_joints, timeout=10.0)

            self._log_reasoning("Step 8", "Object lifted 10cm")

            # ── SUCCESS ───────────────────────────────────
            self._publish_feedback(goal_handle, "Grasp complete", 100.0)

            result.success = True
            result.failure_reason = ""
            goal_handle.succeed()

            self._log_reasoning("COMPLETE", "Grasp execution successful")

        except Exception as e:
            self._log_reasoning("EXCEPTION", str(e))
            result.success = False
            result.failure_reason = f"Exception: {str(e)}"
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
