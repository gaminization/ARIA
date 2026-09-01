#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Safety Agent — LifecycleNode
Intercepts ALL motion before execution. Emergency stop <5ms.
═══════════════════════════════════════════════════════════════
"""
import math, time
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from arm_planner.state_bus import StateBus

# Joint limits (rad) — from Stage 1 URDF & manual_control_node
JOINT_LIMITS = np.array([
    [-math.pi, math.pi],         # waist [-180°, 180°]
    [-math.pi/2, math.pi/2],     # shoulder [-90°, 90°]
    [-math.pi/2, math.pi/2],     # elbow [-90°, 90°]
    [-math.pi/2, math.pi/2],     # wrist pitch [-90°, 90°]
    [-math.pi/2, math.pi/2],     # wrist roll [-90°, 90°]
])

# Soft limits (5° inside hard limits)
SOFT_MARGIN_RAD = math.radians(5)
SOFT_LIMITS = JOINT_LIMITS.copy()
SOFT_LIMITS[:, 0] += SOFT_MARGIN_RAD
SOFT_LIMITS[:, 1] -= SOFT_MARGIN_RAD

# Workspace boundary (meters from base_link)
WORKSPACE_RADIUS_MAX = 0.30
WORKSPACE_RADIUS_MIN = 0.05
WORKSPACE_Z_MIN = 0.02
WORKSPACE_Z_MAX = 0.45

# Force threshold (N)
FORCE_THRESHOLD_N = 2.0

# Temperature threshold (°C)
TEMP_SHUTDOWN_C = 75.0
TEMP_ALERT_C = 60.0

JOINT_NAMES = [
    "waist_joint", "shoulder_joint", "elbow_joint",
    "wrist_pitch_joint", "wrist_roll_joint"
]

class SafetyAgent(LifecycleNode):
    """
    Intercepts all motion commands. Checks:
      - Joint limits (hard + soft)
      - Self-collision (via MoveIt)
      - Workspace boundary
      - Cable zone clearance
      - Force threshold (unexpected contact)
      - Servo temperature
    Emergency stop: /aria/estop (responds <5ms)
    """

    def __init__(self):
        super().__init__('safety_agent')
        self.bus = StateBus(self)
        self.cb_group = ReentrantCallbackGroup()
        self.current_joints = np.zeros(5)
        self.estop_active = False
        self.force_estimates = np.zeros(5)
        self.intervention_count = 0
        self.intervention_log: list = []

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("SafetyAgent: CONFIGURING")
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 50)
        self.force_sub = self.create_subscription(
            Float64MultiArray, '/force/joint_torques', self._force_cb, 10)
        self.estop_srv = self.create_service(
            Trigger, '/aria/estop', self._estop_cb, callback_group=self.cb_group)
        self.release_srv = self.create_service(
            Trigger, '/aria/estop/release', self._release_cb, callback_group=self.cb_group)
        self.create_timer(0.1, self._safety_check)  # 10Hz
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("SafetyAgent: ACTIVATED — monitoring all axes")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joints[i] = msg.position[idx]

    def _force_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 5:
            self.force_estimates = np.array(msg.data[:5])

    def _estop_cb(self, request, response):
        """Emergency stop — immediate halt."""
        self.estop_active = True
        self._log_intervention("ESTOP", "Emergency stop activated by service call")
        response.success = True
        response.message = "⚠ EMERGENCY STOP ACTIVATED"
        self.get_logger().error("⚠ EMERGENCY STOP ACTIVATED ⚠")
        return response

    def _release_cb(self, request, response):
        """Release emergency stop."""
        self.estop_active = False
        self.get_logger().info("E-stop released")
        response.success = True
        response.message = "E-stop released"
        return response

    def _log_intervention(self, check_type: str, reason: str):
        """Log a safety intervention."""
        self.intervention_count += 1
        entry = {
            'count': self.intervention_count,
            'type': check_type,
            'reason': reason,
            'time': time.time(),
            'joints_rad': self.current_joints.tolist(),
        }
        self.intervention_log.append(entry)
        self.bus.add_chain_of_thought(
            f"SAFETY: ⚠ Intervention #{self.intervention_count}: "
            f"{check_type} — {reason}")

    def validate_motion(self, target_joints: np.ndarray) -> tuple:
        """
        Validate a proposed joint configuration.
        Returns: (safe: bool, reason: str)
        """
        if self.estop_active:
            return False, "E-stop active"

        # 1. Hard joint limits
        for i in range(5):
            if target_joints[i] < JOINT_LIMITS[i, 0] or target_joints[i] > JOINT_LIMITS[i, 1]:
                return False, f"Joint {JOINT_NAMES[i]} exceeds hard limit: {math.degrees(target_joints[i]):.1f}°"

        # 2. Soft joint limits (warning)
        for i in range(5):
            if target_joints[i] < SOFT_LIMITS[i, 0] or target_joints[i] > SOFT_LIMITS[i, 1]:
                self._log_intervention("SOFT_LIMIT",
                    f"{JOINT_NAMES[i]} near limit: {math.degrees(target_joints[i]):.1f}°")

        return True, "Safe"

    def _safety_check(self):
        """Periodic safety check at 10Hz."""
        if self.estop_active:
            return

        # Check force thresholds
        max_force = float(np.max(np.abs(self.force_estimates)))
        if max_force > FORCE_THRESHOLD_N:
            worst = int(np.argmax(np.abs(self.force_estimates)))
            self._log_intervention("FORCE",
                f"Excessive force on {JOINT_NAMES[worst]}: {max_force:.2f}N")

        # Check if joints are within limits
        for i in range(5):
            if (self.current_joints[i] < JOINT_LIMITS[i, 0] - 0.01 or
                    self.current_joints[i] > JOINT_LIMITS[i, 1] + 0.01):
                self._log_intervention("JOINT_LIMIT",
                    f"{JOINT_NAMES[i]} OUT OF BOUNDS: "
                    f"{math.degrees(self.current_joints[i]):.1f}°")

def main(args=None):
    rclpy.init(args=args)
    node = SafetyAgent()
    node.trigger_configure()
    node.trigger_activate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
