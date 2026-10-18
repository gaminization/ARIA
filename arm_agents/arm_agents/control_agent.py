#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Control Agent — LifecycleNode
Joint-level command execution. MoveIt2 + trajectory controller.
Triggers visual servoing. Reports execution status.
═══════════════════════════════════════════════════════════════
"""
import math, time
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import Twist
from arm_interfaces.srv import SetAllJoints, SolveIK
from arm_planner.state_bus import StateBus

TRACKING_ERROR_THRESHOLD_RAD = 0.1  # Alert if tracking error exceeds
VISUAL_SERVO_DISTANCE_M = 0.08     # Activate visual servo within 8cm

# ── VLA execution safety gate ──────────────────────────────────
# The VLA policy emits raw joint deltas that are NOT safety-checked.
# ControlAgent is the only consumer that turns them into motion, so it
# clamps each step and requires an explicit vla-active + no-estop gate.
VLA_MAX_STEP_RAD = math.radians(4.0)   # max |delta| accepted per joint/step
VLA_MIN_PERIOD_S = 0.15                # don't execute faster than ~6.7 Hz
VLA_CMD_SPEED_DPS = 20.0               # conservative joint speed cap
# (waist, shoulder, elbow, wrist_pitch) limits in radians
VLA_JOINT_LIMITS_RAD = [
    (-math.pi, math.pi),
    (-math.pi / 2, math.pi / 2),
    (-math.pi / 2, math.pi / 2),
    (-math.pi / 2, math.pi / 2),
]

class ControlAgent(LifecycleNode):
    """
    Interfaces with ros2_control joint_trajectory_controller.
    Monitors execution feedback (tracking error).
    Triggers visual servoing for final approach.
    """

    JOINT_NAMES = [
        "waist_joint", "shoulder_joint", "elbow_joint",
        "wrist_pitch_joint"
    ]

    def __init__(self):
        super().__init__('control_agent')
        self.bus = StateBus(self)
        self.cb_group = ReentrantCallbackGroup()
        self.current_joints = np.zeros(4)
        self.commanded_joints = np.zeros(4)
        self.tracking_error = np.zeros(4)
        self.visual_servo_active = False
        self.is_moving = False
        self._last_error_log_time = 0.0
        # VLA execution gate
        self.vla_active = False
        self.estop_active = False
        self._last_vla_cmd_time = 0.0
        self._vla_cmds = 0

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("ControlAgent: CONFIGURING")
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 50)
        self.servo_active_sub = self.create_subscription(
            Bool, '/visual_servo/active', self._servo_active_cb, 10)
        self.joints_client = self.create_client(
            SetAllJoints, '/aria/set_all_joints', callback_group=self.cb_group)
        self.ik_client = self.create_client(
            SolveIK, '/aria/ik/solve', callback_group=self.cb_group)
        # VLA delta consumer — turns policy deltas into real, gated motion.
        self.vla_delta_sub = self.create_subscription(
            Float64MultiArray, '/aria/vla/joint_deltas',
            self._vla_delta_cb, 10, callback_group=self.cb_group)
        self.vla_active_sub = self.create_subscription(
            Bool, '/aria/vla/active', self._vla_active_cb, 10)
        self.estop_sub = self.create_subscription(
            Bool, '/aria/estop_active', self._estop_cb, 10)
        self.create_timer(0.1, self._monitor)  # 10Hz monitoring
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("ControlAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(self.JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joints[i] = msg.position[idx]
        if not self.is_moving:
            self.commanded_joints = self.current_joints.copy()

    def _servo_active_cb(self, msg: Bool):
        self.visual_servo_active = msg.data

    def _vla_active_cb(self, msg: Bool):
        was = self.vla_active
        self.vla_active = bool(msg.data)
        if self.vla_active != was:
            self.bus.add_chain_of_thought(
                f"CONTROL: VLA execution gate "
                f"{'ENABLED' if self.vla_active else 'disabled'}")

    def _estop_cb(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def _vla_delta_cb(self, msg: Float64MultiArray):
        """Execute a VLA joint-delta command through the safety gate.

        Deltas arrive as [d_waist, d_shoulder, d_elbow, d_wrist, gripper]
        in radians. They are clamped per-step and per joint-limit before
        being sent to /aria/set_all_joints. Execution is refused when the
        VLA gate is off or an e-stop is active.
        """
        if not self.vla_active:
            return
        if self.estop_active:
            self.bus.add_chain_of_thought(
                "CONTROL: VLA delta rejected — e-stop active")
            return
        data = list(msg.data)
        if len(data) < len(self.JOINT_NAMES):
            self.bus.add_chain_of_thought(
                "CONTROL: VLA delta malformed "
                f"(expected >= {len(self.JOINT_NAMES)} values, got {len(data)})")
            return

        now = time.time()
        if now - self._last_vla_cmd_time < VLA_MIN_PERIOD_S:
            return  # rate-limit to avoid command flooding
        self._last_vla_cmd_time = now

        deltas = np.asarray(data[:len(self.JOINT_NAMES)], dtype=float)
        if not np.all(np.isfinite(deltas)):
            self.bus.add_chain_of_thought(
                "CONTROL: VLA delta rejected — non-finite values")
            return

        # Per-step clamp.
        clipped = np.clip(deltas, -VLA_MAX_STEP_RAD, VLA_MAX_STEP_RAD)
        if not np.allclose(clipped, deltas):
            self.bus.add_chain_of_thought(
                "CONTROL: VLA delta clamped per-step "
                f"(max {math.degrees(np.max(np.abs(clipped))):.1f}°)")

        target = self.current_joints + clipped
        for i, (lo, hi) in enumerate(VLA_JOINT_LIMITS_RAD):
            target[i] = float(np.clip(target[i], lo, hi))

        if self._command_joints_async(target, speed_dps=VLA_CMD_SPEED_DPS):
            self._vla_cmds += 1

    def _command_joints_async(self, target_rad: np.ndarray,
                              speed_dps: float = 30.0) -> bool:
        """Fire a joint command without blocking the callback (VLA path)."""
        if not self.joints_client.service_is_ready():
            if not self.joints_client.wait_for_service(timeout_sec=0.1):
                return False
        self.commanded_joints = np.asarray(target_rad, dtype=float).copy()
        self.is_moving = True
        req = SetAllJoints.Request()
        req.angles_deg = [
            float(math.degrees(a)) for a in target_rad] + [44.0]
        req.speed_deg_per_s = speed_dps
        self.joints_client.call_async(req)
        return True

    def _monitor(self):
        """Monitor tracking error when moving."""
        if not self.is_moving:
            return
        self.tracking_error = np.abs(self.commanded_joints - self.current_joints)
        max_error = float(np.max(self.tracking_error))
        if max_error < TRACKING_ERROR_THRESHOLD_RAD:
            # Arrived — stop reporting as moving so _joint_cb resumes sync.
            self.is_moving = False
            return
        now = time.time()
        if max_error > TRACKING_ERROR_THRESHOLD_RAD and (now - self._last_error_log_time > 3.0):
            self._last_error_log_time = now
            worst_joint = int(np.argmax(self.tracking_error))
            self.bus.add_chain_of_thought(
                f"CONTROL: High tracking error on {self.JOINT_NAMES[worst_joint]}: "
                f"{math.degrees(max_error):.1f}° — possible load or stall")

    def move_to_joints(self, target_rad: np.ndarray, speed_dps: float = 30.0) -> bool:
        """Command arm to joint position."""
        target_rad = np.asarray(target_rad, dtype=float)
        if target_rad.shape != (len(self.JOINT_NAMES),):
            self.get_logger().error(
                f"Expected {len(self.JOINT_NAMES)} arm joint targets, "
                f"got shape {target_rad.shape}")
            return False
        self.commanded_joints = target_rad.copy()
        self.bus.add_chain_of_thought(
            f"CONTROL: Moving to [{', '.join(f'{math.degrees(a):.1f}°' for a in target_rad)}] "
            f"at {speed_dps}°/s")
        if not self.joints_client.wait_for_service(timeout_sec=2.0):
            return False
        req = SetAllJoints.Request()
        req.angles_deg = [float(math.degrees(a)) for a in target_rad] + [44.0]
        req.speed_deg_per_s = speed_dps
        future = self.joints_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if future.done() and future.result() and future.result().success:
            return True
        return False

    def wait_for_arrival(self, target_rad: np.ndarray,
                         tolerance_rad: float = 0.05, timeout: float = 15.0) -> bool:
        """Block until arm reaches target or timeout."""
        target_rad = np.asarray(target_rad, dtype=float)
        if target_rad.shape != (len(self.JOINT_NAMES),):
            self.get_logger().error(
                f"Expected {len(self.JOINT_NAMES)} arm joint targets, "
                f"got shape {target_rad.shape}")
            return False
        start = time.time()
        while time.time() - start < timeout:
            error = np.max(np.abs(self.current_joints - target_rad))
            if error < tolerance_rad:
                return True
            time.sleep(0.05)
            rclpy.spin_once(self, timeout_sec=0.01)
        return False

def main(args=None):
    rclpy.init(args=args)
    node = ControlAgent()
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
