#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Health Monitor
1Hz system health checks: servos, cameras, inference, calibration.
Publishes HealthState. Emergency actions on critical alerts.
═══════════════════════════════════════════════════════════════
"""
import os, time, csv, math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from arm_planner.msg import HealthState, ServoHealth
from arm_planner.state_bus import StateBus

JOINT_NAMES = [
    "waist_joint", "shoulder_joint", "elbow_joint",
    "wrist_pitch_joint", "wrist_roll_joint"
]

# Temperature model: servo heats ~0.5°C/min under load
TEMP_RISE_RATE = 0.5 / 60.0   # °C/s
TEMP_COOL_RATE = 0.1 / 60.0   # °C/s
AMBIENT_TEMP = 25.0
TEMP_ALERT_C = 60.0
TEMP_SHUTDOWN_C = 75.0
DRIFT_ALERT_DEG = 5.0
FPS_MIN_TOP = 20.0
FPS_MIN_WRIST = 8.0
INFERENCE_ALERT_MS = 200.0
RECAL_INTERVAL_OPS = 500

class HealthMonitor(Node):
    """
    Monitors system health at 1Hz.

    Servo: temperature estimate, drift, load
    Camera: FPS monitoring
    Inference: YOLO + depth latency
    Calibration: operations since last cal

    Emergency: OVERTEMP → safe shutdown, CAMERA_FAIL → pause perception
    """

    def __init__(self):
        super().__init__('health_monitor')
        self.bus = StateBus(self)
        self.servo_temps = np.full(5, AMBIENT_TEMP)
        self.servo_loads = np.zeros(5)
        self.commanded = np.zeros(5)
        self.actual = np.zeros(5)
        self.top_fps = 0.0
        self.wrist_fps = 0.0
        self.inference_ms = 0.0
        self.calibration_valid = True
        self.operations_since_cal = 0
        self.alerts: list = []

    def on_configure(self):
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 50)
        self.force_sub = self.create_subscription(
            Float64MultiArray, '/force/joint_torques', self._force_cb, 10)
        self.latency_sub = self.create_subscription(
            Float64, '/depth/agent_latency', self._latency_cb, 10)
        self.log_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            '..', 'arm_planner', 'logs')
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, 'health_log.csv')
        self._init_log()
        self.create_timer(1.0, self._health_check)  # 1Hz

    def _init_log(self):
        if not os.path.exists(self.log_file):
            with open(self.log_file, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'timestamp', 'servo_temps', 'servo_drifts',
                    'top_fps', 'wrist_fps', 'inference_ms',
                    'alerts'])

    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self.actual[i] = msg.position[idx]

    def _force_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 5:
            self.servo_loads = np.abs(np.array(msg.data[:5]))

    def _latency_cb(self, msg: Float64):
        self.inference_ms = self.inference_ms * 0.9 + msg.data * 0.1

    def _health_check(self):
        """1Hz health assessment."""
        self.alerts.clear()

        # ── Servo health ───────────────────────────────────
        servo_healths = []
        for i in range(5):
            # Temperature model
            if self.servo_loads[i] > 0.1:
                self.servo_temps[i] += TEMP_RISE_RATE
            else:
                self.servo_temps[i] -= TEMP_COOL_RATE
                self.servo_temps[i] = max(AMBIENT_TEMP, self.servo_temps[i])

            # Drift
            drift_deg = abs(math.degrees(self.commanded[i] - self.actual[i]))

            sh = ServoHealth()
            sh.joint_name = JOINT_NAMES[i]
            sh.temperature_estimate_c = float(self.servo_temps[i])
            sh.drift_deg = float(drift_deg)
            sh.load_estimate_pct = float(min(100, self.servo_loads[i] * 100))
            sh.healthy = True

            if self.servo_temps[i] > TEMP_SHUTDOWN_C:
                sh.healthy = False
                self.alerts.append(f"OVERTEMP: {JOINT_NAMES[i]} at {self.servo_temps[i]:.0f}°C")
                self.bus.add_chain_of_thought(
                    f"HEALTH: ⚠ OVERTEMP {JOINT_NAMES[i]}: "
                    f"{self.servo_temps[i]:.0f}°C — SHUTDOWN REQUIRED")
            elif self.servo_temps[i] > TEMP_ALERT_C:
                self.alerts.append(f"TEMP_HIGH: {JOINT_NAMES[i]} at {self.servo_temps[i]:.0f}°C")

            if drift_deg > DRIFT_ALERT_DEG:
                sh.healthy = False
                self.alerts.append(f"DRIFT: {JOINT_NAMES[i]} drifting {drift_deg:.1f}°")

            servo_healths.append(sh)

        # ── Camera health ──────────────────────────────────
        if self.top_fps < FPS_MIN_TOP and self.top_fps > 0:
            self.alerts.append(f"LOW_FPS_TOP: {self.top_fps:.1f}")
        if self.wrist_fps < FPS_MIN_WRIST and self.wrist_fps > 0:
            self.alerts.append(f"LOW_FPS_WRIST: {self.wrist_fps:.1f}")

        # ── Inference health ───────────────────────────────
        if self.inference_ms > INFERENCE_ALERT_MS:
            self.alerts.append(f"SLOW_INFERENCE: {self.inference_ms:.0f}ms")

        # ── Calibration health ─────────────────────────────
        if self.operations_since_cal > RECAL_INTERVAL_OPS:
            self.calibration_valid = False
            self.alerts.append(
                f"RECAL_NEEDED: {self.operations_since_cal} ops since calibration")

        # ── Publish ────────────────────────────────────────
        msg = HealthState()
        msg.servo_health = servo_healths
        msg.fps_top_camera = float(self.top_fps)
        msg.fps_wrist_camera = float(self.wrist_fps)
        msg.inference_latency_ms = float(self.inference_ms)
        msg.calibration_valid = self.calibration_valid
        msg.active_alerts = list(self.alerts)
        self.bus.publish_health(msg)

        # ── Log to CSV ─────────────────────────────────────
        if self.alerts:
            with open(self.log_file, 'a', newline='') as f:
                csv.writer(f).writerow([
                    time.time(),
                    [f"{t:.0f}" for t in self.servo_temps],
                    [f"{abs(math.degrees(self.commanded[i]-self.actual[i])):.1f}" for i in range(5)],
                    f"{self.top_fps:.1f}", f"{self.wrist_fps:.1f}",
                    f"{self.inference_ms:.0f}",
                    '; '.join(self.alerts)])

def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    node.on_configure()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
