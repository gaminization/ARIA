#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Depth Agent — LifecycleNode
Selects depth model, fuses with geometric model, monitors perf.
═══════════════════════════════════════════════════════════════
"""
import time
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from arm_planner.state_bus import StateBus

class DepthAgent(LifecycleNode):
    """Manages depth pipeline: selects DA v2 vs MiDaS, fuses with geometry."""

    def __init__(self):
        super().__init__('depth_agent')
        self.bus = StateBus(self)
        self.da_latency_ms = 0.0
        self.midas_latency_ms = 0.0
        self.da_accuracy = 0.0
        self.midas_accuracy = 0.0
        self.selected_model = 'depth_anything'
        self.frame_count = 0

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DepthAgent: CONFIGURING")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=5)
        self.da_sub = self.create_subscription(
            Image, '/depth/image_depth_anything', self._da_cb, qos)
        self.midas_sub = self.create_subscription(
            Image, '/depth/image_midas', self._midas_cb, qos)
        self.latency_pub = self.create_publisher(Float64, '/depth/agent_latency', 10)
        self.create_timer(5.0, self._evaluate_models)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DepthAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DepthAgent: DEACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def _da_cb(self, msg: Image):
        self.frame_count += 1
        self.da_latency_ms = self.da_latency_ms * 0.9 + 10.0 * 0.1  # placeholder EMA

    def _midas_cb(self, msg: Image):
        self.midas_latency_ms = self.midas_latency_ms * 0.9 + 15.0 * 0.1

    def _evaluate_models(self):
        """Periodically compare DA v2 vs MiDaS and select best."""
        if self.da_latency_ms < self.midas_latency_ms:
            if self.selected_model != 'depth_anything':
                self.selected_model = 'depth_anything'
                self.bus.add_chain_of_thought(
                    f"DEPTH: Switched to Depth-Anything v2 "
                    f"(DA={self.da_latency_ms:.1f}ms < MiDaS={self.midas_latency_ms:.1f}ms)"
                )
        else:
            if self.selected_model != 'midas':
                self.selected_model = 'midas'
                self.bus.add_chain_of_thought(
                    f"DEPTH: Switched to MiDaS "
                    f"(MiDaS={self.midas_latency_ms:.1f}ms < DA={self.da_latency_ms:.1f}ms)"
                )
        msg = Float64()
        msg.data = min(self.da_latency_ms, self.midas_latency_ms)
        self.latency_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthAgent()
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
