#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Depth Agent — LifecycleNode
Selects depth model, fuses with geometric model, monitors perf.
═══════════════════════════════════════════════════════════════
"""
import time
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import Float64, Float64MultiArray
from arm_planner.state_bus import StateBus

class DepthAgent(LifecycleNode):
    """Reports measured inference timings for the active depth backends."""

    def __init__(self):
        super().__init__('depth_agent')
        self.bus = StateBus(self)
        self.da_latency_ms = 0.0
        self.midas_latency_ms = 0.0

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DepthAgent: CONFIGURING")
        self.metrics_sub = self.create_subscription(
            Float64MultiArray,
            '/depth/benchmark_metrics',
            self._metrics_cb,
            10,
        )
        self.latency_pub = self.create_publisher(Float64, '/depth/agent_latency', 10)
        self.create_timer(5.0, self._evaluate_models)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DepthAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DepthAgent: DEACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def _metrics_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warning(
                "Ignoring malformed depth metrics; expected DA and MiDaS timings")
            return
        da_ms, midas_ms = msg.data[:2]
        if da_ms > 0.0:
            self.da_latency_ms = (
                da_ms if self.da_latency_ms == 0.0
                else self.da_latency_ms * 0.9 + da_ms * 0.1
            )
        if midas_ms > 0.0:
            self.midas_latency_ms = (
                midas_ms if self.midas_latency_ms == 0.0
                else self.midas_latency_ms * 0.9 + midas_ms * 0.1
            )

    def _evaluate_models(self):
        """Publish the fastest measured backend latency; this does not switch models."""
        observed = [
            latency for latency in (self.da_latency_ms, self.midas_latency_ms)
            if latency > 0.0
        ]
        msg = Float64()
        msg.data = min(observed) if observed else 0.0
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
