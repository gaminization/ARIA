#!/usr/bin/env python3
"""
Activates all ARIA LifecycleNode agents and orchestrators.
"""
import sys
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

AGENTS = [
    'planning_agent', 'skill_agent', 'control_agent', 'safety_agent',
    'affordance_agent', 'reachability_agent', 'depth_agent', 'tracking_agent',
    'attention_agent', 'memory_agent', 'world_model_agent', 'learning_agent',
    'evaluation_agent', 'dialogue_agent', 'vision_agent', 'openvla_executor',
    'task_manager', 'memory_manager', 'health_monitor'
]

def activate_all():
    rclpy.init()
    node = Node('lifecycle_activator')
    node.get_logger().info("Activating all ARIA agents...")

    for name in AGENTS:
        change_srv = f"/{name}/change_state"
        get_srv = f"/{name}/get_state"

        get_client = node.create_client(GetState, get_srv)
        change_client = node.create_client(ChangeState, change_srv)

        if not change_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().warn(f"Agent /{name} not available, skipping.")
            continue

        # Check current state
        future = get_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        curr_state = future.result().current_state.label if future.done() else "unknown"

        if curr_state == "unconfigured":
            # Transition to configure (id=1)
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_CONFIGURE
            fut = change_client.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            curr_state = "inactive"

        if curr_state == "inactive":
            # Transition to activate (id=3)
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE
            fut = change_client.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)

        # Get final state
        fut = get_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        final_state = fut.result().current_state.label if fut.done() else "unknown"
        node.get_logger().info(f"  ✓ {name}: {final_state}")

    node.get_logger().info("All available agents activated.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    activate_all()
