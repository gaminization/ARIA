#!/usr/bin/env python3
"""
Activates all ARIA LifecycleNode agents and orchestrators.

Robustness: lifecycle activation is a three-step handshake
(unconfigured -> configure -> inactive -> activate) that occasionally
loses a race with node spin-up under load. Each agent is therefore
retried a few times and, crucially, the whole sweep is repeated until
every *required* agent reports 'active' (or attempts are exhausted), so
a transient miss never leaves the stack half-started.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

# Required for planning/execution: if these are not active the task
# manager hangs in PLANNING and nothing runs. The rest are telemetry
# helpers and must not block the mission if they miss.
REQUIRED_AGENTS = [
    'llm_planning_agent', 'skill_agent', 'control_agent', 'safety_agent',
    'task_manager',
]
AGENTS = [
    'llm_planning_agent', 'skill_agent', 'control_agent', 'safety_agent',
    'affordance_agent', 'reachability_agent', 'depth_agent', 'tracking_agent',
    'attention_agent', 'memory_agent', 'world_model_agent', 'learning_agent',
    'evaluation_agent', 'dialogue_agent', 'vision_agent', 'openvla_executor',
    'task_manager',
]

MAX_ROUNDS = 4
WAIT_SERVICE_S = 8.0


def _get_state(node, name, get_client):
    fut = get_client.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
    if fut.done() and fut.result():
        return fut.result().current_state.label
    return "unknown"


def _request(node, client, transition_id, timeout_s=15.0):
    req = ChangeState.Request()
    req.transition.id = transition_id
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_s)
    return fut.done()


def activate_agent(node, name):
    """Configure + activate one agent. Returns final state label."""
    change_client = node.create_client(
        ChangeState, f"/{name}/change_state")
    get_client = node.create_client(GetState, f"/{name}/get_state")

    if not change_client.wait_for_service(timeout_sec=WAIT_SERVICE_S):
        return "unavailable"

    state = _get_state(node, name, get_client)
    if state == "active":
        return "active"

    if state == "unconfigured":
        _request(node, change_client, Transition.TRANSITION_CONFIGURE)
        state = _get_state(node, name, get_client)

    if state == "inactive":
        _request(node, change_client, Transition.TRANSITION_ACTIVATE)
        state = _get_state(node, name, get_client)

    return state


def activate_all():
    rclpy.init()
    node = Node('lifecycle_activator')
    node.get_logger().info("Activating all ARIA agents...")

    final = {name: "unknown" for name in AGENTS}

    for round_no in range(1, MAX_ROUNDS + 1):
        pending = []
        for name in AGENTS:
            if final.get(name) == "active":
                continue
            state = activate_agent(node, name)
            final[name] = state
            node.get_logger().info(
                f"  [{round_no}] {name}: {state}")
            if state != "active":
                pending.append(name)

        missing_required = [n for n in REQUIRED_AGENTS if final.get(n) != "active"]
        if not pending or (round_no >= 2 and not missing_required):
            break
        if round_no < MAX_ROUNDS and pending:
            time.sleep(2.0)

    missing_required = [n for n in REQUIRED_AGENTS if final.get(n) != "active"]
    if missing_required:
        node.get_logger().error(
            f"REQUIRED agents did not activate: {missing_required}")
    else:
        node.get_logger().info("All available agents activated.")

    node.destroy_node()
    rclpy.shutdown()
    return not missing_required


if __name__ == "__main__":
    ok = activate_all()
    sys.exit(0 if ok else 1)
