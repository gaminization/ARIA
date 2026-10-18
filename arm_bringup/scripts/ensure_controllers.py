#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Controller Bring-up (robust)

Brings ros2_control controllers to the ``active`` state and verifies it via
``list_controllers`` instead of trusting a single spawner exit code.

Why this exists
---------------
The stock ``controller_manager spawner`` can time out waiting for the
``load_controller`` service response when the controller manager is briefly
unresponsive (e.g. CPU contention at start-up). When that happens the
controller is often loaded *anyway*, so the spawner's own retry then fails with
"A controller named ... was already loaded" and exits non-zero, which makes the
launch file shut the whole system down.

This node calls the controller manager services directly and, for each
controller, converges regardless of the intermediate state:

    not loaded -> load -> configure -> activate
    loaded     -> configure -> activate
    configured -> activate
    active     -> done

Every step is confirmed by re-reading ``list_controllers``; if a service call
times out but the desired state was actually reached, that is not treated as a
failure. It exits 0 only when every requested controller is genuinely active,
and exits 1 otherwise - it never reports success it has not verified.
═══════════════════════════════════════════════════════════════
"""
import argparse
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)


class ControllerEnsurer(Node):
    def __init__(self, controller_manager: str, timeout: float):
        super().__init__("ensure_controllers")
        self._cm = controller_manager
        self._timeout = timeout
        self._list = self.create_client(
            ListControllers, f"{self._cm}/list_controllers")
        self._load = self.create_client(
            LoadController, f"{self._cm}/load_controller")
        self._configure = self.create_client(
            ConfigureController, f"{self._cm}/configure_controller")
        self._switch = self.create_client(
            SwitchController, f"{self._cm}/switch_controller")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)

    # ── low level helper ────────────────────────────────────
    def _call(self, client, request, timeout: float):
        """Call a controller_manager service, returning None on timeout/error."""
        if not client.wait_for_service(timeout_sec=min(timeout, 30.0)):
            self.get_logger().warn(
                f"service {client.srv_name} not available within "
                f"{min(timeout, 30.0):.0f}s")
            return None
        future = client.call_async(request)
        # The node must be spun for the service response to be delivered.
        self._executor.spin_until_future_complete(future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().warn(f"service {client.srv_name} call timed out")
            return None
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"service {client.srv_name} raised: {exc}")
            return None

    def _states(self):
        """Return {controller_name: state} or None on failure."""
        res = self._call(self._list, ListControllers.Request(), self._timeout)
        if res is None:
            return None
        return {c.name: c.state for c in res.controller}

    # ── orchestration ───────────────────────────────────────
    def ensure(self, controllers, max_attempts: int = 5) -> bool:
        for attempt in range(1, max_attempts + 1):
            states = self._states()
            if states is None:
                self.get_logger().warn(
                    f"attempt {attempt}/{max_attempts}: controller manager not "
                    f"reachable yet")
                time.sleep(2.0)
                continue

            remaining = [c for c in controllers if states.get(c) != "active"]
            if not remaining:
                self.get_logger().info(f"✅ controllers active: {controllers}")
                return True

            self.get_logger().info(
                f"attempt {attempt}/{max_attempts}: activating {remaining} "
                f"(current: { {c: states.get(c) for c in controllers} })")

            to_activate = []
            for name in remaining:
                state = states.get(name)
                if state is None:
                    req = LoadController.Request()
                    req.name = name
                    res = self._call(self._load, req, self._timeout)
                    if res is None or not getattr(res, "ok", False):
                        # Could already be loaded under a race; re-check.
                        if (self._states() or {}).get(name) is None:
                            self.get_logger().warn(f"{name}: load failed")
                            continue
                    state = "loaded"
                if state == "loaded":
                    req = ConfigureController.Request()
                    req.name = name
                    res = self._call(self._configure, req, self._timeout)
                    if res is None or not getattr(res, "ok", False):
                        if (self._states() or {}).get(name) not in (
                                "configured", "active"):
                            self.get_logger().warn(f"{name}: configure failed")
                            continue
                to_activate.append(name)

            if to_activate:
                req = SwitchController.Request()
                req.activate_controllers = to_activate
                req.deactivate_controllers = []
                req.strictness = SwitchController.Request.BEST_EFFORT
                res = self._call(self._switch, req, self._timeout)
                if res is None or not getattr(res, "ok", False):
                    self.get_logger().warn(
                        f"switch_controller reported failure for {to_activate}")

            time.sleep(1.5)  # re-check actual state on next iteration

        states = self._states() or {}
        achieved = {c: states.get(c) for c in controllers}
        self.get_logger().error(
            f"❌ controllers not active after {max_attempts} attempts: "
            f"{achieved}")
        return all(v == "active" for v in achieved.values())


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("controllers", nargs="+")
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--max-attempts", type=int, default=5)
    args, _ = parser.parse_known_args(argv)

    rclpy.init()
    node = ControllerEnsurer(args.controller_manager, args.timeout)
    try:
        ok = node.ensure(args.controllers, max_attempts=args.max_attempts)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
