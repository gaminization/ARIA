#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Reachability Agent — LifecycleNode
Pre-checks if target poses are reachable before planning.
Suggests nearest reachable alternative when not.
═══════════════════════════════════════════════════════════════
"""
import math
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Point
from arm_interfaces.srv import CheckReachability, SolveIK
from arm_planner.state_bus import StateBus

# Workspace bounds from DH params (arm_length ≈ 0.145 + 0.115 = 0.26m)
WORKSPACE_RADIUS_MAX = 0.26    # meters from base
WORKSPACE_RADIUS_MIN = 0.05    # too close to base
WORKSPACE_Z_MIN = 0.02         # just above table
WORKSPACE_Z_MAX = 0.45         # arm fully up

# Cable zones (from cable_constraints.yaml)
CABLE_ZONES = [
    {'center': [0.0, -0.03, 0.035], 'half_size': [0.015, 0.015, 0.035]},
    {'center': [0.0, -0.02, 0.14],  'half_size': [0.01, 0.01, 0.07]},
    {'center': [0.0, -0.015, 0.28], 'half_size': [0.008, 0.008, 0.05]},
]

class ReachabilityAgent(LifecycleNode):
    """
    Pre-checks pose reachability.

    Checks:
      1. Within workspace bounds (DH params)
      2. IK has a solution (quick test)
      3. Cable zone clearance
      4. Suggests nearest alternative if not reachable
    """

    def __init__(self):
        super().__init__('reachability_agent')
        self.bus = StateBus(self)
        self.cb_group = ReentrantCallbackGroup()

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("ReachabilityAgent: CONFIGURING")
        self.create_service(CheckReachability, '/aria/reachability/check',
                            self._check_cb, callback_group=self.cb_group)
        self.ik_client = self.create_client(SolveIK, '/aria/ik/solve',
                                            callback_group=self.cb_group)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("ReachabilityAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _check_cb(self, request, response):
        """Service: /aria/reachability/check"""
        pos = request.target_pose.pose.position
        x, y, z = pos.x, pos.y, pos.z

        # 1. Workspace bounds
        r_xy = math.sqrt(x**2 + y**2)
        reasons = []

        if r_xy > WORKSPACE_RADIUS_MAX:
            reasons.append(f"XY radius {r_xy:.3f}m > max {WORKSPACE_RADIUS_MAX:.3f}m")
        if r_xy < WORKSPACE_RADIUS_MIN:
            reasons.append(f"XY radius {r_xy:.3f}m < min {WORKSPACE_RADIUS_MIN:.3f}m")
        if z < WORKSPACE_Z_MIN:
            reasons.append(f"Z={z:.3f}m < min {WORKSPACE_Z_MIN:.3f}m")
        if z > WORKSPACE_Z_MAX:
            reasons.append(f"Z={z:.3f}m > max {WORKSPACE_Z_MAX:.3f}m")

        # 2. Cable zone check
        for i, zone in enumerate(CABLE_ZONES):
            cx, cy, cz = zone['center']
            hx, hy, hz = zone['half_size']
            if (abs(x - cx) < hx + 0.005 and
                abs(y - cy) < hy + 0.005 and
                abs(z - cz) < hz + 0.005):
                reasons.append(f"Inside cable zone {i+1}")

        if reasons:
            response.reachable = False
            response.reason = '; '.join(reasons)
            response.alternative_pose = self._find_nearest(x, y, z)
            response.distance_to_boundary_m = self._dist_to_boundary(r_xy, z)
            self.bus.add_chain_of_thought(
                f"REACHABILITY: ({x:.3f},{y:.3f},{z:.3f}) NOT reachable: "
                f"{response.reason}")
            return response

        # 3. IK test
        if self.ik_client.wait_for_service(timeout_sec=1.0):
            ik_req = SolveIK.Request()
            ik_req.target_pose = request.target_pose
            ik_req.allow_fallback = False
            future = self.ik_client.call_async(ik_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                ik_resp = future.result()
                if ik_resp and not ik_resp.success:
                    response.reachable = False
                    response.reason = f"IK failed: {ik_resp.solver_used}"
                    response.alternative_pose = self._find_nearest(x, y, z)
                    self.bus.add_chain_of_thought(
                        f"REACHABILITY: IK failed for ({x:.3f},{y:.3f},{z:.3f})")
                    return response

        # All checks passed
        response.reachable = True
        response.reason = "Within workspace, IK solution exists"
        response.distance_to_boundary_m = self._dist_to_boundary(r_xy, z)
        self.bus.add_chain_of_thought(
            f"REACHABILITY: ({x:.3f},{y:.3f},{z:.3f}) ✓ reachable")
        return response

    def _dist_to_boundary(self, r_xy: float, z: float) -> float:
        """Distance to nearest workspace boundary."""
        return min(
            WORKSPACE_RADIUS_MAX - r_xy,
            r_xy - WORKSPACE_RADIUS_MIN,
            z - WORKSPACE_Z_MIN,
            WORKSPACE_Z_MAX - z,
        )

    def _find_nearest(self, x: float, y: float, z: float) -> PoseStamped:
        """Find nearest reachable pose by clamping to workspace."""
        r_xy = math.sqrt(x**2 + y**2) or 0.001
        # Clamp radius
        clamped_r = max(WORKSPACE_RADIUS_MIN + 0.01,
                       min(WORKSPACE_RADIUS_MAX - 0.01, r_xy))
        scale = clamped_r / r_xy
        nx, ny = x * scale, y * scale
        nz = max(WORKSPACE_Z_MIN + 0.01, min(WORKSPACE_Z_MAX - 0.01, z))

        alt = PoseStamped()
        alt.header.frame_id = 'base_link'
        alt.pose.position = Point(x=nx, y=ny, z=nz)
        alt.pose.orientation.w = 1.0
        return alt

def main(args=None):
    rclpy.init(args=args)
    node = ReachabilityAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
