#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Affordance Agent — LifecycleNode
Object affordance database + Bayesian learning from grasp outcomes.
═══════════════════════════════════════════════════════════════
"""
import os, yaml, math
from typing import Dict
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from geometry_msgs.msg import PoseStamped, Quaternion
from arm_interfaces.srv import GetAffordanceGrasp
from arm_planner.state_bus import StateBus

# Pre-loaded affordance database
DEFAULT_AFFORDANCES: Dict[str, dict] = {
    'cup':         {'grasp': 'handle', 'avoid': ['rim'], 'approach': 'horizontal', 'success_rate': 0.8},
    'bottle':      {'grasp': 'neck', 'avoid': ['cap'], 'approach': 'top_down', 'success_rate': 0.85},
    'screwdriver': {'grasp': 'shaft', 'avoid': [], 'approach': 'horizontal', 'success_rate': 0.7},
    'paintbrush':  {'grasp': 'handle', 'avoid': ['bristles'], 'approach': 'horizontal', 'success_rate': 0.75},
    'cube':        {'grasp': 'top', 'avoid': [], 'approach': 'top_down', 'success_rate': 0.9},
    'cylinder':    {'grasp': 'body', 'avoid': [], 'approach': 'side', 'success_rate': 0.8},
    'scissors':    {'grasp': 'handle_holes', 'avoid': ['blade'], 'approach': 'top', 'success_rate': 0.65},
    'ball':        {'grasp': 'top', 'avoid': [], 'approach': 'top_down', 'success_rate': 0.7},
    'box':         {'grasp': 'top', 'avoid': [], 'approach': 'top_down', 'success_rate': 0.85},
    'pen':         {'grasp': 'body', 'avoid': [], 'approach': 'side', 'success_rate': 0.7},
}

APPROACH_QUATERNIONS = {
    'top_down':   Quaternion(x=0.0, y=-0.7071, z=0.0, w=0.7071),  # pitch -90°
    'horizontal': Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    'side':       Quaternion(x=0.0, y=0.0, z=0.7071, w=0.7071),   # yaw 90°
    'top':        Quaternion(x=0.0, y=-0.7071, z=0.0, w=0.7071),
}

class AffordanceAgent(LifecycleNode):
    """Affordance-based grasp strategy selection with Bayesian learning."""

    def __init__(self):
        super().__init__('affordance_agent')
        self.bus = StateBus(self)
        self.affordances = dict(DEFAULT_AFFORDANCES)
        self.grasp_history: list = []

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("AffordanceAgent: CONFIGURING")
        self.declare_parameter('db_path', '')
        db_path = self.get_parameter('db_path').value
        if not db_path:
            db_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'data', 'affordance_db.yaml')
        self.db_path = db_path
        self._load_db()
        self.create_service(GetAffordanceGrasp, '/aria/affordance/get_grasp', self._get_grasp_cb)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("AffordanceAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._save_db()
        return TransitionCallbackReturn.SUCCESS

    def _load_db(self):
        if os.path.exists(self.db_path):
            try:
                with open(self.db_path, 'r') as f:
                    loaded = yaml.safe_load(f)
                if loaded:
                    self.affordances.update(loaded)
                    self.get_logger().info(f"Loaded {len(self.affordances)} affordances")
            except Exception as e:
                self.get_logger().warn(f"Could not load affordance DB: {e}")

    def _save_db(self):
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        with open(self.db_path, 'w') as f:
            yaml.dump(self.affordances, f, default_flow_style=False)
        self.get_logger().info(f"Saved affordance DB to {self.db_path}")

    def update_from_outcome(self, object_class: str, grasp_region: str, success: bool):
        """Bayesian update of affordance success rate after a grasp attempt."""
        key = object_class.lower()
        if key not in self.affordances:
            self.affordances[key] = {
                'grasp': grasp_region, 'avoid': [], 'approach': 'top_down', 'success_rate': 0.5}
        aff = self.affordances[key]
        # Bayesian update: weighted running average
        alpha = 0.1  # learning rate
        old_rate = aff.get('success_rate', 0.5)
        aff['success_rate'] = old_rate * (1 - alpha) + (1.0 if success else 0.0) * alpha
        self.grasp_history.append({
            'class': key, 'region': grasp_region, 'success': success,
            'new_rate': aff['success_rate']})
        self.bus.add_chain_of_thought(
            f"AFFORDANCE: Updated {key} success rate: "
            f"{old_rate:.2f} → {aff['success_rate']:.2f}")
        self._save_db()

    def _get_grasp_cb(self, request, response):
        """Service: /aria/affordance/get_grasp"""
        cls = request.object_class.lower()
        aff = self.affordances.get(cls, DEFAULT_AFFORDANCES.get('cube'))
        approach = aff.get('approach', 'top_down')
        region = aff.get('grasp', 'top')
        conf = aff.get('success_rate', 0.5)
        avoid = aff.get('avoid', [])

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.pose.position = request.object_pose.pose.position
        grasp_pose.pose.orientation = APPROACH_QUATERNIONS.get(
            approach, APPROACH_QUATERNIONS['top_down'])

        # Adjust Z for approach type
        if approach in ('top_down', 'top'):
            grasp_pose.pose.position.z += 0.02  # 2cm above center
        elif approach == 'horizontal':
            pass  # Same height

        reasoning = (
            f"Object class '{cls}': grasp region='{region}', "
            f"approach='{approach}', avoid={avoid}, "
            f"historical success rate={conf:.0%}")

        self.bus.add_chain_of_thought(f"AFFORDANCE: {reasoning}")

        response.success = True
        response.grasp_pose = grasp_pose
        response.confidence = float(conf)
        response.grasp_region = region
        response.approach_direction = approach
        response.reasoning = reasoning
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AffordanceAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
