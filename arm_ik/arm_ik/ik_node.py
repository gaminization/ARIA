#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Production IK Node
Uses benchmark-selected solver with automatic fallback.
Provides /aria/ik/solve and /aria/ik/fk_service.
═══════════════════════════════════════════════════════════════
"""
import math
import os
import time

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from arm_interfaces.srv import SolveIK, ComputeFK
from arm_ik.ik_solvers.aria_analytical_ik import (
    IKResult, SolverConfig, forward_kinematics,
    solve_from_pose_stamped, _rotation_matrix_to_quaternion,
)
from arm_ik.ik_solvers.ikpy_solver import solve_ikpy
from arm_ik.ik_solvers.robotics_toolbox_solver import solve_rtb
from arm_ik.ik_solvers.neural_ik_solver import solve_neural


class IKNode(Node):
    """
    Production IK node.
    Loads selected solver from benchmark config.
    Falls back to secondary solver on failure.
    """

    def __init__(self):
        super().__init__('ik_node')
        self.get_logger().info("═══ ARIA IK Production Node ═══")

        # Load solver selection from benchmark config
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg = get_package_share_directory('arm_ik')
                config_path = os.path.join(pkg, 'config', 'selected_solver.yaml')
            except Exception:
                config_path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'config', 'selected_solver.yaml'
                )

        self.primary_solver = 'analytical'
        self.fallback_solver = 'rtb_LM'

        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            self.primary_solver = config.get('primary_solver', 'analytical')
            self.fallback_solver = config.get('fallback_solver', 'rtb_LM')
            self.get_logger().info(
                f"Loaded config: primary={self.primary_solver}, "
                f"fallback={self.fallback_solver}"
            )
        else:
            self.get_logger().warn(
                f"Config not found at {config_path}, using defaults"
            )

        # Statistics
        self.solve_count = 0
        self.success_count = 0
        self.fallback_count = 0

        # Services
        self.create_service(SolveIK, '/aria/ik/solve', self._solve_cb)
        self.create_service(ComputeFK, '/aria/ik/fk_service', self._fk_cb)

        self.get_logger().info(
            f"IK Node ready: primary={self.primary_solver}, "
            f"fallback={self.fallback_solver}"
        )

    def _run_solver(self, solver_name: str,
                    target_pos: np.ndarray,
                    target_orient: np.ndarray,
                    config: SolverConfig) -> IKResult:
        """Run a specific solver by name."""
        if solver_name == 'analytical':
            return solve_from_pose_stamped(target_pos, target_orient, config)
        elif solver_name == 'ikpy':
            return solve_ikpy(target_pos, config=config)
        elif solver_name == 'rtb_LM':
            return solve_rtb(target_pos, method='LM', config=config)
        elif solver_name == 'rtb_GN':
            return solve_rtb(target_pos, method='GN', config=config)
        elif solver_name == 'neural':
            return solve_neural(target_pos, target_orient)
        else:
            return IKResult(success=False, message=f"Unknown: {solver_name}")

    def _solve_cb(self, request, response):
        """
        Service: /aria/ik/solve

        Logic:
          1. Try primary solver
          2. If fails AND allow_fallback: try fallback solver
          3. If still fails: return success=false
        """
        self.solve_count += 1

        # Extract target from PoseStamped
        pose = request.target_pose.pose
        target_pos = np.array([
            pose.position.x, pose.position.y, pose.position.z
        ])
        target_orient = np.array([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        ])

        # Current joints for warm-starting
        current = None
        if len(request.current_joints) >= 5:
            current = np.array(request.current_joints[:5])

        config = SolverConfig(
            current_joints=current,
            tolerance_m=0.005,
        )

        # Step 1: Try primary solver
        result = self._run_solver(
            self.primary_solver, target_pos, target_orient, config
        )

        solver_used = self.primary_solver

        # Step 2: Fallback if needed
        if not result.success and request.allow_fallback:
            self.get_logger().info(
                f"Primary solver ({self.primary_solver}) failed, "
                f"trying fallback ({self.fallback_solver})"
            )
            result = self._run_solver(
                self.fallback_solver, target_pos, target_orient, config
            )
            solver_used = self.fallback_solver
            if result.success:
                self.fallback_count += 1

        # Build response
        response.success = result.success
        response.joint_angles = result.joint_angles.tolist() if result.success else []
        response.solver_used = solver_used
        response.solve_time_ms = result.solve_time_ms
        response.position_error_mm = result.position_error_m * 1000 if result.success else -1.0
        response.message = result.message

        if result.success:
            self.success_count += 1

        # Log
        self.get_logger().info(
            f"IK solve #{self.solve_count}: "
            f"{'✅' if result.success else '❌'} "
            f"solver={solver_used}, "
            f"time={result.solve_time_ms:.2f}ms, "
            f"err={result.position_error_m*1000:.1f}mm "
            f"(total: {self.success_count}/{self.solve_count} success, "
            f"{self.fallback_count} fallbacks)"
        )

        return response

    def _fk_cb(self, request, response):
        """Service: /aria/ik/fk_service — compute FK from joint angles."""
        if len(request.joint_angles) < 5:
            response.success = False
            response.message = "Need at least 5 joint angles"
            return response

        joints = np.array(request.joint_angles[:5])
        T = forward_kinematics(joints)

        pos = T[:3, 3]
        quat = _rotation_matrix_to_quaternion(T[:3, :3])

        response.end_effector_pose = PoseStamped()
        response.end_effector_pose.header.frame_id = 'base_link'
        response.end_effector_pose.header.stamp = self.get_clock().now().to_msg()
        response.end_effector_pose.pose.position.x = float(pos[0])
        response.end_effector_pose.pose.position.y = float(pos[1])
        response.end_effector_pose.pose.position.z = float(pos[2])
        response.end_effector_pose.pose.orientation.x = float(quat[0])
        response.end_effector_pose.pose.orientation.y = float(quat[1])
        response.end_effector_pose.pose.orientation.z = float(quat[2])
        response.end_effector_pose.pose.orientation.w = float(quat[3])

        response.success = True
        response.message = f"FK computed: pos=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
