#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Cable Scene Updater
Reads current joint angles and updates cable obstacle poses
in the MoveIt2 planning scene at 10Hz.
Cables move with the arm — this is cable-aware planning.
═══════════════════════════════════════════════════════════════
"""
import math
import os

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from std_msgs.msg import Header


class CableSceneUpdater(Node):
    """Updates cable collision objects in the MoveIt2 planning scene."""

    JOINT_NAMES = [
        "waist_joint", "shoulder_joint", "elbow_joint",
        "wrist_pitch_joint", "wrist_roll_joint"
    ]

    def __init__(self):
        super().__init__('cable_scene_updater')
        self.get_logger().info("═══ Cable Scene Updater starting ═══")

        # Load cable configuration
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg = get_package_share_directory('arm_moveit_config')
                config_path = os.path.join(pkg, 'config', 'cable_constraints.yaml')
            except Exception:
                config_path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'config', 'cable_constraints.yaml'
                )

        self.cable_zones = []
        self.safety_clearance = 0.005
        update_rate = 10.0

        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            self.cable_zones = config.get('cable_zones', [])
            self.safety_clearance = config.get('safety_clearance_m', 0.005)
            update_rate = config.get('update_rate_hz', 10.0)
            self.get_logger().info(
                f"Loaded {len(self.cable_zones)} cable zones from {config_path}"
            )
        else:
            self.get_logger().warn(f"Cable config not found: {config_path}")

        # Current joint positions
        self.current_joints = {name: 0.0 for name in self.JOINT_NAMES}

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10
        )

        # Publisher for planning scene updates
        self.scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10
        )

        # Timer at configured rate
        self.timer = self.create_timer(1.0 / update_rate, self._update_scene)

        self.get_logger().info(
            f"Cable scene updater ready, {len(self.cable_zones)} zones, "
            f"{update_rate}Hz update"
        )

    def _joint_state_cb(self, msg: JointState):
        """Update current joint positions."""
        for i, name in enumerate(msg.name):
            if name in self.current_joints:
                self.current_joints[name] = msg.position[i]

    def _get_link_transform(self, parent_link: str) -> np.ndarray:
        """
        Get the current world-frame pose of a link using FK.

        Uses tf2 lookups when available, otherwise computes from
        joint angles using simple kinematic chain.
        """
        # Simplified: compute directly from joint angles
        # Full implementation would use tf2 lookups
        joints = [self.current_joints.get(n, 0.0) for n in self.JOINT_NAMES]

        # Base height
        D_BASE = 0.105
        L_UPPER = 0.145
        L_FORE = 0.115
        L_WRIST = 0.055

        theta1, theta2, theta3, theta4, theta5 = joints

        # Base position
        T = np.eye(4)
        T[2, 3] = D_BASE

        if parent_link == 'upper_arm_link':
            # Position at shoulder, oriented by theta1, theta2
            c1, s1 = math.cos(theta1), math.sin(theta1)
            c2, s2 = math.cos(theta2), math.sin(theta2)

            T[0, 3] = L_UPPER / 2 * c2 * c1
            T[1, 3] = L_UPPER / 2 * c2 * s1
            T[2, 3] = D_BASE + L_UPPER / 2 * s2

        elif parent_link == 'forearm_link':
            c1, s1 = math.cos(theta1), math.sin(theta1)
            c23 = math.cos(theta2 + theta3)
            s23 = math.sin(theta2 + theta3)

            # End of upper arm + half of forearm
            x_upper = L_UPPER * math.cos(theta2)
            z_upper = L_UPPER * math.sin(theta2)
            x_fore = L_FORE / 2 * c23
            z_fore = L_FORE / 2 * s23

            T[0, 3] = (x_upper + x_fore) * c1
            T[1, 3] = (x_upper + x_fore) * s1
            T[2, 3] = D_BASE + z_upper + z_fore

        elif parent_link == 'wrist_link':
            c1, s1 = math.cos(theta1), math.sin(theta1)
            c234 = math.cos(theta2 + theta3 + theta4)
            s234 = math.sin(theta2 + theta3 + theta4)

            x_upper = L_UPPER * math.cos(theta2)
            z_upper = L_UPPER * math.sin(theta2)
            x_fore = L_FORE * math.cos(theta2 + theta3)
            z_fore = L_FORE * math.sin(theta2 + theta3)
            x_wrist = L_WRIST / 2 * c234
            z_wrist = L_WRIST / 2 * s234

            T[0, 3] = (x_upper + x_fore + x_wrist) * c1
            T[1, 3] = (x_upper + x_fore + x_wrist) * s1
            T[2, 3] = D_BASE + z_upper + z_fore + z_wrist

        return T

    def _update_scene(self):
        """Update cable obstacle positions in planning scene."""
        if not self.cable_zones:
            return

        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        for zone in self.cable_zones:
            T = self._get_link_transform(zone['parent_link'])

            # Create collision object
            co = CollisionObject()
            co.header = Header()
            co.header.frame_id = 'world'
            co.header.stamp = self.get_clock().now().to_msg()
            co.id = zone['name']
            co.operation = CollisionObject.ADD

            # Box primitive with safety margin
            dims = zone['dimensions']
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [
                dims[0] + 2 * self.safety_clearance,
                dims[1] + 2 * self.safety_clearance,
                dims[2] + 2 * self.safety_clearance,
            ]
            co.primitives.append(box)

            # Pose from parent link transform + offset
            offset = zone.get('offset', [0, 0, 0])
            pose = Pose()
            pose.position.x = T[0, 3] + offset[0]
            pose.position.y = T[1, 3] + offset[1]
            pose.position.z = T[2, 3] + offset[2]
            pose.orientation.w = 1.0
            co.primitive_poses.append(pose)

            scene_msg.world.collision_objects.append(co)

        self.scene_pub.publish(scene_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CableSceneUpdater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
