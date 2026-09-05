#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Conveyor Workpiece Spawner Utility
# Spawns precision workpieces (Good or Defective) onto the infeed conveyor.
# Usage:
#   ros2 run arm_bringup conveyor_part_spawner.py --type good
#   ros2 run arm_bringup conveyor_part_spawner.py --type defect
# ═══════════════════════════════════════════════════════════════
import argparse
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity


class ConveyorPartSpawner(Node):
    """Dynamically spawn industrial workpieces onto the active conveyor."""

    def __init__(self):
        super().__init__("conveyor_part_spawner")
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")

    def spawn(self, part_type: str = "good", x: float = 0.20, y: float = 0.55, z: float = 0.645) -> bool:
        bringup_pkg = get_package_share_directory("arm_bringup")
        model_name = "workpiece_good" if part_type.lower() == "good" else "workpiece_defect"
        sdf_path = os.path.join(bringup_pkg, "models", model_name, "model.sdf")

        if not os.path.exists(sdf_path):
            self.get_logger().error(f"Model SDF not found: {sdf_path}")
            return False

        with open(sdf_path, "r") as f:
            xml_content = f.read()

        entity_name = f"{model_name}_{int(time.time() * 1000) % 100000}"

        self.get_logger().info(f"Connecting to Gazebo /spawn_entity service...")
        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Gazebo /spawn_entity service not available!")
            return False

        req = SpawnEntity.Request()
        req.name = entity_name
        req.xml = xml_content
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = float(z)

        self.get_logger().info(f"Spawning '{entity_name}' at x={x:.2f}, y={y:.2f}, z={z:.2f}...")
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        res = future.result()
        if res and res.success:
            self.get_logger().info(f"✅ Successfully spawned '{entity_name}' on infeed conveyor!")
            return True
        else:
            msg = res.status_message if res else "No response"
            self.get_logger().error(f"❌ Failed to spawn: {msg}")
            return False


def main(args=None):
    parser = argparse.ArgumentParser(description="Spawn workpiece onto ARIA infeed conveyor.")
    parser.add_argument("--type", choices=["good", "defect"], default="good", help="Type of workpiece to spawn")
    parser.add_argument("--y", type=float, default=0.55, help="Y position on conveyor (0.10 to 0.65)")

    # Separate ROS arguments from CLI arguments
    parsed_args, ros_args = parser.parse_known_args(args if args is not None else sys.argv[1:])

    rclpy.init(args=ros_args)
    spawner = ConveyorPartSpawner()
    try:
        spawner.spawn(part_type=parsed_args.type, y=parsed_args.y)
    finally:
        spawner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
