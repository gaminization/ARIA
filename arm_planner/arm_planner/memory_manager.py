#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Memory Manager
Manages all persistent memory systems:
  - WorldModel DB
  - Skill performance data
  - Calibration offsets
  - Task history
═══════════════════════════════════════════════════════════════
"""
import os
import yaml
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Int32

from arm_planner.world_model_db import WorldModelDB


class MemoryManager(Node):
    """
    Manages all persistent memory systems.

    On startup:  load world model, skill perf, calibration
    On shutdown: flush all to disk
    Periodic:    auto-save every 60 seconds

    Services:
      /aria/memory/reset_object  — remove object from model
      /aria/memory/export        — dump world model to YAML
      /aria/memory/import        — load scene from YAML
      /aria/memory/stats         — get memory statistics
    """

    def __init__(self):
        super().__init__('memory_manager')
        self.get_logger().info("═══ ARIA Memory Manager starting ═══")

        # Paths
        self.data_dir = self.declare_parameter(
            'data_dir', '').value
        if not self.data_dir:
            self.data_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'data')
        os.makedirs(self.data_dir, exist_ok=True)

        # ── World Model DB ─────────────────────────────────
        self.world_db = WorldModelDB(
            os.path.join(self.data_dir, 'world_model.db'))
        self.get_logger().info(
            f"World model loaded: {self.world_db.object_count()} objects")

        # ── Skill Performance ──────────────────────────────
        self.skill_perf_path = os.path.join(
            self.data_dir, 'skill_perf.yaml')
        self.skill_perf = self._load_yaml(self.skill_perf_path) or {}
        self.get_logger().info(
            f"Skill performance loaded: {len(self.skill_perf)} skills")

        # ── Calibration Offsets ────────────────────────────
        self.cal_path = os.path.join(
            os.path.dirname(self.data_dir),
            '..', 'arm_ik', 'config', 'calibration_offsets.yaml')
        self.calibration = self._load_yaml(self.cal_path) or {
            'joint_offsets_deg': [0.0] * 5,
            'link_corrections_mm': [0.0] * 4,
            'last_calibration': '',
            'operations_since_cal': 0,
        }

        # ── Services ───────────────────────────────────────
        self.create_service(
            Trigger, '/aria/memory/export', self._export_cb)
        self.create_service(
            Trigger, '/aria/memory/stats', self._stats_cb)

        # For reset_object: use a String service with object_id
        self.reset_sub = self.create_subscription(
            Int32, '/aria/memory/reset_object',
            self._reset_object_cb, 10)
        self.import_sub = self.create_subscription(
            String, '/aria/memory/import',
            self._import_cb, 10)

        # ── Auto-save timer ────────────────────────────────
        self.create_timer(60.0, self._auto_save)

        self.get_logger().info("Memory Manager ready")

    def _load_yaml(self, path: str) -> Optional[dict]:
        """Load a YAML file, return None if not found."""
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    return yaml.safe_load(f) or {}
            except Exception as e:
                self.get_logger().warn(f"Failed to load {path}: {e}")
        return None

    def _save_yaml(self, data: dict, path: str):
        """Save data to YAML."""
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    def _auto_save(self):
        """Periodic save of all data."""
        self._save_yaml(self.skill_perf, self.skill_perf_path)
        self._save_yaml(self.calibration, self.cal_path)

    def shutdown(self):
        """Flush all data to disk."""
        self.get_logger().info("Memory Manager: flushing to disk...")
        self._save_yaml(self.skill_perf, self.skill_perf_path)
        self._save_yaml(self.calibration, self.cal_path)
        self.get_logger().info("Memory Manager: all data saved")

    # ── Service Callbacks ──────────────────────────────────

    def _reset_object_cb(self, msg):
        """Remove object from world model."""
        obj_id = msg.data
        self.world_db.remove_object(obj_id)
        self.get_logger().info(f"Removed object {obj_id} from world model")

    def _export_cb(self, request, response):
        """Export world model to YAML."""
        export_path = os.path.join(self.data_dir, 'world_model_export.yaml')
        self.world_db.export_to_yaml(export_path)
        response.success = True
        response.message = f"Exported to {export_path}"
        self.get_logger().info(f"World model exported to {export_path}")
        return response

    def _import_cb(self, msg):
        """Import world model from YAML."""
        path = msg.data
        if os.path.exists(path):
            self.world_db.import_from_yaml(path)
            self.get_logger().info(
                f"Imported world model from {path}: "
                f"{self.world_db.object_count()} objects")
        else:
            self.get_logger().error(f"Import file not found: {path}")

    def _stats_cb(self, request, response):
        """Get memory statistics."""
        task_stats = self.world_db.get_task_stats()
        stats = (
            f"Objects: {self.world_db.object_count()} | "
            f"Skills tracked: {len(self.skill_perf)} | "
            f"Tasks: {task_stats['total_tasks']} "
            f"({task_stats['success_rate']:.0%} success) | "
            f"Ops since cal: {self.calibration.get('operations_since_cal', 0)}")
        response.success = True
        response.message = stats
        return response

    # ── Public API ─────────────────────────────────────────

    def log_skill_execution(self, skill_name: str, success: bool,
                            duration_s: float, details: str = ""):
        """Log a skill execution for performance tracking."""
        if skill_name not in self.skill_perf:
            self.skill_perf[skill_name] = {
                'attempts': 0, 'successes': 0,
                'total_duration': 0.0, 'failures': [],
            }
        entry = self.skill_perf[skill_name]
        entry['attempts'] += 1
        entry['total_duration'] += duration_s
        if success:
            entry['successes'] += 1
        else:
            entry['failures'].append({
                'time': time.time(), 'details': details})
            # Keep only last 20 failures
            entry['failures'] = entry['failures'][-20:]

    def increment_operations(self):
        """Increment calibration operations counter."""
        self.calibration['operations_since_cal'] = \
            self.calibration.get('operations_since_cal', 0) + 1


def main(args=None):
    rclpy.init(args=args)
    node = MemoryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
