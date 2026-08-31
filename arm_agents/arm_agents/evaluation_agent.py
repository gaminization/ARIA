#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Evaluation Agent — LifecycleNode
Performance logging. Metrics CSV. Trend analysis. Reports.
═══════════════════════════════════════════════════════════════
"""
import os, time, csv, math
from collections import deque
from typing import Dict
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_srvs.srv import Trigger
from arm_planner.msg import TaskState
from arm_planner.state_bus import StateBus

class EvaluationAgent(LifecycleNode):
    """
    Performance tracking for self-supervised improvement.

    Metrics tracked (→ metrics.csv):
      pick_success_rate, task_completion_rate, planning_time_ms,
      inference_time_ms, recovery_rate, ik_error_mm,
      localization_error_mm

    Reports: daily summary, trend analysis, failure breakdown.
    """

    def __init__(self):
        super().__init__('evaluation_agent')
        self.bus = StateBus(self)
        self.metrics: Dict[str, deque] = {
            'pick_success': deque(maxlen=100),
            'task_completion': deque(maxlen=100),
            'planning_time_ms': deque(maxlen=100),
            'inference_time_ms': deque(maxlen=100),
            'recovery_success': deque(maxlen=50),
            'ik_error_mm': deque(maxlen=100),
        }
        self.total_tasks = 0
        self.completed_tasks = 0
        self.failed_tasks = 0
        self.failure_breakdown: Dict[str, int] = {}

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("EvaluationAgent: CONFIGURING")
        self.declare_parameter('log_dir', '')
        ld = self.get_parameter('log_dir').value
        if not ld:
            ld = os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), '..', 'arm_planner', 'logs')
        self.log_dir = ld
        os.makedirs(self.log_dir, exist_ok=True)
        self.metrics_file = os.path.join(self.log_dir, 'metrics.csv')
        self._init_csv()
        self.bus.on_change('task', self._on_task)
        self.create_service(Trigger, '/aria/evaluation/report', self._report_cb)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("EvaluationAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _init_csv(self):
        if not os.path.exists(self.metrics_file):
            with open(self.metrics_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'task_id', 'command', 'status',
                    'n_actions', 'n_failures', 'confidence',
                    'pick_success_rate', 'task_completion_rate',
                ])

    def _on_task(self, msg: TaskState):
        """Log task completion/failure."""
        if msg.task_status in ('COMPLETE', 'FAILED'):
            self.total_tasks += 1
            if msg.task_status == 'COMPLETE':
                self.completed_tasks += 1
                self.metrics['task_completion'].append(1)
            else:
                self.failed_tasks += 1
                self.metrics['task_completion'].append(0)
                # Failure breakdown
                for fe in msg.failure_log:
                    ft = fe.failure_type
                    self.failure_breakdown[ft] = self.failure_breakdown.get(ft, 0) + 1

            # Log to CSV
            with open(self.metrics_file, 'a', newline='') as f:
                writer = csv.writer(f)
                tc_rate = self.completed_tasks / max(self.total_tasks, 1)
                ps_rate = np.mean(list(self.metrics['pick_success'])) if self.metrics['pick_success'] else 0.0
                writer.writerow([
                    time.time(), msg.task_id, msg.current_command,
                    msg.task_status, len(msg.action_queue),
                    len(msg.failure_log), msg.confidence,
                    f'{ps_rate:.3f}', f'{tc_rate:.3f}',
                ])

            self.bus.add_chain_of_thought(
                f"EVALUATION: Task {msg.task_status}. "
                f"Overall: {self.completed_tasks}/{self.total_tasks} "
                f"({self.completed_tasks/max(self.total_tasks,1):.0%})")

    def log_pick_outcome(self, success: bool):
        self.metrics['pick_success'].append(1 if success else 0)

    def _report_cb(self, request, response):
        """Generate performance summary."""
        tc_rate = self.completed_tasks / max(self.total_tasks, 1)
        ps = list(self.metrics['pick_success'])
        ps_rate = np.mean(ps) if ps else 0.0
        pi = list(self.metrics['planning_time_ms'])
        avg_plan = np.mean(pi) if pi else 0.0

        report_lines = [
            f"═══ ARIA Performance Report ═══",
            f"Total tasks: {self.total_tasks}",
            f"Completed: {self.completed_tasks} ({tc_rate:.0%})",
            f"Failed: {self.failed_tasks}",
            f"Pick success rate: {ps_rate:.0%} (last {len(ps)})",
            f"Avg planning time: {avg_plan:.1f}ms",
            f"Failure breakdown: {self.failure_breakdown}",
        ]

        # Trend analysis
        if len(ps) >= 20:
            first_half = np.mean(ps[:len(ps)//2])
            second_half = np.mean(ps[len(ps)//2:])
            trend = "improving ↑" if second_half > first_half else "degrading ↓"
            report_lines.append(f"Pick trend: {trend} ({first_half:.0%} → {second_half:.0%})")

        report = '\n'.join(report_lines)
        self.get_logger().info(report)
        response.success = True
        response.message = report
        return response

def main(args=None):
    rclpy.init(args=args)
    node = EvaluationAgent()
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
