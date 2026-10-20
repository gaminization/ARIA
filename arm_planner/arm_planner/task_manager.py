#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Task Manager — Top-Level Orchestrator
Receives NL commands. Coordinates all agents via state bus.
Never executes directly. Full chain-of-thought reasoning.

Lifecycle:
  IDLE → PLANNING → EXECUTING → PAUSED/RECOVERY → COMPLETE/FAILED
═══════════════════════════════════════════════════════════════
"""
import threading
import time
import uuid
from enum import Enum
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger

from arm_interfaces.srv import SendCommand, SolveIK, CheckReachability
from arm_planner.msg import (
    TaskState, Action, FailureEvent, VisionState, MemoryState,
)
from arm_planner.state_bus import StateBus
from arm_planner.recovery_manager import (
    RecoveryManager, llm_recovery_plan, build_recovery_actions)
from arm_planner.failure_classifier import classify_failure


class TaskStatus(Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    PAUSED = "PAUSED"
    RECOVERY = "RECOVERY"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


# Confidence threshold — below this, ask user for approval
CONFIDENCE_THRESHOLD = 0.75
# Fully autonomous: auto-approve routine actions after a short grace period
# so the arm never stalls waiting for a human. The LLM re-plans on failure.
DEFAULT_AUTO_APPROVE_AFTER_S = 3.0


ACTION_TIMEOUT_S = 600.0  # composite skills (pick_all/sort_by_color) act on every object
MAX_TASK_RECOVERIES = 3


class TaskManager(LifecycleNode):
    """
    Top-level orchestrator.

    Pipeline for each command:
      1. PlanningAgent decomposes command → subgoals + actions
      2. For each action:
         a. ReachabilityAgent checks feasibility
         b. SafetyAgent validates
         c. SkillAgent executes
         d. If confidence < 0.75 → pause for user approval
         e. Monitor via ControlAgent feedback
      3. On completion: EvaluationAgent logs
      4. On failure: FailureClassifier → RecoveryManager
    """

    def __init__(self):
        super().__init__('task_manager')
        self.get_logger().info("═══ ARIA Task Manager initializing ═══")

        self.cb_group = ReentrantCallbackGroup()

        # State bus
        self.bus = StateBus(self)

        # Autonomous recovery manager
        self.recovery_manager = RecoveryManager(self.bus)

        # Task state
        self.status = TaskStatus.IDLE
        self.current_task_id = ""
        self.action_index = 0
        self._exec_since = {}
        self._paused_since = None
        self._approved_task_id = ''
        self.declare_parameter(
            'auto_approve_after_s', DEFAULT_AUTO_APPROVE_AFTER_S)
        self._recoveries = {}

        # ── Services ───────────────────────────────────────
        self.command_srv = self.create_service(
            SendCommand, '/aria/command',
            self._command_cb,
            callback_group=self.cb_group,
        )
        self.approve_srv = self.create_service(
            Trigger, '/aria/approve',
            self._approve_cb,
            callback_group=self.cb_group,
        )
        self.reject_srv = self.create_service(
            Trigger, '/aria/reject',
            self._reject_cb,
            callback_group=self.cb_group,
        )
        self.cancel_srv = self.create_service(
            Trigger, '/aria/cancel',
            self._cancel_cb,
            callback_group=self.cb_group,
        )

        # ── Service Clients ────────────────────────────────
        self.ik_client = self.create_client(
            SolveIK, '/aria/ik/solve',
            callback_group=self.cb_group)
        self.reachability_client = self.create_client(
            CheckReachability, '/aria/reachability/check',
            callback_group=self.cb_group)

        # ── State change listener ──────────────────────────
        self.bus.on_change('task', self._on_task_state_changed)

        # ── Heartbeat timer ────────────────────────────────
        self.create_timer(0.1, self._tick)  # 10Hz orchestration loop

        self.get_logger().info("Task Manager ready — awaiting commands")

    # ═══════════════════════════════════════════════════════
    # Lifecycle callbacks
    # ═══════════════════════════════════════════════════════
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Task Manager: CONFIGURING")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Task Manager: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Task Manager: DEACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    # ═══════════════════════════════════════════════════════
    # Command reception
    # ═══════════════════════════════════════════════════════
    def _command_cb(self, request, response):
        """
        Service: /aria/command

        Receives a natural language command from the user.
        Begins the planning → execution pipeline.
        """
        command = request.command.strip()

        if not command:
            response.accepted = False
            response.message = "Empty command"
            return response

        # Auto-reset any finished or stuck tasks so personal commands are immediately accepted
        if self.status != TaskStatus.IDLE:
            if self.status in (TaskStatus.COMPLETE, TaskStatus.FAILED):
                self.get_logger().info(f"Clearing completed/failed task ({self.status.value}) for new command.")
                self.status = TaskStatus.IDLE
            else:
                self.get_logger().info(
                    f"Overriding active task ({self.status.value}) with new user command: '{command}'")
                self.status = TaskStatus.IDLE

        # Generate task ID
        task_id = f"task_{uuid.uuid4().hex[:8]}"
        self.current_task_id = task_id

        self.get_logger().info(
            f"\n{'═' * 60}\n"
            f" Command: \"{command}\"\n"
            f" Task ID: {task_id}\n"
            f"{'═' * 60}"
        )

        # Initialize task state
        task = TaskState()
        task.current_command = command
        task.task_id = task_id
        task.task_status = TaskStatus.PLANNING.value
        task.confidence = 0.0
        task.chain_of_thought = [
            f"═══ New Command: \"{command}\" ═══",
            f"Task ID: {task_id}",
            "Status: PLANNING — decomposing command into subgoals...",
        ]
        self.bus.publish_task(task)

        self.status = TaskStatus.PLANNING
        self.action_index = 0

        # Planning happens in _tick via state bus
        # PlanningAgent listens for PLANNING status and decomposes

        response.accepted = True
        response.task_id = task_id
        response.message = f"Command accepted. Planning..."
        return response

    # ═══════════════════════════════════════════════════════
    # 10Hz orchestration loop
    # ═══════════════════════════════════════════════════════
    def _tick(self):
        """
        Main orchestration tick at 10Hz.

        State machine:
          PLANNING  → check if PlanningAgent has finished decomposition
          EXECUTING → execute current action, advance queue
          PAUSED    → wait for user approval
          RECOVERY  → wait for RecoveryManager to finish
          COMPLETE  → log results, go IDLE
          FAILED    → log failure, go IDLE
        """
        task = self.bus.state.task
        if task is None:
            return
        if self.status != TaskStatus.PAUSED:
            self._paused_since = None

        if self.status == TaskStatus.PLANNING:
            self._handle_planning(task)
        elif self.status == TaskStatus.EXECUTING:
            self._handle_executing(task)
        elif self.status == TaskStatus.PAUSED:
            # Low-confidence actions require an explicit operator decision by default.
            # Automatic approval is available only when explicitly configured.
            grace = float(self.get_parameter('auto_approve_after_s').value)
            if self._paused_since is None:
                self._paused_since = time.time()
            elif grace > 0 and time.time() - self._paused_since > grace:
                self.bus.add_chain_of_thought(
                    f"AUTONOMY: no operator response in {grace:.0f}s — auto-approving and proceeding.")
                self._paused_since = None
                self._approve_cb(None, Trigger.Response())
            return
        elif self.status == TaskStatus.RECOVERY:
            self._handle_recovery(task)
        elif self.status == TaskStatus.COMPLETE:
            self._handle_complete(task)
        elif self.status == TaskStatus.FAILED:
            self._handle_failed(task)

    def _handle_planning(self, task: TaskState):
        """
        Wait for PlanningAgent to fill action_queue.
        PlanningAgent sets task_status to EXECUTING when ready.
        """
        if task.task_status == TaskStatus.EXECUTING.value:
            self.status = TaskStatus.EXECUTING
            self.action_index = 0
            self.bus.add_chain_of_thought(
                f"Planning complete. {len(task.action_queue)} actions queued."
            )
        elif task.task_status == TaskStatus.PAUSED.value:
            self.status = TaskStatus.PAUSED
            self.action_index = 0
            self.bus.add_chain_of_thought(
                f"Plan ready ({len(task.action_queue)} actions) but needs operator approval.")
        elif task.task_status == TaskStatus.FAILED.value:
            self.status = TaskStatus.FAILED

    def _handle_executing(self, task: TaskState):
        """
        Execute current action from action_queue.
        Advance index when current action completes.
        """
        if self.action_index >= len(task.action_queue):
            # Check if any action in queue failed to achieve physical success
            any_failed = any(a.status == 'FAILED' for a in task.action_queue)
            if any_failed:
                self.status = TaskStatus.FAILED
                task.task_status = TaskStatus.FAILED.value
                self.bus.add_chain_of_thought("Task FAILED: One or more subgoals failed physical execution.")
            else:
                self.status = TaskStatus.COMPLETE
                task.task_status = TaskStatus.COMPLETE.value
                self.bus.add_chain_of_thought(f"Task COMPLETE: Successfully accomplished '{task.current_command}'.")
            self.bus.publish_task(task)
            return

        action = task.action_queue[self.action_index]

        if action.status == 'COMPLETE':
            # Advance to next action
            self.action_index += 1
            self.bus.add_chain_of_thought(
                f"Action {self.action_index} complete: {action.action_type}"
            )

        elif action.status == 'FAILED':
            self.status = TaskStatus.RECOVERY
            task.task_status = TaskStatus.RECOVERY.value
            self.bus.add_chain_of_thought(
                f"Action FAILED: {action.action_type}. Asking the LLM to diagnose and recover...")
            self.bus.publish_task(task)
            threading.Thread(
                target=self._recover_worker, args=(task, self.action_index), daemon=True).start()

        elif action.status == 'EXECUTING':
            since = self._exec_since.setdefault(
                (task.task_id, self.action_index), time.time())
            if time.time() - since > ACTION_TIMEOUT_S:
                action.status = 'FAILED'
                task.action_queue[self.action_index] = action
                self.bus.add_chain_of_thought(
                    f"WATCHDOG: '{action.action_type}' made no progress for "
                    f"{ACTION_TIMEOUT_S:.0f}s — marking FAILED so recovery can take over.")
                self.bus.publish_task(task)

        elif action.status == 'PENDING':
            # Check confidence before executing
            if (action.confidence < CONFIDENCE_THRESHOLD
                    and task.task_id != self._approved_task_id):
                self.status = TaskStatus.PAUSED
                task.task_status = TaskStatus.PAUSED.value
                task.awaiting_user_approval = True
                self.bus.add_chain_of_thought(
                    f"CONFIDENCE LOW ({action.confidence:.2f} < "
                    f"{CONFIDENCE_THRESHOLD}). "
                    f"Pausing for user approval."
                )
                self.bus.publish_task(task)
            else:
                # Mark as executing — SkillAgent picks it up
                action.status = 'EXECUTING'
                task.action_queue[self.action_index] = action
                self.bus.add_chain_of_thought(
                    f"EXECUTING: {action.action_type}({action.target_object}) "
                    f"[confidence={action.confidence:.2f}]"
                )
                self.bus.publish_task(task)

    def _recover_worker(self, task: TaskState, idx: int):
        """LLM-driven recovery (runs off the executor thread; the LLM call is slow)."""
        try:
            action = task.action_queue[idx]
            ctx = {
                'error_source': action.action_type,
                'action_type': action.action_type,
                'target_object': action.target_object,
                'detection_confidence': action.confidence,
            }
            self.bus.log_failure(classify_failure(ctx))
            n = self._recoveries.get(task.task_id, 0)
            plan = None
            new_actions = None
            if n < MAX_TASK_RECOVERIES:
                self._recoveries[task.task_id] = n + 1
                plan = llm_recovery_plan(task, idx, n + 1)
                if plan:
                    self.bus.add_chain_of_thought(
                        f"RECOVERY (LLM): diagnosis — {plan.get('diagnosis', '?')}")
                    if plan.get('reasoning'):
                        self.bus.add_chain_of_thought(f"RECOVERY (LLM): {plan['reasoning']}")
                new_actions = build_recovery_actions(task, idx, plan)
            if not new_actions:
                self.status = TaskStatus.FAILED
                task.task_status = TaskStatus.FAILED.value
                self.bus.add_chain_of_thought(
                    f"RECOVERY: no viable recovery for {action.action_type}. Mission halted.")
                self.bus.publish_task(task)
                return
            cur = self.bus.state.task
            if cur is not None and cur.task_id == task.task_id:
                task = cur
            queue = list(task.action_queue)
            queue[idx].status = 'RECOVERED'
            queue[idx + 1:idx + 1] = new_actions
            task.action_queue = queue
            self.action_index = idx + 1
            self.status = TaskStatus.EXECUTING
            task.task_status = TaskStatus.EXECUTING.value
            self.bus.add_chain_of_thought(
                "RECOVERY: inserted " + ", ".join(a.action_type for a in new_actions) +
                f" (attempt {self._recoveries[task.task_id]}/{MAX_TASK_RECOVERIES})")
            self.bus.publish_task(task)
        except Exception as e:
            self.get_logger().error(f"Recovery worker error: {e}")
            self.status = TaskStatus.FAILED
            task.task_status = TaskStatus.FAILED.value
            self.bus.publish_task(task)

    def _handle_recovery(self, task: TaskState):
        """Wait for RecoveryManager to finish."""
        # RecoveryManager sets task_status back to EXECUTING or FAILED
        if task.task_status == TaskStatus.EXECUTING.value:
            self.status = TaskStatus.EXECUTING
            self.bus.add_chain_of_thought("Recovery successful. Resuming.")
        elif task.task_status == TaskStatus.FAILED.value:
            self.status = TaskStatus.FAILED

    def _handle_complete(self, task: TaskState):
        """Log completion and return to IDLE."""
        task.task_status = TaskStatus.COMPLETE.value
        self.bus.add_chain_of_thought(
            f"{'═' * 60}\n"
            f"Task COMPLETE: {task.current_command}\n"
            f"{'═' * 60}"
        )
        self.bus.publish_task(task)
        self.status = TaskStatus.IDLE
        self.current_task_id = ""

    def _handle_failed(self, task: TaskState):
        """Log failure and return to IDLE."""
        task.task_status = TaskStatus.FAILED.value
        self.bus.add_chain_of_thought(
            f"Task FAILED: {task.current_command}"
        )
        self.bus.publish_task(task)
        self.status = TaskStatus.IDLE
        self.current_task_id = ""

    # ═══════════════════════════════════════════════════════
    # Approval / Rejection / Cancel
    # ═══════════════════════════════════════════════════════
    def _approve_cb(self, request, response):
        """User approves the paused action."""
        if self.status != TaskStatus.PAUSED:
            response.success = False
            response.message = "Not currently paused"
            return response

        task = self.bus.state.task
        if task:
            task.awaiting_user_approval = False
            task.task_status = TaskStatus.EXECUTING.value

            # Force-set action to EXECUTING
            if self.action_index < len(task.action_queue):
                task.action_queue[self.action_index].status = 'EXECUTING'

            self._approved_task_id = task.task_id
            self.bus.add_chain_of_thought("APPROVED. Resuming execution.")
            self.bus.publish_task(task)

        self.status = TaskStatus.EXECUTING
        response.success = True
        response.message = "Approved — resuming"
        return response

    def _reject_cb(self, request, response):
        """User rejects the paused action."""
        if self.status != TaskStatus.PAUSED:
            response.success = False
            response.message = "Not currently paused"
            return response

        task = self.bus.state.task
        if task:
            task.awaiting_user_approval = False
            task.task_status = TaskStatus.FAILED.value
            self.bus.add_chain_of_thought(
                "User REJECTED action. Task cancelled."
            )
            self.bus.publish_task(task)

        self.status = TaskStatus.FAILED
        response.success = True
        response.message = "Rejected — task cancelled"
        return response

    def _cancel_cb(self, request, response):
        """Cancel the current task and reset task manager to IDLE."""
        task = self.bus.state.task
        if task:
            task.task_status = TaskStatus.IDLE.value
            task.awaiting_user_approval = False
            task.action_queue = []
            task.subgoals = []
            self.bus.add_chain_of_thought("Task RESET / CANCELLED by user.")
            self.bus.publish_task(task)

        self.status = TaskStatus.IDLE
        self.current_task_id = ""
        self.action_index = 0
        response.success = True
        response.message = "Task reset to IDLE"
        self.get_logger().info("Task state cleanly reset to IDLE via /aria/cancel")
        return response

    # ═══════════════════════════════════════════════════════
    def _on_task_state_changed(self, msg: TaskState):
        """React to external task state changes (from agents)."""
        # Sync status from external updates
        if msg.task_id == self.current_task_id:
            try:
                new_status = TaskStatus(msg.task_status)
                if new_status != self.status:
                    self.status = new_status
            except ValueError:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
