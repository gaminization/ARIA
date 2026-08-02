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

        # Task state
        self.status = TaskStatus.IDLE
        self.current_task_id = ""
        self.action_index = 0

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

        if self.status != TaskStatus.IDLE:
            response.accepted = False
            response.task_id = self.current_task_id
            response.message = (
                f"Busy — currently {self.status.value}. "
                f"Use /aria/cancel to abort current task."
            )
            return response

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

        if self.status == TaskStatus.PLANNING:
            self._handle_planning(task)
        elif self.status == TaskStatus.EXECUTING:
            self._handle_executing(task)
        elif self.status == TaskStatus.PAUSED:
            pass  # Waiting for /aria/approve or /aria/reject
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
        elif task.task_status == TaskStatus.FAILED.value:
            self.status = TaskStatus.FAILED

    def _handle_executing(self, task: TaskState):
        """
        Execute current action from action_queue.
        Advance index when current action completes.
        """
        if self.action_index >= len(task.action_queue):
            # All actions complete
            self.status = TaskStatus.COMPLETE
            task.task_status = TaskStatus.COMPLETE.value
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
            # Trigger recovery
            self.status = TaskStatus.RECOVERY
            task.task_status = TaskStatus.RECOVERY.value
            self.bus.add_chain_of_thought(
                f"Action FAILED: {action.action_type}. "
                f"Initiating recovery..."
            )
            self.bus.publish_task(task)

        elif action.status == 'PENDING':
            # Check confidence before executing
            if action.confidence < CONFIDENCE_THRESHOLD:
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

            self.bus.add_chain_of_thought("User APPROVED. Resuming execution.")
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
        """Cancel the current task."""
        if self.status == TaskStatus.IDLE:
            response.success = False
            response.message = "No active task"
            return response

        task = self.bus.state.task
        if task:
            task.task_status = TaskStatus.FAILED.value
            self.bus.add_chain_of_thought("Task CANCELLED by user.")
            self.bus.publish_task(task)

        self.status = TaskStatus.IDLE
        self.current_task_id = ""
        response.success = True
        response.message = "Task cancelled"
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
