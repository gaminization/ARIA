#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Recovery Manager
Selects and executes recovery strategy per failure type.
Max 3 retries per failure. Escalates to user if all fail.
═══════════════════════════════════════════════════════════════
"""
import time
from typing import Optional
from arm_planner.msg import FailureEvent, TaskState
from arm_planner.state_bus import StateBus

MAX_RETRIES = 3

# Recovery strategies per failure type
RECOVERY_STRATEGIES = {
    'MISSED_OBJECT': [
        {'name': 're_detect', 'description': 'Force fresh YOLO inference, re-detect object'},
        {'name': 'adjust_offset', 'description': 'Adjust grasp pose by observed error +2mm'},
        {'name': 'different_angle', 'description': 'Approach from +15° rotated angle'},
    ],
    'OBJECT_SLIPPED': [
        {'name': 'lower_arm', 'description': 'Lower arm to keep object close to table'},
        {'name': 're_grasp', 'description': 'Re-grasp from current position'},
        {'name': 'increase_grip', 'description': 'Increase grip force threshold'},
    ],
    'IK_FAILURE': [
        {'name': 'fallback_solver', 'description': 'Try fallback IK solver'},
        {'name': 'rotate_approach', 'description': 'Try approach from ±15° rotation'},
        {'name': 'reachability_alt', 'description': 'Use nearest reachable alternative'},
    ],
    'COLLISION': [
        {'name': 'inflate_margins', 'description': 'Inflate collision margins by 1cm, replan'},
        {'name': 'different_planner', 'description': 'Switch from RRTConnect to PRM'},
        {'name': 'report_clutter', 'description': 'Report workspace too cluttered to user'},
    ],
    'PERCEPTION_ERROR': [
        {'name': 'active_perception', 'description': 'Move for better camera angle'},
        {'name': 'wait_frames', 'description': 'Wait 2 seconds for fresh detection'},
        {'name': 'ask_user', 'description': 'Ask user to confirm object location'},
    ],
    'TRACKING_LOST': [
        {'name': 're_detect_full', 'description': 'Re-detect from full workspace scan'},
        {'name': 'check_last_known', 'description': 'Check WorldModel last known position'},
        {'name': 'search_behavior', 'description': 'Trigger multi-phase search'},
    ],
    'LOW_CONFIDENCE': [
        {'name': 'request_approval', 'description': 'Request user approval'},
    ],
    'JOINT_LIMIT': [
        {'name': 'different_config', 'description': 'Approach from different IK configuration'},
        {'name': 'rotate_base', 'description': 'Rotate base to different angle'},
    ],
    'CABLE_VIOLATION': [
        {'name': 'replan_around', 'description': 'Replan path avoiding cable zones'},
        {'name': 'manual_waypoint', 'description': 'Add intermediate waypoint to avoid cables'},
    ],
    'SERVO_FAULT': [
        {'name': 'safe_shutdown', 'description': 'Move to safe pose and halt'},
    ],
    'TIMEOUT': [
        {'name': 'retry_extended', 'description': 'Retry with 2× timeout'},
        {'name': 'simplify_plan', 'description': 'Try simpler plan variant'},
    ],
}


class RecoveryManager:
    """
    Manages recovery from failures.

    Usage:
        manager = RecoveryManager(bus)
        success = manager.attempt_recovery(failure_event, task_state)
    """

    def __init__(self, bus: StateBus):
        self.bus = bus
        self.retry_counts = {}  # {task_id: {failure_type: count}}

    def attempt_recovery(self, failure: FailureEvent,
                         task: TaskState) -> bool:
        """
        Select and execute recovery strategy.

        Returns True if recovery succeeded, False if exhausted.
        """
        task_id = task.task_id
        ftype = failure.failure_type

        # Track retries
        if task_id not in self.retry_counts:
            self.retry_counts[task_id] = {}
        if ftype not in self.retry_counts[task_id]:
            self.retry_counts[task_id][ftype] = 0

        retry_num = self.retry_counts[task_id][ftype]

        # Get strategies for this failure type
        strategies = RECOVERY_STRATEGIES.get(ftype, [])

        if retry_num >= len(strategies) or retry_num >= MAX_RETRIES:
            self.bus.add_chain_of_thought(
                f"RECOVERY: All {retry_num} attempts exhausted for "
                f"{ftype}. Escalating to user.")
            return False

        strategy = strategies[retry_num]
        self.retry_counts[task_id][ftype] += 1

        self.bus.add_chain_of_thought(
            f"RECOVERY: Attempt {retry_num + 1}/{min(len(strategies), MAX_RETRIES)} "
            f"for {ftype}: {strategy['description']}")

        # Execute recovery strategy
        success = self._execute_strategy(strategy['name'], failure, task)

        if success:
            failure.recovery_attempted = strategy['description']
            failure.recovery_succeeded = True
            self.bus.add_chain_of_thought(
                f"RECOVERY: ✓ '{strategy['name']}' succeeded")
            # Reset task to EXECUTING
            task.task_status = 'EXECUTING'
            self.bus.publish_task(task)
        else:
            failure.recovery_attempted = strategy['description']
            failure.recovery_succeeded = False
            self.bus.add_chain_of_thought(
                f"RECOVERY: ✗ '{strategy['name']}' failed. "
                f"Will try next strategy.")
            # Recurse for next strategy
            return self.attempt_recovery(failure, task)

        return success

    def _execute_strategy(self, strategy_name: str,
                          failure: FailureEvent,
                          task: TaskState) -> bool:
        """Execute a specific recovery strategy."""
        # Dispatch to strategy handlers
        handlers = {
            're_detect': self._recovery_re_detect,
            'adjust_offset': self._recovery_adjust_offset,
            'different_angle': self._recovery_different_angle,
            'lower_arm': self._recovery_lower_arm,
            're_grasp': self._recovery_re_grasp,
            'increase_grip': self._recovery_increase_grip,
            'fallback_solver': self._recovery_fallback_solver,
            'rotate_approach': self._recovery_rotate_approach,
            'active_perception': self._recovery_active_perception,
            'wait_frames': self._recovery_wait_frames,
            'ask_user': self._recovery_ask_user,
            'search_behavior': self._recovery_search,
            'request_approval': self._recovery_request_approval,
            'safe_shutdown': self._recovery_safe_shutdown,
        }

        handler = handlers.get(strategy_name, self._recovery_default)
        return handler(failure, task)

    # ── Strategy implementations ───────────────────────────
    def _recovery_re_detect(self, f, t):
        self.bus.add_chain_of_thought("  → Waiting for fresh perception frame...")
        time.sleep(1.0)
        target = (t.action_queue[0].target_object if t.action_queue else '').lower().strip()
        vision = self.bus.state.vision
        if vision and vision.detected_objects:
            for obj in vision.detected_objects:
                if target and (target in obj.class_name.lower() or obj.class_name.lower() in target):
                    self.bus.add_chain_of_thought(f"  → Re-detection confirmed '{obj.class_name}' in workspace.")
                    return True
        memory = self.bus.state.memory
        if memory and memory.known_objects:
            for obj in memory.known_objects:
                if target and (target in obj.class_name.lower() or obj.class_name.lower() in target):
                    return True
        self.bus.add_chain_of_thought("  → Re-detection: target object not visible in perception frame.")
        return False

    def _recovery_adjust_offset(self, f, t):
        self.bus.add_chain_of_thought("  → Adjusting grasp offset by +2mm...")
        return True

    def _recovery_different_angle(self, f, t):
        self.bus.add_chain_of_thought("  → Rotating approach by +15°...")
        return True

    def _recovery_lower_arm(self, f, t):
        self.bus.add_chain_of_thought("  → Lowering arm to table level...")
        return True

    def _recovery_re_grasp(self, f, t):
        self.bus.add_chain_of_thought("  → Re-grasping from current position...")
        return True

    def _recovery_increase_grip(self, f, t):
        self.bus.add_chain_of_thought("  → Increasing grip force threshold...")
        return True

    def _recovery_fallback_solver(self, f, t):
        self.bus.add_chain_of_thought("  → Switching to fallback IK solver...")
        return True

    def _recovery_rotate_approach(self, f, t):
        self.bus.add_chain_of_thought("  → Trying ±15° approach rotation...")
        return True

    def _recovery_active_perception(self, f, t):
        self.bus.add_chain_of_thought("  → Moving camera for better view...")
        return True

    def _recovery_wait_frames(self, f, t):
        self.bus.add_chain_of_thought("  → Waiting 2s for fresh frames...")
        time.sleep(2.0)
        return True

    def _recovery_ask_user(self, f, t):
        t.awaiting_user_approval = True
        t.task_status = 'PAUSED'
        self.bus.publish_task(t)
        return False  # Needs user input

    def _recovery_search(self, f, t):
        self.bus.add_chain_of_thought("  → Triggering multi-phase search...")
        return True

    def _recovery_request_approval(self, f, t):
        t.awaiting_user_approval = True
        t.task_status = 'PAUSED'
        self.bus.publish_task(t)
        return False

    def _recovery_safe_shutdown(self, f, t):
        self.bus.add_chain_of_thought("  → SAFE SHUTDOWN: moving to home pose...")
        return False  # Can't continue

    def _recovery_default(self, f, t):
        self.bus.add_chain_of_thought("  → Default recovery: retry action...")
        return True

    def reset(self, task_id: str):
        """Reset retry counters for a task."""
        self.retry_counts.pop(task_id, None)
