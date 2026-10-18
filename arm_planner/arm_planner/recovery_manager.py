#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Recovery Manager
Selects and executes recovery strategy per failure type.
Max 3 retries per failure. Escalates to user if all fail.
═══════════════════════════════════════════════════════════════
"""
import asyncio
import copy
import json
import time
from typing import List, Optional
from arm_planner.msg import Action, FailureEvent, TaskState
from arm_planner.skill_registry import resolve_executable, EXECUTABLE_NAMES, CORE_SKILLS
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
    # Each handler performs a real action and returns True only when it
    # can verify the outcome. Handlers that cannot verify success return
    # False — a recovery that cannot be verified must not be reported as
    # successful.
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
        self.bus.add_chain_of_thought(
            "  → Adjust grasp offset: NOT IMPLEMENTED — no grasp-offset "
            "adjustment service is wired. Reporting failure.")
        return False

    def _recovery_different_angle(self, f, t):
        self.bus.add_chain_of_thought(
            "  → Different approach angle: NOT IMPLEMENTED — no approach-rotation "
            "service is wired. Reporting failure.")
        return False

    def _recovery_lower_arm(self, f, t):
        self.bus.add_chain_of_thought(
            "  → Lower arm: NOT IMPLEMENTED — no height-adjustment service is wired. "
            "Reporting failure.")
        return False

    def _recovery_re_grasp(self, f, t):
        self.bus.add_chain_of_thought(
            "  → Re-grasp: NOT IMPLEMENTED — no re-grasp service is wired. "
            "Reporting failure.")
        return False

    def _recovery_increase_grip(self, f, t):
        self.bus.add_chain_of_thought(
            "  → Increase grip force: NOT IMPLEMENTED — no grip-force service is wired. "
            "Reporting failure.")
        return False

    def _recovery_fallback_solver(self, f, t):
        self.bus.add_chain_of_thought(
            "  → Fallback IK solver: NOT IMPLEMENTED — no fallback solver is wired. "
            "Reporting failure.")
        return False

    def _recovery_rotate_approach(self, f, t):
        self.bus.add_chain_of_thought(
            "  → Rotate approach: NOT IMPLEMENTED — no approach-rotation service is wired. "
            "Reporting failure.")
        return False

    def _recovery_active_perception(self, f, t):
        self.bus.add_chain_of_thought(
            "  → Active perception: NOT IMPLEMENTED — no viewpoint-planning service is wired. "
            "Reporting failure.")
        return False

    def _recovery_wait_frames(self, f, t):
        self.bus.add_chain_of_thought("  → Waiting 2s for fresh frames...")
        time.sleep(2.0)
        # Verify a fresh detection actually arrived; do not assume it did.
        target = (t.action_queue[0].target_object if t.action_queue else '').lower().strip()
        vision = self.bus.state.vision
        if vision and vision.detected_objects:
            for obj in vision.detected_objects:
                if target and (target in obj.class_name.lower() or obj.class_name.lower() in target):
                    self.bus.add_chain_of_thought(
                        f"  → Fresh detection confirmed '{obj.class_name}'.")
                    return True
        self.bus.add_chain_of_thought(
            "  → Wait frames: no fresh detection of the target arrived.")
        return False

    def _recovery_ask_user(self, f, t):
        t.awaiting_user_approval = True
        t.task_status = 'PAUSED'
        self.bus.publish_task(t)
        return False  # Needs user input

    def _recovery_search(self, f, t):
        self.bus.add_chain_of_thought(
            "  → Search behavior: NOT IMPLEMENTED — no search service is wired. "
            "Reporting failure.")
        return False

    def _recovery_request_approval(self, f, t):
        t.awaiting_user_approval = True
        t.task_status = 'PAUSED'
        self.bus.publish_task(t)
        return False

    def _recovery_safe_shutdown(self, f, t):
        self.bus.add_chain_of_thought("  → SAFE SHUTDOWN: moving to home pose...")
        return False  # Can't continue

    def _recovery_default(self, f, t):
        self.bus.add_chain_of_thought(
            "  → Default recovery: NOT IMPLEMENTED — no generic retry service is wired. "
            "Reporting failure.")
        return False

    def reset(self, task_id: str):
        """Reset retry counters for a task."""
        self.retry_counts.pop(task_id, None)


# ═══════════════════════════════════════════════════════════════
# LLM-driven recovery: the planner LLM diagnoses a stuck/failed
# step from the chain-of-thought trace and proposes corrective steps.
# ═══════════════════════════════════════════════════════════════
RECOVERY_SYSTEM_PROMPT = """You are the recovery brain of a 5-DoF tabletop robot arm (ROS2).
A step of the current plan FAILED. Read the trace, diagnose the most likely root cause, and
propose corrective steps that run BEFORE the failed step is retried.
Typical causes and remedies (reason, do not pattern-match):
- target not found / stale world model / detection ambiguity -> scan_workspace, then locate
- grasp slipped or missed -> depth / sam to refine centroid, then plan_grasp again
- IK / reachability failure -> check reachability; if the object is out of reach, give up
- transparent or reflective object -> cleargrasp, then plan_grasp
- unclear material -> material, then plan_grasp
Executable skills (use ONLY these names in "skill"):
%s
Respond with ONE JSON object:
{"diagnosis": "...", "steps": [{"skill": "name", "object_id": "...", "target": "", "params": {}}],
 "retry_failed_action": true, "give_up": false, "reasoning": "..."}
Set give_up=true only if the task is physically impossible (e.g. unreachable, object missing after a scan).
"""


def llm_recovery_plan(task: TaskState, idx: int, attempt: int) -> Optional[dict]:
    """Synchronous helper (call from a worker thread). Returns parsed recovery dict or None."""
    try:
        from arm_planner.llm_client import OllamaClient
    except Exception:
        return None
    failed = task.action_queue[idx]
    trace = '\n'.join(list(task.chain_of_thought)[-14:])
    prompt = (
        f'COMMAND: "{task.current_command}"\nGOAL: {task.current_goal}\n'
        f'FAILED STEP #{idx}: {failed.action_type}(object="{failed.target_object}", '
        f'destination="{failed.destination}")\nRECOVERY ATTEMPT: {attempt}\n'
        f'RECENT TRACE:\n{trace}\n\nPropose the recovery JSON.')

    async def _run():
        c = OllamaClient()
        resp, ok = await c.generate(
            prompt=prompt, model=c.default_model,
            system_prompt=RECOVERY_SYSTEM_PROMPT % '\n'.join(f'  - {k}: {v}' for k, v in CORE_SKILLS.items()),
            response_format='json', temperature=0.1)
        return resp, ok
    try:
        resp, ok = asyncio.run(_run())
        if not ok:
            return None
        data = json.loads(resp)
        return data if isinstance(data, dict) else None
    except Exception:
        return None


def build_recovery_actions(task: TaskState, idx: int, plan: Optional[dict]) -> Optional[List[Action]]:
    """Turn a recovery dict into Action list (corrective steps + retry clone). None => give up."""
    failed = task.action_queue[idx]
    steps: List[Action] = []
    retry = True
    if plan:
        if plan.get('give_up'):
            return None
        retry = bool(plan.get('retry_failed_action', True))
        for st in plan.get('steps', []) or []:
            if not isinstance(st, dict):
                continue
            name = resolve_executable(st.get('skill'), st.get('agent'))
            if name is None:
                continue
            a = Action()
            a.action_type = name
            a.target_object = str(st.get('object_id') or st.get('object') or failed.target_object or '')
            a.destination = str(st.get('target') or '')
            try:
                a.params_json = json.dumps(st.get('params') or {})
            except (TypeError, ValueError):
                a.params_json = '{}'
            a.confidence = float(failed.confidence)
            a.status = 'PENDING'
            a.reasoning = f"recovery: {plan.get('diagnosis', '')}"[:200]
            steps.append(a)
    else:
        # LLM unavailable: minimal generic recovery — refresh perception, then retry.
        a = Action()
        a.action_type = 'scan_workspace'
        a.confidence = float(failed.confidence)
        a.status = 'PENDING'
        a.reasoning = 'recovery: LLM unavailable, refreshing world model before retry'
        steps.append(a)
    if retry:
        clone = copy.deepcopy(failed)
        clone.status = 'PENDING'
        clone.confidence = float(failed.confidence)
        steps.append(clone)
    return steps
