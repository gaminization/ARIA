"""Shared evidence checks and process cleanup for ARIA simulation audits."""

import os
import signal
import subprocess
import time
from dataclasses import dataclass
from typing import Iterable, Sequence


TASK_TRANSITIONS = {
    "PLANNING": {"EXECUTING", "PAUSED", "FAILED"},
    "EXECUTING": {"PAUSED", "RECOVERY", "COMPLETE", "FAILED"},
    "PAUSED": {"EXECUTING", "RECOVERY", "FAILED"},
    "RECOVERY": {"EXECUTING", "PAUSED", "FAILED"},
    "COMPLETE": set(),
    "FAILED": set(),
}


@dataclass(frozen=True)
class PickEvidence:
    passed: bool
    max_lift_m: float
    continuous_hold_s: float


def evaluate_pick_hold(
    initial_z: float,
    samples: Iterable[tuple[float, float]],
    min_lift_m: float = 0.05,
    min_hold_s: float = 2.0,
    max_sample_gap_s: float = 0.25,
) -> PickEvidence:
    max_lift = 0.0
    longest_hold = 0.0
    hold_start = None
    previous_time = None

    for timestamp, z in samples:
        max_lift = max(max_lift, z - initial_z)
        if previous_time is not None and timestamp - previous_time > max_sample_gap_s:
            hold_start = None

        if z - initial_z >= min_lift_m:
            if hold_start is None:
                hold_start = timestamp
            longest_hold = max(longest_hold, timestamp - hold_start)
        else:
            hold_start = None
        previous_time = timestamp

    return PickEvidence(
        passed=max_lift >= min_lift_m and longest_hold >= min_hold_s,
        max_lift_m=max_lift,
        continuous_hold_s=longest_hold,
    )


def validate_task_trace(
    snapshots: Sequence[tuple[str, Sequence[str]]],
) -> tuple[bool, str]:
    if not snapshots:
        return False, "No task-state messages were observed"

    statuses = [status for status, _ in snapshots]
    if statuses[0] != "PLANNING":
        return False, f"Task did not start in PLANNING (started in {statuses[0]})"

    for previous, current in zip(statuses, statuses[1:]):
        if current == previous:
            continue
        if current not in TASK_TRANSITIONS.get(previous, set()):
            return False, f"Invalid task-state transition: {previous} -> {current}"

    if "EXECUTING" not in statuses:
        return False, "Task never entered EXECUTING"
    if statuses[-1] != "COMPLETE":
        return False, f"Task ended in {statuses[-1]}, not COMPLETE"

    final_actions = snapshots[-1][1]
    if not final_actions:
        return False, "Completed task has no action queue"
    if any(status != "COMPLETE" for status in final_actions):
        return False, f"Completed task has unfinished actions: {list(final_actions)}"

    for _, action_statuses in snapshots:
        if "FAILED" in action_statuses:
            continue
        executing = [i for i, status in enumerate(action_statuses) if status == "EXECUTING"]
        if len(executing) > 1:
            return False, "More than one action was EXECUTING at once"
        if executing:
            active = executing[0]
            if any(status != "COMPLETE" for status in action_statuses[:active]):
                return False, "An action started before its predecessor completed"
            if any(status == "COMPLETE" for status in action_statuses[active + 1:]):
                return False, "An action completed before its predecessor"

    return True, "Task and action state machine completed in order"


def stop_owned_process_group(
    process: subprocess.Popen,
    interrupt_timeout_s: float = 10.0,
    terminate_timeout_s: float = 5.0,
) -> None:
    process_group = process.pid

    def group_exists() -> bool:
        try:
            os.killpg(process_group, 0)
            return True
        except ProcessLookupError:
            return False

    def wait_for_group(timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if not group_exists():
                return True
            time.sleep(0.1)
        return not group_exists()

    if not group_exists():
        process.wait()
        return

    os.killpg(process_group, signal.SIGINT)
    if wait_for_group(interrupt_timeout_s):
        process.wait()
        return

    os.killpg(process_group, signal.SIGTERM)
    if wait_for_group(terminate_timeout_s):
        process.wait()
        return

    os.killpg(process_group, signal.SIGKILL)
    process.wait()
