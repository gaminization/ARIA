#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Failure Classifier
Pattern-matches error context → FailureEvent with type + cause
+ recommended recovery strategy.
═══════════════════════════════════════════════════════════════
"""
from arm_planner.msg import FailureEvent
import rclpy

FAILURE_TYPES = {
    "MISSED_OBJECT":    "Gripper closed but no contact detected",
    "OBJECT_SLIPPED":   "Contact lost during transport",
    "IK_FAILURE":       "No IK solution found for target pose",
    "COLLISION":        "Path planner could not find collision-free path",
    "PERCEPTION_ERROR": "Object detection confidence too low",
    "TRACKING_LOST":    "Tracking ID lost for N frames",
    "LOW_CONFIDENCE":   "Confidence below threshold, user approval needed",
    "JOINT_LIMIT":      "Motion would exceed joint limit",
    "CABLE_VIOLATION":  "Path passes through cable zone",
    "SERVO_FAULT":      "Servo not responding or overheating",
    "TIMEOUT":          "Action exceeded time limit",
}

# Pattern matching rules: (condition_fn, failure_type, recovery_hint)
CLASSIFICATION_RULES = [
    (lambda ctx: ctx.get('gripper_fully_closed', False) and ctx.get('expected_contact', False),
     'MISSED_OBJECT', 'Re-detect and adjust grasp pose'),

    (lambda ctx: ctx.get('force_dropped', False) and ctx.get('was_holding', False),
     'OBJECT_SLIPPED', 'Lower arm, re-grasp from current position'),

    (lambda ctx: 'ik' in ctx.get('error_source', '').lower() or ctx.get('ik_failed', False),
     'IK_FAILURE', 'Try fallback solver or approach from different angle'),

    (lambda ctx: 'collision' in ctx.get('error_source', '').lower(),
     'COLLISION', 'Inflate margins and replan, or try different planner'),

    (lambda ctx: ctx.get('detection_confidence', 1.0) < 0.4,
     'PERCEPTION_ERROR', 'Trigger active perception for better view'),

    (lambda ctx: ctx.get('tracking_lost', False),
     'TRACKING_LOST', 'Re-detect from scratch, check last known position'),

    (lambda ctx: ctx.get('confidence', 1.0) < 0.75 and not ctx.get('user_approved', False),
     'LOW_CONFIDENCE', 'Request user approval'),

    (lambda ctx: ctx.get('joint_limit_hit', False),
     'JOINT_LIMIT', 'Approach from different configuration'),

    (lambda ctx: ctx.get('cable_zone_violation', False),
     'CABLE_VIOLATION', 'Replan avoiding cable zones'),

    (lambda ctx: ctx.get('servo_fault', False),
     'SERVO_FAULT', 'Safe shutdown, check servo'),

    (lambda ctx: ctx.get('timeout', False),
     'TIMEOUT', 'Retry with extended timeout or simpler plan'),
]


def classify(error_context: dict) -> FailureEvent:
    """
    Classify an error into a FailureEvent.

    Args:
        error_context: dict with keys describing the error:
            error_source, gripper_fully_closed, expected_contact,
            force_dropped, was_holding, ik_failed, detection_confidence,
            tracking_lost, confidence, user_approved, joint_limit_hit,
            cable_zone_violation, servo_fault, timeout,
            action_type, target_object

    Returns:
        FailureEvent with type, cause, and recommended recovery.
    """
    event = FailureEvent()
    event.timestamp = rclpy.clock.Clock().now().to_msg()
    event.action_at_failure = error_context.get('action_type', 'unknown')

    # Try each rule in order
    for condition_fn, failure_type, recovery_hint in CLASSIFICATION_RULES:
        try:
            if condition_fn(error_context):
                event.failure_type = failure_type
                event.cause = FAILURE_TYPES.get(failure_type, 'Unknown cause')
                event.recovery_attempted = recovery_hint
                event.recovery_succeeded = False  # not yet attempted
                return event
        except Exception:
            continue

    # Default: unknown failure
    event.failure_type = 'UNKNOWN'
    event.cause = str(error_context.get('error_message', 'Unknown error'))
    event.recovery_attempted = 'Manual intervention'
    event.recovery_succeeded = False
    return event


def classify_from_exception(exception: Exception,
                            action_type: str = '') -> FailureEvent:
    """Convenience: classify from a Python exception."""
    ctx = {
        'error_message': str(exception),
        'error_source': type(exception).__name__,
        'action_type': action_type,
    }
    # Infer failure type from exception text
    msg = str(exception).lower()
    if 'ik' in msg or 'inverse kinematics' in msg:
        ctx['ik_failed'] = True
    elif 'timeout' in msg or 'timed out' in msg:
        ctx['timeout'] = True
    elif 'collision' in msg:
        ctx['error_source'] = 'collision'
    elif 'servo' in msg or 'motor' in msg:
        ctx['servo_fault'] = True

    return classify(ctx)
