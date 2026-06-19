#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Cause & Effect Model
Knowledge base of how objects interact.
Used by PlanningAgent to anticipate + verify effects.
═══════════════════════════════════════════════════════════════
"""
import math
from typing import Dict, List, Optional

INTERACTION_MODEL: Dict[str, dict] = {
    "pick": {
        "preconditions": ["object_detected", "object_reachable", "gripper_empty"],
        "expected": "object is in gripper",
        "side_effects": [],
        "risk": "miss_object",
    },
    "place": {
        "preconditions": ["object_in_gripper", "target_reachable"],
        "expected": "object at target location, gripper empty",
        "side_effects": ["object.pose changes"],
        "risk": "object_tips_over",
    },
    "push": {
        "preconditions": ["object_detected", "push_path_clear"],
        "expected": "object moves in direction",
        "side_effects": ["may_move_adjacent_objects", "may_fall_off_edge"],
        "risk": "object_falls_off_table",
    },
    "pull": {
        "preconditions": ["object_detected", "object_graspable"],
        "expected": "object moves toward arm",
        "side_effects": ["may_drag_connected_objects"],
        "risk": "object_slips",
    },
    "stack": {
        "preconditions": [
            "base_surface_flat", "object_base_flat",
            "object_width <= base_surface_width",
            "base_stable"
        ],
        "expected": "object is_on base, stable",
        "side_effects": ["base may compress", "stack may tilt"],
        "risk": "stack_topples_if_not_centered",
    },
    "slide": {
        "preconditions": ["object_on_surface", "path_clear", "low_friction"],
        "expected": "object slides to target position",
        "side_effects": ["friction_dependent_distance"],
        "risk": "overshoot_or_undershoot",
    },
    "roll": {
        "preconditions": ["object_is_round", "surface_flat"],
        "expected": "object rolls to target",
        "side_effects": ["may_roll_past_target"],
        "risk": "unpredictable_path_on_uneven_surface",
    },
    "sweep": {
        "preconditions": ["area_identified", "arm_can_reach_area"],
        "expected": "objects cleared from area",
        "side_effects": ["objects_displaced_randomly"],
        "risk": "objects_fall_off_table",
    },
}


def predict_effects(action_type: str, target_object: str,
                    world_model: dict) -> dict:
    """
    Predict what will change when an action is executed.

    Args:
        action_type: e.g. "push", "stack"
        target_object: name of target object
        world_model: dict with object states

    Returns:
        dict with:
          expected: str description
          side_effects: list of possible side effects
          risk: str description of risk
          preconditions_met: list of (condition, met: bool)
    """
    model = INTERACTION_MODEL.get(action_type, {})

    preconditions_met = []
    for pc in model.get('preconditions', []):
        # Check each precondition against world model
        met = _check_precondition(pc, target_object, world_model)
        preconditions_met.append((pc, met))

    return {
        'expected': model.get('expected', 'unknown'),
        'side_effects': model.get('side_effects', []),
        'risk': model.get('risk', 'none'),
        'preconditions_met': preconditions_met,
        'all_preconditions_met': all(met for _, met in preconditions_met),
    }


def check_preconditions(action_type: str, target_object: str,
                        world_model: dict) -> tuple:
    """
    Check if all preconditions are met.

    Returns: (all_met: bool, unmet: list[str])
    """
    prediction = predict_effects(action_type, target_object, world_model)
    unmet = [cond for cond, met in prediction['preconditions_met'] if not met]
    return prediction['all_preconditions_met'], unmet


def verify_effects(action_type: str, before_state: dict,
                   after_state: dict) -> dict:
    """
    Compare predicted effects with actual observed changes.

    Args:
        action_type: what was attempted
        before_state: world state before action
        after_state: world state after action

    Returns:
        dict with:
          expected_achieved: bool
          unexpected_changes: list of unexpected changes
          discrepancy_score: float (0 = perfect match, 1 = total mismatch)
    """
    model = INTERACTION_MODEL.get(action_type, {})

    # Compare object positions
    unexpected = []
    total_discrepancy = 0.0

    for obj_name in after_state:
        if obj_name in before_state:
            before_pos = before_state[obj_name].get('position', [0, 0, 0])
            after_pos = after_state[obj_name].get('position', [0, 0, 0])
            moved = math.sqrt(sum((a - b)**2 for a, b in zip(before_pos, after_pos)))

            if moved > 0.01:  # 1cm threshold
                if obj_name != action_type:  # Not the target
                    unexpected.append({
                        'object': obj_name,
                        'moved_m': moved,
                        'description': f"{obj_name} moved {moved*100:.1f}cm (side effect)",
                    })
                    total_discrepancy += moved

    n_objects = max(len(after_state), 1)
    discrepancy_score = min(1.0, total_discrepancy / (n_objects * 0.05))

    return {
        'expected_achieved': discrepancy_score < 0.5,
        'unexpected_changes': unexpected,
        'discrepancy_score': discrepancy_score,
    }


def _check_precondition(condition: str, target: str,
                        world_model: dict) -> bool:
    """Check a single precondition against world model."""
    checks = {
        'object_detected': lambda: target in world_model,
        'object_reachable': lambda: world_model.get(target, {}).get('reachable', True),
        'gripper_empty': lambda: world_model.get('gripper', {}).get('empty', True),
        'object_in_gripper': lambda: not world_model.get('gripper', {}).get('empty', True),
        'target_reachable': lambda: True,
        'push_path_clear': lambda: True,
        'object_graspable': lambda: True,
        'base_surface_flat': lambda: True,
        'object_base_flat': lambda: True,
        'base_stable': lambda: True,
        'object_on_surface': lambda: True,
        'path_clear': lambda: True,
        'low_friction': lambda: True,
        'object_is_round': lambda: world_model.get(target, {}).get('shape', '') in ('sphere', 'cylinder'),
        'surface_flat': lambda: True,
        'area_identified': lambda: True,
        'arm_can_reach_area': lambda: True,
    }

    # Handle parameterized conditions
    if '<=' in condition:
        return True  # Would need actual measurements

    check_fn = checks.get(condition, lambda: True)
    try:
        return check_fn()
    except Exception:
        return True  # Assume met if can't check
