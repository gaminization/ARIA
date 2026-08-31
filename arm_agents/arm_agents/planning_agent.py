#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Planning Agent — LifecycleNode
Rule-based NLP task decomposition. No LLM required.
Parses commands → extracts (verb, object, target, conditions)
→ maps to canonical templates → expands to action queue.
═══════════════════════════════════════════════════════════════
"""
import re
from typing import Dict, List, Optional, Tuple
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from geometry_msgs.msg import PoseStamped
from arm_planner.msg import TaskState, Action
from arm_planner.state_bus import StateBus

# ═══════════════════════════════════════════════════════════════
# Command templates
# ═══════════════════════════════════════════════════════════════
VERB_SYNONYMS = {
    'pick': ['pick', 'grab', 'grasp', 'get', 'take', 'lift'],
    'place': ['place', 'put', 'set', 'drop', 'lay'],
    'stack': ['stack', 'pile'],
    'push': ['push', 'shove', 'nudge'],
    'pull': ['pull', 'drag'],
    'sort': ['sort', 'organize', 'arrange', 'group'],
    'inspect': ['inspect', 'look', 'examine', 'check', 'show'],
    'find': ['find', 'locate', 'search', 'where'],
    'slide': ['slide', 'move'],
    'sweep': ['sweep', 'clear', 'clean'],
    'roll': ['roll'],
}

# Template: verb → list of action types
TASK_TEMPLATES: Dict[str, List[str]] = {
    'pick': ['locate', 'plan_grasp', 'execute_grasp', 'lift'],
    'place': ['locate_object', 'locate_target', 'pick', 'transport', 'place', 'verify'],
    'stack': ['locate_object', 'locate_base', 'pick', 'align_over', 'place_on', 'verify_stable'],
    'push': ['locate', 'plan_push_path', 'approach', 'execute_push', 'verify'],
    'pull': ['locate', 'approach', 'grip_light', 'execute_pull', 'verify'],
    'sort': ['locate_all', 'classify', 'pick_each', 'place_in_zone'],
    'inspect': ['locate', 'move_camera_around', 'capture_views', 'report'],
    'find': ['search_workspace', 'report_position'],
    'slide': ['locate', 'plan_slide_path', 'execute_slide', 'verify'],
    'sweep': ['identify_area', 'plan_sweep', 'execute_sweep'],
    'roll': ['locate', 'plan_roll', 'execute_roll', 'verify'],
}

# Color words for object matching
COLORS = ['red', 'blue', 'green', 'yellow', 'white', 'black', 'orange', 'pink', 'purple']

# Prepositions that separate object from target
PREPOSITIONS = ['in', 'on', 'onto', 'into', 'to', 'toward', 'towards',
                'near', 'beside', 'next', 'above', 'below']


class PlanningAgent(LifecycleNode):
    """
    NLP task decomposition agent.

    Approach (no LLM — rule-based + templates):
      1. Parse command → extract (action_verb, object, target, conditions)
      2. Map to canonical task template
      3. Expand template to subgoals + action queue
      4. Compute confidence score
    """

    def __init__(self):
        super().__init__('planning_agent')
        self.bus = StateBus(self)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("PlanningAgent: CONFIGURING")
        self.bus.on_change('task', self._on_task_changed)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("PlanningAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_task_changed(self, msg: TaskState):
        """React to PLANNING status — decompose the command."""
        if msg.task_status != 'PLANNING':
            return
        if not msg.current_command:
            return
        self._decompose(msg)

    def _decompose(self, task: TaskState):
        """
        Parse and decompose a natural language command.
        Updates TaskState with subgoals and action queue.
        """
        command = task.current_command.lower().strip()

        # ── Step 1: Parse command ──────────────────────────
        verb, obj, target, conditions = self._parse_command(command)
        parse_confidence = 0.0

        if verb:
            parse_confidence = 0.9
            self.bus.add_chain_of_thought(
                f"PLANNING: Parsed command: verb='{verb}' "
                f"object='{obj}' target='{target}'")
        else:
            parse_confidence = 0.3
            verb = 'pick'  # default fallback
            self.bus.add_chain_of_thought(
                f"PLANNING: Could not parse verb. "
                f"Defaulting to 'pick'. Low confidence.")

        # ── Step 2: Map to template ────────────────────────
        canonical_verb = self._get_canonical_verb(verb)
        template = TASK_TEMPLATES.get(canonical_verb, TASK_TEMPLATES['pick'])

        # ── Step 3: Expand to subgoals + actions ───────────
        subgoals = []
        actions = []

        for i, action_type in enumerate(template):
            subgoal = self._expand_action(action_type, obj, target, i)
            subgoals.append(subgoal['description'])

            action = Action()
            action.action_type = subgoal['action_type']
            action.target_object = obj or ''
            action.destination = target or ''
            action.confidence = subgoal['confidence']
            action.status = 'PENDING'
            action.reasoning = subgoal['reasoning']
            actions.append(action)

        # ── Step 4: Confidence scoring ─────────────────────
        # Check if objects exist in memory
        obj_confidence = 1.0
        memory = self.bus.state.memory
        if memory and obj:
            found = any(
                wo.name == obj or wo.class_name in obj
                for wo in memory.known_objects)
            if not found:
                obj_confidence = 0.6
                self.bus.add_chain_of_thought(
                    f"PLANNING: Object '{obj}' not in world model. "
                    f"Will need to search.")

        overall_confidence = parse_confidence * obj_confidence

        # ── Step 5: Update TaskState ───────────────────────
        task.current_goal = f"{canonical_verb} {obj}" + \
            (f" → {target}" if target else "")
        task.subgoals = subgoals
        task.action_queue = actions
        task.confidence = overall_confidence

        # Set requires_approval if low confidence
        if overall_confidence < 0.75:
            task.awaiting_user_approval = True
            task.task_status = 'PAUSED'
            self.bus.add_chain_of_thought(
                f"PLANNING: Overall confidence {overall_confidence:.2f} < 0.75. "
                f"Requesting user approval.")
        else:
            task.task_status = 'EXECUTING'

        self.bus.add_chain_of_thought(
            f"PLANNING: Decomposed into {len(subgoals)} subgoals, "
            f"confidence={overall_confidence:.2f}")
        for i, sg in enumerate(subgoals):
            self.bus.add_chain_of_thought(f"  SUBGOAL {i+1}: {sg}")

        self.bus.publish_task(task)

    def _parse_command(self, command: str) -> Tuple[
            Optional[str], Optional[str], Optional[str], dict]:
        """
        Parse NL command into (verb, object, target, conditions).

        Examples:
          "pick up the red cube"       → ('pick', 'red cube', None, {})
          "put the red cube in the box" → ('place', 'red cube', 'box', {})
          "stack blue on red"          → ('stack', 'blue', 'red', {})
          "sort all objects by color"  → ('sort', 'all objects', None, {'by': 'color'})
        """
        # Remove articles and filler words
        cleaned = re.sub(r'\b(the|a|an|this|that|please|can you|could you)\b',
                         '', command).strip()
        cleaned = re.sub(r'\s+', ' ', cleaned)

        words = cleaned.split()
        if not words:
            return None, None, None, {}

        # Find verb
        verb = None
        verb_idx = -1
        for i, word in enumerate(words):
            for canonical, synonyms in VERB_SYNONYMS.items():
                if word in synonyms:
                    verb = word
                    verb_idx = i
                    break
            if verb:
                break

        if verb is None:
            return None, ' '.join(words), None, {}

        # Remove "up" after pick
        remaining = words[verb_idx + 1:]
        if remaining and remaining[0] == 'up':
            remaining = remaining[1:]

        # Split on prepositions to find object and target
        prep_idx = -1
        prep_word = None
        for i, word in enumerate(remaining):
            if word in PREPOSITIONS:
                prep_idx = i
                prep_word = word
                break

        if prep_idx >= 0:
            obj_words = remaining[:prep_idx]
            target_words = remaining[prep_idx + 1:]
            # Remove "to" duplicates like "next to"
            if target_words and target_words[0] == 'to':
                target_words = target_words[1:]
            obj = ' '.join(obj_words) if obj_words else None
            target = ' '.join(target_words) if target_words else None
        else:
            obj = ' '.join(remaining) if remaining else None
            target = None

        conditions = {}
        if 'by color' in command:
            conditions['by'] = 'color'

        return verb, obj, target, conditions

    def _get_canonical_verb(self, verb: str) -> str:
        """Map a synonym to its canonical verb."""
        for canonical, synonyms in VERB_SYNONYMS.items():
            if verb in synonyms:
                return canonical
        return 'pick'

    def _expand_action(self, action_type: str, obj: str, target: str,
                       index: int) -> dict:
        """Expand a template action into a detailed subgoal."""
        descriptions = {
            'locate': f"Locate '{obj}' in workspace",
            'locate_object': f"Locate object '{obj}'",
            'locate_target': f"Locate target '{target}'",
            'locate_all': f"Locate all objects in workspace",
            'locate_base': f"Locate base object '{target}'",
            'plan_grasp': f"Plan grasp for '{obj}'",
            'execute_grasp': f"Execute grasp on '{obj}'",
            'lift': f"Lift '{obj}' from surface",
            'transport': f"Transport '{obj}' to '{target}'",
            'place': f"Place '{obj}' at target location",
            'place_on': f"Place '{obj}' on '{target}'",
            'verify': "Verify action result",
            'verify_stable': "Verify stack is stable",
            'align_over': f"Align '{obj}' precisely over '{target}'",
            'plan_push_path': f"Plan push path for '{obj}'",
            'approach': f"Approach '{obj}'",
            'execute_push': f"Execute push on '{obj}'",
            'grip_light': f"Grip '{obj}' lightly",
            'execute_pull': f"Pull '{obj}'",
            'classify': "Classify objects by properties",
            'pick_each': "Pick each object sequentially",
            'place_in_zone': "Place in designated zone",
            'move_camera_around': f"Move camera to inspect '{obj}'",
            'capture_views': "Capture multiple views",
            'report': "Generate inspection report",
            'search_workspace': f"Search workspace for '{obj}'",
            'report_position': "Report object position",
            'plan_slide_path': f"Plan slide path for '{obj}'",
            'execute_slide': f"Slide '{obj}' to target",
            'identify_area': "Identify area to clear",
            'plan_sweep': "Plan sweep motion",
            'execute_sweep': "Execute sweep",
            'plan_roll': f"Plan roll for '{obj}'",
            'execute_roll': f"Roll '{obj}' to target",
        }

        reasoning = {
            'locate': f"Need to find '{obj}' before any manipulation. Using top camera + YOLO.",
            'plan_grasp': f"Computing grasp pose via AffordanceAgent for '{obj}'.",
            'execute_grasp': f"Running grasp executor: approach → descend → close gripper.",
            'lift': "Lifting 10cm to clear surface.",
            'transport': f"Moving to '{target}' while maintaining grip.",
            'place': "Descending and releasing at target.",
        }

        return {
            'action_type': action_type,
            'description': descriptions.get(action_type, action_type),
            'confidence': 0.85,
            'reasoning': reasoning.get(action_type,
                                       f"Executing {action_type}"),
        }


def main(args=None):
    rclpy.init(args=args)
    node = PlanningAgent()
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
