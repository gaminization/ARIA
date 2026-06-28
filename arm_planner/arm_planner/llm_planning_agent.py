#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA LLM Planning Agent — LifecycleNode
Drop-in upgrade for rule-based PlanningAgent.
Uses local Ollama LLM for flexible task decomposition.

Graceful degradation:
  Ollama available  → LLM planning mode
  Ollama unavailable → falls back to rule-based PlanningAgent

Publishes:
  /aria/planning/llm_plan        (String: raw JSON)
  /aria/planning/mode            (String: "llm" or "rulebased")
  /aria/planning/last_plan_time_ms (Float64)
═══════════════════════════════════════════════════════════════
"""
import asyncio
import json
import os
import time
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import String, Float64

from arm_planner.msg import TaskState, Action
from arm_planner.state_bus import StateBus
from arm_planner.llm_client import OllamaClient

# Import the original PlanningAgent for fallback
from arm_agents.planning_agent import PlanningAgent as RuleBasedPlanningAgent

# ═══════════════════════════════════════════════════════════════
# Prompt loader
# ═══════════════════════════════════════════════════════════════
_PROMPTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'prompts')


def _load_prompt(filename: str) -> str:
    """Load a prompt template from the prompts directory."""
    path = os.path.join(_PROMPTS_DIR, filename)
    try:
        with open(path, 'r') as f:
            return f.read()
    except FileNotFoundError:
        return ""


# ═══════════════════════════════════════════════════════════════
# Plan JSON Schema Validator
# ═══════════════════════════════════════════════════════════════
REQUIRED_PLAN_KEYS = {
    'goal', 'subgoals', 'actions', 'confidence',
    'requires_approval', 'ambiguity_notes',
}

REQUIRED_ACTION_KEYS = {
    'step', 'agent', 'reasoning',
}

VALID_AGENTS = {
    'VisionAgent', 'DepthAgent', 'TrackingAgent', 'AffordanceAgent',
    'ControlAgent', 'SafetyAgent', 'MemoryAgent', 'WorldModelAgent',
    'SkillAgent', 'ReachabilityAgent',
}

VALID_SKILLS = {
    'pick', 'place', 'push', 'pull', 'stack', 'sort',
    'inspect', 'slide', 'roll', 'sweep',
}


def validate_plan_json(plan: dict) -> tuple[bool, str]:
    """
    Validate a plan JSON against the expected schema.
    Returns (is_valid, error_message).
    """
    # Check required top-level keys
    missing = REQUIRED_PLAN_KEYS - set(plan.keys())
    if missing:
        return False, f"Missing required keys: {missing}"

    # Check types
    if not isinstance(plan.get('goal'), str):
        return False, "'goal' must be a string"
    if not isinstance(plan.get('subgoals'), list):
        return False, "'subgoals' must be a list"
    if not isinstance(plan.get('actions'), list):
        return False, "'actions' must be a list"
    if not isinstance(plan.get('confidence'), (int, float)):
        return False, "'confidence' must be a number"

    confidence = plan['confidence']
    if not (0.0 <= confidence <= 1.0):
        return False, f"'confidence' must be 0.0-1.0, got {confidence}"

    # Validate actions
    for i, action in enumerate(plan['actions']):
        if not isinstance(action, dict):
            return False, f"Action {i} must be a dict"
        action_missing = REQUIRED_ACTION_KEYS - set(action.keys())
        if action_missing:
            return False, f"Action {i} missing keys: {action_missing}"

        agent = action.get('agent', '')
        if agent and agent not in VALID_AGENTS:
            # Warning, not error — LLM might use slightly different names
            pass

        skill = action.get('skill')
        if skill and skill not in VALID_SKILLS:
            pass  # Also a soft warning

    return True, ""


# ═══════════════════════════════════════════════════════════════
# LLM Planning Agent
# ═══════════════════════════════════════════════════════════════
class LLMPlanningAgent(LifecycleNode):
    """
    LLM-backed planning agent for ARIA.

    Registers on the same state bus topic as the original
    PlanningAgent, making it a drop-in upgrade. TaskManager
    does not change.

    Planning pipeline:
      1. Build world context string
      2. Load LLM (if not loaded)
      3. Construct prompt from template + runtime context
      4. Generate plan via Ollama
      5. Validate plan JSON
      6. Unload LLM to free VRAM
      7. Return plan to TaskManager via state bus

    Fallback: If Ollama is unavailable at any point, falls
    back to the original rule-based PlanningAgent.
    """

    def __init__(self):
        super().__init__('llm_planning_agent')
        self.get_logger().info("═══ ARIA LLM Planning Agent initializing ═══")

        self.bus = StateBus(self)
        self.client = OllamaClient()
        self.system_prompt = _load_prompt('system_prompt_planner.txt')

        # Mode tracking
        self._mode = "initializing"
        self._fallback_active = False

        # Create a reference to the rule-based decompose logic
        # without instantiating a full node — we just need the methods
        self._rule_parser = _RuleBasedFallback()

        # Performance tracking
        self._last_plan_time_ms = 0.0
        self._plans_generated = 0
        self._fallbacks_used = 0

        # Async event loop reference
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("LLMPlanningAgent: CONFIGURING")

        # Publishers
        self.plan_pub = self.create_publisher(
            String, '/aria/planning/llm_plan', 10)
        self.mode_pub = self.create_publisher(
            String, '/aria/planning/mode', 10)
        self.time_pub = self.create_publisher(
            Float64, '/aria/planning/last_plan_time_ms', 10)

        # Listen for PLANNING status on state bus
        self.bus.on_change('task', self._on_task_changed)

        # Check Ollama availability asynchronously
        self.create_timer(1.0, self._check_ollama_once)
        self.create_timer(30.0, self._periodic_health_check)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("LLMPlanningAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("LLMPlanningAgent: DEACTIVATED")
        # Unload model on deactivation
        self._run_async(self.client.unload_model())
        return TransitionCallbackReturn.SUCCESS

    # ── Async bridge ───────────────────────────────────────
    def _run_async(self, coro):
        """Run an async coroutine from sync ROS callbacks."""
        try:
            loop = asyncio.get_event_loop()
            if loop.is_running():
                # Schedule on existing loop
                asyncio.ensure_future(coro)
                return None
            else:
                return loop.run_until_complete(coro)
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                return loop.run_until_complete(coro)
            finally:
                pass  # Don't close — may be reused

    # ── Health check ───────────────────────────────────────
    def _check_ollama_once(self):
        """One-time check on startup."""
        self._run_async(self._async_check_ollama())
        # Cancel the one-shot timer (only need to check once)
        # ROS2 doesn't support cancel, so we just set a flag
        self._initial_check_done = True

    def _periodic_health_check(self):
        """Periodic health check every 30 seconds."""
        self._run_async(self._async_check_ollama())

    async def _async_check_ollama(self):
        """Check if Ollama is available and set mode accordingly."""
        available = await self.client.is_available()

        if available and self._mode != "llm":
            self._mode = "llm"
            self._fallback_active = False
            self.get_logger().info(
                "✅ Ollama is available — LLM planning mode active")
        elif not available and self._mode != "rulebased":
            self._mode = "rulebased"
            self._fallback_active = True
            self.get_logger().warn(
                "⚠ Ollama unavailable — falling back to rule-based planning")

        self._publish_mode()

    def _publish_mode(self):
        """Publish current planning mode."""
        msg = String()
        msg.data = self._mode
        self.mode_pub.publish(msg)

    # ── Task state listener ────────────────────────────────
    def _on_task_changed(self, msg: TaskState):
        """React to PLANNING status — decompose the command."""
        if msg.task_status != 'PLANNING':
            return
        if not msg.current_command:
            return
        self._run_async(self._async_decompose(msg))

    # ── Main planning pipeline ─────────────────────────────
    async def _async_decompose(self, task: TaskState):
        """
        Full LLM planning pipeline with fallback.

        Steps:
          1. Check Ollama availability
          2. Build world context
          3. Load model
          4. Generate plan
          5. Validate plan
          6. Unload model
          7. Update task state
        """
        command = task.current_command.strip()
        t_start = time.monotonic()

        self.bus.add_chain_of_thought(
            f"LLM_PLANNING: Received command: \"{command}\"")
        self.bus.add_chain_of_thought(
            f"LLM_PLANNING: Current mode: {self._mode}")

        # ── Step 1: Check availability ─────────────────────
        available = await self.client.is_available()
        if not available:
            self.bus.add_chain_of_thought(
                "LLM_PLANNING: Ollama unavailable — using rule-based fallback")
            self._fallback_decompose(task)
            return

        # ── Step 2: Build world context ────────────────────
        context = self._build_world_context()
        self.bus.add_chain_of_thought(
            f"LLM_PLANNING: World context built ({len(context)} chars)")

        # ── Step 3: Load model ─────────────────────────────
        model = self.client.default_model
        loaded = await self.client.load_model(model)
        if not loaded:
            self.bus.add_chain_of_thought(
                "LLM_PLANNING: Failed to load model — using fallback")
            self._fallback_decompose(task)
            return

        self.bus.add_chain_of_thought(
            f"LLM_PLANNING: Model '{model}' loaded into VRAM")

        # ── Step 4: Construct prompt ───────────────────────
        prompt = self._build_prompt(command, context)

        # ── Step 5: Generate plan ──────────────────────────
        self.bus.add_chain_of_thought(
            "LLM_PLANNING: Generating plan via LLM...")

        response, success = await self.client.generate(
            prompt=prompt,
            model=model,
            system_prompt=self.system_prompt,
            response_format="json",
            temperature=0.1,
        )

        if not success:
            self.bus.add_chain_of_thought(
                "LLM_PLANNING: LLM generation failed — using fallback")
            await self._maybe_unload(model)
            self._fallback_decompose(task)
            return

        # ── Step 6: Parse and validate ─────────────────────
        try:
            plan = json.loads(response)
        except json.JSONDecodeError:
            self.bus.add_chain_of_thought(
                "LLM_PLANNING: JSON parse failed — using fallback")
            await self._maybe_unload(model)
            self._fallback_decompose(task)
            return

        valid, error = validate_plan_json(plan)
        if not valid:
            self.bus.add_chain_of_thought(
                f"LLM_PLANNING: Plan validation failed: {error} — "
                f"using fallback")
            await self._maybe_unload(model)
            self._fallback_decompose(task)
            return

        # ── Step 7: Cross-check with world model ──────────
        plan = self._cross_check_plan(plan)

        # ── Step 8: Unload model ───────────────────────────
        await self._maybe_unload(model)

        # ── Step 9: Convert plan to TaskState ──────────────
        elapsed_ms = (time.monotonic() - t_start) * 1000
        self._last_plan_time_ms = elapsed_ms
        self._plans_generated += 1

        self._apply_plan_to_task(task, plan, elapsed_ms)

        # Publish raw plan JSON
        plan_msg = String()
        plan_msg.data = json.dumps(plan, indent=2)
        self.plan_pub.publish(plan_msg)

        # Publish timing
        time_msg = Float64()
        time_msg.data = elapsed_ms
        self.time_pub.publish(time_msg)

        self.bus.add_chain_of_thought(
            f"LLM_PLANNING: Plan generated in {elapsed_ms:.0f}ms — "
            f"{len(plan.get('actions', []))} actions, "
            f"confidence={plan.get('confidence', 0):.2f}")

    async def _maybe_unload(self, model: str):
        """Unload model if configured to do so."""
        if self.client.unload_after_planning:
            await self.client.unload_model(model)
            self.bus.add_chain_of_thought(
                "LLM_PLANNING: Model unloaded — VRAM freed for execution")

    # ── World context builder ──────────────────────────────
    def _build_world_context(self) -> str:
        """Build world context string from state bus."""
        sections = []

        # Objects from memory
        memory = self.bus.state.memory
        if memory and memory.known_objects:
            obj_lines = []
            for wo in memory.known_objects:
                pos = wo.last_known_pose.pose.position
                obj_lines.append(
                    f"  - {wo.name} (class={wo.class_name}, "
                    f"pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}], "
                    f"state={wo.lifecycle_state})")
            sections.append("Known objects:\n" + '\n'.join(obj_lines))
        else:
            sections.append("Known objects: none (workspace not yet scanned)")

        # Joint state
        joints = self.bus.state.joints
        if joints and joints.name:
            joint_strs = [
                f"  {n}: {p:.2f}rad"
                for n, p in zip(joints.name, joints.position)]
            sections.append("Joint state:\n" + '\n'.join(joint_strs))
        else:
            sections.append("Joint state: unknown")

        # Spatial relations from world model (if available)
        memory = self.bus.state.memory
        if memory and memory.spatial_relations:
            rel_lines = []
            for r in memory.spatial_relations[:10]:
                rel_lines.append(
                    f"  - {r.subject} {r.relation} {r.object} "
                    f"(conf={r.confidence:.2f})")
            sections.append(
                "Spatial relations:\n" + '\n'.join(rel_lines))
        else:
            sections.append("Spatial relations: none computed")

        # Task history
        task = self.bus.state.task
        if task and task.chain_of_thought:
            recent = task.chain_of_thought[-5:]
            sections.append(
                "Recent chain of thought:\n" +
                '\n'.join(f"  {entry}" for entry in recent))

        return '\n\n'.join(sections)

    def _build_prompt(self, command: str, context: str) -> str:
        """Build the complete prompt for the LLM."""
        # Inject episodic context if available
        episodic_context = "No relevant past tasks found."
        try:
            from arm_planner.episodic_memory import EpisodicMemory
            em = EpisodicMemory()
            episodic_context = em.get_planning_context(command)
        except (ImportError, Exception):
            pass

        prompt = (
            f"COMMAND: \"{command}\"\n\n"
            f"CURRENT WORLD STATE:\n{context}\n\n"
            f"EPISODIC MEMORY:\n{episodic_context}\n\n"
            f"Generate a plan to accomplish this command. "
            f"Respond with ONLY valid JSON matching the schema."
        )
        return prompt

    # ── Plan cross-checking ────────────────────────────────
    def _cross_check_plan(self, plan: dict) -> dict:
        """
        Cross-check plan against current world model.
        Adjust confidence if referenced objects don't exist.
        """
        memory = self.bus.state.memory
        known_names = set()
        if memory and memory.known_objects:
            for wo in memory.known_objects:
                known_names.add(wo.name.lower())
                known_names.add(wo.class_name.lower())

        # Check if actions reference unknown objects
        unknown_refs = 0
        for action in plan.get('actions', []):
            params = action.get('parameters', {})
            for key, val in params.items():
                if isinstance(val, str) and 'object' in key.lower():
                    if val.lower() not in known_names and val != 'null':
                        unknown_refs += 1

        if unknown_refs > 0 and plan['confidence'] > 0.5:
            # Reduce confidence if referencing unknown objects
            reduction = min(0.2, unknown_refs * 0.05)
            plan['confidence'] = max(0.3, plan['confidence'] - reduction)
            if not plan.get('ambiguity_notes'):
                plan['ambiguity_notes'] = ""
            plan['ambiguity_notes'] += (
                f" Note: {unknown_refs} object references not found in "
                f"world model — may need to locate first.")

        # Set requires_approval based on final confidence
        if plan['confidence'] < self.client.config.get(
                'require_approval_below', 0.65):
            plan['requires_approval'] = True

        return plan

    # ── Convert plan to TaskState ──────────────────────────
    def _apply_plan_to_task(self, task: TaskState, plan: dict,
                            elapsed_ms: float):
        """Apply the LLM-generated plan to the TaskState."""
        task.current_goal = plan.get('goal', task.current_command)
        task.subgoals = plan.get('subgoals', [])
        task.confidence = plan.get('confidence', 0.5)

        # Convert plan actions to Action messages
        actions = []
        for act in plan.get('actions', []):
            action = Action()
            action.action_type = act.get('skill') or act.get('agent', '')
            action.target_object = str(
                act.get('parameters', {}).get('object_id', ''))
            action.destination = str(
                act.get('parameters', {}).get('target', ''))
            action.confidence = plan.get('confidence', 0.5)
            action.status = 'PENDING'
            action.reasoning = act.get('reasoning', '')
            actions.append(action)

        task.action_queue = actions

        # Approval gate
        if plan.get('requires_approval', False):
            task.awaiting_user_approval = True
            task.task_status = 'PAUSED'
            self.bus.add_chain_of_thought(
                f"LLM_PLANNING: Confidence {plan['confidence']:.2f} — "
                f"requesting user approval")
            if plan.get('ambiguity_notes'):
                self.bus.add_chain_of_thought(
                    f"LLM_PLANNING: Ambiguity: {plan['ambiguity_notes']}")
        else:
            task.task_status = 'EXECUTING'

        # Log subgoals
        for i, sg in enumerate(task.subgoals):
            self.bus.add_chain_of_thought(f"  SUBGOAL {i+1}: {sg}")

        self.bus.publish_task(task)

    # ── Rule-based fallback ────────────────────────────────
    def _fallback_decompose(self, task: TaskState):
        """Fall back to rule-based planning."""
        self._fallbacks_used += 1
        self._mode = "rulebased"
        self._publish_mode()

        self.bus.add_chain_of_thought(
            "LLM_PLANNING: FALLBACK — using rule-based decomposition")

        # Use the rule-based parser
        self._rule_parser.decompose(task, self.bus)

    # ── Stats ──────────────────────────────────────────────
    def get_stats(self) -> dict:
        """Get planning statistics."""
        return {
            'mode': self._mode,
            'plans_generated': self._plans_generated,
            'fallbacks_used': self._fallbacks_used,
            'last_plan_time_ms': self._last_plan_time_ms,
            'model_loaded': self.client.loaded_model,
        }


# ═══════════════════════════════════════════════════════════════
# Rule-Based Fallback (extracted from original PlanningAgent)
# ═══════════════════════════════════════════════════════════════
class _RuleBasedFallback:
    """
    Standalone rule-based decomposer extracted from the original
    PlanningAgent. No ROS node — just the parsing logic.

    This ensures the original planning_agent.py is NEVER modified.
    """

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

    TASK_TEMPLATES = {
        'pick': ['locate', 'plan_grasp', 'execute_grasp', 'lift'],
        'place': ['locate_object', 'locate_target', 'pick',
                  'transport', 'place', 'verify'],
        'stack': ['locate_object', 'locate_base', 'pick',
                  'align_over', 'place_on', 'verify_stable'],
        'push': ['locate', 'plan_push_path', 'approach',
                 'execute_push', 'verify'],
        'pull': ['locate', 'approach', 'grip_light',
                 'execute_pull', 'verify'],
        'sort': ['locate_all', 'classify', 'pick_each',
                 'place_in_zone'],
        'inspect': ['locate', 'move_camera_around',
                    'capture_views', 'report'],
        'find': ['search_workspace', 'report_position'],
        'slide': ['locate', 'plan_slide_path', 'execute_slide',
                  'verify'],
        'sweep': ['identify_area', 'plan_sweep', 'execute_sweep'],
        'roll': ['locate', 'plan_roll', 'execute_roll', 'verify'],
    }

    PREPOSITIONS = [
        'in', 'on', 'onto', 'into', 'to', 'toward', 'towards',
        'near', 'beside', 'next', 'above', 'below',
    ]

    def decompose(self, task: TaskState, bus: StateBus):
        """Rule-based task decomposition."""
        import re

        command = task.current_command.lower().strip()
        cleaned = re.sub(
            r'\b(the|a|an|this|that|please|can you|could you)\b',
            '', command).strip()
        cleaned = re.sub(r'\s+', ' ', cleaned)
        words = cleaned.split()

        if not words:
            task.task_status = 'FAILED'
            bus.publish_task(task)
            return

        # Find verb
        verb = None
        verb_idx = -1
        for i, word in enumerate(words):
            for canonical, synonyms in self.VERB_SYNONYMS.items():
                if word in synonyms:
                    verb = word
                    verb_idx = i
                    break
            if verb:
                break

        if verb is None:
            verb = 'pick'
            verb_idx = 0

        # Get canonical verb
        canonical_verb = 'pick'
        for cv, synonyms in self.VERB_SYNONYMS.items():
            if verb in synonyms:
                canonical_verb = cv
                break

        # Extract object and target
        remaining = words[verb_idx + 1:]
        if remaining and remaining[0] == 'up':
            remaining = remaining[1:]

        prep_idx = -1
        for i, word in enumerate(remaining):
            if word in self.PREPOSITIONS:
                prep_idx = i
                break

        if prep_idx >= 0:
            obj = ' '.join(remaining[:prep_idx]) or None
            target_words = remaining[prep_idx + 1:]
            if target_words and target_words[0] == 'to':
                target_words = target_words[1:]
            target = ' '.join(target_words) or None
        else:
            obj = ' '.join(remaining) if remaining else None
            target = None

        # Expand template
        template = self.TASK_TEMPLATES.get(
            canonical_verb, self.TASK_TEMPLATES['pick'])

        subgoals = []
        actions = []
        for i, action_type in enumerate(template):
            action = Action()
            action.action_type = action_type
            action.target_object = obj or ''
            action.destination = target or ''
            action.confidence = 0.85
            action.status = 'PENDING'
            action.reasoning = f"Rule-based: {action_type}"
            subgoals.append(f"{action_type} {obj or ''}")
            actions.append(action)

        task.current_goal = f"{canonical_verb} {obj or 'object'}"
        if target:
            task.current_goal += f" → {target}"
        task.subgoals = subgoals
        task.action_queue = actions
        task.confidence = 0.85 if verb else 0.3

        if task.confidence < 0.75:
            task.awaiting_user_approval = True
            task.task_status = 'PAUSED'
        else:
            task.task_status = 'EXECUTING'

        bus.add_chain_of_thought(
            f"RULE_BASED: Decomposed into {len(subgoals)} subgoals, "
            f"confidence={task.confidence:.2f}")
        bus.publish_task(task)


# ═══════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = LLMPlanningAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
