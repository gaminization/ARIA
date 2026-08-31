#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA LLM Dialogue Agent — LifecycleNode
Drop-in upgrade for rule-based DialogueAgent.
Uses local LLM for richer natural language communication.

Capabilities added over original DialogueAgent:
  - Conversational task clarification via LLM
  - Context-aware failure explanations
  - Cross-session references via EpisodicMemory
  - Proactive suggestions based on task patterns
  - Visual planning (when Qwen2.5-VL is enabled)

Graceful degradation: If Ollama is down, falls back to
template-based responses (same as original DialogueAgent).
═══════════════════════════════════════════════════════════════
"""
import asyncio
import json
import logging
import os
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import String, Bool

from arm_planner.msg import TaskState
from arm_planner.state_bus import StateBus
from arm_planner.llm_client import OllamaClient
from arm_planner.episodic_memory import EpisodicMemory

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════
# Prompt loader
# ═══════════════════════════════════════════════════════════════
_PROMPTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    '..', 'arm_planner', 'prompts')


def _load_prompt(filename: str) -> str:
    """Load a prompt template."""
    path = os.path.join(_PROMPTS_DIR, filename)
    try:
        with open(path, 'r') as f:
            return f.read()
    except FileNotFoundError:
        return ""


# ═══════════════════════════════════════════════════════════════
# Template-based fallback responses
# ═══════════════════════════════════════════════════════════════
FAILURE_TEMPLATES = {
    'MISSED_OBJECT': (
        "I couldn't grasp the object. My gripper closed but "
        "didn't make contact. This might be because the object "
        "position was slightly off. Try adjusting lighting."),
    'OBJECT_SLIPPED': (
        "I had the object but it slipped during transport. "
        "The grip force might be too low for this object."),
    'IK_FAILURE': (
        "I can't reach that position. It might be outside "
        "my workspace or blocked by cables."),
    'COLLISION': (
        "I couldn't find a safe path. The workspace might "
        "be too cluttered. Try moving some objects."),
    'PERCEPTION_ERROR': (
        "I couldn't see the object clearly enough. "
        "Detection confidence was too low."),
    'TRACKING_LOST': (
        "I lost track of the object. It may have been "
        "moved or occluded."),
    'TIMEOUT': (
        "The action took too long. Something may be stuck."),
}


# ═══════════════════════════════════════════════════════════════
# LLM Dialogue Agent
# ═══════════════════════════════════════════════════════════════
class LLMDialogueAgent(LifecycleNode):
    """
    LLM-enhanced dialogue agent for ARIA.

    Replaces template-based responses with LLM-generated
    natural language when Ollama is available. Falls back
    to templates when LLM is unavailable.

    Publishes:
      /aria/dialogue/output        (String: user-facing text)
      /aria/dialogue/requires_input (Bool)
    """

    def __init__(self):
        super().__init__('llm_dialogue_agent')
        self.get_logger().info(
            "═══ ARIA LLM Dialogue Agent initializing ═══")

        self.bus = StateBus(self)
        self.client = OllamaClient()
        self.memory = EpisodicMemory()
        self.system_prompt = _load_prompt('system_prompt_dialogue.txt')

        # State tracking
        self.last_status = ''
        self.last_action_idx = -1
        self._llm_available = False
        self._task_start_time = 0.0

        # Track task for episodic recording
        self._current_command = ""
        self._current_goal = ""
        self._current_plan_json = ""

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("LLMDialogueAgent: CONFIGURING")

        self.output_pub = self.create_publisher(
            String, '/aria/dialogue/output', 10)
        self.input_pub = self.create_publisher(
            Bool, '/aria/dialogue/requires_input', 10)

        self.bus.on_change('task', self._on_task)

        # Check LLM availability
        self.create_timer(2.0, self._check_llm)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("LLMDialogueAgent: ACTIVATED")
        self._say("ARIA ready. I can understand natural language commands "
                  "and explain my reasoning.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ── Output helpers ─────────────────────────────────────
    def _say(self, text: str):
        """Publish a dialogue message."""
        msg = String()
        msg.data = text
        self.output_pub.publish(msg)
        self.get_logger().info(f"[DIALOGUE] {text}")

    def _set_requires_input(self, needs: bool):
        """Signal whether user input is needed."""
        msg = Bool()
        msg.data = needs
        self.input_pub.publish(msg)

    # ── LLM availability ───────────────────────────────────
    def _check_llm(self):
        """Periodically check if LLM is available."""
        self._run_async(self._async_check_llm())

    async def _async_check_llm(self):
        available = await self.client.is_available()
        if available != self._llm_available:
            self._llm_available = available
            mode = "LLM-enhanced" if available else "template-based"
            self.get_logger().info(
                f"Dialogue mode: {mode}")

    def _run_async(self, coro):
        """Run async from sync callbacks."""
        try:
            loop = asyncio.get_event_loop()
            if loop.is_running():
                asyncio.ensure_future(coro)
            else:
                loop.run_until_complete(coro)
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(coro)

    # ── Task state handler ─────────────────────────────────
    def _on_task(self, msg: TaskState):
        """React to task state changes and generate dialogue."""
        status = msg.task_status

        # Track command for episodic recording
        if msg.current_command and msg.current_command != self._current_command:
            self._current_command = msg.current_command
            self._current_goal = msg.current_goal
            self._task_start_time = rclpy.clock.Clock().now().nanoseconds / 1e9

        if status != self.last_status:
            self.last_status = status

            if status == 'PLANNING':
                self._handle_planning(msg)
            elif status == 'EXECUTING':
                self._handle_executing(msg)
            elif status == 'PAUSED':
                self._handle_paused(msg)
            elif status == 'RECOVERY':
                self._handle_recovery(msg)
            elif status == 'COMPLETE':
                self._handle_complete(msg)
            elif status == 'FAILED':
                self._handle_failed(msg)

        # Progress updates during execution
        if status == 'EXECUTING':
            self._report_progress(msg)

    # ── Status handlers ────────────────────────────────────
    def _handle_planning(self, msg: TaskState):
        """Planning started."""
        # Check for temporal references
        ref = self._check_temporal_reference(msg.current_command)
        if ref:
            self._say(
                f"🤔 I found a similar task from {ref.age_description}: "
                f"\"{ref.command}\". Let me plan based on what worked "
                f"{'well' if ref.success else 'and what went wrong'} "
                f"last time.")
        else:
            self._say(
                f"🤔 Understanding command: \"{msg.current_command}\"...")

    def _handle_executing(self, msg: TaskState):
        """Execution started."""
        n = len(msg.action_queue)
        self._say(
            f"🚀 Executing plan: {n} actions queued. "
            f"Goal: {msg.current_goal}")

    def _handle_paused(self, msg: TaskState):
        """Task paused for approval."""
        if self._llm_available:
            self._run_async(
                self._llm_approval_request(msg))
        else:
            self._template_approval_request(msg)

    def _handle_recovery(self, msg: TaskState):
        """Recovery in progress."""
        if msg.failure_log:
            last_fail = msg.failure_log[-1]
            if self._llm_available:
                self._run_async(
                    self._llm_failure_explanation(
                        last_fail, recovering=True))
            else:
                explanation = FAILURE_TEMPLATES.get(
                    last_fail.failure_type,
                    f"Problem: {last_fail.failure_type}")
                self._say(
                    f"⚠ {explanation} Attempting recovery...")

    def _handle_complete(self, msg: TaskState):
        """Task completed."""
        import time as _time
        duration = _time.time() - self._task_start_time

        self._say(f"✅ Task complete: \"{msg.current_command}\"")
        self._set_requires_input(False)

        # Record to episodic memory
        self.memory.record_episode(
            command=self._current_command,
            goal=self._current_goal,
            plan_json=self._current_plan_json,
            success=True,
            duration_s=duration,
        )

        # Proactive suggestions
        self._check_proactive_suggestions()

    def _handle_failed(self, msg: TaskState):
        """Task failed."""
        import time as _time
        duration = _time.time() - self._task_start_time

        if msg.failure_log and self._llm_available:
            self._run_async(
                self._llm_failure_explanation(msg.failure_log[-1]))
        elif msg.failure_log:
            last_fail = msg.failure_log[-1]
            explanation = FAILURE_TEMPLATES.get(
                last_fail.failure_type,
                f"Failure: {last_fail.failure_type}. "
                f"Cause: {last_fail.cause}")
            self._say(
                f"❌ Task failed: \"{msg.current_command}\"\n"
                f"{explanation}")
        else:
            self._say(f"❌ Task failed: \"{msg.current_command}\"")

        self._set_requires_input(False)

        # Record failure to episodic memory
        self.memory.record_episode(
            command=self._current_command,
            goal=self._current_goal,
            plan_json=self._current_plan_json,
            success=False,
            duration_s=duration,
        )

    # ── LLM-powered responses ──────────────────────────────
    async def _llm_approval_request(self, msg: TaskState):
        """Generate an LLM-powered approval request."""
        context = self._build_dialogue_context(msg)

        prompt = (
            f"The robot needs user approval to proceed.\n\n"
            f"Context:\n{context}\n\n"
            f"Generate a natural, friendly approval request "
            f"explaining what the robot plans to do and why "
            f"it's uncertain. Keep it under 3 sentences."
        )

        response, success = await self.client.generate(
            prompt=prompt,
            system_prompt=self.system_prompt,
            response_format="json",
            temperature=0.3,
        )

        if success:
            try:
                data = json.loads(response)
                message = data.get('message', '')
                if message:
                    self._say(f"⏸ {message}")
                    self._set_requires_input(True)
                    return
            except json.JSONDecodeError:
                pass

        # Fallback to template
        self._template_approval_request(msg)

    async def _llm_failure_explanation(
        self,
        failure,
        recovering: bool = False,
    ):
        """Generate an LLM-powered failure explanation."""
        prompt = (
            f"The robot encountered a failure during task execution.\n\n"
            f"Failure type: {failure.failure_type}\n"
            f"Cause: {failure.cause}\n"
            f"Currently recovering: {recovering}\n\n"
            f"Generate a natural, helpful explanation of what went "
            f"wrong and what the user can do to help. Be honest "
            f"and suggest concrete fixes. Keep it under 3 sentences."
        )

        response, success = await self.client.generate(
            prompt=prompt,
            system_prompt=self.system_prompt,
            response_format="json",
            temperature=0.3,
        )

        if success:
            try:
                data = json.loads(response)
                message = data.get('message', '')
                if message:
                    prefix = "⚠" if recovering else "❌"
                    suffix = " Attempting recovery..." if recovering else ""
                    self._say(f"{prefix} {message}{suffix}")
                    return
            except json.JSONDecodeError:
                pass

        # Fallback
        explanation = FAILURE_TEMPLATES.get(
            failure.failure_type,
            f"Problem: {failure.failure_type}. Cause: {failure.cause}")
        prefix = "⚠" if recovering else "❌"
        self._say(f"{prefix} {explanation}")

    # ── Template fallbacks ─────────────────────────────────
    def _template_approval_request(self, msg: TaskState):
        """Template-based approval request (fallback)."""
        reason = ""
        for action in msg.action_queue:
            if action.status == 'PENDING' and action.confidence < 0.75:
                reason = (
                    f"I'm about to execute '{action.action_type}' on "
                    f"'{action.target_object}', but my confidence is "
                    f"only {action.confidence:.0%}. "
                    f"Reason: {action.reasoning}")
                break

        if not reason:
            reason = f"Overall task confidence: {msg.confidence:.0%}"

        self._say(
            f"⏸ Paused — I need your approval.\n"
            f"{reason}\n"
            f"Should I proceed? "
            f"Call /aria/approve (yes) or /aria/reject (no)")
        self._set_requires_input(True)

    # ── Progress reporting ─────────────────────────────────
    def _report_progress(self, msg: TaskState):
        """Report action progress during execution."""
        for i, action in enumerate(msg.action_queue):
            if action.status == 'COMPLETE' and i > self.last_action_idx:
                self.last_action_idx = i
                self._say(
                    f"  ✓ {action.action_type}"
                    f"({action.target_object})")
            elif action.status == 'EXECUTING' and i > self.last_action_idx:
                self._say(
                    f"  → {action.action_type}"
                    f"({action.target_object})...")

    # ── Cross-session references ───────────────────────────
    def _check_temporal_reference(
        self,
        command: str,
    ) -> Optional['Episode']:
        """Check if command references a past task."""
        temporal_words = [
            'last', 'previous', 'again', 'yesterday', 'before',
            'same', 'repeat', 'did', 'what we', 'like before',
        ]
        command_lower = command.lower()
        if any(w in command_lower for w in temporal_words):
            return self.memory.resolve_temporal_reference(command)
        return None

    # ── Proactive suggestions ──────────────────────────────
    def _check_proactive_suggestions(self):
        """
        Check if we should make proactive suggestions based
        on task patterns.
        """
        recent = self.memory.get_recent_episodes(20)
        if len(recent) < 5:
            return

        # Count command patterns
        from collections import Counter
        command_types = Counter()
        for ep in recent:
            # Extract the verb
            words = ep.command.lower().split()
            if words:
                command_types[words[0]] += 1

        # If any command type used 5+ times
        for verb, count in command_types.most_common(3):
            if count >= 5:
                self._say(
                    f"💡 I've done '{verb}' tasks {count} times recently. "
                    f"Would you like me to optimize this as a named skill?")
                break

    # ── Context builder ────────────────────────────────────
    def _build_dialogue_context(self, msg: TaskState) -> str:
        """Build context string for LLM dialogue generation."""
        sections = [
            f"Current command: \"{msg.current_command}\"",
            f"Task status: {msg.task_status}",
            f"Confidence: {msg.confidence:.2f}",
            f"Goal: {msg.current_goal}",
        ]

        # Current action details
        for action in msg.action_queue:
            if action.status in ('PENDING', 'EXECUTING'):
                sections.append(
                    f"Next action: {action.action_type} on "
                    f"{action.target_object} "
                    f"(confidence={action.confidence:.2f})")
                break

        # Recent chain of thought
        if msg.chain_of_thought:
            recent_cot = msg.chain_of_thought[-3:]
            sections.append("Recent reasoning:")
            for entry in recent_cot:
                sections.append(f"  {entry}")

        # Scene objects
        memory = self.bus.state.memory
        if memory and memory.known_objects:
            obj_names = [wo.name for wo in memory.known_objects[:5]]
            sections.append(f"Objects in scene: {', '.join(obj_names)}")

        return '\n'.join(sections)


# ═══════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = LLMDialogueAgent()
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
