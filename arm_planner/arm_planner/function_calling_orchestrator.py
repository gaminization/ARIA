#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Function Calling Orchestrator
ReAct (Reasoning + Acting) loop powered by local LLM.

The LLM dynamically selects which agent/tool to call and with
what parameters. Each iteration: LLM decides → execute → feed
result back for next decision.

This replaces hardcoded agent calls in TaskManager with
flexible, LLM-driven orchestration.
═══════════════════════════════════════════════════════════════
"""
import asyncio
import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from arm_planner.llm_client import OllamaClient

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════
# Data types
# ═══════════════════════════════════════════════════════════════
@dataclass
class ToolResult:
    """Result from a tool execution."""
    tool_name: str
    success: bool
    data: Any = None
    error: str = ""
    duration_ms: float = 0.0


@dataclass
class ReActStep:
    """A single step in the ReAct loop."""
    iteration: int
    thought: str
    tool_name: str
    tool_args: Dict[str, Any]
    result: Optional[ToolResult] = None
    timestamp: float = 0.0


@dataclass
class TaskResult:
    """Final result of the ReAct loop."""
    success: bool
    summary: str
    steps: List[ReActStep] = field(default_factory=list)
    total_duration_ms: float = 0.0
    iterations_used: int = 0
    chain_of_thought: List[str] = field(default_factory=list)


# ═══════════════════════════════════════════════════════════════
# Tool Schema Registry
# ═══════════════════════════════════════════════════════════════
TOOL_SCHEMAS: Dict[str, Dict[str, Any]] = {
    "detect_object": {
        "description": "Detect an object in the workspace using YOLO vision",
        "parameters": {
            "class_name": {
                "type": "string",
                "description": "Object class to find (e.g., 'bottle', 'cube')",
                "required": True,
            },
            "color": {
                "type": "string",
                "description": "Optional color filter (e.g., 'red', 'blue')",
                "required": False,
            },
        },
        "returns": "ObjectDetection with position [x,y,z] and confidence",
    },
    "check_reachability": {
        "description": "Check if a 3D position is within the arm's reachable workspace",
        "parameters": {
            "position": {
                "type": "array",
                "description": "[x, y, z] position in meters (base frame)",
                "required": True,
            },
            "orientation": {
                "type": "array",
                "description": "Optional [rx, ry, rz, rw] quaternion",
                "required": False,
            },
        },
        "returns": "bool reachable, optional alternative_pose if unreachable",
    },
    "get_grasp_strategy": {
        "description": "Compute the best grasp pose for an object based on its shape and affordances",
        "parameters": {
            "object_id": {
                "type": "integer",
                "description": "Object tracking ID from detection",
                "required": True,
            },
            "strategy_hint": {
                "type": "string",
                "description": "Preferred strategy: 'top_down', 'side', 'handle'",
                "required": False,
            },
        },
        "returns": "grasp_pose, approach_pose, retreat_pose, confidence",
    },
    "execute_skill": {
        "description": "Execute a predefined robot manipulation skill",
        "parameters": {
            "skill_name": {
                "type": "string",
                "description": "One of: pick, place, push, pull, stack, sort, inspect, slide, roll, sweep",
                "required": True,
            },
            "skill_args": {
                "type": "object",
                "description": "Skill-specific arguments (e.g., object_id, target, direction)",
                "required": True,
            },
        },
        "returns": "success bool, duration_s, details, failure_reason if failed",
    },
    "query_world_model": {
        "description": "Query the persistent world model for object information and spatial relations",
        "parameters": {
            "query": {
                "type": "string",
                "description": "Natural language query about the scene (e.g., 'what objects are on the left?')",
                "required": True,
            },
        },
        "returns": "List of WorldObject matching query with positions and states",
    },
    "ask_user": {
        "description": "Ask the user a clarifying question or request approval",
        "parameters": {
            "question": {
                "type": "string",
                "description": "The question to ask the user",
                "required": True,
            },
            "question_type": {
                "type": "string",
                "description": "One of: 'approval', 'clarification', 'confirmation'",
                "required": True,
            },
        },
        "returns": "User's response as a string",
    },
    "move_to_pose": {
        "description": "Move the arm to a specific joint configuration or Cartesian pose",
        "parameters": {
            "target": {
                "type": "string",
                "description": "Named pose ('home', 'ready', 'folded') or [x,y,z] position",
                "required": True,
            },
            "speed": {
                "type": "number",
                "description": "Speed factor 0.0-1.0 (default 0.5)",
                "required": False,
            },
        },
        "returns": "success bool, final joint positions",
    },
    "scan_workspace": {
        "description": "Physically sweep the arm across 9 overhead viewpoints so the wrist camera (acting as a top-down sensor) can find all objects on the table. Use this when the world model is empty or the object is not found.",
        "parameters": {
            "target_object": {
                "type": "string",
                "description": "Optional: specific object to search for. Empty = scan everything.",
                "required": False,
            },
        },
        "returns": "List of detected objects with 3D poses registered in world model",
    },
    "get_sam_mask": {
        "description": "Get pixel-accurate SAM2 segmentation mask for an object. Use after YOLO detection to refine the grasp centroid from bbox center to actual object silhouette center.",
        "parameters": {
            "object_id": {
                "type": "string",
                "description": "Object class or ID to segment",
                "required": True,
            },
        },
        "returns": "Binary mask, refined centroid (x,y), mask area_px, grasp_point_world",
    },
    "get_depth": {
        "description": "Get monocular depth estimate for an object using Depth-Anything v2. Returns object height above table and safe approach distance. For transparent objects use correct_transparent_depth instead.",
        "parameters": {
            "object_id": {
                "type": "string",
                "description": "Object class or ID",
                "required": True,
            },
        },
        "returns": "object_height_m, approach_offset_m, depth_confidence",
    },
    "check_material": {
        "description": "Classify the surface material of an object (transparent/glass, opaque/plastic, metal/reflective, fabric/deformable, wood). Critical for routing to the correct depth and grasp agent.",
        "parameters": {
            "object_id": {
                "type": "string",
                "description": "Object class or ID",
                "required": True,
            },
        },
        "returns": "material_class, is_transparent, is_reflective, is_deformable, confidence",
    },
    "correct_transparent_depth": {
        "description": "Use ClearGrasp to correct depth estimation failure on transparent/glass objects. DepthAnything sees through glass and estimates background depth — ClearGrasp fixes this using surface normal inference. ALWAYS use for transparent objects.",
        "parameters": {
            "object_id": {
                "type": "string",
                "description": "Object class or ID (must be transparent)",
                "required": True,
            },
        },
        "returns": "corrected_depth_m, surface_normal, approach_offset_m, confidence",
    },
    "task_complete": {
        "description": "Signal that the task is complete. Call this when all goals are achieved.",
        "parameters": {
            "summary": {
                "type": "string",
                "description": "Brief summary of what was accomplished",
                "required": True,
            },
        },
        "returns": "Terminal — ends the ReAct loop",
    },
    "task_failed": {
        "description": "Signal that the task has failed and cannot be recovered. Call this only when all options are exhausted.",
        "parameters": {
            "reason": {
                "type": "string",
                "description": "Why the task failed",
                "required": True,
            },
        },
        "returns": "Terminal — ends the ReAct loop",
    },
}


def _build_tool_descriptions() -> str:
    """Build a formatted tool description string for the LLM prompt."""
    lines = ["Available tools:\n"]
    for name, schema in TOOL_SCHEMAS.items():
        lines.append(f"  {name}: {schema['description']}")
        params = schema.get('parameters', {})
        if params:
            for pname, pinfo in params.items():
                req = " (required)" if pinfo.get('required') else " (optional)"
                lines.append(
                    f"    - {pname}: {pinfo['type']} — "
                    f"{pinfo['description']}{req}")
        lines.append(f"    Returns: {schema.get('returns', 'void')}")
        lines.append("")
    return '\n'.join(lines)


# ═══════════════════════════════════════════════════════════════
# Function Calling Orchestrator
# ═══════════════════════════════════════════════════════════════
class FunctionCallingOrchestrator:
    """
    ReAct (Reasoning + Acting) loop powered by local LLM.

    Each iteration:
      1. LLM receives: task + tool schemas + history of past steps
      2. LLM outputs: thought + tool call (as JSON)
      3. Orchestrator executes the tool
      4. Result is fed back to LLM for next decision
      5. Loop terminates on task_complete or task_failed

    All intermediate reasoning is logged as chain-of-thought.
    """

    REACT_SYSTEM_PROMPT = """You are ARIA's task executor. You have access to tools to control a 5-DoF robotic arm.

For each step, respond with ONLY a JSON object:
{
  "thought": "Your reasoning about what to do next — think through the full perception chain",
  "tool": "tool_name",
  "arguments": {"arg1": "value1", "arg2": "value2"}
}

Rules:
- Always think before acting. Explain your reasoning in "thought".
- Only call one tool per step.
- If world model is empty or object not found: use scan_workspace FIRST.
- After YOLO detection: use get_sam_mask to refine grasp centroid.
- After detection: use check_material to decide depth/grasp routing.
- For transparent/glass objects: ALWAYS use correct_transparent_depth (not get_depth).
- Always use get_depth (or correct_transparent_depth) before planning a grasp — object height sets safe approach.
- Always use check_reachability before executing a grasp.
- If something fails: reason about why and try a different approach.
- Call task_complete when the goal is achieved.
- Call task_failed only when all recovery options are exhausted.
- Never repeat the same failed action without changing parameters.
- Do NOT hardcode grasp strategies per object type — reason about it from perception output.

{tool_descriptions}
"""

    def __init__(
        self,
        client: Optional[OllamaClient] = None,
        tool_executors: Optional[Dict[str, Callable]] = None,
    ):
        self.client = client or OllamaClient()
        self.tool_executors = tool_executors or {}
        self._tool_descriptions = _build_tool_descriptions()

    def register_tool(self, name: str, executor: Callable):
        """
        Register a tool executor function.

        The executor should accept **kwargs and return a ToolResult.
        """
        self.tool_executors[name] = executor

    async def run_react_loop(
        self,
        command: str,
        world_context: str = "",
        max_iterations: int = 15,
        timeout_s: float = 120.0,
    ) -> TaskResult:
        """
        Run the full ReAct loop for a command.

        Args:
            command:        Natural language command from user
            world_context:  Current world state string
            max_iterations: Maximum loop iterations
            timeout_s:      Total timeout for the entire loop

        Returns:
            TaskResult with success status, steps, and chain of thought
        """
        result = TaskResult(
            success=False,
            summary="",
            chain_of_thought=[f"═══ ReAct Loop: \"{command}\" ═══"],
        )

        t_start = time.monotonic()

        # Build system prompt with tool descriptions
        system_prompt = self.REACT_SYSTEM_PROMPT.format(
            tool_descriptions=self._tool_descriptions)

        # Build conversation history
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": (
                f"TASK: {command}\n\n"
                f"WORLD STATE:\n{world_context}\n\n"
                f"Begin executing the task. Think step by step."
            )},
        ]

        for iteration in range(1, max_iterations + 1):
            # Check timeout
            elapsed = time.monotonic() - t_start
            if elapsed > timeout_s:
                result.summary = f"Timeout after {elapsed:.0f}s"
                result.chain_of_thought.append(
                    f"[ReAct] ⏰ Timeout at iteration {iteration}")
                break

            result.chain_of_thought.append(
                f"[ReAct] ── Iteration {iteration}/{max_iterations} ──")

            # ── LLM decides next action ────────────────────
            response, success = await self.client.chat(
                messages=messages,
                response_format="json",
                temperature=0.1,
                timeout_s=min(30.0, timeout_s - elapsed),
            )

            if not success:
                result.chain_of_thought.append(
                    "[ReAct] ❌ LLM generation failed")
                result.summary = "LLM generation failed during ReAct loop"
                break

            # Parse LLM response
            try:
                decision = json.loads(response)
            except json.JSONDecodeError:
                result.chain_of_thought.append(
                    f"[ReAct] ❌ Invalid JSON from LLM: {response[:100]}")
                messages.append({
                    "role": "user",
                    "content": "Your response was not valid JSON. "
                               "Respond with ONLY a JSON object with "
                               "'thought', 'tool', and 'arguments' keys.",
                })
                continue

            thought = decision.get('thought', '')
            tool_name = decision.get('tool', '')
            tool_args = decision.get('arguments', {})

            step = ReActStep(
                iteration=iteration,
                thought=thought,
                tool_name=tool_name,
                tool_args=tool_args,
                timestamp=time.time(),
            )

            result.chain_of_thought.append(
                f"[ReAct] 💭 Thought: {thought}")
            result.chain_of_thought.append(
                f"[ReAct] 🔧 Tool: {tool_name}({json.dumps(tool_args)})")

            # ── Check for terminal actions ─────────────────
            if tool_name == 'task_complete':
                summary = tool_args.get('summary', 'Task completed')
                result.success = True
                result.summary = summary
                result.chain_of_thought.append(
                    f"[ReAct] ✅ Task complete: {summary}")
                step.result = ToolResult(
                    tool_name='task_complete', success=True,
                    data=summary)
                result.steps.append(step)
                break

            if tool_name == 'task_failed':
                reason = tool_args.get('reason', 'Unknown failure')
                result.success = False
                result.summary = f"Failed: {reason}"
                result.chain_of_thought.append(
                    f"[ReAct] ❌ Task failed: {reason}")
                step.result = ToolResult(
                    tool_name='task_failed', success=False,
                    error=reason)
                result.steps.append(step)
                break

            # ── Execute tool ───────────────────────────────
            tool_result = await self._execute_tool(
                tool_name, tool_args)
            step.result = tool_result

            result.chain_of_thought.append(
                f"[ReAct] 📋 Result: success={tool_result.success}, "
                f"data={str(tool_result.data)[:200]}")

            result.steps.append(step)

            # ── Feed result back to LLM ────────────────────
            messages.append({
                "role": "assistant",
                "content": response,
            })
            messages.append({
                "role": "user",
                "content": (
                    f"Tool result for {tool_name}:\n"
                    f"  success: {tool_result.success}\n"
                    f"  data: {json.dumps(tool_result.data) if tool_result.data else 'null'}\n"
                    f"  error: {tool_result.error or 'none'}\n\n"
                    f"What should I do next?"
                ),
            })

        else:
            # Max iterations reached without terminal action
            result.summary = (
                f"Max iterations ({max_iterations}) reached "
                f"without completion")
            result.chain_of_thought.append(
                f"[ReAct] ⚠ Max iterations reached")

        # Finalize
        result.total_duration_ms = (time.monotonic() - t_start) * 1000
        result.iterations_used = len(result.steps)

        result.chain_of_thought.append(
            f"[ReAct] ═══ Loop ended: {result.summary} "
            f"({result.total_duration_ms:.0f}ms, "
            f"{result.iterations_used} iterations) ═══")

        return result

    async def _execute_tool(
        self,
        tool_name: str,
        tool_args: Dict[str, Any],
    ) -> ToolResult:
        """
        Execute a tool by name with given arguments.
        Uses registered executors, or returns a simulated result.
        """
        t_start = time.monotonic()

        if tool_name not in TOOL_SCHEMAS:
            return ToolResult(
                tool_name=tool_name,
                success=False,
                error=f"Unknown tool: {tool_name}",
                duration_ms=0.0,
            )

        # Check if we have a registered executor
        executor = self.tool_executors.get(tool_name)
        if executor:
            try:
                if asyncio.iscoroutinefunction(executor):
                    data = await executor(**tool_args)
                else:
                    data = await asyncio.get_event_loop().run_in_executor(
                        None, lambda: executor(**tool_args))

                duration = (time.monotonic() - t_start) * 1000
                if isinstance(data, ToolResult):
                    data.duration_ms = duration
                    return data
                return ToolResult(
                    tool_name=tool_name,
                    success=True,
                    data=data,
                    duration_ms=duration,
                )
            except Exception as e:
                duration = (time.monotonic() - t_start) * 1000
                logger.error(f"Tool {tool_name} execution error: {e}")
                return ToolResult(
                    tool_name=tool_name,
                    success=False,
                    error=str(e),
                    duration_ms=duration,
                )

        # No executor registered — return simulated result
        return self._simulate_tool(tool_name, tool_args, t_start)

    def _simulate_tool(
        self,
        tool_name: str,
        tool_args: Dict[str, Any],
        t_start: float,
    ) -> ToolResult:
        """
        No executor is registered for this tool, so it did NOT run.

        Returns an explicit failure. Fabricating a success here would make
        the planner believe a tool executed when it did not, which is
        exactly the kind of fake result this project must not produce.
        """
        duration = (time.monotonic() - t_start) * 1000
        logger.warning(
            f"Tool '{tool_name}' has no registered executor — reporting "
            f"failure (no simulated result).")
        return ToolResult(
            tool_name=tool_name,
            success=False,
            error=(f"No executor registered for tool '{tool_name}'. "
                   "The tool did not execute; no result is available."),
            duration_ms=duration,
            data=None,
        )
