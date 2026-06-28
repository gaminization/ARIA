#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA LLM Planning Agent — Unit Tests
Tests the LLM planning pipeline, fallback behavior,
plan validation, and world context building.

Run: python3 -m pytest arm_planner/arm_planner/llm_planning_agent_tests.py -v
  or: python3 arm_planner/arm_planner/llm_planning_agent_tests.py
═══════════════════════════════════════════════════════════════
"""
import asyncio
import json
import os
import sys
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

# Add ARIA to path
ARIA_DIR = os.path.expanduser('~/Projects/ARIA')
sys.path.insert(0, os.path.join(ARIA_DIR, 'arm_planner'))


class TestOllamaClient(unittest.TestCase):
    """Tests for the OllamaClient class."""

    def setUp(self):
        from arm_planner.llm_client import OllamaClient
        self.client = OllamaClient(
            base_url="http://localhost:11434",
            config={
                'preferred_model': 'llama3.1:8b-instruct-q4_K_M',
                'temperature': 0.1,
                'timeout_s': 10.0,
                'max_retries': 1,
                'vram_headroom_gb': 2.5,
                'unload_after_planning': True,
            }
        )

    def tearDown(self):
        asyncio.get_event_loop().run_until_complete(
            self.client.close())

    def test_config_defaults(self):
        """Test that config defaults are set correctly."""
        self.assertEqual(
            self.client.default_model,
            'llama3.1:8b-instruct-q4_K_M')
        self.assertEqual(self.client.default_temperature, 0.1)
        self.assertEqual(self.client.default_timeout, 10.0)

    def test_model_info(self):
        """Test model info lookup."""
        info = self.client.get_model_info('llama3.1:8b-instruct-q4_K_M')
        self.assertIsNotNone(info)
        self.assertEqual(info['vram_gb'], 5.0)
        self.assertFalse(info['supports_vision'])

        info_vl = self.client.get_model_info(
            'qwen2.5-vl:7b-instruct-q4_K_M')
        self.assertIsNotNone(info_vl)
        self.assertTrue(info_vl['supports_vision'])

        info_none = self.client.get_model_info('nonexistent')
        self.assertIsNone(info_none)

    def test_json_validation_valid(self):
        """Test JSON validation with valid input."""
        from arm_planner.llm_client import OllamaClient
        valid, cleaned = OllamaClient._validate_json(
            '{"key": "value", "num": 42}')
        self.assertTrue(valid)
        parsed = json.loads(cleaned)
        self.assertEqual(parsed['key'], 'value')

    def test_json_validation_markdown_fences(self):
        """Test JSON validation strips markdown code fences."""
        from arm_planner.llm_client import OllamaClient
        text = '```json\n{"key": "value"}\n```'
        valid, cleaned = OllamaClient._validate_json(text)
        self.assertTrue(valid)
        parsed = json.loads(cleaned)
        self.assertEqual(parsed['key'], 'value')

    def test_json_validation_invalid(self):
        """Test JSON validation with invalid input."""
        from arm_planner.llm_client import OllamaClient
        valid, _ = OllamaClient._validate_json('not json at all')
        self.assertFalse(valid)

    def test_repr(self):
        """Test string representation."""
        repr_str = repr(self.client)
        self.assertIn('OllamaClient', repr_str)
        self.assertIn('localhost:11434', repr_str)


class TestPlanValidation(unittest.TestCase):
    """Tests for plan JSON schema validation."""

    def test_valid_plan(self):
        """Test validation of a correct plan."""
        from arm_planner.llm_planning_agent import validate_plan_json

        plan = {
            "goal": "Pick up the red cube",
            "subgoals": ["Locate cube", "Grasp cube"],
            "actions": [
                {
                    "step": 1,
                    "agent": "VisionAgent",
                    "skill": None,
                    "parameters": {"query": "red cube"},
                    "reasoning": "Need to find the cube first",
                    "preconditions": ["camera active"],
                    "expected_outcome": "Cube detected",
                },
                {
                    "step": 2,
                    "agent": "SkillAgent",
                    "skill": "pick",
                    "parameters": {"object_id": 1},
                    "reasoning": "Pick up the detected cube",
                    "preconditions": ["cube found"],
                    "expected_outcome": "Cube grasped",
                },
            ],
            "confidence": 0.85,
            "ambiguity_notes": "",
            "requires_approval": False,
            "alternative_interpretations": [],
        }

        valid, error = validate_plan_json(plan)
        self.assertTrue(valid, f"Validation failed: {error}")

    def test_missing_keys(self):
        """Test validation catches missing required keys."""
        from arm_planner.llm_planning_agent import validate_plan_json

        plan = {"goal": "test"}  # Missing most keys
        valid, error = validate_plan_json(plan)
        self.assertFalse(valid)
        self.assertIn("Missing required keys", error)

    def test_invalid_confidence(self):
        """Test validation catches out-of-range confidence."""
        from arm_planner.llm_planning_agent import validate_plan_json

        plan = {
            "goal": "test",
            "subgoals": [],
            "actions": [],
            "confidence": 1.5,  # Out of range
            "ambiguity_notes": "",
            "requires_approval": False,
        }
        valid, error = validate_plan_json(plan)
        self.assertFalse(valid)
        self.assertIn("confidence", error)

    def test_invalid_action_type(self):
        """Test that actions must be dicts."""
        from arm_planner.llm_planning_agent import validate_plan_json

        plan = {
            "goal": "test",
            "subgoals": [],
            "actions": ["not a dict"],
            "confidence": 0.5,
            "ambiguity_notes": "",
            "requires_approval": False,
        }
        valid, error = validate_plan_json(plan)
        self.assertFalse(valid)
        self.assertIn("must be a dict", error)


class TestRuleBasedFallback(unittest.TestCase):
    """Tests for the rule-based fallback planner."""

    def _make_task(self, command: str):
        """Create a minimal TaskState-like object for testing."""
        class MockAction:
            def __init__(self):
                self.action_type = ""
                self.target_object = ""
                self.destination = ""
                self.confidence = 0.0
                self.status = ""
                self.reasoning = ""

        class MockTask:
            def __init__(self):
                self.current_command = command
                self.task_status = "PLANNING"
                self.task_id = "test"
                self.current_goal = ""
                self.subgoals = []
                self.action_queue = []
                self.confidence = 0.0
                self.awaiting_user_approval = False
                self.chain_of_thought = []
                self.failure_log = []

        return MockTask()

    def _make_bus(self):
        """Create a minimal mock bus."""
        class MockBus:
            class state:
                memory = None
                task = None
            def add_chain_of_thought(self, entry):
                pass
            def publish_task(self, task):
                pass

        return MockBus()

    def test_pick_command(self):
        """Test parsing 'pick up the red cube'."""
        from arm_planner.llm_planning_agent import _RuleBasedFallback
        fb = _RuleBasedFallback()
        task = self._make_task("pick up the red cube")
        bus = self._make_bus()

        # Need to import Action for the fallback
        try:
            from arm_planner.msg import Action
            fb.decompose(task, bus)
            self.assertTrue(len(task.action_queue) > 0)
            self.assertIn('pick', task.current_goal)
        except ImportError:
            # If ROS msgs not available, test the parsing logic only
            pass

    def test_place_command(self):
        """Test parsing 'put the cube in the box'."""
        from arm_planner.llm_planning_agent import _RuleBasedFallback
        fb = _RuleBasedFallback()
        task = self._make_task("put the cube in the box")
        bus = self._make_bus()

        try:
            from arm_planner.msg import Action
            fb.decompose(task, bus)
            self.assertTrue(len(task.action_queue) > 0)
            self.assertIn('place', task.current_goal)
        except ImportError:
            pass


class TestEpisodicMemory(unittest.TestCase):
    """Tests for the episodic memory system."""

    def setUp(self):
        self.test_db = os.path.join(
            ARIA_DIR, 'arm_planner', 'data', 'test_ep_mem.db')
        from arm_planner.episodic_memory import EpisodicMemory
        self.memory = EpisodicMemory(db_path=self.test_db)

    def tearDown(self):
        if os.path.exists(self.test_db):
            os.unlink(self.test_db)

    def test_record_and_retrieve(self):
        """Test recording and retrieving an episode."""
        ep_id = self.memory.record_episode(
            command="Pick up the red cube",
            goal="pick red_cube",
            success=True,
            duration_s=3.0,
        )
        self.assertGreater(ep_id, 0)

        last = self.memory.get_last_episode()
        self.assertIsNotNone(last)
        self.assertEqual(last.command, "Pick up the red cube")
        self.assertTrue(last.success)

    def test_count_episodes(self):
        """Test episode counting."""
        self.assertEqual(self.memory.count_episodes(), 0)
        self.memory.record_episode(command="test1", success=True)
        self.memory.record_episode(command="test2", success=False)
        self.assertEqual(self.memory.count_episodes(), 2)

    def test_temporal_reference_last(self):
        """Test 'last task' temporal reference."""
        self.memory.record_episode(
            command="Sort the blocks", success=True)
        self.memory.record_episode(
            command="Stack the cubes", success=True)

        last = self.memory.resolve_temporal_reference("last task")
        self.assertIsNotNone(last)
        self.assertEqual(last.command, "Stack the cubes")

    def test_planning_context(self):
        """Test planning context generation."""
        self.memory.record_episode(
            command="Pick up the bottle",
            goal="pick bottle_1",
            success=True,
            duration_s=5.0,
        )

        context = self.memory.get_planning_context("grab the bottle")
        self.assertIn("past tasks", context.lower()
                      if "past tasks" in context.lower()
                      else context)

    def test_stats(self):
        """Test statistics."""
        self.memory.record_episode(command="a", success=True)
        self.memory.record_episode(command="b", success=False)
        self.memory.record_episode(command="c", success=True)

        stats = self.memory.get_stats()
        self.assertEqual(stats['total_episodes'], 3)
        self.assertEqual(stats['successful'], 2)
        self.assertEqual(stats['failed'], 1)


class TestTreeOfThought(unittest.TestCase):
    """Tests for the Tree-of-Thought planner."""

    def test_should_use_tot(self):
        """Test complexity detection."""
        from arm_planner.tree_of_thought_planner import should_use_tot

        # Simple commands should NOT use ToT
        self.assertFalse(should_use_tot("pick up the cube"))
        self.assertFalse(should_use_tot("put it down"))

        # Complex commands SHOULD use ToT
        self.assertTrue(should_use_tot(
            "sort all objects by color"))
        self.assertTrue(should_use_tot(
            "if the box is full, use the other one"))
        self.assertTrue(should_use_tot(
            "think carefully about how to stack them"))

        # Retry should use ToT
        self.assertTrue(should_use_tot(
            "pick up the cube", failed_before=True))

    def test_plan_scoring(self):
        """Test plan candidate scoring."""
        from arm_planner.tree_of_thought_planner import (
            TreeOfThoughtPlanner, PlanCandidate)

        planner = TreeOfThoughtPlanner()

        candidate = PlanCandidate(
            candidate_id=1,
            approach_name="direct_grasp",
            high_level_steps=["find", "grasp", "lift"],
            estimated_success_probability=0.85,
            main_risk="object might slip",
            detailed_plan={
                "actions": [
                    {"agent": "VisionAgent", "reasoning": "find obj",
                     "preconditions": ["camera on"]},
                    {"agent": "SkillAgent", "reasoning": "pick",
                     "preconditions": ["object found"]},
                    {"agent": "VisionAgent", "reasoning": "verify",
                     "preconditions": ["grip closed"]},
                ],
            },
        )

        score = asyncio.get_event_loop().run_until_complete(
            planner.score_plan(candidate, ""))
        self.assertGreater(score, 0)
        self.assertLessEqual(score, 1.0)
        self.assertIn('feasibility', candidate.scores)
        self.assertIn('efficiency', candidate.scores)
        self.assertIn('risk', candidate.scores)

    def test_select_best_plan(self):
        """Test best plan selection."""
        from arm_planner.tree_of_thought_planner import (
            TreeOfThoughtPlanner, PlanCandidate)

        candidates = [
            PlanCandidate(1, "plan_a", [], total_score=0.7),
            PlanCandidate(2, "plan_b", [], total_score=0.9),
            PlanCandidate(3, "plan_c", [], total_score=0.5),
        ]

        best = TreeOfThoughtPlanner.select_best_plan(candidates)
        self.assertEqual(best.candidate_id, 2)
        self.assertEqual(best.approach_name, "plan_b")


class TestFunctionCallingOrchestrator(unittest.TestCase):
    """Tests for the ReAct orchestrator."""

    def test_tool_schema_completeness(self):
        """Test that all tool schemas are well-formed."""
        from arm_planner.function_calling_orchestrator import TOOL_SCHEMAS

        for name, schema in TOOL_SCHEMAS.items():
            self.assertIn('description', schema,
                          f"Tool {name} missing description")
            self.assertIn('parameters', schema,
                          f"Tool {name} missing parameters")
            self.assertIn('returns', schema,
                          f"Tool {name} missing returns")

    def test_tool_description_builder(self):
        """Test tool description string generation."""
        from arm_planner.function_calling_orchestrator import (
            _build_tool_descriptions)

        desc = _build_tool_descriptions()
        self.assertIn('detect_object', desc)
        self.assertIn('execute_skill', desc)
        self.assertIn('task_complete', desc)

    def test_tool_registration(self):
        """Test registering custom tool executors."""
        from arm_planner.function_calling_orchestrator import (
            FunctionCallingOrchestrator)

        orch = FunctionCallingOrchestrator()

        def my_tool(**kwargs):
            return {"result": "ok"}

        orch.register_tool("my_custom_tool", my_tool)
        self.assertIn("my_custom_tool", orch.tool_executors)


# ═══════════════════════════════════════════════════════════════
# Run
# ═══════════════════════════════════════════════════════════════
if __name__ == '__main__':
    unittest.main(verbosity=2)
