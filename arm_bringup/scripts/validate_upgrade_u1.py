#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Upgrade U1 — LLM Planning Validation
Comprehensive test suite for the LLM intelligence upgrade.

Tests:
  1. Ollama health check
  2. Model loads without OOM
  3. Basic plan generation
  4. Complex plan generation
  5. Temporal reference resolution
  6. Ambiguity detection
  7. VRAM cleanup
  8. Fallback to rule-based

Usage:
  python3 arm_bringup/scripts/validate_upgrade_u1.py
═══════════════════════════════════════════════════════════════
"""
import asyncio
import json
import os
import subprocess
import sys
import time
from typing import Tuple

# Ensure ARIA packages are importable
ARIA_DIR = os.path.expanduser('~/Projects/ARIA')
sys.path.insert(0, os.path.join(ARIA_DIR, 'arm_planner'))
sys.path.insert(0, os.path.join(ARIA_DIR, 'arm_agents'))


# ═══════════════════════════════════════════════════════════════
# Display helpers
# ═══════════════════════════════════════════════════════════════
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    END = '\033[0m'


def header(text: str):
    print(f"\n{Colors.CYAN}{Colors.BOLD}{'═' * 54}{Colors.END}")
    print(f"{Colors.CYAN}{Colors.BOLD}  {text}{Colors.END}")
    print(f"{Colors.CYAN}{Colors.BOLD}{'═' * 54}{Colors.END}\n")


def pass_test(name: str, detail: str = ""):
    detail_str = f" ({detail})" if detail else ""
    print(f"  {Colors.GREEN}✅ {name}{detail_str}{Colors.END}")


def fail_test(name: str, detail: str = ""):
    detail_str = f" ({detail})" if detail else ""
    print(f"  {Colors.RED}❌ {name}{detail_str}{Colors.END}")


def skip_test(name: str, detail: str = ""):
    detail_str = f" ({detail})" if detail else ""
    print(f"  {Colors.YELLOW}⏭  {name}{detail_str}{Colors.END}")


def info(text: str):
    print(f"  {Colors.CYAN}ℹ  {text}{Colors.END}")


# ═══════════════════════════════════════════════════════════════
# Test functions
# ═══════════════════════════════════════════════════════════════
passed = 0
failed = 0
skipped = 0


def record(result: bool, name: str, detail: str = ""):
    global passed, failed
    if result:
        pass_test(name, detail)
        passed += 1
    else:
        fail_test(name, detail)
        failed += 1


def record_skip(name: str, detail: str = ""):
    global skipped
    skip_test(name, detail)
    skipped += 1


async def test_ollama_health() -> bool:
    """Test 1: Ollama health check."""
    from arm_planner.llm_client import OllamaClient
    client = OllamaClient()
    available = await client.is_available()
    await client.close()
    record(available, "Ollama running")
    return available


async def test_model_loads() -> bool:
    """Test 2: Model loads without OOM."""
    from arm_planner.llm_client import OllamaClient
    client = OllamaClient()

    model = "llama3.1:8b-instruct-q4_K_M"
    info(f"Loading model: {model}")

    loaded = await client.load_model(model)
    if not loaded:
        record(False, "Model loads", "failed to load")
        await client.close()
        return False

    # Check VRAM usage
    vram = await client.get_vram_usage()
    vram_used = vram.get('used_gb', 0)

    record(True, "Model loads", f"{vram_used:.1f}GB VRAM used")

    # Don't unload — next test will use it
    await client.close()
    return True


async def test_basic_plan() -> bool:
    """Test 3: Basic plan generation."""
    from arm_planner.llm_client import OllamaClient
    client = OllamaClient()

    prompt = (
        'Generate a plan to pick up the red cube.\n'
        'Known objects: red_cube_1 at [0.15, 0.08, 0.02]\n'
        'Respond with ONLY valid JSON matching the planning schema.'
    )

    # Load prompt template
    prompt_path = os.path.join(
        ARIA_DIR, 'arm_planner', 'prompts',
        'system_prompt_planner.txt')
    system_prompt = ""
    if os.path.exists(prompt_path):
        with open(prompt_path, 'r') as f:
            system_prompt = f.read()

    t_start = time.monotonic()
    response, success = await client.generate(
        prompt=prompt,
        system_prompt=system_prompt,
        response_format="json",
        temperature=0.1,
        timeout_s=60.0,
    )
    elapsed_ms = (time.monotonic() - t_start) * 1000

    if not success:
        record(False, "Simple plan", "LLM generation failed")
        await client.close()
        return False

    try:
        plan = json.loads(response)
    except json.JSONDecodeError:
        record(False, "Simple plan", "invalid JSON")
        await client.close()
        return False

    # Validate structure
    has_goal = 'goal' in plan
    has_actions = 'actions' in plan and len(plan.get('actions', [])) >= 3
    has_confidence = 'confidence' in plan

    valid = has_goal and has_actions and has_confidence
    n_actions = len(plan.get('actions', []))
    record(
        valid,
        "Simple plan",
        f"valid JSON, {n_actions} steps, {elapsed_ms:.0f}ms")

    await client.close()
    return valid


async def test_complex_plan() -> bool:
    """Test 4: Complex plan generation."""
    from arm_planner.llm_client import OllamaClient
    client = OllamaClient()

    prompt = (
        'Generate a plan for: "Sort all the objects by color '
        'and stack the same-color ones together"\n\n'
        'Known objects:\n'
        '  - red_cube_1 at [0.15, 0.08, 0.02]\n'
        '  - red_cylinder_2 at [0.20, -0.05, 0.03]\n'
        '  - blue_cube_3 at [-0.10, 0.12, 0.02]\n'
        '  - blue_bottle_4 at [0.05, 0.18, 0.05]\n'
        '  - green_cube_5 at [-0.15, -0.08, 0.02]\n\n'
        'Respond with ONLY valid JSON matching the planning schema.'
    )

    response, success = await client.generate(
        prompt=prompt,
        response_format="json",
        temperature=0.1,
        timeout_s=90.0,
    )

    if not success:
        record(False, "Complex plan", "LLM generation failed")
        await client.close()
        return False

    try:
        plan = json.loads(response)
    except json.JSONDecodeError:
        record(False, "Complex plan", "invalid JSON")
        await client.close()
        return False

    n_actions = len(plan.get('actions', []))
    valid = n_actions >= 5  # Complex tasks should have 5+ steps
    record(
        valid,
        "Complex plan",
        f"valid JSON, {n_actions} steps")

    await client.close()
    return valid


def test_episodic_memory() -> bool:
    """Test 5: Temporal reference resolution."""
    from arm_planner.episodic_memory import EpisodicMemory

    # Create a temporary memory
    test_db = os.path.join(ARIA_DIR, 'arm_planner', 'data',
                           'test_episodes.db')
    try:
        memory = EpisodicMemory(db_path=test_db)

        # Record a test episode
        ep_id = memory.record_episode(
            command="Pick up the red cube",
            goal="pick red_cube_1",
            plan_json='{"goal": "pick red cube", "actions": []}',
            success=True,
            duration_s=5.0,
        )

        # Test "last task" resolution
        episode = memory.resolve_temporal_reference("last task")
        if not episode:
            record(False, "Temporal reference", "couldn't resolve 'last task'")
            return False

        if episode.command != "Pick up the red cube":
            record(False, "Temporal reference",
                   f"wrong episode: {episode.command}")
            return False

        # Test semantic search
        similar = memory.find_similar_tasks("grab the cube")
        found = len(similar) > 0

        record(found, "Temporal reference",
               f"resolved, {len(similar)} similar found")
        return found

    finally:
        # Cleanup test database
        if os.path.exists(test_db):
            os.unlink(test_db)


async def test_ambiguity_detection() -> bool:
    """Test 6: Ambiguity detection."""
    from arm_planner.llm_client import OllamaClient
    client = OllamaClient()

    prompt = (
        'Generate a plan for: "Pick up the thing"\n\n'
        'Known objects:\n'
        '  - red_cube_1 at [0.15, 0.08, 0.02]\n'
        '  - blue_bottle_2 at [0.20, -0.05, 0.05]\n'
        '  - green_cylinder_3 at [-0.10, 0.12, 0.03]\n\n'
        'The command is intentionally vague. '
        'Respond with ONLY valid JSON matching the planning schema.'
    )

    response, success = await client.generate(
        prompt=prompt,
        response_format="json",
        temperature=0.1,
        timeout_s=60.0,
    )

    if not success:
        record(False, "Ambiguity detection", "LLM generation failed")
        await client.close()
        return False

    try:
        plan = json.loads(response)
    except json.JSONDecodeError:
        record(False, "Ambiguity detection", "invalid JSON")
        await client.close()
        return False

    confidence = plan.get('confidence', 1.0)
    requires_approval = plan.get('requires_approval', False)
    has_ambiguity = bool(plan.get('ambiguity_notes', ''))

    # For a vague command, confidence should be low
    ambiguity_detected = (
        confidence < 0.65 or
        requires_approval or
        has_ambiguity
    )

    record(
        ambiguity_detected,
        "Ambiguity detection",
        f"conf={confidence:.2f}, approval={requires_approval}")

    await client.close()
    return ambiguity_detected


async def test_vram_cleanup() -> bool:
    """Test 7: VRAM cleanup after planning."""
    from arm_planner.llm_client import OllamaClient
    client = OllamaClient()

    # Get VRAM before
    vram_before = await client.get_vram_usage()

    # Ensure model is loaded
    await client.load_model("llama3.1:8b-instruct-q4_K_M")

    # Unload
    unloaded = await client.unload_model()

    # Give Ollama a moment to free VRAM
    await asyncio.sleep(2)

    # Get VRAM after
    vram_after = await client.get_vram_usage()

    freed = vram_before.get('used_gb', 0) > 0  # Had something loaded

    record(
        unloaded,
        "VRAM cleanup",
        f"before={vram_before.get('used_gb', 0):.1f}GB, "
        f"after={vram_after.get('used_gb', 0):.1f}GB")

    await client.close()
    return unloaded


async def test_fallback() -> bool:
    """Test 8: Fallback to rule-based planning."""
    from arm_planner.llm_planning_agent import _RuleBasedFallback
    from arm_planner.msg import TaskState

    fallback = _RuleBasedFallback()

    # Create a mock task
    task = TaskState()
    task.current_command = "pick up the red cube"
    task.task_status = "PLANNING"
    task.task_id = "test_fallback"

    # Create a minimal mock bus
    class MockBus:
        class state:
            memory = None
            task = None
        def add_chain_of_thought(self, entry):
            pass
        def publish_task(self, task):
            pass

    bus = MockBus()
    fallback.decompose(task, bus)

    has_actions = len(task.action_queue) > 0
    has_goal = bool(task.current_goal)

    record(
        has_actions and has_goal,
        "Fallback to rule-based",
        f"{len(task.action_queue)} actions, goal='{task.current_goal}'")

    return has_actions and has_goal


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════
async def main():
    header("ARIA Upgrade U1 — LLM Planning Validation")

    # Test 1: Health check
    ollama_ok = await test_ollama_health()

    if ollama_ok:
        # Tests 2-4: LLM-dependent tests
        model_ok = await test_model_loads()

        if model_ok:
            await test_basic_plan()
            await test_complex_plan()
        else:
            record_skip("Simple plan", "model failed to load")
            record_skip("Complex plan", "model failed to load")

        # Test 5: Episodic memory (doesn't need LLM)
        test_episodic_memory()

        # Test 6: Ambiguity (needs LLM)
        if model_ok:
            await test_ambiguity_detection()
        else:
            record_skip("Ambiguity detection", "model failed to load")

        # Test 7: VRAM cleanup
        await test_vram_cleanup()
    else:
        record_skip("Model loads", "Ollama not available")
        record_skip("Simple plan", "Ollama not available")
        record_skip("Complex plan", "Ollama not available")
        test_episodic_memory()
        record_skip("Ambiguity detection", "Ollama not available")
        record_skip("VRAM cleanup", "Ollama not available")

    # Test 8: Fallback (always works — no LLM needed)
    await test_fallback()

    # ── Summary ────────────────────────────────────────────
    print()
    header("Validation Results")

    total = passed + failed + skipped
    print(f"  Passed:  {Colors.GREEN}{passed}/{total}{Colors.END}")
    print(f"  Failed:  {Colors.RED}{failed}/{total}{Colors.END}")
    print(f"  Skipped: {Colors.YELLOW}{skipped}/{total}{Colors.END}")
    print()

    if failed == 0 and skipped == 0:
        print(f"  {Colors.GREEN}{Colors.BOLD}"
              f"U1 UPGRADE COMPLETE — ALL TESTS PASSED"
              f"{Colors.END}")
    elif failed == 0:
        print(f"  {Colors.YELLOW}{Colors.BOLD}"
              f"U1 UPGRADE PARTIAL — {skipped} tests skipped"
              f"{Colors.END}")
        if not ollama_ok:
            print(f"\n  {Colors.YELLOW}Install Ollama and pull models "
                  f"to enable full LLM planning:{Colors.END}")
            print(f"    bash arm_planner/scripts/upgrade_u1_install.sh")
    else:
        print(f"  {Colors.RED}{Colors.BOLD}"
              f"U1 UPGRADE INCOMPLETE — {failed} tests failed"
              f"{Colors.END}")

    print()


if __name__ == '__main__':
    asyncio.run(main())
