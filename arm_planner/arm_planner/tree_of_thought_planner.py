#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Tree-of-Thought Planner
Generates multiple plan candidates, evaluates each, selects
the best one. For complex multi-step tasks.

Activated when:
  - Task has 5+ expected steps
  - Task has conditional logic ("if X then Y else Z")
  - Task failed once and is being retried
  - User explicitly requests careful planning

VRAM note: ToT makes 3+ LLM calls. Model is loaded once,
kept for all calls, then unloaded.
═══════════════════════════════════════════════════════════════
"""
import asyncio
import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from arm_planner.llm_client import OllamaClient

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════
# Data Types
# ═══════════════════════════════════════════════════════════════
@dataclass
class PlanCandidate:
    """A candidate plan generated during Tree-of-Thought."""
    candidate_id: int
    approach_name: str
    high_level_steps: List[str]
    detailed_plan: Optional[dict] = None
    estimated_success_probability: float = 0.5
    main_risk: str = ""
    scores: Dict[str, float] = field(default_factory=dict)
    total_score: float = 0.0


@dataclass
class ToTResult:
    """Result of Tree-of-Thought planning."""
    selected_plan: Optional[PlanCandidate]
    all_candidates: List[PlanCandidate]
    generation_time_ms: float = 0.0
    scoring_time_ms: float = 0.0
    total_time_ms: float = 0.0
    chain_of_thought: List[str] = field(default_factory=list)


# ═══════════════════════════════════════════════════════════════
# Complexity Detector
# ═══════════════════════════════════════════════════════════════
def should_use_tot(command: str, failed_before: bool = False) -> bool:
    """
    Determine if a command warrants Tree-of-Thought planning.

    Returns True if:
      - Command implies 5+ steps (sort, organize, complex assembly)
      - Command has conditional logic (if/then/else)
      - Task previously failed (retry with different approach)
      - User explicitly asks for careful planning
    """
    command_lower = command.lower()

    # Explicit user request
    careful_phrases = [
        'think carefully', 'plan carefully', 'consider options',
        'think about', 'best way', 'most efficient', 'optimal',
    ]
    if any(phrase in command_lower for phrase in careful_phrases):
        return True

    # Conditional logic
    conditional_words = ['if', 'unless', 'either', 'otherwise', 'else']
    if any(w in command_lower.split() for w in conditional_words):
        return True

    # Multi-step keywords that imply 5+ actions
    complex_verbs = [
        'sort', 'organize', 'arrange', 'group', 'classify',
        'build', 'assemble', 'construct', 'create', 'setup',
    ]
    if any(v in command_lower for v in complex_verbs):
        return True

    # Conjunctions implying multi-task
    conjunctions = [' and then ', ' then ', ' after that ', ' next ']
    conjunction_count = sum(
        1 for c in conjunctions if c in command_lower)
    if conjunction_count >= 2:
        return True

    # Previous failure
    if failed_before:
        return True

    return False


# ═══════════════════════════════════════════════════════════════
# Tree-of-Thought Planner
# ═══════════════════════════════════════════════════════════════
class TreeOfThoughtPlanner:
    """
    Tree-of-Thought planner for complex multi-step tasks.

    Tree structure:
      Root:    initial task description
      Level 1: N high-level approaches (breadth = n_candidates)
      Level 2: expand each approach into detailed steps (depth)
      Evaluate: score each complete plan
      Select:   pick highest scoring plan

    All LLM calls use a single loaded model instance to
    minimize VRAM load/unload cycles.
    """

    # Scoring weights
    FEASIBILITY_WEIGHT = 0.40
    EFFICIENCY_WEIGHT = 0.20
    RISK_WEIGHT = 0.30
    PRECONDITION_WEIGHT = 0.10

    def __init__(
        self,
        client: Optional[OllamaClient] = None,
        n_candidates: int = 3,
    ):
        self.client = client or OllamaClient()
        self.n_candidates = n_candidates

    async def plan(
        self,
        task: str,
        world_context: str,
        n_candidates: Optional[int] = None,
    ) -> ToTResult:
        """
        Full Tree-of-Thought planning pipeline.

        1. Generate N approach candidates
        2. Expand each into a detailed plan
        3. Score each plan
        4. Select the best

        Args:
            task:          Natural language task description
            world_context: Current world state string
            n_candidates:  Override default candidate count

        Returns:
            ToTResult with selected plan and all candidates
        """
        n = n_candidates or self.n_candidates
        t_start = time.monotonic()

        result = ToTResult(
            selected_plan=None,
            all_candidates=[],
            chain_of_thought=[
                f"[ToT] ═══ Tree-of-Thought Planning ═══",
                f"[ToT] Task: \"{task}\"",
                f"[ToT] Generating {n} plan candidates...",
            ],
        )

        # ── Step 1: Generate approach candidates ───────────
        t_gen = time.monotonic()
        candidates = await self.generate_plan_candidates(
            task, world_context, n)
        result.generation_time_ms = (time.monotonic() - t_gen) * 1000

        if not candidates:
            result.chain_of_thought.append(
                "[ToT] ❌ No candidates generated")
            result.total_time_ms = (time.monotonic() - t_start) * 1000
            return result

        result.all_candidates = candidates
        for c in candidates:
            result.chain_of_thought.append(
                f"[ToT] Candidate {c.candidate_id}: {c.approach_name} "
                f"(est. success={c.estimated_success_probability:.0%})")

        # ── Step 2: Expand each candidate ──────────────────
        result.chain_of_thought.append(
            "[ToT] Expanding candidates into detailed plans...")

        for candidate in candidates:
            detailed = await self._expand_candidate(
                candidate, task, world_context)
            if detailed:
                candidate.detailed_plan = detailed

        # ── Step 3: Score each candidate ───────────────────
        t_score = time.monotonic()
        result.chain_of_thought.append(
            "[ToT] Scoring plan candidates...")

        for candidate in candidates:
            score = await self.score_plan(candidate, world_context)
            candidate.total_score = score

        result.scoring_time_ms = (time.monotonic() - t_score) * 1000

        # ── Step 4: Select best ────────────────────────────
        best = self.select_best_plan(candidates)
        result.selected_plan = best

        # Log scores
        for c in sorted(candidates, key=lambda x: x.total_score,
                        reverse=True):
            scores_str = ', '.join(
                f"{k}={v:.2f}" for k, v in c.scores.items())
            marker = " ← SELECTED" if c is best else ""
            result.chain_of_thought.append(
                f"[ToT] Plan {c.candidate_id} ({c.approach_name}): "
                f"total={c.total_score:.3f} [{scores_str}]{marker}")

        result.total_time_ms = (time.monotonic() - t_start) * 1000
        result.chain_of_thought.append(
            f"[ToT] ═══ Selected: {best.approach_name} "
            f"(score={best.total_score:.3f}, "
            f"{result.total_time_ms:.0f}ms total) ═══")

        return result

    # ── Candidate Generation ───────────────────────────────
    async def generate_plan_candidates(
        self,
        task: str,
        world_context: str,
        n_candidates: int = 3,
    ) -> List[PlanCandidate]:
        """
        Generate N different approaches to accomplish a task.

        Prompts the LLM to brainstorm multiple high-level strategies,
        each with a name, steps, estimated success probability, and
        main risk.
        """
        prompt = (
            f"You are planning a task for a 5-DoF robotic arm.\n\n"
            f"TASK: \"{task}\"\n\n"
            f"WORLD STATE:\n{world_context}\n\n"
            f"Generate exactly {n_candidates} DIFFERENT approaches to "
            f"accomplish this task. Each approach should be a "
            f"fundamentally different strategy.\n\n"
            f"Respond with ONLY a JSON object:\n"
            f'{{\n'
            f'  "approaches": [\n'
            f'    {{\n'
            f'      "approach_name": "short_descriptive_name",\n'
            f'      "high_level_steps": ["step1", "step2", ...],\n'
            f'      "estimated_success_probability": 0.0 to 1.0,\n'
            f'      "main_risk": "what could go wrong"\n'
            f'    }}\n'
            f'  ]\n'
            f'}}'
        )

        response, success = await self.client.generate(
            prompt=prompt,
            response_format="json",
            temperature=0.3,  # Slightly higher for creativity
        )

        if not success:
            logger.error("Failed to generate plan candidates")
            return []

        try:
            data = json.loads(response)
            approaches = data.get('approaches', [])
        except (json.JSONDecodeError, KeyError):
            logger.error(f"Invalid candidate JSON: {response[:200]}")
            return []

        candidates = []
        for i, approach in enumerate(approaches[:n_candidates]):
            candidates.append(PlanCandidate(
                candidate_id=i + 1,
                approach_name=approach.get('approach_name', f'plan_{i+1}'),
                high_level_steps=approach.get('high_level_steps', []),
                estimated_success_probability=float(
                    approach.get('estimated_success_probability', 0.5)),
                main_risk=approach.get('main_risk', 'unknown'),
            ))

        return candidates

    # ── Candidate Expansion ────────────────────────────────
    async def _expand_candidate(
        self,
        candidate: PlanCandidate,
        task: str,
        world_context: str,
    ) -> Optional[dict]:
        """
        Expand a high-level approach into a detailed executable plan
        matching the standard plan JSON schema.
        """
        steps_str = '\n'.join(
            f"  {i+1}. {s}"
            for i, s in enumerate(candidate.high_level_steps))

        prompt = (
            f"You are expanding a high-level plan into detailed "
            f"executable actions for a 5-DoF robotic arm.\n\n"
            f"TASK: \"{task}\"\n"
            f"APPROACH: {candidate.approach_name}\n"
            f"HIGH-LEVEL STEPS:\n{steps_str}\n\n"
            f"WORLD STATE:\n{world_context}\n\n"
            f"Expand this into a detailed plan. Each action must "
            f"specify the agent, skill (if any), parameters, "
            f"reasoning, preconditions, and expected outcome.\n\n"
            f"Respond with ONLY a JSON object matching this schema:\n"
            f'{{\n'
            f'  "goal": "string",\n'
            f'  "subgoals": ["subgoal1", ...],\n'
            f'  "actions": [\n'
            f'    {{\n'
            f'      "step": 1,\n'
            f'      "agent": "AgentName",\n'
            f'      "skill": "skill_name or null",\n'
            f'      "parameters": {{}},\n'
            f'      "reasoning": "why",\n'
            f'      "preconditions": ["condition"],\n'
            f'      "expected_outcome": "what happens"\n'
            f'    }}\n'
            f'  ],\n'
            f'  "confidence": 0.0 to 1.0,\n'
            f'  "ambiguity_notes": "",\n'
            f'  "requires_approval": false,\n'
            f'  "alternative_interpretations": []\n'
            f'}}'
        )

        response, success = await self.client.generate(
            prompt=prompt,
            response_format="json",
            temperature=0.1,
        )

        if not success:
            return None

        try:
            return json.loads(response)
        except json.JSONDecodeError:
            return None

    # ── Plan Scoring ───────────────────────────────────────
    async def score_plan(
        self,
        plan: PlanCandidate,
        world_context: str,
    ) -> float:
        """
        Score a plan candidate on multiple dimensions:
          feasibility:   all steps reachable + objects exist (0-1)
          efficiency:    fewer steps preferred (0-1)
          risk:          inverse of main_risk severity (0-1)
          precondition:  all preconditions checkable (0-1)

        total = 0.4*feasibility + 0.2*efficiency +
                0.3*risk + 0.1*precondition
        """
        # ── Feasibility score ──────────────────────────────
        feasibility = plan.estimated_success_probability

        # Bonus if detailed plan exists and has valid structure
        if plan.detailed_plan:
            actions = plan.detailed_plan.get('actions', [])
            if actions:
                # Check that actions have required fields
                valid_actions = sum(
                    1 for a in actions
                    if 'agent' in a and 'reasoning' in a)
                feasibility = (feasibility + valid_actions / len(actions)) / 2

        # ── Efficiency score ───────────────────────────────
        n_steps = len(plan.high_level_steps)
        if plan.detailed_plan:
            n_steps = len(plan.detailed_plan.get('actions', []))

        # Fewer steps = better (normalize: 3 steps = 1.0, 15 steps = 0.2)
        efficiency = max(0.2, min(1.0, 1.0 - (n_steps - 3) * 0.066))

        # ── Risk score ─────────────────────────────────────
        risk_severity = {
            'collision': 0.2,
            'unreachable': 0.3,
            'slip': 0.5,
            'unstable': 0.4,
            'occlusion': 0.6,
            'lighting': 0.7,
            'unknown': 0.5,
        }
        risk = 0.5  # default
        risk_lower = plan.main_risk.lower()
        for keyword, severity in risk_severity.items():
            if keyword in risk_lower:
                risk = severity
                break
        risk = 1.0 - risk  # Invert: low risk → high score

        # ── Precondition score ─────────────────────────────
        precondition = 0.8  # Default
        if plan.detailed_plan:
            actions = plan.detailed_plan.get('actions', [])
            preconditions_present = sum(
                1 for a in actions
                if a.get('preconditions') and len(a['preconditions']) > 0)
            if actions:
                precondition = preconditions_present / len(actions)

        # ── Compute total ──────────────────────────────────
        plan.scores = {
            'feasibility': round(feasibility, 3),
            'efficiency': round(efficiency, 3),
            'risk': round(risk, 3),
            'precondition': round(precondition, 3),
        }

        total = (
            self.FEASIBILITY_WEIGHT * feasibility +
            self.EFFICIENCY_WEIGHT * efficiency +
            self.RISK_WEIGHT * risk +
            self.PRECONDITION_WEIGHT * precondition
        )

        return round(total, 4)

    # ── Plan Selection ─────────────────────────────────────
    @staticmethod
    def select_best_plan(
        candidates: List[PlanCandidate],
    ) -> PlanCandidate:
        """
        Select the highest-scoring plan candidate.
        Logs all scores for transparency.
        """
        if not candidates:
            raise ValueError("No candidates to select from")

        return max(candidates, key=lambda c: c.total_score)
