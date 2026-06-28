#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA LLM Planning Agent — arm_agents entry point
Forwards to the full implementation in arm_planner.
═══════════════════════════════════════════════════════════════
"""
from arm_planner.llm_planning_agent import LLMPlanningAgent, main

__all__ = ['LLMPlanningAgent', 'main']

if __name__ == "__main__":
    main()
