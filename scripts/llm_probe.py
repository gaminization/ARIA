#!/usr/bin/env python3
"""Probe the planner LLM offline: print the raw response and validation result for a command."""
import asyncio
import json
import sys
import time

sys.path.insert(0, '/home/gaminizer/Projects/ARIA/arm_planner')
from arm_planner.llm_client import OllamaClient
from arm_planner.llm_planning_agent import validate_plan_json, _load_prompt

cmd = sys.argv[1] if len(sys.argv) > 1 else 'pick up the red mug'


async def main():
    c = OllamaClient()
    sysp = _load_prompt('system_prompt_planner.txt')
    ctx = ("Known objects:\n  - mug_1 (class=mug, pos=[0.22,-0.06,0.608], state=Detected)\n"
           "  - duck_1 (class=duck, pos=[0.27,0.01,0.608], state=Detected)\n"
           "Joint state: unknown\nSpatial relations: none computed")
    prompt = (f'COMMAND: "{cmd}"\n\nCURRENT WORLD STATE:\n{ctx}\n\n'
              'Generate a plan to accomplish this command. Respond with ONLY valid JSON matching the schema.')
    t = time.time()
    r, ok = await c.generate(prompt=prompt, model=c.default_model, system_prompt=sysp,
                             response_format='json', temperature=0.1)
    print(f'ok={ok} {time.time() - t:.1f}s len={len(r)}')
    print(r[:3000])
    try:
        print(validate_plan_json(json.loads(r)))
    except Exception as e:
        print('ERR', e)

asyncio.run(main())
