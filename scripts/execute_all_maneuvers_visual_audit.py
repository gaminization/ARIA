#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
PROJECT ARIA: MASTER AUTONOMOUS MULTI-WORLD VISUAL AUDIT ORCHESTRATOR
═══════════════════════════════════════════════════════════════════════════════
Sequentially launches and executes all 10 physical manipulation maneuvers
across 3 worlds via pure autonomous natural language commands to /aria/command:

  World 1: aria_tester_workspace.world (Maneuvers 1 - 6)
  World 2: aria_industrial_workcell.world (Maneuvers 7 - 8)
  World 3: aria_workspace.world (Maneuvers 9 - 10)

Constraints strictly enforced:
  - 100% Autonomous execution (zero hardcoded joint angles or coordinates).
  - Controlled servo velocity: <= 15.0 deg/s for realistic, graceful motion.
  - Genuine in-hand physical elevation & ODE attachment verified in Gazebo.
  - Synchronized 3-camera HUDs captured in mid-air and saved to artifact dir.
═══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import time
import subprocess

WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARTIFACT_DIR = os.environ.get(
    "ARIA_AUDIT_ARTIFACT_DIR",
    os.path.join(WORKSPACE_ROOT, "runs", "visual_audit"),
)

SCRIPTS = [
    ("SUITE 1: aria_tester_workspace.world", "scripts/run_world1_tester_autonomous.py", [
        "visual_01_wave_gesture.png",
        "visual_02_coffee_drinking_mug.png",
        "visual_03_jenga_solve_extract.png",
        "visual_04_cleargrasp_glass_pick.png",
        "visual_05_affordance_pan_handle.png",
        "visual_06_pick_place_plate_duck.png",
    ]),
    ("SUITE 2: aria_industrial_workcell.world", "scripts/run_world2_industrial_autonomous.py", [
        "visual_07_industrial_conveyor_sort.png",
        "visual_08_industrial_button_press.png",
    ]),
    ("SUITE 3: aria_workspace.world", "scripts/run_world3_workspace_autonomous.py", [
        "visual_09_stacking_cubes.png",
        "visual_10_sorting_dish.png",
    ]),
]


def main():
    print("═" * 80)
    print("  🚀 PROJECT ARIA: AUTHORITATIVE AUTONOMOUS MULTI-WORLD VISUAL AUDIT")
    print("  Architecture: Natural Language Command -> LLM Planner -> Dynamic Perception -> Skill Execution")
    print(f"  Artifact Directory: {ARTIFACT_DIR}")
    print("═" * 80)

    total_start = time.time()
    results = {}

    for suite_name, script_rel, expected_artifacts in SCRIPTS:
        print(f"\n{'━' * 80}")
        print(f"  ▶️  STARTING {suite_name}")
        print(f"      Script: {script_rel}")
        print(f"{'━' * 80}")

        script_path = os.path.join(WORKSPACE_ROOT, script_rel)

        env = os.environ.copy()
        env["GAZEBO_IP"] = "127.0.0.1"
        env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:11345"
        env["DISPLAY"] = os.environ.get("DISPLAY", ":1")
        env["ARIA_AUDIT_ARTIFACT_DIR"] = ARTIFACT_DIR

        cmd = (
            f'source "{WORKSPACE_ROOT}/install/setup.bash" && '
            f'exec "{sys.executable}" "{script_path}"'
        )
        t0 = time.time()
        ret = subprocess.call(["bash", "-c", cmd], env=env)
        duration = time.time() - t0

        artifacts_found = []
        for art in expected_artifacts:
            full_p = os.path.join(ARTIFACT_DIR, art)
            if os.path.exists(full_p) and os.path.getsize(full_p) > 50000:
                artifacts_found.append(art)

        results[suite_name] = {
            "returncode": ret,
            "duration": duration,
            "artifacts_found": len(artifacts_found),
            "artifacts_expected": len(expected_artifacts)
        }
        print(f"\n  {suite_name} finished in {duration:.1f}s (Artifacts: {len(artifacts_found)}/{len(expected_artifacts)})")

    total_duration = time.time() - total_start
    print("\n" + "═" * 80)
    print("  🏁 MASTER AUTONOMOUS VISUAL AUDIT COMPLETE SUMMARY")
    print("═" * 80)
    print(f"{'SUITE':<45} | {'STATUS':<8} | {'TIME':<8} | {'ARTIFACTS':<10}")
    print("─" * 80)
    for suite, data in results.items():
        status = "PASSED" if data["returncode"] == 0 and data["artifacts_found"] == data["artifacts_expected"] else "CHECK"
        art_str = f"{data['artifacts_found']}/{data['artifacts_expected']}"
        print(f"{suite:<45} | {status:<8} | {data['duration']:>6.1f}s | {art_str:<10}")
    print("─" * 80)
    print(f"Total Audit Execution Time: {total_duration:.1f}s")
    print(f"All images saved to: {ARTIFACT_DIR}")
    return 0 if all(
        data["returncode"] == 0
        and data["artifacts_found"] == data["artifacts_expected"]
        for data in results.values()
    ) else 1


if __name__ == "__main__":
    sys.exit(main())
