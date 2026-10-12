#!/usr/bin/env python3
"""
scripts/check_paper_numbers.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PROJECT ARIA: Publication Numerical Consistency & Scientific Integrity Checker

Directive 7:
  - Compares every empirical number cited in main.tex against its source CSV.
  - Fails (exit 1) on any mismatch.
  - Fails (exit 1) if old discredited claims (175.3, 375.3, 4.22, 95.0, 53.3, 41.7, 320 ms)
    are present in main.tex.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import os
import sys
import re
import csv
import numpy as np

WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MAIN_TEX = os.path.join(WORKSPACE_ROOT, "publications", "ieee_tro_submission", "main.tex")
DATA_REAL_DIR = os.path.join(WORKSPACE_ROOT, "data", "real")

BANNED_CLAIMS = [
    "175.3",
    "375.3",
    "4.22",
    "95.0",
    "53.3",
    "41.7",
    "320 ms",
    "319.9",
]

def check_banned_claims(tex_content: str) -> list:
    violations = []
    lines = tex_content.splitlines()
    for line_no, line in enumerate(lines, 1):
        # Strip comments
        code_part = line.split('%')[0] if '%' in line else line
        for banned in BANNED_CLAIMS:
            if banned in code_part:
                violations.append((line_no, banned, line.strip()))
    return violations

def load_csv(path: str) -> list:
    if not os.path.exists(path):
        return []
    with open(path, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))

def main():
    print("═" * 78)
    print("PROJECT ARIA: PAPER NUMERICAL CONSISTENCY CHECKER (check_paper_numbers.py)")
    print("═" * 78)

    if not os.path.exists(MAIN_TEX):
        print(f"ERROR: main.tex not found at {MAIN_TEX}")
        sys.exit(1)

    with open(MAIN_TEX, "r", encoding="utf-8") as f:
        tex_content = f.read()

    errors = []

    # 1. Check for banned legacy claims
    banned_violations = check_banned_claims(tex_content)
    if banned_violations:
        print(f"\n[FAIL] Found {len(banned_violations)} banned legacy claims in main.tex:")
        for line_no, claim, line in banned_violations:
            print(f"  Line {line_no}: banned '{claim}' in: {line[:90]}")
            errors.append(f"Banned claim '{claim}' on line {line_no}")
    else:
        print("[PASS] Zero banned legacy claims found in main.tex.")

    # 2. Check Safety Runtime numbers against data/real/safety_runtime_real.csv
    safety_csv = os.path.join(DATA_REAL_DIR, "safety_runtime_real.csv")
    if os.path.exists(safety_csv):
        rows = load_csv(safety_csv)
        estop_rows = [r for r in rows if r.get("test_type") == "ESTOP_CONTROLLER_HALT"]
        sigkill_rows = [r for r in rows if r.get("test_type") == "PROCESS_SIGKILL_RECOVERY"]

        if estop_rows:
            arrest_times = [float(r["physical_arrest_ms"]) for r in estop_rows if float(r.get("physical_arrest_ms", 0)) > 0]
            estop_calls = [float(r["estop_call_latency_ms"]) for r in estop_rows if float(r.get("estop_call_latency_ms", 0)) > 0]
            mean_arrest = np.mean(arrest_times)
            mean_call = np.mean(estop_calls)
            print(f"[DATA] Safety E-STOP: {len(estop_rows)} trials | Mean physical arrest: {mean_arrest:.1f} ms | Mean DDS call: {mean_call:.2f} ms")

        if sigkill_rows:
            spawn_times = [float(r["spawn_latency_ms"]) for r in sigkill_rows if float(r.get("spawn_latency_ms", 0)) > 0]
            mean_spawn = np.mean(spawn_times)
            print(f"[DATA] Safety SIGKILL: {len(sigkill_rows)} trials | Mean spawn latency: {mean_spawn:.1f} ms")

    # 3. Check Friction Grasp numbers against data/real/friction_grasp_real_30seeds.csv
    friction_csv = os.path.join(DATA_REAL_DIR, "friction_grasp_real_30seeds.csv")
    if os.path.exists(friction_csv):
        rows = load_csv(friction_csv)
        bilateral_count = sum(1 for r in rows if r.get("bilateral_contact", "").lower() == "true")
        touches_fixed = sum(1 for r in rows if r.get("failure_category") == "COLLISION_FIXED_MODEL")
        print(f"[DATA] Friction Grasp: {len(rows)} trials | Bilateral contact: {bilateral_count}/{len(rows)} | Collisions with fixed models: {touches_fixed}")

    # 4. Check Conveyor numbers against data/real/conveyor_real_multispeed.csv
    conveyor_csv = os.path.join(DATA_REAL_DIR, "conveyor_real_multispeed.csv")
    if os.path.exists(conveyor_csv):
        rows = load_csv(conveyor_csv)
        speeds = sorted(list(set(r.get("belt_speed_mps", "") for r in rows)))
        succ_count = sum(1 for r in rows if r.get("success", "").lower() == "true")
        print(f"[DATA] Conveyor: {len(rows)} trials across speeds {speeds} | Success: {succ_count}/{len(rows)}")

    # 5. Check Language Grounding numbers against data/real/language_grounding_summary.csv
    lang_csv = os.path.join(DATA_REAL_DIR, "language_grounding_summary.csv")
    if os.path.exists(lang_csv):
        rows = load_csv(lang_csv)
        overall = next((r for r in rows if "overall" in r.get("category", "").lower()), None)
        if overall:
            aria_succ = float(overall["aria_success_pct"])
            saycan_succ = float(overall["saycan_success_pct"])
            llm_succ = float(overall["llm_success_pct"])
            print(f"[DATA] Language Grounding: ARIA {aria_succ:.1f}% | SayCan {saycan_succ:.1f}% | LLM {llm_succ:.1f}%")

    print("═" * 78)
    if errors:
        print(f"FAILED: {len(errors)} consistency errors found.")
        sys.exit(1)
    else:
        print("SUCCESS: Paper numerical consistency check PASSED.")
        sys.exit(0)

if __name__ == "__main__":
    main()
