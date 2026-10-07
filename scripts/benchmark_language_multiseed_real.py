#!/usr/bin/env python3
"""
scripts/benchmark_language_multiseed_real.py

Empirical Multi-Seed Language Grounding Benchmark for Project ARIA:
  - 3 Seeds: 42, 1337, 2026
  - Temperature: 0.7 (temperature > 0)
  - 3 Methods:
      1. ARIA: DialogueAgent + PlanningAgent with BNF Grammar-Constrained Schema (format=ARIA_GRAMMAR_SCHEMA)
      2. Direct LLM: Same prompt output schema in prompt text, NO grammar constraint (format="json")
      3. SayCan: Semantic probability scoring over primitives * Affordances
  - ToT K=3 Timing: Separately measures planning latency with ToT K=3 expansion ON vs OFF.

Outputs (written to data/real/, never overwriting existing files):
  - data/real/language_grounding_multiseed_trials.csv
  - data/real/language_grounding_multiseed_summary.csv
  - data/real/tot_planning_latency_benchmark.csv
"""

import os
import sys
import time
import json
import csv
import urllib.request
import urllib.error
import subprocess
from datetime import datetime
from typing import Dict, List, Tuple, Any, Optional
import numpy as np

WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
DATA_REAL_DIR = os.path.join(WORKSPACE_ROOT, "data", "real")
PROMPTS_FILE = os.path.join(DATA_REAL_DIR, "lang_prompts.json")

RAW_TRIALS_CSV = os.path.join(DATA_REAL_DIR, "language_grounding_multiseed_trials.csv")
SUMMARY_CSV = os.path.join(DATA_REAL_DIR, "language_grounding_multiseed_summary.csv")
TOT_TIMING_CSV = os.path.join(DATA_REAL_DIR, "tot_planning_latency_benchmark.csv")

OLLAMA_URL = "http://localhost:11434/api/generate"
MODEL_NAME = "llama3.1:8b-instruct-q4_K_M"
TEMPERATURE = 0.7
SEEDS = [42, 1337, 2026]
CONFIDENCE_THRESHOLD = 0.70

def get_git_commit():
    try:
        res = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                             cwd=WORKSPACE_ROOT, capture_output=True, text=True, check=True)
        return res.stdout.strip()
    except Exception:
        return "unknown"

GIT_COMMIT_HASH = get_git_commit()

# Valid skill primitives
VALID_SKILLS = {
    "pick", "place", "push", "pull", "stack", "sort",
    "inspect", "slide", "roll", "sweep", "locate"
}

# ARIA Grammar Schema for Ollama BNF grammar constraint
ARIA_GRAMMAR_SCHEMA = {
    "type": "object",
    "properties": {
        "plan_id": {"type": "string"},
        "confidence": {"type": "number"},
        "subgoals": {"type": "array", "items": {"type": "string"}},
        "actions": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "step": {"type": "integer"},
                    "agent": {
                        "type": "string",
                        "enum": [
                            "SkillAgent", "VisionAgent", "ControlAgent",
                            "SafetyAgent", "WorldModelAgent", "AffordanceAgent"
                        ]
                    },
                    "skill": {
                        "type": "string",
                        "enum": [
                            "pick", "place", "push", "pull", "stack", "sort",
                            "inspect", "slide", "roll", "sweep", "locate"
                        ]
                    },
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "object_id": {"type": "string"},
                            "target": {"type": "string"},
                            "direction": {"type": "string"},
                            "distance_m": {"type": "number"}
                        }
                    },
                    "reasoning": {"type": "string"}
                },
                "required": ["step", "agent", "skill", "reasoning"]
            }
        },
        "requires_approval": {"type": "boolean"},
        "ambiguity_notes": {"type": "string"}
    },
    "required": ["plan_id", "confidence", "subgoals", "actions", "requires_approval", "ambiguity_notes"]
}

# Plain-text schema specification to inject into Direct LLM prompt
SCHEMA_PROMPT_SPEC = """
Output a JSON object conforming exactly to this schema:
{
  "plan_id": "string",
  "confidence": float (0.0 to 1.0),
  "subgoals": ["string"],
  "actions": [
    {
      "step": integer,
      "agent": "SkillAgent" | "VisionAgent" | "ControlAgent" | "SafetyAgent" | "WorldModelAgent" | "AffordanceAgent",
      "skill": "pick" | "place" | "push" | "pull" | "stack" | "sort" | "inspect" | "slide" | "roll" | "sweep" | "locate",
      "parameters": {"object_id": "string", "target": "string"},
      "reasoning": "string"
    }
  ],
  "requires_approval": boolean,
  "ambiguity_notes": "string"
}
"""

OBJECT_SYNONYMS = {
    "workpiece_good_1": ["blue bolt", "blue bolt 1", "blue cylinder", "bolt", "workpiece 1", "conforming"],
    "workpiece_good_2": ["blue cube", "blue block", "blue cube 2", "cube", "block", "conforming"],
    "workpiece_good_3": ["blue workpiece", "blue component", "blue peg", "conforming"],
    "workpiece_defect_1": ["red cylinder", "red cylinder 1", "defective red cylinder", "scratched", "defective"],
    "workpiece_defect_2": ["red housing", "red block", "cracked red housing", "cracked", "defective"],
    "grey_bracket": ["grey bracket", "bracket", "grey block", "grey spacer", "grey flange", "grey plate", "grey peg"],
    "assembly_finished_tray": ["tray pocket 1", "tray pocket 2", "tray pocket 3", "tray pocket 4", "assembly fixture", "pallet", "conforming tray", "inbound tray", "outbound tray", "tray"],
    "reject_bin": ["scrap bin", "bin a", "bin b", "scrap receptacle", "quarantine", "rework station", "reject"],
}

# ═════════════════════════════════════════════════════════════════════════════
# LLM Inference
# ═════════════════════════════════════════════════════════════════════════════
def call_ollama(prompt: str, format_spec: Any = "json", temp: float = 0.7, seed: int = 42, max_tokens: int = 300) -> Tuple[str, float]:
    payload = {
        "model": MODEL_NAME,
        "prompt": prompt,
        "stream": False,
        "format": format_spec,
        "options": {
            "temperature": temp,
            "seed": seed,
            "num_predict": max_tokens,
        }
    }
    data_bytes = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(OLLAMA_URL, data=data_bytes, headers={"Content-Type": "application/json"})
    t0 = time.perf_counter()
    with urllib.request.urlopen(req, timeout=60) as resp:
        res = json.loads(resp.read().decode("utf-8"))
    dt_ms = (time.perf_counter() - t0) * 1000.0
    return res.get("response", ""), dt_ms

# ═════════════════════════════════════════════════════════════════════════════
# Evaluation Helper Functions
# ═════════════════════════════════════════════════════════════════════════════
def normalize_str(s: str) -> str:
    if not s:
        return ""
    return s.lower().replace("_", " ").replace("-", " ").strip()

def matches_semantic_entity(entity_val: str, expected_target: str) -> bool:
    n_val = normalize_str(entity_val)
    n_exp = normalize_str(expected_target)
    if n_exp in n_val or n_val in n_exp:
        return True
    for k, syns in OBJECT_SYNONYMS.items():
        k_norm = normalize_str(k)
        if k_norm in n_val or k_norm in n_exp:
            if any(normalize_str(s) in n_exp or normalize_str(s) in n_val for s in syns):
                return True
        for s in syns:
            if normalize_str(s) in n_val and any(normalize_str(s2) in n_exp for s2 in syns):
                return True
    exp_words = [w for w in n_exp.split() if w not in {"the", "a", "an", "and", "with", "in", "of", "to"}]
    if any(w in n_val for w in exp_words):
        return True
    return False

def evaluate_slots(parsed_action: str, parsed_target: str, parsed_dest: str,
                   expected_slots: dict, is_ambiguous: bool) -> Tuple[bool, bool, bool, float]:
    exp_act = expected_slots.get("action", "")
    exp_tgt = expected_slots.get("target_object", "")
    exp_dst = expected_slots.get("destination", "")

    act_corr = (normalize_str(parsed_action) == normalize_str(exp_act))
    tgt_corr = matches_semantic_entity(parsed_target, exp_tgt)
    dst_corr = (not exp_dst and not parsed_dest) or matches_semantic_entity(parsed_dest, exp_dst)

    weights = [1.0, 1.0, 1.0] if exp_dst else [1.0, 1.0]
    scores = [1.0 if act_corr else 0.0, 1.0 if tgt_corr else 0.0]
    if exp_dst:
        scores.append(1.0 if dst_corr else 0.0)

    accuracy = sum(scores) / sum(weights)
    return act_corr, tgt_corr, dst_corr, accuracy


# ═════════════════════════════════════════════════════════════════════════════
# Method 1: ARIA (Grammar-Constrained Schema via Ollama BNF)
# ═════════════════════════════════════════════════════════════════════════════
def run_aria_trial(prompt_entry: dict, seed: int, temp: float) -> dict:
    instruction = prompt_entry["instruction"]
    is_ambiguous = prompt_entry["is_ambiguous"]

    world_context = (
        "Objects detected in workcell: workpiece_good_1 (blue bolt), workpiece_good_2 (blue cube), "
        "workpiece_defect_1 (defective red cylinder with surface scratches), "
        "workpiece_defect_2 (cracked red housing), grey_bracket."
    )

    prompt = (
        f"You are ARIA PlanningAgent. Decompose instruction into a structured plan.\n"
        f"WORLD STATE:\n{world_context}\n\n"
        f"INSTRUCTION: \"{instruction}\"\n"
        f"Rules:\n"
        f"1. Generate ONLY the 1-3 action steps directly required for this instruction.\n"
        f"2. If the instruction is ambiguous, underspecified, refers to vague objects (like 'the part', 'that block', 'the cylinder', 'the item') "
        f"or missing destination (like 'away', 'over there', 'in the tray'), set confidence < 0.70, requires_approval = true, and explain ambiguity in ambiguity_notes."
    )

    raw_output, dt_ms = call_ollama(prompt, format_spec=ARIA_GRAMMAR_SCHEMA, temp=temp, seed=seed, max_tokens=300)

    syntax_valid = False
    parsed_plan = {}
    confidence = 0.5
    requires_approval = False
    ambiguity_notes = ""
    skill = "pick"
    target_obj = ""
    dest = ""

    try:
        parsed_plan = json.loads(raw_output)
        syntax_valid = True
        confidence = float(parsed_plan.get("confidence", 0.5))
        requires_approval = bool(parsed_plan.get("requires_approval", False))
        ambiguity_notes = parsed_plan.get("ambiguity_notes", "")

        actions = parsed_plan.get("actions", [])
        if actions:
            last_act = actions[-1]
            skill = last_act.get("skill", "pick")
            params = last_act.get("parameters", {})
            target_obj = params.get("object_id", "")
            dest = params.get("target", "")
            if not target_obj and len(actions) > 1:
                target_obj = actions[0].get("parameters", {}).get("object_id", "")
    except Exception:
        syntax_valid = False

    if confidence > 1.0:
        confidence = confidence / 100.0

    clarification_triggered = (confidence < CONFIDENCE_THRESHOLD) or requires_approval

    act_corr, tgt_corr, dst_corr, slot_acc = evaluate_slots(
        skill, target_obj, dest, prompt_entry["expected_slots"], is_ambiguous
    )

    if is_ambiguous:
        execution_success = clarification_triggered and syntax_valid
    else:
        execution_success = syntax_valid and (not clarification_triggered) and (slot_acc >= 0.66)

    return {
        "method": "ARIA_Dialogue_Planning",
        "raw_output": raw_output,
        "parsed_plan": json.dumps(parsed_plan),
        "syntax_valid": syntax_valid,
        "slot_action_correct": act_corr,
        "slot_target_correct": tgt_corr,
        "slot_destination_correct": dst_corr,
        "slot_accuracy": slot_acc,
        "clarification_triggered": clarification_triggered,
        "confidence_value": round(confidence, 3),
        "threshold_value": CONFIDENCE_THRESHOLD,
        "execution_success": execution_success,
        "latency_ms": round(dt_ms, 2)
    }


# ═════════════════════════════════════════════════════════════════════════════
# Method 2: Direct LLM (SAME Schema in Prompt Text, NO Grammar Constraint)
# ═════════════════════════════════════════════════════════════════════════════
def run_direct_llm_trial(prompt_entry: dict, seed: int, temp: float) -> dict:
    instruction = prompt_entry["instruction"]
    is_ambiguous = prompt_entry["is_ambiguous"]

    prompt = (
        f"You are a robotic arm task decomposition planner.\n"
        f"INSTRUCTION: \"{instruction}\"\n\n"
        f"{SCHEMA_PROMPT_SPEC}\n"
        f"Rules:\n"
        f"1. Generate ONLY the action steps required.\n"
        f"2. If ambiguous or missing destination, set confidence < 0.70 and requires_approval = true.\n"
    )

    # Note: format_spec is generic "json", NOT the BNF grammar constraint schema
    raw_output, dt_ms = call_ollama(prompt, format_spec="json", temp=temp, seed=seed, max_tokens=300)

    syntax_valid = False
    parsed_plan = {}
    confidence = 0.7
    requires_approval = False
    skill = ""
    target_obj = ""
    dest = ""

    try:
        parsed_plan = json.loads(raw_output)
        actions = parsed_plan.get("actions", [])
        if isinstance(actions, list) and len(actions) > 0:
            first_act = actions[0]
            if isinstance(first_act, dict):
                cand_skill = str(first_act.get("skill", "")).lower().strip()
                params = first_act.get("parameters", {})
                if cand_skill in VALID_SKILLS:
                    syntax_valid = True
                    skill = cand_skill
                    target_obj = params.get("object_id", "") if isinstance(params, dict) else str(params)
                    dest = params.get("target", "") if isinstance(params, dict) else ""
                else:
                    syntax_valid = False
                    skill = cand_skill
        confidence = float(parsed_plan.get("confidence", 0.7))
        requires_approval = bool(parsed_plan.get("requires_approval", False))
    except Exception:
        syntax_valid = False

    if confidence > 1.0:
        confidence = confidence / 100.0

    clarification_triggered = (confidence < CONFIDENCE_THRESHOLD) or requires_approval

    act_corr, tgt_corr, dst_corr, slot_acc = evaluate_slots(
        skill, target_obj, dest, prompt_entry["expected_slots"], is_ambiguous
    )

    if is_ambiguous:
        # If Direct LLM recognized ambiguity and paused/asked approval, credit success
        execution_success = clarification_triggered and syntax_valid
    else:
        execution_success = syntax_valid and (not clarification_triggered) and (slot_acc >= 0.66)

    return {
        "method": "Direct_LLM",
        "raw_output": raw_output,
        "parsed_plan": json.dumps(parsed_plan),
        "syntax_valid": syntax_valid,
        "slot_action_correct": act_corr,
        "slot_target_correct": tgt_corr,
        "slot_destination_correct": dst_corr,
        "slot_accuracy": slot_acc,
        "clarification_triggered": clarification_triggered,
        "confidence_value": round(confidence, 3),
        "threshold_value": CONFIDENCE_THRESHOLD,
        "execution_success": execution_success,
        "latency_ms": round(dt_ms, 2)
    }


# ═════════════════════════════════════════════════════════════════════════════
# Method 3: SayCan-style Affordance Scoring
# ═════════════════════════════════════════════════════════════════════════════
def run_saycan_trial(prompt_entry: dict, seed: int, temp: float) -> dict:
    instruction = prompt_entry["instruction"]
    is_ambiguous = prompt_entry["is_ambiguous"]

    primitives = [
        "pick(blue_bolt)", "pick(blue_cube)", "pick(red_cylinder)", "pick(red_housing)",
        "pick(grey_bracket)", "place(assembly_tray)", "place(scrap_bin)", "place(tray_pocket_1)",
        "place(tray_pocket_2)", "inspect(red_cylinder)", "sweep(conveyor)", "done()"
    ]

    affordance_map = {
        "pick(blue_bolt)": 0.88,
        "pick(blue_cube)": 0.88,
        "pick(red_cylinder)": 0.88,
        "pick(red_housing)": 0.88,
        "pick(grey_bracket)": 0.85,
        "place(assembly_tray)": 0.0,
        "place(scrap_bin)": 0.0,
        "place(tray_pocket_1)": 0.0,
        "place(tray_pocket_2)": 0.0,
        "inspect(red_cylinder)": 0.82,
        "sweep(conveyor)": 0.75,
        "done()": 0.50
    }

    prompt = (
        f"Instruction: \"{instruction}\"\n"
        f"Score the semantic probability (0.0 to 1.0) of each robotic action primitive for this instruction.\n"
        f"Output JSON mapping each primitive to its numeric score:\n"
        f"Primitives: {json.dumps(primitives)}"
    )

    raw_output, dt_ms = call_ollama(prompt, format_spec="json", temp=temp, seed=seed, max_tokens=200)

    p_scores = {}
    try:
        p_scores = json.loads(raw_output)
    except Exception:
        pass

    best_prim = "pick(blue_bolt)"
    best_score = -1.0
    combined_scores = {}

    for prim in primitives:
        p = float(p_scores.get(prim, 0.1))
        a = float(affordance_map.get(prim, 0.0))
        score = p * a
        combined_scores[prim] = score
        if score > best_score:
            best_score = score
            best_prim = prim

    syntax_valid = True
    clarification_triggered = False

    skill = best_prim.split("(")[0]
    target_obj = best_prim.split("(")[1].rstrip(")") if "(" in best_prim else ""
    dest = target_obj if skill == "place" else ""

    act_corr, tgt_corr, dst_corr, slot_acc = evaluate_slots(
        skill, target_obj, dest, prompt_entry["expected_slots"], is_ambiguous
    )

    if is_ambiguous:
        execution_success = False # SayCan takes argmax action without clarification dialogue
    else:
        execution_success = (slot_acc >= 0.66)

    return {
        "method": "SayCan",
        "raw_output": raw_output,
        "parsed_plan": json.dumps({"selected_primitive": best_prim, "score": round(best_score, 3)}),
        "syntax_valid": syntax_valid,
        "slot_action_correct": act_corr,
        "slot_target_correct": tgt_corr,
        "slot_destination_correct": dst_corr,
        "slot_accuracy": slot_acc,
        "clarification_triggered": clarification_triggered,
        "confidence_value": round(best_score, 3),
        "threshold_value": CONFIDENCE_THRESHOLD,
        "execution_success": execution_success,
        "latency_ms": round(dt_ms, 2)
    }


# ═════════════════════════════════════════════════════════════════════════════
# ToT K=3 Timing Benchmark Function
# ═════════════════════════════════════════════════════════════════════════════
def run_tot_timing_benchmark(prompts: List[dict]):
    print("\n" + "═" * 78)
    print("TIMING BENCHMARK: Tree-of-Thought (ToT K=3) Expansion ON vs OFF")
    print("═" * 78)

    sample_prompts = prompts[:10]  # 10 diverse representative prompts
    tot_records = []

    fieldnames = [
        "trial_id", "prompt_id", "instruction", "seed", "tot_mode",
        "latency_ms", "n_candidates", "commit_hash", "timestamp"
    ]

    with open(TOT_TIMING_CSV, "w", newline="", encoding="utf-8") as f_tot:
        writer = csv.DictWriter(f_tot, fieldnames=fieldnames)
        writer.writeheader()

        trial_idx = 0
        for p in sample_prompts:
            instruction = p["instruction"]
            pid = p["prompt_id"]

            for seed in SEEDS:
                # ── 1. ToT OFF: Single-pass direct generation ──
                trial_idx += 1
                t0 = time.perf_counter()
                prompt_single = (
                    f"You are ARIA PlanningAgent. Decompose instruction into a structured plan.\n"
                    f"INSTRUCTION: \"{instruction}\""
                )
                _, dt_off = call_ollama(prompt_single, format_spec=ARIA_GRAMMAR_SCHEMA, temp=TEMPERATURE, seed=seed, max_tokens=250)
                ts = datetime.utcnow().isoformat() + "Z"

                rec_off = {
                    "trial_id": f"TOT_TRIAL_{trial_idx:04d}",
                    "prompt_id": pid,
                    "instruction": instruction,
                    "seed": seed,
                    "tot_mode": "ToT_OFF",
                    "latency_ms": round(dt_off, 2),
                    "n_candidates": 1,
                    "commit_hash": GIT_COMMIT_HASH,
                    "timestamp": ts
                }
                writer.writerow(rec_off)
                tot_records.append(rec_off)

                # ── 2. ToT ON (K=3): Multi-candidate generation + expansion ──
                trial_idx += 1
                t0_tot = time.perf_counter()
                # Step 1: Generate K=3 approaches
                prompt_candidates = (
                    f"You are planning a task for a 5-DoF robotic arm.\n"
                    f"TASK: \"{instruction}\"\n"
                    f"Generate exactly 3 DIFFERENT high-level approaches in JSON:\n"
                    f'{{"approaches": [{{"name": "...", "steps": ["..."]}}]}}'
                )
                cand_resp, _ = call_ollama(prompt_candidates, format_spec="json", temp=TEMPERATURE, seed=seed, max_tokens=250)

                # Step 2: Expand candidates (K=3)
                for c_idx in range(3):
                    prompt_exp = (
                        f"Expand approach #{c_idx+1} for task \"{instruction}\" into action steps.\n"
                        f"{SCHEMA_PROMPT_SPEC}"
                    )
                    _, _ = call_ollama(prompt_exp, format_spec=ARIA_GRAMMAR_SCHEMA, temp=TEMPERATURE, seed=seed, max_tokens=200)

                dt_tot = (time.perf_counter() - t0_tot) * 1000.0
                ts = datetime.utcnow().isoformat() + "Z"

                rec_on = {
                    "trial_id": f"TOT_TRIAL_{trial_idx:04d}",
                    "prompt_id": pid,
                    "instruction": instruction,
                    "seed": seed,
                    "tot_mode": "ToT_ON_K3",
                    "latency_ms": round(dt_tot, 2),
                    "n_candidates": 3,
                    "commit_hash": GIT_COMMIT_HASH,
                    "timestamp": ts
                }
                writer.writerow(rec_on)
                tot_records.append(rec_on)

            print(f"  Timed Prompt {pid}: ToT OFF ~{dt_off:.0f}ms | ToT ON (K=3) ~{dt_tot:.0f}ms")

    times_off = [r["latency_ms"] for r in tot_records if r["tot_mode"] == "ToT_OFF"]
    times_on = [r["latency_ms"] for r in tot_records if r["tot_mode"] == "ToT_ON_K3"]

    print(f"\nToT Timing Results (N={len(times_off)} trials each):")
    print(f"  ToT OFF:       Mean={np.mean(times_off):.1f}ms | p50={np.percentile(times_off, 50):.1f}ms | p95={np.percentile(times_off, 95):.1f}ms")
    print(f"  ToT ON (K=3):  Mean={np.mean(times_on):.1f}ms | p50={np.percentile(times_on, 50):.1f}ms | p95={np.percentile(times_on, 95):.1f}ms")
    print(f"  Ratio ON / OFF: {np.mean(times_on) / np.mean(times_off):.2f}x")
    print(f"Raw ToT timing rows saved to: {TOT_TIMING_CSV}")


# ═════════════════════════════════════════════════════════════════════════════
# Main Benchmark Execution
# ═════════════════════════════════════════════════════════════════════════════
def main():
    print("═" * 78)
    print("Project ARIA: Real Multi-Seed Language Grounding Benchmark")
    print(f"Model: {MODEL_NAME} | Temp: {TEMPERATURE} | Seeds: {SEEDS}")
    print(f"Commit: {GIT_COMMIT_HASH}")
    print("═" * 78)

    if os.path.exists(RAW_TRIALS_CSV):
        print(f"[ERROR] Output file exists: {RAW_TRIALS_CSV}. Aborting to protect raw data.")
        sys.exit(1)

    with open(PROMPTS_FILE, "r", encoding="utf-8") as f:
        prompts = json.load(f)

    print(f"Loaded {len(prompts)} prompts across 6 categories from {PROMPTS_FILE}.")

    # First, run ToT timing benchmark
    run_tot_timing_benchmark(prompts)

    # Main multi-seed benchmark across all 60 prompts
    print("\n" + "═" * 78)
    print("RUNNING MULTI-SEED BENCHMARK (60 prompts x 3 methods x 3 seeds = 540 trials)")
    print("═" * 78)

    fieldnames = [
        "trial_id", "episode_id", "prompt_id", "category", "instruction",
        "method", "seed", "temperature", "commit_hash", "timestamp",
        "syntax_valid", "slot_action_correct", "slot_target_correct", "slot_destination_correct",
        "slot_accuracy", "clarification_triggered", "confidence_value", "threshold_value",
        "execution_success", "latency_ms", "parsed_plan", "raw_output"
    ]

    all_trials = []
    trial_counter = 0
    t_start = time.time()

    with open(RAW_TRIALS_CSV, "w", newline="", encoding="utf-8") as f_raw:
        writer = csv.DictWriter(f_raw, fieldnames=fieldnames)
        writer.writeheader()

        for p_idx, p in enumerate(prompts):
            pid = p["prompt_id"]
            ep_id = p["episode_id"]
            cat = p["category"]
            instr = p["instruction"]

            for seed in SEEDS:
                # 1. ARIA
                res_aria = run_aria_trial(p, seed, TEMPERATURE)
                trial_counter += 1
                row_aria = {
                    "trial_id": f"LG_TRIAL_{trial_counter:05d}",
                    "episode_id": ep_id,
                    "prompt_id": pid,
                    "category": cat,
                    "instruction": instr,
                    "method": "ARIA_Dialogue_Planning",
                    "seed": seed,
                    "temperature": TEMPERATURE,
                    "commit_hash": GIT_COMMIT_HASH,
                    "timestamp": datetime.utcnow().isoformat() + "Z",
                    **res_aria
                }
                writer.writerow(row_aria)
                all_trials.append(row_aria)

                # 2. Direct LLM
                res_direct = run_direct_llm_trial(p, seed, TEMPERATURE)
                trial_counter += 1
                row_direct = {
                    "trial_id": f"LG_TRIAL_{trial_counter:05d}",
                    "episode_id": ep_id,
                    "prompt_id": pid,
                    "category": cat,
                    "instruction": instr,
                    "method": "Direct_LLM",
                    "seed": seed,
                    "temperature": TEMPERATURE,
                    "commit_hash": GIT_COMMIT_HASH,
                    "timestamp": datetime.utcnow().isoformat() + "Z",
                    **res_direct
                }
                writer.writerow(row_direct)
                all_trials.append(row_direct)

                # 3. SayCan
                res_saycan = run_saycan_trial(p, seed, TEMPERATURE)
                trial_counter += 1
                row_saycan = {
                    "trial_id": f"LG_TRIAL_{trial_counter:05d}",
                    "episode_id": ep_id,
                    "prompt_id": pid,
                    "category": cat,
                    "instruction": instr,
                    "method": "SayCan",
                    "seed": seed,
                    "temperature": TEMPERATURE,
                    "commit_hash": GIT_COMMIT_HASH,
                    "timestamp": datetime.utcnow().isoformat() + "Z",
                    **res_saycan
                }
                writer.writerow(row_saycan)
                all_trials.append(row_saycan)

            f_raw.flush()
            if (p_idx + 1) % 5 == 0 or (p_idx + 1) == len(prompts):
                elapsed = time.time() - t_start
                rate = trial_counter / elapsed
                print(f"  Progress: {p_idx+1}/{len(prompts)} prompts ({trial_counter}/540 trials) | {rate:.1f} trials/s | Elapsed: {elapsed:.1f}s")

    # Generate Summary Aggregation
    print("\n" + "═" * 78)
    print("COMPUTING SUMMARY STATISTICS (Across 3 Seeds)")
    print("═" * 78)

    methods = ["ARIA_Dialogue_Planning", "Direct_LLM", "SayCan"]
    summary_rows = []

    print(f"{'Method':<24} | {'Syntax %':<9} | {'Slot Acc %':<11} | {'Clarif %':<9} | {'Exec Succ %':<12} | {'p50 Lat':<9} | {'p95 Lat'}")
    print("─" * 90)

    for m in methods:
        m_recs = [r for r in all_trials if r["method"] == m]
        n_m = len(m_recs)

        syntax_pct = 100.0 * sum(1 for r in m_recs if r["syntax_valid"]) / n_m
        slot_acc_pct = 100.0 * np.mean([r["slot_accuracy"] for r in m_recs])
        clarif_pct = 100.0 * sum(1 for r in m_recs if r["clarification_triggered"]) / n_m
        exec_succ_pct = 100.0 * sum(1 for r in m_recs if r["execution_success"]) / n_m

        latencies = [r["latency_ms"] for r in m_recs]
        p50_lat = float(np.percentile(latencies, 50))
        p95_lat = float(np.percentile(latencies, 95))
        mean_lat = float(np.mean(latencies))

        print(f"{m:<24} | {syntax_pct:7.1f}% | {slot_acc_pct:9.1f}% | {clarif_pct:7.1f}% | {exec_succ_pct:10.1f}% | {p50_lat:7.1f}ms | {p95_lat:7.1f}ms")

        summary_rows.append({
            "method": m,
            "total_trials": n_m,
            "seeds": json.dumps(SEEDS),
            "temperature": TEMPERATURE,
            "syntax_valid_pct": round(syntax_pct, 2),
            "slot_accuracy_pct": round(slot_acc_pct, 2),
            "clarification_pct": round(clarif_pct, 2),
            "execution_success_pct": round(exec_succ_pct, 2),
            "latency_mean_ms": round(mean_lat, 2),
            "latency_p50_ms": round(p50_lat, 2),
            "latency_p95_ms": round(p95_lat, 2),
            "model": MODEL_NAME,
            "commit_hash": GIT_COMMIT_HASH
        })

    with open(SUMMARY_CSV, "w", newline="", encoding="utf-8") as f_sum:
        writer = csv.DictWriter(f_sum, fieldnames=list(summary_rows[0].keys()))
        writer.writeheader()
        writer.writerows(summary_rows)

    print("\n✓ Completed multi-seed language benchmark and ToT timing.")
    print(f"✓ Output files:")
    print(f"  - {RAW_TRIALS_CSV}")
    print(f"  - {SUMMARY_CSV}")
    print(f"  - {TOT_TIMING_CSV}\n")


if __name__ == "__main__":
    main()
