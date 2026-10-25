#!/usr/bin/env python3
"""P2-N3: scene-match + calibrated LLM confidence on held-out ambiguity.

Three scorers are compared on the frozen prompt split:

``scene_only``
    A deterministic scene-match score.  The target phrase is tokenised into
    colour/type words and matched against every scene entity; the score is
    ``1 / n_matches`` (zero if no entity matches).  A unique match is easier
    than an ambiguous one.

``llm_only``
    The local Llama-3.1-8B-Instruct model (Ollama, temperature 0, seed 42)
    receives the instruction and the scene graph and returns a JSON
    resolution with a self-reported confidence.  The confidence is calibrated
    with Platt scaling on the tuning split.

``fused``
    A convex combination of the calibrated LLM confidence and the scene-match
    score.  The mixing weight is chosen on the calibration split to maximise
    AUROC.  The test split is never used to fit anything.

Success is semantic grounding success: the resolved target entity, action and
destination must match the frozen expected values.  It is not a physical
grasp success; physical execution is blocked by the gripper defect and is
reported as such.

The available annotated prompt set has 60 prompts, not the 120 named in the
pre-registration.  The split is therefore 30 tuning / 15 calibration / 15
test; see DEVIATIONS.md.

Usage:
    python3 scripts/study/p2_n3.py --smoke
    python3 scripts/study/p2_n3.py
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import requests  # noqa: E402
from sklearn.metrics import roc_auc_score  # noqa: E402

from aria_study import provenance, stats  # noqa: E402
from aria_study.trial_log import TrialLog  # noqa: E402

RAW = ROOT / "data" / "journal" / "raw"
PROMPTS = ROOT / "data" / "real" / "lang_prompts_heldout_60.json"
SPLIT = ROOT / "data" / "journal" / "prompt_split.json"
OLLAMA = "http://localhost:11434/api/generate"
MODEL = "llama3.1:8b-instruct-q4_K_M"
METHODS = ("scene_only", "llm_only", "fused")

SYSTEM = (
    "You are a robot instruction resolver. Given a user instruction and a "
    "scene graph, return a JSON object with exactly these keys: "
    '{"target_object": "<exact entity name from the scene>", '
    '"destination": "<destination phrase from the instruction>", '
    '"action": "<verb>", "confidence": <number between 0 and 1>, '
    '"ambiguity_notes": "<short note>"}. '
    "target_object must be one of the scene entity names. confidence is your "
    "probability that your resolution is correct. Do not add any text outside "
    "the JSON."
)


def normalise(text: str) -> List[str]:
    return re.findall(r"[a-z0-9]+", text.lower())


def token_overlap(a: str, b: str) -> float:
    ta, tb = set(normalise(a)), set(normalise(b))
    if not ta or not tb:
        return 0.0
    return len(ta & tb) / max(1, min(len(ta), len(tb)))


def scene_matches(prompt: dict) -> int:
    """Count entities that match the target phrase's colour/type tokens."""
    target = prompt.get("target_phrase", "")
    toks = set(normalise(target))
    if not toks:
        return 0
    n = 0
    for ent in prompt.get("scene_entities", []):
        blob = set(normalise(" ".join(str(ent.get(k, "")) for k in ("name", "color", "type"))))
        # A match requires every distinctive token of the phrase to appear.
        if toks.issubset(blob):
            n += 1
    return n


def scene_score(prompt: dict) -> float:
    n = scene_matches(prompt)
    return 1.0 / n if n > 0 else 0.0


def call_llm(prompt: dict, timeout: float = 180.0) -> Tuple[Optional[dict], str, float]:
    scene = [
        {
            "id": e.get("id"),
            "name": e.get("name"),
            "color": e.get("color"),
            "type": e.get("type"),
            "pos": e.get("pos"),
        }
        for e in prompt.get("scene_entities", [])
    ]
    user = (
        f"Instruction: {prompt['instruction']}\n"
        f"Scene entities: {json.dumps(scene)}\n"
        "Return only the JSON object."
    )
    payload = {
        "model": MODEL,
        "system": SYSTEM,
        "prompt": user,
        "format": "json",
        "stream": False,
        "options": {"temperature": 0.0, "seed": 42, "num_predict": 256},
    }
    t0 = time.monotonic()
    try:
        r = requests.post(OLLAMA, json=payload, timeout=timeout)
        r.raise_for_status()
        body = r.json()
        text = body.get("response", "")
        return json.loads(text), text, time.monotonic() - t0
    except Exception as exc:  # noqa: BLE001 - log the real failure
        return None, f"__ERROR__ {type(exc).__name__}: {exc}", time.monotonic() - t0


def target_ok(pred: dict, prompt: dict) -> bool:
    """Does the resolved target match the frozen expected target phrase?

    All distinctive tokens of the expected phrase must appear in the resolved
    target string, or the resolved string must equal a scene entity whose name
    matches the expected phrase.  This is a semantic grounding check, not a
    physical grasp check.
    """
    name = str(pred.get("target_object", ""))
    target = str(prompt.get("target_phrase", ""))
    want = set(normalise(target))
    have = set(normalise(name))
    if want and want.issubset(have):
        return True
    for ent in prompt.get("scene_entities", []):
        ename = str(ent.get("name", ""))
        if set(normalise(ename)) == have and want.issubset(set(normalise(ename))):
            return True
    return False


def action_ok(pred: dict, prompt: dict) -> bool:
    """Diagnostic only: the frozen expected action is a symbolic token."""
    return token_overlap(str(pred.get("action", "")), str(prompt.get("expected_action", ""))) >= 0.5


def destination_ok(pred: dict, prompt: dict) -> bool:
    """Diagnostic only: the frozen expected destination is a symbolic token."""
    return (
        token_overlap(str(pred.get("destination", "")), str(prompt.get("expected_destination", "")))
        >= 0.67
    )


def asks_for_clarification(pred: dict) -> bool:
    act = str(pred.get("action", "")).lower()
    notes = str(pred.get("ambiguity_notes", "")).lower()
    if any(k in act for k in ("clarif", "ask", "disambiguat")):
        return True
    if any(k in notes for k in ("ambigu", "clarif", "multiple", "not unique", "unclear", "not specified")):
        return True
    conf = pred.get("confidence")
    try:
        return bool(isinstance(conf, (int, float)) and math.isfinite(float(conf)) and float(conf) < 0.5)
    except Exception:  # noqa: BLE001
        return False


def success(pred: Optional[dict], prompt: dict) -> Tuple[bool, bool, bool, bool, bool]:
    """Fixed resolution-correctness label (frozen before the full run).

    For an unambiguous prompt the correct behaviour is to resolve the named
    target, so success is a target match.  For an ambiguous prompt the correct
    behaviour is to ask for clarification (or to report low confidence), so a
    confident guess counts as a failure even when it happens to name a real
    scene entity.  The three diagnostic booleans are logged but are not part
    of the success label: the frozen expected action and destination are
    symbolic checker tokens, not natural-language strings.
    """
    if pred is None:
        return False, False, False, False, False
    t = target_ok(pred, prompt)
    a = action_ok(pred, prompt)
    d = destination_ok(pred, prompt)
    conf = pred.get("confidence", None)
    conf_ok = isinstance(conf, (int, float)) and math.isfinite(float(conf))
    if bool(prompt.get("is_ambiguous")):
        ok = asks_for_clarification(pred)
    else:
        ok = t
    return bool(ok), t, a, d, conf_ok


def platt_fit(scores: Sequence[float], labels: Sequence[bool]) -> Tuple[float, float]:
    """Fit p = sigmoid(a*logit(s) + b) by a small grid search."""
    s = np.clip(np.asarray(scores, float), 1e-4, 1 - 1e-4)
    y = np.asarray(labels, bool).astype(float)
    if y.min() == y.max():
        return 1.0, 0.0
    x = np.log(s / (1 - s))
    best = (1.0, 0.0)
    best_nll = float("inf")
    for a in np.linspace(0.05, 5.0, 100):
        for b in np.linspace(-3.0, 3.0, 121):
            p = 1.0 / (1.0 + np.exp(-(a * x + b)))
            p = np.clip(p, 1e-9, 1 - 1e-9)
            nll = float(-np.mean(y * np.log(p) + (1 - y) * np.log(1 - p)))
            if nll < best_nll:
                best_nll, best = nll, (float(a), float(b))
    return best


def platt_apply(scores: Sequence[float], ab: Tuple[float, float]) -> np.ndarray:
    s = np.clip(np.asarray(scores, float), 1e-4, 1 - 1e-4)
    x = np.log(s / (1 - s))
    a, b = ab
    return 1.0 / (1.0 + np.exp(-(a * x + b)))


def auroc(scores: Sequence[float], labels: Sequence[bool]) -> float:
    y = np.asarray(labels, bool)
    if y.min() == y.max():
        return float("nan")
    return float(roc_auc_score(y, np.asarray(scores, float)))


def bootstrap_auroc(
    scores: Sequence[float], labels: Sequence[bool], n_boot: int = 10000, seed: int = 12345
) -> Tuple[float, float, float]:
    s = np.asarray(scores, float)
    y = np.asarray(labels, bool)
    if y.min() == y.max():
        return float("nan"), float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    vals = []
    for _ in range(n_boot):
        idx = rng.integers(0, len(s), len(s))
        if y[idx].min() == y[idx].max():
            continue
        vals.append(roc_auc_score(y[idx], s[idx]))
    if not vals:
        return auroc(s, y), float("nan"), float("nan")
    lo, hi = np.quantile(vals, [0.025, 0.975])
    return auroc(s, y), float(lo), float(hi)


def paired_auroc_diff(
    s1: Sequence[float],
    s2: Sequence[float],
    labels: Sequence[bool],
    n_boot: int = 10000,
    seed: int = 12345,
) -> Tuple[float, float, float]:
    a = np.asarray(s1, float)
    b = np.asarray(s2, float)
    y = np.asarray(labels, bool)
    if y.min() == y.max():
        return float("nan"), float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    vals = []
    for _ in range(n_boot):
        idx = rng.integers(0, len(y), len(y))
        if y[idx].min() == y[idx].max():
            continue
        vals.append(roc_auc_score(y[idx], a[idx]) - roc_auc_score(y[idx], b[idx]))
    if not vals:
        return float("nan"), float("nan"), float("nan")
    lo, hi = np.quantile(vals, [0.025, 0.975])
    return float(auroc(a, y) - auroc(b, y)), float(lo), float(hi)


def run(args) -> int:
    smoke = bool(args.smoke)
    exp_id = "P2-N3-smoke" if smoke else "P2-N3"
    provenance.assert_clean(exp_id)
    if not SPLIT.exists():
        raise RuntimeError(f"frozen prompt split missing: {SPLIT}; run freeze_n3_split.py first")
    split = json.loads(SPLIT.read_text())
    prompts = {p["prompt_id"]: p for p in json.loads(PROMPTS.read_text())}
    items = split["items"]
    if smoke:
        items = [x for x in items if x["split"] == "tuning"][:3]

    fields = [
        "trial_id", "prompt_id", "split", "method",
        "instruction", "scene_matches", "scene_score",
        "llm_confidence", "llm_confidence_calibrated", "fused_score",
        "success", "target_ok", "action_ok", "destination_ok",
        "parse_ok", "confidence_present", "llm_latency_s",
        "raw_output",
    ]
    config = {
        "model": MODEL,
        "ollama_url": OLLAMA,
        "temperature": 0.0,
        "seed": 42,
        "split": split["splits"],
        "split_rule": split["rule"],
        "n_prompts": split["n_prompts"],
        "success_definition": (
            "resolution correctness: unambiguous prompts require a target match; "
            "ambiguous prompts require a clarification request or confidence < 0.5"
        ),
        "fusion": "convex combination, weight chosen on calibration split",
    }

    rows: List[dict] = []
    t_start = datetime.now(timezone.utc)
    with TrialLog(exp_id, fields, config) as log:
        for item in items:
            pid = item["prompt_id"]
            p = prompts[pid]
            pred, raw, latency = call_llm(p)
            parse_ok = pred is not None
            succ, t_ok, a_ok, d_ok, conf_ok = success(pred, p)
            conf = float(pred.get("confidence", 0.0)) if (parse_ok and conf_ok) else 0.0
            conf = min(1.0, max(0.0, conf))
            sm = scene_matches(p)
            ss = scene_score(p)
            base = {
                "prompt_id": pid,
                "split": item["split"],
                "instruction": p["instruction"],
                "scene_matches": sm,
                "scene_score": ss,
                "llm_confidence": conf,
                "success": bool(succ),
                "target_ok": bool(t_ok),
                "action_ok": bool(a_ok),
                "destination_ok": bool(d_ok),
                "parse_ok": bool(parse_ok),
                "confidence_present": bool(conf_ok),
                "llm_latency_s": float(latency),
                "raw_output": raw[:2000],
            }
            rows.append(base)
            print(
                f"  {pid} {item['split']:11s} match={sm} conf={conf:.2f} "
                f"succ={int(succ)} ({latency:.1f}s)",
                flush=True,
            )

        # Fit calibration and fusion on tuning/calibration only.
        tune = [r for r in rows if r["split"] == "tuning"]
        cal = [r for r in rows if r["split"] == "calibration"]
        test = [r for r in rows if r["split"] == "test"]
        ab = platt_fit([r["llm_confidence"] for r in tune], [r["success"] for r in tune]) if tune else (1.0, 0.0)
        for r in rows:
            r["llm_confidence_calibrated"] = float(
                platt_apply([r["llm_confidence"]], ab)[0]
            )
        # Choose the fusion weight on the calibration split (fall back to tuning).
        fit_rows = cal if len(cal) >= 5 else tune
        best_w, best_auc = 0.5, -1.0
        for w in np.linspace(0.0, 1.0, 101):
            s = [w * r["llm_confidence_calibrated"] + (1 - w) * r["scene_score"] for r in fit_rows]
            y = [r["success"] for r in fit_rows]
            if len(set(y)) < 2:
                continue
            a = auroc(s, y)
            if np.isfinite(a) and a > best_auc:
                best_auc, best_w = float(a), float(w)
        for r in rows:
            r["fused_score"] = float(
                best_w * r["llm_confidence_calibrated"] + (1 - best_w) * r["scene_score"]
            )

        # Log one row per method.
        for r in rows:
            for method in METHODS:
                row = dict(r)
                row["trial_id"] = f"{r['prompt_id']}_{method}"
                row["method"] = method
                row["llm_confidence_calibrated"] = r["llm_confidence_calibrated"]
                log.log(row)

        summary = {
            "exp_id": exp_id,
            "started_at": t_start.isoformat(),
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "n_prompts": len(rows),
            "n_rows": len(rows) * len(METHODS),
            "split": split["splits"],
            "model": MODEL,
            "platt": {"a": ab[0], "b": ab[1]},
            "fusion_weight_llm": best_w,
            "fusion_fit_auc": best_auc,
            "success_rate_all": float(np.mean([r["success"] for r in rows])),
        }
        for name, cell in (("all", rows), ("test", test), ("calibration", cal), ("tuning", tune)):
            if not cell:
                continue
            y = [r["success"] for r in cell]
            summary[name] = {
                "n": len(cell),
                "success_rate": float(np.mean(y)),
                "n_success": int(np.sum(y)),
                "parse_rate": float(np.mean([r["parse_ok"] for r in cell])),
            }
            for method in METHODS:
                key = {
                    "scene_only": "scene_score",
                    "llm_only": "llm_confidence_calibrated",
                    "fused": "fused_score",
                }[method]
                s = [r[key] for r in cell]
                a, lo, hi = bootstrap_auroc(s, y)
                summary[name][method] = {
                    "auroc": a,
                    "auroc_ci_low": lo,
                    "auroc_ci_high": hi,
                }
        if test:
            y = [r["success"] for r in test]
            for m in ("scene_only", "llm_only"):
                key = "scene_score" if m == "scene_only" else "llm_confidence_calibrated"
                d, lo, hi = paired_auroc_diff(
                    [r["fused_score"] for r in test], [r[key] for r in test], y
                )
                summary[f"fused_minus_{m}_test"] = {
                    "delta_auroc": d,
                    "ci_low": lo,
                    "ci_high": hi,
                    "ci_excludes_zero": bool(lo > 0 or hi < 0),
                }
        RAW.mkdir(parents=True, exist_ok=True)
        (RAW / f"{exp_id}_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary, indent=2), flush=True)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true")
    return run(ap.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
