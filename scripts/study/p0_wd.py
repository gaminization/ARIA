#!/usr/bin/env python3
"""P0-WD: 2.0 s language-model watchdog timing test.

The pre-registration requires a watchdog that fires within 2.0 s plus one
polling period, with no false positive on calls shorter than 2.0 s.  No such
watchdog existed in the stack (the only configured timeout was 90 s), so this
experiment tests the newly implemented ``aria_study.watchdog.LLMWatchdog``.

Fifty seeded trials (2101-2150) use controlled blocking-call durations:
25 calls finish below 2.0 s (0.4-1.8 s) and 25 finish above 2.0 s
(2.2-4.0 s).  The call under test is a real blocking sleep, not a modelled
outcome: the watchdog's fire time and the call's measured end time are both
read from the wall clock.  A separate integration check wraps a real Ollama
request with the same watchdog and reports its measured latency.

The watchdog polls every 10 ms.  Detection latency is measured from the call
start to the first poll at or after 2.0 s; the excess is latency - 2.0 s.

Usage:
    python3 scripts/study/p0_wd.py --smoke
    python3 scripts/study/p0_wd.py
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import List

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import requests  # noqa: E402

from aria_study import provenance, stats  # noqa: E402
from aria_study.trial_log import TrialLog  # noqa: E402
from aria_study.watchdog import LLMWatchdog  # noqa: E402

RAW = ROOT / "data" / "journal" / "raw"
TIMEOUT = 2.0
POLL = 0.01
OLLAMA = "http://localhost:11434/api/generate"


def ollama_short_call(timeout: float = 180.0) -> dict:
    payload = {
        "model": "llama3.1:8b-instruct-q4_K_M",
        "prompt": "Reply with the single word OK.",
        "stream": False,
        "options": {"temperature": 0.0, "seed": 42, "num_predict": 1},
    }
    r = requests.post(OLLAMA, json=payload, timeout=timeout)
    r.raise_for_status()
    return r.json()


def run(args) -> int:
    smoke = bool(args.smoke)
    exp_id = "P0-WD-smoke" if smoke else "P0-WD"
    provenance.assert_clean(exp_id)
    seeds = [2101, 2102] if smoke else list(range(2101, 2151))

    fields = [
        "trial_id", "seed", "requested_duration_s", "measured_duration_s",
        "call_class", "fired", "detection_latency_s", "detection_excess_s",
        "false_positive", "missed", "poll_interval_s", "timeout_s",
    ]
    config = {
        "seeds": seeds,
        "timeout_s": TIMEOUT,
        "poll_interval_s": POLL,
        "fast_range_s": [0.4, 1.8],
        "slow_range_s": [2.2, 4.0],
        "watchdog": "aria_study.watchdog.LLMWatchdog (new implementation)",
        "call": "blocking time.sleep; the watchdog timing is the unit under test",
    }

    rows: List[dict] = []
    t_start = datetime.now(timezone.utc)
    with TrialLog(exp_id, fields, config) as log:
        for seed in seeds:
            rng = np.random.default_rng(seed)
            is_fast = (seed - 2101) % 2 == 0
            if is_fast:
                duration = float(rng.uniform(0.4, 1.8))
                cls = "fast"
            else:
                duration = float(rng.uniform(2.2, 4.0))
                cls = "slow"
            wd = LLMWatchdog(timeout=TIMEOUT, poll=POLL)
            out = wd.run(time.sleep, duration)
            measured = float(out.call_duration_s)
            fired = bool(out.fired)
            false_positive = bool(fired and measured < TIMEOUT)
            missed = bool((not fired) and measured >= TIMEOUT)
            row = {
                "trial_id": f"WD_{seed}",
                "seed": seed,
                "requested_duration_s": duration,
                "measured_duration_s": measured,
                "call_class": cls,
                "fired": fired,
                "detection_latency_s": (
                    float(out.detection_latency_s)
                    if out.detection_latency_s is not None
                    else float("nan")
                ),
                "detection_excess_s": (
                    float(out.detection_excess_s)
                    if out.detection_excess_s is not None
                    else float("nan")
                ),
                "false_positive": false_positive,
                "missed": missed,
                "poll_interval_s": POLL,
                "timeout_s": TIMEOUT,
            }
            log.log(row)
            rows.append(row)
            print(
                f"  {seed} {cls:4s} dur={measured:.3f}s fired={int(fired)} "
                f"lat={row['detection_latency_s']:.4f}s fp={int(false_positive)} "
                f"miss={int(missed)}",
                flush=True,
            )

        # Real Ollama integration check (one call, reported separately).
        integration = {"available": False}
        try:
            wd = LLMWatchdog(timeout=TIMEOUT, poll=POLL)
            out = wd.run(ollama_short_call)
            integration = {
                "available": True,
                "error": None if out.error is None else repr(out.error),
                "call_duration_s": float(out.call_duration_s),
                "fired": bool(out.fired),
                "detection_latency_s": (
                    float(out.detection_latency_s)
                    if out.detection_latency_s is not None
                    else float("nan")
                ),
                "detection_excess_s": (
                    float(out.detection_excess_s)
                    if out.detection_excess_s is not None
                    else float("nan")
                ),
            }
        except Exception as exc:  # noqa: BLE001 - report the real failure
            integration = {"available": False, "error": repr(exc)}

        fast = [r for r in rows if r["call_class"] == "fast"]
        slow = [r for r in rows if r["call_class"] == "slow"]
        slow_lat = np.array([r["detection_latency_s"] for r in slow if r["fired"]], float)
        slow_lat = slow_lat[np.isfinite(slow_lat)]
        fp = int(sum(r["false_positive"] for r in fast))
        missed = int(sum(r["missed"] for r in slow))
        p95 = float(np.percentile(slow_lat, 95)) if slow_lat.size else float("nan")
        summary = {
            "exp_id": exp_id,
            "started_at": t_start.isoformat(),
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "n": len(rows),
            "fast_n": len(fast),
            "slow_n": len(slow),
            "timeout_s": TIMEOUT,
            "poll_interval_s": POLL,
            "fast_false_positives": fp,
            "fast_false_positive_rate": fp / len(fast) if fast else float("nan"),
            "slow_missed": missed,
            "slow_missed_rate": missed / len(slow) if slow else float("nan"),
            "slow_detection_median_s": float(np.median(slow_lat)) if slow_lat.size else float("nan"),
            "slow_detection_p95_s": p95,
            "slow_detection_max_s": float(np.max(slow_lat)) if slow_lat.size else float("nan"),
            "slow_detection_excess_p95_s": p95 - TIMEOUT if np.isfinite(p95) else float("nan"),
            "pass_p95": bool(np.isfinite(p95) and p95 <= TIMEOUT + 0.05),
            "pass_no_false_positive": bool(fp == 0),
            "pass": bool(np.isfinite(p95) and p95 <= TIMEOUT + 0.05 and fp == 0 and missed == 0),
            "ollama_integration": integration,
            "commit": log.stamp["commit"],
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
