#!/usr/bin/env python3
"""Freeze the P2-N3 prompt partition before any LLM evaluation.

The pre-registration specifies a deterministic SHA-256 partition of the
available prompts.  Only 60 prompts carry scene-entity annotations in this
repository (``data/real/lang_prompts_heldout_60.json``); the other 60 prompts
have no scene graph, so they cannot be used for scene-match scoring.  The
shortfall is recorded in DEVIATIONS.md and the split is 30 tuning / 15
calibration / 15 test, preserving the pre-registered proportions.

Usage:
    python3 scripts/study/freeze_n3_split.py
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROMPTS = ROOT / "data" / "real" / "lang_prompts_heldout_60.json"
OUT = ROOT / "data" / "journal" / "prompt_split.json"


def main() -> int:
    prompts = json.loads(PROMPTS.read_text())
    items = []
    for p in prompts:
        text = p["instruction"].strip()
        h = hashlib.sha256(text.encode("utf-8")).hexdigest()
        items.append({"prompt_id": p["prompt_id"], "instruction": text, "sha256": h})
    items.sort(key=lambda x: x["sha256"])
    n = len(items)
    n_tune = n // 2
    n_cal = n // 4
    for i, item in enumerate(items):
        if i < n_tune:
            item["split"] = "tuning"
        elif i < n_tune + n_cal:
            item["split"] = "calibration"
        else:
            item["split"] = "test"
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(
        json.dumps(
            {
                "source": str(PROMPTS.relative_to(ROOT)),
                "rule": "sort by SHA-256(instruction); 30 tuning / 15 calibration / 15 test",
                "n_prompts": n,
                "splits": {
                    "tuning": sum(x["split"] == "tuning" for x in items),
                    "calibration": sum(x["split"] == "calibration" for x in items),
                    "test": sum(x["split"] == "test" for x in items),
                },
                "items": items,
            },
            indent=2,
        )
        + "\n"
    )
    print(f"wrote {OUT}: {n} prompts")
    for s in ("tuning", "calibration", "test"):
        print(s, sum(x["split"] == s for x in items))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
