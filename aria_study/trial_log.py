"""Append-only per-trial logging with mandatory provenance."""
from __future__ import annotations

import csv
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable

from . import provenance

RAW_DIR = provenance.ROOT / "data" / "journal" / "raw"


class TrialLog:
    """One CSV + one JSONL file per experiment.

    The CSV is the analysis table. The JSONL keeps nested config and any extra
    fields without widening the CSV schema. Both are flushed per row so a
    crash cannot silently drop completed trials.
    """

    def __init__(self, exp_id: str, fieldnames: Iterable[str], config: Dict[str, Any] | None = None):
        provenance.assert_clean(exp_id)
        RAW_DIR.mkdir(parents=True, exist_ok=True)
        self.exp_id = exp_id
        self.csv_path = RAW_DIR / f"{exp_id}.csv"
        self.jsonl_path = RAW_DIR / f"{exp_id}.jsonl"
        self.fieldnames = list(fieldnames)
        self.config = config or {}
        self._stamp = provenance.stamp()
        self._csv_f = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._jsonl_f = open(self.jsonl_path, "w", encoding="utf-8")
        self._writer = csv.DictWriter(self._csv_f, fieldnames=self.fieldnames, extrasaction="ignore")
        self._writer.writeheader()
        self._csv_f.flush()
        self._n = 0

    def log(self, row: Dict[str, Any]) -> None:
        if self._stamp["dirty"]:
            raise RuntimeError("refusing to log a dirty trial")
        out = dict(row)
        out.setdefault("exp_id", self.exp_id)
        out.setdefault("commit", self._stamp["commit"])
        out.setdefault("short_commit", self._stamp["short_commit"])
        out.setdefault("dirty", False)
        out.setdefault("logged_at", datetime.now(timezone.utc).isoformat())
        self._writer.writerow(out)
        self._csv_f.flush()
        rec = {"row": out, "config": self.config, "monotonic_ns": time.monotonic_ns()}
        self._jsonl_f.write(json.dumps(rec, default=str) + "\n")
        self._jsonl_f.flush()
        self._n += 1

    def close(self) -> None:
        self._csv_f.close()
        self._jsonl_f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    @property
    def count(self) -> int:
        return self._n
