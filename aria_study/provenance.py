"""Provenance helpers for the ARIA journal study.

Every experiment refuses to run on a dirty tree and stamps its rows with the
commit hash that produced them.
"""
from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import List

ROOT = Path(__file__).resolve().parents[1]

# Outputs produced *during* a run must not make the tree dirty. They are
# either git-ignored or filtered here so the provenance check stays strict
# about code while allowing append-only logs.
_IGNORED_PREFIXES = (
    "data/journal/raw/",
    "data/journal/figures/",
    "data/journal/models/",
    "log/",
    "build/",
    "install/",
)


def _git(*args: str) -> str:
    return subprocess.check_output(
        ["git", *args], cwd=str(ROOT), stderr=subprocess.DEVNULL, text=True
    ).strip()


def commit() -> str:
    return _git("rev-parse", "HEAD")


def short_commit() -> str:
    return _git("rev-parse", "--short", "HEAD")


def dirty_paths() -> List[str]:
    out = _git("status", "--porcelain")
    paths: List[str] = []
    for line in out.splitlines():
        if not line.strip():
            continue
        path = line[3:].strip()
        if " -> " in path:
            path = path.split(" -> ", 1)[1]
        if any(path.startswith(p) for p in _IGNORED_PREFIXES):
            continue
        paths.append(path)
    return paths


def is_dirty() -> bool:
    return bool(dirty_paths())


def assert_clean(context: str = "experiment") -> None:
    paths = dirty_paths()
    if paths:
        shown = "\n  ".join(paths[:20])
        raise RuntimeError(
            f"Refusing to run {context} on a dirty tree. Commit or stash first.\n  {shown}"
        )


def stamp() -> dict:
    return {"commit": commit(), "short_commit": short_commit(), "dirty": False}
