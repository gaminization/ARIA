"""Small, dependency-light statistics used by the study scripts."""
from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple

import numpy as np


def wilson(k: int, n: int, z: float = 1.959963984540054) -> Tuple[float, float, float]:
    """Wilson score interval for a binomial proportion."""
    if n <= 0:
        return float("nan"), float("nan"), float("nan")
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt((p * (1 - p) + z * z / (4 * n)) / n) / denom
    return p, max(0.0, centre - half), min(1.0, centre + half)


def bootstrap_ci(
    values: Sequence[float],
    stat=np.median,
    n_boot: int = 10000,
    alpha: float = 0.05,
    seed: int = 12345,
) -> Tuple[float, float, float]:
    """Percentile bootstrap CI. Returns (point, low, high)."""
    arr = np.asarray(list(values), dtype=float)
    if arr.size == 0:
        return float("nan"), float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, arr.size, size=(n_boot, arr.size))
    stats = np.array([stat(arr[i]) for i in idx])
    lo, hi = np.quantile(stats, [alpha / 2, 1 - alpha / 2])
    return float(stat(arr)), float(lo), float(hi)


def bootstrap_diff_ci(
    a: Sequence[float],
    b: Sequence[float],
    stat=np.mean,
    n_boot: int = 10000,
    alpha: float = 0.05,
    seed: int = 12345,
) -> Tuple[float, float, float]:
    """Paired bootstrap CI of stat(a) - stat(b). Inputs must be aligned."""
    aa = np.asarray(list(a), dtype=float)
    bb = np.asarray(list(b), dtype=float)
    if aa.size != bb.size:
        raise ValueError("paired bootstrap needs equal-length inputs")
    if aa.size == 0:
        return float("nan"), float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, aa.size, size=(n_boot, aa.size))
    diffs = np.array([stat(aa[i]) - stat(bb[i]) for i in idx])
    lo, hi = np.quantile(diffs, [alpha / 2, 1 - alpha / 2])
    return float(stat(aa) - stat(bb)), float(lo), float(hi)


def mcnemar_exact(b: int, c: int) -> float:
    """Two-sided exact McNemar p-value (binomial test on discordant pairs)."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    # two-sided exact binomial with p=0.5
    tail = sum(math.comb(n, i) for i in range(0, k + 1)) / (2 ** n)
    return min(1.0, 2.0 * tail)


def odds_ratio(a: int, b: int, c: int, d: int) -> Tuple[float, float, float]:
    """Odds ratio and 95% CI with Haldane-Anscombe correction.

    Table is [[a, b], [c, d]]: a=success treat, b=failure treat,
    c=success control, d=failure control.
    """
    aa, bb, cc, dd = a + 0.5, b + 0.5, c + 0.5, d + 0.5
    orv = (aa * dd) / (bb * cc)
    se = math.sqrt(1 / aa + 1 / bb + 1 / cc + 1 / dd)
    lo = math.exp(math.log(orv) - 1.959963984540054 * se)
    hi = math.exp(math.log(orv) + 1.959963984540054 * se)
    return orv, lo, hi


def holm(pvalues: Iterable[float]) -> list:
    """Holm-Bonferroni adjusted p-values, order preserved."""
    ps = list(pvalues)
    m = len(ps)
    order = sorted(range(m), key=lambda i: ps[i])
    adjusted = [0.0] * m
    running = 0.0
    for rank, i in enumerate(order):
        val = min(1.0, (m - rank) * ps[i])
        running = max(running, val)
        adjusted[i] = running
    return adjusted


def risk_difference_ci(k1: int, n1: int, k2: int, n2: int) -> Tuple[float, float, float]:
    """Newcombe hybrid score interval for p1 - p2 (unpaired)."""
    p1, l1, u1 = wilson(k1, n1)
    p2, l2, u2 = wilson(k2, n2)
    diff = p1 - p2
    lo = diff - math.sqrt((p1 - l1) ** 2 + (u2 - p2) ** 2)
    hi = diff + math.sqrt((u1 - p1) ** 2 + (p2 - l2) ** 2)
    return diff, lo, hi


def summarize_binary(rows: Sequence[bool]) -> dict:
    k = int(sum(bool(x) for x in rows))
    n = len(rows)
    p, lo, hi = wilson(k, n)
    return {"k": k, "n": n, "rate": p, "wilson_low": lo, "wilson_high": hi}
