#!/usr/bin/env python3
"""P2-N4: does kinematic pre-verification reduce execution-time IK failures?

The deployed planner trusts ``IKResult.success``.  That flag is set whenever the
Newton-Raphson routine returns any candidate, even when the residual is large
(see ``aria_analytical_ik.solve_analytical``: ``success=True`` is hard-coded).
A plan built on such a candidate is executed and fails at run time.

This experiment compares two pipelines on identical sampled wrist targets:

``baseline``
    One call to ``solve_analytical``; execute the returned joints.  The trial
    fails when the URDF forward kinematics of those joints misses the target by
    more than ``SUCCESS_MM``.

``n4``
    Kinematic pre-verification: accept a candidate only when its FK residual is
    <= ``VERIFY_MM`` and every joint keeps a margin of ``LIMIT_MARGIN`` rad from
    its limit.  If verification fails, replan from up to ``REPLANS`` nearby
    targets.  If no candidate verifies, the plan is rejected and counted as a
    failure in the intention-to-treat analysis.  Coverage (executed / total) is
    reported so rejection cannot be hidden.

Targets are sampled from a box that deliberately includes poses outside the
arm's reach, because that is where the success flag lies.  The box is frozen in
the config and was not tuned on the evaluation seeds.

Usage:
    python3 scripts/study/p2_n4.py --smoke
    python3 scripts/study/p2_n4.py
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from aria_study import provenance, stats  # noqa: E402
from aria_study.kinematics import (  # noqa: E402
    JOINT_LIMITS,
    forward_kinematics,
    world_to_base,
)
from aria_study.trial_log import TrialLog  # noqa: E402

sys.path.insert(0, str(ROOT / "arm_ik" / "arm_ik" / "ik_solvers"))
from aria_analytical_ik import SolverConfig, solve_analytical  # noqa: E402

SUCCESS_MM = 5.0
VERIFY_MM = 2.0
LIMIT_MARGIN = 0.02
REPLANS = 3
NEARBY_M = 0.02
BOX = {
    "x": [0.08, 0.30],
    "y": [-0.18, 0.18],
    "z": [0.66, 0.78],
}


def fk_error_mm(q: np.ndarray, target_base: np.ndarray) -> float:
    T = forward_kinematics(np.asarray(q, float))
    return float(np.linalg.norm(T[:3, 3] - target_base) * 1000.0)


def limit_margin(q: np.ndarray) -> float:
    q = np.asarray(q, float)
    return float(np.min(np.minimum(q - JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1] - q)))


def solve(target_world: np.ndarray):
    cfg = SolverConfig()
    return solve_analytical(world_to_base(target_world), target_pitch=1.0, config=cfg)


def baseline(target_world: np.ndarray) -> dict:
    res = solve(target_world)
    q = np.asarray(res.joint_angles, float)
    err = fk_error_mm(q, world_to_base(target_world))
    ok = bool(res.success) and err <= SUCCESS_MM
    return {
        "executed": 1,
        "verified": int(bool(res.success)),
        "success": int(ok),
        "fk_error_mm": err,
        "limit_margin_rad": limit_margin(q),
        "replans": 0,
        "solver_success_flag": int(bool(res.success)),
    }


def n4(target_world: np.ndarray, rng: np.random.Generator) -> dict:
    """Verify, then replan locally when verification fails."""
    replans = 0
    candidate = np.asarray(target_world, float)
    last = None
    for attempt in range(REPLANS + 1):
        res = solve(candidate)
        q = np.asarray(res.joint_angles, float)
        err = fk_error_mm(q, world_to_base(candidate))
        margin = limit_margin(q)
        verified = bool(res.success) and err <= VERIFY_MM and margin >= LIMIT_MARGIN
        last = {
            "executed": 1,
            "verified": int(verified),
            "success": int(verified and err <= SUCCESS_MM),
            "fk_error_mm": err,
            "limit_margin_rad": margin,
            "replans": replans,
            "solver_success_flag": int(bool(res.success)),
        }
        if verified:
            return last
        if attempt < REPLANS:
            replans += 1
            candidate = np.asarray(target_world, float) + rng.uniform(
                -NEARBY_M, NEARBY_M, size=3
            )
    # Nothing verified: reject the plan.  Intention-to-treat counts it as a
    # failure; coverage records that it was never executed.
    last["executed"] = 0
    last["success"] = 0
    last["replans"] = replans
    return last


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true")
    args = ap.parse_args()

    provenance.assert_clean("P2-N4")

    seeds = [5101, 5102] if args.smoke else list(range(5101, 5201))
    exp_id = "P2-N4-smoke" if args.smoke else "P2-N4"

    fields = [
        "trial_id", "seed", "method",
        "target_x", "target_y", "target_z",
        "executed", "verified", "success", "replans",
        "solver_success_flag", "fk_error_mm", "limit_margin_rad",
    ]
    config = {
        "seeds": seeds,
        "box": BOX,
        "success_mm": SUCCESS_MM,
        "verify_mm": VERIFY_MM,
        "limit_margin_rad": LIMIT_MARGIN,
        "replans": REPLANS,
        "nearby_m": NEARBY_M,
        "solver": "aria_analytical_ik.solve_analytical",
        "target_frame": "world; converted to base_link for the solver",
        "metric": "executed plan whose FK position residual exceeds success_mm",
    }

    rows = []
    t_start = datetime.now(timezone.utc)
    with TrialLog(exp_id, fields, config) as log:
        trial = 0
        for seed in seeds:
            rng = np.random.default_rng(seed)
            target = np.array(
                [
                    rng.uniform(*BOX["x"]),
                    rng.uniform(*BOX["y"]),
                    rng.uniform(*BOX["z"]),
                ]
            )
            for method, fn in (("baseline", baseline), ("n4", n4)):
                out = fn(target) if method == "baseline" else fn(target, rng)
                row = {
                    "trial_id": trial,
                    "seed": seed,
                    "method": method,
                    "target_x": float(target[0]),
                    "target_y": float(target[1]),
                    "target_z": float(target[2]),
                    **out,
                }
                log.log(row)
                rows.append(row)
                trial += 1
                print(
                    f"  trial {trial}: seed={seed} {method:8s} "
                    f"err={out['fk_error_mm']:.2f}mm ok={out['success']} "
                    f"replans={out['replans']} exec={out['executed']}",
                    flush=True,
                )

    summary = {
        "exp_id": exp_id,
        "started_at": t_start.isoformat(),
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "n": len(rows),
        "seeds": seeds,
        "commit": provenance.commit(),
        "config": config,
    }
    for method in ("baseline", "n4"):
        cell = [r for r in rows if r["method"] == method]
        s = stats.summarize_binary([r["success"] for r in cell])
        executed = [r for r in cell if r["executed"]]
        ex = stats.summarize_binary([r["success"] for r in executed]) if executed else None
        summary[method] = {
            **s,
            "coverage": len(executed) / len(cell),
            "executed_failure_rate": (
                1.0 - ex["rate"] if ex else None
            ),
            "fk_error_median_mm": float(np.median([r["fk_error_mm"] for r in cell])),
            "fk_error_p95_mm": float(np.quantile([r["fk_error_mm"] for r in cell], 0.95)),
            "solver_success_flag_rate": float(
                np.mean([r["solver_success_flag"] for r in cell])
            ),
            "false_success_rate": float(
                np.mean(
                    [
                        bool(r["solver_success_flag"]) and r["fk_error_mm"] > SUCCESS_MM
                        for r in cell
                    ]
                )
            ),
            "mean_replans": float(np.mean([r["replans"] for r in cell])),
        }

    paired = {}
    for r in rows:
        paired.setdefault(r["seed"], {})[r["method"]] = r
    pairs = [v for v in paired.values() if "baseline" in v and "n4" in v]
    b = sum(v["baseline"]["success"] and not v["n4"]["success"] for v in pairs)
    c = sum(v["n4"]["success"] and not v["baseline"]["success"] for v in pairs)
    rd, rd_lo, rd_hi = stats.bootstrap_diff_ci(
        [v["n4"]["success"] for v in pairs],
        [v["baseline"]["success"] for v in pairs],
        np.mean,
    )
    summary["paired"] = {
        "n_pairs": len(pairs),
        "baseline_only_success": b,
        "n4_only_success": c,
        "risk_difference_n4_minus_baseline": rd,
        "risk_difference_ci_low": rd_lo,
        "risk_difference_ci_high": rd_hi,
        "mcnemar_p": stats.mcnemar_exact(b, c),
    }

    RAW = ROOT / "data" / "journal" / "raw"
    (RAW / f"{exp_id}_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
