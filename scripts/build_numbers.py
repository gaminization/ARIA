#!/usr/bin/env python3
"""Build every number in the paper from raw experiment outputs.

Reads only ``data/journal/raw/*_summary.json`` and the per-trial CSVs that the
experiment scripts wrote.  Writes:

* ``publications/ieee_tro_submission/numbers.tex`` — LaTeX macros, one per
  reported quantity.
* ``data/journal/ledger.csv`` — one row per reported quantity with its value,
  sample size, confidence interval, source file and source commit.
* ``data/journal/figures/*.pdf`` — figures drawn from the raw CSVs.

No number is typed into this file.  If an experiment is missing, the script
aborts instead of substituting a placeholder.
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / "data" / "journal" / "raw"
FIG = ROOT / "data" / "journal" / "figures"
TEX = ROOT / "publications" / "ieee_tro_submission" / "numbers.tex"
LEDGER = ROOT / "data" / "journal" / "ledger.csv"

REQUIRED = ["P0-CONV", "P0-REACH", "P0-ESTOP", "P2-N4"]


def load_summary(exp: str) -> dict:
    path = RAW / f"{exp}_summary.json"
    if not path.exists():
        raise SystemExit(f"missing {path}; run {exp} first")
    return json.loads(path.read_text())


def rows(exp: str) -> list:
    path = RAW / f"{exp}.csv"
    if not path.exists():
        raise SystemExit(f"missing {path}")
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


_COMMIT_CACHE: dict[str, str] = {}


def row_commit(exp: str) -> str:
    """The single commit stamped on every row of an experiment."""
    if exp in _COMMIT_CACHE:
        return _COMMIT_CACHE[exp]
    commits = {r.get("commit", "") for r in rows(exp)}
    if len(commits) != 1 or "" in commits:
        raise SystemExit(
            f"{exp}: expected one non-empty commit on every row, found {sorted(commits)}"
        )
    _COMMIT_CACHE[exp] = commits.pop()
    return _COMMIT_CACHE[exp]


def f3(x) -> str:
    return f"{float(x):.3f}"


def f2(x) -> str:
    return f"{float(x):.2f}"


def pct(x, digits: int = 1) -> str:
    return f"{100.0 * float(x):.{digits}f}"


def pval(p: float) -> str:
    p = float(p)
    if p <= 0.0:
        return r"$p<10^{-300}$"
    if p < 0.001:
        return f"$p={p:.1e}$"
    return f"$p={p:.3f}$"


def sci(x, digits: int = 2) -> str:
    """LaTeX scientific notation for a small magnitude."""
    x = float(x)
    if x == 0.0:
        return "0"
    exp = int(np.floor(np.log10(abs(x))))
    mant = x / (10.0 ** exp)
    return rf"${mant:.{digits}f}\times10^{{{exp}}}$"


def main() -> int:
    for exp in REQUIRED:
        if not (RAW / f"{exp}_summary.json").exists():
            raise SystemExit(f"missing summary for {exp}")

    conv = load_summary("P0-CONV")
    reach = load_summary("P0-REACH")
    estop = load_summary("P0-ESTOP")
    n4 = load_summary("P2-N4")

    conv_rows = rows("P0-CONV")
    estop_rows = rows("P0-ESTOP")
    n4_rows = rows("P2-N4")

    for exp, summary in (("P0-CONV", conv), ("P0-REACH", reach),
                         ("P0-ESTOP", estop), ("P2-N4", n4)):
        rc = row_commit(exp)
        if summary.get("commit") and summary["commit"] != rc:
            print(
                f"warning: {exp} summary commit {summary['commit'][:7]} differs from "
                f"row commit {rc[:7]}; using the row commit",
                file=sys.stderr,
            )

    macros: list[tuple[str, str]] = []
    ledger: list[dict] = []

    def add(name: str, value: str, *, exp: str, metric: str, unit: str,
            n: int | str = "", ci_low="", ci_high="", source: str = "",
            note: str = "") -> None:
        macros.append((name, value))
        ledger.append({
            "macro": name,
            "exp_id": exp,
            "metric": metric,
            "value": value,
            "unit": unit,
            "n": n,
            "ci_low": ci_low,
            "ci_high": ci_high,
            "source_file": source or f"data/journal/raw/{exp}_summary.json",
            "source_commit": row_commit(exp),
            "note": note,
        })

    # ---------------------------------------------------------------- conveyor
    add("convN", str(conv["n"]), exp="P0-CONV", metric="trials", unit="count",
        n=conv["n"], note="40 seeds x 3 commanded speeds")
    for label, speed in (("A", "0.04"), ("B", "0.08"), ("C", "0.12")):
        cell = conv[f"speed_{speed}"]
        add(f"convRealized{label}", f3(cell["median_realized_speed"]),
            exp="P0-CONV", metric=f"median realized part speed at {speed} m/s",
            unit="m/s", n=cell["n"],
            ci_low=f3(cell["median_realized_ci_low"]),
            ci_high=f3(cell["median_realized_ci_high"]))
        add(f"convRelErr{label}", pct(cell["median_rel_error"]),
            exp="P0-CONV", metric=f"median relative speed error at {speed} m/s",
            unit="percent", n=cell["n"])
        add(f"convBeltLink{label}", sci(cell["belt_link_speed_mean"]),
            exp="P0-CONV", metric=f"mean visual belt link speed at {speed} m/s",
            unit="m/s", n=cell["n"])
        add(f"convRoller{label}", f"{cell['head_roller_speed_mean']:.0f}",
            exp="P0-CONV", metric=f"mean roller angular speed at {speed} m/s",
            unit="rad/s", n=cell["n"])
    offsets = [
        float(conv[f"speed_{s}"]["median_speed_error"]) for s in ("0.04", "0.08", "0.12")
    ]
    add("convOffsetMean", f3(float(np.mean(offsets))), exp="P0-CONV",
        metric="mean of the three median speed offsets", unit="m/s",
        n=conv["n"], note="realized minus commanded")
    add("convTcpDevMax", sci(max(conv[f"speed_{s}"]["tcp_dev_mm_max"] for s in ("0.04","0.08","0.12"))),
        exp="P0-CONV", metric="max TCP deviation during conveyor operation",
        unit="mm", n=conv["n"])
    add("convWindowS", f"{float(conv_rows[0]['window_sim_s']):.1f}",
        exp="P0-CONV", metric="measurement window", unit="s", n=conv["n"])
    add("convReportedMatchPct", pct(conv["speed_reported_matches_cmd_fraction"]),
        exp="P0-CONV", metric="fraction of trials whose reported speed equals command",
        unit="percent", n=conv["n"])
    add("convRtf", f2(conv["rtf_mean"]), exp="P0-CONV",
        metric="mean real-time factor", unit="ratio", n=conv["n"])

    # ------------------------------------------------------------- reachability
    cur, old = reach["current"], reach["old_8d83b97"]
    add("reachN", f"{reach['n']:,}".replace(",", "{,}"), exp="P0-REACH",
        metric="target poses", unit="count", n=reach["n"])
    add("reachRecallCurrentPct", pct(cur["recall_5mm"], 2), exp="P0-REACH",
        metric="recall at 5 mm, current solver", unit="percent", n=reach["n"],
        ci_low=pct(cur["wilson_low"], 2), ci_high=pct(cur["wilson_high"], 2))
    add("reachRecallCurrentLoPct", pct(cur["wilson_low"], 2), exp="P0-REACH",
        metric="Wilson lower bound, current solver", unit="percent", n=reach["n"])
    add("reachRecallCurrentHiPct", pct(cur["wilson_high"], 2), exp="P0-REACH",
        metric="Wilson upper bound, current solver", unit="percent", n=reach["n"])
    add("reachRecallOldPct", pct(old["recall_5mm"], 3), exp="P0-REACH",
        metric="recall at 5 mm, historical solver", unit="percent", n=reach["n"],
        ci_low=pct(old["wilson_low"], 3), ci_high=pct(old["wilson_high"], 3))
    add("reachRecallOldLoPct", pct(old["wilson_low"], 3), exp="P0-REACH",
        metric="Wilson lower bound, historical solver", unit="percent", n=reach["n"])
    add("reachRecallOldHiPct", pct(old["wilson_high"], 3), exp="P0-REACH",
        metric="Wilson upper bound, historical solver", unit="percent", n=reach["n"])
    add("reachMedianErrCurrentMm", f3(cur["median_err_mm"]), exp="P0-REACH",
        metric="median position error, current solver", unit="mm", n=reach["n"])
    add("reachTailErrCurrentMm", f3(cur["p95_err_mm"]), exp="P0-REACH",
        metric="p95 position error, current solver", unit="mm", n=reach["n"])
    add("reachMedianErrOldMm", f3(old["median_err_mm"]), exp="P0-REACH",
        metric="median position error, historical solver", unit="mm", n=reach["n"])
    add("reachTailErrOldMm", f3(old["p95_err_mm"]), exp="P0-REACH",
        metric="p95 position error, historical solver", unit="mm", n=reach["n"])
    add("reachMedianSolveCurrentMs", f3(cur["median_solve_ms"]), exp="P0-REACH",
        metric="median solve time, current solver", unit="ms", n=reach["n"])
    add("reachMedianSolveOldMs", f3(old["median_solve_ms"]), exp="P0-REACH",
        metric="median solve time, historical solver", unit="ms", n=reach["n"])
    add("reachMcNemarB", str(reach["mcnemar_b_current_only"]), exp="P0-REACH",
        metric="discordant pairs, current only", unit="count", n=reach["n"])
    add("reachMcNemarC", str(reach["mcnemar_c_old_only"]), exp="P0-REACH",
        metric="discordant pairs, historical only", unit="count", n=reach["n"])
    add("reachMcNemarP", pval(reach["mcnemar_p"]), exp="P0-REACH",
        metric="McNemar exact p", unit="p-value", n=reach["n"])
    add("reachGapExplainedPct", pct(reach["fraction_of_historical_gap_explained"]),
        exp="P0-REACH", metric="fraction of historical gap explained",
        unit="percent", n=reach["n"],
        note=">100% means the revision difference is larger than the reported gap")

    # -------------------------------------------------------------------- E-STOP
    add("estopN", str(estop["n"]), exp="P0-ESTOP", metric="trials", unit="count",
        n=estop["n"], note="50 seeds x 2 paths")
    for path, label in (("command", "Command"), ("safety", "Safety")):
        cell = estop[path]
        add(f"estop{label}Success", f"{cell['halt_success_rate']*100:.0f}",
            exp="P0-ESTOP", metric=f"halt success, {path} path", unit="percent",
            n=cell["n"], ci_low=pct(cell["halt_success_wilson_low"]),
            ci_high=pct(cell["halt_success_wilson_high"]))
        add(f"estop{label}TzeroMs", f2(cell["t_zero_cmd_median_ms"]),
            exp="P0-ESTOP", metric=f"median signal-to-message latency, {path}",
            unit="ms", n=cell["n"])
        if cell["t_stop_median_s"] is not None:
            add(f"estop{label}TstopMs", f"{1000*cell['t_stop_median_s']:.0f}",
                exp="P0-ESTOP", metric=f"median stop time, {path}", unit="ms",
                n=cell["n"])
            add(f"estop{label}TstopTailMs", f"{1000*cell['t_stop_p95_s']:.0f}",
                exp="P0-ESTOP", metric=f"p95 stop time, {path}", unit="ms",
                n=cell["n"])
        add(f"estop{label}OvershootMed", f3(cell["overshoot_median_rad"]),
            exp="P0-ESTOP", metric=f"median overshoot, {path}", unit="rad",
            n=cell["n"])
        add(f"estop{label}Joints", f"{cell['halt_joint_count_median']:.0f}",
            exp="P0-ESTOP", metric=f"median halt joint count, {path}",
            unit="count", n=cell["n"])
        add(f"estop{label}HasVel", f"{cell['halt_has_velocities_fraction']*100:.0f}",
            exp="P0-ESTOP", metric=f"fraction of halt messages with velocities, {path}",
            unit="percent", n=cell["n"])
        add(f"estop{label}SuccessSensitivity",
            f"{cell['halt_success_005_rate']*100:.0f}",
            exp="P0-ESTOP", metric=f"halt success at 0.05 rad/s, {path}",
            unit="percent", n=cell["n"])
        if cell["t_stop_005_median_s"] is not None:
            sens_ms = 1000 * cell["t_stop_005_median_s"]
            add(f"estop{label}TstopSensitivityMs",
                f"{sens_ms:.0f}", exp="P0-ESTOP",
                metric=f"median stop time at 0.05 rad/s, {path}", unit="ms",
                n=cell["n"])

    # ----------------------------------------------------------------------- N4
    base, n4c = n4["baseline"], n4["n4"]
    add("nFourN", str(n4["n"]), exp="P2-N4", metric="planned targets",
        unit="count", n=n4["n"], note="100 targets x 2 pipelines")
    add("nFourBaselineSuccessPct", pct(base["rate"]), exp="P2-N4",
        metric="intention-to-treat success, verification off", unit="percent",
        n=base["n"], ci_low=pct(base["wilson_low"]), ci_high=pct(base["wilson_high"]))
    add("nFourBaselineLoPct", pct(base["wilson_low"]), exp="P2-N4",
        metric="Wilson lower bound, verification off", unit="percent", n=base["n"])
    add("nFourBaselineHiPct", pct(base["wilson_high"]), exp="P2-N4",
        metric="Wilson upper bound, verification off", unit="percent", n=base["n"])
    add("nFourSuccessPct", pct(n4c["rate"]), exp="P2-N4",
        metric="intention-to-treat success, N4 verification on", unit="percent",
        n=n4c["n"], ci_low=pct(n4c["wilson_low"]), ci_high=pct(n4c["wilson_high"]))
    add("nFourNFourLoPct", pct(n4c["wilson_low"]), exp="P2-N4",
        metric="Wilson lower bound, N4", unit="percent", n=n4c["n"])
    add("nFourNFourHiPct", pct(n4c["wilson_high"]), exp="P2-N4",
        metric="Wilson upper bound, N4", unit="percent", n=n4c["n"])
    add("nFourRiskDiffPct", f"{100*n4['paired']['risk_difference_n4_minus_baseline']:+.1f}",
        exp="P2-N4", metric="paired risk difference, N4 minus baseline",
        unit="percentage points", n=n4["paired"]["n_pairs"],
        ci_low=f"{100*n4['paired']['risk_difference_ci_low']:+.1f}",
        ci_high=f"{100*n4['paired']['risk_difference_ci_high']:+.1f}")
    add("nFourRiskDiffLoPct", f"{100*n4['paired']['risk_difference_ci_low']:+.1f}",
        exp="P2-N4", metric="paired risk difference lower bound",
        unit="percentage points", n=n4["paired"]["n_pairs"])
    add("nFourRiskDiffHiPct", f"{100*n4['paired']['risk_difference_ci_high']:.1f}",
        exp="P2-N4", metric="paired risk difference upper bound",
        unit="percentage points", n=n4["paired"]["n_pairs"])
    add("nFourP", pval(n4["paired"]["mcnemar_p"]), exp="P2-N4",
        metric="McNemar exact p", unit="p-value", n=n4["paired"]["n_pairs"])
    add("nFourFalseSuccessPct", pct(base["false_success_rate"]), exp="P2-N4",
        metric="baseline solver success flag with >5 mm FK error", unit="percent",
        n=base["n"])
    add("nFourBaselineExecFailPct", pct(base["executed_failure_rate"]), exp="P2-N4",
        metric="execution failure rate among baseline plans", unit="percent",
        n=base["n"])
    add("nFourExecFailPct", pct(n4c["executed_failure_rate"]), exp="P2-N4",
        metric="execution failure rate among N4-executed plans", unit="percent",
        n=int(round(n4c["coverage"] * n4c["n"])))
    pair_rows = {}
    for r in n4_rows:
        pair_rows.setdefault(r["seed"], {})[r["method"]] = r
    pairs = [v for v in pair_rows.values() if "baseline" in v and "n4" in v]
    add("nFourBothSuccess", str(sum(
        v["baseline"]["success"] == "1" and v["n4"]["success"] == "1" for v in pairs)),
        exp="P2-N4", metric="targets solved by both pipelines", unit="count",
        n=len(pairs))
    add("nFourBaselineOnly", str(sum(
        v["baseline"]["success"] == "1" and v["n4"]["success"] == "0" for v in pairs)),
        exp="P2-N4", metric="targets solved only by the baseline", unit="count",
        n=len(pairs))
    add("nFourOnly", str(sum(
        v["baseline"]["success"] == "0" and v["n4"]["success"] == "1" for v in pairs)),
        exp="P2-N4", metric="targets solved only by N4", unit="count",
        n=len(pairs))
    add("nFourNeither", str(sum(
        v["baseline"]["success"] == "0" and v["n4"]["success"] == "0" for v in pairs)),
        exp="P2-N4", metric="targets solved by neither pipeline", unit="count",
        n=len(pairs))
    add("nFourCoveragePct", pct(n4c["coverage"]), exp="P2-N4",
        metric="N4 coverage (executed / total)", unit="percent", n=n4c["n"])
    add("nFourReplansMean", f2(n4c["mean_replans"]), exp="P2-N4",
        metric="mean replans per target, N4", unit="count", n=n4c["n"])

    # ------------------------------------------------------------------ figures
    FIG.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 9, "pdf.fonttype": 42})

    # Fig 1: realized vs commanded conveyor speed.
    fig, ax = plt.subplots(figsize=(3.4, 2.5))
    cmds = [0.04, 0.08, 0.12]
    med = [conv[f"speed_{s}"]["median_realized_speed"] for s in ("0.04", "0.08", "0.12")]
    lo = [conv[f"speed_{s}"]["median_realized_ci_low"] for s in ("0.04", "0.08", "0.12")]
    hi = [conv[f"speed_{s}"]["median_realized_ci_high"] for s in ("0.04", "0.08", "0.12")]
    ax.errorbar(cmds, med, yerr=[np.array(med) - np.array(lo), np.array(hi) - np.array(med)],
                marker="o", capsize=3, label="realized (median, 95% CI)")
    ax.plot([0, 0.13], [0, 0.13], "--", color="0.5", label="commanded = realized")
    ax.set_xlabel("commanded belt speed (m/s)")
    ax.set_ylabel("realized part speed (m/s)")
    ax.set_xlim(0, 0.13)
    ax.set_ylim(0, 0.13)
    ax.legend(frameon=False, fontsize=7, loc="upper left")
    fig.tight_layout()
    fig.savefig(FIG / "fig_conv_speed.pdf")
    plt.close(fig)

    # Fig 2: relative error by commanded speed.
    fig, ax = plt.subplots(figsize=(3.4, 2.5))
    rel = [100 * conv[f"speed_{s}"]["median_rel_error"] for s in ("0.04", "0.08", "0.12")]
    ax.bar([f"{c:.2f}" for c in cmds], rel, color=["#4c72b0", "#dd8452", "#55a868"])
    ax.axhline(0, color="0.2", linewidth=0.8)
    ax.set_ylabel("median relative error (%)")
    ax.set_xlabel("commanded belt speed (m/s)")
    for i, v in enumerate(rel):
        ax.text(i, v - 3, f"{v:.1f}", ha="center", va="top", fontsize=7)
    fig.tight_layout()
    fig.savefig(FIG / "fig_conv_error.pdf")
    plt.close(fig)

    # Fig 3: reachability error ECDF.
    fig, ax = plt.subplots(figsize=(3.4, 2.5))
    for solver, label in (("current", "current"), ("old_8d83b97", "historical")):
        vals = np.array([float(r["err_mm"]) for r in rows("P0-REACH") if r["solver"] == solver])
        vals = np.sort(vals)
        y = np.arange(1, vals.size + 1) / vals.size
        ax.plot(vals, y, label=label, linewidth=1.2)
    ax.set_xscale("log")
    ax.set_xlabel("position error (mm)")
    ax.set_ylabel("ECDF")
    ax.legend(frameon=False, fontsize=7)
    fig.tight_layout()
    fig.savefig(FIG / "fig_reach_ecdf.pdf")
    plt.close(fig)

    # Fig 4: E-STOP overshoot per path.
    fig, ax = plt.subplots(figsize=(3.4, 2.5))
    data = [
        [float(r["overshoot_rad"]) for r in estop_rows if r["path"] == "command"],
        [float(r["overshoot_rad"]) for r in estop_rows if r["path"] == "safety"],
    ]
    ax.boxplot(data, tick_labels=["command", "safety-agent"], widths=0.5)
    ax.set_ylabel("post-signal overshoot (rad)")
    fig.tight_layout()
    fig.savefig(FIG / "fig_estop_overshoot.pdf")
    plt.close(fig)

    # Fig 5: N4 intention-to-treat success with Wilson CIs.
    fig, ax = plt.subplots(figsize=(3.0, 2.4))
    labels = ["off", "N4"]
    vals = [100 * base["rate"], 100 * n4c["rate"]]
    err_lo = [100 * (base["rate"] - base["wilson_low"]), 100 * (n4c["rate"] - n4c["wilson_low"])]
    err_hi = [100 * (base["wilson_high"] - base["rate"]), 100 * (n4c["wilson_high"] - n4c["rate"])]
    ax.bar(labels, vals, yerr=[err_lo, err_hi], capsize=4,
           color=["#8c8c8c", "#4c72b0"])
    ax.set_ylim(0, 100)
    ax.set_ylabel("intention-to-treat success (%)")
    fig.tight_layout()
    fig.savefig(FIG / "fig_n4_success.pdf")
    plt.close(fig)

    # ------------------------------------------------------------- write macros
    lines = [
        "% Auto-generated by scripts/build_numbers.py. Do not edit.",
        "% Every value is read from data/journal/raw; see data/journal/ledger.csv.",
    ]
    for name, value in macros:
        lines.append(rf"\newcommand{{\{name}}}{{{value}}}")
    TEX.write_text("\n".join(lines) + "\n")

    paper_fig = TEX.parent / "figures"
    paper_fig.mkdir(parents=True, exist_ok=True)
    import shutil
    for pdf in FIG.glob("*.pdf"):
        shutil.copy2(pdf, paper_fig / pdf.name)

    LEDGER.parent.mkdir(parents=True, exist_ok=True)
    with LEDGER.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[
            "macro", "exp_id", "metric", "value", "unit", "n",
            "ci_low", "ci_high", "source_file", "source_commit", "note",
        ])
        w.writeheader()
        w.writerows(ledger)

    print(f"wrote {TEX} ({len(macros)} macros)")
    print(f"wrote {LEDGER} ({len(ledger)} rows)")
    print(f"wrote 5 figures to {FIG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
