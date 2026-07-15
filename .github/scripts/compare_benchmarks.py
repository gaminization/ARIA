#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Benchmark Comparison Tool
Compares current benchmark results against stored baseline.
Fails CI if any metric regresses beyond threshold.

Usage:
  python3 .github/scripts/compare_benchmarks.py \
    --current benchmark_ik_20260701.csv \
    --baseline benchmarks/baseline_ik.csv \
    --threshold 0.10
═══════════════════════════════════════════════════════════════
"""
import argparse
import csv
import glob
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass
class BenchmarkMetric:
    name: str
    value: float
    unit: str = ''
    higher_is_better: bool = True


# Metrics tracked across commits
TRACKED_METRICS = {
    'ik_success_rate': {
        'display': 'IK Success Rate',
        'unit': '%',
        'higher_is_better': True,
    },
    'ik_mean_solve_time_ms': {
        'display': 'IK Mean Solve Time',
        'unit': 'ms',
        'higher_is_better': False,
    },
    'ik_position_error_mm': {
        'display': 'IK Position Error',
        'unit': 'mm',
        'higher_is_better': False,
    },
    'ik_p95_solve_time_ms': {
        'display': 'IK P95 Solve Time',
        'unit': 'ms',
        'higher_is_better': False,
    },
    'perception_inference_ms': {
        'display': 'Perception Inference Time',
        'unit': 'ms',
        'higher_is_better': False,
    },
    'task_success_rate': {
        'display': 'Task Success Rate',
        'unit': '%',
        'higher_is_better': True,
    },
}


def load_csv_metrics(filepath: str) -> Dict[str, float]:
    """Load benchmark metrics from a CSV file."""
    metrics = {}
    if not os.path.exists(filepath):
        return metrics

    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row.get('metric', row.get('name', ''))
            value = row.get('value', row.get('score', '0'))
            try:
                metrics[name.strip()] = float(value)
            except (ValueError, TypeError):
                pass
    return metrics


def load_simple_csv(filepath: str) -> Dict[str, float]:
    """Load metrics from a simple two-column CSV (metric,value)."""
    metrics = {}
    if not os.path.exists(filepath):
        return metrics

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('metric'):
                continue
            parts = line.split(',')
            if len(parts) >= 2:
                try:
                    metrics[parts[0].strip()] = float(parts[1].strip())
                except (ValueError, TypeError):
                    pass
    return metrics


def compare_metrics(
    current: Dict[str, float],
    baseline: Dict[str, float],
    threshold: float = 0.10,
) -> Tuple[bool, List[dict]]:
    """
    Compare current metrics against baseline.

    Returns:
        (all_pass, results_list)
        all_pass: True if no metric regressed beyond threshold
        results_list: per-metric comparison results
    """
    results = []
    all_pass = True

    for metric_key, info in TRACKED_METRICS.items():
        if metric_key not in current and metric_key not in baseline:
            continue

        current_val = current.get(metric_key)
        baseline_val = baseline.get(metric_key)

        if current_val is None or baseline_val is None:
            results.append({
                'name': info['display'],
                'status': 'SKIP',
                'current': current_val,
                'baseline': baseline_val,
                'change_pct': None,
                'message': 'Missing data',
            })
            continue

        # Calculate relative change
        if baseline_val == 0:
            change_pct = 0.0
        else:
            change_pct = (current_val - baseline_val) / abs(baseline_val)

        # Check regression
        higher_is_better = info['higher_is_better']
        if higher_is_better:
            # Regression = value decreased
            is_regression = change_pct < -threshold
        else:
            # Regression = value increased (e.g., solve time went up)
            is_regression = change_pct > threshold

        status = 'PASS'
        if is_regression:
            status = 'FAIL'
            all_pass = False
        elif abs(change_pct) > threshold * 0.5:
            status = 'WARN'

        results.append({
            'name': info['display'],
            'status': status,
            'current': current_val,
            'baseline': baseline_val,
            'change_pct': change_pct * 100,
            'unit': info['unit'],
            'message': '',
        })

    return all_pass, results


def print_report(results: List[dict], all_pass: bool):
    """Print formatted comparison report."""
    print("\n" + "═" * 65)
    print("  ARIA Benchmark Comparison Report")
    print("═" * 65)

    status_icons = {'PASS': '✅', 'FAIL': '❌', 'WARN': '⚠️', 'SKIP': '⏭️'}

    for r in results:
        icon = status_icons.get(r['status'], '?')
        name = r['name']
        unit = r.get('unit', '')

        if r['current'] is not None and r['baseline'] is not None:
            change = r.get('change_pct', 0)
            direction = '+' if change >= 0 else ''
            print(
                f"  {icon} {name:30s} "
                f"{r['baseline']:.2f}{unit} → {r['current']:.2f}{unit} "
                f"({direction}{change:.1f}%)")
        else:
            print(f"  {icon} {name:30s}  {r['message']}")

    print("═" * 65)
    if all_pass:
        print("  ✅ All benchmarks within acceptable range")
    else:
        print("  ❌ REGRESSION DETECTED — check metrics above")
        print("  Likely cause: check recent changes in this PR/commit")
    print("═" * 65 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description='ARIA Benchmark Comparison')
    parser.add_argument(
        '--current', required=True,
        help='Path to current benchmark CSV (supports glob)')
    parser.add_argument(
        '--baseline', required=True,
        help='Path to baseline benchmark CSV')
    parser.add_argument(
        '--threshold', type=float, default=0.10,
        help='Regression threshold (fraction, default 0.10 = 10%%)')

    args = parser.parse_args()

    # Load current (supports glob)
    current_files = glob.glob(args.current)
    if not current_files:
        print(f"⚠️  No current benchmark files matching: {args.current}")
        print("  Skipping comparison (no data).")
        sys.exit(0)

    current_metrics = {}
    for f in current_files:
        metrics = load_csv_metrics(f)
        if not metrics:
            metrics = load_simple_csv(f)
        current_metrics.update(metrics)

    # Load baseline
    baseline_metrics = load_csv_metrics(args.baseline)
    if not baseline_metrics:
        baseline_metrics = load_simple_csv(args.baseline)

    if not baseline_metrics:
        print(f"⚠️  No baseline data at: {args.baseline}")
        print("  First run — saving current as baseline.")

        # Save current as baseline
        os.makedirs(os.path.dirname(args.baseline) or '.', exist_ok=True)
        with open(args.baseline, 'w') as f:
            f.write("metric,value\n")
            for k, v in current_metrics.items():
                f.write(f"{k},{v}\n")
        sys.exit(0)

    # Compare
    all_pass, results = compare_metrics(
        current_metrics, baseline_metrics, args.threshold)
    print_report(results, all_pass)

    sys.exit(0 if all_pass else 1)


if __name__ == '__main__':
    main()
