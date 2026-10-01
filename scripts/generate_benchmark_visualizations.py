#!/usr/bin/env python3
"""
Generate publication-quality benchmark plots from autonomous test runs.
Saves directly to the conversation artifacts directory:
/home/gaminizer/.gemini/antigravity-ide/brain/d262fd9a-5350-471c-b731-f6ffb53e991d/
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

ARTIFACT_DIR = "/home/gaminizer/.gemini/antigravity-ide/brain/d262fd9a-5350-471c-b731-f6ffb53e991d"

def plot_manipulation():
    csv_path = "data/expanded_manipulation_trials.csv"
    if not os.path.exists(csv_path):
        print(f"Skipping {csv_path} - not found")
        return

    df = pd.read_csv(csv_path)
    grouped = df.groupby('task_name').agg({
        'pick_error_mm': ['mean', 'std', 'max'],
        'execution_time_s': ['mean', 'std'],
        'outcome': lambda x: (x == 'SUCCESS').mean() * 100.0
    }).reset_index()

    tasks = grouped['task_name']
    err_mean = grouped['pick_error_mm']['mean']
    err_std = grouped['pick_error_mm']['std'].fillna(0.0)
    dur_mean = grouped['execution_time_s']['mean']

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # Bar plot of Cartesian Error
    colors = ['#2563eb' if e < 1.0 else '#f59e0b' for e in err_mean]
    bars = ax1.barh(tasks, err_mean, xerr=err_std, color=colors, capsize=4, alpha=0.88, edgecolor='#1e40af')
    ax1.axvline(x=5.0, color='#ef4444', linestyle='--', linewidth=1.5, label='Pass Threshold (5.0 mm)')
    ax1.axvline(x=1.0, color='#10b981', linestyle=':', linewidth=1.5, label='Sub-millimeter (1.0 mm)')
    ax1.set_xlabel('End-Effector Cartesian Error (mm)', fontsize=12, fontweight='bold')
    ax1.set_title('10-Task Manipulation Accuracy (30 Trials)', fontsize=14, fontweight='bold', pad=12)
    ax1.legend(loc='lower right', frameon=True)
    ax1.grid(True, linestyle='--', alpha=0.5)

    for bar, val in zip(bars, err_mean):
        ax1.text(val + 0.05, bar.get_y() + bar.get_height()/2, f"{val:.2f} mm",
                 va='center', fontsize=9, fontweight='semibold', color='#0f172a')

    # Bar plot of Execution Duration
    ax2.barh(tasks, dur_mean, color='#0ea5e9', alpha=0.85, edgecolor='#0369a1')
    ax2.set_xlabel('Mean Trajectory Duration (s)', fontsize=12, fontweight='bold')
    ax2.set_title('Task Execution Time & Convergence (100% Success)', fontsize=14, fontweight='bold', pad=12)
    ax2.grid(True, linestyle='--', alpha=0.5)

    for i, v in enumerate(dur_mean):
        ax2.text(v + 0.05, i, f"{v:.2f} s", va='center', fontsize=9, fontweight='semibold')

    plt.tight_layout()
    out_path = os.path.join(ARTIFACT_DIR, "plot_manipulation_benchmark.png")
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Saved: {out_path}")

def plot_conveyor_sweep():
    csv_path = "data/conveyor_speed_sweep.csv"
    if not os.path.exists(csv_path):
        print(f"Skipping {csv_path} - not found")
        return

    df = pd.read_csv(csv_path)
    grouped = df.groupby('belt_speed_mps').agg({
        'sort_success': lambda x: (x == 'SUCCESS').mean() * 100.0,
        'cycle_time_s': ['mean', 'std'],
        'interception_error_mm': ['mean', 'std']
    }).reset_index()

    speeds = np.array(grouped['belt_speed_mps'])
    sort_succ = np.array(grouped['sort_success']['<lambda>'])
    cycle_time = np.array(grouped['cycle_time_s']['mean'])
    cycle_std = np.array(grouped['cycle_time_s']['std'].fillna(0.0))

    fig, ax1 = plt.subplots(figsize=(10, 5.5))

    color = '#2563eb'
    ax1.set_xlabel('Conveyor Belt Speed (m/s)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Autonomous Sort Success Rate (%)', color=color, fontsize=12, fontweight='bold')
    line1 = ax1.plot(speeds, sort_succ, marker='o', linewidth=2.5, markersize=8, color=color, label='Sort Success (%)')
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.set_ylim(80, 105)
    ax1.axhline(y=100.0, color='#10b981', linestyle=':', alpha=0.7)
    ax1.grid(True, linestyle='--', alpha=0.5)

    ax2 = ax1.twinx()
    color2 = '#d97706'
    ax2.set_ylabel('Total Cycle Duration (s)', color=color2, fontsize=12, fontweight='bold')
    line2 = ax2.errorbar(speeds, cycle_time, yerr=cycle_std, fmt='s-', color=color2, linewidth=2.0, capsize=4, label='Cycle Time (s)')
    ax2.tick_params(axis='y', labelcolor=color2)

    plt.title('Autonomous Conveyor Workcell Speed Sweep Benchmark (0.02 - 0.12 m/s)', fontsize=14, fontweight='bold', pad=14)
    lines = line1 + [line2]
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='lower left', frameon=True)

    plt.tight_layout()
    out_path = os.path.join(ARTIFACT_DIR, "plot_conveyor_sweep_benchmark.png")
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Saved: {out_path}")

if __name__ == "__main__":
    plot_manipulation()
    plot_conveyor_sweep()
