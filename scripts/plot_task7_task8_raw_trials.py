#!/usr/bin/env python3
"""
scripts/plot_task7_task8_raw_trials.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Plots 3 raw trials (seeds 101, 102, 103) for Task 07 and Task 08 from raw telemetry.
Shows true object and TCP trajectories from /gazebo/model_states and TF.
"""

import os
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
TELEMETRY_DIR = os.path.join(WORKSPACE_ROOT, "data", "real", "telemetry")
FIG_DIR = os.path.join(WORKSPACE_ROOT, "publications", "ieee_tro_submission", "figures")
os.makedirs(FIG_DIR, exist_ok=True)
OUT_PNG = os.path.join(FIG_DIR, "raw_trials_tasks_7_8.png")

seeds = [101, 102, 103]
fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=False)

# ── Task 07: Pivot / Reorientation (Seeds 101, 102, 103) ──
for idx, seed in enumerate(seeds):
    ax = axes[0, idx]
    path = os.path.join(TELEMETRY_DIR, f"SCRIPTED_task_07_SEED_{seed}_telemetry.csv")
    df = pd.read_csv(path)
    t = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) * 1e-9

    ax.plot(t, df['workpiece_x'] * 1000, label='Workpiece X', color='#1f77b4', lw=2)
    ax.plot(t, df['workpiece_y'] * 1000, label='Workpiece Y', color='#ff7f0e', lw=2)
    ax.plot(t, df['workpiece_z'] * 1000, label='Workpiece Z', color='#2ca02c', lw=2)
    ax.plot(t, df['tf_ee_x'] * 1000, '--', label='TCP X', color='#1f77b4', alpha=0.6)
    ax.plot(t, df['tf_ee_y'] * 1000, '--', label='TCP Y', color='#ff7f0e', alpha=0.6)
    ax.plot(t, df['tf_ee_z'] * 1000, '--', label='TCP Z', color='#2ca02c', alpha=0.6)

    init_p = np.array([df['workpiece_x'].iloc[0], df['workpiece_y'].iloc[0], df['workpiece_z'].iloc[0]])
    final_p = np.array([df['workpiece_x'].iloc[-1], df['workpiece_y'].iloc[-1], df['workpiece_z'].iloc[-1]])
    d_xyz = np.linalg.norm(final_p - init_p) * 1000

    ax.set_title(f"Task 07 (Pivot) - Seed {seed}\nActual Δpos={d_xyz:.1f} mm (Cmd err=4.011° hardcoded)", fontsize=10)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (mm)")
    ax.grid(True, alpha=0.3)
    if idx == 0:
        ax.legend(loc='lower left', fontsize=8)

# ── Task 08: Drawer / Slide (Seeds 101, 102, 103) ──
for idx, seed in enumerate(seeds):
    ax = axes[1, idx]
    path = os.path.join(TELEMETRY_DIR, f"SCRIPTED_task_08_SEED_{seed}_telemetry.csv")
    df = pd.read_csv(path)
    t = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) * 1e-9

    ax.plot(t, df['workpiece_x'] * 1000, label='Drawer/Obj X', color='#1f77b4', lw=2)
    ax.plot(t, df['workpiece_y'] * 1000, label='Drawer/Obj Y', color='#ff7f0e', lw=2)
    ax.plot(t, df['workpiece_z'] * 1000, label='Drawer/Obj Z', color='#2ca02c', lw=2)
    ax.plot(t, df['tf_ee_x'] * 1000, '--', label='TCP X', color='#1f77b4', alpha=0.6)
    ax.plot(t, df['tf_ee_y'] * 1000, '--', label='TCP Y', color='#ff7f0e', alpha=0.6)
    ax.plot(t, df['tf_ee_z'] * 1000, '--', label='TCP Z', color='#2ca02c', alpha=0.6)

    init_xy = np.array([df['workpiece_x'].iloc[0], df['workpiece_y'].iloc[0]])
    final_xy = np.array([df['workpiece_x'].iloc[-1], df['workpiece_y'].iloc[-1]])
    disp_xy = np.linalg.norm(final_xy - init_xy) * 1000

    ax.set_title(f"Task 08 (Drawer) - Seed {seed}\nActual Δxy={disp_xy:.1f} mm (Cmd disp=35.000 mm hardcoded)", fontsize=10)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (mm)")
    ax.grid(True, alpha=0.3)
    if idx == 0:
        ax.legend(loc='lower left', fontsize=8)

plt.suptitle("Raw Telemetry Trials (Seeds 101, 102, 103) for Manipulation Tasks 07 & 08", fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(OUT_PNG, dpi=300)
print(f"Successfully generated plot at: {OUT_PNG}")
