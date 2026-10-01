#!/usr/bin/env python3
"""
Regenerate Fig 1 (System Architecture) and Fig 7 (Manipulation Benchmark)
strictly conforming to IEEE style, paper_v2 text, Table IV, Table IX, Table X, and Table XIX.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# IEEE styling parameters
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Helvetica']
plt.rcParams['axes.edgecolor'] = '#333333'
plt.rcParams['axes.linewidth'] = 0.8
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'

OUT_DIRS = [
    "/home/gaminizer/Projects/ARIA/paper_v2/figures",
    "/home/gaminizer/Projects/ARIA/paper/figures",
    "/home/gaminizer/Projects/ARIA/new_paper/figures"
]

def generate_fig1_architecture():
    fig, ax = plt.subplots(figsize=(12.0, 8.0))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis('off')
    fig.patch.set_facecolor('#ffffff')

    c_tier1 = '#2B4162' # Perception - deep navy slate
    c_tier2 = '#1E5E58' # Scene Modeling - deep pine teal
    c_tier3 = '#B3541E' # Planning - warm rust amber
    c_bus   = '#114B5F' # Central State Bus - rich deep teal
    c_tier4 = '#315659' # Control & Skills - dark cyan slate
    c_tier5 = '#453F78' # Diagnostics & LfD - deep violet slate

    # Title Banner
    ax.text(50, 98.4, 'ARIA: MODULAR ASYNCHRONOUS MULTI-AGENT ROBOTIC ARCHITECTURE', 
            ha='center', va='center', fontsize=12.5, fontweight='bold', color='#111111')
    ax.text(50, 96.0, 'Decoupled Asynchronous Lifecycle Nodes across Five Operational Tiers with Shared ROS 2 DDS State Bus',
            ha='center', va='center', fontsize=8.2, color='#444444', style='italic')

    # ══════════════════════════════════════════════════════════
    # TIER 1: PERCEPTION LAYER (4 Agents)
    # ══════════════════════════════════════════════════════════
    r1 = patches.FancyBboxPatch((2.5, 80.0), 95.0, 14.2, boxstyle="round,pad=0.4", ec=c_tier1, fc='#F2F5F9', lw=1.5)
    ax.add_patch(r1)
    badge1 = patches.FancyBboxPatch((4.5, 91.2), 37.0, 2.5, boxstyle="round,pad=0.2", ec=c_tier1, fc='#DCE4EE', lw=1.0)
    ax.add_patch(badge1)
    ax.text(23.0, 92.4, 'TIER 1: PERCEPTION LAYER (Asynchronous 10–30 Hz)', fontsize=7.2, fontweight='bold', color=c_tier1, ha='center', va='center')

    tier1_boxes = [
        ('VisionAgent (30 Hz • 1,850 MB)\nOverhead Eye-to-Hand (C270)\nYOLOv8m Detection + SAM2 Masks', 4.5, 81.0, 22.0, 9.2),
        ('DepthAgent (15 Hz • 1,420 MB)\nWrist Eye-in-Hand (ESP32-CAM)\nDepth-Anything v2 + Claw Grounding', 28.5, 81.0, 22.5, 9.2),
        ('TrackingAgent (30 Hz • 120 MB)\nSORT / Kalman 2D Tracklet Filter\nContinuous ID & Velocity State', 53.0, 81.0, 21.0, 9.2),
        ('AttentionAgent (10 Hz • 30 MB)\nDynamic Task ROI Saliency\nFocal Bounding-Box Cropping', 76.0, 81.0, 19.5, 9.2)
    ]
    for text, x, y, w, h in tier1_boxes:
        b = patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.3", ec=c_tier1, fc='#FFFFFF', lw=1.1)
        ax.add_patch(b)
        ax.text(x+w/2, y+h/2, text, ha='center', va='center', fontsize=6.8, color='#222222', multialignment='center')

    # ══════════════════════════════════════════════════════════
    # TIER 2: SCENE MODELING LAYER (3 Agents)
    # ══════════════════════════════════════════════════════════
    r2 = patches.FancyBboxPatch((2.5, 63.0), 46.5, 14.5, boxstyle="round,pad=0.4", ec=c_tier2, fc='#F0F7F6', lw=1.5)
    ax.add_patch(r2)
    badge2 = patches.FancyBboxPatch((4.5, 74.5), 32.0, 2.5, boxstyle="round,pad=0.2", ec=c_tier2, fc='#D8ECE9', lw=1.0)
    ax.add_patch(badge2)
    ax.text(20.5, 75.7, 'TIER 2: SCENE MODELING LAYER', fontsize=7.2, fontweight='bold', color=c_tier2, ha='center', va='center')

    tier2_boxes = [
        ('WorldModelAgent\n(20 Hz • 150 MB)\n3D Scene Graph &\nCentroid Tracking', 4.5, 64.0, 14.0, 9.5),
        ('MemoryAgent\n(Event • 60 MB)\nEpisodic SQLite\nState & History DB', 20.0, 64.0, 13.5, 9.5),
        ('AffordanceAgent\n(10 Hz • 220 MB)\nGraspNet Normals &\nFriction Cone (μ=0.45)', 35.0, 64.0, 12.5, 9.5)
    ]
    for text, x, y, w, h in tier2_boxes:
        b = patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.25", ec=c_tier2, fc='#FFFFFF', lw=1.0)
        ax.add_patch(b)
        ax.text(x+w/2, y+h/2, text, ha='center', va='center', fontsize=6.3, color='#222222', multialignment='center')

    # ══════════════════════════════════════════════════════════
    # TIER 3: TASK PLANNING LAYER (3 Agents)
    # ══════════════════════════════════════════════════════════
    r3 = patches.FancyBboxPatch((51.0, 63.0), 46.5, 14.5, boxstyle="round,pad=0.4", ec=c_tier3, fc='#FFF7F0', lw=1.5)
    ax.add_patch(r3)
    badge3 = patches.FancyBboxPatch((53.0, 74.5), 33.0, 2.5, boxstyle="round,pad=0.2", ec=c_tier3, fc='#FFE6D4', lw=1.0)
    ax.add_patch(badge3)
    ax.text(69.5, 75.7, 'TIER 3: TASK PLANNING LAYER', fontsize=7.2, fontweight='bold', color=c_tier3, ha='center', va='center')

    tier3_boxes = [
        ('PlanningAgent\n(Event • 3,450 MB)\nQuantized Llama-3.1/Mistral\nTree-of-Thoughts (ToT)', 53.0, 64.0, 14.5, 9.5),
        ('DialogueAgent\n(Event • 50 MB)\nHITL Confidence Gate\n(τ = 0.70 Escalation)', 69.0, 64.0, 13.0, 9.5),
        ('ReachabilityAgent\n(50 Hz • 0 MB CPU)\nAnalytical IK Feasibility\nManifold M_task Bounds', 83.5, 64.0, 12.5, 9.5)
    ]
    for text, x, y, w, h in tier3_boxes:
        b = patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.25", ec=c_tier3, fc='#FFFFFF', lw=1.0)
        ax.add_patch(b)
        ax.text(x+w/2, y+h/2, text, ha='center', va='center', fontsize=6.3, color='#222222', multialignment='center')

    # ══════════════════════════════════════════════════════════
    # CENTRAL BACKBONE: UNIFIED ASYNCHRONOUS STATE BUS
    # ══════════════════════════════════════════════════════════
    rb = patches.FancyBboxPatch((2.5, 51.0), 95.0, 8.5, boxstyle="round,pad=0.4", ec=c_bus, fc='#EBF5F5', lw=2.0)
    ax.add_patch(rb)
    ax.text(50, 56.6, 'UNIFIED ASYNCHRONOUS STATE BUS  (/aria/state/*)', 
            ha='center', va='center', fontsize=9.2, fontweight='bold', color=c_bus)
    ax.text(50, 53.2, 'ROS 2 DDS Pub/Sub  •  TaskState  •  VisionState  •  MemoryState  •  HealthState  •  HITL Dialogue Stream', 
            ha='center', va='center', fontsize=7.6, color='#114440')

    # Connecting arrows around State Bus
    ax.annotate('', xy=(25, 80.0), xytext=(25, 78.0), arrowprops=dict(arrowstyle="->", color=c_tier1, lw=1.5))
    ax.annotate('', xy=(75, 80.0), xytext=(75, 78.0), arrowprops=dict(arrowstyle="->", color=c_tier1, lw=1.5))
    ax.annotate('', xy=(25, 63.0), xytext=(25, 59.5), arrowprops=dict(arrowstyle="<->", color=c_tier2, lw=1.5))
    ax.annotate('', xy=(75, 63.0), xytext=(75, 59.5), arrowprops=dict(arrowstyle="<->", color=c_tier3, lw=1.5))

    # ══════════════════════════════════════════════════════════
    # TIER 4: CONTROL & SKILLS LAYER (3 Agents)
    # ══════════════════════════════════════════════════════════
    r4 = patches.FancyBboxPatch((2.5, 31.5), 95.0, 16.5, boxstyle="round,pad=0.4", ec=c_tier4, fc='#F0F4F8', lw=1.5)
    ax.add_patch(r4)
    badge4 = patches.FancyBboxPatch((4.5, 45.0), 43.0, 2.5, boxstyle="round,pad=0.2", ec=c_tier4, fc='#D8E4F0', lw=1.0)
    ax.add_patch(badge4)
    ax.text(26.0, 46.2, 'TIER 4: CONTROL & PROCEDURAL SKILLS LAYER (50–100 Hz)', fontsize=7.2, fontweight='bold', color=c_tier4, ha='center', va='center')

    tier4_boxes = [
        ('SkillAgent (50 Hz • 40 MB VRAM)\n10 Composable Procedural Skills\nQuintic Polynomial Rendezvous (Δv ≤ 0.0075 m/s)\nDynamic Trajectory Interpolation', 4.5, 32.7, 28.5, 11.2),
        ('ControlAgent (50 Hz • 0 MB CPU)\nClosed-Form Analytical IK Solver (0.075 ms)\n50 Hz Joint Setpoint Streamer & Servo Drivers\nODE Dynamic Model / Hardware Interface', 35.5, 32.7, 29.0, 11.2),
        ('SafetyAgent (100 Hz • 20 MB VRAM)\nInterrupt-Driven E-STOP Guard (<5 ms)\nBoundary Violation & Dynamic Collision Arrest\nLifecycle Fault Recovery Supervisor', 66.5, 32.7, 29.5, 11.2)
    ]
    for text, x, y, w, h in tier4_boxes:
        b = patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.3", ec=c_tier4, fc='#FFFFFF', lw=1.1)
        ax.add_patch(b)
        ax.text(x+w/2, y+h/2, text, ha='center', va='center', fontsize=6.8, color='#222222', multialignment='center')

    # Connecting arrows from State Bus to Tier 4
    ax.annotate('', xy=(20, 48.0), xytext=(20, 51.0), arrowprops=dict(arrowstyle="<-", color=c_tier4, lw=1.5))
    ax.annotate('', xy=(50, 48.0), xytext=(50, 51.0), arrowprops=dict(arrowstyle="<-", color=c_tier4, lw=1.5))
    ax.annotate('', xy=(80, 48.0), xytext=(80, 51.0), arrowprops=dict(arrowstyle="<->", color=c_tier4, lw=1.5))

    # ══════════════════════════════════════════════════════════
    # TIER 5: DIAGNOSTICS & LfD LAYER (2 Agents)
    # ══════════════════════════════════════════════════════════
    r5 = patches.FancyBboxPatch((2.5, 10.5), 95.0, 17.5, boxstyle="round,pad=0.4", ec=c_tier5, fc='#F5F4F7', lw=1.5)
    ax.add_patch(r5)
    badge5 = patches.FancyBboxPatch((4.5, 25.0), 45.0, 2.5, boxstyle="round,pad=0.2", ec=c_tier5, fc='#E2E0EA', lw=1.0)
    ax.add_patch(badge5)
    ax.text(27.0, 26.2, 'TIER 5: DIAGNOSTICS & LEARNING-FROM-DEMONSTRATION (LfD)', fontsize=7.2, fontweight='bold', color=c_tier5, ha='center', va='center')

    tier5_boxes = [
        ('EvaluationAgent (Event • 40 MB VRAM)\nSystematic Taxonomy Across 5 Failure Categories (A–E: Occlusion, Slip, Jitter, Ambiguity, Solver)\nAutomated Retrospective Recovery Gating & Online Metric Convergence Profiler', 4.5, 12.0, 44.0, 12.0),
        ('LearningAgent (50 Hz • 80 MB VRAM)\nHDF5 Synchronous Multimodal Demonstration Trajectory Recorder\nTrajectory Replay & Imitation Learning Data Preprocessing Modules', 50.5, 12.0, 45.5, 12.0)
    ]
    for text, x, y, w, h in tier5_boxes:
        b = patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.3", ec=c_tier5, fc='#FFFFFF', lw=1.1)
        ax.add_patch(b)
        ax.text(x+w/2, y+h/2, text, ha='center', va='center', fontsize=6.8, color='#222222', multialignment='center')

    # Bottom Foundation Bar
    rf = patches.FancyBboxPatch((2.5, 2.0), 95.0, 6.5, boxstyle="round,pad=0.3", ec='#777777', fc='#F8F9FA', lw=1.0)
    ax.add_patch(rf)
    ax.text(50, 6.2, 'PHYSICAL EMBODIMENT & DIGITAL TWIN EXECUTION LAYER', 
            ha='center', va='center', fontsize=7.6, fontweight='bold', color='#333333')
    ax.text(50, 3.8, 'Gazebo 11 / ODE Calibrated Physics Simulation (k_θ = 14.2 N·m/rad, μ = 0.45)  •  micro-ROS PCA9685 12-Bit Servo Actuation Pipeline', 
            ha='center', va='center', fontsize=6.8, color='#555555')

    ax.annotate('', xy=(50, 28.0), xytext=(50, 31.5), arrowprops=dict(arrowstyle="<->", color='#555555', lw=1.4))
    ax.annotate('', xy=(50, 8.5), xytext=(50, 10.5), arrowprops=dict(arrowstyle="<->", color='#555555', lw=1.4))

    plt.tight_layout()
    for d in OUT_DIRS:
        os.makedirs(d, exist_ok=True)
        out_file = os.path.join(d, "fig1_system_architecture.png")
        plt.savefig(out_file, dpi=300, bbox_inches='tight')
        print(f"Saved: {out_file}")
    plt.close()


def generate_fig7_vla_benchmark():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.5, 3.8), gridspec_kw={'width_ratios': [1.45, 1]})

    # Panel (a): 10 Tasks + Overall Mean
    tasks = ['T1: Reach', 'T2: Pick', 'T3: Conveyor', 'T4: Tray', 'T5: Obstacle',
             'T6: Servoing', 'T7: Pivoting', 'T8: Slide', 'T9: Stacking', 'T10: Preempt', 'Mean']
    
    aria_primary = [100.0, 90.0, 90.0, 90.0, 90.0, 90.0, 80.0, 90.0, 80.0, 90.0, 89.0]
    lerobot_act  = [90.0,  80.0, 50.0, 80.0, 80.0, 80.0, 50.0, 80.0, 70.0, 70.0, 73.0]
    openvla_7b   = [90.0,  70.0, 40.0, 70.0, 80.0, 80.0, 40.0, 80.0, 60.0, 80.0, 69.0]

    x = np.arange(len(tasks))
    w = 0.25

    ax1.bar(x - w, aria_primary, w, label='ARIA (Primary $N=100$, Ours)', color='#1E3D59', edgecolor='#0F1E2C', lw=0.7)
    ax1.bar(x,     lerobot_act,  w, label='LeRobot ACT (80M)', color='#C85A17', edgecolor='#7D340A', lw=0.7)
    ax1.bar(x + w, openvla_7b,   w, label='OpenVLA-7B (Prismatic)', color='#136762', edgecolor='#0B3B38', lw=0.7)

    # Highlight mean column
    ax1.axvline(x=9.5, color='#888888', linestyle='--', linewidth=0.8)

    ax1.set_ylabel('Success Rate (%)', fontsize=8.5, fontweight='bold')
    ax1.set_title('(a) Task Success Rate Across 10 Manipulation Benchmarks', fontsize=9.0, fontweight='bold')
    ax1.set_xticks(x)
    ax1.set_xticklabels(tasks, rotation=28, ha='right', fontsize=7.2)
    ax1.set_ylim(0, 115)
    ax1.legend(loc='upper right', fontsize=7.0, ncol=3, framealpha=0.9)
    ax1.grid(True, linestyle='--', alpha=0.35, axis='y')

    # Panel (b): Transit Duration vs Step Control / Inference Latency
    models = ['ARIA (Ours)', 'LeRobot ACT', 'OpenVLA-7B']
    durations = [2.25, 3.8, 5.1]     # Table IX
    dur_err   = [0.40, 0.6, 1.2]     # Table IX
    latencies = [20, 35, 140]        # Table IX

    x2 = np.arange(len(models))
    w2 = 0.32

    ax2_twin = ax2.twinx()
    b1 = ax2.bar(x2 - w2/2, durations, yerr=dur_err, capsize=3.5, width=w2, 
                 color='#1E3D59', edgecolor='#0F1E2C', lw=0.7, label='Transit Duration (s)')
    b2 = ax2_twin.bar(x2 + w2/2, latencies, width=w2, 
                      color='#C85A17', edgecolor='#7D340A', lw=0.7, label='Control / Step Latency (ms)')

    ax2.set_ylabel('Mean Transit Duration (s)', color='#1E3D59', fontsize=8.5, fontweight='bold')
    ax2.set_ylim(0, 7.8)
    ax2.tick_params(axis='y', labelcolor='#1E3D59')

    ax2_twin.set_ylabel('Control / Inference Latency (ms)', color='#C85A17', fontsize=8.5, fontweight='bold')
    ax2_twin.set_ylim(0, 165)
    ax2_twin.tick_params(axis='y', labelcolor='#C85A17')

    # Add numeric labels cleanly above error bars
    for i, (bar, d, err) in enumerate(zip(b1, durations, dur_err)):
        y_top = d + err
        ax2.text(bar.get_x() + bar.get_width()/2.0, y_top + 0.3, f'{d:.2f}s', 
                 ha='center', va='bottom', fontsize=7.2, fontweight='bold', color='#1E3D59')

    for bar, lat in zip(b2, latencies):
        yval = bar.get_height()
        ax2_twin.text(bar.get_x() + bar.get_width()/2.0, yval + 3.0, f'{int(lat)}ms', 
                      ha='center', va='bottom', fontsize=7.2, fontweight='bold', color='#C85A17')

    ax2.set_xticks(x2)
    ax2.set_xticklabels(models, fontsize=7.8, fontweight='semibold')
    ax2.set_title('(b) Execution Duration vs Control-Loop Latency', fontsize=9.0, fontweight='bold')
    ax2.grid(True, linestyle='--', alpha=0.35, axis='y')

    plt.tight_layout()
    for d in OUT_DIRS:
        os.makedirs(d, exist_ok=True)
        out_file = os.path.join(d, "fig7_vla_benchmark.png")
        plt.savefig(out_file, dpi=300, bbox_inches='tight')
        print(f"Saved: {out_file}")
    plt.close()

if __name__ == '__main__':
    print("Generating updated Fig 1 and Fig 7...")
    generate_fig1_architecture()
    generate_fig7_vla_benchmark()
    print("Done!")
