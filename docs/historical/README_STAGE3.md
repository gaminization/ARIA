# ARIA Stage 3 — Agent Intelligence System

## Overview

Stage 3 adds the full autonomous intelligence system to ARIA:
- **15 LifecycleNode agents** communicating via unified state bus
- **Persistent world model** (SQLite) with spatial relations
- **NLP task planning** with chain of thought
- **In-hand manipulation** (rotate, reposition, flip, slide-to-tip)
- **VLA model interface** (LeRobot ACT, OpenVLA, Pi0, Gr00t N1)
- **VLA benchmark** (10 standardized tasks)
- **Real-time dashboard** (FastAPI + React)
- **Failure classification & recovery** (11 types, 3 retries)
- **Skill versioning** with automatic rollback

## Architecture

```
User NL Command
     │
     ▼
 TaskManager (state machine)
     │
     ▼
 PlanningAgent (NL → subgoals → action queue)
     │
     ├── VisionAgent → DepthAgent → TrackingAgent
     ├── AffordanceAgent → ReachabilityAgent
     ├── SkillAgent → ControlAgent
     ├── SafetyAgent (always-on, <5ms e-stop)
     ├── AttentionAgent (ROI focus)
     └── DialogueAgent (NL status/approval)
     │
     ├── WorldModelAgent → MemoryAgent (SQLite)
     ├── LearningAgent (HDF5 recording)
     └── EvaluationAgent (metrics CSV)
```

All agents communicate through the **State Bus** (`/aria/state/*`).
No agent directly calls another agent.

## Quick Start

```bash
# 1. Install Stage 3 requirements
pip install -r requirements_stage3.txt

# 2. Build all packages
cd ~/Projects/ARIA
colcon build --symlink-install

# 3. Launch full system
source install/setup.bash
ros2 launch arm_bringup aria_full.launch.py

# 4. Open dashboard
# → http://localhost:8080

# 5. Validate
python3 arm_bringup/scripts/validate_stage3.py
```

## Dashboard

```
┌─────────────────────────────────────────────────────────┐
│                  ARIA CONTROL CENTER                     │
├──────────┬──────────────────┬────────────────────────────┤
│ TOP CAM  │ 3D ARM VIZ       │ TASK CONTROL               │
│ WRIST CAM│ Joint bars       │ NL Command input           │
│ YOLO     │ Workspace bounds │ Confidence ██░░ 92%        │
│ DEPTH    │                  │ [APPROVE] [REJECT] [ESTOP] │
├──────────┴──────────────────┴────────────────────────────┤
│ CHAIN OF THOUGHT (live, scrolling)                       │
├──────────────────────┬───────────────────────────────────┤
│ WORLD MAP (top-down) │ HEALTH: J1🟢 J2🟢 J3🟡 J4🟢 J5🟢  │
│ Object positions     │ FPS: 28.4 | Latency: 42ms         │
└──────────────────────┴───────────────────────────────────┘
```

## VLA Benchmark

```bash
# Run VLA benchmark
python3 -m arm_vla.vla_benchmark

# Output:
# ═════════════════════════════════════════
#  VLA BENCHMARK RESULTS
# ═════════════════════════════════════════
#   RuleBasedBaseline: 89% success | 2.4s avg
#   LeRobot-ACT: 72% success | 3.1s avg
#   OpenVLA: 68% success | 4.2s avg
# ═════════════════════════════════════════
```

## Packages

| Package | Type | Contents |
|---------|------|----------|
| `arm_planner` | ament_cmake | 10 msgs, state bus, task manager, world model DB, failure/recovery, search, health monitor, cause-effect model, memory/skill managers |
| `arm_agents` | ament_python | 15 agents + in-hand manipulation + moving target handler |
| `arm_vla` | ament_python | VLA interface (4 backends) + benchmark |
| `arm_dashboard` | standalone | FastAPI backend + React frontend |
| `arm_interfaces` | ament_cmake | 3 new services (SendCommand, GetAffordanceGrasp, CheckReachability) |

## Validation

```
═══════════════════════════════════════════
  ARIA Stage 3 Validation
═══════════════════════════════════════════
  ✅ 1. Agent Nodes Active: All 15 agents active
  ✅ 2. NL Planning Pipeline: All commands parsed correctly
  ✅ 3. State Bus Topics: All 5 state topics active
  ✅ 4. World Model DB: CRUD, lifecycle, relations, YAML I/O OK
  ✅ 5. Skill Manager: register, execute, stats, rollback OK
  ✅ 6. Failure Classifier: 11 types, pattern matching OK
  ✅ 7. VLA Benchmark: 10 tasks, baseline functional
  ✅ 8. Dashboard API: responding, state endpoint OK
═══════════════════════════════════════════
  RESULTS: 8/8 passed (100%)
  🟢 STAGE 3 COMPLETE — All systems operational.
```
