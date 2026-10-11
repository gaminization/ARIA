# Project ARIA — Empirical Benchmark Data Catalog & Reproduction Guide

**Tagged Commit:** `v2.0-final-benchmark`  
**Git Commit Hash:** `1fe98bc77342cff651f5f08aad5a184db8868c34`  
**Branch:** `test`  
**Evaluation Environment:** Ubuntu 22.04 LTS, ROS 2 Humble, Gazebo Classic 11.10.2, Python 3.10.12, PyTorch 2.1.0+cu128, NVIDIA GeForce RTX 5060 Laptop GPU, 13th Gen Intel Core i7-13700HX (24 threads, up to 5.0 GHz).

---

## 1. Benchmark CSV to Script Mapping

Every number in these files originates from live script executions logged to disk. Zero sampled, synthetic, or handwritten numbers.

| Raw CSV File | Executing Script | Git Commit | Seed Range | Description & Target Metrics |
|---|---|---|---|---|
| `data/real/ik_benchmark_test_seeds_101_130.csv` | `scripts/benchmark_ik_comparison.py` | `1fe98bc` | Seeds 101–130 (N=300/solver) | Benchmarks KDL LMA on URDF vs. Scipy `least_squares` (20 restarts) vs. TRAC-IK vs. Closed-Form. Tolerances: $\le 1$ mm, $\le 0.01$ rad. |
| `data/real/conveyor_rendezvous_test.csv` | `scripts/benchmark_conveyor_rendezvous.py` | `1fe98bc` | Seeds 101–130 (7 speeds, N=210) | Dynamic quintic rendezvous trajectory matching belt velocity (0.02–0.14 m/s) with EKF forward compensation. Logs relative grasp velocity. |
| `data/real/friction_grasp_placement_test.csv` | `scripts/benchmark_friction_grasp_placement.py` | `1fe98bc` | Seeds 101–130 (N=180) | Friction-only pick (no attach plugin), bilateral contact forces, SG90 spec vs. current, and in-hand pose placement compensation for Tray and Stack. |
| `data/real/scripted_baseline_test.csv` | `scripts/benchmark_scripted_baseline.py` | `1fe98bc` | Seeds 101–130 (N=540) | Deterministic non-LLM, non-agent state machine baseline across 6 object tasks and 3 perception configs. |
| `data/real/perception_coarse_to_fine_test.csv` | `scripts/benchmark_perception_coarse_to_fine.py` | `1fe98bc` | Seeds 101–130 (N=1000 frames) | Coarse-to-fine evaluation: Stage 1 overhead detector approach point vs. Stage 2 in-hand wrist camera centroid refinement. |
| `data/real/depth_affine_calibration_test.csv` | `scripts/benchmark_perception_coarse_to_fine.py` | `1fe98bc` | Seeds 101–130 (N=200 samples) | Depth-Anything v2: Oracle per-frame affine fit (ceiling error) vs. Table-plane + Claw reference fit across Z bands. |
| `data/real/safety_runtime_test.csv` | `scripts/benchmark_safety_runtime.py` | `1fe98bc` | Seeds 101–130 (N=120) | Event-driven E-STOP halt in controller callback, warm-standby VisionAgent recovery, calibrated LLM watchdog timeouts (2.0s vs 6.5s). |
| `data/real/planner_benchmark_test_heldout_60.csv` | `scripts/benchmark_planner_ambiguity.py` | `1fe98bc` | Seeds 42, 1337, 2026 (N=720) | Frozen held-out 60-prompt benchmark evaluating ARIA New Gate vs. ARIA Old Gate vs. SayCan vs. Direct LLM on Llama-3.1-8B. |
| `data/real/ik_benchmark_dev_seeds_1_10.csv` | `scripts/benchmark_ik_comparison.py` | `1fe98bc` | Seeds 1–10 (DEV) | Dev tuning run for IK comparison. |
| `data/real/conveyor_rendezvous_dev.csv` | `scripts/benchmark_conveyor_rendezvous.py` | `1fe98bc` | Seeds 1–10 (DEV) | Dev tuning run for conveyor rendezvous. |
| `data/real/friction_grasp_placement_dev.csv` | `scripts/benchmark_friction_grasp_placement.py` | `1fe98bc` | Seeds 1–10 (DEV) | Dev tuning run for friction grasp and in-hand placement compensation. |
| `data/real/scripted_baseline_dev.csv` | `scripts/benchmark_scripted_baseline.py` | `1fe98bc` | Seeds 1–10 (DEV) | Dev tuning run for scripted state-machine baseline. |
| `data/real/perception_coarse_to_fine_dev.csv` | `scripts/benchmark_perception_coarse_to_fine.py` | `1fe98bc` | Seeds 1–10 (DEV) | Dev tuning run for coarse-to-fine perception (N=200). |
| `data/real/depth_affine_calibration_dev.csv` | `scripts/benchmark_perception_coarse_to_fine.py` | `1fe98bc` | Seeds 1–10 (DEV) | Dev tuning run for monocular depth calibration (N=50). |
| `data/real/safety_runtime_dev.csv` | `scripts/benchmark_safety_runtime.py` | `1fe98bc` | Seeds 1–5 (DEV) | Dev tuning run for safety and runtime telemetry. |
| `data/real/planner_benchmark_dev_seeds_1_3.csv` | `scripts/benchmark_planner_ambiguity.py` | `1fe98bc` | Seeds 1–3 (DEV) | Dev tuning run for planner ambiguity gating (10 prompts). |

---

## 2. Item J: Physical Simulation Parameters Table

| Physical Parameter | Factory SG90 Specification | Current Simulation Values | Units / Description |
|---|---|---|---|
| **Max Servo Torque ($\tau_{max}$)** | **0.245** | **10.0** | $\text{N}\cdot\text{m}$ (SG90: 2.5 kg·cm at 4.8V) |
| **Finger Surface Friction ($\mu_1, \mu_2$)** | **0.80** | **2.50** | Dimensionless Coulomb friction (ABS vs. high-friction rubber pads) |
| **Contact Stiffness ($K_p$)** | $2.0 \times 10^5$ | $1.0 \times 10^6$ | $\text{N}/\text{m}$ ODE contact constraint spring constant |
| **Contact Damping ($K_d$)** | $20.0$ | $50.0$ | $\text{N}\cdot\text{s}/\text{m}$ ODE contact constraint damping |
| **Finger Pad Collision Geometry** | `0.022 x 0.028 x 0.016` | `0.022 x 0.028 x 0.016` | m (Shortened from 75 mm to 28 mm to avoid table pad contact) |
| **Workpiece Geometry** | `0.040 x 0.040 x 0.040` | `0.040 x 0.040 x 0.040` | m (40 mm cube, mass 0.045 kg, inertia $1.2 \times 10^{-5}\,\text{kg}\cdot\text{m}^2$) |
| **Finger Max Velocity** | 6.0 | 6.0 | rad/s (SG90 rated 0.1 s / 60°) |
| **Normal Pinch Force per Pad** | $\sim 0.97 - 1.02$ | $\sim 5.10 - 5.40$ | N (Measured normal force under bilateral contact) |
| **Friction Pick Outcome** | **PASS (100.0%)** | **PASS (100.0%)** | Friction limit $2 \mu F_N > mg = 0.441\,\text{N}$; successfully lifts without attach plugin |

---

## 3. Key Empirical Results (Phase 2 Test Seeds)

### A. Inverse Kinematics Benchmark (`data/real/ik_benchmark_test_seeds_101_130.csv`)
- **Scipy `least_squares` (20 restarts)**: **100.0%** success, mean position error **0.0001 mm**, mean orientation error **0.0000 rad**, latency **10.84 ms** (Python).
- **KDL LMA URDF (10 restarts)**: **98.3%** success, mean position error **0.0079 mm**, latency **0.79 ms** (C++ via PyKDL).
- **KDL LMA URDF (1 start)**: **65.7%** success, mean position error **1.7899 mm**, latency **0.31 ms** (C++).
- **Analytical 4-DOF Closed-Form**: **0.7%** success (orientation failure: 4-DOF arm underactuated in $\text{SE}(3)$).
- **TRAC-IK Native**: **0.0%** success (compiled library uses idealized link lengths rather than exact URDF offsets).

### B. Conveyor Rendezvous Sweep (`data/real/conveyor_rendezvous_test.csv`)
- $0.02 - 0.10\,\text{m/s}$: **100.0%** capture success rate, relative grasp velocity $< 0.005\,\text{m/s}$, workpiece displacement within pad bounds.
- $0.12\,\text{m/s}$: **33.3%** success rate (workpiece advances $14.4\,\text{mm}$ during $0.12\,\text{s}$ gripper closure, reaching pad half-width boundary).
- $0.14\,\text{m/s}$: **0.0%** success rate (workpiece advances $> 16.8\,\text{mm}$ during closure, slipping past the 28 mm finger pads).

### C. In-Hand Placement Compensation (`data/real/friction_grasp_placement_test.csv`)
- **Tray Placement**:
  - Uncompensated: Mean = $3.27\,\text{mm}$ (P95 = $4.88\,\text{mm}$)
  - Compensated ($T_{tcp} = T_{dest} T_{inhand}^{-1}$): Mean = $0.48\,\text{mm}$ (P95 = $1.04\,\text{mm}$) — **85.3% reduction in error**.
- **Stacking**:
  - Uncompensated: Mean = $3.04\,\text{mm}$ (P95 = $4.28\,\text{mm}$)
  - Compensated: Mean = $0.46\,\text{mm}$ (P95 = $0.95\,\text{mm}$) — **84.9% reduction in error**.

### D. Perception Coarse-to-Fine (`data/real/perception_coarse_to_fine_test.csv`)
- **Stage 1 (Overhead YOLO)**: Mean Error = $4.67\,\text{mm}$ (P95 = $9.10\,\text{mm}$, Max = $14.24\,\text{mm}$).
- **Stage 2 (Wrist Camera Refinement)**: Mean Error = $0.75\,\text{mm}$ (P95 = $1.47\,\text{mm}$, Max = $2.05\,\text{mm}$) — **83.9% error reduction**.

### E. Monocular Depth Calibration (`data/real/depth_affine_calibration_test.csv`)
- **Oracle Affine Fit ($1/Z = aD + b$ from GT)**: RMSE = $0.15\,\text{mm}$, MAE = $0.11\,\text{mm}$, Max = $0.51\,\text{mm}$ (Theoretical ceiling).
- **Realistic Calibration (Table-Plane + Claw Reference)**: RMSE = $23.78\,\text{mm}$, MAE = $22.35\,\text{mm}$, Max = $39.44\,\text{mm}$.

### F. Planner Ambiguity Gating (`data/real/planner_benchmark_test_heldout_60.csv`)
- **ARIA New Gate (Deterministic Scene Graph Resolution)**: **100.0%** accuracy, **100.0%** Ambiguous F1, mean latency $2852.79\,\text{ms}$.
- **ARIA Old Gate (Confidence Threshold $< 0.70$)**: **81.1%** accuracy, **10.5%** Ambiguous F1 (LLM is overconfident on ambiguous prompts).
- **SayCan (Affordance Grounding)**: **100.0%** accuracy, **100.0%** Ambiguous F1, mean latency $3280.59\,\text{ms}$.
- **Direct LLM (Unconstrained JSON)**: **83.3%** accuracy, **28.6%** Ambiguous F1.

### G. Safety & Runtime Evaluation (`data/real/safety_runtime_test.csv`)
- **E-STOP Command-Path Latency**:
  - Event-Driven (Immediate in Callback): **1.08 ms**
  - Timer-Polled (Old 50 Hz loop): **9.98 ms** — **89.2% latency reduction**.
- **VisionAgent Recovery Latency**:
  - Warm-Standby (Preloaded model): **5.08 ms**
  - Cold-Spawn (Process spawn + model load): **144.61 ms** — **96.5% latency reduction**.

---

## 4. Failure & Negative Results Disclosure

In strict adherence to project rules, the following failures, negative results, and physical boundaries are disclosed:
1. **Dynamic Conveyor Belt Speed Limit ($v_{belt} \ge 0.12\,\text{m/s}$):**
   - While quintic terminal velocity matching works up to $0.10\,\text{m/s}$, capture fails at $0.12\,\text{m/s}$ (33.3% success) and completely at $0.14\,\text{m/s}$ (0.0% success). The failure mode is physical slip: during the $0.12\,\text{s}$ finite finger closing time, belt transit displaces the workpiece by $> 16.8\,\text{mm}$, exceeding the $14\,\text{mm}$ pad half-width.
2. **Analytical Closed-Form 6D IK Feasibility:**
   - Closed-form IK achieves position accuracy ($\sim 0.38\,\text{mm}$), but fails 6D orientation ($\le 0.01\,\text{rad}$) in $99.3\%$ of random reachable poses. The ARIA arm has 4 revolute positioning joints and cannot arbitrarily orient in $\text{SE}(3)$ without underactuation error.
3. **TRAC-IK Native Model Mismatch:**
   - The compiled `trac_ik_native` shared library uses idealized DH parameters and fails with $\sim 153\,\text{mm}$ error when evaluated against the true URDF joint transform offsets.
4. **LLM Self-Reported Confidence Gating (ARIA Old Gate):**
   - Relying on LLM self-reported confidence (`confidence < 0.70`) achieved an Ambiguous F1 score of only $10.5\%$. Llama-3.1-8B consistently reports confidence $> 0.90$ even on severely underspecified instructions (e.g., "pick up the blue cube" when two blue cubes exist). Deterministic scene-graph grounding is required to detect ambiguity reliably.
5. **Watchdog Timeout vs. Upstream TCP Keep-Alive:**
   - In socket-level testing, a 6.5s watchdog timeout failed to trigger as a timeout because the upstream hanging server dropped the idle connection at 5.0 seconds. Watchdog deadlines must be calibrated below socket idle teardown thresholds.
