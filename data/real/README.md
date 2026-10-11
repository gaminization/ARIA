# Project ARIA — Empirical Benchmark Data Catalog (`data/real/`)

All files in this directory contain **empirical measurements** obtained directly from physical simulation in Gazebo Classic, live ROS 2 Humble node execution, hardware/OS timing, or real neural model inference. Zero sampled, synthetic, or rule-assigned outcomes are permitted in this directory.

---

## 1. Catalog of Verified Empirical Data Files

| Data File | Producing Script / Source | Environment & Target Metrics | Measurement Source |
|---|---|---|---|
| `ik_benchmark_test_seeds_101_130.csv` | `scripts/benchmark_ik_comparison.py` | 300 test targets per solver (seeds 101–130). Evaluates KDL LMA on URDF, Scipy `least_squares` (20 restarts), and Closed-Form (position + pitch only). | C++ PyKDL, Scipy TRF optimizer, exact URDF forward kinematics. |
| `ik_benchmark_dev_seeds_1_10.csv` | `scripts/benchmark_ik_comparison.py` | 100 dev tuning targets per solver (seeds 1–10). | C++ PyKDL, Scipy TRF optimizer, exact URDF forward kinematics. |
| `planner_benchmark_test_heldout_60.csv` | `scripts/benchmark_planner_ambiguity.py` | 60 frozen held-out prompts across 3 seeds (N=720 evaluations). Evaluates ARIA New Gate, ARIA Old Gate, SayCan, Direct LLM on Llama-3.1-8B. | Real HTTP inference against Ollama `llama3.1:8b-instruct-q4_K_M`. |
| `planner_benchmark_dev_seeds_1_3.csv` | `scripts/benchmark_planner_ambiguity.py` | 10 dev prompts across 3 seeds. | Real HTTP inference against Ollama `llama3.1:8b-instruct-q4_K_M`. |
| `manipulation_trials_gazebo_20261010T173124Z.csv` | `scripts/benchmark_manipulation_suite_gazebo.py` | Gazebo manipulation suite trials across 6 object tasks and 3 perception configs. | Gazebo ODE physics engine, tf2_ros, `/gazebo/model_states`, and ground-truth checker (`aria_gazebo_manipulation_checker.py`). |
| `perception_comparison_20261010T173124Z.csv` | `scripts/benchmark_manipulation_suite_gazebo.py` | Real detector vs ground-truth comparison across tasks. | Gazebo rendered overhead camera images, YOLOv8 inference, TF lookup. |
| `collision_free_friction_trials.csv` | `scripts/benchmark_manipulation_suite_gazebo.py` | Friction-pick trials verifying collision-free trajectory execution. | Gazebo ODE contact states and `/gazebo/model_states`. |
| `reachability_grid_20261007T144445Z.csv` | `scripts/benchmark_reachability_grid_100k.py` | 100,000 Cartesian grid points evaluated for kinematic reachability. | URDF forward kinematics and analytical reachability verification. |
| `ik_confusion_matrix_20261007T144445Z.csv` | `scripts/benchmark_reachability_grid_100k.py` | Confusion matrix of reachability classifier vs analytical solver. | Direct solver evaluations. |
| `friction_grasp_real_30seeds.csv` | `scripts/benchmark_real_friction_grasp.py` | 30 seeds with 28 mm pads on 40 mm cube. Friction-only mode (no attach). Lift measured from `/gazebo/model_states`, pad forces from `/gripper/contact_left` and `/gripper/contact_right`. | Gazebo ODE contacts, `/gazebo/model_states`, 50 Hz telemetry, rosbag. |
| `conveyor_sweep_real.csv` | `scripts/benchmark_real_conveyor.py` | Multi-speed conveyor sweep (0.02–0.14 m/s, 20 cycles each = 140 cycles) using real belt plugin. Relative TCP-belt velocity measured from TF at contact, release pose from `model_states`. | Gazebo belt plugin, tf2_ros, `/gazebo/model_states`, 50 Hz telemetry, rosbag. |
| `scripted_baseline_real.csv` | `scripts/benchmark_real_scripted_baseline.py` | Deterministic state machine baseline in Gazebo across 6 standardized tasks evaluated with identical `GazeboManipulationSuccessChecker`. | Gazebo ODE physics, tf2_ros, `/gazebo/model_states`, 50 Hz telemetry, rosbag. |
| `perception_coarse_to_fine_real.csv` | `scripts/benchmark_real_perception_coarse_to_fine.py` | Coarse-to-fine perception: captures and saves overhead and wrist frames to disk, runs real YOLOv8 detector, logs boxes, confidences, and errors vs `model_states`. | Gazebo camera renderings, saved PNG frames, real YOLOv8 inference, `model_states`. |
| `depth_anything_real.csv` | `scripts/benchmark_real_depth_anything.py` | Depth-Anything v2 GPU inference on real wrist-camera frames, metric claw calibration vs Gazebo z-buffer ground truth depth maps. Logged frame IDs, RMSE, MAE. | Gazebo wrist camera RGB + z-buffer depth, Depth-Anything v2 Small on CUDA, 50 Hz telemetry, rosbag. |
| `safety_runtime_real.csv` | `scripts/benchmark_real_safety_runtime.py` | Real worker SIGKILL termination measuring lifecycle transition phases from ROS timestamps; real interrupt-driven E-STOP preemption through controller. | Real OS process signals, high-res ROS timestamps, controller callbacks, 50 Hz telemetry, rosbag. |
| `lang_prompts_heldout_60.json` | Dataset specification | Frozen 60-prompt held-out evaluation set for natural language task planning. | Benchmark ground truth. |
| `lang_prompts.json` | Dataset specification | Full 120-prompt corpus covering clear, ambiguous, and boundary cases. | Benchmark ground truth. |

---

## 2. Quarantined Files Removed from `data/real/`

During the project verification audit, 12 CSV files (and their 5 generating scripts) were **quarantined and moved to `data/legacy_synthetic/`** under the classification **"modelled, not measured"**.

### Quarantined CSV Files and Rationale:
1. `conveyor_rendezvous_dev.csv` & `conveyor_rendezvous_test.csv`:
   - **Reason for Removal:** Relative contact velocities, belt positions, and capture successes were computed using kinematic formulas rather than empirical measurements from the active Gazebo belt plugin and `tf2_ros` transform lookups at the moment of contact.
2. `friction_grasp_placement_dev.csv` & `friction_grasp_placement_test.csv`:
   - **Reason for Removal:** Bilateral contact forces and post-grasp placement offsets were sampled from pseudo-random distributions (`rng.uniform`, `rng.normal`) rather than measured from Gazebo bumper contact topics (`/gripper/contact_left`, `/gripper/contact_right`) and ground-truth model states (`/gazebo/model_states`).
3. `scripted_baseline_dev.csv` & `scripted_baseline_test.csv`:
   - **Reason for Removal:** Baseline execution outcomes were rule-simulated by state transitions rather than executed as end-to-end trajectories in the Gazebo workcell evaluated by `aria_gazebo_manipulation_checker.py`.
4. `perception_coarse_to_fine_dev.csv` & `perception_coarse_to_fine_test.csv`:
   - **Reason for Removal:** Detection bounding boxes, confidences, and errors relative to model states were mathematically modelled rather than produced by saving raw camera frames and running real detector inference against `/gazebo/model_states`.
5. `depth_affine_calibration_dev.csv` & `depth_affine_calibration_test.csv`:
   - **Reason for Removal:** Evaluated synthetic affine depth models rather than running Depth-Anything v2 inference on real wrist-camera frames against ground-truth Gazebo metric depth maps.
6. `safety_runtime_dev.csv` & `safety_runtime_test.csv`:
   - **Reason for Removal:** E-STOP latency and VisionAgent crash recovery times were sampled from parameterized Gaussian distributions rather than terminating real OS processes (`SIGKILL`) and logging event-driven ROS 2 controller preemption timestamps.

---

## 3. Strict Operating Rules

1. Any value in a results CSV (`success`, `error`, `latency`, `force`, `offset`) must originate from a measured system: Gazebo ODE physics state, ROS timestamps, or a real neural model's inference output.
2. Seeded randomness is permitted **only** to select initial poses, spawn times, or prompt order inside a single function named `sample_initial_conditions()`.
3. Never sample, assign by formula, or hard-code an outcome column.
4. Never use `time.sleep` to represent a process.
5. All new measured benchmarks must log:
   - Wall-clock ISO timestamp (nanoseconds)
   - Trial wall time
   - rosbag recording path
   - 50 Hz `/gazebo/model_states` + TF log file path
   - Success verified strictly via `aria_gazebo_manipulation_checker.py`.
