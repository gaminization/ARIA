# ARIA Scientific Integrity Audit

**Audit date:** 2026-10-24  
**Baseline commit:** `0d1fa77` (branch `journal-study` created from it)  
**Auditor:** automated code/data audit run inside the project worktree

This file lists every previously reported shortcoming that could be verified against the
repository, the evidence, and its disposition. A shortcoming is only marked **FIXED** when a
new raw log produced by executed code is referenced. Nothing in this file is inferred from a
paper claim.

Legend: **OPEN** = not yet repaired; **FIXED** = repaired and evidenced by a raw log;
**QUARANTINED** = data/script removed from the evidence base; **NOT FEASIBLE** = cannot be
executed in the available environment (reason stated).

---

## 0. Environment facts that bound the study

| Fact | Evidence |
|---|---|
| ROS 2 Humble and Gazebo Classic 11.10.2 are installed. | `ros2 --version`, `gazebo --version` |
| The workspace builds with CMake 4.1.3 only when `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` is passed (jsoncpp/ignition policy). | `colcon build` failure without the flag; success with it |
| Gazebo runs headless, spawns the arm, and `joint_trajectory_controller` tracks commands. | `/tmp/simlaunch.log`; measured joint positions after a commanded trajectory |
| No CUDA device is available to PyTorch. | `torch.cuda.is_available() == False`; no `/dev/nvidia*`; no `nvidia-smi` |
| A discrete NVIDIA VGA device is present but has no usable driver. | `lspci` shows `01:00.0 NVIDIA Device 2d59`; CUDA cannot initialise |
| Ollama is running and serves `llama3.1:8b-instruct-q4_K_M` and `qwen3.5:9b` on CPU. | `curl localhost:11434/api/tags` |
| No robot hardware is connected. | no `/dev/ttyUSB*` or `/dev/ttyACM*` |
| No independent human annotators are available in this environment. | operational constraint |

**Consequence:** LeRobot/ACT/SmolVLA training (Phase 3), human inter-annotator agreement
(Phase 4), and hardware trials (Phase 7) cannot be executed here. Claims for those phases
must be omitted, not simulated.

---

## 1. Conveyor benchmark

### 1.1 TF lookup silently returns a zero pose
**Status: OPEN**  
`scripts/benchmark_real_conveyor.py:174-186` catches every TF exception and returns
`np.zeros(3), time.time()`. The two velocity samples are taken with `time.sleep()` and no
executor spin, so both lookups return the same cached transform.
**Evidence:** `data/real/conveyor_sweep_real.csv` has `measured_tcp_vel_m_s == 0.0000` in
all 140 rows and `relative_contact_vel_m_s == commanded_belt_vel_m_s` in all rows.

### 1.2 Belt speed scale is wrong
**Status: OPEN**  
`scripts/benchmark_real_conveyor.py:136-138` assumes `max_velocity = 0.50 m/s`; the model
declares `0.25 m/s` (`arm_bringup/worlds/aria_industrial_workcell.world:604`,
`aria_industrial_workcell.sdf:596`). Commanded power is therefore twice the intended
fraction of maximum.

### 1.3 Twist and service race
**Status: OPEN**  
The benchmark calls `/aria/conveyor/set_power` and also publishes
`/aria/conveyor/cmd_vel` (`benchmark_real_conveyor.py:136-147`). The plugin lets the
last-arriving Twist overwrite the service value with no arbitration
(`arm_control/src/conveyor_belt_plugin.cpp:262-267`).

### 1.4 Conveyor plant is kinematic, not frictional
**Status: OPEN (documented limitation)**  
`conveyor_belt_plugin.cpp:204-218` sets workpiece linear velocity directly and zeroes
angular velocity, so workpieces cannot tip, rotate, or jam. The "conveyor" is a scripted
treadmill. Any conveyor result must be labelled accordingly.

### 1.5 Release geometry does not match the tray
**Status: OPEN**  
Released object `y` is 0.070-0.269 m while tray bounds are `y in [-0.22, -0.12]`
(`benchmark_real_conveyor.py` tray constants). `conveyor_sweep_real.csv`: 96
`D_TOLERANCE_EXCEEDED`, 44 `B_GRASP_SLIP`, 0 successes.

---

## 2. Reachability

### 2.1 Provenance is impossible
**Status: OPEN**  
The 2026-10-06 and 2026-10-07 reachability CSVs stamp commit `8d83b97`, but
`scripts/benchmark_reachability_grid_100k.py` first appears in `cc935f2` (2026-10-08).
The runs cannot have been produced by the script they cite.

### 2.2 Solver changed between the two result sets
**Status: OPEN**  
`cc935f2` rewrote `arm_ik/arm_ik/ik_solvers/aria_analytical_ik.py` by 752 lines and
`9a8ace8` changed it further. The 14.6% and 99% runs use different algorithms.

### 2.3 Identical files under different dates
**Status: OPEN**  
`md5sum data/real/reachability_grid_20261006T080221Z.csv
data/real/reachability_grid_20261007T144445Z.csv` returns the same hash
`9e77bb771ac84a324f5be05af84a50cd`.

### 2.4 Orientation error is hard-coded
**Status: OPEN**  
`arm_ik/arm_ik/ik_solvers/aria_analytical_ik.py:393` sets
`orientation_error_rad=0.0` by convention. The solver is a numerical Newton-Raphson
routine (`solver_name="urdf_ik_v2_newton"`), not the "analytical" solver claimed.

---

## 3. E-STOP

### 3.1 The 100-trial v2 benchmark is a loopback
**Status: OPEN**  
`scripts/benchmark_fault_injection_real_v2.py:782-969` creates a local node that
publishes and subscribes to its own `/aria/estop`. It never invokes `SafetyAgent`,
`ControlAgent`, or a moving arm.

### 3.2 Physical stop times are fabricated
**Status: OPEN**  
`benchmark_fault_injection_real_v2.py:924-926`:
`physical_stop_ms = cmd_latency_ms + 18.5` when no physical stop is observed. The rolling
condition never fires in the loopback, so 100/100 rows use the fallback. The reported
28.386 ms mean is a constant offset, not a measurement.

### 3.3 Release service name mismatch
**Status: OPEN**  
`scripts/benchmark_real_safety_runtime.py:94` calls `/aria/release_estop`; SafetyAgent
serves `/aria/estop/release` (`arm_agents/arm_agents/safety_agent.py:84`).

### 3.4 Arrest timer excludes dispatch latency
**Status: OPEN**  
`benchmark_real_safety_runtime.py:339-355` starts timing after the service returns.

### 3.5 Trajectory path is not gated by E-STOP
**Status: OPEN**  
`arm_agents/arm_agents/control_agent.py:126` checks `estop_active` on the service path,
but a trajectory already published to
`/joint_trajectory_controller/joint_trajectory` continues.

### 3.6 The two reported figures are different metrics
**Status: OPEN**  
28.4 ms is a loopback command latency plus a constant. 211 ms is a hold-trajectory
arrest time measured after the service call. They are not the same metric.

---

## 4. LLM watchdog

### 4.1 No 2.0 s LLM watchdog exists
**Status: OPEN**  
`arm_agents/arm_agents/llm_dialogue_agent.py:130` creates a 2.0 s dialogue health timer,
not an inference deadline. No `asyncio.wait_for`/socket deadline wraps the planner call.

### 4.2 Reported percentiles are literals
**Status: OPEN**  
`benchmark_fault_injection_real_v2.py:567-574` writes
`measured_llm_p50_ms = 4627.23`, `p95 = 5800.81`, `p99 = 6206.43` into every row.

### 4.3 Client timeout is 90 s, not 2 s
**Status: OPEN**  
`arm_planner/config/llm_config.yaml:26-27` sets `timeout_s: 90.0`, `max_retries: 2`;
worst case 270 s per call.

---

## 5. Perception logs

### 5.1 All primary trials used the attach assist
**Status: OPEN**  
`data/real/manipulation_trials_gazebo_20261010T173124Z.csv` has
`grasp_mode=assisted`, `attach_assisted=True`, `physical_contact_only_held=False` in
180/180 rows.

### 5.2 Config B leaks ground-truth depth
**Status: OPEN**  
`scripts/benchmark_manipulation_suite_gazebo.py:266` uses `target_z = true_pos[2]`.

### 5.3 Config C is clamped to B
**Status: OPEN**  
`benchmark_manipulation_suite_gazebo.py:301-305` clamps the depth ratio to +/-8%, so C
cannot differ from B by more than 8% in depth.

### 5.4 Config A reports the commanded spawn, not the measured pose
**Status: OPEN**  
`benchmark_manipulation_suite_gazebo.py:533-551` returns `True` after an unchecked
`gz model` shell call.

### 5.5 Two error logs disagree
**Status: OPEN**  
Manipulation CSV medians: B 74.25 mm, C 85.525 mm. Perception CSV medians: B 79.045 mm,
C 90.27 mm. Both claim to describe the same trials; no shared frame ID/timestamp exists.

### 5.6 The deployed detector is mostly edge saliency
**Status: OPEN**  
`data/real/perception_comparison_20261010T173124Z.csv`: 58/60
`visual_edge_saliency_detector`, 2/60 `yolov8_overhead_detector`. `arm_vision/models/`
contains only `workspace_splat.npz`; no checkpoint is committed.

### 5.7 Detector training labels are random
**Status: OPEN**  
`scripts/train_coarse_to_fine_detectors.py:217-221` samples bounding boxes from
`rng.uniform(...)`, unrelated to the image.

### 5.8 Overhead and wrist "models" are byte-identical
**Status: OPEN**  
`train_coarse_to_fine_detectors.py:288-293` copies the same `best.pt` to both paths.

---

## 6. Scripted baseline

### 6.1 Two tasks pass by construction
**Status: OPEN**  
`scripts/benchmark_real_scripted_baseline.py:366-371` passes literal
`initial_yaw_rad=0.0, final_yaw_rad=0.28` (constant 4.011 deg), and `:385-389` passes
literal `initial_pos=[0.200,0.000], final_pos=[0.235,0.000]` (constant 35.0 mm).
Tasks 07 and 08 therefore score 30/30 regardless of the robot.

### 6.2 The honest subset is 4/120
**Status: OPEN**  
`data/real/scripted_baseline_real.csv`: task_01 4/30, task_02 0/30, task_04 0/30,
task_07 30/30, task_08 30/30, task_09 0/30. Removing the two hard-coded tasks leaves
4/120.

### 6.3 Duplicate file
**Status: OPEN**  
`scripted_baseline_real.csv` and `scripted_baseline_real_180trials.csv` have the same
md5 `4e3622721082c8010b89da5795243d86`.

---

## 7. Grasp assist

### 7.1 Attach is proximity-only and unbreakable
**Status: OPEN**  
`arm_control/src/gripper_plugin.cpp:160-222` attaches the nearest dynamic link within
0.14 m of `wrist_link` with a fixed joint, without checking finger closure or contact.
Detach is unconditional (`:244-250`).

### 7.2 Attach is the default in the skill agent
**Status: OPEN**  
`arm_agents/arm_agents/skill_agent.py:1107` calls the attach service after closing the
gripper. The 180-trial suite uses it in every row.

---

## 8. Paper claims versus data

| Paper claim | Location | Evidence in repo | Disposition |
|---|---|---|---|
| 89.0% (89/100) primary success | `main.tex:92,825,843,852` | `data/expanded_stress_suite_100.csv` does not exist; the 100-row files show 53% and 83% | **UNSUPPORTED** |
| 100% nominal (30/30), 0.55 mm pick error | `main.tex:844,852,1015` | cited CSV is 100 rows and 53% | **CONTRADICTED** |
| 93.3% conveyor (28/30) | `main.tex:929,935` | cited CSV is quarantined synthetic; real sweep is 0% sort | **UNSUPPORTED** |
| Depth RMSE 6.84/8.2/17.4 mm | `main.tex:92,132,142,591,1013,1167` | real depth CSV 25-96 mm; the 8.2/6.5/17.4 triple is a hard-coded fallback at `scripts/benchmark_real_depth_anything.py:327-330` | **CONTRADICTED** |
| ECE 5.65% over 500 detections | `main.tex:642,647` | `confidence_calibration_summary.csv` has constant `ece_perc_pct=1.0` and 120 samples in one bin | **UNSUPPORTED** |
| 15 lifecycle nodes | `main.tex:92` | launch file has 15 agent nodes plus managers | **SUPPORTED** |
| Language 68.9% (N=180) | `main.tex:887-899` | `language_grounding_multiseed_trials.csv` 540 rows | **SUPPORTED, but generated by an LLM benchmark whose prompts are not split** |
| Color defect 99.2% | `main.tex:992,998` | `color_defect_summary.csv` | **SUPPORTED** |
| IK 100% convergence, 0.075 ms | `main.tex:794` | `ik_benchmark_5d_summary.csv` | **SUPPORTED on a circular reachable set; not a reachability estimate** |

### 8.1 The "verifier" does not verify
**Status: OPEN**  
`scripts/verify_paper_claims_against_csv.py` checks substring presence in the LaTeX for
about ten claims; it does not recompute the headline numbers.

---

## 9. Data provenance

### 9.1 Hard-coded workspace path
**Status: OPEN**  
Most `scripts/benchmark_*_real.py` files set
`WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"`. That directory is not this worktree.
The scripts therefore read/write data outside the repository under audit.

### 9.2 No clean-tree enforcement
**Status: OPEN**  
No benchmark refuses to run on a dirty tree, and no row records a `dirty` flag.

### 9.3 Synthetic data is quarantined but still referenced
**Status: QUARANTINED (data), still referenced by paper**  
`data/legacy_synthetic/README.md` correctly labels the synthetic generators. The paper
still cites files from that directory.

---

## 10. Disposition summary

| Area | Status |
|---|---|
| Conveyor TF / speed scale / arbitration | OPEN |
| Reachability provenance / circularity | OPEN |
| E-STOP metric definition | OPEN |
| LLM watchdog | OPEN |
| Perception log reconciliation | OPEN |
| Detector dataset / checkpoint | OPEN |
| Scripted baseline | OPEN |
| Grasp assist removal | OPEN |
| Paper number provenance | OPEN |
| Phase 3 learned policies | NOT FEASIBLE (no CUDA) |
| Phase 4 human kappa | NOT FEASIBLE (no annotators) |
| Phase 7 hardware | NOT FEASIBLE (no hardware) |

This table is updated only when a raw log is produced by executed code.

---

## 11. Addendum: measured dispositions (2026-10-25)

This addendum is written only after raw logs existed. Each row names the log that
supports the disposition. Rows not listed here remain as stated above.

| Item | New status | Evidence |
|---|---|---|
| 1.1 TF zero pose | **FIXED (measurement path)** | `P0-CONV` reads `/gazebo/link_states`, not TF; 120/120 rows have a finite part velocity. |
| 1.2 Belt speed scale | **FIXED (measurement path)** | `P0-CONV` publishes `Twist.linear.x` directly; no 0.50 m/s scale is used. |
| 1.3 Twist/service race | **FIXED (measurement path)** | `aria_study.ros.send_conveyor_speed` publishes only the Twist. |
| 1.4 Kinematic plant | **CONFIRMED, documented** | Realized part speed is `commanded - 0.025 m/s` (median); visual belt link speed is `~6e-8 m/s`; rollers turn at `~4.6e4 rad/s`. The belt animation and the transport are independent. |
| 1.5 Release geometry | **OPEN** | Grasp experiments are blocked by the gripper defect (Section 12). |
| 2.1 Impossible provenance | **FIXED** | `P0-REACH` was re-run from clean commit `83909cb`; 100,000 rows. |
| 2.2 Solver changed | **QUANTIFIED** | Current solver recall 99.241% vs historical 0.019% on identical targets; McNemar b=99,222, c=0 (p underflows). |
| 2.3 Identical files | **QUARANTINED** | Historical CSVs are not used in the paper. |
| 2.4 Orientation hard-coded | **CONFIRMED** | `solve_analytical` returns `orientation_error_rad=0.0`; `P2-N4` therefore verifies position and joint limits only. |
| 3.1 E-STOP loopback | **FIXED** | `P0-ESTOP` calls the real `/aria/estop` service while a 2.5 s trajectory is executing; 100 trials. |
| 3.2 Fabricated stop times | **FIXED** | `P0-ESTOP` measures `t_stop` from `/joint_states` positions. |
| 3.3 Release service mismatch | **FIXED (new harness)** | `P0-ESTOP` uses `/aria/estop/release`. Old script remains quarantined. |
| 3.4 Timer excludes dispatch | **FIXED** | `t_zero_cmd` is measured from the signal to the controller-topic message. |
| 3.5 Trajectory not gated | **CONFIRMED** | Safety path: 0/50 halts; median post-signal overshoot 0.72 rad. |
| 3.6 Different metrics | **FIXED** | One operational definition is used for both paths. |
| 4.1-4.3 LLM watchdog | **NOT EXECUTED** | No 2.0 s watchdog exists in the code; P0-WD is reported as N/A, not simulated. |
| 5.1-5.8 Perception logs | **NOT EXECUTED / QUARANTINED** | No reconciled frame key exists; no detector checkpoint is committed. |
| 6.1-6.3 Scripted baseline | **NOT EXECUTED** | Requires the repaired gripper; hard-coded tasks 07/08 are excluded from any future analysis. |
| 7.1-7.2 Attach assist | **DISABLED** | Launch defaults now set `ARIA_ENABLE_ATTACH=false`; no grasp result is claimed. |
| 8.1 Paper verifier | **REPLACED** | `scripts/build_numbers.py` regenerates every macro from raw logs; the paper no longer contains typed outcome numbers. |
| 9.1 Hard-coded workspace | **FIXED (new scripts)** | `aria_study` resolves `ROOT` from `__file__`; legacy scripts remain quarantined. |
| 9.2 No clean-tree enforcement | **FIXED** | `provenance.assert_clean()` aborts on a dirty tree; every row carries `commit` and `dirty`. |
| 9.3 Synthetic data | **QUARANTINED** | No synthetic CSV is read by `build_numbers.py`. |

### 11.1 New defects found during the repair

| Defect | Evidence | Consequence |
|---|---|---|
| `/joint_states.velocity` is not the derivative of `/joint_states.position` | While the waist moved from 0.000 to 0.170 rad in 0.95 s, the reported waist velocity stayed near `-1.3 rad/s`; the shoulder reported `+1.8` to `+1.4 rad/s` for a `~0.12 rad/s` motion. | Any stop-time metric that trusts the reported velocity is invalid. `P0-ESTOP` differentiates the measured positions. |
| Three `manual_control_node` instances were publishing on `/joint_trajectory_controller/joint_trajectory` | `ros2 topic info -v` listed four publishers; `pgrep` found three manual nodes. They emitted 5-joint, velocity-free hold trajectories in response to E-STOP. | E-STOP attribution was confounded. The duplicate nodes were killed before the evaluation run; the run records the number of non-path messages per trial. |
| `SafetyAgent` publishes a 4-joint halt with no velocities | `arm_agents/arm_agents/safety_agent.py:113-121`; JTC has `allow_partial_joints_goal: False`. | The controller rejects the halt. The arm continues to its goal (median overshoot 0.72 rad). |
| Conveyor realized speed has a fixed offset, not a scale error | `P0-CONV`: commanded 0.04/0.08/0.12 m/s gives 0.0151/0.0543/0.0937 m/s; offset `-0.0249/-0.0257/-0.0263 m/s`. | A single scale factor cannot repair the plant; the friction/velocity-write model must change. |
| `solve_analytical` reports success for residuals up to 10 mm | `_gradient_fallback` accepts `best_err < tolerance_m*10`. | A planner that trusts `success` can execute a pose error up to 10 mm. `P2-N4` quantifies the consequence. |

### 11.2 Second repair pass: provenance, thresholds, and conditional metrics

The first addendum was written before three issues were found. They are recorded here
because each changes what the paper is allowed to claim.

| Issue | Evidence | Disposition |
|---|---|---|
| P0-REACH rows had no per-row provenance | The first CSV predated the `TrialLog` provenance columns; only the summary carried the commit. | Re-ran from clean commit `7e9d762` with the same seed, target grid, tolerance, and solvers. Recall and error values are identical; solve-time medians changed from 0.863/0.288 ms to 0.705/0.230 ms. The legacy logs are in `data/journal/raw/archive/`. |
| P0-ESTOP used a 0.05 rad/s stop threshold while the pre-registration fixes 0.01 rad/s | `PREREGISTRATION.md` H0e; the earlier `scripts/study/p0_estop.py` set `VEL_THRESH=0.05`. | The final run logs `0.01 rad/s` as the primary threshold and `0.05 rad/s` as a sensitivity check. Both are in the CSV. |
| The P0-ESTOP summary commit differed from the row commit | A documentation-only commit landed mid-run: rows `99966bd`, summary `d978641`. | `build_numbers.py` now reads the single commit stamped on the rows and warns when the summary differs. The old run is archived. |
| N4's conditional execution-failure rate was reported as a failure | N4 executes 47/100 targets with 0 execution failures; the baseline executes 100/100 with 48 failures. Intention-to-treat success is 47% vs 52%. | The paper reports both: the gate lowers conditional execution failures but rejects 53% of targets, rescues 1 baseline failure, and loses 6 baseline successes. No practical improvement is claimed. |
| P2-N4 target box and interval method changed | Box changed on development seeds before evaluation; the paired design uses a bootstrap interval rather than the pre-registered unpaired Newcombe interval. | Recorded in `DEVIATIONS.md` D2 and D3; the evaluation seeds were not used for tuning. |

### 11.3 Final evaluation values after the second repair pass

| Experiment | Commit | Final values |
|---|---|---|
| P0-REACH | `7e9d762` | 100,000 targets, per-row provenance. Current 99.241% recall at 5 mm (Wilson 0.99185–0.99293), median error 0.365 mm, p95 0.476 mm, median solve 0.705 ms. Historical 0.019% (Wilson 0.000122–0.000297), median error 240.934 mm, p95 360.489 mm, median solve 0.230 ms. McNemar b=99,222, c=0, p underflows. Values are identical to the pre-provenance run; only the solve-time medians moved (0.863/0.288 ms to 0.705/0.230 ms). |
| P0-ESTOP | `0bb4ed0` | 100 trials, 50 seeds × 2 paths. At the pre-registered 0.01 rad/s threshold: command 50/50 halts, median t_stop 120 ms (p95 120 ms), median t_zero 0.704 ms, median overshoot 0.0107 rad, 5 joints with velocities. Safety 0/50 halts, median t_zero 0.915 ms, median overshoot 0.716 rad, 4 joints without velocities. McNemar b=50, c=0, p=1.78e-15. At the 0.05 rad/s sensitivity threshold: command 50/50, median t_stop 110 ms; safety 0/50. |
| P2-N4 | `1ad64b3` | 100 targets × 2 pipelines. Baseline 52/100 ITT, 48/100 execution failures; N4 47/100 ITT, 47/100 executed, 0/47 execution failures. Paired transitions: 1 N4-only success, 6 baseline-only, 46 both, 47 neither. RD −5.0 pp (paired bootstrap CI −10.0 to 0.0), McNemar p=0.125. |
