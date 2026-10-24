# ARIA Journal Study — Pre-Registration

**Status:** FROZEN before evaluation. Commit this file before running any evaluation seed.
**Created:** 2026-10-24
**Baseline code commit:** `0da616d` (branch `journal-study`)
**Rule:** Thresholds, seeds, and trial counts below may not be changed after seeing evaluation
data. Any change must be recorded in `DEVIATIONS.md` with the reason and the date, and the
affected experiment must be re-run from scratch under a new experiment ID.

---

## 1. Purpose and claim boundary

This study tests whether the ARIA modular architecture, its repaired benchmarks, its friction
grasp, and four proposed methods (N1-N4) deliver measurable improvements over defined
baselines in Gazebo Classic 11 / ODE. It does **not** claim physical-robot validity, and it
does **not** claim end-to-end VLA capability.

The paper's claims will be generated only from `ledger.csv` by
`scripts/build_numbers.py`. No number may be typed into `main.tex` directly.

### 1.1 Environment-bounded omissions (declared before data collection)

| Requested item | Status | Reason |
|---|---|---|
| Phase 3: LeRobot ACT/SmolVLA/OpenVLA training | **Not executable** | No CUDA device (`torch.cuda.is_available()==False`); CPU training of these policies is not a valid comparison within the available budget |
| Phase 4: two independent human annotators and Cohen's kappa | **Not executable** | No independent annotators available |
| Phase 4: held-out prompts authored by non-authors | **Partially executable** | Existing 120 prompts are reused; provenance is reported as author-generated. A second local model is used as an independent *model* annotator, explicitly not a human |
| Phase 7: hardware | **Not executable** | No robot, no ESP32, no serial device |
| Phase 0f: 5,000-frame detector | **Reduced, declared** | 1,000 auto-labelled Gazebo frames, YOLOv8n at 320 px on CPU; shortfall reported |

These omissions are not filled with modelled numbers. Any table cell that depends on them is
marked `N/A` with the reason.

---

## 2. Hypotheses

| ID | Hypothesis | Primary metric | Decision rule |
|---|---|---|---|
| H0a | After fixing the TF lookup, measured belt velocity agrees with Gazebo ground truth. | mean absolute belt-speed error | PASS if <= 0.010 m/s at every speed |
| H0b | After fixing the speed scale and arbitration, commanded belt speed is realised. | measured/commanded ratio | PASS if ratio in [0.90, 1.10] |
| H0c | Reachability recall is reproducible from a clean commit. | recall on 100,000 targets | Report point estimate + Wilson CI; no threshold |
| H0d | The 14.6% vs 99% discrepancy is explained by a solver change, not by tuning. | recall difference between solver revisions on identical targets | Report; PASS if the revision difference explains >= 90% of the gap |
| H0e | E-STOP stop time is measurable on one instrument for both paths. | t_stop (all |qdot| < 0.01 rad/s) | Report median + bootstrap 95% CI; PASS if p95 <= 0.50 s for the safety path |
| H0f | A 2.0 s LLM watchdog fires within 2.0 s + one polling period. | detection latency | PASS if p95 <= 2.0 s + 0.05 s and no false positive on calls < 2.0 s |
| H0g | The two perception logs reconcile when keyed by frame ID and timestamp. | absolute difference in reported error | PASS if <= 0.1 mm after reconciliation |
| H0h | The scripted baseline reaches <= 10 mm endpoint error on the reach task. | Euclidean endpoint error | PASS if median <= 10 mm and p95 <= 15 mm |
| H1 | Friction grasping without attach reaches >= 80% lift-and-hold on 5 object types. | lift >= 50 mm held >= 2 s | PASS if lower Wilson bound >= 0.80 per object type and overall |
| H1b | Symmetric closure + pre-grasp alignment + effort-based slip check beats naive closure. | lift-and-hold success | PASS if paired McNemar p < 0.05 and risk difference >= 0.10 |
| H1c | Higher pad friction increases lift-and-hold success monotonically. | success vs mu | Report trend; PASS if Spearman rho > 0 with p < 0.05 |
| H2-N1 | Kinematics-anchored recursive calibration beats one-point, two-point, and fixed-scale grounding. | depth RMSE vs distance | PASS if N1 RMSE is lower than all three by a paired bootstrap 95% CI that excludes 0 |
| H2-N2 | Uncertainty-aware gating improves success-per-attempt. | success / attempts | PASS if paired risk difference >= 0.05 with 95% CI excluding 0 |
| H2-N3 | Scene-match + calibrated LLM confidence beats either alone on held-out ambiguity. | AUROC / end-to-end success | PASS if AUROC improves by >= 0.05 with 95% CI excluding 0 |
| H2-N4 | Kinematic pre-verification reduces execution-time IK failures. | IK failure rate | PASS if paired risk difference <= -0.10 with 95% CI excluding 0 |
| H5 | ARIA full outperforms each ablated variant on the six-task suite. | task success | Report Holm-corrected paired tests; no universal threshold |

---

## 3. Tasks and success predicates

The six tasks are the six primary manipulation tasks. Success is a physical predicate measured
from Gazebo state, not from the planner's self-report.

| Task | ID | Success predicate | Source |
|---|---|---|---|
| Pick and lift | T1 | object raised >= 50 mm and held >= 2.0 s, no attach | `aria_gazebo_manipulation_checker.py:38-39` |
| Drawer / slide | T2 | drawer displacement >= 30 mm | `:58` |
| Pivot / reorient | T3 | yaw error <= 0.2618 rad | `:60` |
| Tray placement | T4 | final planar offset <= 5 mm | `:41` |
| Stack | T5 | XY offset <= 3 mm, Z delta 25-38 mm, tilt <= 0.2618 rad | `:43-46` |
| Conveyor sort | T6 | object in target bin, tilt <= 0.3491 rad, relative velocity <= 0.025 m/s | `:62`, benchmark constant |

Perception conditions:

- **A (ground truth):** object pose from `/gazebo/model_states`. Upper bound only; never used
  as a system input in B or C.
- **B (RGB + homography):** detector centroid unprojected through the calibrated overhead
  camera to the table plane. No use of `true_pos[2]`.
- **C (RGB + monocular depth):** detector centroid + Depth-Anything V2 relative depth,
  calibrated by the method under test (fixed scale, one-point, two-point, or N1). No
  ground-truth depth.

No ground-truth fallback is permitted in B or C. A failed detection is logged as a failure,
not replaced by the true pose.

---

## 4. Seeds and splits

- **Development seeds:** 1-100. Used only for tuning code and thresholds. Never reported as
  evaluation.
- **Evaluation seeds:** 1001-1200 (primary), 2001-2100 (faults), 3001-3100 (perception),
  4001-4100 (depth), 5001-5100 (planning).
- **Detector data split:** scenes 1-20 train, 21-24 validation, 25-30 test. Frames are
  randomised per scene; no frame from a test scene appears in training.
- **Prompt split (Phase 4):** 120 existing prompts are partitioned deterministically by
  SHA-256 of the prompt text into 60 tuning / 30 calibration / 30 test. The partition is
  frozen in `data/journal/prompt_split.json` before any LLM evaluation.
- **Object poses:** sampled uniformly in the reachable table region with a seeded RNG;
  distractor count, lighting, and friction are logged per trial.

---

## 5. Experiment register

Every experiment writes one row per trial to `data/journal/raw/<EXP_ID>.csv` and a JSONL
sidecar with the full config. Every row includes `commit`, `dirty`, `seed`, `exp_id`,
`started_at`, `finished_at`.

| EXP ID | Phase | Trials | Seeds | Factors | Primary metric |
|---|---|---:|---|---|---|
| P0-CONV | 0a | 120 (40 x 3 speeds) | 1001-1040 | belt speed 0.04/0.08/0.12 m/s | belt-speed error, TCP tracking error |
| P0-REACH | 0b | 100,000 targets | fixed grid | solver revision (current vs `8d83b97`) | recall, position error, solve time |
| P0-ESTOP | 0c | 100 (50 x 2 paths) | 2001-2050 | command-stop vs safety-agent-stop | t_zero_cmd, t_stop, overshoot |
| P0-WD | 0d | 50 | 2101-2150 | watchdog on, timeout 2.0 s | detection latency, false positives |
| P0-PERCEPT | 0e | 300 frames | 3001-3100 | B vs C, two logging pipelines | error disagreement, mAP50, mAP50-95 |
| P0-DET | 0f | 1,000 train / 120 val / 120 test frames | scenes 1-30 | YOLOv8n 320 px | mAP50, mAP50-95 |
| P0-SCRIPT | 0g | 300 (6 tasks x 50) | 1001-1050 | scripted expert, no attach | success, endpoint error |
| P1-GRASP | 1 | 250 (5 objects x 50) | 1101-1150 | object type | lift-and-hold |
| P1-GRASP-ABL | 1 | 200 (4 variants x 50) | 1101-1150 | naive / symmetric / +align / +slip | lift-and-hold |
| P1-FRICTION | 1 | 80 (4 mu x 20) | 1201-1220 | mu 0.5/1.0/1.5/2.5 | lift-and-hold |
| P1-PLACE | 1 | 150 (3 tasks x 50) | 1301-1350 | tray/stack/conveyor | task success |
| P2-N1 | 2 | 400 (4 methods x 100 frames) | 4001-4100 | fixed/one-point/two-point/N1 | depth RMSE vs distance |
| P2-N1-PICK | 2 | 200 (4 methods x 50) | 4101-4150 | same | pick success |
| P2-N2 | 2 | 300 (2 gating x 3 tasks x 50) | 5001-5050 | gating off/on | success per attempt |
| P2-N3 | 2 | 300 prompts (3 methods x 100) | prompt split | scene-only / LLM-only / fused | AUROC, success |
| P2-N4 | 2 | 200 (2 x 100 plans) | 5101-5200 | verification off/on | IK failure rate, replans |
| P5-SUITE | 5 | 900 (6 tasks x 3 perception x 50) | 1001-1050 | A/B/C | task success |
| P6-ABL | 6 | 750 (15 agents x 50) + 250 variants | 1401-1450 | agent removed | task success |
| P6-PERCEPT | 6 | 200 (4 variants x 50) | 1501-1550 | GT/homography/fixed/N1 | task success |
| P6-GRASP | 6 | 150 (3 variants x 50) | 1601-1650 | oracle/naive/N2 | task success |

### 5.1 Power analysis

For a two-proportion comparison with alpha = 0.05 (two-sided) and 80% power:

- 0.80 vs 0.60: n = 82 per group.
- 0.80 vs 0.65: n = 121 per group.
- 0.90 vs 0.70: n = 58 per group.

Therefore 50 trials per cell detects only large effects (>= 25 percentage points) at ~50-60%
power. The primary comparison cells (T1 pick, T6 conveyor, P1-GRASP) will use 100 trials where
the wall-clock budget permits; 50-trial cells are labelled **exploratory** in every table. This
limitation is reported rather than hidden.

---

## 6. Metrics and statistical tests

- Proportions: Wilson score 95% CI. No normal approximation.
- Paired binary comparisons on matched seeds: McNemar exact test; risk difference with
  Newcombe 95% CI; odds ratio with Haldane-Anscombe correction when a cell is zero.
- Multiple comparisons: Holm-Bonferroni within each experiment family.
- Latencies: median and bootstrap 95% CI (10,000 resamples, percentile method).
- Continuous errors: median, IQR, and paired bootstrap CI of the difference.
- Calibration: ECE (15 bins), Brier score, AUROC, reliability diagram.
- Success-coverage: success rate as a function of the coverage of accepted attempts.
- Effect sizes are reported even when p > 0.05.

All tests are two-sided. A result is called significant only after the Holm correction for its
family.

---

## 7. Provenance and refusal rules

1. `provenance.assert_clean()` runs at the start of every experiment and aborts if
   `git status --porcelain` is non-empty.
2. The commit hash and `dirty` flag are written into every row. A dirty row is invalid.
3. Raw logs are append-only. A failed run is kept and reported; it is not deleted.
4. No script may sample, interpolate, or formula-generate an outcome. The integrity linter
   (`scripts/check_no_synthetic_outcomes.py`) is extended to flag constant outcome columns
   and fallback branches.
5. A benchmark that cannot execute is recorded as `NOT_EXECUTED` with the error text.
6. Development seeds are never used in a reported table.

---

## 8. Stopping rules

- A tuning loop stops after at most 20 configurations or 2 hours of wall-clock, whichever
  comes first. The best configuration on development seeds is frozen before evaluation.
- If a target is not met after the bounded effort, the paper reports the best honest result and
  weakens the claim; the data are not changed.
- If Gazebo crashes mid-trial, the trial is logged as `CRASH` and counted as a failure in the
  intention-to-treat analysis. A separate per-protocol analysis is reported.

---

## 9. Freeze record

| Item | Value |
|---|---|
| Pre-registration commit | filled by `git log -1 --format=%H` at first evaluation |
| Thresholds frozen at | this commit |
| Evaluation started at | not yet |
| Deviations | `DEVIATIONS.md` |

Signed off by: automated study harness (no human PI available in this environment).
