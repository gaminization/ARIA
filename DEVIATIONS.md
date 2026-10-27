# Deviations from the Pre-Registration

**Pre-registration:** `PREREGISTRATION.md`, frozen at commit `cf5d7b4`.
**First evaluation run:** P0-REACH, started `2026-10-24T19:14:41Z`.
**Rule:** a deviation is recorded here; it is never used to change a reported value after
the fact. The affected raw logs are kept.

---

## D1. P0-REACH was re-run to add per-row provenance

The first P0-REACH run (`83909cb`, 100,000 targets) wrote a summary with the commit hash but
its CSV rows predated the `TrialLog` provenance columns. The pre-registration requires the
commit and dirty flag in **every row**. The run was repeated from clean commit `7e9d762`
with the same seed (`20261025`), the same fixed target grid, the same 5 mm tolerance, and the
same two solver revisions. The first raw logs are archived under
`data/journal/raw/archive/P0-REACH-legacy-83909cb.*`.

The rerun is a logging repair, not a re-selection. Recall and error values are expected to be
identical because the target generation and both solvers are deterministic; solve-time
medians may differ because they are wall-clock measurements.

## D2. P2-N4 target box narrowed before the evaluation seeds

The first development version sampled targets from
`x=[0.10,0.32], y=[-0.20,0.20], z=[0.60,0.78]`. The evaluation run uses
`x=[0.08,0.30], y=[-0.18,0.18], z=[0.66,0.78]`, the measured task workspace box.
The change was made on development seeds before evaluation seeds `5101-5200` were used.
Thresholds (`success_mm=5.0`, `verify_mm=2.0`, `limit_margin_rad=0.02`, `replans=3`,
`nearby_m=0.02`) were not changed after the evaluation run.

## D3. P2-N4 risk-difference interval

The pre-registration specifies a Newcombe 95% interval for risk differences. Because the two
pipelines share the same targets, the evaluation reports a paired percentile bootstrap
(10,000 resamples) and keeps the exact McNemar test. The Newcombe routine in
`aria_study/stats.py` is the unpaired variant and is not used for this paired comparison.
This is a statistical-method deviation, reported in the paper.

## D4. P2-N4 primary metric is reported two ways

The pre-registration names "IK failure rate" as the primary metric. The paper reports both
the intention-to-treat success (all sampled targets, rejected plans counted as failures) and
the execution-failure rate among accepted plans. The conditional rate is lower for N4, but
the accepted set is selected after verification; the paper does not claim a pre-registered
pass on the conditional comparison.

## D5. P0-ESTOP stop threshold

The pre-registration defines `t_stop` as sustained `|dq/dt| < 0.01 rad/s` for 0.2 s. An
earlier draft used `0.05 rad/s`. The final run logs both: `0.01 rad/s` is the primary,
pre-registered value, and `0.05 rad/s` is a sensitivity check. The paper reports the primary
value and the sensitivity value.

## D6. P0-ESTOP path-message matching

Three `manual_control_node` instances were publishing on the controller topic. To attribute a
halt to the path under test, the final run matches the command path to a 5-joint message with
velocities and the safety path to a 4-joint message, and logs the number of other trajectory
messages per trial. The duplicate nodes were terminated before the final run. This is an
instrumentation repair made before the reported evaluation run, not a post-hoc filter.

## D7. P0-ESTOP wall-to-sim time mapping

The final run interpolates the recent `(wall, sim-stamp)` pairs from `/joint_states` instead
of extrapolating from the latest sample. A stalled joint-state stream could otherwise map the
same wall instant to different sim times. This is an instrumentation repair made before the
reported evaluation run.

## D8. P0-CONV belt-speed metric

The pre-registration names "belt-speed error". The belt's visual link is not the transport:
`P0-CONV` therefore measures the realized part speed from `/gazebo/link_states`, the visual
belt-link speed, and the roller speed, and reports all three. The belt-speed hypothesis is
evaluated against the realized part speed because that is the quantity the sorting task
depends on.

## D9. Summary commit differs from row commit for P0-ESTOP

The P0-ESTOP summary was written at the end of the run, after a documentation-only commit
(`d978641`) landed. The rows were stamped at the start of the run (`99966bd`). The code is
identical between the two commits (the later commit changes `AUDIT.md` only). `build_numbers.py`
now uses the commit stamped on the rows and warns when the summary differs.

## D10. Phases and methods not executed

After the repairs recorded below, the remaining non-executed items are:

- **N2 / T1 pick-and-lift**: the position-controlled jaws cannot satisfy the pre-registered
  50 mm lift predicate (D12). Every T1-dependent experiment is therefore not executable.
- **Phase 3 learned policies** (LeRobot/ACT/SmolVLA): CUDA is now available, but the policies
  require T1 demonstrations and T1 is not executable. No policy is trained or evaluated.
- **Phase 4 human inter-annotator agreement**: no independent human annotators are available.
  A second language model is not a human substitute, so no Cohen's kappa is reported.
- **Phase 7 hardware**: no robot, ESP32, or serial device is present.
- **Phase 0f 5,000-frame detector**: reduced by the pre-registration to 1,000 train frames
  (1,240 total); the 5,000-frame request is not met (D13).

N1, N3, and P0-WD were subsequently executed and are reported from their raw logs. N1's depth
predictions come from the installed Depth-Anything V2 checkpoint; N3 uses the pre-registered
60-prompt split; P0-WD exercises the 2.0 s watchdog that was added to the stack.

## D11. Freeze-record metadata

Only the metadata rows of `PREREGISTRATION.md` §9 were filled after the first evaluation
(hypotheses, thresholds, seeds, and trial counts were not edited). The pre-registration
commit is `cf5d7b4`; the first evaluation started at `2026-10-24T19:14:41Z`.

## D12. N2 not executed: platform cannot satisfy the T1 pick-and-lift predicate

`P2-N2` requires the pre-registered T1 predicate ("object raised >= 50 mm and held >= 2.0 s,
no attach", `PREREGISTRATION.md` §3). Two independent platform defects prevented it.

1. **Conveyor velocity override pinned the workpieces.** `AriaConveyorPlugin::OnUpdate`
   applied `SetLinearVel(..., min(0, v_z))` and `SetAngularVel(0, 0, 0)` to every workpiece
   inside the `z = 0.620–0.680 m` detection window, regardless of belt state. Upward motion
   was cancelled every tick and rotation was suppressed, so a 50 mm lift was physically
   impossible (it would stall at the top of the window, +32 mm). Fixed by gating the
   override on `|belt_velocity| > 1e-4`; a stopped belt now leaves parts as free bodies.
   This is a platform bug fix, not a change to the study protocol.

2. **The position-controlled jaws cannot regulate grip force.** After the conveyor fix, the
   pads contact the 40 mm cube symmetrically (~9 N per pad) with faces that are vertical and
   antiparallel at the grasp opening (measured normal dot `-1.0000`, tilt `0.00 deg`). But the
   first contact step produces 9–10 N because the gripper is position-controlled and the
   contact is stiff; that shove displaces the 45 g cube 1–7 mm and tilts it, and the cube then
   escapes sideways during the lift. Best observed lift was 0.9 mm. Lowering the pad contact
   `kp`, lowering the joint effort limit (0.18 N·m), and lowering the Gazebo position PID gain
   did not reduce the contact force, so no compliant close was available without replacing the
   gripper controller.

No 50 mm lift was ever observed, so `P2-N2` is recorded as **not executed**, not simulated and
not re-labelled with a weaker predicate. The pad collision boxes were also re-oriented so their
faces are vertical and antiparallel at the grasp opening, and their contact friction was set to
`mu = 3`; those repairs are kept because they are physically correct, but they do not change
the N2 outcome. Any T1-dependent experiment (`P5-SUITE` T1, `P6-GRASP`, `P6-ABL`, learned-policy
demonstrations) is likewise not executable on this platform.

## D13. P0-DET executed on GPU with a newly generated dataset

`PREREGISTRATION.md` §1.1 declared the detector run as "1,000 auto-labelled Gazebo frames,
YOLOv8n at 320 px on CPU" because no CUDA device was available when the protocol was frozen.
A CUDA device (RTX 5060 Laptop, 8 GB; PyTorch 2.10.0+cu128) became available before this run.
The model (YOLOv8n), input resolution (320 px), class definition (one class, `workpiece`), split
(scenes 1-20 train, 21-24 validation, 25-30 test), and metrics (mAP50, mAP50-95) are unchanged;
only the compute device changed, from CPU to `cuda:0`. Training is deterministic (`seed=0`,
`deterministic=True`) and the checkpoint hash is recorded.

The old detector frames were not reused: the audit found their labels were sampled at random
(`AUDIT.md` §5.7) and no checkpoint was committed. P0-DET instead generates 1,240 frames
(1,000 train / 120 validation / 120 test) from Gazebo ground truth projected through the
calibrated overhead camera, with scene seeds and per-image SHA-256 hashes logged. The original
Phase 0f request for 5,000 frames is still not met; the shortfall is reported, not interpolated.
