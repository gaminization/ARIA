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

As declared in `PREREGISTRATION.md` §1.1: learned-policy training (no CUDA), human
inter-annotator agreement (no independent annotators), hardware (no robot), and the full
detector training run (no committed checkpoint and no provenance-clean labelled dataset).
N1 was not executed because the depth model package and checkpoint are absent. N2 was not
executed because grasp attempts are blocked by the gripper defect. N3 was not executed within
the available budget. P0-WD is N/A because no 2.0 s watchdog exists in the code; the only
configured timeout is 90 s.

## D11. Freeze-record metadata

Only the metadata rows of `PREREGISTRATION.md` §9 were filled after the first evaluation
(hypotheses, thresholds, seeds, and trial counts were not edited). The pre-registration
commit is `cf5d7b4`; the first evaluation started at `2026-10-24T19:14:41Z`.
