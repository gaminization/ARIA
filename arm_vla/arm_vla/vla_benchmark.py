#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA VLA Benchmark

Evaluates VLA backends the only way an evaluation may be run:
by measuring what actually happened.

What this benchmark measures (real, always):
  * whether a backend loads its weights           -> load_ok
  * whether it produces a valid action            -> action_ok
  * wall-clock inference latency                  -> inference_ms
  * the dimensionality of the emitted action      -> action_dim

What this benchmark does NOT do:
  * It never invents a task success rate, a final positioning
    error, or a trajectory smoothness score from a random
    number generator, a difficulty prior, or a task-name hash.

Task success can only be reported when a real closed-loop
rollout is supplied (a Gazebo-backed `rollout_fn`, or an
equivalent measured evaluator). Without one, the success
columns are emitted as "unmeasured" and the report says so
explicitly. A benchmark that cannot measure is expected to
report that it cannot measure, not to synthesise a result.

Backends: LeRobot ACT, OpenVLA, Pi0, Gr00t-N1 (see vla_interface.py)
═══════════════════════════════════════════════════════════════
"""
import time
import os
import csv
import argparse
from typing import List, Optional, Callable
from dataclasses import dataclass, field

import numpy as np

from arm_vla.vla_interface import (
    VLAInterface, VLAInput,
)


# ═══════════════════════════════════════════════════════════════
# Benchmark Tasks
# ═══════════════════════════════════════════════════════════════

@dataclass
class BenchmarkTask:
    """A standardized benchmark task."""
    name: str
    instruction: str
    target_object_class: str
    target_position: List[float]   # [x, y, z] in meters
    success_threshold_m: float = 0.03  # 3cm
    timeout_s: float = 30.0
    difficulty: str = "easy"       # easy, medium, hard


BENCHMARK_TASKS: List[BenchmarkTask] = [
    # Easy: single object pick
    BenchmarkTask("pick_cube_center", "Pick up the red cube",
                  "cube", [0.20, 0.0, 0.78], difficulty="easy"),
    BenchmarkTask("pick_cube_left", "Pick up the cube on the left",
                  "cube", [0.15, 0.10, 0.78], difficulty="easy"),
    BenchmarkTask("pick_cylinder", "Pick up the cylinder",
                  "cylinder", [0.22, -0.05, 0.78], difficulty="easy"),

    # Medium: pick and place
    BenchmarkTask("place_cube_box", "Put the cube in the box",
                  "cube", [0.25, 0.08, 0.78], difficulty="medium"),
    BenchmarkTask("stack_two", "Stack the red cube on the blue cube",
                  "cube", [0.18, 0.0, 0.82], difficulty="medium"),
    BenchmarkTask("push_to_zone", "Push the ball to the right",
                  "ball", [0.20, -0.10, 0.78], difficulty="medium"),

    # Hard: multi-step or precision
    BenchmarkTask("sort_by_color", "Sort objects by color",
                  "cube", [0.20, 0.0, 0.78], difficulty="hard"),
    BenchmarkTask("stack_three", "Stack all three cubes",
                  "cube", [0.20, 0.0, 0.86], difficulty="hard"),
    BenchmarkTask("precise_place", "Place the pen exactly on the mark",
                  "pen", [0.22, 0.03, 0.78], difficulty="hard"),
    BenchmarkTask("tool_reorient", "Pick up the screwdriver by the handle",
                  "screwdriver", [0.18, -0.05, 0.78], difficulty="hard"),
]


# ═══════════════════════════════════════════════════════════════
# Result structures
# ═══════════════════════════════════════════════════════════════

@dataclass
class TaskResult:
    """
    Result of a single benchmark task.

    `measured` distinguishes a real observation from an absence of
    evidence. When `measured` is False the numeric outcome fields
    (`success`, `final_error_m`, `smoothness`) are None and must be
    rendered as "unmeasured" rather than as a number.
    """
    task_name: str
    backend_name: str
    measured: bool
    success: Optional[bool] = None
    duration_s: float = 0.0
    final_error_m: Optional[float] = None
    smoothness: Optional[float] = None
    n_steps: int = 0
    inference_ms: float = 0.0
    action_dim: int = 0
    load_ok: bool = False
    action_ok: bool = False
    failure_reason: str = ""

    def outcome_str(self) -> str:
        if not self.measured:
            return "unmeasured"
        return "success" if self.success else "failure"


@dataclass
class BenchmarkReport:
    """Aggregate benchmark report for one backend."""
    backend_name: str
    available: bool
    total_tasks: int
    measured_tasks: int
    successes: Optional[int]          # None when nothing was measured
    success_rate: Optional[float]     # None when nothing was measured
    avg_duration_s: float = 0.0
    avg_inference_ms: float = 0.0
    results: List[TaskResult] = field(default_factory=list)

    def summary_line(self) -> str:
        if not self.available:
            return f"{self.backend_name}: unavailable (model not loaded) — not measured"
        if self.measured_tasks == 0:
            return (
                f"{self.backend_name}: loaded, inference={self.avg_inference_ms:.0f}ms "
                f"| task success NOT measured (no simulator/dataset rollout)"
            )
        return (
            f"{self.backend_name}: "
            f"{self.success_rate:.0%} success over {self.measured_tasks} "
            f"measured task(s) | "
            f"{self.avg_duration_s:.1f}s avg | "
            f"{self.avg_inference_ms:.0f}ms inference"
        )


# ═══════════════════════════════════════════════════════════════
# Rollout contract
# ═══════════════════════════════════════════════════════════════

@dataclass
class RolloutOutcome:
    """
    The result of a REAL closed-loop rollout of one task.

    A rollout function must only return an outcome it actually
    observed in a simulator or on hardware. It must never be a
    fabricated dict of priors.
    """
    success: bool
    final_error_m: float
    n_steps: int
    duration_s: float
    smoothness: Optional[float] = None
    failure_reason: str = ""


# A rollout_fn executes one task end-to-end and returns a measured
# RolloutOutcome. Signature: (task, backend) -> RolloutOutcome.
RolloutFn = Callable[[BenchmarkTask, object], RolloutOutcome]


# ═══════════════════════════════════════════════════════════════
# Rule-Based Baseline
# ═══════════════════════════════════════════════════════════════

class RuleBasedBaseline:
    """
    Rule-based planning baseline (Stage 2 IK + grasp pipeline, no VLA).

    This class owns no model. It can only report a task outcome when
    a real rollout function is supplied by the caller. Without one,
    `execute_task` returns an explicitly unmeasured result — it never
    guesses a success or an error.
    """

    def __init__(self, rollout_fn: Optional[RolloutFn] = None):
        self.name = "RuleBasedBaseline"
        self.rollout_fn = rollout_fn

    def execute_task(self, task: BenchmarkTask,
                     sim_state: Optional[dict] = None) -> TaskResult:
        """
        Measure one baseline task via the supplied rollout.

        With no rollout function this returns measured=False: the
        baseline is present and routable but was not evaluated here.
        """
        t0 = time.time()

        if self.rollout_fn is None:
            return TaskResult(
                task_name=task.name,
                backend_name=self.name,
                measured=False,
                duration_s=time.time() - t0,
                load_ok=True,
                failure_reason=(
                    "not measured: no simulator/dataset rollout was "
                    "supplied, so no success or error can be reported"),
            )

        outcome = self.rollout_fn(task, self)
        trajectory = getattr(outcome, "trajectory", None)
        smoothness = outcome.smoothness
        if smoothness is None and trajectory is not None and len(trajectory) > 3:
            jerk = np.diff(np.asarray(trajectory), n=3, axis=0)
            smoothness = float(np.mean(np.abs(jerk)))

        return TaskResult(
            task_name=task.name,
            backend_name=self.name,
            measured=True,
            success=bool(outcome.success),
            duration_s=float(outcome.duration_s),
            final_error_m=float(outcome.final_error_m),
            smoothness=smoothness,
            n_steps=int(outcome.n_steps),
            inference_ms=0.0,
            load_ok=True,
            action_ok=bool(outcome.success),
            failure_reason=outcome.failure_reason,
        )


# ═══════════════════════════════════════════════════════════════
# VLA Task Executor
# ═══════════════════════════════════════════════════════════════

class VLATaskExecutor:
    """
    Evaluates a VLA backend.

    Two independent measurements are performed:

      1. Availability: run the backend on a real observation and
         record whether it emitted a valid action plus its latency.
         This is always real and is always reported.

      2. Task outcome: delegated to a real `rollout_fn` when one is
         provided. Without a rollout the success/error columns stay
         unmeasured.
    """

    def __init__(self, vla: VLAInterface,
                 rollout_fn: Optional[RolloutFn] = None,
                 observation_provider: Optional[Callable[[BenchmarkTask], "object"]] = None):
        self.vla = vla
        self.rollout_fn = rollout_fn
        self.observation_provider = observation_provider

    def _build_input(self, task: BenchmarkTask) -> VLAInput:
        """Build a VLA input from a real observation if one is provided."""
        top = None
        wrist = None
        joints = np.zeros(4, dtype=np.float32)
        gripper = 0.0
        if self.observation_provider is not None:
            obs = self.observation_provider(task)
            top = getattr(obs, "top_camera_image", None)
            wrist = getattr(obs, "wrist_camera_image", None)
            joints = np.asarray(getattr(obs, "joint_positions", joints),
                                dtype=np.float32)
            gripper = float(getattr(obs, "gripper_position", 0.0))
        return VLAInput(
            top_camera_image=top,
            wrist_camera_image=wrist,
            joint_positions=joints,
            gripper_position=gripper,
            instruction=task.instruction,
            timestep=0,
        )

    def probe_backend(self, task: BenchmarkTask) -> TaskResult:
        """
        Measure real inference behaviour on one task. Never fabricates
        a success: `measured` is False and `success` is None.
        """
        t0 = time.time()
        backend_name = self.vla.active_name
        vla_input = self._build_input(task)
        output = self.vla.predict(vla_input)
        duration = time.time() - t0

        command = np.atleast_1d(
            np.asarray(output.joint_commands, dtype=np.float32))
        action_ok = bool(command.size >= 4 and np.all(np.isfinite(command)))
        notes: List[str] = []
        if vla_input.wrist_camera_image is None and \
                vla_input.top_camera_image is None:
            notes.append("no camera frame supplied")
        if not action_ok:
            notes.append(f"backend returned no usable action "
                         f"(reason: {output.reasoning or 'n/a'})")

        return TaskResult(
            task_name=task.name,
            backend_name=backend_name,
            measured=False,
            success=None,
            final_error_m=None,
            smoothness=None,
            n_steps=0,
            inference_ms=self.vla.get_stats().get("avg_inference_ms", 0.0),
            action_dim=int(command.size),
            load_ok=True,
            action_ok=action_ok,
            duration_s=duration,
            failure_reason="; ".join(notes) if notes else
            "inference measured; task success requires a simulator rollout",
        )

    def execute_task(self, task: BenchmarkTask,
                     sim_state: Optional[dict] = None) -> TaskResult:
        """
        Measure one VLA task.

        Without a rollout function this degrades to `probe_backend`
        (real inference measurement, unmeasured outcome). With one, the
        task is executed closed-loop and the outcome is reported as
        measured.
        """
        if self.rollout_fn is None:
            return self.probe_backend(task)

        t0 = time.time()
        backend_name = self.vla.active_name
        outcome = self.rollout_fn(task, self.vla)
        duration = time.time() - t0

        trajectory = getattr(outcome, "trajectory", None)
        smoothness = outcome.smoothness
        if smoothness is None and trajectory is not None and len(trajectory) > 3:
            jerk = np.diff(np.asarray(trajectory), n=3, axis=0)
            smoothness = float(np.mean(np.abs(jerk)))

        return TaskResult(
            task_name=task.name,
            backend_name=backend_name,
            measured=True,
            success=bool(outcome.success),
            duration_s=float(outcome.duration_s) if outcome.duration_s else duration,
            final_error_m=float(outcome.final_error_m),
            smoothness=smoothness,
            n_steps=int(outcome.n_steps),
            inference_ms=self.vla.get_stats().get("avg_inference_ms", 0.0),
            action_dim=4,
            load_ok=True,
            action_ok=True,
            failure_reason=outcome.failure_reason,
        )


# ═══════════════════════════════════════════════════════════════
# Benchmark Runner
# ═══════════════════════════════════════════════════════════════

class VLABenchmark:
    """
    Runs the VLA benchmark.

    Output:
      Console: per-backend summary (unmeasured outcomes are labelled as such)
      CSV: detailed results (arm_planner/logs/vla_benchmark.csv)

    The CSV always carries a `measured` column so downstream analysis
    cannot silently treat an unmeasured row as a success or failure.
    """

    def __init__(self, output_dir: str = "",
                 rollout_fn: Optional[RolloutFn] = None,
                 observation_provider: Optional[Callable] = None):
        if not output_dir:
            output_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "..", "arm_planner", "logs")
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self.rollout_fn = rollout_fn
        self.observation_provider = observation_provider
        self.vla = VLAInterface()
        self.baseline = RuleBasedBaseline(rollout_fn=rollout_fn)
        self.reports: List[BenchmarkReport] = []

    def run(self, tasks: Optional[List[BenchmarkTask]] = None,
            backends: Optional[List[str]] = None) -> List[BenchmarkReport]:
        if tasks is None:
            tasks = BENCHMARK_TASKS
        if backends is None:
            backends = ["LeRobot-ACT", "OpenVLA"]

        self.reports.clear()
        sim_state: dict = {}

        # ── Baseline ───────────────────────────────────────────
        print("\n" + "═" * 60)
        print(" VLA BENCHMARK — Rule-Based Baseline")
        print("═" * 60)
        baseline_results = []
        for task in tasks:
            result = self.baseline.execute_task(task, sim_state)
            baseline_results.append(result)
            print(f"  {result.outcome_str():>10} {task.name}: "
                  f"{self._detail(result)}")
        self.reports.append(
            self._make_report(self.baseline.name, True, baseline_results))

        # ── VLA backends ───────────────────────────────────────
        for backend_name in backends:
            print(f"\n{'═' * 60}")
            print(f" VLA BENCHMARK — {backend_name}")
            print("═" * 60)

            if not self.vla.select_backend(backend_name):
                self.reports.append(
                    self._make_report(backend_name, False, []))
                continue

            loaded = self.vla.load()
            if not loaded:
                # A model that cannot be loaded is reported as
                # unavailable. No results, real or otherwise, are emitted.
                print(f"  ⚠ {backend_name} unavailable (not loaded) — "
                      f"no results will be reported")
                self.reports.append(
                    self._make_report(backend_name, False, []))
                self.vla.unload()
                continue

            executor = VLATaskExecutor(
                self.vla,
                rollout_fn=self.rollout_fn,
                observation_provider=self.observation_provider)
            results = []
            for task in tasks:
                result = executor.execute_task(task, sim_state)
                results.append(result)
                print(f"  {result.outcome_str():>10} {task.name}: "
                      f"{self._detail(result)}")

            self.reports.append(
                self._make_report(backend_name, True, results))
            self.vla.unload()

        self._print_summary()
        self._save_csv()
        return self.reports

    @staticmethod
    def _detail(result: TaskResult) -> str:
        if not result.measured:
            extra = f", inference={result.inference_ms:.0f}ms"
            if not result.action_ok:
                extra += f", {result.failure_reason}"
            return "unmeasured" + extra
        return (f"{result.duration_s:.1f}s, "
                f"error={result.final_error_m * 100:.1f}cm")

    def _make_report(self, name: str, available: bool,
                     results: List[TaskResult]) -> BenchmarkReport:
        measured = [r for r in results if r.measured]
        if measured:
            successes = sum(1 for r in measured if r.success)
            success_rate = successes / len(measured)
            avg_duration = float(np.mean([r.duration_s for r in measured]))
        else:
            successes = None
            success_rate = None
            avg_duration = (
                float(np.mean([r.duration_s for r in results]))
                if results else 0.0)
        avg_inference = (
            float(np.mean([r.inference_ms for r in results]))
            if results else 0.0)
        return BenchmarkReport(
            backend_name=name,
            available=available,
            total_tasks=len(results),
            measured_tasks=len(measured),
            successes=successes,
            success_rate=success_rate,
            avg_duration_s=avg_duration,
            avg_inference_ms=avg_inference,
            results=results,
        )

    def _print_summary(self):
        print("\n" + "═" * 60)
        print(" VLA BENCHMARK RESULTS")
        print("═" * 60)
        for report in self.reports:
            print(f"  {report.summary_line()}")
        print("═" * 60)

        measured = [r for r in self.reports if r.measured_tasks > 0]
        if not measured:
            print("\n  No task outcome was measured. Reporting a "
                  "'best backend' or a success rate here would be a "
                  "fabrication.")
            print("  Provide a real simulator rollout "
                  "(e.g. the Gazebo manipulation harness) to obtain "
                  "success/error numbers.")
            print()
            return

        best = max(measured, key=lambda r: r.success_rate)
        print(f"\n  Best measured backend: '{best.backend_name}' "
              f"({best.success_rate:.0%} success over "
              f"{best.measured_tasks} tasks)")
        print()

    def _save_csv(self):
        csv_path = os.path.join(self.output_dir, "vla_benchmark.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "backend", "task", "difficulty", "measured", "success",
                "duration_s", "error_m", "smoothness",
                "n_steps", "inference_ms", "action_dim",
                "load_ok", "action_ok", "failure_reason",
            ])
            for report in self.reports:
                for r in report.results:
                    task = next(
                        (t for t in BENCHMARK_TASKS if t.name == r.task_name),
                        None)
                    writer.writerow([
                        r.backend_name, r.task_name,
                        task.difficulty if task else "",
                        1 if r.measured else 0,
                        "" if r.success is None else (1 if r.success else 0),
                        f"{r.duration_s:.3f}",
                        "" if r.final_error_m is None
                        else f"{r.final_error_m:.4f}",
                        "" if r.smoothness is None
                        else f"{r.smoothness:.6f}",
                        0 if not r.measured else r.n_steps,
                        f"{r.inference_ms:.1f}",
                        r.action_dim,
                        1 if r.load_ok else 0,
                        1 if r.action_ok else 0,
                        r.failure_reason,
                    ])
        print(f"  Results saved to: {csv_path}")


def main():
    """Run the VLA benchmark from CLI."""
    parser = argparse.ArgumentParser(
        description="ARIA VLA benchmark (measures real backend behaviour; "
                    "never fabricates task success).")
    parser.add_argument(
        "--output-dir", default="",
        help="directory for vla_benchmark.csv")
    parser.add_argument(
        "--backends", nargs="*", default=None,
        help="backend names to evaluate (default: LeRobot-ACT OpenVLA)")
    parser.add_argument(
        "--check", action="store_true",
        help="profile-only mode: load each backend and report availability "
             "and inference latency without claiming any task outcome")
    args = parser.parse_args()

    benchmark = VLABenchmark(output_dir=args.output_dir)
    if args.check:
        print("\nCHECK MODE: no rollout is available, so task success is "
              "NOT measured. Only backend availability and latency are "
              "reported.\n")
    benchmark.run(backends=args.backends)


if __name__ == "__main__":
    main()
