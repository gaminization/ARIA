#!/usr/bin/env python3
"""P0-ESTOP: command-stop versus safety-agent stop, measured in Gazebo.

Both paths are triggered while the arm is executing the same commanded motion.
The physical outcome is read from ``/joint_states`` (positions and velocities),
not from any node's self-report.

Path ``command`` publishes a five-joint hold trajectory at the measured joint
positions.  Path ``safety`` calls the SafetyAgent ``/aria/estop`` service, which
publishes its own halt trajectory.

Operational definitions (fixed before the run):
  t_zero_cmd   sim seconds from the stop signal to the first trajectory message
               received on the controller topic after the signal.
  t_stop       sim seconds from the signal to the first joint-state sample at
               which every arm-joint speed stays below 0.05 rad/s for 0.2 s.
  overshoot    maximum single-joint absolute displacement after the signal.

Usage:
    python3 scripts/study/p0_estop.py --smoke
    python3 scripts/study/p0_estop.py
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import rclpy  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from trajectory_msgs.msg import JointTrajectory  # noqa: E402

from aria_study import provenance, stats  # noqa: E402
from aria_study.ros import StudyNode  # noqa: E402
from aria_study.trial_log import TrialLog  # noqa: E402

ARM = ["waist_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint"]
ALL = ARM + ["gripper_joint"]

START_Q = (0.0, 0.8, -0.5, 1.2, 0.5)
GOAL_Q = (0.8, 1.2, -1.0, 1.4, 0.5)
MOVE_S = 2.5
SIG_RANGE = (0.50, 0.70)
OBSERVE_S = 1.50
VEL_THRESH = 0.05
HOLD_S = 0.20


class EstopNode(StudyNode):
    def __init__(self):
        super().__init__("aria_p0_estop")
        self.sim_time = -1.0
        self.last_stamp: Optional[float] = None
        self.last_wall: Optional[float] = None
        self.samples: List[Tuple[float, np.ndarray, np.ndarray]] = []
        self.anchors: List[Tuple[float, float]] = []
        self.traj_events: List[Tuple[float, int, bool]] = []
        self.estop_active = False
        self.create_subscription(Clock, "/clock", self._clock_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 100)
        self.create_subscription(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory",
            self._traj_event_cb, 50,
        )
        self.create_subscription(Bool, "/aria/estop_active", self._estop_cb, 10)

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _js_cb(self, msg: JointState):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = np.full(len(ARM), np.nan)
        qd = np.full(len(ARM), np.nan)
        for i, name in enumerate(ARM):
            if name in msg.name:
                j = msg.name.index(name)
                if j < len(msg.position):
                    q[i] = msg.position[j]
                if j < len(msg.velocity):
                    qd[i] = msg.velocity[j]
        self.samples.append((stamp, q, qd))
        now = time.monotonic()
        self.anchors.append((now, stamp))
        if len(self.anchors) > 400:
            self.anchors = self.anchors[-400:]
        self.last_stamp = stamp
        self.last_wall = now

    def _traj_event_cb(self, msg: JointTrajectory):
        self.traj_events.append(
            (time.monotonic(), len(msg.joint_names), bool(msg.points and msg.points[0].velocities))
        )

    def _estop_cb(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def sim_at(self, wall: float) -> float:
        """Map a wall-clock instant to sim time by interpolating joint stamps.

        The previous implementation extrapolated from the single latest sample.
        When the joint-state stream stalls briefly, that anchor moves and the
        same wall instant maps to different sim times.  Interpolation over the
        recent (wall, stamp) pairs is monotone and reproducible.
        """
        if not self.anchors:
            return float("nan")
        arr = np.asarray(self.anchors, dtype=float)
        if arr.shape[0] == 1:
            return float(arr[0, 1])
        if wall <= arr[0, 0]:
            return float(arr[0, 1] + (wall - arr[0, 0]))
        if wall >= arr[-1, 0]:
            # Extrapolate with the median rate over the last 0.5 s.
            recent = arr[arr[:, 0] >= arr[-1, 0] - 0.5]
            if recent.shape[0] >= 2:
                rate = (recent[-1, 1] - recent[0, 1]) / max(1e-9, recent[-1, 0] - recent[0, 0])
            else:
                rate = 1.0
            return float(arr[-1, 1] + (wall - arr[-1, 0]) * rate)
        return float(np.interp(wall, arr[:, 0], arr[:, 1]))

    def wait_sim(self, seconds: float, timeout: float = 30.0) -> None:
        start = self.last_stamp if self.last_stamp is not None else self.sim_time
        self.wait_until(
            lambda: (self.last_stamp if self.last_stamp is not None else -1.0) >= start + seconds,
            timeout,
        )

    def sample_at(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        if not self.samples:
            raise RuntimeError("no joint samples")
        arr = np.array([s[0] for s in self.samples])
        i = int(np.argmin(np.abs(arr - t)))
        return self.samples[i][1], self.samples[i][2]


def stop_metrics(
    samples: List[Tuple[float, np.ndarray, np.ndarray]],
    t_sig: float,
    q_sig: np.ndarray,
    t_end: float,
) -> Tuple[float, float, float, bool, float]:
    after = [s for s in samples if t_sig - 1e-9 <= s[0] <= t_end + 1e-9]
    if len(after) < 5:
        return float("nan"), float("nan"), float("nan"), False, float("nan")
    ts = np.array([s[0] for s in after])
    qs = np.array([s[1] for s in after])
    vd_reported = np.array([s[2] for s in after])
    disp = np.abs(qs - q_sig[None, :])
    overshoot = float(np.max(disp))
    overshoot_norm = float(np.max(np.linalg.norm(qs - q_sig[None, :], axis=1)))

    # ``/joint_states.velocity`` in this workcell is not the derivative of the
    # measured position (it is offset by roughly an order of magnitude and the
    # waist sign is inverted).  Differentiate the measured positions instead;
    # the reported field is kept only as a diagnostic.
    vd = np.gradient(qs, ts, axis=0)
    vd_reported_max = float(np.nanmax(np.abs(vd_reported)))

    t_stop = float("nan")
    for i in range(len(after)):
        if not np.all(np.abs(vd[i]) < VEL_THRESH):
            continue
        window = (ts >= ts[i]) & (ts <= ts[i] + HOLD_S)
        if np.all(np.abs(vd[window]) < VEL_THRESH):
            t_stop = float(ts[i] - t_sig)
            break
    return t_stop, overshoot, overshoot_norm, bool(np.isfinite(t_stop)), vd_reported_max


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true")
    args = ap.parse_args()
    provenance.assert_clean("P0-ESTOP")

    seeds = [2001, 2002] if args.smoke else list(range(2001, 2051))
    paths = ["command", "safety"]
    exp_id = "P0-ESTOP-smoke" if args.smoke else "P0-ESTOP"

    fields = [
        "trial_id", "seed", "path", "move_s", "sig_s",
        "q_sig_0", "q_sig_1", "q_sig_2", "q_sig_3", "v_sig_max",
        "t_zero_cmd_s", "t_stop_s", "halt_success",
        "overshoot_rad", "overshoot_norm_rad",
        "halt_joint_count", "halt_has_velocities", "estop_active_seen",
        "other_traj_events", "v_reported_max_rad_s",
    ]
    config = {
        "seeds": seeds, "paths": paths, "start_q": START_Q, "goal_q": GOAL_Q,
        "move_s": MOVE_S, "sig_range": SIG_RANGE, "observe_s": OBSERVE_S,
        "vel_thresh_rad_s": VEL_THRESH, "hold_s": HOLD_S,
        "t_zero_cmd_def": "sim time from signal to the path-matching controller-topic message",
        "t_stop_def": "sim time to sustained |dq/dt|<0.05 rad/s for 0.2 s, q differentiated from measured positions",
        "overshoot_def": "max single-joint |q-q_sig| after signal",
        "path_match": "command: 5 joints with velocities; safety: 4 joints (SafetyAgent halt)",
    }

    rclpy.init()
    node = EstopNode()
    try:
        if not node.wait_until(lambda: node.sim_time >= 0.0, 20.0):
            raise RuntimeError("/clock not received")
        if not node.wait_for_joint_states(ALL, 20.0):
            raise RuntimeError("joint states missing")
        if not node.cli_set_state.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/gazebo/set_entity_state not available")

        # Ensure the safety service exists; never fake path B.
        safety_cli = node.create_client(Trigger, "/aria/estop")
        release_cli = node.create_client(Trigger, "/aria/estop/release")
        if not safety_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/aria/estop not available; start safety_agent first")

        rows = []
        t_start = datetime.now(timezone.utc)
        with TrialLog(exp_id, fields, config) as log:
            trial = 0
            for seed in seeds:
                rng = np.random.default_rng(seed)
                goal = np.array(GOAL_Q, dtype=float)
                goal[:4] += rng.uniform(-0.05, 0.05, size=4)
                sig_s = float(rng.uniform(*SIG_RANGE))
                order = list(rng.permutation(paths))

                for path in order:
                    # Return to the start and verify arrival.
                    node.samples.clear()
                    node.traj_events.clear()
                    node.send_trajectory_until(START_Q, duration=3.0, tol=0.03, timeout=8.0)
                    node.spin_for(0.3)

                    node.samples.clear()
                    node.traj_events.clear()
                    node.send_trajectory(tuple(goal), MOVE_S)
                    node.wait_sim(sig_s, timeout=20.0)

                    t_sig_wall = time.monotonic()
                    t_sig = node.sim_at(t_sig_wall)
                    q_sig, v_sig = node.sample_at(t_sig)

                    if path == "command":
                        node.send_trajectory(tuple(q_sig) + (float(node.joint_positions["gripper_joint"]),), 0.1)
                    else:
                        fut = safety_cli.call_async(
                            Trigger.Request()
                        )
                        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
                        if not fut.done():
                            raise RuntimeError("/aria/estop call timed out")

                    node.wait_sim(OBSERVE_S, timeout=20.0)
                    obs_end = node.sim_at(time.monotonic())
                    estop_seen = bool(node.estop_active)

                    # Stop whatever is still moving before the next trial.
                    node.send_trajectory_until(START_Q, duration=2.0, tol=0.05, timeout=6.0)
                    if path == "safety":
                        fut = release_cli.call_async(Trigger.Request())
                        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
                        node.estop_active = False

                    t_stop, over, over_norm, ok, v_rep = stop_metrics(
                        node.samples, t_sig, q_sig, obs_end
                    )

                    events = [
                        e for e in node.traj_events
                        if t_sig_wall - 1e-6 <= e[0] <= t_sig_wall + OBSERVE_S + 0.5
                    ]
                    # Select the message that belongs to the path under test.
                    # Other publishers exist on this topic; matching the
                    # signature prevents a foreign message from being credited
                    # as the halt.
                    if path == "command":
                        own = [e for e in events if e[1] == 5 and e[2]]
                    else:
                        own = [e for e in events if e[1] == 4]
                    ev = own[0] if own else None
                    if ev is not None:
                        t_cmd = node.sim_at(ev[0])
                        t_zero = float(t_cmd - t_sig)
                        n_joints, has_vel = ev[1], ev[2]
                    else:
                        t_zero, n_joints, has_vel = float("nan"), 0, False
                    other_events = len(events) - (1 if ev is not None else 0)

                    row = {
                        "trial_id": trial, "seed": seed, "path": path,
                        "move_s": MOVE_S, "sig_s": sig_s,
                        "q_sig_0": float(q_sig[0]), "q_sig_1": float(q_sig[1]),
                        "q_sig_2": float(q_sig[2]), "q_sig_3": float(q_sig[3]),
                        "v_sig_max": float(np.nanmax(np.abs(v_sig))),
                        "t_zero_cmd_s": t_zero, "t_stop_s": t_stop,
                        "halt_success": int(ok),
                        "overshoot_rad": over, "overshoot_norm_rad": over_norm,
                        "halt_joint_count": n_joints,
                        "halt_has_velocities": int(has_vel),
                        "estop_active_seen": int(estop_seen),
                        "other_traj_events": other_events,
                        "v_reported_max_rad_s": v_rep,
                    }
                    log.log(row)
                    rows.append(row)
                    trial += 1
                    print(
                        f"  trial {trial}: seed={seed} path={path} "
                        f"v_sig={row['v_sig_max']:.3f} t_zero={t_zero*1000:.1f}ms "
                        f"t_stop={t_stop:.3f}s ok={ok} over={over:.4f}rad "
                        f"joints={n_joints} vel={has_vel}",
                        flush=True,
                    )

        summary = {
            "exp_id": exp_id,
            "started_at": t_start.isoformat(),
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "n": len(rows), "seeds": seeds, "commit": provenance.commit(),
        }
        for path in paths:
            cell = [r for r in rows if r["path"] == path]
            ok = [bool(r["halt_success"]) for r in cell]
            p, lo, hi = stats.wilson(sum(ok), len(ok))
            tstop = np.array([r["t_stop_s"] for r in cell if r["halt_success"]])
            tzero = np.array([r["t_zero_cmd_s"] for r in cell if np.isfinite(r["t_zero_cmd_s"])])
            over = np.array([r["overshoot_rad"] for r in cell])
            summary[path] = {
                "n": len(cell),
                "halt_success_rate": p,
                "halt_success_wilson_low": lo,
                "halt_success_wilson_high": hi,
                "t_zero_cmd_median_ms": float(np.median(tzero) * 1000) if tzero.size else None,
                "t_zero_cmd_p95_ms": float(np.quantile(tzero, 0.95) * 1000) if tzero.size else None,
                "t_stop_median_s": float(np.median(tstop)) if tstop.size else None,
                "t_stop_p95_s": float(np.quantile(tstop, 0.95)) if tstop.size else None,
                "overshoot_median_rad": float(np.median(over)),
                "overshoot_p95_rad": float(np.quantile(over, 0.95)),
                "halt_joint_count_median": float(np.median([r["halt_joint_count"] for r in cell])),
                "halt_has_velocities_fraction": float(np.mean([r["halt_has_velocities"] for r in cell])),
                "estop_active_fraction": float(np.mean([r["estop_active_seen"] for r in cell])),
                "other_traj_events_mean": float(np.mean([r["other_traj_events"] for r in cell])),
                "path_match_fraction": float(np.mean([r["halt_joint_count"] > 0 for r in cell])),
                "v_reported_max_median": float(np.nanmedian([r["v_reported_max_rad_s"] for r in cell])),
            }
        # Paired comparison of overshoot within each seed.
        pairs = {}
        for r in rows:
            pairs.setdefault(r["seed"], {})[r["path"]] = r
        paired = [v for v in pairs.values() if "command" in v and "safety" in v]
        if paired:
            a = [v["command"]["overshoot_rad"] for v in paired]
            b = [v["safety"]["overshoot_rad"] for v in paired]
            d, lo, hi = stats.bootstrap_diff_ci(a, b, np.mean)
            summary["overshoot_diff_command_minus_safety"] = {
                "mean": d, "ci_low": lo, "ci_high": hi,
            }
            ka = sum(v["command"]["halt_success"] for v in paired)
            kb = sum(v["safety"]["halt_success"] for v in paired)
            summary["mcnemar"] = {
                "b_command_only": sum(
                    v["command"]["halt_success"] and not v["safety"]["halt_success"] for v in paired
                ),
                "c_safety_only": sum(
                    v["safety"]["halt_success"] and not v["command"]["halt_success"] for v in paired
                ),
                "p": stats.mcnemar_exact(
                    sum(v["command"]["halt_success"] and not v["safety"]["halt_success"] for v in paired),
                    sum(v["safety"]["halt_success"] and not v["command"]["halt_success"] for v in paired),
                ),
                "command_success": int(ka), "safety_success": int(kb),
            }

        RAW = ROOT / "data" / "journal" / "raw"
        (RAW / f"{exp_id}_summary.json").write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
