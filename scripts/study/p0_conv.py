#!/usr/bin/env python3
"""P0-CONV: does the commanded belt speed equal the realized part speed?

The conveyor plugin transports workpieces by writing their linear velocity
directly (``SetLinearVel``/``AddForce``), while the visual belt is a prismatic
joint.  This experiment therefore measures three separate things instead of
assuming any of them:

1. the realized transport speed of ``workpiece_01`` from ``/gazebo/link_states``
   (ground truth, not the plugin's self-report),
2. the visual belt link and roller speeds from the same topic (the animation
   path),
3. the world-frame motion of the arm's wrist link while the arm holds a fixed
   joint configuration (TCP deviation during conveyor operation).

Simulation time comes from ``/clock`` because the world runs below real time.

Usage:
    python3 scripts/study/p0_conv.py --smoke
    python3 scripts/study/p0_conv.py
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import rclpy  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402
from gazebo_msgs.msg import LinkStates  # noqa: E402

from aria_study import provenance, stats  # noqa: E402
from aria_study.ros import StudyNode  # noqa: E402
from aria_study.trial_log import TrialLog  # noqa: E402

SPEEDS = (0.04, 0.08, 0.12)
PART_MODEL = "workpiece_01"
PART = "workpiece_01::link"
WRIST = "aria_arm::wrist_link"
BELT = "conveyor_system::belt_visual_surface"
HEAD = "conveyor_system::head_roller"
TAIL = "conveyor_system::tail_roller"
PART_RESET = (0.20, 0.75, 0.655)
HOLD_Q = (0.0, 1.0, -0.6, 1.55, 0.855)
ACCEL_S = 0.30
WINDOW_TICKS = 10  # /clock publishes at 5 Hz, so 10 ticks = 2.0 s sim


class Sampler(StudyNode):
    def __init__(self):
        super().__init__("aria_p0_conv")
        self.sim_time = -1.0
        self.clock_ticks = 0
        self.speed_reported = float("nan")
        self.part_samples: List[Tuple[float, float, float, float]] = []
        self.wrist_samples: List[Tuple[float, np.ndarray]] = []
        self.latest_belt = np.zeros(3)
        self.latest_head = np.zeros(3)
        self.latest_tail = np.zeros(3)
        self.create_subscription(
            Clock, "/clock", self._clock_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Float64, "/aria/conveyor/speed", self._speed_cb, 10
        )
        self.create_subscription(LinkStates, "/gazebo/link_states", self._links_cb, 10)

    def _clock_cb(self, msg: Clock):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if t != self.sim_time:
            self.clock_ticks += 1
        self.sim_time = t

    def _speed_cb(self, msg: Float64):
        self.speed_reported = float(msg.data)

    def _links_cb(self, msg: LinkStates):
        t = self.sim_time
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            if name == PART:
                self.part_samples.append(
                    (
                        t,
                        float(twist.linear.x),
                        float(twist.linear.y),
                        float(pose.position.y),
                    )
                )
            elif name == WRIST:
                self.wrist_samples.append(
                    (
                        t,
                        np.array(
                            [pose.position.x, pose.position.y, pose.position.z],
                            dtype=float,
                        ),
                    )
                )
            elif name == BELT:
                self.latest_belt = np.array(
                    [twist.linear.x, twist.linear.y, twist.linear.z]
                )
            elif name == HEAD:
                self.latest_head = np.array(
                    [twist.linear.x, twist.linear.y, twist.linear.z]
                )
            elif name == TAIL:
                self.latest_tail = np.array(
                    [twist.linear.x, twist.linear.y, twist.linear.z]
                )

    def wait_clock(self, timeout: float = 10.0) -> bool:
        return self.wait_until(lambda: self.sim_time >= 0.0, timeout)

    def wait_ticks(self, n: int, timeout: float = 30.0) -> bool:
        start = self.clock_ticks
        return self.wait_until(lambda: self.clock_ticks >= start + n, timeout)


def set_speed(node: Sampler, speed: float) -> None:
    # Publish a few times: a single sample can be dropped while the plugin is
    # still loading, and the plugin has no acknowledgement on this topic.
    for _ in range(3):
        node.send_conveyor_speed(speed)
        node.spin_for(0.03)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true")
    args = ap.parse_args()

    provenance.assert_clean("P0-CONV")

    seeds = [1001, 1002] if args.smoke else list(range(1001, 1041))
    speeds = [0.04] if args.smoke else list(SPEEDS)
    exp_id = "P0-CONV-smoke" if args.smoke else "P0-CONV"

    fields = [
        "trial_id", "seed", "speed_cmd", "speed_reported",
        "start_x", "start_y", "start_z",
        "window_start_y", "window_end_y", "window_sim_s", "window_wall_s",
        "realized_speed", "displacement_speed", "speed_error", "speed_rel_error",
        "n_velocity_samples",
        "belt_link_speed", "head_roller_speed", "tail_roller_speed",
        "tcp_dev_mm", "arm_joint_drift_max", "rtf",
    ]
    config = {
        "seeds": seeds,
        "speeds": speeds,
        "part": PART,
        "part_reset": PART_RESET,
        "hold_q": HOLD_Q,
        "accel_s": ACCEL_S,
        "window_ticks": WINDOW_TICKS,
        "clock_rate_hz": 5.0,
        "speed_source": "commanded Twist.linear.x",
        "realized_source": "/gazebo/link_states workpiece twist (mean over window)",
    }

    rclpy.init()
    node = Sampler()
    try:
        if not node.wait_clock(20.0):
            raise RuntimeError("/clock not received; is the simulation running?")
        if not node.wait_for_links([PART, WRIST, BELT, HEAD, TAIL], 20.0):
            raise RuntimeError("required links missing from /gazebo/link_states")
        if not node.wait_for_joint_states(
            ["waist_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint",
             "gripper_joint"], 20.0
        ):
            raise RuntimeError("joint states missing")

        # Hold a fixed arm configuration for the whole run.
        set_speed(node, 0.0)
        node.send_trajectory(HOLD_Q, duration=2.0)
        node.spin_for(3.0)
        q_hold = np.array(HOLD_Q[:4], dtype=float)
        hold_err = float(
            np.max(
                np.abs(
                    node.q(
                        ["waist_joint", "shoulder_joint", "elbow_joint",
                         "wrist_pitch_joint"]
                    )
                    - q_hold
                )
            )
        )
        print(f"hold config error: {hold_err:.4f} rad", flush=True)

        rows = []
        t_start = datetime.now(timezone.utc)
        with TrialLog(exp_id, fields, config) as log:
            trial = 0
            for seed in seeds:
                rng = np.random.default_rng(seed)
                for speed in speeds:
                    # Quiesce, then place the part at the infeed end.
                    set_speed(node, 0.0)
                    node.spin_for(0.30)
                    x = PART_RESET[0] + float(rng.uniform(-0.008, 0.008))
                    y0 = PART_RESET[1] + float(rng.uniform(-0.03, 0.03))
                    ok = node.set_model_pose(PART_MODEL, (x, y0, PART_RESET[2]))
                    if not ok:
                        raise RuntimeError("set_entity_state failed for workpiece_01")
                    node.spin_for(0.20)

                    # Command the speed and let the plugin reach steady state.
                    set_speed(node, speed)
                    node.wait_ticks(2, timeout=10.0)

                    # Measure over a fixed number of sim-clock ticks.
                    node.part_samples.clear()
                    node.wrist_samples.clear()
                    wall0 = time.monotonic()
                    sim0 = node.sim_time
                    node.wait_ticks(WINDOW_TICKS, timeout=30.0)
                    wall1 = time.monotonic()
                    sim1 = node.sim_time
                    q_now = node.q(
                        ["waist_joint", "shoulder_joint", "elbow_joint",
                         "wrist_pitch_joint"]
                    )

                    set_speed(node, 0.0)

                    win = [s for s in node.part_samples if s[0] >= sim0 - 1e-9]
                    if len(win) < 5:
                        raise RuntimeError(
                            f"only {len(win)} part samples in window"
                        )
                    vy = np.array([s[2] for s in win])
                    ys = np.array([s[3] for s in win])
                    ts = np.array([s[0] for s in win])
                    realized = float(-np.mean(vy))
                    dt = float(sim1 - sim0)
                    displacement = float((ys[0] - ys[-1]) / dt) if dt > 0 else float("nan")

                    wrist = [w for w in node.wrist_samples if w[0] >= sim0 - 1e-9]
                    if len(wrist) >= 2:
                        P = np.array([w[1] for w in wrist])
                        tcp_dev = float(
                            np.max(np.linalg.norm(P - P.mean(axis=0), axis=1))
                        ) * 1000.0
                    else:
                        tcp_dev = float("nan")

                    row = {
                        "trial_id": trial,
                        "seed": seed,
                        "speed_cmd": speed,
                        "speed_reported": node.speed_reported,
                        "start_x": x,
                        "start_y": y0,
                        "start_z": PART_RESET[2],
                        "window_start_y": float(ys[0]),
                        "window_end_y": float(ys[-1]),
                        "window_sim_s": dt,
                        "window_wall_s": wall1 - wall0,
                        "realized_speed": realized,
                        "displacement_speed": displacement,
                        "speed_error": realized - speed,
                        "speed_rel_error": (realized - speed) / speed,
                        "n_velocity_samples": int(len(win)),
                        "belt_link_speed": float(np.linalg.norm(node.latest_belt)),
                        "head_roller_speed": float(np.linalg.norm(node.latest_head)),
                        "tail_roller_speed": float(np.linalg.norm(node.latest_tail)),
                        "tcp_dev_mm": tcp_dev,
                        "arm_joint_drift_max": float(
                            np.max(np.abs(q_now - q_hold))
                        ),
                        "rtf": float(dt / (wall1 - wall0)) if wall1 > wall0 else float("nan"),
                    }
                    log.log(row)
                    rows.append(row)
                    trial += 1
                    print(
                        f"  trial {trial}: seed={seed} cmd={speed:.2f} "
                        f"realized={realized:.4f} err={row['speed_error']:+.4f} "
                        f"tcp_dev={tcp_dev:.3f}mm",
                        flush=True,
                    )

        summary = {
            "exp_id": exp_id,
            "started_at": t_start.isoformat(),
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "n": len(rows),
            "seeds": seeds,
            "speeds": speeds,
            "commit": provenance.commit(),
        }
        for speed in speeds:
            cell = [r for r in rows if abs(r["speed_cmd"] - speed) < 1e-12]
            real = np.array([r["realized_speed"] for r in cell])
            err = np.array([r["speed_error"] for r in cell])
            rel = np.array([r["speed_rel_error"] for r in cell])
            mean_real = float(real.mean())
            ci = stats.bootstrap_ci(real, np.mean) if len(real) > 1 else (mean_real, mean_real, mean_real)
            summary[f"speed_{speed:.2f}"] = {
                "n": len(cell),
                "mean_realized_speed": mean_real,
                "mean_realized_ci_low": float(ci[1]),
                "mean_realized_ci_high": float(ci[2]),
                "mean_speed_error": float(err.mean()),
                "mean_rel_error": float(rel.mean()),
                "median_rel_error": float(np.median(rel)),
                "max_abs_rel_error": float(np.max(np.abs(rel))),
                "belt_link_speed_mean": float(np.mean([r["belt_link_speed"] for r in cell])),
                "head_roller_speed_mean": float(np.mean([r["head_roller_speed"] for r in cell])),
                "tail_roller_speed_mean": float(np.mean([r["tail_roller_speed"] for r in cell])),
                "tcp_dev_mm_mean": float(np.nanmean([r["tcp_dev_mm"] for r in cell])),
                "tcp_dev_mm_max": float(np.nanmax([r["tcp_dev_mm"] for r in cell])),
            }
        summary["rtf_mean"] = float(np.mean([r["rtf"] for r in rows]))
        summary["hold_config_error_rad"] = hold_err
        summary["arm_joint_drift_max"] = float(
            max(r["arm_joint_drift_max"] for r in rows)
        )

        RAW = ROOT / "data" / "journal" / "raw"
        (RAW / f"{exp_id}_summary.json").write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
