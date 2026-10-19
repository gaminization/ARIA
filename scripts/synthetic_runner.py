#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
PROJECT ARIA: MASTER SYNTHETIC SUITE RUNNER & DEMONSTRATION ORCHESTRATOR
═══════════════════════════════════════════════════════════════════════════════
Orchestrates fast synthetic execution across both industrial and tester worlds.
Runs without LLM latency for lightning-fast demonstration to third parties.

DEFAULT BEHAVIOR:
  Stands by in IDLE state and awaits user prompts via Dashboard or CLI.
  Does NOT run all tasks automatically or all together.

Usage:
  # 1. Standby Server Mode (Default - waits for prompt):
  python3 scripts/synthetic_runner.py --world tester
  python3 scripts/synthetic_runner.py --world industrial

  # 2. Single Prompt Execution:
  python3 scripts/synthetic_runner.py --prompt "wave"
  python3 scripts/synthetic_runner.py --prompt "stack the jenga tower"
  python3 scripts/synthetic_runner.py --prompt "sort the boxes"

  # 3. Interactive Terminal Shell:
  python3 scripts/synthetic_runner.py --interactive

  # 4. Automated All-Task Regression Run (Audit Only):
  python3 scripts/synthetic_runner.py --world tester --run-all
═══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys
import time
import argparse
import subprocess
from typing import List, Optional

WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WORKSPACE_ROOT not in sys.path:
    sys.path.insert(0, WORKSPACE_ROOT)

ARTIFACT_DIR = os.environ.get(
    "ARIA_SYNTHETIC_ARTIFACT_DIR",
    os.path.join(WORKSPACE_ROOT, "runs", "synthetic_audit"),
)
os.makedirs(ARTIFACT_DIR, exist_ok=True)


def is_sim_running() -> bool:
    try:
        res = subprocess.run(["pgrep", "-f", "gzserver"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return res.returncode == 0
    except Exception:
        return False


def start_sim(world_file: str, gui: bool = False) -> subprocess.Popen:
    env = os.environ.copy()
    env["GAZEBO_IP"] = "127.0.0.1"
    env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:11345"
    env["DISPLAY"] = os.environ.get("DISPLAY", ":1")
    gui_str = "true" if gui else "false"

    setup_cmd = f"source {WORKSPACE_ROOT}/install/setup.bash"
    launch_str = f"{setup_cmd} && ros2 launch arm_bringup sim.launch.py world:={world_file} gui:={gui_str}"

    print(f"  Starting background simulation for {world_file} (GUI={gui_str})...")
    proc = subprocess.Popen(
        ["bash", "-c", launch_str],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    time.sleep(12)
    return proc


def stop_sim(proc: Optional[subprocess.Popen]):
    if proc:
        print("  Stopping simulation subprocess group...")
        try:
            os.killpg(os.getpgid(proc.pid), 15)
            time.sleep(1.0)
            os.killpg(os.getpgid(proc.pid), 9)
        except Exception:
            pass
    subprocess.run(["pkill", "-9", "-f", "gzserver"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "gzclient"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "sim.launch.py"], stderr=subprocess.DEVNULL)


def run_server_mode(world: str, interactive: bool = False):
    """Launches synthetic_server.py in standby mode awaiting user prompts."""
    script = os.path.join(WORKSPACE_ROOT, "scripts", "synthetic_server.py")
    env = os.environ.copy()
    env["ARIA_SYNTHETIC_ARTIFACT_DIR"] = ARTIFACT_DIR

    cmd = [sys.executable, script, "-w", world]
    if interactive:
        cmd.append("--interactive")

    print(f"  Launching Synthetic Command Server in STANDBY mode (world: {world})...")
    ret = subprocess.call(cmd, env=env)
    return ret == 0


def run_prompt(prompt_text: str):
    """Dispatches a single prompt via synthetic_prompt_cli.py."""
    script = os.path.join(WORKSPACE_ROOT, "scripts", "synthetic_prompt_cli.py")
    cmd = [sys.executable, script, prompt_text]
    return subprocess.call(cmd) == 0


def run_all_maneuvers(world: str) -> bool:
    """Executes all maneuvers sequentially (used only when --run-all is explicitly passed)."""
    env = os.environ.copy()
    env["ARIA_SYNTHETIC_ARTIFACT_DIR"] = ARTIFACT_DIR
    all_ok = True

    if world in ("tester", "all"):
        script = os.path.join(WORKSPACE_ROOT, "scripts", "synthetic_tester_suite.py")
        cmd = f"source {WORKSPACE_ROOT}/install/setup.bash && python3 {script}"
        t0 = time.time()
        ret = subprocess.call(["bash", "-c", cmd], env=env)
        print(f"  Tester Suite finished in {time.time() - t0:.1f}s (Return code: {ret})")
        all_ok = all_ok and (ret == 0)

    if world in ("industrial", "all"):
        script = os.path.join(WORKSPACE_ROOT, "scripts", "synthetic_industrial_suite.py")
        cmd = f"source {WORKSPACE_ROOT}/install/setup.bash && python3 {script}"
        t0 = time.time()
        ret = subprocess.call(["bash", "-c", cmd], env=env)
        print(f"  Industrial Suite finished in {time.time() - t0:.1f}s (Return code: {ret})")
        all_ok = all_ok and (ret == 0)

    return all_ok


def main():
    parser = argparse.ArgumentParser(description="ARIA Master Synthetic Demonstration Runner")
    parser.add_argument("-w", "--world", choices=["tester", "industrial", "all", "auto"], default="auto",
                        help="World to execute synthetic maneuvers in (default: auto)")
    parser.add_argument("-g", "--gui", action="store_true", help="Launch Gazebo 3D GUI client")
    parser.add_argument("--standalone", action="store_true", help="Force launch dedicated simulation")
    parser.add_argument("--delay", type=float, default=0.0, help="Optional delay before execution (seconds)")
    parser.add_argument("-p", "--prompt", type=str, default="", help="Execute a single natural language prompt then exit")
    parser.add_argument("-i", "--interactive", action="store_true", help="Launch interactive prompt terminal console")
    parser.add_argument("--run-all", action="store_true", help="Run all maneuvers sequentially (audit mode only)")
    args = parser.parse_args()

    if args.delay > 0:
        print(f"  Waiting {args.delay:.1f}s before starting synthetic runtime...")
        time.sleep(args.delay)

    # 1. Direct prompt dispatch
    if args.prompt:
        print(f"  Dispatching single user prompt: \"{args.prompt}\"")
        ok = run_prompt(args.prompt)
        sys.exit(0 if ok else 1)

    print("╔═══════════════════════════════════════════════════════════════════════════╗")
    print("║   🤖 PROJECT ARIA: MASTER SYNTHETIC RUNTIME                              ║")
    print("║   High-Efficiency Demonstrator Suite for 3rd-Party Fast Evaluation        ║")
    print("╚═══════════════════════════════════════════════════════════════════════════╝")
    print(f"  Target World:      {args.world}")
    print(f"  Gazebo GUI:        {args.gui}")
    print(f"  Run All Mode:      {args.run_all} (Default is STANDBY on prompt)")
    print(f"  Artifact Storage:  {ARTIFACT_DIR}")
    print("─────────────────────────────────────────────────────────────────────────────")

    sim_running = is_sim_running()
    sim_proc: Optional[subprocess.Popen] = None

    try:
        # Start standalone simulation if requested and not running
        if args.standalone or not sim_running:
            world_file = "aria_industrial_workcell.world" if args.world == "industrial" else "aria_tester_workspace.world"
            stop_sim(None)
            sim_proc = start_sim(world_file, gui=args.gui)

        if args.run_all:
            # Audit mode: run all tasks
            success = run_all_maneuvers(args.world)
            sys.exit(0 if success else 1)
        else:
            # Standard mode: Run standby server waiting for user prompt
            run_server_mode(args.world, interactive=args.interactive)

    finally:
        if sim_proc:
            stop_sim(sim_proc)


if __name__ == "__main__":
    main()
