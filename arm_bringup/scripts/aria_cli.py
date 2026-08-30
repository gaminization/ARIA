#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA CLI — Unified command-line interface
#
# Usage:
#   python3 aria_cli.py <category> <command> [args]
#
# Install as system command:
#   chmod +x aria_cli.py
#   sudo ln -s $(pwd)/aria_cli.py /usr/local/bin/aria
#
# Then use: aria sim start, aria teach start, etc.
# ═══════════════════════════════════════════════════════════════
import argparse
import os
import sys
import subprocess
import time

# ── Constants ──────────────────────────────────────────────────
ARIA_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(ARIA_DIR, "..", ".."))
LAUNCH_DIR = os.path.join(PROJECT_DIR, "arm_bringup", "launch")
SCRIPT_DIR = os.path.join(PROJECT_DIR, "arm_bringup", "scripts")


# ═══════════════════════════════════════════════════════════════
# Terminal helpers
# ═══════════════════════════════════════════════════════════════
class C:
    RESET  = "\033[0m"
    BOLD   = "\033[1m"
    RED    = "\033[91m"
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    CYAN   = "\033[96m"
    DIM    = "\033[2m"


def banner():
    print(f"""
{C.CYAN}╔═══════════════════════════════════════════╗
║   🤖  ARIA — Robotic Arm AI System       ║
║   Adaptive Robotic Intelligence Arch.     ║
╚═══════════════════════════════════════════╝{C.RESET}
""")


def run_ros2(cmd_str, background=False):
    """Execute a ROS2 command, sourcing the workspace first."""
    full_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {PROJECT_DIR}/install/setup.bash 2>/dev/null; "
        f"{cmd_str}"
    )
    if background:
        subprocess.Popen(
            full_cmd, shell=True, executable="/bin/bash",
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
    else:
        return subprocess.run(
            full_cmd, shell=True, executable="/bin/bash"
        )


def run_service(service_name, srv_type="std_srvs/srv/Trigger", data="{}"):
    """Call a ROS2 service."""
    return run_ros2(
        f'ros2 service call {service_name} {srv_type} "{data}"'
    )


# ═══════════════════════════════════════════════════════════════
# SIMULATION COMMANDS
# ═══════════════════════════════════════════════════════════════
def cmd_sim_start(args):
    """Launch full Gazebo simulation."""
    print(f"{C.GREEN}Launching ARIA simulation...{C.RESET}")
    run_ros2("ros2 launch arm_bringup sim.launch.py")


def cmd_sim_manual(args):
    """Launch simulation with manual control GUI."""
    print(f"{C.GREEN}Launching simulation with manual control...{C.RESET}")
    run_ros2("ros2 launch arm_bringup manual_control.launch.py")


def cmd_sim_headless(args):
    """Launch simulation without GUI (testing)."""
    print(f"{C.GREEN}Launching headless simulation...{C.RESET}")
    run_ros2(
        "ros2 launch arm_bringup sim.launch.py "
        "use_rviz:=false use_gz_gui:=false"
    )


def cmd_sim_tester(args):
    """Launch tester workspace (Bullet3 benchmarks + IAI table + RGB-D)."""
    print(f"{C.GREEN}Launching ARIA Ultimate Tester Workspace...{C.RESET}")
    run_ros2("ros2 launch arm_bringup tester_sim.launch.py")


def cmd_sim_full(args):
    """Launch full system (sim + perception + agents)."""
    print(f"{C.GREEN}Launching full ARIA system...{C.RESET}")
    run_ros2("ros2 launch arm_bringup aria_full.launch.py")


# ═══════════════════════════════════════════════════════════════
# HARDWARE COMMANDS
# ═══════════════════════════════════════════════════════════════
def cmd_hardware_wizard(args):
    """Run the first-time hardware bringup wizard."""
    print(f"{C.GREEN}Starting hardware bringup wizard...{C.RESET}")
    subprocess.run([
        sys.executable,
        os.path.join(SCRIPT_DIR, "hardware_bringup_wizard.py")
    ])


def cmd_hardware_start(args):
    """Launch with real hardware."""
    port = getattr(args, "port", "/dev/ttyUSB0")
    print(f"{C.GREEN}Launching ARIA hardware on {port}...{C.RESET}")
    run_ros2(f"ros2 launch arm_bringup hardware.launch.py port:={port}")


def cmd_hardware_calibrate(args):
    """Run full calibration sequence."""
    print(f"{C.GREEN}Running calibration...{C.RESET}")
    run_service("/aria/calibrate_offsets")


def cmd_hardware_verify_sync(args):
    """Check sim-real alignment."""
    print(f"{C.GREEN}Verifying sim-real sync...{C.RESET}")
    run_service("/aria/verify_sync")


# ═══════════════════════════════════════════════════════════════
# TEACH MODE COMMANDS
# ═══════════════════════════════════════════════════════════════
def cmd_teach_start(args):
    """Enter teach mode (relax servos, start recording)."""
    print(f"{C.YELLOW}Starting teach mode — arm will RELAX...{C.RESET}")
    run_service("/aria/teach/start")


def cmd_teach_waypoint(args):
    """Record current position as waypoint."""
    run_service("/aria/teach/waypoint")


def cmd_teach_stop(args):
    """Stop teach mode, re-engage servos, save trajectory."""
    print(f"{C.GREEN}Stopping teach mode...{C.RESET}")
    run_service("/aria/teach/stop")


def cmd_teach_replay(args):
    """Replay last recorded trajectory."""
    print(f"{C.GREEN}Replaying trajectory...{C.RESET}")
    run_service("/aria/teach/replay")


def cmd_teach_save(args):
    """Save trajectory as named skill."""
    print(f"{C.GREEN}Saving trajectory...{C.RESET}")
    run_service("/aria/teach/save")


# ═══════════════════════════════════════════════════════════════
# INTELLIGENCE COMMANDS
# ═══════════════════════════════════════════════════════════════
def cmd_command(args):
    """Send a natural language command to ARIA."""
    instruction = " ".join(args.instruction)
    print(f'{C.CYAN}Sending command: "{instruction}"{C.RESET}')
    run_service(
        "/aria/command",
        "arm_interfaces/srv/SendCommand",
        f'{{command: "{instruction}"}}'
    )


def cmd_approve(args):
    """Approve pending action."""
    print(f"{C.GREEN}Approving pending action...{C.RESET}")
    run_service("/aria/approve")


def cmd_reject(args):
    """Reject pending action."""
    print(f"{C.RED}Rejecting pending action...{C.RESET}")
    run_service("/aria/reject")


def cmd_estop(args):
    """Emergency stop — all motion halts."""
    print(f"{C.RED}{C.BOLD}🛑 EMERGENCY STOP{C.RESET}")
    run_service("/aria/estop")


# ═══════════════════════════════════════════════════════════════
# LEARN MODE COMMANDS
# ═══════════════════════════════════════════════════════════════
def cmd_learn_start(args):
    """Start learning mode recording."""
    print(f"{C.GREEN}Starting learn mode...{C.RESET}")
    run_service("/aria/learn/start")


def cmd_learn_stop(args):
    """Stop learning mode and save dataset."""
    print(f"{C.GREEN}Stopping learn mode...{C.RESET}")
    run_service("/aria/learn/stop")


# ═══════════════════════════════════════════════════════════════
# SYNC COMMANDS
# ═══════════════════════════════════════════════════════════════
def cmd_sync_sim_to_real(args):
    """Set sync: simulation controls real hardware."""
    print(f"{C.GREEN}Sync mode: SIM → REAL{C.RESET}")
    run_service("/aria/sync_mode")


def cmd_sync_real_to_sim(args):
    """Set sync: real hardware updates simulation."""
    print(f"{C.GREEN}Sync mode: REAL → SIM{C.RESET}")
    run_service("/aria/sync_mode")


def cmd_sync_off(args):
    """Disable sync."""
    print(f"{C.YELLOW}Sync DISABLED{C.RESET}")
    run_service("/aria/sync_mode")


# ═══════════════════════════════════════════════════════════════
# STATUS & BENCHMARK COMMANDS
# ═══════════════════════════════════════════════════════════════
def cmd_status(args):
    """Show system health and node status."""
    print(f"{C.CYAN}ARIA System Status{C.RESET}")
    print("─" * 40)
    run_ros2("ros2 node list 2>/dev/null | grep aria || echo '  No ARIA nodes running'")
    print()
    run_ros2("ros2 topic list 2>/dev/null | grep aria | head -20")


def cmd_benchmark_ik(args):
    """Run IK solver benchmark."""
    print(f"{C.GREEN}Running IK benchmark...{C.RESET}")
    run_ros2("ros2 run arm_ik ik_benchmark_node")


def cmd_benchmark_perception(args):
    """Run perception accuracy test."""
    print(f"{C.GREEN}Running perception benchmark...{C.RESET}")
    run_ros2("ros2 run arm_vision detection_benchmark")


def cmd_benchmark_depth(args):
    """Run depth estimation test."""
    print(f"{C.GREEN}Running depth benchmark...{C.RESET}")
    run_ros2("ros2 run arm_vision depth_benchmark")


# ═══════════════════════════════════════════════════════════════
# ARGUMENT PARSER
# ═══════════════════════════════════════════════════════════════
def build_parser():
    parser = argparse.ArgumentParser(
        prog="aria",
        description="ARIA — Adaptive Robotic Intelligence Architecture CLI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  aria sim start                   Launch Gazebo simulation
  aria hardware wizard             Run first-time hardware setup
  aria teach start                 Enter teach-by-demonstration mode
  aria command "Pick up the cube"  Send natural language command
  aria estop                       Emergency stop
  aria status                      Show system health
        """,
    )

    sub = parser.add_subparsers(dest="category", help="Command category")

    # ── sim ────────────────────────────────────────────────
    sim = sub.add_parser("sim", help="Simulation commands")
    sim_sub = sim.add_subparsers(dest="action")
    sim_sub.add_parser("start", help="Launch default simulation").set_defaults(func=cmd_sim_start)
    sim_sub.add_parser("tester", help="Launch tester world (Bullet3 + IAI table + RGB-D)").set_defaults(func=cmd_sim_tester)
    sim_sub.add_parser("manual", help="Launch with manual control").set_defaults(func=cmd_sim_manual)
    sim_sub.add_parser("headless", help="Launch without GUI").set_defaults(func=cmd_sim_headless)
    sim_sub.add_parser("full", help="Launch full system").set_defaults(func=cmd_sim_full)

    # ── hardware ───────────────────────────────────────────
    hw = sub.add_parser("hardware", help="Hardware commands")
    hw_sub = hw.add_subparsers(dest="action")
    hw_sub.add_parser("wizard", help="First-time setup").set_defaults(func=cmd_hardware_wizard)
    hw_start = hw_sub.add_parser("start", help="Launch hardware")
    hw_start.add_argument("--port", default="/dev/ttyUSB0")
    hw_start.set_defaults(func=cmd_hardware_start)
    hw_sub.add_parser("calibrate", help="Run calibration").set_defaults(func=cmd_hardware_calibrate)
    hw_sub.add_parser("verify-sync", help="Check alignment").set_defaults(func=cmd_hardware_verify_sync)

    # ── teach ──────────────────────────────────────────────
    teach = sub.add_parser("teach", help="Teach-by-demonstration")
    teach_sub = teach.add_subparsers(dest="action")
    teach_sub.add_parser("start", help="Enter teach mode").set_defaults(func=cmd_teach_start)
    teach_sub.add_parser("waypoint", help="Record waypoint").set_defaults(func=cmd_teach_waypoint)
    teach_sub.add_parser("stop", help="Stop and save").set_defaults(func=cmd_teach_stop)
    teach_sub.add_parser("replay", help="Replay trajectory").set_defaults(func=cmd_teach_replay)
    teach_sub.add_parser("save", help="Save as skill").set_defaults(func=cmd_teach_save)

    # ── command ────────────────────────────────────────────
    cmd = sub.add_parser("command", help="Send natural language command")
    cmd.add_argument("instruction", nargs="+", help="Command text")
    cmd.set_defaults(func=cmd_command)

    # ── Direct shortcuts ───────────────────────────────────
    sub.add_parser("approve", help="Approve pending action").set_defaults(func=cmd_approve)
    sub.add_parser("reject", help="Reject pending action").set_defaults(func=cmd_reject)
    sub.add_parser("estop", help="Emergency stop").set_defaults(func=cmd_estop)
    sub.add_parser("status", help="Show system health").set_defaults(func=cmd_status)

    # ── learn ──────────────────────────────────────────────
    learn = sub.add_parser("learn", help="Learning mode")
    learn_sub = learn.add_subparsers(dest="action")
    learn_sub.add_parser("start", help="Start recording").set_defaults(func=cmd_learn_start)
    learn_sub.add_parser("stop", help="Stop and save").set_defaults(func=cmd_learn_stop)

    # ── sync ───────────────────────────────────────────────
    sync = sub.add_parser("sync", help="Sync mode control")
    sync_sub = sync.add_subparsers(dest="action")
    sync_sub.add_parser("sim-to-real", help="Sim controls real").set_defaults(func=cmd_sync_sim_to_real)
    sync_sub.add_parser("real-to-sim", help="Real updates sim").set_defaults(func=cmd_sync_real_to_sim)
    sync_sub.add_parser("off", help="Disable sync").set_defaults(func=cmd_sync_off)

    # ── benchmark ──────────────────────────────────────────
    bench = sub.add_parser("benchmark", help="Run benchmarks")
    bench_sub = bench.add_subparsers(dest="action")
    bench_sub.add_parser("ik", help="IK solver benchmark").set_defaults(func=cmd_benchmark_ik)
    bench_sub.add_parser("perception", help="Perception test").set_defaults(func=cmd_benchmark_perception)
    bench_sub.add_parser("depth", help="Depth estimation test").set_defaults(func=cmd_benchmark_depth)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    if not hasattr(args, "func"):
        if args.category:
            # Category given but no subcommand
            parser.parse_args([args.category, "--help"])
        else:
            banner()
            parser.print_help()
        return

    args.func(args)


if __name__ == "__main__":
    main()
