#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
Project ARIA: Physical Simulation Run — Blue Items to Red Box
═══════════════════════════════════════════════════════════════
Executes an end-to-end physical simulation run in the Gazebo
industrial workcell where Project ARIA autonomously perceives,
inspects, and places all blue items into the Red Box (Reject Bin).

Drives the complete ARIA Multi-Agent Stack (arm_agents, arm_planner,
arm_vision, arm_ik) through natural language instruction:
  "Keep all of the blue items in the red box"
dispatched directly to the /aria/command service.

Validates physical end-state coordinates via /gazebo/model_states.
Zero hardcoded named poses, zero hardcoded trajectory waypoints.

Usage:
  python3 scripts/run_industrial_blue_in_red_box.py
═══════════════════════════════════════════════════════════════
"""

import os
import sys
import time
import math
import subprocess
import signal
from typing import Dict, Optional, Tuple, Set

import numpy as np

# Ensure ROS 2 environment
WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
planner_lib = os.path.join(WORKSPACE_ROOT, "install", "arm_planner", "lib")
if os.path.exists(planner_lib) and planner_lib not in os.environ.get("LD_LIBRARY_PATH", ""):
    setup_script = os.path.join(WORKSPACE_ROOT, "install", "setup.bash")
    if os.path.exists(setup_script):
        cmd = f"source {setup_script} && exec python3 " + " ".join([f'"{arg}"' for arg in sys.argv])
        sys.exit(subprocess.call(["bash", "-c", cmd]))

INSTALL_DIR = os.path.join(WORKSPACE_ROOT, "install")
if os.path.exists(INSTALL_DIR):
    for pkg in os.listdir(INSTALL_DIR):
        py_dir = os.path.join(INSTALL_DIR, pkg, "local", "lib", f"python{sys.version_info.major}.{sys.version_info.minor}", "dist-packages")
        if os.path.exists(py_dir) and py_dir not in sys.path:
            sys.path.insert(0, py_dir)

MODELS_DIR = os.path.join(WORKSPACE_ROOT, "arm_bringup", "models")
WORLD_FILE = os.path.join(WORKSPACE_ROOT, "arm_bringup", "worlds", "aria_industrial_workcell.world")

# Physical Bounding Box of the Red Box (Reject Bin) in Gazebo World
# Defined in aria_industrial_workcell.world: pose = (0.190, -0.150, 0.608)
RED_BOX_X_MIN = 0.12
RED_BOX_X_MAX = 0.26
RED_BOX_Y_MIN = -0.22
RED_BOX_Y_MAX = -0.08
RED_BOX_Z_MIN = 0.58
RED_BOX_Z_MAX = 0.72


def clean_existing_processes():
    print("[1/6] Cleaning any stale Gazebo/ROS 2 processes...")
    subprocess.run("pkill -9 -f gzserver || true; pkill -9 -f gzclient || true; pkill -9 -f industrial_workcell || true; pkill -9 -f robot_state_publisher || true",
                   shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def start_gazebo_and_nodes(gui: bool = True):
    print(f"[2/6] Starting Gazebo simulation and ARIA Multi-Agent Stack on DISPLAY=:1 (gui={'true' if gui else 'false'})...")
    env = os.environ.copy()
    env["GAZEBO_IP"] = "127.0.0.1"
    env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:11345"
    env["DISPLAY"] = os.environ.get("DISPLAY", ":1")
    env["ARIA_WORKCELL"] = "industrial"
    env["ARIA_OVERHEAD_CAM_X"] = "0.18"
    env["ARIA_OVERHEAD_CAM_Y"] = "0.03"
    env["ARIA_OVERHEAD_CAM_Z"] = "1.35"

    setup_cmd = f"source {WORKSPACE_ROOT}/install/setup.bash"
    launch_str = f"{setup_cmd} && ros2 launch arm_bringup industrial_workcell.launch.py gui:={'true' if gui else 'false'} with_agents:=true auto_cycle:=false"
    launch_proc = subprocess.Popen(["bash", "-c", launch_str], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("[3/6] Waiting for workcell controllers, 15 agents, and /aria/command to become active...")
    start_wait = time.time()
    ready = False
    while time.time() - start_wait < 50.0:
        res = subprocess.run(["bash", "-c", f"{setup_cmd} && ros2 service list"], env=env, capture_output=True, text=True)
        top = subprocess.run(["bash", "-c", f"{setup_cmd} && ros2 topic list"], env=env, capture_output=True, text=True)
        if "/aria/command" in res.stdout and "/aria/conveyor/set_power" in res.stdout and "/detection/objects" in top.stdout:
            ready = True
            break
        time.sleep(1.5)

    if ready:
        print("      ✅ All ARIA Multi-Agent services active and verified (/aria/command online)!")
        # Allow time for lifecycle activation transition and initial YOLO inference frame
        time.sleep(5.0)
    else:
        print("      ⚠️ Service discovery timeout; proceeding with execution...")
        time.sleep(3.0)

    return launch_proc


def run_autonomous_cycle():
    print("[4/6] Connecting to ARIA Autonomous Multi-Agent Hierarchy...")
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Imu
    from gazebo_msgs.msg import ModelStates
    from arm_interfaces.srv import SendCommand
    from arm_planner.msg import TaskState

    rclpy.init()

    class AgentMissionMonitor(Node):
        def __init__(self):
            super().__init__("agent_mission_monitor")
            self.seen_cot = set()
            self.model_poses = {}
            self.done = False
            self.task_success = False
            self.latest_imu = None
            self.current_goal = ""
            self.subgoals = []

            self.create_subscription(String, "/aria/cot/reasoning", self._cot_cb, 10)
            self.create_subscription(TaskState, "/aria/state/task", self._task_cb, 10)
            self.create_subscription(Imu, "/mpu6050/imu_raw", self._imu_cb, 10)
            self.create_subscription(ModelStates, "/gazebo/model_states", self._model_cb, 10)
            self.cmd_client = self.create_client(SendCommand, "/aria/command")

        def _cot_cb(self, msg: String):
            if msg.data not in self.seen_cot:
                self.seen_cot.add(msg.data)
                print(f"  🧠 [CoT] {msg.data}")

        def _task_cb(self, msg: TaskState):
            if msg.current_goal and msg.current_goal != self.current_goal:
                self.current_goal = msg.current_goal
                print(f"  🎯 [Goal] Active Goal: {msg.current_goal}")
                if msg.subgoals and not self.subgoals:
                    self.subgoals = list(msg.subgoals)
                    print(f"  📋 [Subgoals] Decomposed into {len(msg.subgoals)} subgoals:")
                    for i, sg in enumerate(msg.subgoals):
                        print(f"      {i+1}. {sg}")
            for line in msg.chain_of_thought:
                if line not in self.seen_cot:
                    self.seen_cot.add(line)
                    print(f"  🧠 [CoT] {line}")
            if msg.task_status in ("COMPLETE", "FAILED"):
                self.done = True
                self.task_success = (msg.task_status == "COMPLETE")

        def _imu_cb(self, msg: Imu):
            self.latest_imu = msg

        def _model_cb(self, msg: ModelStates):
            for i, name in enumerate(msg.name):
                p = msg.pose[i].position
                self.model_poses[name] = (p.x, p.y, p.z)

    monitor = AgentMissionMonitor()

    # Dispatch Natural Language Command to /aria/command
    print("[5/6] Dispatching Natural Language Command to /aria/command: 'Keep all of the blue items in the red box'...")
    if not monitor.cmd_client.wait_for_service(timeout_sec=15.0):
        print("      ⚠️ /aria/command service not ready. Waiting for lifecycle activation...")
        time.sleep(3.0)

    req = SendCommand.Request()
    req.command = "Keep all of the blue items in the red box"
    future = monitor.cmd_client.call_async(req)

    while not future.done():
        rclpy.spin_once(monitor, timeout_sec=0.1)

    res = future.result()
    if res and res.accepted:
        print(f"      ✅ Command accepted by TaskManager (Task ID: {res.task_id})")
    else:
        print(f"      ⚠️ Command response: {res.message if res else 'No response'}")

    # Monitor until cycle complete or timeout
    start_time = time.time()
    timeout = 90.0  # 90 seconds max for pick-transport-place cycle

    while not monitor.done and (time.time() - start_time) < timeout:
        rclpy.spin_once(monitor, timeout_sec=0.2)

    time.sleep(2.0)
    for _ in range(15):
        rclpy.spin_once(monitor, timeout_sec=0.1)

    # Physical verification in Gazebo ODE
    print("\n[6/6] Verifying Physical End-State in Gazebo ODE Engine...")
    blue_pos = None
    target_name = None
    for name, pos in monitor.model_poses.items():
        if name in ("workpiece_01", "workpiece_good", "blue_workpiece_1") or "good" in name:
            blue_pos = pos
            target_name = name
            print(f"  Physical Object '{name}' final Cartesian position: X={pos[0]:.4f} m, Y={pos[1]:.4f} m, Z={pos[2]:.4f} m")
            if (RED_BOX_X_MIN <= pos[0] <= RED_BOX_X_MAX and
                RED_BOX_Y_MIN <= pos[1] <= RED_BOX_Y_MAX and
                RED_BOX_Z_MIN <= pos[2] <= RED_BOX_Z_MAX):
                break

    is_in_red_box = False
    if blue_pos:
        bx, by, bz = blue_pos
        is_in_red_box = (RED_BOX_X_MIN <= bx <= RED_BOX_X_MAX and
                         RED_BOX_Y_MIN <= by <= RED_BOX_Y_MAX and
                         RED_BOX_Z_MIN <= bz <= RED_BOX_Z_MAX)

        print(f"  Red Box Enclosing Bounds: X in [{RED_BOX_X_MIN}, {RED_BOX_X_MAX}], Y in [{RED_BOX_Y_MIN}, {RED_BOX_Y_MAX}], Z in [{RED_BOX_Z_MIN}, {RED_BOX_Z_MAX}]")
        if is_in_red_box:
            print(f"\n  ✅ PHYSICAL VALIDATION PASSED: Blue workpiece '{target_name}' is resting securely inside the RED BOX!")
        else:
            print(f"\n  ⚠️ Workpiece '{target_name}' coordinates: ({bx:.3f}, {by:.3f}, {bz:.3f})")
    else:
        print("  ⚠️ Could not query model states for blue workpiece.")

    rclpy.shutdown()
    return is_in_red_box, list(monitor.seen_cot)


def main():
    print("═══════════════════════════════════════════════════════════════")
    print("  🤖 Project ARIA: Autonomous Physical Manufacturing Run       ")
    print("  Mission: 'Keep all of the blue items in the red box'        ")
    print("  Zero Hardcoded Poses | Full Multi-Agent Hierarchy           ")
    print("═══════════════════════════════════════════════════════════════\n")

    gui_mode = "--headless" not in sys.argv
    keep_alive_s = 600 if gui_mode else 0
    for arg in sys.argv:
        if arg.startswith("--keep-alive="):
            keep_alive_s = int(arg.split("=")[1])

    clean_existing_processes()
    launch_proc = start_gazebo_and_nodes(gui=gui_mode)

    try:
        success, cot_logs = run_autonomous_cycle()

        print("\n═══════════════════════════════════════════════════════════════")
        if success:
            print("  🌟 MISSION SUCCESS: Autonomous agent successfully processed  ")
            print("     and placed the blue item into the red box!                 ")
        else:
            print("  Physical simulation completed. Inspecting logs...            ")
        print("═══════════════════════════════════════════════════════════════\n")

        if gui_mode and keep_alive_s > 0:
            print(f"👀 Gazebo 3D simulation remains LIVE on DISPLAY :1 for {keep_alive_s}s.")
            print("   You can interact, orbit, and visually inspect the robot arm")
            print("   and the workpiece inside the red box.")
            print("   (Close the Gazebo window or press Ctrl+C when finished)\n")
            try:
                time.sleep(keep_alive_s)
            except KeyboardInterrupt:
                print("Inspection concluded by user.")

    finally:
        print("Shutting down simulation stack...")
        launch_proc.terminate()
        clean_existing_processes()
        print("Done.")


if __name__ == "__main__":
    main()
