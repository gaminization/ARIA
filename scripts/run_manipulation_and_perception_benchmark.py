#!/usr/bin/env python3
"""
scripts/run_manipulation_and_perception_benchmark.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Launches Gazebo Classic with the industrial workcell and runs the authoritative
manipulation & perception benchmark suite end-to-end.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import os
import sys
import time
import subprocess
import signal

WORKSPACE_ROOT = "/home/gaminizer/Projects/ARIA"
SETUP = f"source {WORKSPACE_ROOT}/install/setup.bash"

def clean_stale_processes():
    print("[1/5] Cleaning stale Gazebo/ROS 2 processes...")
    subprocess.run("pkill -9 -f gzserver || true; pkill -9 -f gzclient || true; pkill -9 -f robot_state_publisher || true; pkill -9 -f manual_control || true",
                   shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

def start_gazebo_simulation():
    print("[2/5] Starting Gazebo industrial workcell simulation...")
    env = os.environ.copy()
    env["GAZEBO_IP"] = "127.0.0.1"
    env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:11345"
    env["DISPLAY"] = ":1"

    cmd = f"{SETUP} && ros2 launch arm_bringup industrial_workcell.launch.py gui:=false auto_cycle:=false"
    proc = subprocess.Popen(["bash", "-c", cmd], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("[3/5] Waiting for workcell services to become active...")
    start = time.time()
    ready = False
    while time.time() - start < 35.0:
        res = subprocess.run(["bash", "-c", f"{SETUP} && ros2 service list"], capture_output=True, text=True)
        if "/aria/go_named_pose" in res.stdout and "/aria/open_gripper" in res.stdout:
            ready = True
            break
        time.sleep(1.0)

    if ready:
        print("      ✅ Gazebo workcell services and ROS 2 controllers ready.")
    else:
        print("      ⚠️ Service wait timeout; attempting to proceed...")

    time.sleep(2.0)
    return proc

def run_suite():
    print("[4/5] Executing Authoritative Manipulation & Perception Benchmark Suite...")
    env = os.environ.copy()
    env["GAZEBO_IP"] = "127.0.0.1"
    env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:11345"
    env["DISPLAY"] = ":1"

    cmd = f"{SETUP} && python3 {WORKSPACE_ROOT}/scripts/benchmark_manipulation_suite_gazebo.py"
    ret = subprocess.run(["bash", "-c", cmd], env=env)
    return ret.returncode

def main():
    clean_stale_processes()
    proc = None
    try:
        proc = start_gazebo_simulation()
        rc = run_suite()
        print(f"[5/5] Benchmark execution completed with return code {rc}.")
    finally:
        print("Shutting down Gazebo simulation...")
        if proc:
            proc.terminate()
            time.sleep(2)
        subprocess.run("pkill -9 -f gzserver || true; pkill -9 -f gzclient || true; pkill -9 -f robot_state_publisher || true", shell=True)
        time.sleep(1)

if __name__ == "__main__":
    main()
