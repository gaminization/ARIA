#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# ARIA Pick-Up Test — Full System Launch
# Starts tester world sim + all available agents, then sends
# "pick up the red cup" command
# ═══════════════════════════════════════════════════════════════
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash
source /home/gaminizer/Projects/ARIA/install/setup.bash 2>/dev/null || true

export DISPLAY=${DISPLAY:-:1}

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  🤖 ARIA Pick-Up Test — Launching Full System"
echo "  World: aria_tester_workspace (Bullet3 + IAI Table)"
echo "  Task:  Pick up the red cup"
echo "═══════════════════════════════════════════════════════════"
echo ""

# ── Step 1: Launch Gazebo + Robot + Controllers (tester world) ──
echo "[1/4] Starting Gazebo simulation with tester world..."
ros2 launch arm_bringup tester_sim.launch.py use_rviz:=true &
SIM_PID=$!
echo "  Simulation PID: $SIM_PID"

# Wait for Gazebo to be ready
echo "  Waiting for Gazebo to start..."
sleep 15

# Verify simulation is running
if ! ros2 topic list 2>/dev/null | grep -q "/joint_states"; then
    echo "  ⚠ Warning: /joint_states not found yet, waiting longer..."
    sleep 10
fi

echo "  ✓ Simulation running"
echo ""

# ── Step 2: Start AI agents ────────────────────────────────────
echo "[2/4] Starting AI agents..."

# Core agents (these all import successfully)
ros2 run arm_agents planning_agent &
sleep 0.3
ros2 run arm_agents skill_agent &
sleep 0.3
ros2 run arm_agents control_agent &
sleep 0.3
ros2 run arm_agents safety_agent &
sleep 0.3
ros2 run arm_agents affordance_agent &
sleep 0.3
ros2 run arm_agents reachability_agent &
sleep 0.3
ros2 run arm_agents depth_agent &
sleep 0.3
ros2 run arm_agents tracking_agent &
sleep 0.3
ros2 run arm_agents attention_agent &
sleep 0.3
ros2 run arm_agents memory_agent &
sleep 0.3
ros2 run arm_agents world_model_agent &
sleep 0.3
ros2 run arm_agents learning_agent &
sleep 0.3
ros2 run arm_agents evaluation_agent &
sleep 0.3
ros2 run arm_agents dialogue_agent &
sleep 0.3

# Orchestration
ros2 run arm_planner task_manager &
sleep 0.3
ros2 run arm_planner memory_manager &
sleep 0.3
ros2 run arm_planner health_monitor &
sleep 0.3

# IK solver
ros2 run arm_ik ik_node &
sleep 0.3

echo "  ✓ Agents started"
echo ""

# Wait for agents to initialize
echo "[3/4] Waiting for agents to initialize..."
sleep 8

# Verify key services
echo "  Checking for /aria/command service..."
RETRIES=0
while ! ros2 service list 2>/dev/null | grep -q "/aria/command"; do
    RETRIES=$((RETRIES + 1))
    if [ $RETRIES -gt 20 ]; then
        echo "  ⚠ /aria/command service not found after 20 retries"
        break
    fi
    sleep 1
done

if ros2 service list 2>/dev/null | grep -q "/aria/command"; then
    echo "  ✓ /aria/command service is available"
fi

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  🤖 ARIA FULL SYSTEM ONLINE"
echo "  Sending command: 'pick up the red cup'"
echo "═══════════════════════════════════════════════════════════"
echo ""

# ── Step 4: Send the pick-up command ───────────────────────────
echo "[4/4] Sending command..."
ros2 service call /aria/command arm_interfaces/srv/SendCommand "{command: 'pick up the red cup'}" &

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  🎯 Command sent! Watch the simulation."
echo "  The robot should:"
echo "    1. Locate the red mug using vision"
echo "    2. Plan a grasp approach"
echo "    3. Reach out and pick it up"
echo "═══════════════════════════════════════════════════════════"
echo ""

# Keep alive
wait $SIM_PID
