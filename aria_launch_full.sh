#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# ARIA Full System Launcher — Industrial Workcell + Dashboard
# Uses tmux for persistent sessions. Run once.
# ═══════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ROS_SETUP="/opt/ros/humble/setup.bash"
ARIA_SETUP="$SCRIPT_DIR/install/setup.bash"

echo "╔═══════════════════════════════════════════════════════╗"
echo "║   ARIA Industrial Workcell Full System Launcher       ║"
echo "╚═══════════════════════════════════════════════════════╝"

# Kill existing tmux sessions
tmux kill-session -t aria_sim 2>/dev/null || true
tmux kill-session -t aria_dash 2>/dev/null || true

# ── Session 1: Simulation + All ROS2 nodes ──────────────
tmux new-session -d -s aria_sim -x 220 -y 50
tmux send-keys -t aria_sim "source $ROS_SETUP && source $ARIA_SETUP" Enter
tmux send-keys -t aria_sim "ros2 launch arm_bringup aria_full_u3.launch.py world:=aria_tester_workspace.world 2>&1 | tee /tmp/aria_launch.log" Enter

echo "✅ Simulation session started (aria_sim)"
sleep 3

# ── Session 2: Dashboard ────────────────────────────────
tmux new-session -d -s aria_dash -x 220 -y 50
tmux send-keys -t aria_dash "source $ROS_SETUP && source $ARIA_SETUP" Enter
tmux send-keys -t aria_dash "sleep 12 && cd $SCRIPT_DIR && python3 arm_dashboard/app.py 2>&1 | tee /tmp/aria_dashboard.log" Enter

echo "✅ Dashboard session started (aria_dash)"

echo ""
echo "╔═══════════════════════════════════════════════════════╗"
echo "║   Waiting for system to initialize...                 ║"
echo "╚═══════════════════════════════════════════════════════╝"

sleep 15

echo ""
echo "╔═══════════════════════════════════════════════════════╗"
echo "║   System Status                                       ║"
echo "╚═══════════════════════════════════════════════════════╝"
source "$ROS_SETUP" && source "$ARIA_SETUP"

echo "ROS2 Nodes:"
ros2 node list 2>/dev/null | head -40 || echo "  (waiting for nodes...)"

echo ""
echo "Key Topics:"
ros2 topic list 2>/dev/null | grep -E "(wrist_camera|detection|depth|aria/state|joint_states)" | head -15

echo ""
echo "Dashboard: http://localhost:8080"
echo ""
echo "Tmux sessions:"
tmux ls 2>/dev/null

echo ""
echo "To attach: tmux attach -t aria_sim  OR  tmux attach -t aria_dash"
