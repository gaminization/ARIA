#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# ARIA Clean Shutdown Script
# Gracefully stops all simulation instances, agent nodes, and servers
# ═══════════════════════════════════════════════════════════════
set -e

echo "═══════════════════════════════════════════════════════"
echo "  🛑 Shutting down Project ARIA..."
echo "═══════════════════════════════════════════════════════"

# 1. Kill tmux sessions
if command -v tmux >/dev/null 2>&1; then
    tmux kill-session -t aria_sim 2>/dev/null && echo "  ✔ Terminated tmux session: aria_sim" || true
    tmux kill-session -t aria_dash 2>/dev/null && echo "  ✔ Terminated tmux session: aria_dash" || true
fi

# 2. Terminate background processes
pkill -9 -f "gzserver" 2>/dev/null && echo "  ✔ Terminated gzserver" || true
pkill -9 -f "gzclient" 2>/dev/null && echo "  ✔ Terminated gzclient" || true
pkill -9 -f "aria_full_u3.launch.py" 2>/dev/null && echo "  ✔ Terminated aria_full launch" || true
pkill -9 -f "industrial_workcell.launch.py" 2>/dev/null && echo "  ✔ Terminated industrial launch" || true
pkill -9 -f "arm_dashboard/app.py" 2>/dev/null && echo "  ✔ Terminated dashboard server" || true
pkill -9 -f "arm_agents" 2>/dev/null && echo "  ✔ Terminated arm_agents" || true
pkill -9 -f "arm_vision" 2>/dev/null && echo "  ✔ Terminated arm_vision" || true
pkill -9 -f "arm_control" 2>/dev/null && echo "  ✔ Terminated arm_control" || true
pkill -9 -f "arm_ik" 2>/dev/null && echo "  ✔ Terminated arm_ik" || true
pkill -9 -f "arm_planner" 2>/dev/null && echo "  ✔ Terminated arm_planner" || true
pkill -9 -f "arm_learning" 2>/dev/null && echo "  ✔ Terminated arm_learning" || true

# 3. Clean FastRTPS shared memory artifacts
rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null || true
echo "  ✔ Cleaned FastRTPS shared memory segments"

echo "═══════════════════════════════════════════════════════"
echo "  ✅ All ARIA services have been stopped."
echo "═══════════════════════════════════════════════════════"
