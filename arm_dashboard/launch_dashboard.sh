#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# ARIA Control Center — Dashboard Launch Script
# ═══════════════════════════════════════════════════════════════
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /home/gaminizer/Projects/ARIA/install/setup.bash 2>/dev/null || true

export ARIA_DASHBOARD_PORT=8000

echo "🚀 Starting ARIA Control Center on port 8000..."
python3 -m uvicorn app:app --host 0.0.0.0 --port 8000 &
SERVER_PID=$!

sleep 2
xdg-open http://localhost:8000 2>/dev/null || true

echo "═══════════════════════════════════════════════════════════════"
echo "✔ ARIA Control Center Dashboard running at http://localhost:8000"
echo "✔ WebSocket State Stream: ws://localhost:8000/ws/state (10 Hz)"
echo "✔ WebSocket Camera Stream: ws://localhost:8000/ws/cameras (30 fps)"
echo "═══════════════════════════════════════════════════════════════"

wait $SERVER_PID
