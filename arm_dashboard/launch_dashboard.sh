#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# ARIA Control Center — Dashboard Launch Script
# ═══════════════════════════════════════════════════════════════
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="$( cd "$DIR/.." && pwd )"
cd "$DIR"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WORKSPACE_ROOT/install/setup.bash" 2>/dev/null || true

export ARIA_DASHBOARD_PORT="${ARIA_DASHBOARD_PORT:-8000}"

echo "═══════════════════════════════════════════════════════════════"
echo "🚀 Starting ARIA Control Center on port $ARIA_DASHBOARD_PORT..."
echo "═══════════════════════════════════════════════════════════════"

python3 app.py &
SERVER_PID=$!

sleep 2
echo "✔ ARIA Control Center Dashboard running at http://localhost:$ARIA_DASHBOARD_PORT"
echo "✔ REST API Documentation: http://localhost:$ARIA_DASHBOARD_PORT/docs"
echo "✔ WebSocket State Stream: ws://localhost:$ARIA_DASHBOARD_PORT/ws/state (10 Hz)"
echo "✔ WebSocket Camera Stream: ws://localhost:$ARIA_DASHBOARD_PORT/ws/cameras (30 fps)"
echo "═══════════════════════════════════════════════════════════════"

wait $SERVER_PID
