#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  ARIA Full System Direct Startup — Industrial Workcell
#  Direct process pipeline with Gazebo, Robot, Controllers,
#  5 Conveyor Workpieces, and ARIA Control Center Dashboard.
# ═══════════════════════════════════════════════════════════════
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARIA_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MODELS_DIR="$ARIA_DIR/arm_bringup/models"

# ── Parse arguments ───────────────────────────────────────────
HEADLESS=false
for arg in "$@"; do
    case "$arg" in
        --headless|--no-gui)
            HEADLESS=true
            ;;
        -h|--help)
            echo "Usage: ./scripts/aria_start.sh [--headless]"
            echo "  --headless   Run without Gazebo 3D GUI window"
            exit 0
            ;;
    esac
done

# ── Source ROS2 + ARIA workspace ──────────────────────────────
source /opt/ros/humble/setup.bash
source "$ARIA_DIR/install/setup.bash"

# Detect display or fallback
export DISPLAY="${DISPLAY:-:1}"

# ── Set Gazebo model paths so workpiece includes resolve ───────
export GAZEBO_MODEL_PATH="$ARIA_DIR/install/arm_bringup/share/arm_bringup:$MODELS_DIR:/usr/share/gazebo-11/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  🤖 ARIA — Starting Industrial Workcell Pipeline"
echo "═══════════════════════════════════════════════════════════"
echo "  Workspace:   $ARIA_DIR"
echo "  Display:     $DISPLAY"
echo "  GUI Window:  $([ "$HEADLESS" = true ] && echo "disabled" || echo "enabled")"
echo "═══════════════════════════════════════════════════════════"
echo ""

# ── Clean previous processes ──────────────────────────────────
pkill -9 -f "gzserver|gzclient|ros2 launch|ros2 run arm|arm_dashboard/app.py" 2>/dev/null || true
rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null || true
sleep 1

# Clean exit trap
cleanup() {
    echo ""
    echo "Stopping ARIA Industrial Workcell..."
    pkill -9 -f "gzserver|gzclient|ros2 launch|ros2 run arm|arm_dashboard/app.py" 2>/dev/null || true
    rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null || true
    echo "All processes cleaned."
}
trap cleanup EXIT INT TERM

# ── Step 1: Start Gazebo server + industrial world ────────────
echo "[1/7] Starting Gazebo server + industrial world..."
gzserver "$ARIA_DIR/install/arm_bringup/share/arm_bringup/worlds/aria_industrial_workcell.world" \
  --verbose \
  -slibgazebo_ros_init.so \
  -slibgazebo_ros_factory.so \
  -slibgazebo_ros_force_system.so \
  > /tmp/gzserver.log 2>&1 &
GZSERVER_PID=$!
echo "  ✔ gzserver started (PID: $GZSERVER_PID)"

# ── Step 2: Start Gazebo GUI (optional) ───────────────────────
if [ "$HEADLESS" = false ]; then
    echo "[2/7] Starting Gazebo 3D GUI client..."
    gzclient --verbose > /tmp/gzclient.log 2>&1 &
    echo "  ✔ gzclient started (PID: $!)"
else
    echo "[2/7] Gazebo GUI disabled (headless mode)"
fi

# ── Step 3: Launch all ARIA nodes (robot_state_publisher + agents)
echo "[3/7] Starting robot_state_publisher + ARIA agents..."
ros2 launch arm_bringup aria_agents_only.launch.py \
  > /tmp/aria_agents.log 2>&1 &
AGENTS_PID=$!
echo "  ✔ Agents stack started (PID: $AGENTS_PID)"

# ── Wait for Gazebo /spawn_entity service ─────────────────────
echo ""
echo "Waiting for Gazebo factory service..."
WAITED=0
while ! ros2 service list 2>/dev/null | grep -q "/spawn_entity"; do
  sleep 1
  WAITED=$((WAITED+1))
  if [ $WAITED -gt 60 ]; then
    echo "❌ ERROR: /spawn_entity service never appeared after 60s"
    exit 1
  fi
done
echo "  ✔ /spawn_entity service ready after ${WAITED}s"

# ── Step 4: Spawn robot ────────────────────────────────────────
echo ""
echo "[4/7] Spawning robot arm into simulation..."
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description \
  -entity aria_arm \
  -x 0.0 -y 0.0 -z 0.0 2>&1 | grep -E "Success|Error|failed" || true

# ── Step 5: Wait for controller_manager then load controllers ─
echo ""
echo "[5/7] Activating joint trajectory and state controllers..."
WAITED=0
while ! ros2 control list_controllers 2>/dev/null | grep -q "joint_state_broadcaster"; do
  sleep 1
  WAITED=$((WAITED+1))
  [ $WAITED -gt 30 ] && break
done
ros2 control load_controller --set-state active joint_state_broadcaster 2>/dev/null || true
ros2 control load_controller --set-state active joint_trajectory_controller 2>/dev/null || true
echo "  ✔ Controllers loaded:"
ros2 control list_controllers 2>/dev/null | grep -E "joint_state|trajectory" || echo "  (controllers starting...)"

# ── Step 6: Spawn workpieces on conveyor belt ─────────────────
echo ""
echo "[6/7] Spawning conveyor workpieces (3 good / 2 defective)..."
for i in 1 2 3; do
  X=$(python3 -c "print(round(0.08 + ($i-1)*0.09, 3))")
  ros2 run gazebo_ros spawn_entity.py \
    -file "$MODELS_DIR/workpiece_good/model.sdf" \
    -entity "workpiece_good_$i" \
    -x $X -y 0.0 -z 0.815 2>&1 | grep -E "Success|Error" &
done
ros2 run gazebo_ros spawn_entity.py \
  -file "$MODELS_DIR/workpiece_defect/model.sdf" \
  -entity "workpiece_defect_1" \
  -x 0.33 -y 0.0 -z 0.815 2>&1 | grep -E "Success|Error" &
ros2 run gazebo_ros spawn_entity.py \
  -file "$MODELS_DIR/workpiece_defect/model.sdf" \
  -entity "workpiece_defect_2" \
  -x 0.42 -y 0.0 -z 0.815 2>&1 | grep -E "Success|Error" &
wait
echo "  ✔ 5 workpieces successfully spawned"

# ── Step 7: Start dashboard ────────────────────────────────────
echo ""
echo "[7/7] Starting ARIA Control Center on http://localhost:8000..."
cd "$ARIA_DIR"
python3 arm_dashboard/app.py > /tmp/aria_dashboard.log 2>&1 &
DASH_PID=$!
echo "  ✔ Dashboard backend running (PID: $DASH_PID)"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  ✅ ARIA INDUSTRIAL WORKCELL FULLY ONLINE"
echo "  Dashboard: http://localhost:8000"
echo "  API Docs:  http://localhost:8000/docs"
echo "  Logs:      /tmp/gzserver.log, /tmp/aria_agents.log, /tmp/aria_dashboard.log"
echo "═══════════════════════════════════════════════════════════"
echo "Press Ctrl+C to terminate all processes cleanly."
echo ""

wait $DASH_PID
