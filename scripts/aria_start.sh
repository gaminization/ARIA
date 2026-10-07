#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  ARIA Full System Startup — Industrial World with Gripper Cam
# ═══════════════════════════════════════════════════════════════
set -e

ARIA_DIR="/home/gaminizer/Projects/ARIA"
MODELS_DIR="$ARIA_DIR/arm_bringup/models"

# ── Source ROS2 + ARIA workspace ──────────────────────────────
source /opt/ros/humble/setup.bash
source "$ARIA_DIR/install/setup.bash"
export DISPLAY=:1

# ── Set Gazebo model paths so workpiece includes resolve ───────
export GAZEBO_MODEL_PATH="$ARIA_DIR/install/arm_bringup/share/arm_bringup:$MODELS_DIR:/usr/share/gazebo-11/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  🤖 ARIA — Starting Industrial World (gripper cam only)"
echo "═══════════════════════════════════════════════════════════"
echo "GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"
echo ""

# ── Kill any stale Gazebo / ROS2 processes ────────────────────
pkill -9 -f "gzserver|gzclient|ros2 launch|ros2 run arm" 2>/dev/null || true
sleep 2

# ── Step 1: Start Gazebo with industrial world (background) ───
echo "[1/6] Starting Gazebo server + industrial world..."
gzserver "$ARIA_DIR/install/arm_bringup/share/arm_bringup/worlds/aria_industrial_workcell.world" \
  --verbose \
  -slibgazebo_ros_init.so \
  -slibgazebo_ros_factory.so \
  -slibgazebo_ros_force_system.so \
  > /tmp/gzserver.log 2>&1 &
GZSERVER_PID=$!
echo "  gzserver PID=$GZSERVER_PID"

# ── Step 2: Start Gazebo GUI ───────────────────────────────────
echo "[2/6] Starting Gazebo GUI..."
gzclient --verbose > /tmp/gzclient.log 2>&1 &
echo "  gzclient PID=$!"

# ── Step 3: Launch all ARIA nodes (no sim — just nodes) ───────
echo "[3/6] Starting robot_state_publisher + ARIA agents..."
ros2 launch arm_bringup aria_agents_only.launch.py \
  > /tmp/aria_agents.log 2>&1 &
AGENTS_PID=$!
echo "  agents PID=$AGENTS_PID"

# ── Wait for Gazebo /spawn_entity service ─────────────────────
echo ""
echo "Waiting for Gazebo factory service..."
WAITED=0
while ! ros2 service list 2>/dev/null | grep -q "/spawn_entity"; do
  sleep 1
  WAITED=$((WAITED+1))
  if [ $WAITED -gt 60 ]; then
    echo "ERROR: /spawn_entity never appeared after 60s"
    exit 1
  fi
done
echo "  ✅ /spawn_entity ready after ${WAITED}s"

# ── Step 4: Spawn robot ────────────────────────────────────────
echo ""
echo "[4/6] Spawning robot arm..."
ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description \
  -entity aria_arm \
  -x 0.0 -y 0.0 -z 0.0 2>&1 | grep -E "Success|Error|failed"

# ── Step 5: Wait for controller_manager then load controllers ─
echo ""
echo "[5/6] Loading joint controllers..."
WAITED=0
while ! ros2 control list_controllers 2>/dev/null | grep -q "joint_state_broadcaster"; do
  sleep 1
  WAITED=$((WAITED+1))
  [ $WAITED -gt 30 ] && break
done
ros2 control load_controller --set-state active joint_state_broadcaster 2>/dev/null || true
ros2 control load_controller --set-state active joint_trajectory_controller 2>/dev/null || true
echo "  Controllers loaded:"
ros2 control list_controllers 2>/dev/null | grep -E "joint_state|trajectory"

# ── Step 6: Spawn workpieces on conveyor belt ─────────────────
echo ""
echo "[6/6] Spawning workpieces on conveyor belt..."
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
echo "  ✅ 5 workpieces spawned (3 good / 2 defective)"

# ── Step 7: Start dashboard ────────────────────────────────────
echo ""
echo "[7/7] Starting dashboard at http://localhost:8080..."
cd "$ARIA_DIR/arm_dashboard"
python3 app.py &
DASH_PID=$!
echo "  dashboard PID=$DASH_PID"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  ✅ ARIA FULL SYSTEM ONLINE"
echo "  Dashboard: http://localhost:8080"
echo "  Gazebo:    DISPLAY=:1 (industrial world + robot + objects)"
echo "═══════════════════════════════════════════════════════════"
echo ""

# Keep running
wait $DASH_PID
