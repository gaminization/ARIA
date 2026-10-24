#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# ARIA Full System Launcher — Autonomous Multi-Agent Stack + Dashboard
# ═══════════════════════════════════════════════════════════════
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARIA_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ARIA_ROOT"

ROS_SETUP="/opt/ros/humble/setup.bash"
ARIA_SETUP="$ARIA_ROOT/install/setup.bash"
FASTDDS_PROFILE="$ARIA_ROOT/arm_bringup/config/fastdds_udp.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_PROFILE"

# Default configuration
WORLD_NAME="aria_tester_workspace.world"
ENABLE_GUI="false"
LAUNCH_DASHBOARD=true
DASHBOARD_PORT=8000
USE_TMUX=true
ENABLE_OPENVLA=false
SYNTHETIC_MODE=false

print_usage() {
    cat << 'EOF'
Usage: ./scripts/aria_launch_full.sh [OPTIONS]

Options:
  -g, --gui              Launch Gazebo Classic 3D GUI (default: headless simulation)
  -w, --world <NAME>     World selection: 'tester' (default), 'industrial', 'workspace',
                         or filename (e.g. 'aria_tester_workspace.world')
  -p, --port <PORT>      Dashboard web port (default: 8000)
  --no-dashboard         Launch simulation & multi-agent stack without dashboard
  --no-tmux              Launch directly in foreground/subshell (without tmux)
  -s, --synthetic        Launch in fast synthetic mode (rapid demonstration of industrial & tester suites)
  --openvla              Enable OpenVLA-7B policy execution backend
  --status               Show health and status of active ARIA nodes and services
  --stop                 Cleanly stop all running ARIA, Gazebo, and dashboard processes
  -h, --help             Show this help message

Examples:
  ./scripts/aria_launch_full.sh                    # Headless sim + 15 agents + dashboard
  ./scripts/aria_launch_full.sh --synthetic        # Rapid synthetic demonstration suite
  ./scripts/aria_launch_full.sh -s -w industrial   # Industrial synthetic suite
  ./scripts/aria_launch_full.sh -s --gui           # Synthetic demo with Gazebo 3D GUI
  ./scripts/aria_launch_full.sh --openvla          # Full stack with OpenVLA-7B backend
  ./scripts/aria_launch_full.sh --gui              # Launch with Gazebo 3D GUI window
  ./scripts/aria_launch_full.sh -w industrial -g  # Industrial workcell with GUI
  ./scripts/aria_launch_full.sh --status           # Check running nodes & topics
  ./scripts/aria_launch_full.sh --stop             # Shut down all processes
EOF
}

stop_all() {
    echo "Stopping Project ARIA processes..."
    if command -v tmux >/dev/null 2>&1; then
        tmux kill-session -t aria_sim 2>/dev/null || true
        tmux kill-session -t aria_dash 2>/dev/null || true
    fi
    pkill -9 -f "gzserver" 2>/dev/null || true
    pkill -9 -f "gzclient" 2>/dev/null || true
    pkill -9 -f "aria_full_u3.launch.py" 2>/dev/null || true
    pkill -9 -f "arm_dashboard/app.py" 2>/dev/null || true
    pkill -9 -f "arm_agents" 2>/dev/null || true
    pkill -9 -f "arm_vision" 2>/dev/null || true
    pkill -9 -f "arm_control" 2>/dev/null || true
    pkill -9 -f "arm_ik" 2>/dev/null || true
    pkill -9 -f "arm_planner" 2>/dev/null || true
    pkill -9 -f "arm_learning" 2>/dev/null || true
    rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null || true

    # Clear world model SQLite databases to ensure clean restart
    world_model_db_paths=(
        "/home/gaminizer/Projects/ARIA/build/arm_planner/data/world_model.db"
        "/home/gaminizer/Projects/ARIA/arm_planner/data/world_model.db"
        "/home/gaminizer/Projects/ARIA/install/arm_agents/lib/python3.10/arm_planner/data/world_model.db"
        "/home/gaminizer/Projects/ARIA/install/arm_planner/lib/data/world_model.db"
    )
    for db_p in "${world_model_db_paths[@]}"; do
        if [ -f "$db_p" ]; then
            sqlite3 "$db_p" "DELETE FROM objects; DELETE FROM spatial_relations;" 2>/dev/null || true
        fi
    done

    echo "✅ All ARIA processes stopped and shared memory cleaned."
}

check_status() {
    echo "═══════════════════════════════════════════════════════"
    echo "  🤖 Project ARIA: System Status"
    echo "═══════════════════════════════════════════════════════"
    if [ -f "$ROS_SETUP" ]; then
        source "$ROS_SETUP"
    fi
    if [ -f "$ARIA_SETUP" ]; then
        source "$ARIA_SETUP"
    fi

    echo ""
    echo "── Tmux Sessions ───────────────────────────────────────"
    if command -v tmux >/dev/null 2>&1; then
        tmux ls 2>/dev/null || echo "  (no active tmux sessions)"
    else
        echo "  tmux not installed"
    fi

    echo ""
    echo "── Core Processes ──────────────────────────────────────"
    for proc in gzserver gzclient uvicorn app.py detection_node vision_agent; do
        if pgrep -f "$proc" >/dev/null 2>&1; then
            echo "  ✅ $proc is RUNNING (PID $(pgrep -f "$proc" | head -1))"
        else
            echo "  ⚪ $proc is not running"
        fi
    done

    echo ""
    echo "── ROS 2 Controllers ───────────────────────────────────"
    if command -v ros2 >/dev/null 2>&1; then
        timeout 2s ros2 control list_controllers 2>/dev/null || echo "  (controller manager not active)"
    fi

    echo ""
    echo "── Key Active Topics ───────────────────────────────────"
    if command -v ros2 >/dev/null 2>&1; then
        timeout 2s ros2 topic list 2>/dev/null | grep -E "(wrist_camera|top_camera|detection|depth|aria/state|joint_states|joint_trajectory)" | head -15 || echo "  (no active topics)"
    fi

    echo ""
    echo "── Dashboard Endpoint ──────────────────────────────────"
    if curl -s http://localhost:8000/api/health >/dev/null 2>&1; then
        echo "  ✅ Dashboard online: http://localhost:8000"
    else
        echo "  ⚪ Dashboard not responding on http://localhost:8000"
    fi
    echo "═══════════════════════════════════════════════════════"
}

# Parse CLI arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -g|--gui)
            ENABLE_GUI="true"
            shift
            ;;
        -w|--world)
            case "$2" in
                tester)
                    WORLD_NAME="aria_tester_workspace.world"
                    ;;
                industrial)
                    WORLD_NAME="aria_industrial_workcell.world"
                    ;;
                workspace)
                    WORLD_NAME="aria_workspace.world"
                    ;;
                *)
                    WORLD_NAME="$2"
                    ;;
            esac
            shift 2
            ;;
        -p|--port)
            DASHBOARD_PORT="$2"
            shift 2
            ;;
        --no-dashboard)
            LAUNCH_DASHBOARD=false
            shift
            ;;
        --no-tmux)
            USE_TMUX=false
            shift
            ;;
        -s|--synthetic)
            SYNTHETIC_MODE="true"
            shift
            ;;
        --openvla)
            ENABLE_OPENVLA=true
            shift
            ;;
        --status)
            check_status
            exit 0
            ;;
        --stop)
            stop_all
            exit 0
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            # Backward compatibility: if single argument is a world file name
            if [[ "$1" == *.world ]]; then
                WORLD_NAME="$1"
                shift
            else
                echo "Unknown option: $1"
                print_usage
                exit 1
            fi
            ;;
    esac
done

# Prerequisites validation
if [ ! -f "$ROS_SETUP" ]; then
    echo "❌ Error: ROS 2 Humble setup not found at $ROS_SETUP"
    exit 1
fi

if [ ! -f "$ARIA_SETUP" ]; then
    echo "❌ Error: ARIA workspace build not found at $ARIA_SETUP"
    echo "   Please build the workspace first with: colcon build --symlink-install"
    exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
    USE_TMUX=false
fi

echo "╔═══════════════════════════════════════════════════════╗"
echo "║   🤖 ARIA Full System Launcher                       ║"
echo "╚═══════════════════════════════════════════════════════╝"
echo "  Workspace Root: $ARIA_ROOT"
echo "  Target World:   $WORLD_NAME"
echo "  Gazebo 3D GUI:  $ENABLE_GUI"
echo "  Dashboard:      $([ "$LAUNCH_DASHBOARD" = true ] && echo "port $DASHBOARD_PORT" || echo "disabled")"
echo "  Mode:           $([ "$USE_TMUX" = true ] && echo "tmux background sessions" || echo "direct subshell")"
echo "  Stack:          $([ "$SYNTHETIC_MODE" = true ] && echo "SYNTHETIC DEMONSTRATION" || echo "AUTONOMOUS 15-AGENT STACK")"
echo "  OpenVLA-7B:     $([ "$ENABLE_OPENVLA" = true ] && echo "enabled (ARIA_OPENVLA_FORCE_LOAD=1)" || echo "disabled")"
echo "─────────────────────────────────────────────────────────"

# Clean previous sessions
stop_all >/dev/null 2>&1 || true
sleep 1

if [ "$USE_TMUX" = true ]; then
    # ── Session 1: Simulation + ROS 2 Multi-Agent Stack ──────
    tmux start-server
    sleep 0.5
    tmux new-session -d -s aria_sim -x 220 -y 50
    VLA_ENV=""
    if [ "$ENABLE_OPENVLA" = true ]; then
        VLA_ENV="export ARIA_OPENVLA_FORCE_LOAD=1 ARIA_USE_OPENVLA=1 ARIA_ROOT=\"$ARIA_ROOT\" && "
    fi
    WORKCELL_FLAG=""
    if [[ "$WORLD_NAME" == *"industrial"* ]]; then
        WORKCELL_FLAG="export ARIA_WORKCELL=industrial && "
    fi
    tmux send-keys -t aria_sim "cd \"$ARIA_ROOT\" && ${VLA_ENV}${WORKCELL_FLAG}export ARIA_WORLD=\"$WORLD_NAME\" && export ROS_LOCALHOST_ONLY=1 && export GAZEBO_IP=127.0.0.1 && export FASTRTPS_DEFAULT_PROFILES_FILE=\"$FASTDDS_PROFILE\" && source \"$ROS_SETUP\" && source \"$ARIA_SETUP\"" Enter
    tmux send-keys -t aria_sim "ros2 launch arm_bringup aria_full_u3.launch.py world:=\"$WORLD_NAME\" gui:=\"$ENABLE_GUI\" synthetic:=\"$SYNTHETIC_MODE\" 2>&1 | tee /tmp/aria_launch.log" Enter

    echo "✅ Simulation + multi-agent stack started in tmux session 'aria_sim'"

    # ── Session 2: Web Dashboard Backend ─────────────────────
    if [ "$LAUNCH_DASHBOARD" = true ]; then
        tmux new-session -d -s aria_dash -x 220 -y 50
        tmux send-keys -t aria_dash "cd \"$ARIA_ROOT\" && export ROS_LOCALHOST_ONLY=1 && export GAZEBO_IP=127.0.0.1 && export FASTRTPS_DEFAULT_PROFILES_FILE=\"$FASTDDS_PROFILE\" && source \"$ROS_SETUP\" && source \"$ARIA_SETUP\"" Enter
        tmux send-keys -t aria_dash "sleep 12 && export ARIA_DASHBOARD_PORT=$DASHBOARD_PORT && python3 arm_dashboard/app.py 2>&1 | tee /tmp/aria_dashboard.log" Enter
        echo "✅ Dashboard backend initialized in tmux session 'aria_dash'"
    fi

    echo ""
    echo "Waiting for system initialization (15s)..."
    sleep 15

    # Check and activate joint_state_broadcaster and joint_trajectory_controller if needed
    source "$ROS_SETUP" && source "$ARIA_SETUP"
    if ! ros2 control list_controllers 2>/dev/null | grep -q "joint_state_broadcaster.*active"; then
        ros2 control unload_controller joint_state_broadcaster 2>/dev/null || true
        ros2 control load_controller --set-state active joint_state_broadcaster 2>/dev/null || true
    fi
    if ! ros2 control list_controllers 2>/dev/null | grep -q "joint_trajectory_controller.*active"; then
        ros2 control unload_controller joint_trajectory_controller 2>/dev/null || true
        ros2 control load_controller --set-state active joint_trajectory_controller 2>/dev/null || true
    fi

    echo ""
    echo "═══════════════════════════════════════════════════════"
    echo "  🚀 ARIA FULL SYSTEM IS ONLINE"
    echo "═══════════════════════════════════════════════════════"
    if [ "$LAUNCH_DASHBOARD" = true ]; then
        echo "  Web Dashboard:  http://localhost:$DASHBOARD_PORT"
        echo "  REST API Docs:  http://localhost:$DASHBOARD_PORT/docs"
        echo "  State Stream:   ws://localhost:$DASHBOARD_PORT/ws/state"
        echo "  Camera Stream:  ws://localhost:$DASHBOARD_PORT/ws/cameras"
    fi
    echo ""
    echo "  Attach Commands:"
    echo "    tmux attach -t aria_sim   (Simulation & agent logs)"
    if [ "$LAUNCH_DASHBOARD" = true ]; then
        echo "    tmux attach -t aria_dash  (Web dashboard server logs)"
    fi
    echo ""
    echo "  Management Commands:"
    echo "    ./scripts/aria_launch_full.sh --status"
    echo "    ./scripts/aria_launch_full.sh --stop"
    echo "═══════════════════════════════════════════════════════"

else
    # ── Direct Non-tmux Mode ──────────────────────────────────
    source "$ROS_SETUP"
    source "$ARIA_SETUP"

    cleanup() {
        echo ""
        echo "Shutting down ARIA..."
        stop_all
        exit 0
    }
    trap cleanup SIGINT SIGTERM EXIT

    if [ "$ENABLE_OPENVLA" = true ]; then
        export ARIA_OPENVLA_FORCE_LOAD=1
        export ARIA_USE_OPENVLA=1
        export ARIA_ROOT="$ARIA_ROOT"
    fi

    echo "Starting simulation & multi-agent stack..."
    ros2 launch arm_bringup aria_full_u3.launch.py world:="$WORLD_NAME" gui:="$ENABLE_GUI" synthetic:="$SYNTHETIC_MODE" 2>&1 | tee /tmp/aria_launch.log &
    SIM_PID=$!

    if [ "$LAUNCH_DASHBOARD" = true ]; then
        echo "Starting dashboard on port $DASHBOARD_PORT..."
        (sleep 12 && export ARIA_DASHBOARD_PORT=$DASHBOARD_PORT && python3 arm_dashboard/app.py 2>&1 | tee /tmp/aria_dashboard.log) &
        DASH_PID=$!
    fi

    echo ""
    echo "═══════════════════════════════════════════════════════"
    echo "  🚀 ARIA IS RUNNING IN FOREGROUND (Press Ctrl+C to stop)"
    echo "  Dashboard: http://localhost:$DASHBOARD_PORT"
    echo "═══════════════════════════════════════════════════════"

    wait $SIM_PID
fi
