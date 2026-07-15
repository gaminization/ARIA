#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# ARIA Docker Entrypoint
# Sources ROS2, conda env, and ARIA workspace, then exec's CMD.
# ═══════════════════════════════════════════════════════════════
set -e

# ── Source ROS2 ────────────────────────────────────────────
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# ── Source ARIA workspace ──────────────────────────────────
if [ -f /aria_ws/install/setup.bash ]; then
    source /aria_ws/install/setup.bash
fi

# ── Activate conda environment ─────────────────────────────
if [ -d /opt/conda/envs/aria ]; then
    source /opt/conda/etc/profile.d/conda.sh
    conda activate aria
fi

# ── PYTHONPATH for ARIA packages ───────────────────────────
export PYTHONPATH="/aria_ws/src/aria:${PYTHONPATH}"

# ── ROS2 middleware configuration ──────────────────────────
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# ── CUDA visibility ───────────────────────────────────────
export NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
export NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-compute,utility,graphics}

# ── Create runtime directories ─────────────────────────────
mkdir -p /aria_ws/logs /aria_ws/bags /aria_ws/datasets

# ── Display mode info ──────────────────────────────────────
echo "═══════════════════════════════════════════════════════"
echo "  🤖 ARIA Container"
echo "  ROS2:    ${ROS_DISTRO:-humble}"
echo "  Mode:    ${ARIA_MODE:-sim}"
echo "  Domain:  ${ROS_DOMAIN_ID:-0}"
if command -v nvidia-smi &>/dev/null; then
    GPU=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    echo "  GPU:     ${GPU:-none}"
fi
echo "═══════════════════════════════════════════════════════"

# ── Execute command ────────────────────────────────────────
exec "$@"
