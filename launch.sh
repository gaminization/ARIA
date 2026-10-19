#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
# Project ARIA — Universal One-Command Launcher
# Forwarding wrapper to scripts/aria_launch_full.sh
# 
# Modes:
#   Autonomous (default):  ./launch.sh [-w world] [-g]
#   Synthetic (fast demo): ./launch.sh -s / --synthetic [-w world] [-g]
# ═══════════════════════════════════════════════════════════════
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/scripts/aria_launch_full.sh" "$@"

