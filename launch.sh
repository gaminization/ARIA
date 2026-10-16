#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
# Project ARIA — Universal One-Command Launcher
# Forwarding wrapper to scripts/aria_launch_full.sh
# ═══════════════════════════════════════════════════════════════
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/scripts/aria_launch_full.sh" "$@"
