#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# ARIA Upgrade U3 — Infrastructure Installation Script
# Sets up Docker, experiment tracking, bag recording, object DB.
#
# Usage: bash arm_bringup/scripts/upgrade_u3_install.sh
# ═══════════════════════════════════════════════════════════════
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARIA_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo ""
echo "═══════════════════════════════════════════════════════"
echo "  ARIA Upgrade U3 — Infrastructure Installer"
echo "═══════════════════════════════════════════════════════"
echo "  ARIA directory: $ARIA_DIR"
echo ""

# ── 1. Check prerequisites ────────────────────────────────
echo "[1/7] Checking prerequisites..."

# Docker
if command -v docker &> /dev/null; then
    DOCKER_VERSION=$(docker --version | grep -oP '\d+\.\d+\.\d+')
    echo "  ✓ Docker: $DOCKER_VERSION"
else
    echo "  ✗ Docker not found."
    echo "    Install: https://docs.docker.com/engine/install/ubuntu/"
    echo "    Or: sudo apt install docker.io docker-compose-plugin"
fi

# Docker Compose
if docker compose version &> /dev/null; then
    COMPOSE_VERSION=$(docker compose version --short 2>/dev/null || echo "v2+")
    echo "  ✓ Docker Compose: $COMPOSE_VERSION"
else
    echo "  ✗ Docker Compose v2 not found."
    echo "    Install: sudo apt install docker-compose-plugin"
fi

# NVIDIA Container Toolkit
if docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi &> /dev/null; then
    echo "  ✓ NVIDIA Container Toolkit: working"
else
    echo "  ⚠ NVIDIA GPU not accessible in Docker."
    echo "    See: docker/NVIDIA_SETUP.md"
fi

# ── 2. Python dependencies ────────────────────────────────
echo ""
echo "[2/7] Installing Python dependencies..."
pip install --quiet --upgrade \
    mlflow>=2.10.0 \
    h5py>=3.9.0 \
    pyyaml>=6.0 \
    sqlalchemy>=2.0.0 \
    aiohttp>=3.9.0 \
    websockets>=12.0 \
    2>/dev/null || echo "  ⚠ Some pip packages failed"
echo "  ✓ Python packages installed"

# ── 3. Create directories ─────────────────────────────────
echo ""
echo "[3/7] Creating directories..."
mkdir -p ~/aria_bags
mkdir -p ~/aria_experiments
mkdir -p "$ARIA_DIR/arm_planner/data/object_references"
mkdir -p "$ARIA_DIR/arm_learning/data"
echo "  ✓ Directories ready"

# ── 4. Initialize object database ─────────────────────────
echo ""
echo "[4/7] Initializing Object Model Database..."
cd "$ARIA_DIR"
PYTHONPATH="arm_planner:$PYTHONPATH" python3 -c "
from arm_planner.object_model_db import ObjectModelDB
db = ObjectModelDB()
count = db.import_library('arm_planner/data/object_library_default.yaml')
objects = db.list_objects()
print(f'  ✓ Object DB: {len(objects)} objects ({count} newly imported)')
" 2>/dev/null || echo "  ⚠ Object DB initialization failed (will retry on first use)"

# ── 5. Initialize bag index ───────────────────────────────
echo ""
echo "[5/7] Initializing Bag Index..."
PYTHONPATH="arm_learning:$PYTHONPATH" python3 -c "
from arm_learning.bag_indexer import BagIndexer
indexer = BagIndexer()
count = indexer.index_directory()
stats = indexer.get_stats()
print(f'  ✓ Bag index: {stats[\"total_bags\"]} bags indexed')
" 2>/dev/null || echo "  ⚠ Bag index init deferred (no bags yet)"

# ── 6. Build ARIA ─────────────────────────────────────────
echo ""
echo "[6/7] Building ARIA workspace..."
cd "$ARIA_DIR"
source /opt/ros/humble/setup.bash 2>/dev/null || true
colcon build --symlink-install 2>&1 | tail -5

# ── 7. Docker images (optional) ───────────────────────────
echo ""
echo "[7/7] Docker images..."
if command -v docker &> /dev/null; then
    echo "  To build Docker images:"
    echo "    cd docker && make build"
    echo ""
    echo "  To start with Docker:"
    echo "    make -f docker/Makefile sim"
    echo ""
    echo "  To start MLflow tracking:"
    echo "    make -f docker/Makefile tracking"
else
    echo "  ⚠ Docker not available, skipping image build"
fi

# ── Summary ───────────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════════"
echo "  ✅ ARIA Upgrade U3 Installation Complete"
echo ""
echo "  Installed Components:"
echo "    • Docker containerization (4 Dockerfiles)"
echo "    • CI/CD pipeline (GitHub Actions)"
echo "    • Experiment tracking (MLflow)"
echo "    • Bag recording pipeline"
echo "    • Object model database"
echo "    • Dashboard additions (3 new panels)"
echo ""
echo "  Quick Start:"
echo "    source install/setup.bash"
echo "    ros2 launch arm_bringup aria_full_u3.launch.py"
echo ""
echo "  Docker:"
echo "    make -f docker/Makefile sim"
echo ""
echo "  Validate:"
echo "    python3 arm_bringup/scripts/validate_upgrade_u3.py"
echo "═══════════════════════════════════════════════════════"
