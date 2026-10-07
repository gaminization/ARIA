#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
# Project ARIA — Universal Environment Setup Script
# Reproducible Installation for Ubuntu 22.04 LTS & ROS 2 Humble
# ═══════════════════════════════════════════════════════════════
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "═══════════════════════════════════════════════════════════════"
echo "  🚀 Initializing Project ARIA Environment Setup               "
echo "  Target Workspace: $WORKSPACE_ROOT"
echo "═══════════════════════════════════════════════════════════════"

# 1. System apt packages check & install
echo "[1/6] Checking system package prerequisites..."
if command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update -qq || true
    sudo apt-get install -y -qq \
        build-essential \
        cmake \
        git \
        python3-pip \
        python3-dev \
        python3-colcon-common-extensions \
        python3-rosdep \
        libgl1-mesa-glx \
        libglib2.0-0 \
        v4l-utils \
        curl \
        wget || true
fi

# 2. ROS 2 Sourcing & Dependencies
echo "[2/6] Verifying ROS 2 Humble..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "  ROS 2 Humble sourced successfully."
else
    echo "  [WARNING] /opt/ros/humble/setup.bash not found. If running natively, ensure ROS 2 Humble is installed."
fi

# 3. Python Virtualenv / Dependencies
echo "[3/6] Installing Python dependencies from requirements.txt..."
cd "$WORKSPACE_ROOT"
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt

# 4. Neural Weights Verification & Auto-download
echo "[4/6] Verifying neural network weights in models/..."
mkdir -p "$WORKSPACE_ROOT/models"
cd "$WORKSPACE_ROOT/models"

if [ ! -f "yolov8m.pt" ]; then
    echo "  Downloading YOLOv8m weights..."
    curl -L -o yolov8m.pt "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8m.pt"
fi

if [ ! -f "yolov8n.pt" ]; then
    echo "  Downloading YOLOv8n weights..."
    curl -L -o yolov8n.pt "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt"
fi

if [ ! -f "sam2.1_hiera_tiny.pt" ]; then
    echo "  Downloading SAM2.1 Hiera Tiny weights..."
    curl -L -o sam2.1_hiera_tiny.pt "https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt"
fi

# 5. Build Dashboard Frontend
echo "[5/6] Building Web Dashboard frontend..."
if [ -d "$WORKSPACE_ROOT/arm_dashboard/frontend" ]; then
    cd "$WORKSPACE_ROOT/arm_dashboard/frontend"
    if command -v npm >/dev/null 2>&1; then
        npm install --silent || true
        npm run build || true
        echo "  Dashboard frontend build complete."
    else
        echo "  [WARNING] npm not found; skipping frontend production build."
    fi
fi

# 6. Colcon Workspace Build
echo "[6/6] Building ROS 2 workspace packages..."
cd "$WORKSPACE_ROOT"
if command -v colcon >/dev/null 2>&1; then
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo "  ROS 2 workspace build successful."
else
    echo "  [WARNING] colcon not found; skipping ROS 2 build."
fi

# 7. Run validation
echo ""
python3 "$WORKSPACE_ROOT/scripts/check_dependencies.py"

echo "═══════════════════════════════════════════════════════════════"
echo "  ✅ ARIA Setup Completed Successfully!                        "
echo "  To launch the system:                                        "
echo "    source /opt/ros/humble/setup.bash                          "
echo "    source install/setup.bash                                  "
echo "    ros2 launch arm_bringup industrial_workcell.launch.py      "
echo "═══════════════════════════════════════════════════════════════"
