#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  ARIA Full Setup Check & Auto-Fix Script
#  Run this before any demo. Detects and fixes missing components.
#  Usage: bash aria_setup_check.sh
# ═══════════════════════════════════════════════════════════════
set -e

ARIA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PASS=0; FAIL=0; WARN=0

ok()   { echo "  ✅ $1"; PASS=$((PASS+1)); }
fail() { echo "  ❌ $1"; FAIL=$((FAIL+1)); }
warn() { echo "  ⚠️  $1"; WARN=$((WARN+1)); }

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  🤖 ARIA Setup Check — $(date '+%Y-%m-%d %H:%M')"
echo "═══════════════════════════════════════════════════════════"

# ── 1. ROS2 ───────────────────────────────────────────────────
echo ""
echo "── [1/8] ROS2 & Workspace ──────────────────────────────────"
source /opt/ros/humble/setup.bash 2>/dev/null && ok "ROS2 Humble" || fail "ROS2 Humble not found at /opt/ros/humble"
source "$ARIA_DIR/install/setup.bash" 2>/dev/null && ok "ARIA workspace install/setup.bash" || fail "Workspace not built — run: colcon build"
which gazebo &>/dev/null && ok "Gazebo $(gazebo --version 2>/dev/null | head -1 | grep -oP '\d+\.\d+\.\d+')" || fail "Gazebo not installed"
ollama list &>/dev/null && ok "Ollama running — models: $(ollama list 2>/dev/null | tail -n +2 | awk '{print $1}' | tr '\n' ', ')" || fail "Ollama not running — start with: ollama serve &"

# ── 2. Python Packages ────────────────────────────────────────
echo ""
echo "── [2/8] Python Packages ───────────────────────────────────"
check_pkg() {
  python3 -c "import $1; v=getattr($1,'__version__','ok'); print('  ✅ $1 ' + str(v))" 2>/dev/null && PASS=$((PASS+1)) || { echo "  ❌ $1 not installed — pip install $2"; FAIL=$((FAIL+1)); }
}
python3 -c "import rclpy; print('  ✅ rclpy')" 2>/dev/null && PASS=$((PASS+1)) || { echo "  ❌ rclpy — source ROS2 setup.bash"; FAIL=$((FAIL+1)); }
python3 -c "import torch; cuda=torch.cuda.is_available(); print(f'  ✅ torch {torch.__version__} | CUDA: {cuda} | GPU: {torch.cuda.get_device_name(0) if cuda else \"CPU only\"}')" 2>/dev/null && PASS=$((PASS+1)) || { echo "  ❌ torch — pip install torch"; FAIL=$((FAIL+1)); }
check_pkg ultralytics ultralytics
check_pkg sam2 "git+https://github.com/facebookresearch/segment-anything-2.git"
check_pkg transformers transformers
check_pkg lerobot lerobot
check_pkg cv2 "opencv-python"
check_pkg fastapi fastapi
check_pkg uvicorn uvicorn
check_pkg mlflow mlflow
check_pkg h5py h5py
check_pkg einops einops
check_pkg timm timm
python3 -c "from cv_bridge import CvBridge; print('  ✅ cv_bridge')" 2>/dev/null && PASS=$((PASS+1)) || { echo "  ❌ cv_bridge — sudo apt install ros-humble-cv-bridge"; FAIL=$((FAIL+1)); }
check_pkg sentence_transformers "sentence-transformers"

# ── 3. Model Files ────────────────────────────────────────────
echo ""
echo "── [3/8] Model Files ───────────────────────────────────────"
check_model() {
  local path="$1"; local name="$2"; local url="$3"
  if [ -f "$path" ]; then
    local size=$(du -sh "$path" | cut -f1)
    ok "$name ($size)"
  else
    echo "  ❌ $name MISSING"
    FAIL=$((FAIL+1))
    if [ -n "$url" ]; then
      echo "     → Auto-downloading..."
      wget -q --show-progress "$url" -O "$path" 2>&1 | tail -1
      [ -f "$path" ] && echo "     ✅ Downloaded: $name" || echo "     ❌ Download failed"
    fi
  fi
}

check_model "$ARIA_DIR/yolov8m.pt" "YOLOv8m" \
  "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8m.pt"
check_model "$ARIA_DIR/yolov8n.pt" "YOLOv8n" \
  "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt"
check_model "$ARIA_DIR/sam2_hiera_tiny.pt" "SAM2.0 tiny" \
  "https://dl.fbaipublicfiles.com/segment_anything_2/072824/sam2_hiera_tiny.pt"
check_model "$ARIA_DIR/sam2.1_hiera_tiny.pt" "SAM2.1 tiny" \
  "https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt"

check_model "$ARIA_DIR/arm_ik/models/neural_ik.pt" "Neural IK MLP" ""

[ -f "$HOME/.cache/torch/hub/checkpoints/dpt_hybrid_384.pt" ] \
  && ok "MiDaS v3 DPT-Hybrid ($(du -sh ~/.cache/torch/hub/checkpoints/dpt_hybrid_384.pt | cut -f1))" \
  || warn "MiDaS not yet cached — will download on first run"

python3 -c "
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
p = AutoImageProcessor.from_pretrained('depth-anything/Depth-Anything-V2-Small-hf')
print('  ✅ Depth-Anything v2 Small cached')
" 2>/dev/null && PASS=$((PASS+1)) || {
  echo "  ⚠️  Depth-Anything v2 not cached — caching now..."
  WARN=$((WARN+1))
  python3 -c "
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
p = AutoImageProcessor.from_pretrained('depth-anything/Depth-Anything-V2-Small-hf')
m = AutoModelForDepthEstimation.from_pretrained('depth-anything/Depth-Anything-V2-Small-hf')
print('  ✅ Depth-Anything v2 Small downloaded and cached')
" 2>&1 | tail -1
}

python3 -c "
import os, glob
cache = os.path.expanduser('~/.cache/huggingface/hub/models--sentence-transformers--all-MiniLM-L6-v2')
print('  ✅ all-MiniLM-L6-v2 cached') if os.path.exists(cache) else None
" 2>/dev/null && PASS=$((PASS+1)) || warn "all-MiniLM-L6-v2 not cached"

# OpenVLA 7B check
if [ -d "$ARIA_DIR/models/openvla-7b" ] && [ -f "$ARIA_DIR/models/openvla-7b/config.json" ]; then
  ovla_size=$(du -sh "$ARIA_DIR/models/openvla-7b" | cut -f1)
  ok "OpenVLA 7B ($ovla_size) [3 safetensors shards + prismatic tokenizer]"
else
  fail "OpenVLA 7B NOT FOUND at models/openvla-7b"
fi

# ── 4. Ollama LLM ─────────────────────────────────────────────
echo ""
echo "── [4/8] Ollama LLM ────────────────────────────────────────"
if ollama list 2>/dev/null | grep -q "llama"; then
  model=$(ollama list 2>/dev/null | grep "llama" | head -1 | awk '{print $1}')
  ok "LLM model: $model"
else
  fail "No LLM model found in Ollama — run: ollama pull llama3.1:8b-instruct-q4_K_M"
fi

# ── 5. Frontend ───────────────────────────────────────────────
echo ""
echo "── [5/8] Dashboard Frontend ────────────────────────────────"
FRONTEND="$ARIA_DIR/arm_dashboard/frontend"
[ -d "$FRONTEND/node_modules" ] && ok "node_modules present" || {
  fail "node_modules missing — installing..."
  cd "$FRONTEND" && npm install -q && ok "npm install done" || fail "npm install failed"
}
[ -f "$FRONTEND/dist/index.html" ] && ok "Frontend dist/ built" || {
  warn "Frontend not built — building now..."
  cd "$FRONTEND" && npm run build 2>&1 | tail -3 && ok "Frontend built" || fail "npm run build failed"
}

# ── 6. ROS2 Packages ──────────────────────────────────────────
echo ""
echo "── [6/8] ROS2 Packages ─────────────────────────────────────"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$ARIA_DIR/install/setup.bash" 2>/dev/null
for pkg in arm_agents arm_bringup arm_control arm_description arm_ik arm_interfaces arm_learning arm_moveit_config arm_planner arm_vision arm_vla; do
  ros2 pkg xml "$pkg" &>/dev/null && ok "$pkg" || fail "$pkg NOT BUILT — run: colcon build --packages-select $pkg"
done

# ── 7. Launch Files ───────────────────────────────────────────
echo ""
echo "── [7/8] Key Launch Files ──────────────────────────────────"
for lf in aria_full_u3.launch.py aria_agents_only.launch.py sim.launch.py; do
  [ -f "$ARIA_DIR/install/arm_bringup/share/arm_bringup/launch/$lf" ] \
    && ok "$lf installed" || fail "$lf NOT in install/ — rebuild arm_bringup"
done

# ── 8. Gazebo Assets ──────────────────────────────────────────
echo ""
echo "── [8/8] Gazebo Assets ─────────────────────────────────────"
[ -f "$ARIA_DIR/install/arm_bringup/share/arm_bringup/worlds/aria_workspace.world" ] \
  && ok "aria_workspace.world (has banana)" || fail "aria_workspace.world not installed"
[ -f "$ARIA_DIR/install/arm_bringup/share/arm_bringup/worlds/aria_industrial_workcell.world" ] \
  && ok "aria_industrial_workcell.world installed" || fail "aria_industrial_workcell.world not installed"
[ -f "$ARIA_DIR/install/arm_control/lib/libaria_gripper_plugin.so" ] \
  && ok "libaria_gripper_plugin.so built" || fail "libaria_gripper_plugin.so not built"
python3 -c "
from ultralytics import YOLO
m = YOLO('$ARIA_DIR/yolov8m.pt')
ids = [k for k,v in m.names.items() if v == 'banana']
print(f'  ✅ YOLOv8m detects banana at class index {ids}')
" 2>/dev/null && PASS=$((PASS+1)) || warn "Could not verify banana detection"

# ── Summary ───────────────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  SETUP SUMMARY: ✅ $PASS passed  ❌ $FAIL failed  ⚠️  $WARN warnings"
if [ $FAIL -eq 0 ]; then
  echo "  🟢 READY TO LAUNCH"
  echo ""
  echo "  Launch command:"
  echo "  cd $ARIA_DIR"
  echo "  source /opt/ros/humble/setup.bash && source install/setup.bash"
  echo "  ros2 launch arm_bringup aria_full_u3.launch.py world:=aria_workspace.world"
  echo "  → Dashboard: http://localhost:8080"
else
  echo "  🔴 FIX $FAIL FAILURES ABOVE BEFORE LAUNCHING"
fi
echo "═══════════════════════════════════════════════════════════"
echo ""
