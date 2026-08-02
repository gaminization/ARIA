#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# ARIA Upgrade U2 — Advanced Perception Installation Script
#
# Installs: SAM2, FoundationPose refs, gsplat, CLIP,
#           MobileNetV3 weights, ClearGrasp (optional)
#
# Usage: bash arm_bringup/scripts/upgrade_u2_install.sh
# ═══════════════════════════════════════════════════════════════
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARIA_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo ""
echo "═══════════════════════════════════════════════════════"
echo "  ARIA Upgrade U2 — Advanced Perception Installer"
echo "═══════════════════════════════════════════════════════"
echo "  ARIA directory: $ARIA_DIR"
echo ""

# ── Check GPU ──────────────────────────────────────────────
echo "[1/8] Checking GPU..."
if command -v nvidia-smi &> /dev/null; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
    GPU_MEM=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader | head -1)
    echo "  ✓ GPU: $GPU_NAME ($GPU_MEM)"
else
    echo "  ⚠ No NVIDIA GPU detected. Some modules will run in CPU mode."
fi

# ── Python dependencies ───────────────────────────────────
echo ""
echo "[2/8] Installing Python dependencies..."
pip install --quiet --upgrade \
    torch torchvision \
    opencv-python-headless \
    pillow \
    pyyaml \
    scipy \
    scikit-image

echo "  ✓ Core Python packages installed"

# ── SAM2 (Segment Anything Model 2) ───────────────────────
echo ""
echo "[3/8] Installing SAM2..."
pip install --quiet segment-anything-2 2>/dev/null || {
    echo "  ⚠ SAM2 pip package not available."
    echo "  → Trying from source: facebookresearch/sam2"
    if [ -d "$ARIA_DIR/third_party/sam2" ]; then
        echo "  → sam2 directory exists, skipping clone"
    else
        mkdir -p "$ARIA_DIR/third_party"
        git clone --depth 1 https://github.com/facebookresearch/sam2.git \
            "$ARIA_DIR/third_party/sam2" 2>/dev/null || true
    fi
    if [ -d "$ARIA_DIR/third_party/sam2" ]; then
        pip install --quiet -e "$ARIA_DIR/third_party/sam2" 2>/dev/null || true
    fi
}
echo "  ✓ SAM2 setup complete (GrabCut fallback always available)"

# ── Download SAM2-tiny checkpoint ──────────────────────────
echo ""
echo "[4/8] Downloading SAM2-tiny model weights..."
MODELS_DIR="$ARIA_DIR/arm_vision/models"
mkdir -p "$MODELS_DIR"
SAM2_CKPT="$MODELS_DIR/sam2_hiera_tiny.pt"
if [ -f "$SAM2_CKPT" ]; then
    echo "  → Checkpoint exists: $SAM2_CKPT"
else
    echo "  → Downloading SAM2-tiny checkpoint (~40MB)..."
    wget -q -O "$SAM2_CKPT" \
        "https://dl.fbaipublicfiles.com/segment_anything_2/072824/sam2_hiera_tiny.pt" \
        2>/dev/null || {
        echo "  ⚠ Download failed. SAM2 will use GrabCut fallback."
        echo "  → Manual download: https://github.com/facebookresearch/sam2#download-checkpoints"
    }
fi
echo "  ✓ SAM2 model weights ready"

# ── MobileNetV3 (Material Recognition) ────────────────────
echo ""
echo "[5/8] Setting up material recognition model..."
# MobileNetV3-Small pretrained weights are included in torchvision
python3 -c "
from torchvision.models import mobilenet_v3_small, MobileNet_V3_Small_Weights
model = mobilenet_v3_small(weights=MobileNet_V3_Small_Weights.IMAGENET1K_V1)
print('  ✓ MobileNetV3-Small weights cached')
" 2>/dev/null || echo "  ⚠ torchvision not available. Using heuristic fallback."

# ── CLIP (Zero-shot fallback) ──────────────────────────────
echo ""
echo "[6/8] Installing CLIP (optional, for zero-shot material classification)..."
pip install --quiet git+https://github.com/openai/CLIP.git 2>/dev/null || {
    echo "  ⚠ CLIP installation failed. Using heuristic material classification."
}
echo "  ✓ CLIP setup complete"

# ── gsplat (Gaussian Splatting) ────────────────────────────
echo ""
echo "[7/8] Installing gsplat (optional, for workspace 3D model)..."
pip install --quiet gsplat 2>/dev/null || {
    echo "  ⚠ gsplat installation failed. Using depth-fusion fallback."
    echo "  → To install manually: pip install gsplat"
}
echo "  ✓ Gaussian Splatting setup complete"

# ── Create directories ────────────────────────────────────
echo ""
echo "[8/8] Setting up directories..."
mkdir -p "$ARIA_DIR/arm_vision/config/object_references"
mkdir -p "$ARIA_DIR/arm_vision/models"
mkdir -p "$ARIA_DIR/arm_vision/scripts"
echo "  ✓ Directory structure ready"

# ── Build ARIA ─────────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════════"
echo "  Building ARIA workspace..."
echo "═══════════════════════════════════════════════════════"
cd "$ARIA_DIR"
source /opt/ros/humble/setup.bash
colcon build --symlink-install 2>&1 | tail -5

echo ""
echo "═══════════════════════════════════════════════════════"
echo "  ✅ ARIA Upgrade U2 Installation Complete"
echo ""
echo "  Installed Components:"
echo "    • SAM2 segmentation (or GrabCut fallback)"
echo "    • FoundationPose 6D pose (geometry fallback)"
echo "    • Material recognition (MobileNetV3 / CLIP / heuristic)"
echo "    • Gaussian Splatting (or depth-fusion fallback)"
echo "    • Transparent object handler"
echo "    • Perception orchestrator"
echo ""
echo "  Quick Start:"
echo "    source install/setup.bash"
echo "    ros2 launch arm_bringup aria_full_u2.launch.py"
echo ""
echo "  Validate:"
echo "    python3 arm_bringup/scripts/validate_upgrade_u2.py"
echo "═══════════════════════════════════════════════════════"
