#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# ARIA Upgrade U1 — LLM Planning Installation Script
# One-time setup for local LLM intelligence upgrade.
#
# Prerequisites:
#   - Pop!_OS 22.04 or Ubuntu 22.04
#   - NVIDIA GPU with CUDA (RTX 5060 recommended)
#   - ARIA Stages 1–4 complete and building
#
# What this script does:
#   1. Installs Ollama (local LLM inference server)
#   2. Pulls required models (llama3.1:8b, qwen2.5-vl:7b)
#   3. Installs Python dependencies
#   4. Verifies the installation
# ═══════════════════════════════════════════════════════════════
set -euo pipefail

echo "═══════════════════════════════════════════════════════"
echo "  ARIA Upgrade U1 — LLM Planning Installation"
echo "═══════════════════════════════════════════════════════"
echo ""

ARIA_DIR="${HOME}/Projects/ARIA"

# ── Colors ─────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

info()    { echo -e "${CYAN}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[  OK]${NC} $1"; }
warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
error()   { echo -e "${RED}[FAIL]${NC} $1"; }

# ══════════════════════════════════════════════════════════
# Step 1: Install Ollama
# ══════════════════════════════════════════════════════════
echo ""
info "Step 1/5: Installing Ollama..."

if command -v ollama &> /dev/null; then
    success "Ollama already installed: $(ollama --version 2>/dev/null || echo 'version unknown')"
else
    info "Downloading and installing Ollama..."
    curl -fsSL https://ollama.ai/install.sh | sh
    success "Ollama installed"
fi

# Start Ollama service if not running
if ! curl -s http://localhost:11434/api/tags &> /dev/null; then
    info "Starting Ollama service..."
    ollama serve &
    sleep 3
fi

if curl -s http://localhost:11434/api/tags &> /dev/null; then
    success "Ollama server running at http://localhost:11434"
else
    error "Ollama server failed to start"
    echo "  Try: systemctl start ollama"
    echo "  Or:  ollama serve &"
fi

# ══════════════════════════════════════════════════════════
# Step 2: Pull LLM Models
# ══════════════════════════════════════════════════════════
echo ""
info "Step 2/5: Pulling LLM models (this may take a while)..."
echo ""
echo "  Model 1: llama3.1:8b-instruct-q4_K_M (~5GB)"
echo "  Model 2: qwen2.5-vl:7b-instruct-q4_K_M (~5GB)"
echo ""
echo "  TIP: If this is slow, run overnight with:"
echo "    ollama pull llama3.1:8b-instruct-q4_K_M"
echo "    ollama pull qwen2.5-vl:7b-instruct-q4_K_M"
echo ""

info "Pulling llama3.1:8b-instruct-q4_K_M..."
if ollama pull llama3.1:8b-instruct-q4_K_M 2>/dev/null; then
    success "llama3.1:8b pulled"
else
    warn "Failed to pull llama3.1 — may need manual download"
    warn "Run: ollama pull llama3.1:8b-instruct-q4_K_M"
fi

info "Pulling qwen2.5-vl:7b-instruct-q4_K_M..."
if ollama pull qwen2.5-vl:7b-instruct-q4_K_M 2>/dev/null; then
    success "qwen2.5-vl:7b pulled"
else
    warn "Failed to pull qwen2.5-vl — may need manual download"
    warn "Run: ollama pull qwen2.5-vl:7b-instruct-q4_K_M"
fi

echo ""
info "Available models:"
ollama list 2>/dev/null || echo "  (could not list models)"

# ══════════════════════════════════════════════════════════
# Step 3: Install Python Dependencies
# ══════════════════════════════════════════════════════════
echo ""
info "Step 3/5: Installing Python dependencies..."

pip install --quiet \
    sentence-transformers \
    aiohttp \
    pyyaml \
    2>/dev/null && success "Python deps installed" || {
    warn "Some Python deps may have failed"
    echo "  Manual install:"
    echo "    pip install sentence-transformers aiohttp pyyaml"
}

# ══════════════════════════════════════════════════════════
# Step 4: Rebuild ARIA
# ══════════════════════════════════════════════════════════
echo ""
info "Step 4/5: Rebuilding ARIA workspace..."

if [ -d "${ARIA_DIR}" ]; then
    cd "${ARIA_DIR}"
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install 2>&1 | tail -5
    success "ARIA rebuilt"
else
    warn "ARIA directory not found at ${ARIA_DIR}"
    echo "  Please rebuild manually:"
    echo "    cd ~/Projects/ARIA && colcon build --symlink-install"
fi

# ══════════════════════════════════════════════════════════
# Step 5: Verify Installation
# ══════════════════════════════════════════════════════════
echo ""
info "Step 5/5: Verifying installation..."

# Check Ollama
if curl -s http://localhost:11434/api/tags &> /dev/null; then
    success "Ollama server: running"
else
    error "Ollama server: not responding"
fi

# Check GPU
if command -v nvidia-smi &> /dev/null; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    GPU_MEM=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader 2>/dev/null | head -1)
    success "GPU: ${GPU_NAME} (${GPU_MEM})"
else
    warn "nvidia-smi not found — GPU status unknown"
fi

# Check Python imports
python3 -c "
import sys
errors = []

try:
    import aiohttp
except ImportError:
    errors.append('aiohttp')

try:
    import yaml
except ImportError:
    errors.append('pyyaml')

try:
    from sentence_transformers import SentenceTransformer
except ImportError:
    errors.append('sentence-transformers')

if errors:
    print(f'Missing: {errors}', file=sys.stderr)
    sys.exit(1)
print('All Python imports OK')
" && success "Python imports: OK" || error "Python imports: some missing"

# Test LLM client
python3 -c "
import asyncio
import sys
sys.path.insert(0, '${ARIA_DIR}/arm_planner')
from arm_planner.llm_client import OllamaClient
client = OllamaClient()
async def test():
    avail = await client.is_available()
    print(f'Ollama available: {avail}')
    if avail:
        models = await client.list_local_models()
        print(f'Local models: {models}')
    await client.close()
    return avail
result = asyncio.run(test())
sys.exit(0 if result else 1)
" 2>/dev/null && success "LLM client: working" || warn "LLM client: Ollama not available"

# ══════════════════════════════════════════════════════════
echo ""
echo "═══════════════════════════════════════════════════════"
echo "  ARIA Upgrade U1 Installation Complete"
echo "═══════════════════════════════════════════════════════"
echo ""
echo "  Next steps:"
echo "    1. Source your workspace:  source install/setup.bash"
echo "    2. Run validation:         python3 arm_bringup/scripts/validate_upgrade_u1.py"
echo "    3. Launch with LLM:        ros2 launch arm_bringup aria_full_u1.launch.py"
echo ""
echo "  VRAM Management:"
echo "    - LLM loads automatically when planning (5GB)"
echo "    - Unloads after plan generated (frees VRAM for YOLO)"
echo "    - If Ollama is down, system falls back to rule-based"
echo ""
echo "  Troubleshooting:"
echo "    - Model not loading?  ollama pull llama3.1:8b-instruct-q4_K_M"
echo "    - Ollama not running? ollama serve &"
echo "    - VRAM issues?        nvidia-smi  (check usage)"
echo ""
