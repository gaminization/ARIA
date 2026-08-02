# ARIA Upgrade U1 — LLM Planning & Reasoning

> **Upgrade U1** replaces the rule-based `PlanningAgent` and template-based `DialogueAgent` with local LLM-powered versions using [Ollama](https://ollama.ai). The system gracefully degrades to the original rule-based planner when the LLM is unavailable.

---

## Hardware Requirements

| Component | Spec | Notes |
|-----------|------|-------|
| GPU | NVIDIA RTX 5060 8GB (or any 8GB+ VRAM) | Required for LLM inference |
| CPU | Intel i7-13700HX (or equivalent) | Sentence-transformers run on CPU |
| RAM | 16GB+ | LLM + ROS2 + Gazebo |
| Storage | ~15GB free | Ollama models: ~5GB each |

### VRAM Budget

| Component | VRAM Usage | When Active |
|-----------|-----------|-------------|
| Llama 3.1 8B Q4 | ~5.0 GB | During planning only |
| Qwen 2.5-VL 7B Q4 | ~5.2 GB | During visual planning only |
| YOLOv8n | ~0.5 GB | Always (perception) |
| Depth-Anything v2 | ~1.5 GB | Always (depth estimation) |
| **Total budget** | **8.0 GB** | LLM loads/unloads dynamically |

> **Critical**: LLM and perception models cannot run simultaneously at full capacity. The system automatically unloads the LLM after plan generation to free VRAM for execution.

---

## Installation

### Quick Setup

```bash
cd ~/Projects/ARIA
bash arm_planner/scripts/upgrade_u1_install.sh
```

### Manual Setup

```bash
# 1. Install Ollama
curl -fsSL https://ollama.ai/install.sh | sh

# 2. Start Ollama server
ollama serve &

# 3. Pull models (5GB each — may take a while)
ollama pull llama3.1:8b-instruct-q4_K_M
ollama pull qwen2.5-vl:7b-instruct-q4_K_M

# 4. Install Python dependencies
pip install sentence-transformers aiohttp pyyaml

# 5. Rebuild ARIA
cd ~/Projects/ARIA
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 6. Verify
source install/setup.bash
python3 arm_bringup/scripts/validate_upgrade_u1.py
```

---

## Architecture

```
                        ┌──────────────────────────┐
                        │   Natural Language Cmd    │
                        └────────────┬─────────────┘
                                     │
                        ┌────────────▼─────────────┐
                        │   LLM Planning Agent     │
                        │   ┌────────────────────┐ │
                        │   │ Ollama Available?   │ │
                        │   └─────┬────────┬─────┘ │
                        │     Yes │        │ No     │
                        │   ┌─────▼──┐  ┌──▼─────┐ │
                        │   │  LLM   │  │ Rule-  │ │
                        │   │ Planner│  │ Based  │ │
                        │   └─────┬──┘  └──┬─────┘ │
                        │         │        │        │
                        │   ┌─────▼────────▼─────┐ │
                        │   │   Plan Validator    │ │
                        │   └─────────┬──────────┘ │
                        └─────────────┬────────────┘
                                      │
              ┌───────────────────────┼───────────────────────┐
              │                       │                       │
     ┌────────▼──────┐     ┌─────────▼────────┐   ┌─────────▼────────┐
     │  Simple Task  │     │  Complex Task    │   │  Ambiguous Task  │
     │  (< 5 steps)  │     │  (5+ steps)      │   │  (low confidence)│
     │               │     │                  │   │                  │
     │ Direct Plan   │     │ Tree-of-Thought  │   │ Ask User via     │
     │ Execution     │     │ Multi-Candidate  │   │ LLM Dialogue     │
     └───────────────┘     └──────────────────┘   └──────────────────┘
```

---

## Ollama Commands Quick Reference

```bash
# Server management
ollama serve                         # Start server (background: add &)
ollama list                          # List downloaded models
ollama ps                            # Show running/loaded models

# Model management
ollama pull llama3.1:8b-instruct-q4_K_M    # Download model
ollama rm llama3.1:8b-instruct-q4_K_M      # Remove model
ollama show llama3.1:8b-instruct-q4_K_M    # Model info

# Quick test
ollama run llama3.1:8b-instruct-q4_K_M "What is robotics?"

# API endpoints (used by ARIA)
curl http://localhost:11434/api/tags        # List models (health check)
curl http://localhost:11434/api/generate -d '{"model":"llama3.1:8b-instruct-q4_K_M","prompt":"Hello"}'
```

---

## VRAM Management Guide

### Monitoring

```bash
# Real-time GPU usage
watch -n 1 nvidia-smi

# Check what Ollama has loaded
ollama ps

# Force unload all models
curl http://localhost:11434/api/generate -d '{"model":"llama3.1:8b-instruct-q4_K_M","keep_alive":0}'
```

### Automatic VRAM Management

The system handles VRAM automatically:

1. **Planning request received** → LLM loads into VRAM (~5GB, ~3s)
2. **Plan generated** → LLM unloaded from VRAM (keep_alive=0)
3. **Execution begins** → Full VRAM available for YOLO + depth
4. **Next planning request** → LLM loads again

This is controlled by `unload_after_planning: true` in `arm_planner/config/llm_config.yaml`.

### Manual Override

```yaml
# arm_planner/config/llm_config.yaml
unload_after_planning: false  # Keep LLM in VRAM (faster planning, less VRAM for perception)
```

---

## New Files

| File | Purpose |
|------|---------|
| `arm_planner/arm_planner/llm_client.py` | Async Ollama wrapper with VRAM management |
| `arm_planner/arm_planner/llm_planning_agent.py` | LLM-backed planning with rule-based fallback |
| `arm_planner/arm_planner/function_calling_orchestrator.py` | ReAct loop for LLM tool calling |
| `arm_planner/arm_planner/tree_of_thought_planner.py` | Multi-candidate plan generation and scoring |
| `arm_planner/arm_planner/episodic_memory.py` | Cross-session task recall with embeddings |
| `arm_planner/config/llm_config.yaml` | LLM configuration |
| `arm_planner/prompts/system_prompt_planner.txt` | Planning LLM system prompt |
| `arm_planner/prompts/system_prompt_dialogue.txt` | Dialogue LLM system prompt |
| `arm_agents/arm_agents/llm_dialogue_agent.py` | LLM-enhanced dialogue |
| `arm_bringup/launch/aria_full_u1.launch.py` | U1 launch file |
| `arm_bringup/scripts/validate_upgrade_u1.py` | Validation test suite |
| `arm_planner/scripts/upgrade_u1_install.sh` | Installation script |

---

## Launching

```bash
# Source workspace
source install/setup.bash

# Launch with LLM planning (U1)
ros2 launch arm_bringup aria_full_u1.launch.py

# Monitor planning mode
ros2 topic echo /aria/planning/mode

# Send a command
ros2 service call /aria/command arm_interfaces/srv/SendCommand \
  "{command: 'Pick up the red cube and place it in the white box'}"

# Watch chain of thought
ros2 topic echo /aria/state/task
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `Ollama unavailable` | Run `ollama serve &` or `systemctl start ollama` |
| `Model not found` | Run `ollama pull llama3.1:8b-instruct-q4_K_M` |
| `CUDA out of memory` | Unload other models: `ollama ps` then force unload |
| `Slow first planning call` | Model loading takes ~3s. Subsequent calls are faster if model stays loaded |
| `sentence-transformers import error` | `pip install sentence-transformers` |
| `Planning always falls back` | Check `ros2 topic echo /aria/planning/mode` — should show "llm" |
| `JSON parse errors in logs` | LLM sometimes generates invalid JSON. System retries automatically |
| `VRAM not freed` | Force unload: `curl localhost:11434/api/generate -d '{"model":"llama3.1:8b-instruct-q4_K_M","keep_alive":0}'` |
