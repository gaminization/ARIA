# Project ARIA: Quick Reproduction & Setup Guide

This document provides quick setup instructions for Project ARIA. For the exhaustive dependency matrix and architecture breakdown, see [`docs/SETUP_AND_DEPENDENCIES.md`](docs/SETUP_AND_DEPENDENCIES.md).

---

## ⚡ 1-Minute Quickstart (Ubuntu 22.04 LTS + ROS 2 Humble)

```bash
# 1. Clone workspace
git clone https://github.com/gaminization/ARIA.git
cd ARIA

# 2. Run universal environment setup (installs system pkgs, pip requirements, builds ROS2 & frontend)
chmod +x scripts/setup_environment.sh
./scripts/setup_environment.sh

# 3. Validate system alignment
python3 scripts/check_dependencies.py
```

---

## 🚀 Running the Simulation

```bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch Industrial Workcell Simulation (Gazebo + ARIA Multi-Agent Stack)
ros2 launch arm_bringup industrial_workcell.launch.py
```

To run the live telemetry dashboard:
```bash
./scripts/aria_launch_full.sh
# Navigate to http://localhost:3000
```

---

## 🐳 Docker Deployment (Any Linux / Mac / Windows WSL2)

```bash
# Build and run full stack in Docker
docker compose up

# With local Ollama LLM reasoning
docker compose --profile llm up
```

---

## 📖 Complete Documentation Index

* 🛠️ **System Setup & Dependencies:** [`docs/SETUP_AND_DEPENDENCIES.md`](docs/SETUP_AND_DEPENDENCIES.md)
* 🧠 **System & Multi-Agent Architecture:** [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)
* 🦾 **Hardware Specification & Schematics:** [`docs/HARDWARE_SPECIFICATION.md`](docs/HARDWARE_SPECIFICATION.md)
* 👁️ **Perception & Grounding (No Depth Cam):** [`docs/PERCEPTION_AND_GROUNDING.md`](docs/PERCEPTION_AND_GROUNDING.md)
* 📐 **Kinematics & Control (Analytical IK):** [`docs/KINEMATICS_AND_CONTROL.md`](docs/KINEMATICS_AND_CONTROL.md)
* 🎯 **Agents & Skills Engine:** [`docs/AGENTS_AND_SKILLS.md`](docs/AGENTS_AND_SKILLS.md)
* 🗄️ **World Model & Failure Recovery:** [`docs/WORLD_MODEL_AND_FAILURE_RECOVERY.md`](docs/WORLD_MODEL_AND_FAILURE_RECOVERY.md)
* 📊 **Telemetry Dashboard:** [`docs/DASHBOARD_AND_TELEMETRY.md`](docs/DASHBOARD_AND_TELEMETRY.md)
* 🚀 **AI Upgrades & Roadmap:** [`docs/AI_UPGRADES_AND_ROADMAP.md`](docs/AI_UPGRADES_AND_ROADMAP.md)
* 📜 **Historical Stage Reports & Reviews:** [`docs/historical/README.md`](docs/historical/README.md)
