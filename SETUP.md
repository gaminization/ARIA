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

## 🚀 Running Project ARIA

### Option 1: Universal One-Command Launch (Recommended)
Launch the entire system (Gazebo simulation, 15 ROS 2 lifecycle agents, controllers, vision pipeline, and telemetry dashboard):

```bash
# 1. Full system launch (headless simulation + dashboard)
./launch.sh

# 2. Launch with Gazebo Classic 3D GUI window
./launch.sh --gui

# 3. Launch specific simulation worlds:
./launch.sh -w tester --gui      # Tester workspace (default)
./launch.sh -w industrial --gui  # Industrial workcell with conveyor belt & bins
./launch.sh -w workspace --gui   # Tabletop workspace
```

### Option 2: Direct Foreground Startup
To launch the complete industrial workcell with 5 conveyor workpieces in the foreground:
```bash
./scripts/aria_start.sh          # With Gazebo 3D GUI
./scripts/aria_start.sh --headless # Headless mode
```

---

## 📊 Accessing the Control Center Dashboard

Once launched, access the interactive web console:
* **Web Operator GUI:** [http://localhost:8000](http://localhost:8000) (Production FastAPI + built React console)
* **REST API Documentation:** [http://localhost:8000/docs](http://localhost:8000/docs) (Interactive Swagger UI)
* **Real-time State Bus:** `ws://localhost:8000/ws/state` (10 Hz JSON telemetry)
* **Live Camera Stream:** `ws://localhost:8000/ws/cameras` (30 FPS dual overhead + wrist feeds)

*(Optional) For live React frontend development with hot-reloading:*
```bash
cd arm_dashboard/frontend && npm run dev
# Accessible at http://localhost:5173
```

---

## 🛠️ System Monitoring & Process Management

```bash
# Check status of ROS 2 nodes, topics, controllers, and dashboard
./launch.sh --status

# Attach to background tmux logs
tmux attach -t aria_sim   # Simulation & agent output
tmux attach -t aria_dash  # Dashboard backend output

# Gracefully terminate all ARIA, Gazebo, and dashboard processes
./launch.sh --stop
# (or directly: ./scripts/aria_stop.sh)
```

---

## 🔬 Modular Manual Launch (For Developers)

```bash
# Terminal 1: Simulation + ROS 2 Multi-Agent Stack
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm_bringup aria_full_u3.launch.py world:=aria_tester_workspace.world

# Terminal 2: Telemetry Dashboard Backend
source install/setup.bash
python3 arm_dashboard/app.py
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
