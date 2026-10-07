# ARIA Upgrade U3 — Infrastructure

Production infrastructure for the ARIA robot arm system.

## Overview

U3 adds five infrastructure pillars:
- **Docker**: One-command deployment
- **CI/CD**: Automated testing on every commit
- **Experiment Tracking**: MLflow for logging benchmarks, skills, training
- **Bag Recording**: Automatic session capture with failure investigation
- **Object Database**: Central knowledge base of known objects

## Components

### Docker Containerization

| Image | Purpose | GPU |
|-------|---------|-----|
| `aria:base` | Full ROS2 + ML stack | ✓ |
| `aria:sim` | + Gazebo GUI support | ✓ |
| `aria:headless` | CI/CD testing | ✗ |
| `aria:dashboard` | Dashboard on Alpine | ✗ |

```bash
# Build all images
make -f docker/Makefile build

# Run simulation + dashboard
make -f docker/Makefile sim

# Run hardware mode
make -f docker/Makefile hardware

# MLflow tracking
make -f docker/Makefile tracking

# CI tests in container
make -f docker/Makefile ci
```

**Docker Compose services:**

| Service | Port | Profile |
|---------|------|---------|
| `ros2_core` | host network | default |
| `dashboard` | 3000 | default |
| `rosbridge` | 9090 | default |
| `experiment_tracker` | 5000 | `tracking` |
| `ollama` | 11434 | `llm` |

### CI/CD Pipeline

```yaml
# .github/workflows/aria_ci.yml
Triggers: push, PR, nightly schedule
Jobs:
  1. Build — colcon build all packages
  2. Unit Tests — pytest (no GPU)
  3. Import Check — verify all modules
  4. Benchmark — nightly on self-hosted GPU
```

Benchmark comparison detects regressions > 10%:
```
⚠️ REGRESSION: IK success rate dropped from 97.5% to 91.2%
```

### Experiment Tracking

Backend options: MLflow (default, local), W&B (cloud), None (offline).

```python
from arm_learning.experiment_tracker import get_tracker

tracker = get_tracker()
tracker.log_ik_benchmark(results)
tracker.log_skill_performance('pick_up', result)
tracker.log_session_summary(session)
```

Config: `arm_learning/config/tracking_config.yaml`

### Bag Recording

Three modes:

| Mode | Topics | Size/hr | Auto |
|------|--------|---------|------|
| STANDARD | joints + tasks + detections | ~50 MB | Always |
| FULL | + cameras at 5fps | ~1.5 GB | Manual |
| INVESTIGATION | all topics | ~3 GB | On failure |

```bash
# Search recorded bags
python3 arm_learning/arm_learning/bag_indexer.py search \
  --object cup --failure-only

# Convert to training dataset
python3 arm_learning/arm_learning/bag_to_dataset.py convert \
  --input ~/aria_bags/2026-07-01/session_143022_standard \
  --output datasets/cups.hdf5 --success-only
```

Failure investigation: 30s circular pre-buffer + 60s post-failure
automatic recording with full topic capture.

### Object Model Database

```bash
# List known objects
python3 arm_planner/scripts/object_model_cli.py list

# Show details
python3 arm_planner/scripts/object_model_cli.py show blue_cup

# Register new object
python3 arm_planner/scripts/object_model_cli.py register \
  --name green_mug --mass 0.3 --material ceramic --fragile

# Export/import libraries
python3 arm_planner/scripts/object_model_cli.py export --output library.yaml
python3 arm_planner/scripts/object_model_cli.py import library.yaml

# Statistics
python3 arm_planner/scripts/object_model_cli.py stats
```

Pre-loaded with 10 default objects from simulation.

### Dashboard Additions

Three new components:
- **ObjectLibraryPanel**: Browse objects, success rates, affordances
- **ExperimentPanel**: MLflow embed, IK comparison, run history
- **BagRecorderPanel**: Live status bar, mode control, bag search

## Quick Start

```bash
# Install U3
bash arm_bringup/scripts/upgrade_u3_install.sh

# Validate
python3 arm_bringup/scripts/validate_upgrade_u3.py

# Launch full system with U3
ros2 launch arm_bringup aria_full_u3.launch.py

# Or with Docker
make -f docker/Makefile sim
```

## File Inventory (27 files)

| # | File | Purpose |
|---|------|---------|
| 1 | `docker/Dockerfile.base` | Multi-stage base image |
| 2 | `docker/Dockerfile.sim` | GUI-enabled sim image |
| 3 | `docker/Dockerfile.headless` | CI/CD testing image |
| 4 | `docker/Dockerfile.dashboard` | Lightweight dashboard |
| 5 | `docker/entrypoint.sh` | Container entrypoint |
| 6 | `docker-compose.yml` | Full stack orchestration |
| 7 | `docker-compose.ci.yml` | CI test configuration |
| 8 | `docker/Makefile` | Convenience targets |
| 9 | `docker/NVIDIA_SETUP.md` | GPU setup guide |
| 10 | `.github/workflows/aria_ci.yml` | CI pipeline |
| 11 | `.github/workflows/docker_build.yml` | Image build+push |
| 12 | `.github/scripts/compare_benchmarks.py` | Regression detection |
| 13 | `arm_learning/.../experiment_tracker.py` | Tracking interface |
| 14 | `arm_learning/config/tracking_config.yaml` | Tracking config |
| 15 | `arm_learning/.../bag_recorder_node.py` | Session recording |
| 16 | `arm_learning/.../bag_indexer.py` | Bag search+index |
| 17 | `arm_learning/.../bag_to_dataset.py` | HDF5 conversion |
| 18 | `arm_planner/.../object_model_db.py` | Object database |
| 19 | `arm_planner/scripts/object_model_cli.py` | Object CLI |
| 20 | `arm_planner/data/object_library_default.yaml` | Default objects |
| 21 | `arm_dashboard/.../ObjectLibraryPanel.jsx` | Object UI |
| 22 | `arm_dashboard/.../ExperimentPanel.jsx` | Experiment UI |
| 23 | `arm_dashboard/.../BagRecorderPanel.jsx` | Recording UI |
| 24 | `arm_bringup/.../upgrade_u3_install.sh` | Installer |
| 25 | `arm_bringup/.../validate_upgrade_u3.py` | Validation |
| 26 | `arm_bringup/.../aria_full_u3.launch.py` | Launch file |
| 27 | `README_UPGRADE_U3.md` | This file |

## Hardware Requirements

| Resource | Minimum |
|----------|---------|
| GPU | NVIDIA with ≥ 535 driver |
| Docker | ≥ 24.0 |
| Docker Compose | v2+ |
| Disk | ~15 GB for images |
| NVIDIA Container Toolkit | Required for GPU |
