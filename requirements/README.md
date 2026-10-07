# ARIA Dependency Architecture

This directory houses modular Python requirement definitions for Project ARIA.

## Hierarchy & Subsets

| File | Purpose | Key Libraries |
| :--- | :--- | :--- |
| [`requirements.txt`](../requirements.txt) | **Master Unified Requirements** | All production dependencies across all subsystems |
| [`requirements-dev.txt`](../requirements-dev.txt) | **Developer & Testing Suite** | Pytest, coverage, linters, pre-commit |
| [`requirements_core.txt`](./requirements_core.txt) | Kinematics & Robotics | `numpy`, `scipy`, `spatialmath`, `roboticstoolbox`, `ikpy` |
| [`requirements_vision.txt`](./requirements_vision.txt) | Computer Vision & Depth | `opencv`, `torch`, `torchvision`, `ultralytics`, `sam2` |
| [`requirements_vla.txt`](./requirements_vla.txt) | VLA & Autonomous Learning | `transformers`, `lerobot`, `clip`, `wandb`, `mlflow` |
| [`requirements_dashboard.txt`](./requirements_dashboard.txt) | Dashboard Backend | `fastapi`, `uvicorn`, `websockets`, `pydantic` |
| [`requirements_stage2.txt`](./requirements_stage2.txt) | Historical Stage 2 baseline | Preserved for reproducibility |
| [`requirements_stage3.txt`](./requirements_stage3.txt) | Historical Stage 3 baseline | Preserved for reproducibility |
