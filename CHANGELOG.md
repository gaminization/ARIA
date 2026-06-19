# Changelog

All notable changes to ARIA are documented here.
Format: [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)
Versioning: [Semantic Versioning](https://semver.org/spec/v2.0.0.html)

---

## [Unreleased]

### Added
### Changed
### Fixed

---

## [v1.0.0] — Stage 4: Hardware Bridge

### Added
- ESP32 firmware with micro-ROS (WiFi + USB serial transport)
- Bidirectional servo sync: sim→real and real→sim
- Teach mode: physically demonstrate trajectories, replay in sim
- Hardware bringup wizard: interactive first-time setup CLI
- ADC position feedback (Option A: servo pot tap)
- `aria_cli.py`: unified command-line interface for all modes
- `servo_sync_node.py`: transparent sim↔real translation layer
- Hardware-specific calibration offset management
- `validate_stage4.py`: all hardware validation tests

### Changed
- `hardware.launch.py`: now complete (was skeleton in Stage 1)
- `aria_hardware_interface.cpp`: REAL mode fully implemented
- Dashboard: added hardware status indicators

### Fixed
- Joint position commands now sent atomically to prevent partial updates
- micro-ROS heartbeat timeout detection and reconnect logic

---

## [v0.4.0] — Stage 3b: World Model + Skills + Dashboard

### Added
- `world_model_db.py`: SQLite-backed persistent scene model
- 10 predefined skills: pick, place, push, pull, stack, sort,
  inspect, slide, roll, sweep
- In-hand manipulation: rotate, reposition, flip, slide_to_tip
- `vla_interface.py`: LeRobot, OpenVLA integration with benchmark
- FastAPI + React dashboard with live camera, joint viz, world map
- `memory_manager.py` and `skill_manager.py` 
- Object lifecycle tracking: Detected→Tracked→Lost→Recovered
- Skill performance tracking and auto-improvement
- `validate_stage3.py`: full pipeline validation

### Changed
- TaskManager now uses SkillManager for all skill dispatch
- AffordanceAgent integrated with WorldModelAgent for persistence

---

## [v0.3.0] — Stage 3a: Agents + Planning

### Added
- All 15 ARIA agents as ROS2 LifecycleNodes
- Unified state bus (VisionState, MemoryState, TaskState, HealthState)
- NLP task decomposition with command pattern library
- Chain-of-thought execution logging
- Confidence-based decisions with user approval fallback
- Failure classification and automatic recovery handling
- Attention mechanism for ROI focusing
- Active perception mode for seeking occluded objects

### Changed
- TaskManager refactored to orchestrate agents instead of direct function calls

---

## [v0.2.0] — Stage 2: IK + Perception Pipeline

### Added
- Analytical, RTB, IKPy, and Neural Network IK Solvers
- YOLOv8 object detection integration (`vision_node`)
- Depth-Anything depth estimation (`depth_node`)
- Coordinate transform to map camera pixels to base frame coordinates
- Collision avoidance logic
- `validate_stage2.py` validation script

### Changed
- Improved forward kinematics calculation accuracy
- Standardized camera image publishing at 30fps

---

## [v0.1.0] — Stage 1: Gazebo Foundation + Manual Control

### Added
- Complete URDF and xacro definitions for 5-DoF ARIA arm
- Gazebo classic simulation environment setup
- Hardware Abstraction Layer (HAL) interfaces
- Real-time keyboard teleoperation scripts
- GUI slider for manual joint control
- `validate_stage1.py` validation script

### Fixed
- Initial mesh colliders causing erratic physics behavior

---
