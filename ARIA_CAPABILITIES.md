# ARIA — System Capabilities & Testing Guide

This document provides a comprehensive overview of the Adaptive Robotic Intelligence Architecture (ARIA) 5-DoF robotic arm system. It details the capabilities achieved across Stages 1 through 3b, provides a complete command reference, and outlines exactly how to test and validate the system.

---

## 1. System Capabilities by Stage

### Stage 1: Gazebo Foundation & Manual Control
* **Physical Simulation**: Full Gazebo classic simulation of the 5-DoF Techno-Tirupati robotic arm with realistic physics, joint limits, and meshes.
* **Teleoperation**: Real-time manual control via keyboard commands (`W/S`, `A/D`, etc.) and a dedicated GUI slider node.
* **Safety Protocols**: Integrated E-stop system. When activated, all incoming joint commands are blocked until explicitly released.
* **Sensory Feed**: Simulated Logitech C270 (top view) and ESP32-CAM (wrist view) cameras publishing at 30fps and 15fps respectively, alongside an MPU6050 IMU publishing at 100Hz.
* **Named Poses**: Pre-programmed standard poses including `home`, `ready`, `folded`, and `inspect`.

### Stage 2: IK Solvers & Perception
* **Inverse Kinematics Benchmark**: Four interchangeable IK solvers (Analytical, IKPy, Robotics Toolbox, Neural Network) that can be benchmarked and auto-selected based on accuracy and speed.
* **YOLOv8 Vision**: Real-time object detection recognizing up to 6 simulated workspace objects.
* **Depth-Anything Integration**: Monocular depth estimation mapping 2D pixels to 3D base-frame coordinates.
* **Grasp Planning**: Automated, cable-aware motion planning that avoids predefined collision zones and computes approach/retreat trajectories.

### Stage 3a: Agent Intelligence & Planning
* **15-Agent Architecture**: A highly modular system where 15 specific AI agents (Vision, Depth, Tracking, Affordance, Planning, etc.) run as `LifecycleNodes`.
* **Natural Language Processing**: Translates high-level natural language commands (e.g., *"Put the red cube in the white box"*) into executable robotic subgoals.
* **Chain-of-Thought Logging**: Transparent decision-making process with confidence scoring. If confidence falls below 70%, the DialogueAgent requests human approval.
* **Failure Classification**: Automatically identifies failure modes (e.g., `MISSED_OBJECT`, `IK_FAILURE`, `COLLISION`) and triggers predefined recovery behaviors up to 3 times before aborting.

### Stage 3b: World Model, Skills, & Dashboard
* **Persistent World Model**: SQLite-backed memory (`world_model_db.py`) that tracks object lifecycle states (`DETECTED` → `TRACKED` → `LOST` → `RECOVERED`) across sessions.
* **Skill Library**: 10 predefined procedural skills (`pick`, `place`, `push`, `pull`, `stack`, `sort`, `inspect`, `slide`, `roll`, `sweep`).
* **In-Hand Manipulation**: Complex wrist actions utilizing IMU feedback to `rotate`, `reposition_grip`, `flip`, and `slide_to_tip`.
* **VLA Integration**: Abstraction layer for Vision-Language-Action models (LeRobot ACT, OpenVLA) with a built-in 10-task benchmark suite.
* **Real-time Dashboard**: FastAPI backend and React frontend providing a 10Hz state websocket stream, 30fps MJPEG camera feeds, and a dark glassmorphism GUI.

---

## 2. Command Reference

### Base Launch Commands
Launch the core systems. These must be running before you execute other commands.

| Command | Description | Stage |
|---------|-------------|-------|
| `ros2 launch arm_bringup sim.launch.py` | Launches Gazebo, RViz, and spawn the arm | 1 |
| `ros2 launch arm_bringup perception.launch.py` | Launches YOLO and Depth agents | 2 |
| `ros2 launch arm_bringup manual_control.launch.py` | Launches the GUI sliders for manual control | 1 |
| `ros2 launch arm_bringup full_stage2.launch.py` | Launches Sim + Perception + IK | 2 |
| `ros2 launch arm_bringup aria_full.launch.py` | Staged launch of the entire ARIA 3b system (25+ nodes) | 3 |

### Teleoperation & Utilities
| Command | Description | Stage |
|---------|-------------|-------|
| `ros2 run arm_control keyboard_control.py` | Terminal-based WASD teleoperation | 1 |
| `ros2 run arm_control joint_slider_gui.py` | Tkinter GUI for absolute joint angles | 1 |
| `ros2 run arm_ik ik_benchmark_node.py` | Runs the IK solver accuracy benchmark | 2 |
| `ros2 run arm_vla vla_benchmark.py` | Evaluates VLA performance across 10 tasks | 3b |

### Dashboard (Stage 3b)
| Command | Description |
|---------|-------------|
| `cd ~/Projects/ARIA/arm_dashboard/frontend && npm run dev` | Starts the React frontend UI on `localhost:3000` |
| `python3 ~/Projects/ARIA/arm_dashboard/app.py` | Starts the FastAPI backend on `localhost:8000` |

### Service Calls (via CLI)
You can directly invoke ARIA services from the terminal to bypass the agents.

* **Set Joint**: `ros2 service call /aria/set_joint arm_interfaces/srv/SetJoint "{joint_name: 'waist', angle_deg: 45.0, speed_deg_per_s: 30.0}"`
* **Go Named Pose**: `ros2 service call /aria/go_named_pose arm_interfaces/srv/GoNamedPose "{pose_name: 'ready'}"`
* **E-Stop**: `ros2 service call /aria/estop std_srvs/srv/Trigger "{}"`
* **Release E-Stop**: `ros2 service call /aria/release_estop std_srvs/srv/Trigger "{}"`

---

## 3. Comprehensive Testing Guide

To verify the system is working perfectly, you must run the automated validation scripts. Because these tests interact with physical simulation physics, **they must be run on your local machine with a graphical environment (X11/Wayland)** to support Gazebo.

### Setup (Run once before testing any stage)
Open a terminal and compile the workspace:
```bash
cd ~/Projects/ARIA
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Testing Stage 1
1. **Terminal 1**: Launch the simulator.
   ```bash
   source install/setup.bash
   ros2 launch arm_bringup sim.launch.py
   ```
   *(Wait for Gazebo to fully load and the arm to spawn).*
2. **Terminal 2**: Run the validator.
   ```bash
   source install/setup.bash
   python3 arm_bringup/scripts/validate_stage1.py
   ```
   **Expected**: 12/12 tests pass. The arm will dance around, reach limits, open/close gripper, and return to home.

### Testing Stage 2
1. **Terminal 1**: Launch the Stage 2 environment.
   ```bash
   source install/setup.bash
   ros2 launch arm_bringup full_stage2.launch.py
   ```
2. **Terminal 2**: Run the validator.
   ```bash
   source install/setup.bash
   python3 arm_bringup/scripts/validate_stage2.py
   ```
   **Expected**: Validates YOLO detection, depth estimation accuracy, IK solver selection, and collision avoidance zones.

### Testing Stage 3 (3a & 3b)
Stage 3 brings up the entire intelligence architecture. 

1. **Terminal 1**: Launch the full staged system.
   ```bash
   source install/setup.bash
   ros2 launch arm_bringup aria_full.launch.py
   ```
   *(Note: This staggers the launch of 25 nodes over 10 seconds. Wait until the terminal output stabilizes).*
2. **Terminal 2**: Run the Stage 3 validator.
   ```bash
   source install/setup.bash
   python3 arm_bringup/scripts/validate_stage3.py
   ```
   **Expected**: Verifies that all 15 agents are in the `ACTIVE` lifecycle state, tests the NLP pipeline, verifies SQLite World Model CRUD operations, tests the SkillManager rollback system, and pings the Dashboard API.

### Troubleshooting
* **"ModuleNotFoundError"**: Ensure you have run `source install/setup.bash` in *every* new terminal you open.
* **"Failed to create target... already exists"**: Clean your build directory (`rm -rf build/ install/ log/`) and rebuild.
* **Missing Vision Msgs**: Run `sudo apt install ros-humble-vision-msgs` if Stage 2 fails to import detection arrays.
