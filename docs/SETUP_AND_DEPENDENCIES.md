# Project ARIA: Setup, Dependencies & System Reproducibility

> **Target Platform:** Ubuntu 22.04 LTS (Jammy Jellyfish)  
> **ROS 2 Distribution:** Humble Hawksbill  
> **Simulation Engine:** Gazebo Classic 11.10.2+  
> **Reference Compute Envelope:** Lenovo LOQ (Intel Core i7-13700HX, NVIDIA GeForce RTX 5060 Laptop GPU 8GB VRAM, 16GB DDR5 RAM)  
> **Hardware Embodiment:** 5-DoF SG90 + MG995 Manipulator, Dual ESP32-CAM / C270, MPU6050 (<$120 BOM)

---

## 1. System Requirements & Hardware Budget

| Component | Minimum Specification | Recommended Reference Machine |
| :--- | :--- | :--- |
| **Operating System** | Ubuntu 22.04 LTS x86_64 | Ubuntu 22.04.4 LTS (Kernel 5.15 / 6.5) |
| **CPU** | 8 Cores / 16 Threads (x86_64) | Intel Core i7-13700HX (16 Cores / 24 Threads) |
| **GPU / Acceleration** | NVIDIA GPU with $\ge 6.0\,\text{GB}$ VRAM (CUDA 12.0+) | NVIDIA GeForce RTX 5060 Laptop (8.0 GB VRAM) |
| **System Memory (RAM)** | 16 GB DDR4/DDR5 | 16 GB DDR5 4800MHz |
| **Disk Storage** | 25 GB free SSD space | NVMe PCIe Gen 4 SSD |
| **Serial Bus / USB** | 2× USB 3.0 ports (Overhead C270 + ESP32) | FTDI / CH340 USB-UART bridges |

---

## 2. Complete Dependency Inventory

### A. System & ROS 2 Apt Packages
```bash
sudo apt-get update && sudo apt-get install -y \
  build-essential cmake git curl wget v4l-utils \
  python3-pip python3-dev python3-colcon-common-extensions python3-rosdep \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-moveit \
  ros-humble-moveit-ros-planning \
  ros-humble-moveit-ros-move-group \
  ros-humble-rosbridge-server \
  ros-humble-cv-bridge \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-tf2-tools
```

### B. Python Environment Libraries
Project ARIA maintains strict dependency isolation via [`requirements.txt`](../requirements.txt):

| Category | Package | Version | Purpose |
| :--- | :--- | :--- | :--- |
| **Core Numerics** | `numpy` | `^1.24.0, <2.0` | Matrix transformations and vector arithmetic |
| **Kinematics & Optimization** | `scipy` | `^1.10.0` | Spatial transformations and Levenberg-Marquardt |
| | `sympy` | `^1.12.0` | Closed-form Jacobian and Craig MDH analytical proofs |
| | `spatialmath-python` | `^1.1.0` | $SE(3)$ / $SO(3)$ lie-group kinematics |
| | `roboticstoolbox-python` | `^1.1.0` | SerialLink comparative benchmark baseline |
| | `ikpy` | `^3.3.3` | URDF-based numerical optimization baseline |
| | `trimesh` | `^4.0.0` | 3D mesh processing and collision convex hulls |
| **Vision & Foundation Depth** | `opencv-python` | `^4.8.0` | Overhead ray-plane homography & AprilTag calibration |
| | `pillow` | `^10.0.0` | High-fidelity image manipulation |
| | `torch` | `^2.0.0` | PyTorch GPU execution (CUDA 12) |
| | `torchvision` | `^0.15.0` | Image transforms |
| | `ultralytics` | `^8.0.0` | Real-time YOLOv8m instance detection |
| | `timm` | `^0.9.0` | Vision transformer backbone for Depth-Anything v2 |
| | `sam2` | `git+main` | Segment Anything v2 promptable mask extraction |
| **Autonomous VLA & Reasoning** | `transformers` | `^4.40.0` | OpenVLA-7B and local HuggingFace inference |
| | `sentence-transformers` | `^2.2.0` | Semantic vector space for world model retrieval |
| | `clip` | `git+main` | Zero-shot visual affordance grounding |
| | `lerobot` | `^0.1.0` | Trajectory dataset distillation and policy execution |
| **Telemetry & Dashboard** | `fastapi` | `^0.109.0` | Asynchronous telemetry API and control bridge |
| | `uvicorn` | `^0.27.0` | Production ASGI web server |
| | `websockets` | `^12.0` | Low-latency state bus streaming to web GUI |
| | `pydantic` | `^2.0.0` | Strict type schema validation |
| | `pyserial` | `^3.5` | Hardware UART bridge to ESP32 micro-ROS |

### C. Node.js & Web GUI Dependencies
Located in [`arm_dashboard/frontend/package.json`](../arm_dashboard/frontend/package.json):
* **Node.js:** `v18.0.0+` (Tested on `v24.16.0`)
* **Framework:** React 18, Vite 5, TailwindCSS / Vanilla CSS, Lucide React icons, RoSLib.js.

### D. Embedded Microcontroller Firmware
Located in [`esp32_firmware/platformio.ini`](../esp32_firmware/platformio.ini):
* **Platform:** Espressif 32 (`espressif32@6.5.0`)
* **Framework:** Arduino
* **Libraries:** `micro_ros_arduino@^2.0.7`, `Adafruit MPU6050@^2.2.4`, `Adafruit BusIO@^1.14.5`.

---

## 3. Automated Reproduction Walkthrough

### Step 1: Clone and Run the Universal Setup Script
From any fresh Ubuntu 22.04 LTS workstation:
```bash
git clone https://github.com/gaminization/ARIA.git
cd ARIA

# Make setup script executable and run
chmod +x scripts/setup_environment.sh
./scripts/setup_environment.sh
```

### Step 2: Validate Environment Integrity
Run the automated validation tool to confirm that all packages, drivers, and model weights are aligned:
```bash
python3 scripts/check_dependencies.py
```
*Expected Result:*
```
═══════════════════════════════════════════════════════════════
  ✅ ARIA ENVIRONMENT VALIDATION: ALL CRITICAL SYSTEMS READY
  The project is fully configured and ready for simulation & execution.
═══════════════════════════════════════════════════════════════
```

### Step 3: Launch Full Simulation & Workcell
In Terminal 1 (Simulation + ROS 2 Nodes):
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm_bringup industrial_workcell.launch.py
```

In Terminal 2 (FastAPI Telemetry + Web Dashboard):
```bash
source install/setup.bash
./scripts/aria_launch_full.sh
```
Open your browser to `http://localhost:3000` to interact with the ARIA Operator Console.

---

## 4. Universal Docker Reproduction (Any OS / Cloud Machine)

For systems running Fedora, Arch Linux, macOS, or Windows (via WSL2), ARIA provides complete Docker containerization.

### Running with NVIDIA GPU Acceleration (Recommended)
Ensure NVIDIA Container Toolkit is installed:
```bash
docker compose up
```
This automatically boots:
1. `ros2_core`: Headless Gazebo 11 + ARIA 15-Agent ROS 2 stack.
2. `rosbridge`: WebSocket proxy on port `9090`.
3. `dashboard`: Vite React GUI on port `3000`.

### Running with Optional Services
* **With Local LLM (Ollama Llama 3.1 8B):**
  ```bash
  docker compose --profile llm up
  ```
* **With MLflow Experiment Tracking:**
  ```bash
  docker compose --profile tracking up
  ```

---

## 5. Troubleshooting & Gotchas

1. **Gazebo Classic EMFILE Socket Exhaustion:**  
   *Symptom:* `Socket operation on non-socket` or `Too many open files`.  
   *Fix:* Set `GAZEBO_IP=127.0.0.1` and `GAZEBO_MASTER_URI=http://127.0.0.1:11345` in your shell profile.
2. **Camera Permission on Hardware:**  
   *Fix:* Add your user to the `video` and `dialout` groups: `sudo usermod -a -G dialout,video $USER`.
3. **GPU VRAM Spikes:**  
   If running OpenVLA-7B concurrently with SAM2 on an 8GB GPU, launch OpenVLA with 4-bit quantization enabled (`--load_in_4bit`).
