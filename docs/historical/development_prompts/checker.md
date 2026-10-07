# ARIA Pre-Flight Preparation Guide
## Pop!_OS 22.04 + RTX 5060 + 5-DoF Arm

---

## HARDWARE CHECKLIST

**The Arm:**
- Techno-Tirupati 5-DoF kit (fully assembled, servos wired)
- Confirm all 6 servos move freely by hand before powering
- Arm securely mounted to a flat base plate

**Replace Arduino Nano with ESP32:**
- ESP32 DevKit C v4 (recommended) or Wemos D32 — buy this if you haven't
- Micro-USB or USB-C cable for ESP32 (depending on board)
- The original HC-05 Bluetooth module is no longer needed — set it aside

**Cameras:**
- Logitech C270 USB webcam + USB-A cable
- ESP32-CAM module (or use a second USB webcam for wrist initially)
- Camera stand or mount for overhead view — minimum 70cm tall, stable

**Sensors and Wiring:**
- MPU6050 module (the blue breakout board with 8 pins)
- 4-pin dupont wires for MPU6050 → ESP32 (SDA, SCL, VCC, GND)
- Jumper wires, breadboard for prototyping the ESP32 connections
- 6 × 10K ohm potentiometers (optional — for real-to-sim servo feedback Option B, easier than tapping internal servo pots)

**Power:**
- 5V 3A USB power supply (for servo rail — USB hub or dedicated supply)
- Separate USB for ESP32 data/flash line
- Do NOT power 6 servos from the ESP32 5V pin — use a dedicated supply

**Tools:**
- Small Phillips screwdriver (servo horns)
- Multimeter (for checking ADC voltage ranges per servo)
- Soldering iron (optional — only needed if you tap servo pots for Option A feedback)

**Calibration:**
- Print 2 copies of AprilTag ID 0 (family: tag36h11) at 10×10cm on plain white paper
- Tape one to a flat rigid piece of cardboard for the workspace calibration board

---

## PHASE 1 — SYSTEM PREPARATION

Run these first, before anything else.

**Update and base tools:**
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl wget git vim build-essential cmake \
  python3-pip python3-venv python3-dev \
  libssl-dev libffi-dev libusb-1.0-0-dev \
  v4l-utils usbutils htop tmux
```

**Check your GPU — RTX 5060 needs driver 570+ and CUDA 12.8+:**
```bash
nvidia-smi        # check driver version
nvcc --version    # check if CUDA toolkit already installed
```

Pop!_OS NVIDIA edition should already have drivers. If driver is below 570:
```bash
sudo apt install system76-driver-nvidia   # Pop!_OS way
# OR use the NVIDIA official .run installer for driver 570+
```

**Install CUDA Toolkit 12.8 (RTX 5060 Blackwell requires this):**
```bash
# Go to: https://developer.nvidia.com/cuda-downloads
# Select: Linux > x86_64 > Ubuntu > 22.04 > deb (network)
# Follow the exact commands shown on that page for CUDA 12.8
# Then:
echo 'export PATH=/usr/local/cuda-12.8/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:$PATH' >> ~/.bashrc
source ~/.bashrc
nvcc --version  # should now show 12.8
```

**Serial port access (for ESP32):**
```bash
sudo usermod -aG dialout $USER
sudo usermod -aG plugdev $USER
# Log out and back in for this to take effect
# Verify after login: groups | grep dialout
```

**Camera access:**
```bash
sudo usermod -aG video $USER
# Verify: ls /dev/video*  (shows available cameras when plugged in)
```

---

## PHASE 2 — ROS2 HUMBLE

```bash
# Set up locale (required by ROS2)
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop-full
sudo apt install -y python3-colcon-common-extensions \
  python3-rosdep python3-vcstool python3-rosinstall-generator

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS2 in every terminal automatically
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## PHASE 3 — GAZEBO HARMONIC

```bash
# Add Gazebo repository
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable \
  $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y gz-harmonic

# ROS2-Gazebo bridge (connects Humble to Harmonic)
sudo apt install -y ros-humble-ros-gzharmonic
sudo apt install -y ros-humble-gz-ros2-control
```

---

## PHASE 4 — ROS2 PACKAGES (All-in-one install)

```bash
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-gripper-controllers \
  ros-humble-moveit \
  ros-humble-moveit-ros-planning \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-visual-tools \
  ros-humble-moveit-servo \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-rviz-visual-tools \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-tf2-geometry-msgs \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-vision-msgs \
  ros-humble-image-transport \
  ros-humble-image-pipeline \
  ros-humble-cv-bridge \
  ros-humble-usb-cam \
  ros-humble-v4l2-camera \
  ros-humble-camera-calibration \
  ros-humble-camera-info-manager \
  ros-humble-apriltag-ros \
  ros-humble-nav-msgs \
  ros-humble-visualization-msgs \
  ros-humble-action-msgs \
  ros-humble-lifecycle-msgs \
  ros-humble-rclcpp-lifecycle \
  ros-humble-rclpy \
  ros-humble-micro-ros-agent \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-image \
  ros-humble-plotjuggler-ros
```

---

## PHASE 5 — PYTHON ENVIRONMENT

Use a dedicated conda environment to keep ML packages isolated from system Python.

**Install Miniconda:**
```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
# Follow prompts, say yes to conda init
source ~/.bashrc
```

**Create the ARIA environment:**
```bash
conda create -n aria python=3.10 -y
conda activate aria

# Source ROS2 inside conda (add to ~/.bashrc or do manually each session)
# NOTE: do NOT conda activate in .bashrc — activate manually
```

**Core Python packages:**
```bash
pip install --upgrade pip setuptools wheel

# Robotics
pip install ikpy
pip install roboticstoolbox-python
pip install spatialmath-python
pip install scipy numpy matplotlib

# ROS2 Python (re-install for conda environment)
pip install empy catkin-pkg lark

# Computer Vision
pip install opencv-python opencv-contrib-python
pip install Pillow imageio
pip install pyrealsense2  # optional, may be needed for some depth utils

# Database and utils
pip install sqlalchemy
pip install h5py        # for LeRobot dataset format
pip install pyyaml
pip install rich        # for nice terminal output
pip install fastapi uvicorn websockets  # for dashboard
pip install asyncio aiohttp

# Plotting and analysis
pip install pandas seaborn plotly
```

---

## PHASE 6 — AI/ML STACK

**PyTorch with CUDA 12.8 (RTX 5060 Blackwell):**
```bash
conda activate aria
pip install torch torchvision torchaudio \
  --index-url https://download.pytorch.org/whl/cu128
# Verify:
python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
# Should print: True, and your RTX 5060 name
```

**YOLO (Ultralytics):**
```bash
pip install ultralytics
# Downloads yolov8m.pt on first run automatically
# Pre-download now to avoid waiting later:
python -c "from ultralytics import YOLO; YOLO('yolov8m.pt')"
```

**Depth-Anything v2:**
```bash
git clone https://github.com/DepthAnything/Depth-Anything-V2
cd Depth-Anything-V2
pip install -r requirements.txt
# Download the small model checkpoint (fastest, good for real-time):
# Go to their HuggingFace page and download depth_anything_v2_vits.pth
# Save to: Depth-Anything-V2/checkpoints/
cd ..
```

**MiDaS (fallback depth model):**
```bash
pip install timm
# MiDaS loads via torch.hub automatically on first use:
python -c "import torch; torch.hub.load('intel-isl/MiDaS', 'DPT_Hybrid')"
```

**GraspNet:**
```bash
git clone https://github.com/graspnet/graspnet-baseline
cd graspnet-baseline
pip install -r requirements.txt
# Also needs MinkowskiEngine (CUDA build):
pip install ninja
git clone https://github.com/NVIDIA/MinkowskiEngine
cd MinkowskiEngine
python setup.py install --blas_include_dirs=/usr/include/mkl \
  --blas=mkl
cd ../..
# Note: MinkowskiEngine build takes 5-10 minutes on first install
```

**LeRobot (Hugging Face):**
```bash
pip install lerobot
# Or from source for latest:
git clone https://github.com/huggingface/lerobot
cd lerobot
pip install -e ".[dev]"
cd ..
```

**ByteTrack (object tracking):**
```bash
git clone https://github.com/ifzhang/ByteTrack
cd ByteTrack
pip install -r requirements.txt
pip install -e .
cd ..
```

**AprilTag Python bindings:**
```bash
pip install pupil-apriltags
pip install dt-apriltags
```

**Other ML utilities:**
```bash
pip install transformers huggingface-hub
pip install einops  # needed by several vision transformers
pip install open3d  # 3D point cloud processing for GraspNet
pip install pycocotools  # for YOLO evaluation metrics
```

---

## PHASE 7 — ESP32 TOOLCHAIN

**PlatformIO (firmware development IDE for ESP32):**
```bash
pip install platformio
# Install VS Code extension separately if using VS Code:
# Extensions → search "PlatformIO IDE" → install
# Or use CLI only (command line is fine for this project)

# Test install:
pio --version
```

**micro-ROS agent (already installed via ROS2 packages above):**
```bash
# Verify:
ros2 run micro_ros_agent micro_ros_agent --help
```

**ESP32 board support in PlatformIO:**
```bash
pio platform install espressif32
# This downloads the ESP32 Arduino framework and toolchain (~500MB)
```

**USB rules for ESP32 (so you don't need sudo for flashing):**
```bash
# Check what rule files already exist:
ls /etc/udev/rules.d/ | grep -i esp
# If none, add:
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-esp32.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"' \
  | sudo tee -a /etc/udev/rules.d/99-esp32.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## PHASE 8 — ROS2 WORKSPACE SETUP

```bash
mkdir -p ~/aria_ws/src
cd ~/aria_ws
colcon build --symlink-install  # initial empty build to verify setup
source install/setup.bash
echo "source ~/aria_ws/install/setup.bash" >> ~/.bashrc

# Create the package directory structure that the prompts will fill:
mkdir -p ~/aria_ws/src/arm_description/urdf
mkdir -p ~/aria_ws/src/arm_description/meshes
mkdir -p ~/aria_ws/src/arm_bringup/launch
mkdir -p ~/aria_ws/src/arm_bringup/worlds
mkdir -p ~/aria_ws/src/arm_bringup/config
mkdir -p ~/aria_ws/src/arm_bringup/scripts
mkdir -p ~/aria_ws/src/arm_bringup/docs
mkdir -p ~/aria_ws/src/arm_control/include/arm_control
mkdir -p ~/aria_ws/src/arm_control/src
mkdir -p ~/aria_ws/src/arm_control/scripts
mkdir -p ~/aria_ws/src/arm_control/config
mkdir -p ~/aria_ws/src/arm_vision/arm_vision
mkdir -p ~/aria_ws/src/arm_vision/sim
mkdir -p ~/aria_ws/src/arm_vision/config
mkdir -p ~/aria_ws/src/arm_ik/arm_ik/ik_solvers
mkdir -p ~/aria_ws/src/arm_ik/data
mkdir -p ~/aria_ws/src/arm_ik/models
mkdir -p ~/aria_ws/src/arm_ik/config
mkdir -p ~/aria_ws/src/arm_moveit_config/config
mkdir -p ~/aria_ws/src/arm_moveit_config/scripts
mkdir -p ~/aria_ws/src/arm_planner/arm_planner
mkdir -p ~/aria_ws/src/arm_planner/msg
mkdir -p ~/aria_ws/src/arm_planner/data
mkdir -p ~/aria_ws/src/arm_planner/logs
mkdir -p ~/aria_ws/src/arm_agents/arm_agents
mkdir -p ~/aria_ws/src/arm_vla/arm_vla
mkdir -p ~/aria_ws/src/arm_learning/arm_learning
mkdir -p ~/aria_ws/src/arm_learning/datasets
mkdir -p ~/aria_ws/src/arm_dashboard/frontend/src
mkdir -p ~/aria_ws/src/arm_dashboard/frontend/public
mkdir -p ~/aria_ws/src/arm_testing/tests
mkdir -p ~/aria_ws/src/arm_testing/benchmarks
mkdir -p ~/aria_ws/src/esp32_firmware/src
```

**Add model paths to environment:**
```bash
cat >> ~/.bashrc << 'EOF'

# ARIA environment
conda activate aria 2>/dev/null || true
export ARIA_WS=~/aria_ws
export DEPTH_ANYTHING_PATH=~/Depth-Anything-V2
export GRASPNET_PATH=~/graspnet-baseline
export LEROBOT_PATH=~/lerobot
EOF
source ~/.bashrc
```

---

## PHASE 9 — VERIFICATION TESTS

Run each of these. All should pass before you feed any prompts.

```bash
# 1. ROS2 working
ros2 --version
# Expected: ros2cli 0.18.x or similar

# 2. Gazebo working
gz sim --version
# Expected: 8.x.x (Harmonic)

# 3. ROS2 + Gazebo bridge
ros2 pkg list | grep gz
# Expected: multiple ros_gz packages listed

# 4. MoveIt2 working
ros2 pkg list | grep moveit
# Expected: moveit, moveit_ros_planning, etc.

# 5. GPU working
conda activate aria
python -c "
import torch
print('PyTorch:', torch.__version__)
print('CUDA available:', torch.cuda.is_available())
print('CUDA version:', torch.version.cuda)
print('GPU:', torch.cuda.get_device_name(0))
"
# Expected: True, 12.8, RTX 5060 (or similar name)

# 6. YOLO working on GPU
python -c "
from ultralytics import YOLO
import torch
model = YOLO('yolov8m.pt')
print('YOLO loaded, device:', 'cuda' if torch.cuda.is_available() else 'cpu')
"

# 7. Depth-Anything accessible
python -c "import sys; sys.path.insert(0, '~/Depth-Anything-V2'); print('Depth-Anything path OK')"

# 8. ikpy working
python -c "import ikpy; print('ikpy:', ikpy.__version__)"

# 9. roboticstoolbox working
python -c "import roboticstoolbox; print('RTB working')"

# 10. PlatformIO working
pio --version

# 11. micro-ROS agent installed
ros2 pkg prefix micro_ros_agent
# Expected: /opt/ros/humble

# 12. Camera detected (plug in C270 first)
v4l2-ctl --list-devices
# Expected: HD Webcam C270 listed with /dev/video0 or similar

# 13. ESP32 detected (plug in ESP32 first)
ls /dev/ttyUSB*
# Expected: /dev/ttyUSB0 or similar
```

---

## OPTIONAL BUT RECOMMENDED

**VS Code with useful extensions:**
```bash
# Install VS Code if not already:
sudo snap install code --classic
# Then install these extensions:
# - ROS (Microsoft)
# - C/C++ (Microsoft)  
# - Python (Microsoft)
# - PlatformIO IDE
# - URDF (smilerobotics)
# - GitLens
```

**PlotJuggler for ROS2 topic visualization:**
```bash
sudo apt install -y ros-humble-plotjuggler-ros
# Extremely useful for debugging joint angles and sensor data
```

**rqt tools:**
```bash
sudo apt install -y ros-humble-rqt ros-humble-rqt-common-plugins \
  ros-humble-rqt-robot-monitor ros-humble-rqt-tf-tree
```

**Node.js for React dashboard (Prompt 3b):**
```bash
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs
node --version  # should be 20.x
npm --version
```

**tmux config for multi-terminal work:**
```bash
# You'll be running Gazebo, multiple ROS2 nodes, and the dashboard
# simultaneously. tmux makes this much easier.
cat > ~/.tmux.conf << 'EOF'
set -g mouse on
set -g history-limit 10000
bind | split-window -h
bind - split-window -v
EOF
```

---

## PHASE 10 — HARDWARE PHYSICAL SETUP

Do this once, physically, before software testing.

**Arm mounting:**
- Bolt or clamp arm base to table so it cannot shift during operation
- Arm base center position: left-center of workspace, 15cm from front edge
- Verify arm can sweep without hitting camera stand, cables, or table edge

**Camera stand:**
- Mount C270 directly overhead or at 45° angle, 70–80cm above table surface
- Angle downward to see the full 40×40cm workspace in frame
- Cable managed so it doesn't enter the workspace area

**ESP32 wiring:**
- ESP32 pin to servo signal wire mapping matches the firmware (D3, D5, D6, D9, D10, D11)
- All servo power (red) wires to 5V supply rail, NOT ESP32 5V pin
- All servo ground (black/brown) wires to common ground with ESP32
- MPU6050: SDA→D21, SCL→D22, VCC→3.3V, GND→GND

**Workspace:**
- Clear, flat, matte surface (not reflective — YOLO works better)
- White or light gray works best for object contrast
- AprilTag board placed at fixed position, visible from top camera, 30–35cm from arm base

---

## SUMMARY — WHAT TO DO IN ORDER

```
1. Buy ESP32 DevKit C if you don't have one
2. Print AprilTag ID 0 at 10cm size
3. Install all system packages (Phase 1)
4. Install ROS2 Humble (Phase 2)
5. Install Gazebo Harmonic (Phase 3)
6. Install ROS2 packages (Phase 4)
7. Set up conda + Python packages (Phase 5)
8. Set up AI/ML stack (Phase 6) ← takes longest, ~1-2 hours
9. Set up ESP32 toolchain (Phase 7)
10. Create workspace structure (Phase 8)
11. Run all verification tests (Phase 9) ← must all pass before running prompts
12. Physical arm setup (Phase 10) ← can do in parallel with software
13. Start with Prompt 1
```

The AI/ML stack (Phase 6) is the most time-consuming. MinkowskiEngine in particular takes 5–10 minutes to compile. Start that running and set up the physical hardware in parallel.