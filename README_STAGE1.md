# ARIA — Stage 1: Clean Simulation

> Physical arm in Gazebo. Manual joint control working.
> NO perception, NO AI, NO noise, NO servo dynamics lag.

## Quick Start

```bash
# 1. Build all packages
cd ~/Projects/ARIA
colcon build --packages-select arm_interfaces arm_description arm_control arm_bringup
source install/setup.bash

# 2. Launch simulation with manual control (RECOMMENDED)
ros2 launch arm_bringup manual_control.launch.py

# 3. In a separate terminal, launch keyboard control
source install/setup.bash
ros2 run arm_control keyboard_control.py

# 4. Validate everything works
source install/setup.bash
ros2 run arm_bringup validate_stage1.py
```

## Package Structure

```
ARIA/
├── arm_interfaces/           # Custom service definitions
│   └── srv/
│       ├── SetJoint.srv      # Command single joint
│       ├── SetAllJoints.srv  # Command all joints
│       └── GoNamedPose.srv   # Move to named pose
│
├── arm_description/          # Robot URDF/Xacro
│   └── urdf/
│       ├── aria_arm.urdf.xacro        # Main robot description
│       └── aria_arm.gazebo.xacro      # Gazebo plugins & sensors
│
├── arm_control/              # Hardware abstraction & control
│   ├── include/arm_control/
│   │   └── aria_hardware_interface.hpp
│   ├── src/
│   │   └── aria_hardware_interface.cpp
│   ├── config/
│   │   ├── aria_controllers.yaml      # Controller gains
│   │   └── ros2_control.yaml          # Hardware params
│   └── scripts/
│       ├── manual_control_node.py     # Main control node
│       ├── keyboard_control.py        # Terminal keyboard UI
│       └── joint_slider_gui.py        # Tkinter slider GUI
│
├── arm_bringup/              # Launch & simulation
│   ├── launch/
│   │   ├── sim.launch.py              # Base simulation
│   │   ├── manual_control.launch.py   # ★ Start here
│   │   └── hardware.launch.py        # Skeleton (Stage 4)
│   ├── worlds/
│   │   └── aria_workspace.sdf        # Gazebo world
│   ├── config/
│   │   └── aria_rviz.rviz            # RViz configuration
│   └── scripts/
│       └── validate_stage1.py         # Automated tests
│
└── README_STAGE1.md          # This file
```

## Robot Specification

### Joints (6 actuated)

| Joint | Name | Axis | Range | Servo | Mass |
|-------|------|------|-------|-------|------|
| J1 | Waist | Z | -90° to +90° | MG995 | 55g |
| J2 | Shoulder | Y | 0° to 180° | MG995 | 55g |
| J3 | Elbow | Y | 0° to 150° | MG995 | 55g |
| J4 | Wrist Pitch | Y | -90° to +90° | SG90 | 9g |
| J5 | Wrist Roll | X | -90° to +90° | SG90 | 9g |
| J6 | Gripper | Z | 0° (closed) to 45° (open) | SG90 | 9g |

### Named Poses

| Pose | Waist | Shoulder | Elbow | W.Pitch | W.Roll | Gripper |
|------|-------|----------|-------|---------|--------|---------|
| home | 0° | 90° | 75° | 0° | 0° | 20° |
| ready | 0° | 45° | 135° | 0° | 0° | 20° |
| folded | 0° | 160° | 20° | 0° | 0° | 10° |
| inspect | 0° | 60° | 120° | -30° | 0° | 20° |

### Sensors (Clean — zero noise)

| Sensor | Type | Topic | Rate |
|--------|------|-------|------|
| Top Camera | Logitech C270 | /top_camera/image_raw | 30 fps |
| Wrist Camera | ESP32-CAM | /wrist_camera/image_raw | 15 fps |
| IMU | MPU6050 | /mpu6050/imu_raw | 100 Hz |
| Contact L | Force | /gripper/contact_left | 30 Hz |
| Contact R | Force | /gripper/contact_right | 30 Hz |

## Control Interface

### ROS2 Services

| Service | Type | Description |
|---------|------|-------------|
| /aria/set_joint | SetJoint | Command single joint by name + angle |
| /aria/set_all_joints | SetAllJoints | Command all 6 joints |
| /aria/go_named_pose | GoNamedPose | Move to named pose |
| /aria/open_gripper | Trigger | Open gripper to ~44° |
| /aria/close_gripper | Trigger | Close gripper to ~1° |
| /aria/estop | Trigger | Emergency stop — halt all motion |
| /aria/release_estop | Trigger | Release e-stop |

### Keyboard Control

```
q/a: Joint 1 (Waist)        ±step
w/s: Joint 2 (Shoulder)     ±step
e/d: Joint 3 (Elbow)        ±step
r/f: Joint 4 (Wrist Pitch)  ±step
t/g: Joint 5 (Wrist Roll)   ±step
y/h: Joint 6 (Gripper)      open/close

1: Fine step (1°)
5: Medium step (5°)  ← default
0: Coarse step (15°)

SPACE: Go home
p:     Cycle named poses
ESC/x: Emergency stop
c:     Clear e-stop
Q:     Quit
```

## Safety Features

- **Soft limits**: 1° margin from hard limits on all joints
- **Speed cap**: Maximum 90°/s per joint
- **NaN rejection**: Any NaN/Inf values immediately rejected
- **E-stop**: Immediate motion halt, holds current position
- **Limit coloring**: Green (safe) → Orange (<10° from limit) → Red (at limit)

## Validation

Run after first launch to verify all systems:

```bash
ros2 run arm_bringup validate_stage1.py
```

**12 automated tests:**
1. URDF loads without errors
2. All 6 joints in /joint_states
3. Each joint responds to commands
4. Joint limits enforced
5. Home position reachable
6. All named poses reachable
7. Gripper opens and closes
8. Camera topics active
9. IMU topic active
10. E-stop blocks commands
11. TF tree complete
12. No NaN in joint states

## Dependencies

- ROS2 Humble (or Iron/Jazzy)
- Gazebo Harmonic (gz-sim)
- gz_ros2_control
- ros_gz (bridge)

## What's Next

**Stage 2**: Perception — camera calibration, object detection, depth estimation
**Stage 3**: AI — inverse kinematics, motion planning, grasp planning
**Stage 4**: Hardware — ESP32 serial protocol, servo calibration
