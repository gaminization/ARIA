# Project ARIA: Hardware Specification & Physical Build Guide

## 1. System Overview & Bill of Materials (BOM)

ARIA is designed to demonstrate high-capability robotic manipulation using accessible, low-cost commercial components. The entire bill of materials totals under **\$120 USD**, contrasting with industrial arms costing thousands of dollars.

| Component | Quantity | Purpose / Role | Unit Cost (USD) |
| :--- | :---: | :--- | :---: |
| **MG995 High-Torque Metal Gear Servo** | 3 | Joints 1, 2, 3 (Waist, Shoulder, Elbow) | $\sim \$7.50$ |
| **SG90 Micro Servo** | 2 | Joints 4, 5 (Wrist Pitch, Gripper Clamp) | $\sim \$2.50$ |
| **ESP32 Microcontroller (NodeMCU / WROOM)** | 1 | Micro-ROS servo PWM generation & IMU polling | $\sim \$5.00$ |
| **ESP32-CAM or Logitech C270 Web Camera** | 2 | Overhead workspace camera & wrist-mounted camera | $\sim \$9.00 - \$22.00$ |
| **MPU6050 6-Axis IMU Sensor** | 1 | End-effector tilt, vibration, and dynamic feedback | $\sim \$1.80$ |
| **PCA9685 16-Channel 12-Bit PWM Driver** | 1 | I2C servo signal generation (jitter suppression) | $\sim \$3.50$ |
| **5V / 10A DC Regulated Power Supply** | 1 | High-current isolated servo bus power | $\sim \$14.00$ |
| **3D Printed Arm Chassis (PETG/ABS)** | 1 | Mechanical links, base pedestal, and parallel gripper | $\sim \$18.00$ (filament) |
| **Hardware Fasteners (M3/M4 screws, bearings)** | 1 kit | Link pivots, horn mounts, and structural joints | $\sim \$8.00$ |
| **Total Estimated Hardware Cost** | — | — | **$\le \$110 - \$125$** |

---

## 2. Robotic Manipulator Kinematics & Actuators

The arm is configured as a **5-Degree-of-Freedom (5-DoF) revolute articulated arm** with an azimuth base and a three-link planar pitch subsystem.

```
       [Joint 5: Gripper Servo (SG90)]
             |
       [Joint 4: Wrist Pitch Servo (SG90)]
             |  L_forearm = 0.115 m
       [Joint 3: Elbow Servo (MG995)]
             |  L_upperarm = 0.145 m
       [Joint 2: Shoulder Servo (MG995)]
             |  H_base = 0.105 m
       [Joint 1: Waist Azimuth Servo (MG995)]
             |
    ═════════╧═════════ (Optical Table Surface / Pedestal)
```

### Servo Technical Specifications:
| Actuator Spec | MG995 (Joints 1, 2, 3) | SG90 (Joints 4, 5) |
| :--- | :--- | :--- |
| **Gear Train** | Metal alloy gearing | Nylon / Plastic gearing |
| **Operating Voltage** | $4.8\,\text{V} - 6.6\,\text{V}$ (Nominal $6.0\,\text{V}$) | $4.8\,\text{V} - 6.0\,\text{V}$ (Nominal $5.0\,\text{V}$) |
| **Stall Torque** | $10.0\,\text{kg}\cdot\text{cm}$ ($0.98\,\text{N}\cdot\text{m}$) @ 6V | $1.8\,\text{kg}\cdot\text{cm}$ ($0.18\,\text{N}\cdot\text{m}$) @ 5V |
| **Operating Speed** | $0.16\,\text{s} / 60^\circ$ ($6.54\,\text{rad/s}$ unloaded) | $0.10\,\text{s} / 60^\circ$ ($10.47\,\text{rad/s}$ unloaded) |
| **Software Velocity Limit** | **$1.50\,\text{rad/s}$ ($85.9^\circ/\text{s}$)** | **$2.50\,\text{rad/s}$ ($143.2^\circ/\text{s}$)** |
| **Travel Range** | $0^\circ - 180^\circ$ (Joint 3 restricted to $0^\circ - 150^\circ$) | $0^\circ - 180^\circ$ |
| **PWM Frequency** | $50\,\text{Hz}$ ($20\,\text{ms}$ period) | $50\,\text{Hz}$ ($20\,\text{ms}$ period) |
| **Pulse Width Range** | $500\,\mu\text{s} - 2500\,\mu\text{s}$ | $500\,\mu\text{s} - 2400\,\mu\text{s}$ |

---

## 3. Sensors: Cameras and IMU

### A. Dual Monocular Vision Sensing
1. **Overhead Workspace Camera (Logitech C270 or ESP32-CAM):**
   - **Position:** Rigidly mounted $1.45\,\text{m}$ above table surface pointing nadir ($[0.0, 0.0, 1.45]\,\text{m}$ in world frame).
   - **Resolution:** $1920 \times 1080$ @ $30\,\text{FPS}$ or $1280 \times 720$ @ $30\,\text{FPS}$.
   - **Optics:** Calibrated horizontal field of view $\text{HFOV} = 1.25\,\text{rad}$ ($71.6^\circ$). Focal length $f_x = f_y = 1330.59\,\text{px}$ (for 1080p).
   - **Role:** High-level scene parsing, object discovery, bounding box detection, and table plane ray-plane intersection.

2. **Wrist-Mounted Eye-in-Hand Camera (ESP32-CAM or Micro USB Cam):**
   - **Position:** Attached to link 4 facing parallel to the gripper tool vector.
   - **Resolution:** $640 \times 480$ @ $25\,\text{FPS}$.
   - **Role:** Active perception, viewpoint shifts for occluded objects, close-range visual servoing, and monocular relative disparity grounding.

### B. End-Effector MPU6050 IMU
- **Location:** Mounted on the gripper chassis behind the pinch fingers.
- **Interface:** I2C Bus (`SDA = GPIO 21`, `SCL = GPIO 22` on ESP32). Address: `0x68`.
- **Sensors:** 3-axis accelerometer ($\pm 2g$) + 3-axis gyroscope ($\pm 250^\circ/\text{s}$).
- **Role:**
  - Measures dynamic tilt angle during payload transport.
  - Detects physical contact collisions via transient acceleration spikes ($> 2.5g$).
  - Monitors high-frequency servo vibration to track gear lash and joint mechanical wear.

---

## 4. Electrical Schematics & Pinout Connections

```
                     ┌───────────────────────────────┐
                     │   5V / 10A DC Power Supply    │
                     └───────┬───────────────┬───────┘
                        +5V  │               │ GND
                             ▼               ▼
                   ┌───────────────────────────────────┐
                   │   PCA9685 16-Channel PWM Board    │
                   │ V+ (Servo Power)       GND (Pwr)  │
                   └───────┬───────────────────┬───────┘
                           │                   │
             ┌─────────────┼───────────────────┼─────────────┐
             │ CH0 (Waist) │ CH1 (Shoulder)    │ CH2 (Elbow) │ CH3 (Wrist) │ CH4 (Grip)
             ▼             ▼                   ▼             ▼             ▼
          [MG995]       [MG995]             [MG995]       [SG90]        [SG90]

  ESP32 Microcontroller:
  ┌─────────────────────────┐
  │ 3V3       ──> VCC (PCA9685 Logic & MPU6050)
  │ GND       ──> Common System GND (Power supply GND + PCA9685 + MPU6050)
  │ GPIO 21   ──> SDA (PCA9685 & MPU6050 I2C Data)
  │ GPIO 22   ──> SCL (PCA9685 & MPU6050 I2C Clock)
  │ GPIO 19   ──> Hardware Emergency Stop (NC Pushbutton to GND)
  │ USB / COM ──> Host PC running ROS 2 Humble (micro-ROS Agent)
  └─────────────────────────┘
```

> [!CAUTION]
> **Common Ground Requirement:** The ground lines from the 5V/10A servo supply and the ESP32 microcontroller must be tied together. Failure to establish a common ground will result in severe servo jitter, erratic positioning, and potential damage to logic pins. Never power servos directly from the ESP32 3.3V or 5V rail.

---

## 5. Micro-ROS Firmware Architecture

The ESP32 runs native firmware located in [`esp32_firmware/src/main.cpp`](file:///home/gaminizer/Projects/ARIA/esp32_firmware/src/main.cpp) compiled via PlatformIO:
- **Transport:** High-speed Serial CDC (`921600 baud`) or WiFi UDP transport to the `micro_ros_agent` ROS 2 container.
- **Node:** `aria_esp32_hardware_bridge`.
- **Subscribed Topics:**
  - `/joint_commands` (`std_msgs/msg/Float64MultiArray`): Array of 5 joint angles in radians. Firmware applies servo calibration offsets:
    $$\theta_i^{\text{PWM}} = \theta_i^{\text{zero}} + \text{sign}_i \cdot \theta_i^{\text{rad}} \cdot \left(\frac{2000\,\mu\text{s}}{\pi}\right)$$
- **Published Topics:**
  - `/joint_states` (`sensor_msgs/msg/JointState` @ $50\,\text{Hz}$): Measured or commanded joint angles.
  - `/aria/imu/data` (`sensor_msgs/msg/Imu` @ $100\,\text{Hz}$): Accelerometer and angular velocity from the end-effector MPU6050.
  - `/aria/estop` (`std_msgs/msg/Bool`): Hardware E-STOP button status.
