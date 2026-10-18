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
- **Sensors:** 3-axis accelerometer ($\pm 4g$) + 3-axis gyroscope ($\pm 500^\circ/\text{s}$), configured in firmware (`ACCEL_CONFIG`/`GYRO_CONFIG`).
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

> [!NOTE]
> **PCA9685 is an optional upgrade.** The shipped firmware
> ([`esp32_firmware/`](../esp32_firmware/)) drives the five servos **directly
> from ESP32 GPIOs** via the `ESP32Servo` library (`waist=GPIO3`,
> `shoulder=GPIO5`, `elbow=GPIO6`, `wrist_pitch=GPIO10`, `gripper=GPIO11`),
> so no PCA9685 is required. Add the PCA9685 board (I2C `SDA=GPIO21`,
> `SCL=GPIO22`) only if you want dedicated 12-bit PWM generation; if you do,
> wire the servos to its channels instead of the GPIOs above.

---

## 5. Micro-ROS Firmware Architecture

The ESP32 runs native firmware located in [`esp32_firmware/src/main.cpp`](file:///home/gaminizer/Projects/ARIA/esp32_firmware/src/main.cpp) compiled via PlatformIO:
- **Transport:** USB-serial CDC (`SERIAL_BAUD = 921600`) or WiFi UDP, selected by the firmware `USE_WIFI` define. The baud/UDP settings must match the `microros_agent` started by [`arm_bringup/launch/hardware.launch.py`](file:///home/gaminizer/Projects/ARIA/arm_bringup/launch/hardware.launch.py) (`transport:=serial baud:=921600`, or `transport:=wifi agent_port:=8888`).
- **Node:** `aria_esp32`.
- **Subscribed Topics:**
  - `/aria/servo_commands` (`sensor_msgs/msg/JointState`): 5 joint target angles in **degrees**, in canonical order `[waist, shoulder, elbow, wrist_pitch, gripper]`. Firmware clamps to per-joint limits and rate-limits each servo.
  - `/aria/gripper_command` (`std_msgs/msg/Float32`): $0.0$ = open, $1.0$ = closed.
  - `/aria/teach_mode` (`std_msgs/msg/Bool`): `true` detaches all servos (limp); `false` re-attaches at the current position.
- **Published Topics:**
  - `/aria/servo_states` (`sensor_msgs/msg/JointState` @ $50\,\text{Hz}$): per-joint position (degrees) from ADC feedback or the commanded value if ADC feedback is disabled.
  - `/aria/imu` (`sensor_msgs/msg/Imu` @ $100\,\text{Hz}$): accelerometer and angular velocity from the end-effector MPU6050.
  - `/aria/esp32_health` (`std_msgs/msg/Float32MultiArray`): `[uptime_s, vcc_volts, adc_feedback_enabled, teach_mode_active]` ($50\,\text{Hz}$ / $1\,\text{Hz}$ heartbeat).
  - `/aria/heartbeat` (`std_msgs/msg/Header` @ $1\,\text{Hz}$).

The ROS-side bridge is [`arm_control/scripts/servo_sync_node.py`](file:///home/gaminizer/Projects/ARIA/arm_control/scripts/servo_sync_node.py) (SIM↔REAL mirroring), or the `arm_control/AriaHardwareInterface` ros2_control plugin which bridges the same topics directly; use one or the other, not both.
