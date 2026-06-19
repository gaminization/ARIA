# ARIA Hardware Setup Guide

Complete guide for connecting the ARIA 5-DoF robotic arm to real hardware using ESP32.

---

## Prerequisites

| Item | Spec | Notes |
|------|------|-------|
| ESP32 board | Wemos D32 or DevKit V1 | Must have WiFi + enough GPIO |
| Servos (×3) | MG995 | Waist, Shoulder, Elbow |
| Servos (×3) | SG90 | Wrist Pitch, Wrist Roll, Gripper |
| Power supply | 5V 5A+ | Dedicated servo power — NOT USB! |
| USB cable | Micro-USB or USB-C | For flashing and serial |
| PC | Ubuntu 22.04 | ROS2 Humble installed |

---

## 1. Wiring Diagram

### ESP32 → Servo Connections

| Joint | Servo | ESP32 Pin | ADC Pin (feedback) |
|-------|-------|-----------|-------------------|
| J1 Waist | MG995 | GPIO 3 | GPIO 34 |
| J2 Shoulder | MG995 | GPIO 5 | GPIO 35 |
| J3 Elbow | MG995 | GPIO 6 | GPIO 32 |
| J4 Wrist Pitch | SG90 | GPIO 10 | GPIO 25 |
| J5 Wrist Roll | SG90 | GPIO 9 | GPIO 33 |
| J6 Gripper | SG90 | GPIO 11 | GPIO 26 |

### Power Wiring

```
5V Power Supply ──┬──► Servo VCC rail (all 6 servos)
                  └──► ESP32 VIN (optional, can use USB)

GND ──────────────┬──► Servo GND rail
                  └──► ESP32 GND

⚠️  CRITICAL: Do NOT power servos from ESP32's USB 5V pin.
    MG995 servos draw up to 1.5A each under load.
    Use a dedicated 5V 5A+ power supply.
```

---

## 2. Flash ESP32 Firmware

### Install PlatformIO

```bash
pip install platformio
```

### Build and Upload

```bash
cd ~/Projects/ARIA/esp32_firmware

# Edit WiFi credentials in src/main.cpp:
#   WIFI_SSID, WIFI_PASS, ROS_AGENT_IP

pio run -e esp32dev --target upload

# Monitor serial output:
pio device monitor -b 115200
```

You should see:

```
═══════════════════════════════════════
  ARIA ESP32 Firmware v1.0.0
═══════════════════════════════════════
[ARIA] Connecting to WiFi: YOUR_SSID
[ARIA] WiFi connected, micro-ROS transport ready.
[ARIA] micro-ROS node 'aria_esp32' initialized.
[ARIA] Waiting for commands...
```

---

## 3. Start micro-ROS Agent

On your PC, start the agent that bridges ESP32 micro-ROS to the main ROS2 network:

### WiFi mode (recommended)

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### USB Serial mode

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

---

## 4. Run the Bringup Wizard

First-time setup — run once:

```bash
python3 arm_bringup/scripts/hardware_bringup_wizard.py
# (or: aria hardware wizard)
```

The wizard will:
1. Detect the ESP32
2. Verify all servo connections
3. Calibrate ADC feedback
4. Align sim-to-real offsets
5. Test the safety layer

---

## 5. Launch Hardware System

```bash
# Source workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch
ros2 launch arm_bringup hardware.launch.py

# Or with options:
ros2 launch arm_bringup hardware.launch.py port:=/dev/ttyUSB1 feedback:=adc
```

---

## 6. Feedback Options

### Option A: Tapped Pot Wiper (Recommended)

See [SERVO_FEEDBACK_MOD.md](SERVO_FEEDBACK_MOD.md) for the step-by-step guide.

- Open servo case
- Solder wire to potentiometer wiper (center pin)
- Route to ESP32 ADC pin
- No performance impact

### Option B: External Potentiometer

- Mount 10K pot to each joint axis
- More reliable signal
- More mechanical work
- Read via same ADC pins

### Option C: Command Tracking Only (No Mod)

- Tracks commanded angles only
- Cannot detect servo stalls
- Teach mode works via fixed increments
- Good for initial testing

Set in `esp32_firmware/src/main.cpp`:

```cpp
#define INITIAL_FEEDBACK_MODE  FEEDBACK_ADC       // Option A/B
#define INITIAL_FEEDBACK_MODE  FEEDBACK_COMMANDED  // Option C
```

---

## 7. Teach Mode

```bash
# Enter teach mode (arm relaxes)
aria teach start

# Record waypoints
aria teach waypoint   # at each desired position

# Stop and save
aria teach stop

# Replay the trajectory
aria teach replay
```

---

## 8. Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| ESP32 not detected | Wrong USB port | Check `ls /dev/ttyUSB*` |
| Servos not moving | No power | Check 5V supply, power LED |
| Servos jittering | Insufficient current | Use beefier 5V supply |
| ADC reads ~0 or ~4095 | Pot wire loose | Reseat ADC wiper connection |
| Sim-real mismatch | Uncalibrated offsets | Run `aria hardware calibrate` |
| Servo "kicks" on re-engage | Position jump | Verify ADC readings before re-attach |
| micro-ROS timeout | WiFi issues | Check agent IP in firmware, try USB |

---

## 9. Safety Notes

- **Always** keep the E-stop command ready: `aria estop`
- Never run servos without the 5V power supply connected
- MG995 servos can exert significant force — keep fingers clear
- The software rate-limits all servos to 120°/s maximum
- If no command is received for 500ms, servos hold their last position
