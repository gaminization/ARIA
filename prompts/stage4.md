<context>
STAGE 4 of 4 for ARIA.
Prerequisites: Stages 1–3 complete and validated in simulation.

STAGE 4 GOAL:
Connect simulation to real Techno-Tirupati arm.
ESP32 replaces the original Arduino Nano.

TWO CRITICAL FEATURES:
  1. SIM → REAL:  Command joint in simulation → real servo moves
  2. REAL → SIM:  Manually rotate real servo → simulation updates

Physical arm servo assignment (from assembly manual wiring):
  Signal wire order at controller:
    D3  → Waist      (MG995, joint 1)
    D5  → Shoulder   (MG995, joint 2)
    D6  → Elbow      (MG995, joint 3)
    D9  → Wrist Roll (SG90,  joint 4)
    D10 → Wrist Pitch(SG90,  joint 5)
    D11 → Gripper    (SG90,  joint 6)

NOTE: Original kit uses Arduino Nano + HC-05.
      This project UPGRADES to ESP32 (Wemos D32 or DevKit).
      Benefits: WiFi (no HC-05 needed), more pins,
                micro-ROS support, ADC for servo feedback.
      
SERVO FEEDBACK (for REAL → SIM mapping):
  SG90 and MG995 both have internal potentiometers.
  To read position: tap the pot wiper wire from inside the servo.
  
  Option A (Recommended - tap pot):
    Open servo, attach thin wire to pot center pin (wiper)
    Route wire to ESP32 ADC pin
    ADC reads 0–3.3V → maps to 0–180°
    NO modification to servo performance
    
  Option B (External pot):
    Attach 10K pot to each joint axis externally
    Read via ESP32 ADC
    More reliable, more work to mount
    
  Option C (Command tracking only - no hardware mod):
    Only tracks commanded positions (not actual if servo stalls)
    Teach mode works by commanding and verifying contact
    Include as fallback in firmware
    
  Firmware implements all 3 options, selected via config.
</context>

<task>
Generate all Stage 4 files.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 1 — ESP32 FIRMWARE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: esp32_firmware/src/main.cpp

Arduino-style ESP32 firmware with micro-ROS.

#include <micro_ros_arduino.h>
#include <ESP32Servo.h>

Configuration (edit at top of file):
  #define USE_WIFI true        // or false for USB serial
  #define WIFI_SSID "..."
  #define WIFI_PASS "..."
  #define ROS_AGENT_IP "192.168.x.x"  // PC running micro-ros-agent
  #define ROS_AGENT_PORT 8888
  
  // Servo pins (matching assembly manual wiring)
  #define SERVO_WAIST_PIN      3
  #define SERVO_SHOULDER_PIN   5
  #define SERVO_ELBOW_PIN      6
  #define SERVO_WRIST_ROLL_PIN 9
  #define SERVO_WRIST_PITCH_PIN 10
  #define SERVO_GRIPPER_PIN    11
  
  // ADC pins for position feedback (Option A: tapped pot)
  #define ADC_WAIST_PIN        34
  #define ADC_SHOULDER_PIN     35
  #define ADC_ELBOW_PIN        32
  #define ADC_WRIST_ROLL_PIN   33
  #define ADC_WRIST_PITCH_PIN  25
  #define ADC_GRIPPER_PIN      26
  
  // Servo calibration (pulse widths in microseconds)
  // CALIBRATE THESE FOR YOUR SPECIFIC SERVOS
  #define MG995_MIN_PULSE  600
  #define MG995_MAX_PULSE  2400
  #define SG90_MIN_PULSE   500
  #define SG90_MAX_PULSE   2500
  
  // Feedback mode
  #define FEEDBACK_MODE  FEEDBACK_ADC  // or FEEDBACK_COMMANDED

micro-ROS setup:
  Node: "aria_esp32"
  
  Subscribers:
    /aria/servo_commands (sensor_msgs/JointState):
      Receives: joint angles in degrees [6 values]
      Drives servos via PWM
      
    /aria/gripper_command (std_msgs/Float32):
      0.0 = open, 1.0 = closed
      Mapped to gripper servo range
      
    /aria/teach_mode (std_msgs/Bool):
      true: disable torque (relax servos for manual movement)
      false: re-enable torque
      
  Publishers (at 50Hz):
    /aria/servo_states (sensor_msgs/JointState):
      position: [6] ADC-read angles OR commanded angles
      velocity: [6] (estimated from position delta)
      
    /aria/esp32_health (custom msg or Float32MultiArray):
      [uptime, vcc_voltage, adc_readings_valid, feedback_mode]
      
    /aria/heartbeat (std_msgs/Header):
      Published at 1Hz. ROS2 detects timeout if missing.

Servo angle mapping:
  float deg_to_pulse(float angle_deg, ServoType type):
    if (type == MG995):
      return map(angle_deg, 0, 180, MG995_MIN_PULSE, MG995_MAX_PULSE)
    else:  // SG90
      return map(angle_deg, 0, 180, SG90_MIN_PULSE, SG90_MAX_PULSE)
      
  float adc_to_deg(int adc_val):
    // ADC: 0-4095 (12-bit) → 0-3.3V → 0-180°
    // Linear mapping, calibrate endpoints per servo
    return (float)adc_val / 4095.0 * 180.0

Safety (hardware layer):
  Servo rate limiting (max 120°/s software limit in firmware)
  If no command received for > 500ms: hold last position
  If VCC drops below 4.5V: log warning (brownout risk)
  
Teach mode implementation:
  When /aria/teach_mode = true:
    Set all servos to "detach" (remove PWM signal)
    → Servos go limp, user can manually move arm
    Continue reading ADC for position feedback
    ADC readings published to /aria/servo_states.position
    
  When /aria/teach_mode = false:
    Re-attach servos at CURRENT ADC-read position
    → No snap-to-commanded, smooth re-enable
    Resume normal command tracking

File: esp32_firmware/src/servo_manager.cpp
File: esp32_firmware/src/servo_manager.h
  Class managing all 6 servos:
    init(), set_angle(joint_id, deg),
    get_angle_commanded(joint_id), get_angle_feedback(joint_id),
    set_speed_limit(joint_id, deg_per_s),
    enable_torque(joint_id), disable_torque(joint_id)

File: esp32_firmware/platformio.ini
  [env:esp32dev]
  platform = espressif32
  board = esp32dev
  framework = arduino
  lib_deps =
    micro-ros-platformio
    madhephaestus/ESP32Servo@^0.11.0

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 2 — BIDIRECTIONAL SERVO MAPPING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_control/arm_control/servo_sync_node.py

class ServoSyncNode(Node):
  """
  THE BRIDGE between simulation and real hardware.
  
  Two sync directions:
  
  SIM → REAL (Forward sync, default):
    Sim joint_states → ESP32 servo commands
    This is normal operation mode.
    
  REAL → SIM (Reverse sync, teach mode):
    ESP32 ADC readings → Sim joint positions
    Used for: teach mode, manual positioning, calibration.
    
  Both directions use this node as the single translator.
  """
  
  SYNC_MODES = {
    "SIM_TO_REAL":  "Normal: sim drives hardware",
    "REAL_TO_SIM":  "Teach: hardware drives sim",
    "MIRROR":       "Both: commands and feedback synchronized",
    "DISABLED":     "No sync, manual control only"
  }
  
  # Joint angle calibration: real servo 0°/180° = model 0°/180°?
  # Small offset per joint to handle assembly angle variations
  CALIBRATION_OFFSETS_DEG = [0, 0, 0, 0, 0, 0]  # tuned during setup
  
  # ──────────────────────────────────────────
  # SIM → REAL
  # ──────────────────────────────────────────
  def sim_to_real_callback(self, joint_states: JointState):
    """
    Receives joint states from simulation (ros2_control).
    Converts to servo commands and sends to ESP32.
    
    Coordinate transform:
      sim uses radians (ROS standard)
      servo needs degrees (0–180°)
      
      servo_deg[i] = rad_to_deg(sim_rad[i]) + CALIBRATION_OFFSETS[i]
      
    Apply joint-to-servo mapping:
      sim joint order: [waist, shoulder, elbow, wrist_pitch, wrist_roll, gripper]
      servo pin order: [D3, D5, D6, D9, D10, D11]
      
    Publish to: /aria/servo_commands (JointState in degrees)
    """
    
  # ──────────────────────────────────────────
  # REAL → SIM
  # ──────────────────────────────────────────
  def real_to_sim_callback(self, servo_states: JointState):
    """
    Receives actual servo positions from ESP32 (ADC feedback).
    Updates simulation joint positions to match real arm.
    
    Used when in teach mode: user physically moves arm.
    Sim follows the real arm in real-time.
    
    Coordinate transform (reverse of above):
      adc_deg[i] → sim_rad[i] = deg_to_rad(adc_deg[i] - CALIBRATION_OFFSETS[i])
      
    Publishes to: /joint_states with ESP32 readings as source
    This causes the sim arm to mirror the real arm.
    
    Also: saves trajectory if record_teach=True
    Used by LearningAgent for learn mode data collection.
    """
    
  # ──────────────────────────────────────────
  # Sync mode switching
  # ──────────────────────────────────────────
  Service /aria/sync_mode (SetSyncMode.srv):
    Request:  {mode: str}
    Response: {success: bool, previous_mode: str}
    
    On switch to REAL_TO_SIM:
      1. Announce: "Teach mode activated. Arm torque will relax.
                   Move arm to desired position, then press confirm."
      2. Send /aria/teach_mode=true to ESP32 (relax servos)
      3. Start reading /aria/servo_states → update sim
      
    On switch back to SIM_TO_REAL:
      1. Current sim position = current real position (already synced)
      2. Send /aria/teach_mode=false to ESP32 (re-engage torque)
      3. Start sending sim commands to ESP32

  # ──────────────────────────────────────────
  # Calibration offset management
  # ──────────────────────────────────────────
  Service /aria/calibrate_offsets:
    """
    Fine-tune the sim↔real mapping.
    
    Process:
      1. Move arm to 5 known poses via sim commands
      2. At each pose: read sim angles + ADC feedback angles
      3. Compute per-joint offset:
           offset[i] = mean(sim_angle[i] - adc_angle[i]) over 5 poses
      4. Save to arm_control/config/servo_calibration.yaml
      5. Hot-reload offsets without restart
    """
    
  Service /aria/verify_sync:
    """
    Verify sim and real arm are synchronized.
    Sends 3 test commands, checks ADC feedback matches within 3°.
    Returns: {synchronized: bool, max_error_deg: float}
    """

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 3 — TEACH MODE (Full Implementation)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_control/arm_control/teach_mode_node.py

class TeachModeNode(Node):
  """
  Handles the full teach-by-demonstration workflow.
  
  User flow:
    aria teach start         → relax arm, start recording
    [user physically moves arm to each waypoint]
    aria teach waypoint      → record current position
    aria teach stop          → re-engage, save trajectory
    aria teach replay        → execute recorded trajectory
    aria teach save <name>   → save as named skill
  """
  
  Service /aria/teach/start:
    1. Sync mode → REAL_TO_SIM
    2. LearningAgent starts recording
    3. Log: "Teach mode started. Arm is relaxed."
    4. Terminal displays: real-time joint angles from ADC
    
  Service /aria/teach/waypoint:
    Records current ADC angles as waypoint in trajectory
    Logs: "Waypoint N recorded: [angles]"
    
  Service /aria/teach/stop:
    1. Sync mode → SIM_TO_REAL (re-engage at current position)
    2. LearningAgent stops recording
    3. Saved: episode HDF5 + waypoint YAML
    4. Reports: "Trajectory recorded: N waypoints, X seconds"
    
  Service /aria/teach/replay (name: str):
    Load trajectory from file
    Execute via joint_trajectory_controller
    Quintic spline between waypoints
    
  Service /aria/teach/save (name: str):
    Saves recorded trajectory as a named skill
    Available via SkillManager.execute_skill(name)
    
  Real-time display during teach mode:
    Terminal:
    ══════════════════════════════════════
     ARIA TEACH MODE — Arm is RELAXED
    ══════════════════════════════════════
     J1 Waist:       [ 92.3°] (ADC)
     J2 Shoulder:    [ 88.1°] (ADC)
     J3 Elbow:       [ 91.5°] (ADC)
     J4 Wrist Pitch: [ 89.2°] (ADC)
     J5 Wrist Roll:  [ 90.0°] (ADC)
     J6 Gripper:     [ 22.1°] (ADC)
    ──────────────────────────────────────
     Waypoints recorded: 0
     Recording: [●] ACTIVE
     Sim: FOLLOWING real arm in real-time
    ══════════════════════════════════════
    [W] Record waypoint | [S] Stop | [Q] Quit

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 4 — HARDWARE BRINGUP WIZARD
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_bringup/scripts/hardware_bringup_wizard.py

  Interactive CLI for first hardware setup.
  Run ONCE before first hardware use.
  
  STEP 1: ESP32 Connection
    - Detect ESP32 on serial ports (list all ttyUSB*)
    - User selects port
    - Test: ping → pong
    - Check firmware version
    - Print: "ESP32 connected ✅"
    
  STEP 2: Servo Safety Check
    "Is the workspace clear? (nothing within 30cm of arm) [y/N]"
    "Are all servo cables connected? [y/N]"
    "Is the arm currently near home position (roughly upright)? [y/N]"
    
  STEP 3: Individual Servo Test
    For each servo (j1 to j6):
      "Testing joint J{i} ({name})..."
      Wiggle ±5° from home
      "Did joint J{i} move? [y/N]"
      If N: diagnostic help printed
      
  STEP 4: Feedback Calibration (if Option A/B)
    For each joint:
      Command 0° → read ADC → store as adc_min
      Command 180° → read ADC → store as adc_max
      adc_range[i] = adc_max - adc_min
      Print: "J{i}: ADC range = {adc_min}–{adc_max} ({adc_range})"
      If range < 500: warn "Low ADC range, check pot connection"
    Save to arm_control/config/servo_calibration.yaml
    
  STEP 5: Sim-to-Real Alignment
    Command sim to HOME position
    "Does the real arm match home position? [y/N]"
    If N:
      "Is the arm within 10°? [y/N]"
      If yes: fine-tune offsets automatically
      If no:  "Move arm to home manually, then press Enter"
    Run /aria/calibrate_offsets
    Print: "Calibration offsets: [±Xdeg per joint]"
    
  STEP 6: Verify Sync
    Run /aria/verify_sync
    Print: sync accuracy per joint
    
  STEP 7: AprilTag Calibration
    "Is the AprilTag board in its fixed position? [y/N]"
    Run /aria/calibrate (from Stage 2)
    Print: "FK error before: Xmm → after: Ymm"
    
  STEP 8: Safety Layer Test
    Command a joint past soft limit
    Verify SafetyAgent intercepts it
    Print: "Safety layer functional ✅"
    
  STEP 9: Ready
    Print summary:
    ══════════════════════════════════════════
    ARIA HARDWARE SETUP COMPLETE
    ══════════════════════════════════════════
    ESP32 firmware:     v1.x ✅
    Servo feedback:     ADC (Option A) ✅ 
    Calibration offsets saved ✅
    FK error: 3.2mm ✅
    Safety layer: OK ✅
    ══════════════════════════════════════════
    Run: ros2 launch arm_bringup hardware.launch.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 5 — HARDWARE LAUNCH (Complete)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_bringup/launch/hardware.launch.py
  (Replaces the skeleton from Stage 1)
  
  Arguments:
    port:=/dev/ttyUSB0     ESP32 USB port
    calibrate:=false       Run AprilTag calibration on startup
    feedback:=adc          adc | commanded | none
    teach_mode:=false      Start in teach mode
    
  Launch sequence:
    1. robot_state_publisher (URDF → TF)
    2. micro-ros-agent (bridges ESP32 micro-ROS to ROS2)
       Command: ros2 run micro_ros_agent micro_ros_agent serial --dev {port}
    3. ros2_control (AriaHardwareInterface in REAL mode)
    4. All controllers (joint_trajectory, gripper, state_broadcaster)
    5. servo_sync_node (starts in SIM_TO_REAL mode)
    6. camera_node (opens real cameras: C270 + ESP32-CAM stream)
    7. All Stage 2 nodes (detection, depth, etc.)
    8. All Stage 3 agents
    9. Health monitor
    10. Dashboard
    11. If calibrate:=true → run apriltag_calibration_node
    
  Wait gates:
    Wait for ESP32 heartbeat before starting controllers
    Wait for all agents ACTIVE before accepting commands
    Print: "ARIA Hardware system ready."

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 6 — ARIA CLI (Full)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_bringup/scripts/aria_cli.py

  aria sim start                 → launch simulation
  aria sim manual                → launch with manual control GUI
  aria sim headless              → no GUI (for testing)
  
  aria hardware wizard           → run bringup wizard (first time)
  aria hardware start            → launch with real hardware
  aria hardware calibrate        → run full calibration
  aria hardware verify-sync      → check sim-real alignment
  
  aria teach start               → enter teach mode
  aria teach waypoint            → record current position
  aria teach stop                → end teach mode, save
  aria teach replay <name>       → execute named teach trajectory
  
  aria command "<instruction>"   → send NL command to ARIA
  aria approve                   → approve pending action
  aria reject                    → reject pending action
  aria estop                     → emergency stop
  
  aria learn start               → start learn mode recording
  aria learn stop                → stop and save dataset
  
  aria status                    → show system health
  aria benchmark ik              → run IK solver benchmark
  aria benchmark perception      → run perception accuracy test
  aria benchmark depth           → run depth estimation test
  
  aria sync sim-to-real          → sim controls real hardware
  aria sync real-to-sim          → real hardware updates sim
  aria sync off                  → disable sync

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 7 — STAGE 4 VALIDATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_bringup/scripts/validate_stage4.py

  Run WITH real hardware connected.
  
  Test 1: SIM → REAL
    In sim: command each joint to 45° then 135°
    Assert: real servo moves to match (within 3°)
    Assert: no lag > 200ms
    Print: "Sim→Real latency per joint: [ms]"
    
  Test 2: REAL → SIM
    Enable teach mode (relax arm)
    User: move each joint manually
    Assert: sim follows in real-time
    Assert: ADC reading updates /joint_states
    Print: "Real→Sim sync active ✅"
    
  Test 3: BIDIRECTIONAL CONSISTENCY
    Start in SIM_TO_REAL
    Command sim to HOME
    Switch to REAL_TO_SIM
    User: move arm 30° from home
    Switch back to SIM_TO_REAL
    Assert: sim starts from current real position
    Assert: no position jump
    Print: "Bidirectional transition: smooth ✅"
    
  Test 4: TEACH + REPLAY
    Teach mode: record 5-waypoint trajectory manually
    Replay on hardware
    Assert: trajectory executed, joints within 5° of waypoints
    
  Test 5: FULL HARDWARE TASK
    Place real object on table
    Command: "Pick up the object"
    Assert: real arm grasps object
    Assert: no collisions, no e-stop triggered
    Print: "Real-world pick success: ✅"
    
  Output:
  ══════════════════════════════════════════════
  ARIA Stage 4 — Hardware Validation
  ══════════════════════════════════════════════
  ✅ Sim→Real: max error 2.1°, latency 85ms
  ✅ Real→Sim: ADC sync active, 50Hz update
  ✅ Bidirectional: smooth transition
  ✅ Teach+Replay: waypoint accuracy 3.2°
  ✅ Real pick: SUCCESS
  ══════════════════════════════════════════════
  SYSTEM FULLY OPERATIONAL ON HARDWARE
  ══════════════════════════════════════════════
</task>

<output_order>
1.  esp32_firmware/src/main.cpp
2.  esp32_firmware/src/servo_manager.cpp
3.  esp32_firmware/src/servo_manager.h
4.  esp32_firmware/platformio.ini
5.  arm_control/arm_control/servo_sync_node.py
6.  arm_control/arm_control/teach_mode_node.py
7.  arm_bringup/scripts/hardware_bringup_wizard.py
8.  arm_bringup/launch/hardware.launch.py (COMPLETE version)
9.  arm_bringup/scripts/aria_cli.py
10. arm_bringup/scripts/validate_stage4.py
11. arm_control/config/servo_calibration.yaml (template)
12. arm_bringup/docs/HARDWARE_SETUP.md
13. arm_bringup/docs/SERVO_FEEDBACK_MOD.md (pot tapping guide)

[STAGE4 CHECKPOINT] Files: X/13 — Resume: <next_file>
</output_order>