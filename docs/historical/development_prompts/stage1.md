<context>
You are building ARIA — a 5-DoF robotic arm AI system.
This is STAGE 1 of 4.

STAGE 1 GOAL:
Physical arm in Gazebo. Manual joint control working.
NO perception, NO AI, NO noise, NO servo dynamics lag.
Only after this stage passes do we add intelligence.

VALIDATE: Every joint moves, limits are enforced, RViz matches sim.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PHYSICAL ARM (Techno-Tirupati 5-DoF, NEW GRIPPER)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Source: assembly manual. Joint assignment (from wiring diagram):

  Joint 1 — Waist:       MG995, Z-axis rotation
                         In white base cylinder (bottom)
                         Range: -90° to +90°

  Joint 2 — Shoulder:    MG995, Y-axis rotation
                         In orange waist bracket
                         Range: 0° to 180°

  Joint 3 — Elbow:       MG995, Y-axis rotation
                         At junction of Arm1 and Arm2
                         Range: 0° to 150°

  Joint 4 — Wrist Pitch: SG90, Y-axis rotation
                         At junction of Arm2 and Arm3
                         Range: -90° to +90°

  Joint 5 — Wrist Roll:  SG90, X-axis rotation
                         On gripper holder
                         Range: -90° to +90°

  Joint 6 — Gripper:     SG90, gear-driven
                         NEW GRIPPER DESIGN: 2 meshing gears,
                         4 blue curved fingers (2 pairs),
                         opens symmetrically via gear mechanism
                         Range: 0° (closed) to 45° (open)

Link dimensions (xacro params, all tunable):
  base_cylinder:      d=80mm, h=70mm  (white)
  waist_bracket:      h=35mm, w=80mm  (orange, curved)
  upper_arm (arm1):   L=145mm, 25×20mm cross section, slight curve (orange)
  forearm (arm2):     L=115mm, 30×25mm cross section, housing shape (orange)
  wrist (arm3):       L=55mm, 20×18mm (orange, small housing)
  gripper_holder:     L=40mm (white mount)
  gear_body:          30mm diameter gear mechanism housing (white)
  finger_pair:        L=80mm each, curved blue plastic
                      4 total fingers (2 left, 2 right)

Servo masses (for inertia):
  MG995: 55g, SG90: 9g
Link material: PLA, density=1240 kg/m³

Cameras:
  top_camera_link:   Logitech C270, fixed to world
                     Position: 80cm above table, 40cm in front
                     Angle: 45° downward toward workspace
                     FOV: 60° horizontal, resolution: 1280×720

  wrist_camera_link: ESP32-CAM, fixed to gripper_holder
                     Position: front face of gripper holder
                     FOV: 66° horizontal, resolution: 640×480

IMU:
  imu_link: fixed to wrist_link (represents MPU6050)
  6-axis: 3-axis gyro + 3-axis accel

DH Parameters (for IK reference in Stage 2):
  d1=0.070, a1=0,     α1=0°    (waist)
  d2=0,     a2=0,     α2=90°   (shoulder, perpendicular)
  d3=0,     a3=0.145, α3=0°    (elbow)
  d4=0,     a4=0.115, α4=0°    (wrist pitch)
  d5=0,     a5=0.055, α5=90°   (wrist roll)
  d6=0.040, a6=0,     α6=0°    (gripper)
</context>

<task>
Generate all Stage 1 files. Clean simulation only.
Every function complete, every file runnable.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 1 — ROBOT DESCRIPTION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_description/urdf/aria_arm.urdf.xacro

All dimensions as xacro:property (comment each one):
  <!-- Derived from assembly manual + photo measurements -->
  <xacro:property name="base_radius"       value="0.040"/>
  <xacro:property name="base_height"       value="0.070"/>
  <xacro:property name="upper_arm_length"  value="0.145"/>
  <xacro:property name="forearm_length"    value="0.115"/>
  <xacro:property name="wrist_length"      value="0.055"/>
  <xacro:property name="gripper_length"    value="0.040"/>
  <xacro:property name="finger_length"     value="0.080"/>
  ... etc

Links (each with visual + collision + inertial):
  base_link           → white cylinder, mesh: cylinder
  waist_link          → orange curved bracket
  upper_arm_link      → orange tube, slightly curved (approximate as cylinder)
  forearm_link        → orange rectangular housing (box)
  wrist_link          → small orange housing (box)
  gripper_holder_link → white rectangular mount
  gear_housing_link   → circular white gear housing
  finger_left_1_link  → left finger pair (one mesh for both fingers)
  finger_right_1_link → right finger pair (mirrored)
  top_camera_link     → fixed to world via stand
  wrist_camera_link   → fixed to gripper_holder_link
  imu_link            → fixed to wrist_link

Joints:
  waist_joint:          revolute, Z, -1.5708 to 1.5708 rad
  shoulder_joint:       revolute, Y, 0 to 3.1416 rad
  elbow_joint:          revolute, Y, 0 to 2.6180 rad
  wrist_pitch_joint:    revolute, Y, -1.5708 to 1.5708 rad
  wrist_roll_joint:     revolute, X, -1.5708 to 1.5708 rad
  gripper_joint:        revolute, Z, 0 to 0.7854 rad (gear opening)
  finger_mimic_joint:   revolute, mimic gripper_joint, multiplier=-1
                        (gears mesh: opposite direction)
  
  Fixed joints:
  top_camera_joint:     world → top_camera_link
                        xyz="0.40 0.0 0.80" rpy="0 0.7854 0"
  wrist_camera_joint:   gripper_holder → wrist_camera_link
                        xyz="0.02 0.0 0.01" rpy="0 0 0"
  imu_joint:            wrist_link → imu_link
                        xyz="0.0 0.0 0.0" rpy="0 0 0"

Inertia tensors:
  Compute from cylinder/box formulas.
  Comment each: "# upper_arm: cylinder 145mm×22mm, mass≈35g"
  Include servo masses at each joint location.

File: arm_description/urdf/aria_arm.gazebo.xacro

  <plugin name="gz_ros2_control">
    <!-- Use GazeboSimSystem for Gazebo Harmonic -->
  </plugin>

  Camera: top_camera
    <sensor type="camera">
      <camera>
        <horizontal_fov>1.0472</horizontal_fov>  <!-- 60 deg -->
        <image><width>1280</width><height>720</height></image>
        <clip><near>0.01</near><far>10.0</far></clip>
        <noise><type>gaussian</type>
               <mean>0.0</mean><stddev>0.0</stddev>  <!-- CLEAN -->
        </noise>
      </camera>
      <update_rate>30</update_rate>
    </sensor>

  Camera: wrist_camera
    Resolution 640×480, FOV 66°, 15fps, noise=0 (CLEAN)

  IMU: mpu6050_sim
    <sensor type="imu">
      <imu>
        <angular_velocity>
          <x><noise type="gaussian"><mean>0</mean><stddev>0</stddev></noise></x>
          ... (all zero noise, CLEAN)
        </angular_velocity>
        <linear_acceleration>... same, zero noise ...</linear_acceleration>
      </imu>
      <update_rate>100</update_rate>
    </sensor>

  Contact sensor: gripper_contact
    On finger_left_1_link and finger_right_1_link
    Publishes: /gripper/contact

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 2 — HARDWARE ABSTRACTION LAYER
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CRITICAL: This is the ONLY code that knows if we are in sim or hardware.
Everything above this layer is identical in both modes.

File: arm_control/include/arm_control/aria_hardware_interface.hpp
File: arm_control/src/aria_hardware_interface.cpp

class AriaHardwareInterface : public hardware_interface::SystemInterface {
  
  Parameters (from ros2_control URDF):
    use_sim: bool  → selects backend
    serial_port: string (for hardware mode, default /dev/ttyUSB0)
    baud_rate: int (default 115200)
  
  Joint names (in order):
    "waist_joint", "shoulder_joint", "elbow_joint",
    "wrist_pitch_joint", "wrist_roll_joint", "gripper_joint"
  
  State interfaces per joint: position, velocity
  Command interfaces per joint: position
  
  on_init(info):
    Load parameters
    Log: "ARIA Hardware Interface: mode = [SIM|HARDWARE]"
    
  on_configure(state):
    SIM:  // nothing, Gazebo handles it
          Log: "Sim mode: using Gazebo joint states"
    REAL: open serial port, verify ESP32 responds
          Send: {"cmd": "ping"}, expect: {"resp": "pong"}
          
  on_activate(state):
    SIM:  spawn controllers (handled by launch)
    REAL: enable servo torque, move to home position slowly
    
  on_deactivate(state):
    SIM:  nothing
    REAL: disable servo torque (safe power-off)
    
  read(time, period):
    SIM:  read from /joint_states subscriber (updated by Gazebo)
    REAL: read from ESP32 serial buffer
          Parse: {"pos": [j1..j6], "vel": [v1..v6], "ts": ts}
    → store in hw_positions_[], hw_velocities_[]
    
  write(time, period):
    SIM:  publish JointTrajectory to controller
    REAL: send to ESP32:
          {"cmd": [j1..j6], "gripper": gval, "ts": timestamp}
    
  Private:
    // SIM: subscriber to /joint_states
    // REAL: serial port fd, read/write buffers, CRC check
}

File: arm_control/config/aria_controllers.yaml

joint_trajectory_controller:
  ros__parameters:
    joints: [waist_joint, shoulder_joint, elbow_joint,
             wrist_pitch_joint, wrist_roll_joint]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
    gains:
      waist_joint:       {p: 100.0, d: 10.0, i: 0.01}
      shoulder_joint:    {p: 100.0, d: 10.0, i: 0.01}
      elbow_joint:       {p: 100.0, d: 10.0, i: 0.01}
      wrist_pitch_joint: {p: 50.0,  d: 5.0,  i: 0.01}
      wrist_roll_joint:  {p: 50.0,  d: 5.0,  i: 0.01}

gripper_action_controller:
  ros__parameters:
    joint: gripper_joint
    action_monitor_rate: 20.0
    goal_tolerance: 0.01

joint_state_broadcaster:
  ros__parameters:
    joints: [waist_joint, shoulder_joint, elbow_joint,
             wrist_pitch_joint, wrist_roll_joint, gripper_joint]

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 3 — GAZEBO WORLD (CLEAN)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_bringup/worlds/aria_workspace.sdf

<world name="aria_workspace">
  <physics type="dart">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>

  <!-- Clean directional lighting, no shadows yet -->
  <light type="directional" name="main_light">
    <pose>0 0 5 0 0 0</pose>
    <diffuse>0.9 0.9 0.9 1</diffuse>
    <direction>0 0.2 -1</direction>
    <cast_shadows>false</cast_shadows>  <!-- CLEAN: no shadows -->
  </light>

  <!-- Table: standard desk 80x60x75cm -->
  <model name="table">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="table_top">
      <collision><geometry><box><size>0.80 0.60 0.02</size></box></geometry>
        <surface><friction><ode><mu>0.5</mu></ode></friction></surface>
      </collision>
      <visual><geometry><box><size>0.80 0.60 0.02</size></box></geometry>
        <material><ambient>0.6 0.4 0.2 1</ambient></material>
      </visual>
      <pose>0 0 0.75 0 0 0</pose>
    </link>
    <link name="table_legs">... 4 legs, height 0.74m ...</link>
  </model>

  <!-- Arm base position on table -->
  <!-- The URDF base_link is fixed at this pose by the launch file -->
  <!-- Arm mount: left-center of table: xyz="-0.15 0.0 0.76" -->

  <!-- Camera stand (holds top_camera_link) -->
  <model name="camera_stand">
    <static>true</static>
    <pose>0.40 0.0 0.76 0 0 0</pose>
    <link name="stand_body">
      <visual><geometry><cylinder><radius>0.015</radius><length>0.80</length>
      </cylinder></geometry></visual>
      <collision>... same ...</collision>
    </link>
  </model>

  <!-- Test objects: SIMPLE COLORED SHAPES, no complex textures -->

  <model name="red_cube">
    <pose>0.20 0.05 0.775 0 0 0</pose>
    <link name="body">
      <inertial><mass>0.05</mass>...</inertial>
      <collision><geometry><box><size>0.05 0.05 0.05</size></box></geometry>
        <surface><friction><ode><mu>0.5</mu></ode></friction></surface>
      </collision>
      <visual><geometry><box><size>0.05 0.05 0.05</size></box></geometry>
        <material><ambient>0.9 0.1 0.1 1</ambient></material>
      </visual>
    </link>
  </model>

  <model name="blue_cylinder">
    <pose>0.25 -0.05 0.775 0 0 0</pose>
    mass=0.2, radius=0.03, length=0.15, blue #1E88E5
  </model>

  <model name="green_sphere">
    <pose>0.18 -0.08 0.775 0 0 0</pose>
    mass=0.03, radius=0.02, green #43A047
  </model>

  <model name="yellow_block">
    <pose>0.30 0.08 0.775 0 0 0</pose>
    mass=0.1, size=0.08x0.04x0.03, yellow #FDD835
  </model>

  <model name="white_box">  <!-- target container -->
    <pose>0.28 -0.12 0.775 0 0 0</pose>
    mass=0.1, open-top box 0.15x0.10x0.08, white
  </model>

  <!-- AprilTag board (fixed, for calibration in Stage 2) -->
  <model name="apriltag_board">
    <static>true</static>
    <pose>0.35 0.20 0.76 0 0 0</pose>
    <link name="board">
      <visual><geometry><box><size>0.10 0.10 0.001</size></box></geometry>
        <material><!-- white with tag pattern, use solid white for now -->
        </material>
      </visual>
    </link>
  </model>
</world>

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 4 — MANUAL CONTROL SYSTEM (First-Class Feature)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_control/scripts/manual_control_node.py

class ManualControlNode(Node):
  """
  Primary manual control interface for ARIA.
  All control methods publish to /joint_trajectory_controller.
  Safety: soft limits (1° margin), max speed 90°/s, e-stop.
  """

  JOINT_NAMES = ["waist", "shoulder", "elbow",
                 "wrist_pitch", "wrist_roll", "gripper"]
  
  # Home = all joints at 90° (center), gripper slightly open
  HOME_ANGLES_DEG = [90.0, 90.0, 90.0, 90.0, 90.0, 20.0]
  
  # Joint limits in degrees [min, max]
  JOINT_LIMITS_DEG = [
    [-90.0,  90.0],  # waist
    [  0.0, 180.0],  # shoulder
    [  0.0, 150.0],  # elbow
    [-90.0,  90.0],  # wrist_pitch
    [-90.0,  90.0],  # wrist_roll
    [  0.0,  45.0],  # gripper (0=closed, 45=open)
  ]
  
  SOFT_LIMIT_MARGIN_DEG = 1.0  # degrees before hard limit

  Services:
    /aria/set_joint (SetJoint.srv):
      request:  {joint_name: str, angle_deg: float, speed_deg_per_s: float}
      response: {success: bool, message: str, actual_angle_deg: float}
      
    /aria/set_all_joints (SetAllJoints.srv):
      request:  {angles_deg: float[6], speed_deg_per_s: float}
      response: {success: bool, expected_duration_s: float}
      
    /aria/go_named_pose (GoNamedPose.srv):
      request:  {pose_name: str}  # "home","ready","folded","inspect"
      response: {success: bool, message: str}
      Named poses:
        home:    [90, 90, 90, 90, 90, 20]
        ready:   [90, 45, 135, 90, 90, 20]
        folded:  [90, 160, 20, 90, 90, 10]
        inspect: [90, 60, 120, 60, 90, 20]
      
    /aria/open_gripper:  response {success}
    /aria/close_gripper: response {success}
    /aria/estop:         response {success}
                         IMMEDIATE: cancel all motion, hold position
    /aria/release_estop: response {success}
    
  Topics subscribed:
    /aria/joint_stream (sensor_msgs/JointState):
      Streaming control at up to 50Hz
      Each message processed only if not in e-stop
      
  Topics published:
    /aria/manual_status (10Hz):
      {
        current_angles_deg: [6],
        target_angles_deg: [6],
        motion_complete: bool,
        estop_active: bool,
        joints_within_limits: bool,
        timestamp: stamp
      }
      
  Safety checks (on EVERY command):
    1. E-stop check: if active, reject all motion commands
    2. Limit check: soft limits enforced, log if near hard limit
    3. Speed check: cap at 90°/s per joint
    4. NaN check: reject any NaN/Inf values

File: arm_control/scripts/keyboard_control.py

  Terminal keyboard control node.
  Uses curses for clean display.
  
  Key mappings:
    q/a: joint 1 (waist)         ±step
    w/s: joint 2 (shoulder)      ±step
    e/d: joint 3 (elbow)         ±step
    r/f: joint 4 (wrist pitch)   ±step
    t/g: joint 5 (wrist roll)    ±step
    y/h: joint 6 (gripper)       open/close
    
    1: step = 1°  (fine)
    5: step = 5°  (default)
    0: step = 15° (coarse)
    
    SPACE: go_home
    p:     cycle named poses (home→ready→folded→inspect)
    ESC/x: emergency stop
    c:     clear e-stop
    q:     quit keyboard control
    
  Terminal display (updates at 10Hz):
  ╔══════════════════════════════════════╗
  ║  ARIA Keyboard Control   [RUNNING]   ║
  ╠══════════════════════════════════════╣
  ║  J1 Waist:       [ 90.0°] ← q/a     ║
  ║  J2 Shoulder:    [ 90.0°] ← w/s     ║
  ║  J3 Elbow:       [ 90.0°] ← e/d     ║
  ║  J4 Wrist Pitch: [ 90.0°] ← r/f     ║
  ║  J5 Wrist Roll:  [ 90.0°] ← t/g     ║
  ║  J6 Gripper:     [ 20.0°] ← y/h     ║
  ╠══════════════════════════════════════╣
  ║  Step: 5° [1=fine|5=med|0=coarse]   ║
  ║  SPACE:Home  p:Poses  ESC:E-Stop    ║
  ╠══════════════════════════════════════╣
  ║  Status: AT POSITION                 ║
  ╚══════════════════════════════════════╝

File: arm_control/scripts/joint_slider_gui.py

  Tkinter GUI. NO web dependencies.
  
  Layout:
  ┌─ ARIA Joint Control ─────────────────────────┐
  │  ┌─ Joints ────────────────────────────────┐ │
  │  │  Waist       [━━━━━━●─────] 90.0°       │ │
  │  │  Shoulder    [━━━━━━●─────] 90.0°       │ │
  │  │  Elbow       [━━━━━━●─────] 90.0°       │ │
  │  │  Wrist Pitch [━━━━━━●─────] 90.0°       │ │
  │  │  Wrist Roll  [━━━━━━●─────] 90.0°       │ │
  │  │  Gripper     [●─────────── ] 20.0°      │ │
  │  └─────────────────────────────────────────┘ │
  │  [Send to Robot] [Home] [Ready] [Folded]      │
  │  [Open Gripper]  [Close Gripper]              │
  │  [🔴 E-STOP]     [Release E-Stop]            │
  │  ─────────────────────────────────────────── │
  │  ┌─ Live Status ───────────────────────────┐ │
  │  │  Actual:  [90.0, 90.0, 90.0, 90.0...]  │ │
  │  │  Target:  [90.0, 90.0, 90.0, 90.0...]  │ │
  │  │  Status:  AT POSITION                   │ │
  │  └─────────────────────────────────────────┘ │
  │  ┌─ 2D View ───────────────────────────────┐ │
  │  │  [Line drawing of arm, updates 10Hz]    │ │
  │  │  Side view: base → upper → fore → grip  │ │
  │  └─────────────────────────────────────────┘ │
  └───────────────────────────────────────────────┘

  2D arm view: canvas drawing using current joint angles
    Uses forward kinematics (simple 2D projection)
    Shows workspace boundary circle
    Highlights joint if near limit (orange) or at limit (red)
  
  Slider range: set from JOINT_LIMITS_DEG
  Slider color: green (safe) → orange (<10° from limit) → red (at limit)
  
  Thread-safe: GUI in main thread, ROS2 in daemon thread

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 5 — LAUNCH SYSTEM
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_bringup/launch/sim.launch.py
  Arguments:
    use_rviz:=true/false   (default: true)
    use_gz_gui:=true/false (default: true)
    paused:=true/false     (default: false)
  
  Launch sequence:
    1. Gazebo Harmonic with aria_workspace.sdf
    2. robot_state_publisher (URDF → TF)
    3. Spawn ARIA robot model in Gazebo at table position
    4. gz_ros2_control spawner
    5. joint_trajectory_controller spawner
    6. gripper_action_controller spawner
    7. joint_state_broadcaster spawner
    8. manual_control_node
    9. RViz2 with aria_rviz.rviz (if use_rviz:=true)
    
  Wait conditions:
    - Wait for Gazebo to be ready before spawning robot
    - Wait for controllers to be active before launching control node
    - Log: "ARIA Simulation ready. Launch keyboard or GUI control."

File: arm_bringup/launch/manual_control.launch.py
  Includes sim.launch.py with use_rviz:=true
  Adds: keyboard_control.py (in separate terminal)
  Adds: joint_slider_gui.py
  
  This is the RECOMMENDED first-launch file.
  README should say: "Start here → aria_launch manual_control.launch.py"

File: arm_bringup/launch/hardware.launch.py
  (Skeleton for Stage 4 — do NOT run yet)
  Arguments: port:=/dev/ttyUSB0, calibrate:=false
  Comment: "# Complete implementation in Stage 4 hardware prompt"

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 6 — RVIZ CONFIGURATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_bringup/config/aria_rviz.rviz

Panels:
  Displays:
    - RobotModel (alpha=1.0, show collision=false, show visual=true)
    - TF (show names=true, show axes=true, marker scale=0.1)
    - Grid (z=0.75, plane=XY, color=gray, lines=20)
    - Image: /top_camera/image_raw    (top-left viewport)
    - Image: /wrist_camera/image_raw  (bottom-left viewport)
    - Axes at /tool_frame (frame for end-effector, scale=0.05)

Views:
  Default: isometric, looking down at workspace
  Camera pos: xyz=[0.5, -0.5, 0.8], pointing at [0, 0, 0.75]
  
Fixed frame: base_link

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 7 — STAGE 1 VALIDATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_bringup/scripts/validate_stage1.py

Automated test checklist. Run after first launch.

Tests:
  1. URDF loads without errors
     → check /robot_description is published
  
  2. All 6 joint names in /joint_states
     → subscribe for 2s, verify all joints present
     
  3. Each joint responds to command
     → command each joint to 45° then back to 90°
     → verify actual position reaches target (±2°)
     
  4. Joint limits enforced
     → command waist to 180° (beyond limit)
     → verify clipped to 90° (soft limit)
     
  5. Home position reachable
     → call /aria/go_named_pose "home"
     → verify all joints within 2° of HOME_ANGLES_DEG
     
  6. Named poses reachable
     → cycle through: home, ready, folded, inspect
     
  7. Gripper opens and closes
     → close gripper → verify angle < 5°
     → open gripper  → verify angle > 40°
     
  8. Camera topics active
     → /top_camera/image_raw: receiving at ≥25fps
     → /wrist_camera/image_raw: receiving at ≥10fps
     
  9. IMU topic active
     → /mpu6050/imu_raw: receiving at ≥50Hz
     
  10. E-stop works
      → activate e-stop
      → send joint command → verify rejected
      → release e-stop → verify commands accepted
      
  11. TF tree complete
      → verify: world → base_link → ... → gripper_holder_link
      → verify: world → top_camera_link
      
  12. No NaN in joint states (monitor for 10s)
  
Output:
  ══════════════════════════════════
  ARIA Stage 1 Validation Results
  ══════════════════════════════════
  ✅ PASS URDF loads
  ✅ PASS All 6 joints in /joint_states
  ✅ PASS Joint response (waist: ✅ shoulder: ✅ ...)
  ✅ PASS Joint limits enforced
  ✅ PASS Home position
  ✅ PASS Named poses
  ✅ PASS Gripper
  ✅ PASS Camera topics
  ✅ PASS IMU topic
  ✅ PASS E-stop
  ✅ PASS TF tree
  ✅ PASS No NaN
  ══════════════════════════════════
  RESULT: STAGE 1 COMPLETE — Ready for Stage 2
  ══════════════════════════════════
</task>

<output_order>
Announce each file: ═══ FILE: path/to/file ═══

1.  arm_description/urdf/aria_arm.urdf.xacro
2.  arm_description/urdf/aria_arm.gazebo.xacro
3.  arm_description/package.xml
4.  arm_description/CMakeLists.txt
5.  arm_control/include/arm_control/aria_hardware_interface.hpp
6.  arm_control/src/aria_hardware_interface.cpp
7.  arm_control/config/aria_controllers.yaml
8.  arm_control/config/ros2_control.yaml
9.  arm_control/package.xml
10. arm_control/CMakeLists.txt
11. arm_bringup/worlds/aria_workspace.sdf
12. arm_control/scripts/manual_control_node.py
13. arm_control/scripts/keyboard_control.py
14. arm_control/scripts/joint_slider_gui.py
15. arm_bringup/launch/sim.launch.py
16. arm_bringup/launch/manual_control.launch.py
17. arm_bringup/launch/hardware.launch.py (skeleton)
18. arm_bringup/config/aria_rviz.rviz
19. arm_bringup/scripts/validate_stage1.py
20. arm_bringup/package.xml
21. arm_bringup/CMakeLists.txt
22. README_STAGE1.md

[STAGE1 CHECKPOINT] Files: X/22 — Resume: <next_file>
</output_order>