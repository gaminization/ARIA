// ═══════════════════════════════════════════════════════════════
// ARIA ESP32 Firmware v1.1.0 — main.cpp
// micro-ROS node: "aria_esp32"
//
// Bridges ROS2 ↔ real servo hardware + MPU6050 end-effector IMU.
// Subscribers: servo commands, gripper, teach mode
// Publishers:  servo states (50Hz), imu (100Hz), health, heartbeat (1Hz)
//
// Build with PlatformIO:
//   pio run -e esp32dev --target upload
// ═══════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/header.h>

#include "servo_manager.h"

// ═══════════════════════════════════════════════════════════════
// CONFIGURATION — Edit for your setup
// ═══════════════════════════════════════════════════════════════
#define USE_WIFI          true
#define WIFI_SSID         "YOUR_WIFI_SSID"
#define WIFI_PASS         "YOUR_WIFI_PASSWORD"
#define ROS_AGENT_IP      "192.168.1.100"
#define ROS_AGENT_PORT    8888

// Servo pins (matching assembly manual wiring diagram)
#define SERVO_WAIST_PIN        3
#define SERVO_SHOULDER_PIN     5
#define SERVO_ELBOW_PIN        6
#define SERVO_WRIST_ROLL_PIN   9
#define SERVO_WRIST_PITCH_PIN  10
#define SERVO_GRIPPER_PIN      11

// ADC pins for position feedback (Option A: tapped pot wiper)
#define ADC_WAIST_PIN          34
#define ADC_SHOULDER_PIN       35
#define ADC_ELBOW_PIN          32
#define ADC_WRIST_ROLL_PIN     33
#define ADC_WRIST_PITCH_PIN    25
#define ADC_GRIPPER_PIN        26

// MPU6050 I2C Configuration (End-effector IMU)
#define MPU6050_I2C_SDA_PIN    21
#define MPU6050_I2C_SCL_PIN    22
#define MPU6050_I2C_ADDR       0x68

// Servo calibration (pulse widths in microseconds)
// CALIBRATE THESE FOR YOUR SPECIFIC SERVOS
#define MG995_MIN_PULSE  600
#define MG995_MAX_PULSE  2400
#define SG90_MIN_PULSE   500
#define SG90_MAX_PULSE   2500

// Feedback mode: FEEDBACK_ADC or FEEDBACK_COMMANDED
#define INITIAL_FEEDBACK_MODE  FEEDBACK_ADC

// Timing
#define SERVO_UPDATE_HZ   50     // Servo state publish rate
#define IMU_UPDATE_HZ     100    // IMU publish rate (100Hz)
#define HEARTBEAT_HZ      1      // Heartbeat rate
#define COMMAND_TIMEOUT_MS 500   // Hold position if no cmd received

// Safety
#define MAX_SPEED_DPS     120.0f // Max servo speed (degrees/second)
#define VCC_WARN_VOLTAGE  4.5f   // Brownout warning threshold

// ═══════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════
ServoManager servo_mgr;

// micro-ROS entities
rcl_allocator_t      allocator;
rclc_support_t       support;
rcl_node_t           node;
rclc_executor_t      executor;

// Subscribers
rcl_subscription_t   sub_servo_commands;
rcl_subscription_t   sub_gripper_command;
rcl_subscription_t   sub_teach_mode;

// Publishers
rcl_publisher_t      pub_servo_states;
rcl_publisher_t      pub_imu;
rcl_publisher_t      pub_health;
rcl_publisher_t      pub_heartbeat;

// Timers
rcl_timer_t          timer_servo_states;
rcl_timer_t          timer_imu;
rcl_timer_t          timer_heartbeat;

// Messages
sensor_msgs__msg__JointState msg_servo_commands;
std_msgs__msg__Float32       msg_gripper_command;
std_msgs__msg__Bool          msg_teach_mode;
sensor_msgs__msg__JointState msg_servo_states;
sensor_msgs__msg__Imu        msg_imu;
std_msgs__msg__Float32MultiArray msg_health;
std_msgs__msg__Header        msg_heartbeat;

// State
bool     teach_mode_active = false;
bool     mpu6050_available = false;
uint32_t last_command_time = 0;
uint32_t loop_start_time = 0;
uint32_t prev_loop_time = 0;

// Joint name strings for micro-ROS messages
const char* joint_names[NUM_JOINTS] = {
    "waist_joint", "shoulder_joint", "elbow_joint",
    "wrist_pitch_joint", "wrist_roll_joint", "gripper_joint"
};

// ═══════════════════════════════════════════════════════════════
// ERROR HANDLING MACROS
// ═══════════════════════════════════════════════════════════════
#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { \
    Serial.printf("[ERROR] micro-ROS call failed: %d at line %d\n", rc, __LINE__); \
    error_loop(); }}

#define RCSOFTCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { \
    Serial.printf("[WARN] micro-ROS soft error: %d at line %d\n", rc, __LINE__); }}

void error_loop() {
    while (true) {
        // Blink built-in LED rapidly to indicate error
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

// ═══════════════════════════════════════════════════════════════
// MPU6050 SENSOR ROUTINES
// ═══════════════════════════════════════════════════════════════
bool init_mpu6050() {
    Wire.begin(MPU6050_I2C_SDA_PIN, MPU6050_I2C_SCL_PIN, 400000);
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up MPU6050
    if (Wire.endTransmission() != 0) {
        Serial.println("[WARN] MPU6050 IMU not detected at I2C address 0x68");
        return false;
    }

    // Configure Gyro Full Scale: +/- 500 deg/s
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x1B); // GYRO_CONFIG
    Wire.write(0x08);
    Wire.endTransmission();

    // Configure Accel Full Scale: +/- 4g
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x1C); // ACCEL_CONFIG
    Wire.write(0x08);
    Wire.endTransmission();

    Serial.println("[ARIA] MPU6050 End-Effector IMU initialized (I2C 0x68).");
    return true;
}

void read_mpu6050(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    if (!mpu6050_available) {
        ax = 0.0f; ay = 0.0f; az = 9.80665f;
        gx = 0.0f; gy = 0.0f; gz = 0.0f;
        return;
    }

    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(0x3B); // Starting at ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) {
        ax = 0.0f; ay = 0.0f; az = 9.80665f;
        gx = 0.0f; gy = 0.0f; gz = 0.0f;
        return;
    }

    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (size_t)14, true);
    if (Wire.available() >= 14) {
        int16_t raw_ax = (Wire.read() << 8) | Wire.read();
        int16_t raw_ay = (Wire.read() << 8) | Wire.read();
        int16_t raw_az = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Skip temperature (TEMP_OUT_H, TEMP_OUT_L)
        int16_t raw_gx = (Wire.read() << 8) | Wire.read();
        int16_t raw_gy = (Wire.read() << 8) | Wire.read();
        int16_t raw_gz = (Wire.read() << 8) | Wire.read();

        // Convert to SI units:
        // +/- 4g sensitivity = 8192 LSB/g -> m/s^2
        ax = (raw_ax / 8192.0f) * 9.80665f;
        ay = (raw_ay / 8192.0f) * 9.80665f;
        az = (raw_az / 8192.0f) * 9.80665f;

        // +/- 500 deg/s sensitivity = 65.5 LSB/(deg/s) -> rad/s
        gx = (raw_gx / 65.5f) * (float)(M_PI / 180.0f);
        gy = (raw_gy / 65.5f) * (float)(M_PI / 180.0f);
        gz = (raw_gz / 65.5f) * (float)(M_PI / 180.0f);
    }
}

// ═══════════════════════════════════════════════════════════════
// CALLBACK: /aria/servo_commands (JointState)
// ═══════════════════════════════════════════════════════════════
void servo_commands_callback(const void* msg_in) {
    const sensor_msgs__msg__JointState* msg =
        (const sensor_msgs__msg__JointState*)msg_in;

    if (teach_mode_active) {
        // In teach mode, ignore incoming commands
        return;
    }

    last_command_time = millis();

    // Apply position commands (angles in degrees)
    size_t count = msg->position.size;
    if (count > NUM_JOINTS) count = NUM_JOINTS;

    for (size_t i = 0; i < count; i++) {
        servo_mgr.set_angle(i, (float)msg->position.data[i]);
    }
}

// ═══════════════════════════════════════════════════════════════
// CALLBACK: /aria/gripper_command (Float32: 0.0=open, 1.0=closed)
// ═══════════════════════════════════════════════════════════════
void gripper_command_callback(const void* msg_in) {
    const std_msgs__msg__Float32* msg =
        (const std_msgs__msg__Float32*)msg_in;

    if (teach_mode_active) return;

    last_command_time = millis();

    // Map 0.0–1.0 to gripper range (open=55°, closed=0°)
    float grip_pct = constrain(msg->data, 0.0f, 1.0f);
    float grip_deg = 55.0f * (1.0f - grip_pct);  // 0%→55° (open), 100%→0° (closed)
    servo_mgr.set_angle(JOINT_GRIPPER, grip_deg);
}

// ═══════════════════════════════════════════════════════════════
// CALLBACK: /aria/teach_mode (Bool)
// ═══════════════════════════════════════════════════════════════
void teach_mode_callback(const void* msg_in) {
    const std_msgs__msg__Bool* msg =
        (const std_msgs__msg__Bool*)msg_in;

    if (msg->data && !teach_mode_active) {
        // Entering teach mode — relax all servos
        teach_mode_active = true;
        servo_mgr.disable_all_torque();
        Serial.println("[ARIA] TEACH MODE ACTIVATED — servos relaxed");
    }
    else if (!msg->data && teach_mode_active) {
        // Exiting teach mode — re-engage at current position
        teach_mode_active = false;
        servo_mgr.enable_all_torque();
        Serial.println("[ARIA] TEACH MODE DEACTIVATED — servos re-engaged");
    }
}

// ═══════════════════════════════════════════════════════════════
// TIMER: Publish servo states at 50Hz
// ═══════════════════════════════════════════════════════════════
void timer_servo_states_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    // Fill position and velocity arrays
    for (int i = 0; i < NUM_JOINTS; i++) {
        msg_servo_states.position.data[i] = servo_mgr.get_angle_feedback(i);
        msg_servo_states.velocity.data[i] = servo_mgr.get_velocity(i);
    }

    // Timestamp
    msg_servo_states.header.stamp.sec = (int32_t)(millis() / 1000);
    msg_servo_states.header.stamp.nanosec =
        (uint32_t)((millis() % 1000) * 1000000);

    RCSOFTCHECK(rcl_publish(&pub_servo_states, &msg_servo_states, NULL));
}

// ═══════════════════════════════════════════════════════════════
// TIMER: Publish IMU states at 100Hz
// ═══════════════════════════════════════════════════════════════
void timer_imu_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    float ax = 0.0f, ay = 0.0f, az = 9.80665f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    read_mpu6050(ax, ay, az, gx, gy, gz);

    msg_imu.linear_acceleration.x = ax;
    msg_imu.linear_acceleration.y = ay;
    msg_imu.linear_acceleration.z = az;

    msg_imu.angular_velocity.x = gx;
    msg_imu.angular_velocity.y = gy;
    msg_imu.angular_velocity.z = gz;

    // Default identity orientation quaternion
    msg_imu.orientation.w = 1.0;
    msg_imu.orientation.x = 0.0;
    msg_imu.orientation.y = 0.0;
    msg_imu.orientation.z = 0.0;

    msg_imu.header.stamp.sec = (int32_t)(millis() / 1000);
    msg_imu.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

    RCSOFTCHECK(rcl_publish(&pub_imu, &msg_imu, NULL));
}

// ═══════════════════════════════════════════════════════════════
// TIMER: Publish heartbeat at 1Hz
// ═══════════════════════════════════════════════════════════════
void timer_heartbeat_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    msg_heartbeat.stamp.sec = (int32_t)(millis() / 1000);
    msg_heartbeat.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

    RCSOFTCHECK(rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL));

    // Also publish health data
    // [uptime_s, vcc_estimate, adc_valid, feedback_mode, imu_active]
    float uptime_s = millis() / 1000.0f;
    msg_health.data.data[0] = uptime_s;
    msg_health.data.data[1] = 5.0f;  // VCC estimate
    msg_health.data.data[2] = (servo_mgr.get_feedback_mode() == FEEDBACK_ADC) ? 1.0f : 0.0f;
    msg_health.data.data[3] = teach_mode_active ? 1.0f : 0.0f;

    RCSOFTCHECK(rcl_publish(&pub_health, &msg_health, NULL));

    // Log status
    Serial.printf("[ARIA] t=%.0fs teach=%d feedback=%s imu=%s\n",
                  uptime_s,
                  teach_mode_active,
                  servo_mgr.get_feedback_mode() == FEEDBACK_ADC ? "ADC" : "CMD",
                  mpu6050_available ? "OK" : "NONE");
}

// ═══════════════════════════════════════════════════════════════
// MEMORY ALLOCATION for micro-ROS messages
// ═══════════════════════════════════════════════════════════════
void allocate_messages() {
    // ── Servo commands (subscriber) ──
    msg_servo_commands.position.data = (double*)malloc(NUM_JOINTS * sizeof(double));
    msg_servo_commands.position.size = NUM_JOINTS;
    msg_servo_commands.position.capacity = NUM_JOINTS;
    msg_servo_commands.velocity.data = (double*)malloc(NUM_JOINTS * sizeof(double));
    msg_servo_commands.velocity.size = NUM_JOINTS;
    msg_servo_commands.velocity.capacity = NUM_JOINTS;
    msg_servo_commands.name.data = (rosidl_runtime_c__String*)
        malloc(NUM_JOINTS * sizeof(rosidl_runtime_c__String));
    msg_servo_commands.name.size = NUM_JOINTS;
    msg_servo_commands.name.capacity = NUM_JOINTS;

    // ── Servo states (publisher) ──
    msg_servo_states.position.data = (double*)malloc(NUM_JOINTS * sizeof(double));
    msg_servo_states.position.size = NUM_JOINTS;
    msg_servo_states.position.capacity = NUM_JOINTS;
    msg_servo_states.velocity.data = (double*)malloc(NUM_JOINTS * sizeof(double));
    msg_servo_states.velocity.size = NUM_JOINTS;
    msg_servo_states.velocity.capacity = NUM_JOINTS;
    msg_servo_states.name.data = (rosidl_runtime_c__String*)
        malloc(NUM_JOINTS * sizeof(rosidl_runtime_c__String));
    msg_servo_states.name.size = NUM_JOINTS;
    msg_servo_states.name.capacity = NUM_JOINTS;

    for (int i = 0; i < NUM_JOINTS; i++) {
        msg_servo_states.position.data[i] = 0.0;
        msg_servo_states.velocity.data[i] = 0.0;
        rosidl_runtime_c__String__assign(
            &msg_servo_states.name.data[i], joint_names[i]);
    }

    // ── Health (publisher) ──
    msg_health.data.data = (float*)malloc(4 * sizeof(float));
    msg_health.data.size = 4;
    msg_health.data.capacity = 4;
    for (int i = 0; i < 4; i++) msg_health.data.data[i] = 0.0f;

    // ── Frame IDs ──
    static const char frame_id[] = "aria_esp32";
    msg_servo_states.header.frame_id.data = (char*)frame_id;
    msg_servo_states.header.frame_id.size = strlen(frame_id);
    msg_servo_states.header.frame_id.capacity = strlen(frame_id) + 1;

    msg_heartbeat.frame_id.data = (char*)frame_id;
    msg_heartbeat.frame_id.size = strlen(frame_id);
    msg_heartbeat.frame_id.capacity = strlen(frame_id) + 1;

    static const char imu_frame_id[] = "end_effector_imu";
    msg_imu.header.frame_id.data = (char*)imu_frame_id;
    msg_imu.header.frame_id.size = strlen(imu_frame_id);
    msg_imu.header.frame_id.capacity = strlen(imu_frame_id) + 1;

    // Standard covariance diagonal
    for (int i = 0; i < 9; i++) {
        msg_imu.orientation_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
        msg_imu.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.001 : 0.0;
        msg_imu.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.05 : 0.0;
    }
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n═══════════════════════════════════════");
    Serial.println("  ARIA ESP32 Firmware v1.1.0");
    Serial.println("  Hardware: SG90/MG995 + MPU6050 IMU");
    Serial.println("═══════════════════════════════════════");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // ── Initialize MPU6050 I2C sensor ──
    mpu6050_available = init_mpu6050();

    // ── Initialize servo manager ──
    servo_mgr.set_feedback_mode(INITIAL_FEEDBACK_MODE);
    servo_mgr.init();

    // ── micro-ROS transport ──
#if USE_WIFI
    Serial.printf("[ARIA] Connecting to WiFi: %s\n", WIFI_SSID);
    set_microros_wifi_transports(
        (char*)WIFI_SSID, (char*)WIFI_PASS,
        (char*)ROS_AGENT_IP, ROS_AGENT_PORT
    );
    Serial.println("[ARIA] WiFi connected, micro-ROS transport ready.");
#else
    Serial.println("[ARIA] Using USB serial transport.");
    set_microros_serial_transports(Serial);
#endif

    delay(2000);  // Wait for agent connection

    // ── micro-ROS init ──
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "aria_esp32", "", &support));

    // ── Subscribers ──
    RCCHECK(rclc_subscription_init_default(
        &sub_servo_commands, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/aria/servo_commands"));

    RCCHECK(rclc_subscription_init_default(
        &sub_gripper_command, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/aria/gripper_command"));

    RCCHECK(rclc_subscription_init_default(
        &sub_teach_mode, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/aria/teach_mode"));

    // ── Publishers ──
    RCCHECK(rclc_publisher_init_default(
        &pub_servo_states, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/aria/servo_states"));

    RCCHECK(rclc_publisher_init_default(
        &pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/aria/imu"));

    RCCHECK(rclc_publisher_init_default(
        &pub_health, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/aria/esp32_health"));

    RCCHECK(rclc_publisher_init_default(
        &pub_heartbeat, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
        "/aria/heartbeat"));

    // ── Timers ──
    unsigned int servo_period_ms = 1000 / SERVO_UPDATE_HZ;
    RCCHECK(rclc_timer_init_default(
        &timer_servo_states, &support,
        RCL_MS_TO_NS(servo_period_ms),
        timer_servo_states_callback));

    unsigned int imu_period_ms = 1000 / IMU_UPDATE_HZ;
    RCCHECK(rclc_timer_init_default(
        &timer_imu, &support,
        RCL_MS_TO_NS(imu_period_ms),
        timer_imu_callback));

    RCCHECK(rclc_timer_init_default(
        &timer_heartbeat, &support,
        RCL_MS_TO_NS(1000 / HEARTBEAT_HZ),
        timer_heartbeat_callback));

    // ── Executor (3 subs + 3 timers = 6 handles) ──
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &sub_servo_commands,
        &msg_servo_commands, &servo_commands_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &sub_gripper_command,
        &msg_gripper_command, &gripper_command_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &sub_teach_mode,
        &msg_teach_mode, &teach_mode_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_servo_states));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_heartbeat));

    // Allocate message memory
    allocate_messages();

    prev_loop_time = millis();
    last_command_time = millis();

    Serial.println("[ARIA] micro-ROS node 'aria_esp32' initialized with MPU6050 IMU.");
    Serial.println("[ARIA] Waiting for commands...");
    digitalWrite(LED_BUILTIN, LOW);
}

// ═══════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
    uint32_t now = millis();
    uint32_t dt_ms = now - prev_loop_time;
    prev_loop_time = now;

    // ── Process micro-ROS callbacks ──
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

    // ── Update servos (rate limiting + write PWM) ──
    servo_mgr.update(dt_ms);

    // ── Command timeout safety ──
    if (!teach_mode_active && (now - last_command_time) > COMMAND_TIMEOUT_MS) {
        // No commands received — hold current position
    }

    // ── Loop rate control (~200Hz internal loop) ──
    uint32_t elapsed = millis() - now;
    if (elapsed < 5) {
        delay(5 - elapsed);
    }
}
