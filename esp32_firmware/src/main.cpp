// ═══════════════════════════════════════════════════════════════
// ARIA ESP32 Firmware — main.cpp
// micro-ROS node: "aria_esp32"
//
// Bridges ROS2 ↔ real servo hardware.
// Subscribers: servo commands, gripper, teach mode
// Publishers:  servo states (50Hz), health, heartbeat (1Hz)
//
// Build with PlatformIO:
//   pio run -e esp32dev --target upload
// ═══════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
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
rcl_publisher_t      pub_health;
rcl_publisher_t      pub_heartbeat;

// Timers
rcl_timer_t          timer_servo_states;
rcl_timer_t          timer_heartbeat;

// Messages
sensor_msgs__msg__JointState msg_servo_commands;
std_msgs__msg__Float32       msg_gripper_command;
std_msgs__msg__Bool          msg_teach_mode;
sensor_msgs__msg__JointState msg_servo_states;
std_msgs__msg__Float32MultiArray msg_health;
std_msgs__msg__Header        msg_heartbeat;

// State
bool     teach_mode_active = false;
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
// TIMER: Publish heartbeat at 1Hz
// ═══════════════════════════════════════════════════════════════
void timer_heartbeat_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    msg_heartbeat.stamp.sec = (int32_t)(millis() / 1000);
    msg_heartbeat.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

    RCSOFTCHECK(rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL));

    // Also publish health data
    // [uptime_s, vcc_estimate, adc_valid, feedback_mode]
    float uptime_s = millis() / 1000.0f;
    msg_health.data.data[0] = uptime_s;
    msg_health.data.data[1] = 5.0f;  // VCC estimate (TODO: actual measurement)
    msg_health.data.data[2] = (servo_mgr.get_feedback_mode() == FEEDBACK_ADC) ? 1.0f : 0.0f;
    msg_health.data.data[3] = teach_mode_active ? 1.0f : 0.0f;

    RCSOFTCHECK(rcl_publish(&pub_health, &msg_health, NULL));

    // Log status
    Serial.printf("[ARIA] t=%.0fs teach=%d feedback=%s\n",
                  uptime_s,
                  teach_mode_active,
                  servo_mgr.get_feedback_mode() == FEEDBACK_ADC ? "ADC" : "CMD");
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
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n═══════════════════════════════════════");
    Serial.println("  ARIA ESP32 Firmware v1.0.0");
    Serial.println("═══════════════════════════════════════");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

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

    RCCHECK(rclc_timer_init_default(
        &timer_heartbeat, &support,
        RCL_MS_TO_NS(1000 / HEARTBEAT_HZ),
        timer_heartbeat_callback));

    // ── Executor (3 subs + 2 timers = 5 handles) ──
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
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
    RCCHECK(rclc_executor_add_timer(&executor, &timer_heartbeat));

    // Allocate message memory
    allocate_messages();

    prev_loop_time = millis();
    last_command_time = millis();

    Serial.println("[ARIA] micro-ROS node 'aria_esp32' initialized.");
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
        // No commands received — hold current position (already holding,
        // just don't move further). This is passive safety.
    }

    // ── Loop rate control (~200Hz internal loop) ──
    uint32_t elapsed = millis() - now;
    if (elapsed < 5) {
        delay(5 - elapsed);
    }
}
