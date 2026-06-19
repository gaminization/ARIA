// ═══════════════════════════════════════════════════════════════
// ARIA ESP32 Servo Manager — Header
// Manages 6 servos: 3×MG995 (waist, shoulder, elbow)
//                    3×SG90  (wrist pitch, wrist roll, gripper)
// ═══════════════════════════════════════════════════════════════
#pragma once

#include <ESP32Servo.h>

// ── Servo types ───────────────────────────────────────────────
enum ServoType {
    SERVO_MG995,
    SERVO_SG90
};

// ── Feedback modes ────────────────────────────────────────────
enum FeedbackMode {
    FEEDBACK_ADC,        // Option A/B: read potentiometer via ADC
    FEEDBACK_COMMANDED   // Option C: track commanded position only
};

// ── Per-joint configuration ───────────────────────────────────
struct JointConfig {
    const char* name;
    uint8_t     servo_pin;
    uint8_t     adc_pin;
    ServoType   type;
    float       min_deg;
    float       max_deg;
    float       home_deg;
    uint16_t    min_pulse_us;
    uint16_t    max_pulse_us;
    float       adc_min;       // ADC reading at min_deg (calibrated)
    float       adc_max;       // ADC reading at max_deg (calibrated)
    float       speed_limit;   // max degrees per second
};

// ── Number of joints ──────────────────────────────────────────
#define NUM_JOINTS 6

// ── Joint indices ─────────────────────────────────────────────
#define JOINT_WAIST        0
#define JOINT_SHOULDER     1
#define JOINT_ELBOW        2
#define JOINT_WRIST_PITCH  3
#define JOINT_WRIST_ROLL   4
#define JOINT_GRIPPER      5

// ═══════════════════════════════════════════════════════════════
// ServoManager class
// ═══════════════════════════════════════════════════════════════
class ServoManager {
public:
    ServoManager();

    /// Initialize all servos and ADC pins
    void init();

    /// Set target angle for a joint (degrees), respects rate limiting
    /// Returns the actual angle after clamping to limits
    float set_angle(uint8_t joint_id, float angle_deg);

    /// Get the last commanded angle (degrees)
    float get_angle_commanded(uint8_t joint_id) const;

    /// Get the ADC-feedback angle (degrees), or commanded if FEEDBACK_COMMANDED
    float get_angle_feedback(uint8_t joint_id) const;

    /// Read raw ADC value for a joint
    int get_adc_raw(uint8_t joint_id) const;

    /// Set per-joint speed limit (degrees/second)
    void set_speed_limit(uint8_t joint_id, float deg_per_s);

    /// Enable torque (re-attach servo PWM at current feedback position)
    void enable_torque(uint8_t joint_id);

    /// Disable torque (detach servo PWM — servo goes limp)
    void disable_torque(uint8_t joint_id);

    /// Enable/disable torque for ALL joints
    void enable_all_torque();
    void disable_all_torque();

    /// Check if torque is currently enabled for a joint
    bool is_torque_enabled(uint8_t joint_id) const;

    /// Update rate-limited servo positions. Call every loop iteration.
    /// dt_ms: time since last update in milliseconds
    void update(unsigned long dt_ms);

    /// Set feedback mode (ADC or COMMANDED)
    void set_feedback_mode(FeedbackMode mode);
    FeedbackMode get_feedback_mode() const;

    /// Get joint configuration (read-only)
    const JointConfig& get_config(uint8_t joint_id) const;

    /// Set ADC calibration values for a joint
    void set_adc_calibration(uint8_t joint_id, float adc_min, float adc_max);

    /// Move all joints to home position
    void go_home();

    /// Estimate velocity from position delta (degrees/second)
    float get_velocity(uint8_t joint_id) const;

private:
    Servo         _servos[NUM_JOINTS];
    JointConfig   _configs[NUM_JOINTS];
    FeedbackMode  _feedback_mode;

    // State tracking
    float  _commanded_deg[NUM_JOINTS];    // target angle
    float  _current_deg[NUM_JOINTS];      // rate-limited current angle
    float  _prev_feedback_deg[NUM_JOINTS];// previous feedback for velocity
    float  _velocity_dps[NUM_JOINTS];     // estimated velocity (deg/s)
    bool   _torque_enabled[NUM_JOINTS];

    // Internal helpers
    float _clamp_angle(uint8_t joint_id, float deg) const;
    float _adc_to_deg(uint8_t joint_id) const;
    uint16_t _deg_to_pulse(uint8_t joint_id, float deg) const;
};
