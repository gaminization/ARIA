// ═══════════════════════════════════════════════════════════════
// ARIA ESP32 Servo Manager — Implementation
// ═══════════════════════════════════════════════════════════════
#include "servo_manager.h"
#include <Arduino.h>

// ── Default pin assignments (from assembly manual wiring) ─────
// These match the #defines in main.cpp but are set here as defaults.
// Override via set_adc_calibration() after init.

// Default pulse widths
#define MG995_MIN_PULSE_DEFAULT  600
#define MG995_MAX_PULSE_DEFAULT  2400
#define SG90_MIN_PULSE_DEFAULT   500
#define SG90_MAX_PULSE_DEFAULT   2500

// Default speed limit (degrees per second)
#define DEFAULT_SPEED_LIMIT  120.0f

// ═══════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════
ServoManager::ServoManager()
    : _feedback_mode(FEEDBACK_ADC)
{
    // Default joint configurations
    // These will be overridden by main.cpp pin definitions
    _configs[JOINT_WAIST] = {
        "waist", 3, 34, SERVO_MG995,
        -90.0f, 90.0f, 0.0f,
        MG995_MIN_PULSE_DEFAULT, MG995_MAX_PULSE_DEFAULT,
        0.0f, 4095.0f, DEFAULT_SPEED_LIMIT
    };
    _configs[JOINT_SHOULDER] = {
        "shoulder", 5, 35, SERVO_MG995,
        0.0f, 180.0f, 90.0f,
        MG995_MIN_PULSE_DEFAULT, MG995_MAX_PULSE_DEFAULT,
        0.0f, 4095.0f, DEFAULT_SPEED_LIMIT
    };
    _configs[JOINT_ELBOW] = {
        "elbow", 6, 32, SERVO_MG995,
        0.0f, 180.0f, 90.0f,
        MG995_MIN_PULSE_DEFAULT, MG995_MAX_PULSE_DEFAULT,
        0.0f, 4095.0f, DEFAULT_SPEED_LIMIT
    };
    _configs[JOINT_WRIST_PITCH] = {
        "wrist_pitch", 10, 25, SERVO_SG90,
        0.0f, 180.0f, 90.0f,
        SG90_MIN_PULSE_DEFAULT, SG90_MAX_PULSE_DEFAULT,
        0.0f, 4095.0f, DEFAULT_SPEED_LIMIT
    };
    _configs[JOINT_WRIST_ROLL] = {
        "wrist_roll", 9, 33, SERVO_SG90,
        0.0f, 180.0f, 90.0f,
        SG90_MIN_PULSE_DEFAULT, SG90_MAX_PULSE_DEFAULT,
        0.0f, 4095.0f, DEFAULT_SPEED_LIMIT
    };
    _configs[JOINT_GRIPPER] = {
        "gripper", 11, 26, SERVO_SG90,
        0.0f, 55.0f, 20.0f,
        SG90_MIN_PULSE_DEFAULT, SG90_MAX_PULSE_DEFAULT,
        0.0f, 4095.0f, DEFAULT_SPEED_LIMIT
    };

    for (int i = 0; i < NUM_JOINTS; i++) {
        _commanded_deg[i] = _configs[i].home_deg;
        _current_deg[i] = _configs[i].home_deg;
        _prev_feedback_deg[i] = _configs[i].home_deg;
        _velocity_dps[i] = 0.0f;
        _torque_enabled[i] = true;
    }
}

// ═══════════════════════════════════════════════════════════════
// init — attach servos and configure ADC
// ═══════════════════════════════════════════════════════════════
void ServoManager::init() {
    // Configure ADC resolution (ESP32: 12-bit = 0–4095)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // Full 0–3.3V range

    for (int i = 0; i < NUM_JOINTS; i++) {
        // Attach servo with calibrated pulse range
        _servos[i].setPeriodHertz(50);  // Standard 50Hz servo PWM
        _servos[i].attach(
            _configs[i].servo_pin,
            _configs[i].min_pulse_us,
            _configs[i].max_pulse_us
        );

        // Configure ADC pin as input
        pinMode(_configs[i].adc_pin, INPUT);

        // Move to home position
        _current_deg[i] = _configs[i].home_deg;
        _commanded_deg[i] = _configs[i].home_deg;
        _servos[i].writeMicroseconds(
            _deg_to_pulse(i, _configs[i].home_deg)
        );

        _torque_enabled[i] = true;
    }

    Serial.println("[ServoManager] All 6 servos initialized at home positions.");
}

// ═══════════════════════════════════════════════════════════════
// set_angle — set target angle with clamping
// ═══════════════════════════════════════════════════════════════
float ServoManager::set_angle(uint8_t joint_id, float angle_deg) {
    if (joint_id >= NUM_JOINTS) return 0.0f;

    float clamped = _clamp_angle(joint_id, angle_deg);
    _commanded_deg[joint_id] = clamped;
    return clamped;
}

// ═══════════════════════════════════════════════════════════════
// get_angle_commanded
// ═══════════════════════════════════════════════════════════════
float ServoManager::get_angle_commanded(uint8_t joint_id) const {
    if (joint_id >= NUM_JOINTS) return 0.0f;
    return _commanded_deg[joint_id];
}

// ═══════════════════════════════════════════════════════════════
// get_angle_feedback — ADC reading or commanded fallback
// ═══════════════════════════════════════════════════════════════
float ServoManager::get_angle_feedback(uint8_t joint_id) const {
    if (joint_id >= NUM_JOINTS) return 0.0f;

    if (_feedback_mode == FEEDBACK_ADC) {
        return _adc_to_deg(joint_id);
    }
    return _commanded_deg[joint_id];
}

// ═══════════════════════════════════════════════════════════════
// get_adc_raw
// ═══════════════════════════════════════════════════════════════
int ServoManager::get_adc_raw(uint8_t joint_id) const {
    if (joint_id >= NUM_JOINTS) return 0;
    return analogRead(_configs[joint_id].adc_pin);
}

// ═══════════════════════════════════════════════════════════════
// Speed limit
// ═══════════════════════════════════════════════════════════════
void ServoManager::set_speed_limit(uint8_t joint_id, float deg_per_s) {
    if (joint_id >= NUM_JOINTS) return;
    _configs[joint_id].speed_limit = deg_per_s;
}

// ═══════════════════════════════════════════════════════════════
// Torque control
// ═══════════════════════════════════════════════════════════════
void ServoManager::enable_torque(uint8_t joint_id) {
    if (joint_id >= NUM_JOINTS) return;

    if (!_torque_enabled[joint_id]) {
        // Re-attach at CURRENT feedback position (no snap)
        float current_pos = get_angle_feedback(joint_id);
        _current_deg[joint_id] = current_pos;
        _commanded_deg[joint_id] = current_pos;

        _servos[joint_id].attach(
            _configs[joint_id].servo_pin,
            _configs[joint_id].min_pulse_us,
            _configs[joint_id].max_pulse_us
        );
        _servos[joint_id].writeMicroseconds(
            _deg_to_pulse(joint_id, current_pos)
        );
        _torque_enabled[joint_id] = true;

        Serial.printf("[ServoManager] J%d %s torque ENABLED at %.1f°\n",
                      joint_id, _configs[joint_id].name, current_pos);
    }
}

void ServoManager::disable_torque(uint8_t joint_id) {
    if (joint_id >= NUM_JOINTS) return;

    if (_torque_enabled[joint_id]) {
        _servos[joint_id].detach();
        _torque_enabled[joint_id] = false;

        Serial.printf("[ServoManager] J%d %s torque DISABLED (limp)\n",
                      joint_id, _configs[joint_id].name);
    }
}

void ServoManager::enable_all_torque() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        enable_torque(i);
    }
}

void ServoManager::disable_all_torque() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        disable_torque(i);
    }
}

bool ServoManager::is_torque_enabled(uint8_t joint_id) const {
    if (joint_id >= NUM_JOINTS) return false;
    return _torque_enabled[joint_id];
}

// ═══════════════════════════════════════════════════════════════
// update — rate-limited servo driving, call every loop cycle
// ═══════════════════════════════════════════════════════════════
void ServoManager::update(unsigned long dt_ms) {
    float dt_s = (float)dt_ms / 1000.0f;
    if (dt_s <= 0.0f) return;

    for (int i = 0; i < NUM_JOINTS; i++) {
        // Update velocity estimate from feedback
        float fb_now = get_angle_feedback(i);
        _velocity_dps[i] = (fb_now - _prev_feedback_deg[i]) / dt_s;
        _prev_feedback_deg[i] = fb_now;

        // Skip if torque disabled (teach mode — servo is limp)
        if (!_torque_enabled[i]) continue;

        // Rate-limit: move _current_deg toward _commanded_deg
        float error = _commanded_deg[i] - _current_deg[i];
        float max_step = _configs[i].speed_limit * dt_s;

        if (fabsf(error) <= max_step) {
            _current_deg[i] = _commanded_deg[i];
        } else {
            _current_deg[i] += (error > 0 ? max_step : -max_step);
        }

        // Write to servo hardware
        _servos[i].writeMicroseconds(
            _deg_to_pulse(i, _current_deg[i])
        );
    }
}

// ═══════════════════════════════════════════════════════════════
// Feedback mode
// ═══════════════════════════════════════════════════════════════
void ServoManager::set_feedback_mode(FeedbackMode mode) {
    _feedback_mode = mode;
    Serial.printf("[ServoManager] Feedback mode: %s\n",
                  mode == FEEDBACK_ADC ? "ADC" : "COMMANDED");
}

FeedbackMode ServoManager::get_feedback_mode() const {
    return _feedback_mode;
}

// ═══════════════════════════════════════════════════════════════
// Configuration access
// ═══════════════════════════════════════════════════════════════
const JointConfig& ServoManager::get_config(uint8_t joint_id) const {
    return _configs[joint_id < NUM_JOINTS ? joint_id : 0];
}

void ServoManager::set_adc_calibration(uint8_t joint_id, float adc_min, float adc_max) {
    if (joint_id >= NUM_JOINTS) return;
    _configs[joint_id].adc_min = adc_min;
    _configs[joint_id].adc_max = adc_max;
    Serial.printf("[ServoManager] J%d ADC calibration: %.0f – %.0f\n",
                  joint_id, adc_min, adc_max);
}

// ═══════════════════════════════════════════════════════════════
// go_home — move all joints to home position
// ═══════════════════════════════════════════════════════════════
void ServoManager::go_home() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        _commanded_deg[i] = _configs[i].home_deg;
    }
    Serial.println("[ServoManager] Going to home position.");
}

// ═══════════════════════════════════════════════════════════════
// get_velocity — estimated from position delta
// ═══════════════════════════════════════════════════════════════
float ServoManager::get_velocity(uint8_t joint_id) const {
    if (joint_id >= NUM_JOINTS) return 0.0f;
    return _velocity_dps[joint_id];
}

// ═══════════════════════════════════════════════════════════════
// PRIVATE: clamp angle to joint limits
// ═══════════════════════════════════════════════════════════════
float ServoManager::_clamp_angle(uint8_t joint_id, float deg) const {
    const JointConfig& cfg = _configs[joint_id];
    if (deg < cfg.min_deg) return cfg.min_deg;
    if (deg > cfg.max_deg) return cfg.max_deg;
    return deg;
}

// ═══════════════════════════════════════════════════════════════
// PRIVATE: convert ADC reading to degrees
// ═══════════════════════════════════════════════════════════════
float ServoManager::_adc_to_deg(uint8_t joint_id) const {
    int raw = analogRead(_configs[joint_id].adc_pin);
    const JointConfig& cfg = _configs[joint_id];

    float adc_range = cfg.adc_max - cfg.adc_min;
    if (adc_range < 1.0f) adc_range = 4095.0f;  // Prevent divide-by-zero

    float normalized = ((float)raw - cfg.adc_min) / adc_range;
    normalized = constrain(normalized, 0.0f, 1.0f);

    float deg_range = cfg.max_deg - cfg.min_deg;
    return cfg.min_deg + normalized * deg_range;
}

// ═══════════════════════════════════════════════════════════════
// PRIVATE: convert degrees to pulse width (microseconds)
// ═══════════════════════════════════════════════════════════════
uint16_t ServoManager::_deg_to_pulse(uint8_t joint_id, float deg) const {
    const JointConfig& cfg = _configs[joint_id];
    float deg_range = cfg.max_deg - cfg.min_deg;
    if (deg_range < 0.1f) deg_range = 180.0f;

    float normalized = (deg - cfg.min_deg) / deg_range;
    normalized = constrain(normalized, 0.0f, 1.0f);

    return (uint16_t)(cfg.min_pulse_us +
                      normalized * (cfg.max_pulse_us - cfg.min_pulse_us));
}
