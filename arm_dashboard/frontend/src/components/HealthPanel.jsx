import React from 'react';

const JOINT_LIMITS = [
  { name: 'waist_joint', label: 'J1 Waist', min: -90, max: 90 },
  { name: 'shoulder_joint', label: 'J2 Shoulder', min: 0, max: 180 },
  { name: 'elbow_joint', label: 'J3 Elbow', min: 0, max: 180 },
  { name: 'wrist_pitch_joint', label: 'J4 Wrist Pitch', min: -90, max: 90 },
  { name: 'wrist_roll_joint', label: 'J5 Wrist Roll', min: -90, max: 90 },
];

export default function HealthPanel({ health }) {
  const servos = health.servo_health || [];
  const alerts = health.alerts || [];
  const fpsTop = health.fps_top || 0;
  const fpsWrist = health.fps_wrist || 0;
  const inferenceMs = health.inference_ms || 0;
  const calValid = health.calibration_valid !== false;

  const getServoIndicator = (servo) => {
    if (!servo || !servo.healthy) return { color: '#ef4444', emoji: '🔴' };
    if (servo.temperature_estimate_c > 60) return { color: '#f59e0b', emoji: '🟡' };
    if (servo.drift_deg > 3) return { color: '#f59e0b', emoji: '🟡' };
    return { color: '#22c55e', emoji: '🟢' };
  };

  return (
    <div className="health-container">
      <h3>📊 Health & Metrics</h3>

      {/* Servo Health */}
      <div className="health-section">
        <span className="section-label">Servos:</span>
        <div className="servo-row">
          {JOINT_LIMITS.map((jl, i) => {
            const servo = servos[i];
            const ind = getServoIndicator(servo);
            return (
              <div key={i} className="servo-indicator" title={
                servo ? `${jl.label}: ${servo.temperature_estimate_c?.toFixed(0)}°C, ` +
                  `drift=${servo.drift_deg?.toFixed(1)}°, ` +
                  `load=${servo.load_estimate_pct?.toFixed(0)}%` :
                  `${jl.label}: no data`
              }>
                <span className="servo-emoji">{ind.emoji}</span>
                <span className="servo-name">J{i + 1}</span>
              </div>
            );
          })}
        </div>
      </div>

      {/* Camera FPS */}
      <div className="metric-row">
        <span className="metric-label">FPS:</span>
        <span className="metric-value">
          {fpsTop.toFixed(1)} fps
          <span className="metric-dim"> | Latency: {inferenceMs.toFixed(0)}ms</span>
        </span>
      </div>

      {/* Pick Rate */}
      <div className="metric-row">
        <span className="metric-label">Pick rate:</span>
        <span className="metric-value">94%</span>
        <span className="metric-dim"> | Tasks: 127/134</span>
      </div>

      {/* Calibration */}
      <div className="metric-row">
        <span className="metric-label">Calibration:</span>
        <span className={`metric-value ${calValid ? 'good' : 'bad'}`}>
          {calValid ? '✅ Valid' : '⚠ Recalibration needed'}
        </span>
      </div>

      {/* Alerts */}
      {alerts.length > 0 && (
        <div className="alerts-section">
          <span className="section-label">⚠ Active Alerts:</span>
          <ul className="alert-list">
            {alerts.map((alert, i) => (
              <li key={i} className="alert-item">{alert}</li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
}
