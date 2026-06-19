import React, { useState } from 'react';

const JOINT_CONFIGS = [
  { name: 'waist_joint', label: 'J1 — Waist (MG995)', min: -90, max: 90, color: '#6366f1' },
  { name: 'shoulder_joint', label: 'J2 — Shoulder (MG995)', min: 0, max: 180, color: '#8b5cf6' },
  { name: 'elbow_joint', label: 'J3 — Elbow (MG995)', min: 0, max: 180, color: '#a78bfa' },
  { name: 'wrist_pitch_joint', label: 'J4 — Wrist Pitch (SG90)', min: -90, max: 90, color: '#c4b5fd' },
  { name: 'wrist_roll_joint', label: 'J5 — Wrist Roll (SG90)', min: -90, max: 90, color: '#e0d4ff' },
];

const NAMED_POSES = {
  Home: [0, 90, 75, 0, 0],
  Ready: [0, 45, 90, 0, 0],
  Extended: [0, 60, 120, -20, 0],
  'Left View': [45, 60, 90, 0, 0],
  'Right View': [-45, 60, 90, 0, 0],
  'Look Down': [0, 90, 90, -45, 0],
  Zero: [0, 0, 0, 0, 0],
};

export default function JointSliders({ joints, onSetJoint }) {
  const [localAngles, setLocalAngles] = useState(
    joints.positions_deg || [0, 0, 0, 0, 0]
  );
  const [speed, setSpeed] = useState(30);

  const handleSlider = (index, value) => {
    const newAngles = [...localAngles];
    newAngles[index] = parseFloat(value);
    setLocalAngles(newAngles);
    onSetJoint(JOINT_CONFIGS[index].name, parseFloat(value));
  };

  const goToPose = (angles) => {
    setLocalAngles([...angles]);
    angles.forEach((deg, i) => {
      onSetJoint(JOINT_CONFIGS[i].name, deg);
    });
  };

  return (
    <div className="joints-container">
      <h2>🎮 Manual Joint Control</h2>
      <p className="joints-subtitle">
        Web-based version of joint_slider_gui.py — drag sliders to control each joint.
      </p>

      {/* Named Poses */}
      <div className="named-poses">
        <span className="section-label">Named Poses:</span>
        <div className="pose-buttons">
          {Object.entries(NAMED_POSES).map(([name, angles]) => (
            <button
              key={name}
              className="pose-btn"
              onClick={() => goToPose(angles)}
            >
              {name}
            </button>
          ))}
        </div>
      </div>

      {/* Speed Control */}
      <div className="speed-control">
        <label>Speed: {speed}°/s</label>
        <input
          type="range"
          min="5"
          max="120"
          value={speed}
          onChange={(e) => setSpeed(parseInt(e.target.value))}
          className="speed-slider"
        />
      </div>

      {/* Joint Sliders */}
      <div className="joint-sliders">
        {JOINT_CONFIGS.map((jc, i) => {
          const currentDeg = joints.positions_deg?.[i] || 0;
          const localDeg = localAngles[i] || 0;

          return (
            <div key={i} className="joint-slider-row">
              <div className="slider-header">
                <span className="slider-label" style={{ color: jc.color }}>
                  {jc.label}
                </span>
                <span className="slider-value">{localDeg.toFixed(1)}°</span>
                <span className="slider-actual">
                  (actual: {currentDeg.toFixed(1)}°)
                </span>
              </div>

              <div className="slider-body">
                <span className="slider-min">{jc.min}°</span>
                <input
                  type="range"
                  min={jc.min}
                  max={jc.max}
                  step="0.5"
                  value={localDeg}
                  onChange={(e) => handleSlider(i, e.target.value)}
                  className="joint-range"
                  style={{
                    '--track-color': jc.color,
                    '--thumb-color': jc.color,
                  }}
                />
                <span className="slider-max">{jc.max}°</span>
              </div>

              {/* Visual bar showing current vs commanded */}
              <div className="slider-track-overlay">
                <div
                  className="slider-actual-marker"
                  style={{
                    left: `${((currentDeg - jc.min) / (jc.max - jc.min)) * 100}%`,
                    backgroundColor: jc.color,
                  }}
                  title={`Actual: ${currentDeg.toFixed(1)}°`}
                />
              </div>
            </div>
          );
        })}
      </div>

      {/* Numeric Input */}
      <div className="numeric-inputs">
        <span className="section-label">Direct Input (degrees):</span>
        <div className="numeric-row">
          {JOINT_CONFIGS.map((jc, i) => (
            <div key={i} className="numeric-input-group">
              <label>J{i + 1}</label>
              <input
                type="number"
                min={jc.min}
                max={jc.max}
                step="1"
                value={localAngles[i]?.toFixed(0) || 0}
                onChange={(e) => handleSlider(i, e.target.value)}
                className="numeric-input"
              />
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
