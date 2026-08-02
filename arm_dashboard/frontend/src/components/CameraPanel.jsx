import React from 'react';

export default function CameraPanel({ topFrame, wristFrame, detections }) {
  return (
    <div className="camera-container">
      <h3>📷 Camera Feeds</h3>

      <div className="camera-feed">
        <label>Top Camera (1280×720)</label>
        <div className="feed-box">
          {topFrame ? (
            <img
              src={`data:image/jpeg;base64,${topFrame}`}
              alt="Top camera"
              className="camera-img"
            />
          ) : (
            <div className="feed-placeholder">
              <div className="no-feed-icon">📹</div>
              <span>No camera feed</span>
              <span className="feed-hint">Launch simulation to activate</span>
            </div>
          )}
          {/* YOLO detection overlay */}
          <svg className="detection-overlay" viewBox="0 0 1280 720">
            {detections.map((det, i) => (
              <g key={i}>
                <rect
                  x={det.bbox_x - det.bbox_w / 2}
                  y={det.bbox_y - det.bbox_h / 2}
                  width={det.bbox_w}
                  height={det.bbox_h}
                  fill="none"
                  stroke="#00ff88"
                  strokeWidth="2"
                  rx="3"
                />
                <text
                  x={det.bbox_x - det.bbox_w / 2 + 4}
                  y={det.bbox_y - det.bbox_h / 2 - 6}
                  fill="#00ff88"
                  fontSize="14"
                  fontFamily="monospace"
                >
                  {det.class_name} {(det.confidence * 100).toFixed(0)}%
                </text>
              </g>
            ))}
          </svg>
        </div>
      </div>

      <div className="camera-feed">
        <label>Wrist Camera (640×480)</label>
        <div className="feed-box small">
          {wristFrame ? (
            <img
              src={`data:image/jpeg;base64,${wristFrame}`}
              alt="Wrist camera"
              className="camera-img"
            />
          ) : (
            <div className="feed-placeholder small">
              <span>No wrist feed</span>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
