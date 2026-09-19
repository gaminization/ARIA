import React, { useRef, useEffect, useState } from 'react';

/**
 * CameraPanel — Gripper Camera Primary Mode
 *
 * Shows the wrist/gripper eye-in-hand camera as the ONLY primary feed.
 * YOLO detection bboxes are overlaid directly on the gripper view.
 * Top camera is deliberately omitted (gripper-only mode).
 */
export default function CameraPanel({ topFrame, wristFrame, sideFrame, detections, health }) {
  const canvasRef = useRef(null);
  const imgRef = useRef(null);
  const [fps, setFps] = useState(0);
  const [showObserver, setShowObserver] = useState(true);
  const prevFrameRef = useRef(null);
  const fpsCountRef = useRef(0);
  const fpsTimerRef = useRef(null);

  // FPS counter
  useEffect(() => {
    if (wristFrame && wristFrame !== prevFrameRef.current) {
      prevFrameRef.current = wristFrame;
      fpsCountRef.current += 1;
    }
    if (!fpsTimerRef.current) {
      fpsTimerRef.current = setInterval(() => {
        setFps(fpsCountRef.current);
        fpsCountRef.current = 0;
      }, 1000);
    }
    return () => {};
  }, [wristFrame]);

  useEffect(() => () => clearInterval(fpsTimerRef.current), []);

  // Draw detection overlay on canvas
  useEffect(() => {
    const canvas = canvasRef.current;
    const img = imgRef.current;
    if (!canvas || !img || !wristFrame) return;

    const drawOverlay = () => {
      const ctx = canvas.getContext('2d');
      canvas.width = img.naturalWidth || 640;
      canvas.height = img.naturalHeight || 480;
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      const scaleX = canvas.width / 640;
      const scaleY = canvas.height / 480;

      detections.forEach((det, i) => {
        const x = (det.bbox_x - det.bbox_w / 2) * scaleX;
        const y = (det.bbox_y - det.bbox_h / 2) * scaleY;
        const w = det.bbox_w * scaleX;
        const h = det.bbox_h * scaleY;
        const conf = (det.confidence * 100).toFixed(0);

        // Glow effect
        ctx.shadowColor = '#00ff88';
        ctx.shadowBlur = 8;

        // Bbox
        ctx.strokeStyle = '#00ff88';
        ctx.lineWidth = 2;
        ctx.strokeRect(x, y, w, h);

        // Corner accents
        const cs = 12;
        ctx.lineWidth = 3;
        [[x, y], [x + w - cs, y], [x, y + h - cs], [x + w - cs, y + h - cs]].forEach(([cx, cy], ci) => {
          ctx.beginPath();
          if (ci === 0) { ctx.moveTo(cx, cy + cs); ctx.lineTo(cx, cy); ctx.lineTo(cx + cs, cy); }
          else if (ci === 1) { ctx.moveTo(cx, cy); ctx.lineTo(cx + cs, cy); ctx.moveTo(cx + cs, cy); ctx.lineTo(cx + cs, cy + cs); }
          else if (ci === 2) { ctx.moveTo(cx, cy); ctx.lineTo(cx, cy + cs); ctx.lineTo(cx + cs, cy + cs); }
          else { ctx.moveTo(cx, cy); ctx.lineTo(cx + cs, cy); ctx.moveTo(cx + cs, cy); ctx.lineTo(cx + cs, cy + cs); }
          ctx.stroke();
        });

        ctx.shadowBlur = 0;

        // Label background
        const label = `${det.class_name} ${conf}%`;
        ctx.font = 'bold 13px monospace';
        const tw = ctx.measureText(label).width;
        ctx.fillStyle = 'rgba(0,20,10,0.85)';
        ctx.fillRect(x, y > 18 ? y - 20 : y + 2, tw + 10, 18);

        // Label text
        ctx.fillStyle = '#00ff88';
        ctx.fillText(label, x + 5, y > 18 ? y - 6 : y + 14);

        // Tracking ID badge
        if (det.tracking_id !== undefined) {
          ctx.fillStyle = 'rgba(0,120,255,0.85)';
          ctx.fillRect(x + w - 30, y + 2, 28, 16);
          ctx.fillStyle = '#fff';
          ctx.font = 'bold 11px monospace';
          ctx.fillText(`#${det.tracking_id}`, x + w - 27, y + 13);
        }
      });

      // Crosshair at center
      ctx.strokeStyle = 'rgba(255,255,100,0.5)';
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(canvas.width / 2 - 20, canvas.height / 2);
      ctx.lineTo(canvas.width / 2 + 20, canvas.height / 2);
      ctx.moveTo(canvas.width / 2, canvas.height / 2 - 20);
      ctx.lineTo(canvas.width / 2, canvas.height / 2 + 20);
      ctx.stroke();
      ctx.setLineDash([]);
    };

    if (img.complete && img.naturalWidth > 0) {
      drawOverlay();
    } else {
      img.onload = drawOverlay;
    }
  }, [wristFrame, detections]);

  const cvPipelineSteps = [
    { id: 'yolo', label: 'YOLO v8m', active: detections.length > 0 },
    { id: 'depth', label: 'Depth-Anything v2', active: !!wristFrame },
    { id: 'sam2', label: 'SAM2 Mask', active: detections.length > 0 },
    { id: 'grasp', label: 'GraspNet v2', active: detections.length > 0 },
  ];

  return (
    <div className="camera-container">
      {/* Header */}
      <div className="camera-header">
        <div className="camera-title">
          <span className="camera-icon">👁</span>
          <span>Gripper Eye-in-Hand Camera</span>
          <span className="camera-subtitle">· wrist_camera/image_raw · 640×480</span>
        </div>
        <div className="camera-badges">
          {wristFrame && (
            <span className="badge live">
              <span className="live-dot" /> LIVE
            </span>
          )}
          <span className="badge fps">{fps} fps</span>
          <span className="badge objects">{detections.length} obj</span>
        </div>
      </div>

      {/* Main Gripper Feed */}
      <div className="gripper-feed-main">
        {wristFrame ? (
          <div className="feed-overlay-wrapper">
            <img
              ref={imgRef}
              src={`data:image/jpeg;base64,${wristFrame}`}
              alt="Gripper camera"
              className="camera-img-main"
            />
            <canvas
              ref={canvasRef}
              className="detection-canvas"
            />
          </div>
        ) : (
          <div className="feed-placeholder-large">
            <div className="pulse-ring" />
            <span className="feed-icon-large">👁</span>
            <span className="feed-title">Gripper Camera Standby</span>
            <span className="feed-hint">Launch simulation — wrist_camera publishes on /wrist_camera/image_raw</span>
            <code className="feed-cmd">ros2 launch arm_bringup aria_full_u3.launch.py</code>
          </div>
        )}
      </div>

      {/* Ground-Truth Verification Cameras (Side & Top Viewports — Observer Only) */}
      <div className="observer-verification-bar">
        <div className="observer-header" onClick={() => setShowObserver(!showObserver)}>
          <span className="obs-badge">👁 Ground-Truth Verification (Observer Feeds)</span>
          <span className="obs-hint">Not used in robot decision making · Proves physical task completion</span>
          <button type="button" className="obs-toggle-btn">{showObserver ? '▼ Hide Verification Views' : '▲ Show Verification Views'}</button>
        </div>
        {showObserver && (
          <div className="observer-feeds-grid">
            <div className="obs-camera-box">
              <div className="obs-cam-title">📹 Side Camera (/side_camera/image_raw)</div>
              {sideFrame ? (
                <img src={`data:image/jpeg;base64,${sideFrame}`} alt="Side camera" className="obs-img" />
              ) : (
                <div className="obs-placeholder">Observer: Waiting for /side_camera/image_raw</div>
              )}
            </div>
            <div className="obs-camera-box">
              <div className="obs-cam-title">📡 Overhead Camera (/top_camera/image_raw)</div>
              {topFrame ? (
                <img src={`data:image/jpeg;base64,${topFrame}`} alt="Overhead camera" className="obs-img" />
              ) : (
                <div className="obs-placeholder">Observer: Waiting for /top_camera/image_raw</div>
              )}
            </div>
          </div>
        )}
      </div>

      {/* CV Pipeline Status Strip */}
      <div className="cv-pipeline-strip">
        <span className="cv-pipeline-label">CV Pipeline:</span>
        {cvPipelineSteps.map(step => (
          <div key={step.id} className={`cv-step ${step.active ? 'active' : 'inactive'}`}>
            <span className={`cv-dot ${step.active ? 'active' : ''}`} />
            {step.label}
          </div>
        ))}
        {health && (
          <span className="cv-latency">
            {health.inference_ms ? `${health.inference_ms.toFixed(0)}ms inference` : ''}
          </span>
        )}
      </div>

      {/* Object detection list */}
      {detections.length > 0 && (
        <div className="detection-list">
          {detections.map((det, i) => (
            <div key={i} className="det-chip">
              <span className="det-name">{det.class_name}</span>
              <span className="det-conf">{(det.confidence * 100).toFixed(0)}%</span>
              {det.tracking_id !== undefined && (
                <span className="det-id">#{det.tracking_id}</span>
              )}
              <span className={`det-state ${det.lifecycle_state || ''}`}>
                {det.lifecycle_state || 'DETECTED'}
              </span>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
