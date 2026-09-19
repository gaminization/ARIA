import React, { useRef, useEffect } from 'react';

/**
 * WorldMap — Live 2D top-down world model visualization.
 * Shows the robotic arm workspace, discovered objects,
 * spatial relations, and lifecycle states in real time.
 */

const CANVAS_W = 340;
const CANVAS_H = 260;
const SCALE = 420; // px per meter
const CX = CANVAS_W / 2; // robot arm waist origin X (170)
const CY = CANVAS_H - 45; // robot arm waist origin Y (215)

const LIFECYCLE_COLORS = {
  DETECTED: '#facc15',
  TRACKED: '#00ff88',
  LOST: '#ef4444',
  RECOVERED: '#a78bfa',
  MOVED: '#38bdf8',
  REMOVED: '#6b7280',
};

const OBJECT_ICONS = {
  banana: '🍌',
  mug: '☕',
  cup: '☕',
  duck: '🦆',
  bottle: '🍾',
  plate: '🍽',
  orange: '🍊',
  can: '🥫',
  bowl: '🥣',
  box: '📦',
};

function worldToCanvas(wx, wy) {
  // Top-down view:
  // Robot base is at (0, 0).
  // World +X (forward) -> Canvas UP (-Y)
  // World +Y (left) -> Canvas LEFT (-X)
  // World -Y (right) -> Canvas RIGHT (+X)
  return {
    cx: CX - wy * SCALE,
    cy: CY - wx * SCALE,
  };
}

export default function WorldMap({ objects, relations, onSelectObject }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    canvas.width = CANVAS_W;
    canvas.height = CANVAS_H;

    // Background
    ctx.fillStyle = '#0d1117';
    ctx.fillRect(0, 0, CANVAS_W, CANVAS_H);

    // Coordinate Grid
    ctx.strokeStyle = '#1e2d1e';
    ctx.lineWidth = 1;
    for (let i = 0; i <= CANVAS_W; i += 20) {
      ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, CANVAS_H); ctx.stroke();
    }
    for (let i = 0; i <= CANVAS_H; i += 20) {
      ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(CANVAS_W, i); ctx.stroke();
    }

    // Table boundary (optical table: X in [0.02, 0.42], Y in [-0.28, +0.28])
    const tl = worldToCanvas(0.42, 0.28);
    const br = worldToCanvas(0.02, -0.28);
    const tableW = br.cx - tl.cx;
    const tableH = br.cy - tl.cy;

    ctx.strokeStyle = '#22c55e44';
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 3]);
    ctx.strokeRect(tl.cx, tl.cy, tableW, tableH);
    ctx.setLineDash([]);

    ctx.fillStyle = '#22c55e44';
    ctx.font = '10px monospace';
    ctx.fillText('OPTICAL TABLE WORKSPACE', tl.cx + 20, tl.cy + 14);

    // Range distance rings from arm base
    [0.15, 0.25, 0.35].forEach((r) => {
      ctx.beginPath();
      ctx.arc(CX, CY, r * SCALE, Math.PI, 0, false);
      ctx.strokeStyle = '#3b82f622';
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.fillStyle = '#3b82f644';
      ctx.font = '8px monospace';
      ctx.fillText(`${(r * 100).toFixed(0)}cm`, CX + r * SCALE - 22, CY - 3);
    });

    // Robot Arm Base (origin 0,0)
    ctx.beginPath();
    ctx.arc(CX, CY, 12, 0, Math.PI * 2);
    ctx.fillStyle = '#1e3a8a';
    ctx.fill();
    ctx.strokeStyle = '#60a5fa';
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.fillStyle = '#93c5fd';
    ctx.font = 'bold 9px monospace';
    ctx.fillText('BASE', CX - 10, CY + 3);

    // Spatial relation lines
    if (relations && relations.length > 0) {
      relations.forEach((rel) => {
        const subj = (objects || []).find((o) => (o.display_name || o.name) === rel.subject);
        const obj = (objects || []).find((o) => (o.display_name || o.name) === rel.object);
        if (!subj || !obj || rel.object === 'table') return;

        const p1 = worldToCanvas(subj.pos_x, subj.pos_y);
        const p2 = worldToCanvas(obj.pos_x, obj.pos_y);

        ctx.beginPath();
        ctx.moveTo(p1.cx, p1.cy);
        ctx.lineTo(p2.cx, p2.cy);
        ctx.strokeStyle = 'rgba(139,92,246,0.4)';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.stroke();
        ctx.setLineDash([]);

        const mx = (p1.cx + p2.cx) / 2;
        const my = (p1.cy + p2.cy) / 2;
        ctx.fillStyle = '#a78bfa';
        ctx.font = '9px monospace';
        ctx.fillText(rel.relation.replace('is_', ''), mx + 2, my - 2);
      });
    }

    // Discovered Objects on Canvas
    (objects || []).forEach((obj) => {
      const { cx, cy } = worldToCanvas(obj.pos_x, obj.pos_y);
      const color = LIFECYCLE_COLORS[obj.lifecycle_state] || '#facc15';

      // Glow effect
      ctx.shadowColor = color;
      ctx.shadowBlur = 8;

      // Object circle
      ctx.beginPath();
      ctx.arc(cx, cy, 9, 0, Math.PI * 2);
      ctx.fillStyle = color + '44';
      ctx.fill();
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.shadowBlur = 0;

      // Object Icon / Label
      const icon = OBJECT_ICONS[obj.class_name?.toLowerCase()] || '📍';
      ctx.font = '11px sans-serif';
      ctx.fillText(icon, cx - 6, cy + 4);

      // Label card
      const label = (obj.display_name || obj.name || 'obj').slice(0, 14);
      ctx.font = 'bold 9px monospace';
      const tw = ctx.measureText(label).width;
      ctx.fillStyle = 'rgba(0,0,0,0.8)';
      ctx.fillRect(cx - tw / 2 - 3, cy - 22, tw + 6, 12);
      ctx.fillStyle = color;
      ctx.fillText(label, cx - tw / 2, cy - 13);

      // Coordinates
      ctx.fillStyle = '#aaa';
      ctx.font = '8px monospace';
      ctx.fillText(`(${obj.pos_x.toFixed(2)}, ${obj.pos_y.toFixed(2)})`, cx - 22, cy + 18);
    });

  }, [objects, relations]);

  return (
    <div className="world-map-container">
      <div className="panel-header">
        <span className="panel-icon">🌐</span>
        <span className="panel-title">World Model & Scene Graph</span>
        <span className="panel-badge">{(objects || []).length} objects</span>
      </div>

      <div className="world-canvas-wrapper">
        <canvas ref={canvasRef} className="world-canvas" />
      </div>

      {/* Discovered Objects Card List */}
      <div className="world-objects-panel">
        <div className="wop-header">
          <span>Active Table Objects (Eye-in-Hand Discovered)</span>
        </div>
        {(!objects || objects.length === 0) ? (
          <div className="world-empty-notice">
            <span>No objects registered yet</span>
            <span className="world-sub">Run active table scan or pick command to discover objects</span>
          </div>
        ) : (
          <div className="world-objects-grid">
            {objects.map((obj, i) => {
              const icon = OBJECT_ICONS[obj.class_name?.toLowerCase()] || '📍';
              const stateColor = LIFECYCLE_COLORS[obj.lifecycle_state] || '#facc15';
              return (
                <div key={i} className="world-object-card">
                  <div className="woc-top">
                    <span className="woc-icon">{icon}</span>
                    <span className="woc-name">{obj.display_name || obj.name}</span>
                    <span className="woc-badge" style={{ color: stateColor, borderColor: stateColor }}>
                      {obj.lifecycle_state || 'DETECTED'}
                    </span>
                  </div>
                  <div className="woc-coords">
                    <span>X: {obj.pos_x.toFixed(3)}m</span>
                    <span>Y: {obj.pos_y.toFixed(3)}m</span>
                    <span>Z: {obj.pos_z.toFixed(3)}m</span>
                  </div>
                  {onSelectObject && (
                    <button
                      type="button"
                      className="woc-target-btn"
                      onClick={() => onSelectObject(obj)}
                    >
                      🎯 Target Object
                    </button>
                  )}
                </div>
              );
            })}
          </div>
        )}
      </div>

      {relations && relations.length > 0 && (
        <div className="relations-list">
          {relations.slice(0, 4).map((r, i) => (
            <div key={i} className="relation-chip">
              <span className="rel-subj">{r.subject}</span>
              <span className="rel-verb">{r.relation}</span>
              <span className="rel-obj">{r.object}</span>
              <span className="rel-conf">{(r.confidence * 100).toFixed(0)}%</span>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
