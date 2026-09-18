import React, { useRef, useEffect } from 'react';

/**
 * WorldMap — Live 2D top-down world model visualization.
 * Shows the robotic arm workspace, discovered objects,
 * spatial relations, and lifecycle states in real time.
 */

const TABLE_SIZE = 300; // mm on each side (matches world_model_db.py: ±0.30m)
const CANVAS_W = 340;
const CANVAS_H = 280;
const SCALE = (CANVAS_W - 40) / (TABLE_SIZE * 2 / 1000); // px per meter
const CX = CANVAS_W / 2; // center x
const CY = CANVAS_H / 2; // center y

const LIFECYCLE_COLORS = {
  DETECTED: '#facc15',
  TRACKED: '#00ff88',
  LOST: '#ef4444',
  RECOVERED: '#a78bfa',
  MOVED: '#38bdf8',
  REMOVED: '#6b7280',
};

function worldToCanvas(wx, wy) {
  // wx, wy in meters; table is centered at (0.30, 0.0) in world
  // Adjust to local coords: subtract table center offset
  const lx = wx - 0.30;
  const ly = wy - 0.00;
  return {
    cx: CX + lx * SCALE,
    cy: CY - ly * SCALE,  // y-up in world → y-down in canvas
  };
}

export default function WorldMap({ objects, relations }) {
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

    // Grid
    ctx.strokeStyle = '#1e2d1e';
    ctx.lineWidth = 1;
    for (let i = 0; i <= CANVAS_W; i += 20) {
      ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, CANVAS_H); ctx.stroke();
    }
    for (let i = 0; i <= CANVAS_H; i += 20) {
      ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(CANVAS_W, i); ctx.stroke();
    }

    // Table boundary
    const tableHalfPx = 0.30 * SCALE;
    ctx.strokeStyle = '#22c55e33';
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.strokeRect(CX - tableHalfPx, CY - tableHalfPx, tableHalfPx * 2, tableHalfPx * 2);
    ctx.setLineDash([]);

    // Table label
    ctx.fillStyle = '#22c55e44';
    ctx.font = '10px monospace';
    ctx.fillText('TABLE WORKSPACE', CX - 55, CY - tableHalfPx + 14);

    // Arm base (origin)
    ctx.beginPath();
    ctx.arc(CX, CY + 0.10 * SCALE, 10, 0, Math.PI * 2);
    ctx.strokeStyle = '#60a5fa';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#60a5fa33';
    ctx.fill();
    ctx.fillStyle = '#60a5fa';
    ctx.font = 'bold 9px monospace';
    ctx.fillText('ARM', CX - 8, CY + 0.10 * SCALE + 3);

    // Spatial relation lines
    if (relations && relations.length > 0) {
      const nameToObj = {};
      (objects || []).forEach(o => { nameToObj[o.display_name || o.name] = o; });

      relations.forEach(rel => {
        const subj = objects.find(o => (o.display_name || o.name) === rel.subject);
        const obj = objects.find(o => (o.display_name || o.name) === rel.object);
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

        // Relation label
        const mx = (p1.cx + p2.cx) / 2;
        const my = (p1.cy + p2.cy) / 2;
        ctx.fillStyle = '#a78bfa';
        ctx.font = '9px monospace';
        ctx.fillText(rel.relation.replace('is_', ''), mx + 2, my - 2);
      });
    }

    // Objects
    (objects || []).forEach((obj, i) => {
      const { cx, cy } = worldToCanvas(obj.pos_x, obj.pos_y);
      const color = LIFECYCLE_COLORS[obj.lifecycle_state] || '#facc15';

      // Glow
      ctx.shadowColor = color;
      ctx.shadowBlur = 10;

      // Object dot
      ctx.beginPath();
      ctx.arc(cx, cy, 8, 0, Math.PI * 2);
      ctx.fillStyle = color + '44';
      ctx.fill();
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.shadowBlur = 0;

      // Object label
      const label = (obj.display_name || obj.name || 'obj').slice(0, 12);
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 10px monospace';
      const tw = ctx.measureText(label).width;
      ctx.fillStyle = 'rgba(0,0,0,0.7)';
      ctx.fillRect(cx - tw / 2 - 2, cy + 10, tw + 4, 13);
      ctx.fillStyle = color;
      ctx.fillText(label, cx - tw / 2, cy + 21);

      // Position
      ctx.fillStyle = '#666';
      ctx.font = '8px monospace';
      ctx.fillText(`(${obj.pos_x.toFixed(2)}, ${obj.pos_y.toFixed(2)})`, cx - 22, cy + 33);
    });

    // Legend
    const legendY = CANVAS_H - 40;
    let lx = 10;
    Object.entries(LIFECYCLE_COLORS).forEach(([state, color]) => {
      ctx.fillStyle = color;
      ctx.fillRect(lx, legendY, 8, 8);
      ctx.fillStyle = '#888';
      ctx.font = '8px monospace';
      ctx.fillText(state, lx + 10, legendY + 8);
      lx += ctx.measureText(state).width + 22;
    });

  }, [objects, relations]);

  return (
    <div className="world-map-container">
      <div className="panel-header">
        <span className="panel-icon">🌐</span>
        <span className="panel-title">World Model</span>
        <span className="panel-badge">{(objects || []).length} objects</span>
      </div>
      <canvas ref={canvasRef} className="world-canvas" />
      {(!objects || objects.length === 0) && (
        <div className="world-empty">
          <span>No objects in scene yet</span>
          <span className="world-hint">VLA will discover & track objects via gripper camera</span>
        </div>
      )}
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
