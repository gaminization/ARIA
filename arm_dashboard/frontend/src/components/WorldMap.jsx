import React, { useRef, useEffect } from 'react';

// Table dimensions in meters (centered at robot base)
const TABLE_X_MIN = 0.0;
const TABLE_X_MAX = 0.60;
const TABLE_Y_MIN = -0.30;
const TABLE_Y_MAX = 0.30;
const WORKSPACE_RADIUS = 0.26;

// Canvas pixel size
const CW = 320;
const CH = 320;

// Object colors by class
const CLASS_COLORS = {
  cube: '#ef4444',
  cylinder: '#3b82f6',
  ball: '#22c55e',
  box: '#f59e0b',
  bottle: '#8b5cf6',
  cup: '#ec4899',
  pen: '#6366f1',
  screwdriver: '#14b8a6',
  paintbrush: '#f97316',
  scissors: '#dc2626',
};

export default function WorldMap({ objects, relations }) {
  const canvasRef = useRef(null);

  const toCanvas = (x, y) => {
    // Map world coords to canvas: X→right, Y→up
    const px = ((y - TABLE_Y_MIN) / (TABLE_Y_MAX - TABLE_Y_MIN)) * CW;
    const py = CH - ((x - TABLE_X_MIN) / (TABLE_X_MAX - TABLE_X_MIN)) * CH;
    return [px, py];
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    // Clear
    ctx.fillStyle = '#0f0f1a';
    ctx.fillRect(0, 0, CW, CH);

    // Table surface
    ctx.fillStyle = '#1a1a2e';
    ctx.fillRect(0, 0, CW, CH);

    // Grid
    ctx.strokeStyle = 'rgba(99, 102, 241, 0.1)';
    ctx.lineWidth = 0.5;
    for (let i = 0; i <= 6; i++) {
      const x = (i / 6) * CW;
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, CH); ctx.stroke();
      const y = (i / 6) * CH;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(CW, y); ctx.stroke();
    }

    // Workspace boundary (circle from robot base)
    const [bx, by] = toCanvas(0, 0);
    const rPx = (WORKSPACE_RADIUS / (TABLE_X_MAX - TABLE_X_MIN)) * CH;
    ctx.strokeStyle = 'rgba(99, 102, 241, 0.3)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 3]);
    ctx.beginPath();
    ctx.arc(bx, by, rPx, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);

    // Robot base
    ctx.fillStyle = '#6366f1';
    ctx.beginPath();
    ctx.arc(bx, by, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#fff';
    ctx.font = '9px monospace';
    ctx.fillText('BASE', bx - 14, by + 18);

    // Spatial relations (lines between objects)
    if (relations && objects) {
      ctx.strokeStyle = 'rgba(139, 92, 246, 0.3)';
      ctx.lineWidth = 1;
      relations.forEach(rel => {
        // Find subject and object positions
        // Relations may have subject_name/object_name or indices
      });
    }

    // Objects
    if (objects) {
      objects.forEach((obj, i) => {
        const x = obj.pos_x || (obj.last_known_pose?.pose?.position?.x ?? 0.2);
        const y = obj.pos_y || (obj.last_known_pose?.pose?.position?.y ?? 0.0);
        const [px, py] = toCanvas(x, y);
        const cls = obj.class_name || 'cube';
        const color = CLASS_COLORS[cls] || '#888';
        const name = obj.display_name || obj.name || cls;

        // Object circle
        ctx.beginPath();
        ctx.arc(px, py, 10, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.globalAlpha = 0.8;
        ctx.fill();
        ctx.globalAlpha = 1.0;
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Label
        ctx.fillStyle = '#e0e0ff';
        ctx.font = '10px monospace';
        ctx.fillText(name, px - 15, py - 14);

        // Lifecycle indicator
        const lifecycle = obj.lifecycle_state || '';
        if (lifecycle === 'LOST') {
          ctx.strokeStyle = '#ef4444';
          ctx.lineWidth = 2;
          ctx.setLineDash([3, 3]);
          ctx.beginPath();
          ctx.arc(px, py, 14, 0, Math.PI * 2);
          ctx.stroke();
          ctx.setLineDash([]);
        } else if (lifecycle === 'MOVED') {
          ctx.strokeStyle = '#f59e0b';
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.arc(px, py, 14, 0, Math.PI * 2);
          ctx.stroke();
        }
      });
    }

    // Legend
    ctx.fillStyle = '#666';
    ctx.font = '9px monospace';
    ctx.fillText('Top-down world map', 5, CH - 5);

  }, [objects, relations]);

  return (
    <div className="world-map-container">
      <h3>🗺 World Map</h3>
      <canvas
        ref={canvasRef}
        width={CW}
        height={CH}
        className="world-canvas"
      />
    </div>
  );
}
