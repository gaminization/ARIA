import React, { useRef, useEffect } from 'react';

// DH parameters from Stage 1
const DH = {
  d1: 0.070,  // base height
  a2: 0.145,  // upper arm
  a3: 0.115,  // forearm
  d5: 0.060,  // wrist to gripper
};

// Colors for each joint/link
const COLORS = ['#6366f1', '#8b5cf6', '#a78bfa', '#c4b5fd', '#e0d4ff'];

export default function ArmVisualization({ joints }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width;
    const H = canvas.height;

    // Clear
    ctx.fillStyle = '#0f0f1a';
    ctx.fillRect(0, 0, W, H);

    // Scale: 1m = 400px, origin at center-bottom
    const scale = 400;
    const ox = W / 2;
    const oy = H - 60;

    const pos = joints.positions_rad || [0, 0, 0, 0, 0];
    const [q1, q2, q3, q4, q5] = pos;

    // Side view (ignore waist rotation for 2D viz — project onto XZ plane)
    // Base
    ctx.fillStyle = '#1e1e2e';
    ctx.fillRect(ox - 30, oy - 5, 60, 10);

    // Link 1: vertical (base to shoulder)
    const x0 = ox;
    const y0 = oy;
    const x1 = x0;
    const y1 = y0 - DH.d1 * scale;

    drawLink(ctx, x0, y0, x1, y1, COLORS[0], 'J1');

    // Link 2: upper arm (shoulder)
    const angle2 = -(q2 - Math.PI / 2); // Adjust for display
    const x2 = x1 + DH.a2 * scale * Math.cos(angle2);
    const y2 = y1 - DH.a2 * scale * Math.sin(angle2);
    drawLink(ctx, x1, y1, x2, y2, COLORS[1], 'J2');

    // Link 3: forearm (elbow)
    const angle3 = angle2 + (Math.PI - q3);
    const x3 = x2 + DH.a3 * scale * Math.cos(angle3);
    const y3 = y2 - DH.a3 * scale * Math.sin(angle3);
    drawLink(ctx, x2, y2, x3, y3, COLORS[2], 'J3');

    // Link 4: wrist
    const angle4 = angle3 - q4;
    const x4 = x3 + DH.d5 * scale * Math.cos(angle4);
    const y4 = y3 - DH.d5 * scale * Math.sin(angle4);
    drawLink(ctx, x3, y3, x4, y4, COLORS[3], 'J4');

    // Gripper (small fork)
    const gLen = 15;
    const gSpread = 8;
    const gx1 = x4 + gLen * Math.cos(angle4) - gSpread * Math.sin(angle4);
    const gy1 = y4 - gLen * Math.sin(angle4) - gSpread * Math.cos(angle4);
    const gx2 = x4 + gLen * Math.cos(angle4) + gSpread * Math.sin(angle4);
    const gy2 = y4 - gLen * Math.sin(angle4) + gSpread * Math.cos(angle4);
    ctx.strokeStyle = '#22c55e';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x4, y4);
    ctx.lineTo(gx1, gy1);
    ctx.moveTo(x4, y4);
    ctx.lineTo(gx2, gy2);
    ctx.stroke();

    // Draw joint circles
    [[x0, y0], [x1, y1], [x2, y2], [x3, y3], [x4, y4]].forEach(([x, y], i) => {
      ctx.beginPath();
      ctx.arc(x, y, 6, 0, Math.PI * 2);
      ctx.fillStyle = COLORS[i];
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    });

    // Joint angle readouts
    const degs = joints.positions_deg || [0, 0, 0, 0, 0];
    const names = ['Waist', 'Shoulder', 'Elbow', 'Wrist P', 'Wrist R'];
    ctx.fillStyle = '#e0e0ff';
    ctx.font = '13px monospace';

    names.forEach((name, i) => {
      const pct = Math.abs(degs[i]) / 180;
      const barX = 15;
      const barY = 20 + i * 28;
      const barW = 110;
      const barH = 16;

      // Label
      ctx.fillStyle = '#a0a0c0';
      ctx.fillText(`${name}:`, barX, barY + 12);

      // Bar background
      ctx.fillStyle = '#1a1a2e';
      ctx.fillRect(barX + 72, barY, barW, barH);

      // Bar fill
      const grad = ctx.createLinearGradient(barX + 72, 0, barX + 72 + barW, 0);
      grad.addColorStop(0, COLORS[i]);
      grad.addColorStop(1, '#22c55e');
      ctx.fillStyle = grad;
      ctx.fillRect(barX + 72, barY, barW * pct, barH);

      // Border
      ctx.strokeStyle = '#333';
      ctx.lineWidth = 1;
      ctx.strokeRect(barX + 72, barY, barW, barH);

      // Value
      ctx.fillStyle = '#fff';
      ctx.fillText(`${degs[i]?.toFixed(1) || '0.0'}°`, barX + 190, barY + 12);
    });

    // Workspace boundary arc
    ctx.strokeStyle = 'rgba(99, 102, 241, 0.2)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.arc(ox, oy - DH.d1 * scale, (DH.a2 + DH.a3) * scale, 0, Math.PI, true);
    ctx.stroke();
    ctx.setLineDash([]);

  }, [joints]);

  return (
    <div className="arm-viz-container">
      <h3>🦾 Arm Visualization</h3>
      <canvas
        ref={canvasRef}
        width={420}
        height={360}
        className="arm-canvas"
      />
    </div>
  );
}

function drawLink(ctx, x1, y1, x2, y2, color, label) {
  // Link body
  ctx.strokeStyle = color;
  ctx.lineWidth = 8;
  ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();

  // Outline
  ctx.strokeStyle = 'rgba(255,255,255,0.15)';
  ctx.lineWidth = 10;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();

  // Redraw colored link on top
  ctx.strokeStyle = color;
  ctx.lineWidth = 6;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
}
