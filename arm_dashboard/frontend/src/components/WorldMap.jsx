import React, { useRef, useEffect, useState, useMemo } from 'react';
import { Map, Layers, Target, Compass, Play, Crosshair, RefreshCw, Trash2, CheckCircle2 } from 'lucide-react';

export default function WorldMap({ memoryState, visionState, jointState }) {
  const canvasRef = useRef(null);
  const containerRef = useRef(null);
  const [dimensions, setDimensions] = useState({ width: 400, height: 280 });
  const [selectedObj, setSelectedObj] = useState(null);
  const [isDispatching, setIsDispatching] = useState(false);
  const [dispatchStatus, setDispatchStatus] = useState(null);
  const [hoveredObj, setHoveredObj] = useState(null);
  const [isResetting, setIsResetting] = useState(false);

  // Measure and track container size dynamically (crisp at any resolution / fullscreen)
  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const ro = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect;
        if (width > 40 && height > 40) {
          setDimensions({
            width: Math.floor(width),
            height: Math.floor(height),
          });
        }
      }
    });
    ro.observe(container);
    return () => ro.disconnect();
  }, []);

  // Preserve all valid objects from memory state
  const rawObjects = memoryState?.known_objects || [];

  // Group multiple instances and assign clean display labels (e.g. Mug #1, Mug #2)
  const displayObjects = useMemo(() => {
    const classCounters = {};
    return rawObjects
      .filter((obj) => {
        const x = obj.pos_x ?? 0;
        const y = obj.pos_y ?? 0;
        const z = obj.pos_z ?? 0.61;
        // Workspace gating bounds
        return x >= -0.35 && x <= 0.40 && y >= -0.40 && y <= 0.40 && z >= 0.50;
      })
      .slice()
      .sort((a, b) => (a.id ?? 0) - (b.id ?? 0))
      .map((obj) => {
        const cls = (obj.class_name || obj.name || 'workpiece').toLowerCase().replace(/_\d+$/, '');
        classCounters[cls] = (classCounters[cls] || 0) + 1;
        const instanceNum = classCounters[cls];
        return {
          ...obj,
          cleanClass: cls,
          instanceNum,
          displayName: `${cls.charAt(0).toUpperCase() + cls.slice(1)} #${instanceNum}`,
          dist: Math.hypot(obj.pos_x ?? 0, obj.pos_y ?? 0),
        };
      });
  }, [rawObjects]);

  // Current robot arm base heading (joint 1 = waist)
  const waistDeg = jointState?.current_angles?.[0] ?? jointState?.positions_deg?.[0] ?? 0.0;
  const waistRad = (waistDeg * Math.PI) / 180.0;

  // Handle canvas click to select object (operates in CSS pixel space)
  const handleCanvasClick = (e) => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;

    const width = rect.width;
    const height = rect.height;
    const scale = Math.min(width, height) / 0.85;
    const originX = width / 2;
    const originY = height / 2;

    const toCanvasX = (wx) => originX + wx * scale;
    const toCanvasY = (wy) => originY - wy * scale;

    let clicked = null;
    for (const obj of displayObjects) {
      const cx = toCanvasX(obj.pos_x);
      const cy = toCanvasY(obj.pos_y);
      const dist = Math.hypot(clickX - cx, clickY - cy);
      if (dist < 26) {
        clicked = obj;
        break;
      }
    }
    setSelectedObj(clicked);
    setDispatchStatus(null);
  };

  // Handle canvas hover (operates in CSS pixel space)
  const handleCanvasMouseMove = (e) => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;

    const width = rect.width;
    const height = rect.height;
    const scale = Math.min(width, height) / 0.85;
    const originX = width / 2;
    const originY = height / 2;

    const toCanvasX = (wx) => originX + wx * scale;
    const toCanvasY = (wy) => originY - wy * scale;

    let hovered = null;
    for (const obj of displayObjects) {
      const cx = toCanvasX(obj.pos_x);
      const cy = toCanvasY(obj.pos_y);
      if (Math.hypot(mx - cx, my - cy) < 22) {
        hovered = obj;
        break;
      }
    }
    setHoveredObj(hovered);
  };

  // Dispatch pick command for selected object
  const handlePickTarget = async (actionType = 'pick') => {
    if (!selectedObj) return;
    setIsDispatching(true);
    try {
      const res = await fetch('/api/target_object', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ object_id: selectedObj.id, action: actionType }),
      });
      const data = await res.json();
      setDispatchStatus(data.success ? `✓ Dispatched ${actionType}` : `✗ ${data.message}`);
    } catch (err) {
      setDispatchStatus('✗ Connection error');
    } finally {
      setIsDispatching(false);
    }
  };

  // Reset World Model database
  const handleResetWorldModel = async () => {
    setIsResetting(true);
    try {
      await fetch('/api/reset_world_model', { method: 'POST' });
      setSelectedObj(null);
      setDispatchStatus('World Model Reset');
      setTimeout(() => setDispatchStatus(null), 3000);
    } catch (err) {
      console.error(err);
    } finally {
      setIsResetting(false);
    }
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const dpr = Math.min(window.devicePixelRatio || 1, 2);

    const width = dimensions.width;
    const height = dimensions.height;

    // Buffer dimensions at high-DPI
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);

    ctx.save();
    ctx.scale(dpr, dpr);

    // Background: Dark CAD Blueprint
    ctx.fillStyle = '#080a0f';
    ctx.fillRect(0, 0, width, height);

    // Coordinate mapping centered at robot arm base
    const scale = Math.min(width, height) / 0.85;
    const originX = width / 2;
    const originY = height / 2;

    const toCanvasX = (wx) => originX + wx * scale;
    const toCanvasY = (wy) => originY - wy * scale;

    // 1. Millimeter Precision Grid (Minor 2cm, Major 10cm)
    const step2cm = 0.02 * scale;
    const step10cm = 0.10 * scale;

    // Minor grid
    ctx.strokeStyle = 'rgba(30, 41, 59, 0.4)';
    ctx.lineWidth = 0.5;
    for (let x = originX % step2cm; x < width; x += step2cm) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke();
    }
    for (let y = originY % step2cm; y < height; y += step2cm) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
    }

    // Major grid
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.12)';
    ctx.lineWidth = 1;
    for (let x = originX % step10cm; x < width; x += step10cm) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke();
    }
    for (let y = originY % step10cm; y < height; y += step10cm) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
    }

    // 2. Optical Table & Workcell Geometry
    // Optical Table Surface (+X region: 0.0 to 0.35m, Y: -0.32 to +0.32m)
    const tableLeft = toCanvasX(0.01);
    const tableTop = toCanvasY(0.32);
    const tableW = toCanvasX(0.35) - tableLeft;
    const tableH = toCanvasY(-0.32) - tableTop;

    ctx.fillStyle = 'rgba(15, 23, 42, 0.65)';
    ctx.fillRect(tableLeft, tableTop, tableW, tableH);
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(tableLeft, tableTop, tableW, tableH);

    // Left Pedestal & Bin Zone
    const leftPedX = toCanvasX(-0.25);
    const leftPedY = toCanvasY(0.26);
    const leftPedW = toCanvasX(-0.10) - leftPedX;
    const leftPedH = toCanvasY(0.10) - leftPedY;
    ctx.fillStyle = 'rgba(30, 27, 46, 0.5)';
    ctx.fillRect(leftPedX, leftPedY, leftPedW, leftPedH);
    ctx.strokeStyle = 'rgba(168, 85, 247, 0.35)';
    ctx.strokeRect(leftPedX, leftPedY, leftPedW, leftPedH);

    // Right Pedestal & Bin Zone
    const rightPedX = toCanvasX(-0.25);
    const rightPedY = toCanvasY(-0.10);
    const rightPedW = toCanvasX(-0.10) - rightPedX;
    const rightPedH = toCanvasY(-0.26) - rightPedY;
    ctx.fillStyle = 'rgba(20, 35, 30, 0.5)';
    ctx.fillRect(rightPedX, rightPedY, rightPedW, rightPedH);
    ctx.strokeStyle = 'rgba(16, 185, 129, 0.35)';
    ctx.strokeRect(rightPedX, rightPedY, rightPedW, rightPedH);

    // Rear Jenga & Sorting Tray
    const rearX = toCanvasX(-0.28);
    const rearY = toCanvasY(0.08);
    const rearW = toCanvasX(-0.14) - rearX;
    const rearH = toCanvasY(-0.08) - rearY;
    ctx.fillStyle = 'rgba(35, 30, 20, 0.5)';
    ctx.fillRect(rearX, rearY, rearW, rearH);
    ctx.strokeStyle = 'rgba(245, 158, 11, 0.35)';
    ctx.strokeRect(rearX, rearY, rearW, rearH);

    // Workcell zone annotations
    ctx.fillStyle = '#475569';
    ctx.font = '7.5px monospace';
    ctx.fillText('OPTICAL TABLE WORKSPACE (+X)', tableLeft + 6, tableTop + 11);
    ctx.fillText('PEDESTAL L', leftPedX + 4, leftPedY + 10);
    ctx.fillText('PEDESTAL R', rightPedX + 4, rightPedY + 10);
    ctx.fillText('REAR TRAY', rearX + 4, rearY + 10);

    // 3. Radar Distance Range Rings (0.10m, 0.20m, 0.30m, 0.38m limit)
    const rings = [
      { r: 0.10, label: '0.1m CORE' },
      { r: 0.20, label: '0.2m REACH' },
      { r: 0.30, label: '0.3m EXTENDED' },
      { r: 0.38, label: '0.38m LIMIT' },
    ];

    rings.forEach(({ r, label }) => {
      const rPx = r * scale;
      ctx.beginPath();
      ctx.arc(originX, originY, rPx, 0, Math.PI * 2);
      ctx.strokeStyle = r === 0.38 ? 'rgba(239, 68, 68, 0.4)' : 'rgba(56, 189, 248, 0.2)';
      ctx.setLineDash(r === 0.38 ? [4, 4] : [2, 4]);
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.fillStyle = 'rgba(100, 116, 139, 0.65)';
      ctx.font = '7px monospace';
      ctx.fillText(label, originX + rPx - 26, originY - 3);
    });

    // 4. Sector Crosshair Axes
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.25)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(originX - 0.40 * scale, originY);
    ctx.lineTo(originX + 0.40 * scale, originY);
    ctx.moveTo(originX, originY - 0.40 * scale);
    ctx.lineTo(originX, originY + 0.40 * scale);
    ctx.stroke();

    // 5. Robot Arm Base & Live Heading Pointer (Joint 1 waist)
    // Base CNC mounting plate
    ctx.beginPath();
    ctx.arc(originX, originY, 12, 0, Math.PI * 2);
    ctx.fillStyle = '#1e293b';
    ctx.fill();
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2;
    ctx.stroke();

    // Base bolt holes
    for (let i = 0; i < 4; i++) {
      const ang = (i * Math.PI) / 2 + Math.PI / 4;
      ctx.beginPath();
      ctx.arc(originX + Math.cos(ang) * 7.5, originY + Math.sin(ang) * 7.5, 1.2, 0, Math.PI * 2);
      ctx.fillStyle = '#94a3b8';
      ctx.fill();
    }

    // Live Waist Heading Pointer Needle
    const needleLen = 0.22 * scale;
    const pointerX = originX + Math.cos(waistRad) * needleLen;
    const pointerY = originY - Math.sin(waistRad) * needleLen;

    ctx.beginPath();
    ctx.moveTo(originX, originY);
    ctx.lineTo(pointerX, pointerY);
    ctx.strokeStyle = '#06b6d4';
    ctx.lineWidth = 2;
    ctx.stroke();

    // Needle arrow tip
    const tipSize = 6;
    ctx.beginPath();
    ctx.moveTo(pointerX, pointerY);
    ctx.lineTo(
      pointerX - tipSize * Math.cos(waistRad - Math.PI / 6),
      pointerY + tipSize * Math.sin(waistRad - Math.PI / 6)
    );
    ctx.lineTo(
      pointerX - tipSize * Math.cos(waistRad + Math.PI / 6),
      pointerY + tipSize * Math.sin(waistRad + Math.PI / 6)
    );
    ctx.closePath();
    ctx.fillStyle = '#06b6d4';
    ctx.fill();

    // 6. Draw Realistic Workpiece Vector CAD Footprints
    displayObjects.forEach((obj, idx) => {
      const cx = toCanvasX(obj.pos_x);
      const cy = toCanvasY(obj.pos_y);

      const isSelected = selectedObj && selectedObj.id === obj.id;
      const isHovered = hoveredObj && hoveredObj.id === obj.id;
      const cls = obj.cleanClass;

      // Color scheme
      let strokeColor = '#38bdf8';
      let fillColor = 'rgba(56, 189, 248, 0.25)';

      if (cls.includes('banana')) {
        strokeColor = '#facc15';
        fillColor = 'rgba(250, 204, 21, 0.35)';
      } else if (cls.includes('duck')) {
        strokeColor = '#fbbf24';
        fillColor = 'rgba(251, 191, 36, 0.35)';
      } else if (cls.includes('mug')) {
        if (obj.instanceNum === 2 || obj.displayName.includes('2')) {
          strokeColor = '#06b6d4'; // Cyan mug
          fillColor = 'rgba(6, 182, 212, 0.35)';
        } else {
          strokeColor = '#f87171'; // Red mug
          fillColor = 'rgba(248, 113, 113, 0.35)';
        }
      } else if (cls.includes('bottle')) {
        strokeColor = '#38bdf8';
        fillColor = 'rgba(56, 189, 248, 0.35)';
      } else if (cls.includes('orange')) {
        strokeColor = '#fb923c';
        fillColor = 'rgba(251, 146, 60, 0.35)';
      } else if (cls.includes('block') || cls.includes('jenga')) {
        strokeColor = '#f59e0b';
        fillColor = 'rgba(245, 158, 11, 0.35)';
      } else if (cls.includes('plate') || cls.includes('pan')) {
        strokeColor = '#94a3b8';
        fillColor = 'rgba(148, 163, 184, 0.30)';
      }

      ctx.save();
      ctx.translate(cx, cy);

      // Selection Glow / Crosshairs
      if (isSelected || isHovered) {
        ctx.beginPath();
        ctx.arc(0, 0, 16, 0, Math.PI * 2);
        ctx.strokeStyle = isSelected ? '#00f0ff' : 'rgba(56, 189, 248, 0.6)';
        ctx.lineWidth = isSelected ? 2 : 1.5;
        ctx.setLineDash([3, 3]);
        ctx.stroke();
        ctx.setLineDash([]);

        // Reticle ticks
        ctx.beginPath();
        ctx.moveTo(-20, 0); ctx.lineTo(-12, 0);
        ctx.moveTo(12, 0); ctx.lineTo(20, 0);
        ctx.moveTo(0, -20); ctx.lineTo(0, -12);
        ctx.moveTo(0, 12); ctx.lineTo(0, 20);
        ctx.strokeStyle = isSelected ? '#00f0ff' : '#38bdf8';
        ctx.lineWidth = 1.5;
        ctx.stroke();
      }

      // Render realistic geometry based on workpiece type
      ctx.strokeStyle = strokeColor;
      ctx.fillStyle = fillColor;
      ctx.lineWidth = 1.8;

      if (cls.includes('banana')) {
        // Curved Banana Arc
        ctx.beginPath();
        ctx.arc(0, 0, 9, -0.4, Math.PI * 0.9, false);
        ctx.arc(0, 2, 7, Math.PI * 0.9, -0.4, true);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
      } else if (cls.includes('mug')) {
        // Mug Body Circle with Handle Arc
        ctx.beginPath();
        ctx.arc(0, 0, 7.5, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        // Handle
        ctx.beginPath();
        ctx.arc(7.5, 0, 3.5, -Math.PI / 2, Math.PI / 2);
        ctx.stroke();
      } else if (cls.includes('bottle')) {
        // Cylindrical Bottle with Cap Ring
        ctx.beginPath();
        ctx.arc(0, 0, 8, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(0, 0, 3.5, 0, Math.PI * 2);
        ctx.fillStyle = strokeColor;
        ctx.fill();
      } else if (cls.includes('duck')) {
        // Duck Body with Beak Triangle
        ctx.beginPath();
        ctx.arc(-1, 0, 7.5, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        // Beak
        ctx.beginPath();
        ctx.moveTo(6.5, -2); ctx.lineTo(11, 0); ctx.lineTo(6.5, 2);
        ctx.fillStyle = '#ea580c';
        ctx.fill();
      } else if (cls.includes('block') || cls.includes('jenga')) {
        // Rectangular Precision Jenga Block (75mm x 25mm scaled)
        const bw = 16;
        const bh = 7;
        ctx.fillRect(-bw / 2, -bh / 2, bw, bh);
        ctx.strokeRect(-bw / 2, -bh / 2, bw, bh);
      } else if (cls.includes('plate') || cls.includes('pan')) {
        // Concentric Plate Rim
        ctx.beginPath();
        ctx.arc(0, 0, 11, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(0, 0, 7.5, 0, Math.PI * 2);
        ctx.stroke();
      } else {
        // Default Workpiece: Hexagonal Nut / Part
        ctx.beginPath();
        for (let i = 0; i < 6; i++) {
          const a = (i * Math.PI) / 3;
          const px = Math.cos(a) * 7.5;
          const py = Math.sin(a) * 7.5;
          if (i === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        }
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
      }

      ctx.restore();

      // Clean CAD Label Badge with anti-collision offset & dark backing
      const labelY = cy < originY ? cy - (14 + (idx % 2) * 8) : cy + (16 + (idx % 2) * 8);
      const labelText = obj.displayName.toUpperCase();
      ctx.font = `${isSelected ? 'bold ' : ''}8.5px monospace`;
      const tw = ctx.measureText(labelText).width;

      ctx.fillStyle = 'rgba(8, 11, 18, 0.85)';
      ctx.fillRect(cx - tw / 2 - 3, labelY - 8, tw + 6, 11);
      ctx.strokeStyle = isSelected ? '#38bdf8' : 'rgba(56, 189, 248, 0.25)';
      ctx.lineWidth = 0.8;
      ctx.strokeRect(cx - tw / 2 - 3, labelY - 8, tw + 6, 11);

      ctx.fillStyle = isSelected ? '#38bdf8' : '#e2e8f0';
      ctx.textAlign = 'center';
      ctx.fillText(labelText, cx, labelY);

      // Coordinate metric text
      ctx.fillStyle = '#64748b';
      ctx.font = '7px monospace';
      ctx.fillText(`(${Math.round(obj.pos_x * 1000)}, ${Math.round(obj.pos_y * 1000)})`, cx, labelY + (cy < originY ? -9 : 10));
      ctx.textAlign = 'start';
    });

    if (displayObjects.length === 0) {
      ctx.fillStyle = '#64748b';
      ctx.font = '10px monospace';
      ctx.textAlign = 'center';
      ctx.fillText('Scanning workcell environment...', originX, originY - 35);
      ctx.textAlign = 'start';
    }

    ctx.restore();
  }, [displayObjects, selectedObj, hoveredObj, waistRad, dimensions]);

  return (
    <div className="panel-card h-full flex flex-col bg-[#0b0e14] border border-[#1e293b] rounded-md overflow-hidden select-none relative">
      {/* Header Toolbar */}
      <div className="h-8 px-3 border-b border-[#1e293b] bg-[#0d121c] flex items-center justify-between">
        <div className="flex items-center gap-2">
          <Crosshair className="w-3.5 h-3.5 text-cyan-400" />
          <span className="font-mono text-xs font-bold text-slate-200">WORLD MODEL 2D CAD TWIN</span>
          <span className="px-1.5 py-0.2 bg-cyan-950 text-cyan-400 border border-cyan-800 rounded text-[9px] font-mono">
            {displayObjects.length} WORKPIECES
          </span>
        </div>

        {/* Quick Actions */}
        <div className="flex items-center gap-1.5">
          <button
            onClick={handleResetWorldModel}
            disabled={isResetting}
            title="Clear stale workpieces and re-map"
            className="px-1.5 py-0.5 bg-[#182030] hover:bg-rose-950 text-slate-400 hover:text-rose-300 border border-slate-700/50 rounded text-[9px] font-mono flex items-center gap-1 transition-colors cursor-pointer"
          >
            <Trash2 className="w-2.5 h-2.5" />
            RESET
          </button>
        </div>
      </div>

      {/* Interactive Workpiece Quick-Select Pill Strip */}
      <div className="h-7 px-2 border-b border-[#1e293b] bg-[#090d14] flex items-center gap-1 overflow-x-auto">
        {displayObjects.map((obj) => (
          <button
            key={obj.id}
            onClick={() => { setSelectedObj(obj); setDispatchStatus(null); }}
            className={`px-2 py-0.5 rounded text-[9px] font-mono whitespace-nowrap transition-colors cursor-pointer flex items-center gap-1 ${
              selectedObj && selectedObj.id === obj.id
                ? 'bg-cyan-950 border border-cyan-400 text-cyan-200 font-bold shadow-sm'
                : 'bg-[#131926] border border-[#1e293b] text-slate-400 hover:text-slate-200 hover:border-slate-600'
            }`}
          >
            <span className="w-1.5 h-1.5 rounded-full bg-cyan-400" />
            {obj.displayName}
          </button>
        ))}
        {displayObjects.length === 0 && (
          <span className="text-slate-600 text-[9px] font-mono">NO WORKPIECES MAPPED</span>
        )}
      </div>

      {/* CAD Canvas Viewport */}
      <div ref={containerRef} className="flex-1 min-h-0 bg-[#080a0f] relative p-1 flex items-center justify-center overflow-hidden">
        <canvas
          ref={canvasRef}
          style={{ width: `${dimensions.width}px`, height: `${dimensions.height}px` }}
          onClick={handleCanvasClick}
          onMouseMove={handleCanvasMouseMove}
          onMouseLeave={() => setHoveredObj(null)}
          className="cursor-crosshair block"
        />

        {/* Selected Workpiece Floating Telemetry Card */}
        {selectedObj && (
          <div className="absolute bottom-2 left-2 right-2 bg-[#090e17]/95 backdrop-blur border border-cyan-500/70 rounded-md p-2 text-xs font-mono shadow-2xl flex items-center justify-between z-20 animate-in fade-in slide-in-from-bottom-2">
            <div className="flex flex-col gap-0.5">
              <div className="flex items-center gap-2">
                <Crosshair className="w-3.5 h-3.5 text-cyan-400" />
                <span className="font-bold text-white uppercase">{selectedObj.displayName}</span>
                <span className="text-[9px] text-cyan-300 bg-cyan-950 px-1 py-0.2 rounded border border-cyan-800">
                  ID: #{selectedObj.id}
                </span>
                <span className="text-[9px] text-emerald-400 bg-emerald-950/70 px-1 py-0.2 rounded border border-emerald-800">
                  {selectedObj.dist < 0.38 ? 'IN REACH' : 'MARGINAL'}
                </span>
              </div>
              <div className="text-[10px] text-slate-300 flex gap-3 mt-0.5">
                <span>X: <strong className="text-cyan-400">{selectedObj.pos_x.toFixed(3)}m</strong></span>
                <span>Y: <strong className="text-cyan-400">{selectedObj.pos_y.toFixed(3)}m</strong></span>
                <span>R: <strong className="text-slate-200">{(selectedObj.dist * 100).toFixed(1)}cm</strong></span>
              </div>
            </div>

            <div className="flex items-center gap-1.5">
              {dispatchStatus && (
                <span className="text-[10px] font-bold text-cyan-300 mr-1">{dispatchStatus}</span>
              )}
              <button
                onClick={() => handlePickTarget('inspect')}
                disabled={isDispatching}
                className="px-2 py-1 bg-[#1a2333] hover:bg-[#25324a] text-slate-300 text-[10px] rounded border border-slate-700 transition-colors cursor-pointer"
              >
                INSPECT
              </button>
              <button
                onClick={() => handlePickTarget('pick')}
                disabled={isDispatching}
                className="px-2.5 py-1 bg-gradient-to-r from-cyan-600 to-blue-600 hover:from-cyan-500 hover:to-blue-500 text-white font-bold text-[10px] rounded shadow-sm flex items-center gap-1 transition-all cursor-pointer"
              >
                <Play className="w-2.5 h-2.5 fill-current" />
                {isDispatching ? 'DISPATCHING...' : 'PICK'}
              </button>
              <button
                onClick={() => setSelectedObj(null)}
                className="text-slate-400 hover:text-white text-xs px-1.5 cursor-pointer"
              >
                ✕
              </button>
            </div>
          </div>
        )}
      </div>

      {/* Footer Status Strip */}
      <div className="h-6 px-3 border-t border-[#1e293b] bg-[#090d14] flex items-center justify-between text-[9px] font-mono text-slate-400">
        <span className="flex items-center gap-1.5">
          <span className="w-1.5 h-1.5 rounded-full bg-emerald-400" />
          HEADING: {waistDeg.toFixed(1)}° WAIST
        </span>
        <span className="text-cyan-400 font-bold">SPATIAL GATE: 14.0cm MULTI-INSTANCE | 9.5cm EXCLUSION</span>
      </div>
    </div>
  );
}
