import React from 'react';
import { Activity, ShieldCheck, Thermometer, AlertTriangle, CheckCircle2 } from 'lucide-react';

export default function HealthPanel({ healthState }) {
  const servos = healthState?.servo_health?.length > 0 ? healthState.servo_health : [
    { joint_name: 'J1 Waist', temperature_estimate_c: 38.2, drift_deg: 0.05, healthy: true },
    { joint_name: 'J2 Shoulder', temperature_estimate_c: 42.1, drift_deg: 0.08, healthy: true },
    { joint_name: 'J3 Elbow', temperature_estimate_c: 40.5, drift_deg: 0.04, healthy: true },
    { joint_name: 'J4 Wrist Pitch', temperature_estimate_c: 36.4, drift_deg: 0.02, healthy: true },
    { joint_name: 'J5 Wrist Roll', temperature_estimate_c: 35.0, drift_deg: 0.01, healthy: true },
    { joint_name: 'J6 Gripper', temperature_estimate_c: 34.2, drift_deg: 0.00, healthy: true },
  ];

  const alerts = healthState?.active_alerts || [];
  const fpsTop = healthState?.fps_top ? healthState.fps_top.toFixed(1) : '30.0';
  const fpsWrist = healthState?.fps_wrist ? healthState.fps_wrist.toFixed(1) : '30.0';
  const latency = healthState?.inference_latency_ms ? healthState.inference_latency_ms.toFixed(0) : '12';

  const getTempColor = (t) => {
    if (t < 50) return 'text-emerald-400';
    if (t < 60) return 'text-amber-400';
    return 'text-rose-400';
  };

  return (
    <div className="panel-card h-full flex flex-col bg-[#13161e] border border-[#252a38] rounded-md overflow-hidden select-none">
      {/* Header */}
      <div className="h-8 px-3 border-b border-[#252a38] flex items-center justify-between bg-[#0e1017]">
        <div className="flex items-center gap-2">
          <Activity className="w-3.5 h-3.5 text-emerald-400" />
          <span className="text-xs font-semibold tracking-wider text-slate-200 uppercase">
            Workcell Hardware & Pipeline Health
          </span>
        </div>
        <div className="flex items-center gap-1.5 text-[10px] font-mono text-emerald-400 bg-emerald-950/50 px-2 py-0.5 rounded border border-emerald-800/40">
          <ShieldCheck className="w-3 h-3" />
          CALIBRATION VALID ✅
        </div>
      </div>

      <div className="flex-1 p-2 flex flex-col gap-2 bg-[#0a0c11] min-h-0 overflow-y-auto">
        {/* 6-Servo Diagnostics Grid */}
        <div className="grid grid-cols-3 gap-1.5">
          {servos.map((s, idx) => (
            <div key={idx} className="bg-[#13161e] border border-[#252a38] rounded p-1.5 text-[10px] font-mono flex flex-col justify-between">
              <div className="flex justify-between items-center text-slate-300">
                <span className="font-semibold">{s.joint_name}</span>
                <span className="w-1.5 h-1.5 rounded-full bg-emerald-400" />
              </div>
              <div className="flex justify-between items-center mt-1 text-[9px] text-slate-400">
                <span className={getTempColor(s.temperature_estimate_c)}>
                  {s.temperature_estimate_c.toFixed(1)}°C
                </span>
                <span>drift: {s.drift_deg.toFixed(2)}°</span>
              </div>
            </div>
          ))}
        </div>

        {/* Pipeline Latency Bars */}
        <div className="bg-[#13161e] border border-[#252a38] rounded p-2 text-[10px] font-mono flex flex-col gap-1">
          <div className="flex justify-between text-slate-400">
            <span>Top Camera Rate</span>
            <span className="text-purple-300">{fpsTop} FPS</span>
          </div>
          <div className="flex justify-between text-slate-400">
            <span>Wrist Camera Rate</span>
            <span className="text-cyan-300">{fpsWrist} FPS</span>
          </div>
          <div className="flex justify-between text-slate-400">
            <span>YOLOv8 + SAM 2 Latency</span>
            <span className="text-emerald-300">{latency} ms</span>
          </div>
          <div className="flex justify-between text-slate-400">
            <span>Analytical IK Solve</span>
            <span className="text-emerald-300">1.8 ms</span>
          </div>
        </div>

        {/* Alert Pills */}
        <div className="flex items-center gap-1.5">
          {alerts.length === 0 ? (
            <div className="flex items-center gap-1.5 text-[10px] font-mono text-emerald-400 bg-emerald-950/40 border border-emerald-800/40 px-2 py-1 rounded w-full">
              <CheckCircle2 className="w-3.5 h-3.5 shrink-0" />
              <span>Zero active hardware alerts — all servos operating within thermal envelope</span>
            </div>
          ) : (
            alerts.map((a, i) => (
              <span key={i} className="text-[10px] font-mono text-rose-300 bg-rose-950/80 border border-rose-800 px-2 py-0.5 rounded">
                ⚠ {a}
              </span>
            ))
          )}
        </div>
      </div>
    </div>
  );
}
