import React from 'react';
import { Activity, ShieldCheck, Thermometer, AlertTriangle, CheckCircle2 } from 'lucide-react';

export default function HealthPanel({ healthState }) {
  const servos = healthState?.servo_health || [];
  const alerts = healthState?.active_alerts || [];
  const formatMetric = (value, digits = 1, positiveOnly = false) =>
    Number.isFinite(value) && (!positiveOnly || value > 0)
      ? value.toFixed(digits)
      : 'N/A';

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
          ESTIMATES · NOT SENSORS
        </div>
      </div>

      <div className="flex-1 p-2 flex flex-col gap-2 bg-[#0a0c11] min-h-0 overflow-y-auto">
        {/* Servo diagnostics for the four arm axes and gripper */}
        <div className="grid grid-cols-3 gap-1.5">
          {servos.map((s, idx) => (
            <div key={idx} className="bg-[#13161e] border border-[#252a38] rounded p-1.5 text-[10px] font-mono flex flex-col justify-between">
              <div className="flex justify-between items-center text-slate-300">
                <span className="font-semibold">{s.joint_name}</span>
                <span className={`w-1.5 h-1.5 rounded-full ${s.healthy === true ? 'bg-emerald-400' : s.healthy === false ? 'bg-rose-400' : 'bg-slate-500'}`} />
              </div>
              <div className="flex justify-between items-center mt-1 text-[9px] text-slate-400">
                <span className={Number.isFinite(s.temperature_estimate_c) ? getTempColor(s.temperature_estimate_c) : ''}>
                  {formatMetric(s.temperature_estimate_c)}°C est.
                </span>
                <span>drift: {formatMetric(s.drift_deg, 2)}°</span>
              </div>
            </div>
          ))}
          {servos.length === 0 && (
            <div className="col-span-3 text-center text-[10px] font-mono text-slate-500 py-2">
              No servo health telemetry received
            </div>
          )}
        </div>

        {/* Pipeline Latency Bars */}
        <div className="bg-[#13161e] border border-[#252a38] rounded p-2 text-[10px] font-mono flex flex-col gap-1">
          <div className="flex justify-between text-slate-400">
            <span>Top Camera Rate</span>
            <span className="text-purple-300">{formatMetric(healthState?.fps_top, 1, true)} FPS</span>
          </div>
          <div className="flex justify-between text-slate-400">
            <span>Wrist Camera Rate</span>
            <span className="text-cyan-300">{formatMetric(healthState?.fps_wrist, 1, true)} FPS</span>
          </div>
          <div className="flex justify-between text-slate-400">
            <span>YOLOv8 + SAM 2 Latency</span>
            <span className="text-emerald-300">{formatMetric(healthState?.inference_latency_ms, 0, true)} ms</span>
          </div>
        </div>

        {/* Alert Pills */}
        <div className="flex items-center gap-1.5">
          {alerts.length === 0 && servos.length === 0 ? (
            <div className="text-[10px] font-mono text-slate-400 w-full">
              Health and alert status unavailable until telemetry arrives
            </div>
          ) : alerts.length === 0 ? (
            <div className="flex items-center gap-1.5 text-[10px] font-mono text-emerald-400 bg-emerald-950/40 border border-emerald-800/40 px-2 py-1 rounded w-full">
              <CheckCircle2 className="w-3.5 h-3.5 shrink-0" />
              <span>No active alerts reported by the health monitor</span>
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
