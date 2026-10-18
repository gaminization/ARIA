import React from 'react';
import { Award } from 'lucide-react';

/**
 * Benchmark comparison panel.
 *
 * ARIA's own metrics are shown only when the backend actually reports them
 * (via healthState); otherwise "—" is shown. Literature baselines are labelled
 * as such — they are not measurements made by ARIA and must not be presented
 * as if they were.
 */
export default function ComparisonPanel({ healthState }) {
  const fmt = (v, suffix = '') =>
    Number.isFinite(v) ? `${v}${suffix}` : '—';

  const ariaTgsr = healthState?.task_grasp_success_rate;
  const ariaLatency = healthState?.inference_latency_ms;

  return (
    <div className="panel-card h-full flex flex-col bg-[#13161e] border border-[#252a38] rounded-md overflow-hidden select-none">
      <div className="h-8 px-3 border-b border-[#252a38] flex items-center justify-between bg-[#0e1017]">
        <div className="flex items-center gap-2">
          <Award className="w-3.5 h-3.5 text-amber-400" />
          <span className="text-xs font-semibold tracking-wider text-slate-200 uppercase">
            Benchmark Comparison
          </span>
        </div>
        <span className="text-[10px] font-mono text-slate-400 bg-slate-800 px-2 py-0.5 rounded border border-slate-700">
          ARIA measured · baseline = literature
        </span>
      </div>

      <div className="flex-1 p-3 flex flex-col justify-around bg-[#0a0c11] text-xs font-mono">
        <div className="grid grid-cols-2 gap-3">
          {/* ARIA System — values come from the backend or are shown as unavailable */}
          <div className="bg-[#13161e] border border-cyan-500/40 rounded p-2.5 flex flex-col gap-2">
            <div className="flex items-center justify-between text-cyan-300 font-bold">
              <span>ARIA (Ours)</span>
              <span className="text-[10px] bg-cyan-950 px-1.5 py-0.5 rounded border border-cyan-800">ACTIVE</span>
            </div>
            <div className="flex flex-col gap-1">
              <div className="flex justify-between text-slate-400">
                <span>Task Grasp Success (TGSR):</span>
                <span className="text-emerald-400 font-bold">
                  {fmt(ariaTgsr, '%')}
                </span>
              </div>
              <div className="flex justify-between text-slate-400">
                <span>Inference Latency:</span>
                <span className="text-cyan-300 font-bold">
                  {fmt(ariaLatency, ' ms')}
                </span>
              </div>
              <div className="flex justify-between text-slate-400">
                <span>Object Generalization:</span>
                <span className="text-purple-300 font-bold">Open-World</span>
              </div>
            </div>
          </div>

          {/* Literature baseline — NOT measured by ARIA */}
          <div className="bg-[#13161e] border border-[#252a38] rounded p-2.5 flex flex-col gap-2 opacity-80">
            <div className="flex items-center justify-between text-slate-300 font-bold">
              <span>Chen et al. (2025)</span>
              <span className="text-[10px] bg-slate-800 px-1.5 py-0.5 rounded">LITERATURE</span>
            </div>
            <div className="flex flex-col gap-1">
              <div className="flex justify-between text-slate-400">
                <span>Task Grasp Success (TGSR):</span>
                <span className="text-slate-200 font-bold">83.0% *</span>
              </div>
              <div className="flex justify-between text-slate-400">
                <span>Inference Latency:</span>
                <span className="text-slate-200 font-bold">245 ms *</span>
              </div>
              <div className="flex justify-between text-slate-400">
                <span>Object Generalization:</span>
                <span className="text-slate-400">Fixed Category</span>
              </div>
            </div>
          </div>
        </div>

        <div className="bg-[#13161e] border border-[#252a38] rounded p-2 text-[10px] text-slate-500 flex items-center justify-between">
          <span>ARIA values are measured by the backend; "—" means not yet reported. Baseline values are literature references, not ARIA measurements.</span>
          <span className="text-slate-500 font-bold">* literature</span>
        </div>
      </div>
    </div>
  );
}
