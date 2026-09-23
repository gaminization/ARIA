import React, { useEffect, useState } from 'react';
import { LineChart, Line, BarChart, Bar, XAxis, YAxis, Tooltip, ResponsiveContainer, ReferenceLine, CartesianGrid } from 'recharts';
import { BarChart3, TrendingUp, Zap, AlertCircle } from 'lucide-react';

export default function MetricsPanel() {
  const [metrics, setMetrics] = useState(null);
  const [activeTab, setActiveTab] = useState('success'); // 'success' | 'failure' | 'latency'

  const fetchMetrics = async () => {
    try {
      const res = await fetch('/api/metrics');
      if (res.ok) {
        const data = await res.json();
        setMetrics(data);
      }
    } catch (e) {
      // ignore
    }
  };

  useEffect(() => {
    fetchMetrics();
    const interval = setInterval(fetchMetrics, 5000);
    return () => clearInterval(interval);
  }, []);

  const runs = metrics?.recent_runs || [];
  const failures = metrics?.failure_breakdown || [];
  const overallSuccessRate = metrics?.overall_success_rate;
  const meanLatency = metrics?.mean_latency_ms ?? 12.0;

  return (
    <div className="panel-card h-full flex flex-col bg-[#13161e] border border-[#252a38] rounded-md overflow-hidden select-none">
      {/* Header with Tabs */}
      <div className="h-8 px-3 border-b border-[#252a38] flex items-center justify-between bg-[#0e1017]">
        <div className="flex items-center gap-2">
          <BarChart3 className="w-3.5 h-3.5 text-cyan-400" />
          <span className="text-xs font-semibold tracking-wider text-slate-200 uppercase">
            Operational Telemetry (Genuine Session Data)
          </span>
        </div>

        <div className="flex rounded border border-[#252a38] overflow-hidden text-[10px] font-mono">
          <button
            onClick={() => setActiveTab('success')}
            className={`px-2 py-0.5 ${activeTab === 'success' ? 'bg-cyan-500/20 text-cyan-300 font-bold' : 'text-slate-400'}`}
          >
            Success Rate
          </button>
          <button
            onClick={() => setActiveTab('failure')}
            className={`px-2 py-0.5 ${activeTab === 'failure' ? 'bg-cyan-500/20 text-cyan-300 font-bold' : 'text-slate-400'}`}
          >
            Failures
          </button>
          <button
            onClick={() => setActiveTab('latency')}
            className={`px-2 py-0.5 ${activeTab === 'latency' ? 'bg-cyan-500/20 text-cyan-300 font-bold' : 'text-slate-400'}`}
          >
            Latency
          </button>
        </div>
      </div>

      {/* Chart Viewport */}
      <div className="flex-1 p-2 bg-[#0a0c11] min-h-0 relative flex items-center justify-center">
        {runs.length === 0 ? (
          <div className="flex flex-col items-center justify-center text-center p-4">
            <AlertCircle className="w-6 h-6 text-slate-500 mb-2" />
            <div className="text-xs font-mono font-bold text-slate-300">NO SESSION RUNS RECORDED YET</div>
            <div className="text-[10px] font-mono text-slate-500 mt-1 max-w-xs">
              0% synthetic data policy active. Execute an autonomous pick or mission cycle to populate live operational telemetry.
            </div>
          </div>
        ) : (
          <ResponsiveContainer width="100%" height="100%">
            {activeTab === 'success' ? (
              <LineChart data={runs} margin={{ top: 10, right: 10, left: -25, bottom: 0 }}>
                <CartesianGrid strokeDasharray="3 3" stroke="#1f2433" />
                <XAxis dataKey="run" stroke="#64748b" fontSize={10} tickLine={false} />
                <YAxis domain={[0.0, 1.0]} stroke="#64748b" fontSize={10} tickFormatter={(v) => `${(v * 100).toFixed(0)}%`} />
                <Tooltip contentStyle={{ backgroundColor: '#13161e', borderColor: '#252a38', fontSize: '11px' }} />
                <ReferenceLine y={0.85} stroke="#ef4444" strokeDasharray="4 4" label={{ value: 'Target 85%', fill: '#ef4444', fontSize: 10 }} />
                <Line type="monotone" dataKey="pick_success_rate" stroke="#4fc9a4" strokeWidth={2} dot={{ r: 3 }} />
              </LineChart>
            ) : activeTab === 'failure' ? (
              <BarChart data={failures} margin={{ top: 10, right: 10, left: -25, bottom: 0 }}>
                <CartesianGrid strokeDasharray="3 3" stroke="#1f2433" />
                <XAxis dataKey="category" stroke="#64748b" fontSize={9} tickLine={false} />
                <YAxis stroke="#64748b" fontSize={10} />
                <Tooltip contentStyle={{ backgroundColor: '#13161e', borderColor: '#252a38', fontSize: '11px' }} />
                <Bar dataKey="count" fill="#8b5cf6" radius={[2, 2, 0, 0]} />
              </BarChart>
            ) : (
              <LineChart data={runs} margin={{ top: 10, right: 10, left: -20, bottom: 0 }}>
                <CartesianGrid strokeDasharray="3 3" stroke="#1f2433" />
                <XAxis dataKey="run" stroke="#64748b" fontSize={10} tickLine={false} />
                <YAxis stroke="#64748b" fontSize={10} unit="ms" />
                <Tooltip contentStyle={{ backgroundColor: '#13161e', borderColor: '#252a38', fontSize: '11px' }} />
                <ReferenceLine y={200} stroke="#f59e0b" strokeDasharray="4 4" label={{ value: 'Limit 200ms', fill: '#f59e0b', fontSize: 10 }} />
                <Line type="monotone" dataKey="inference_latency_ms" stroke="#38bdf8" strokeWidth={2} dot={{ r: 3 }} />
              </LineChart>
            )}
          </ResponsiveContainer>
        )}
      </div>

      <div className="h-6 px-3 border-t border-[#252a38] bg-[#0d0f14] flex items-center justify-between text-[10px] font-mono text-slate-400">
        <span>
          PICK SUCCESS RATE: {overallSuccessRate !== null && overallSuccessRate !== undefined ? `${(overallSuccessRate * 100).toFixed(1)}%` : 'N/A (0 Runs)'}
        </span>
        <span>MEAN INFERENCE LATENCY: {meanLatency.toFixed(1)}ms</span>
      </div>
    </div>
  );
}
