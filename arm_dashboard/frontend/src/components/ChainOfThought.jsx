import React, { useState, useRef, useEffect } from 'react';
import { Terminal, Copy, Check, Filter, ArrowDown } from 'lucide-react';

export default function ChainOfThought({ logs = [] }) {
  const [filter, setFilter] = useState('ALL');
  const [copied, setCopied] = useState(false);
  const [autoScroll, setAutoScroll] = useState(true);
  const scrollContainerRef = useRef(null);

  // Auto-scroll to bottom only when user enables auto-scroll
  useEffect(() => {
    if (autoScroll && scrollContainerRef.current) {
      scrollContainerRef.current.scrollTop = scrollContainerRef.current.scrollHeight;
    }
  }, [logs, autoScroll]);

  // Handle user scrolling
  const handleScroll = () => {
    if (!scrollContainerRef.current) return;
    const { scrollTop, scrollHeight, clientHeight } = scrollContainerRef.current;
    const isAtBottom = scrollHeight - scrollTop - clientHeight < 40;
    setAutoScroll(isAtBottom);
  };

  const copyToClipboard = () => {
    const text = logs.map((l) => (typeof l === 'string' ? l : l.text || '')).join('\n');
    navigator.clipboard.writeText(text);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  const getAgentColor = (text) => {
    const upper = text.toUpperCase();
    if (upper.includes('VISION') || upper.includes('YOLO') || upper.includes('LOCATE')) return 'text-blue-400';
    if (upper.includes('PLANNING') || upper.includes('TASK')) return 'text-purple-400';
    if (upper.includes('CONTROL') || upper.includes('SERVO') || upper.includes('PICK') || upper.includes('IK')) return 'text-emerald-400';
    if (upper.includes('SAFETY') || upper.includes('ESTOP') || upper.includes('ERROR') || upper.includes('FAIL')) return 'text-rose-400 font-bold';
    if (upper.includes('MEMORY') || upper.includes('WORLD')) return 'text-amber-400';
    if (upper.includes('DIALOGUE')) return 'text-teal-400';
    if (upper.includes('AFFORDANCE')) return 'text-indigo-400';
    return 'text-slate-300';
  };

  const filteredLogs = logs.filter((l) => {
    const text = (typeof l === 'string' ? l : l.text || '').toUpperCase();
    if (filter === 'ERRORS') return text.includes('ERROR') || text.includes('FAIL') || text.includes('⚠') || text.includes('ESTOP');
    if (filter === 'VISION') return text.includes('VISION') || text.includes('LOCATE');
    if (filter === 'PLANNING') return text.includes('PLAN') || text.includes('AFFORDANCE');
    if (filter === 'CONTROL') return text.includes('PICK') || text.includes('SERVO') || text.includes('IK') || text.includes('CONTROL');
    return true;
  });

  return (
    <div className="panel-card h-full flex flex-col bg-[#13161e] border border-[#252a38] rounded-md overflow-hidden select-none">
      {/* Header */}
      <div className="h-9 px-3 border-b border-[#252a38] flex items-center justify-between bg-[#0e1017]">
        <div className="flex items-center gap-2">
          <Terminal className="w-3.5 h-3.5 text-cyan-400" />
          <span className="text-xs font-semibold tracking-wider text-slate-200 uppercase">
            Decision Trace & Multi-Agent CoT Log
          </span>
          <span className="text-[10px] font-mono text-slate-500">({filteredLogs.length} events)</span>
        </div>

        <div className="flex items-center gap-2 text-xs font-mono">
          {/* Filter Dropdown */}
          <div className="flex items-center gap-1 bg-[#1a1f2c] border border-[#252a38] rounded px-1.5 py-0.5">
            <Filter className="w-3 h-3 text-slate-400" />
            <select
              value={filter}
              onChange={(e) => setFilter(e.target.value)}
              className="bg-transparent text-[10px] text-slate-300 focus:outline-none cursor-pointer"
            >
              <option value="ALL" className="bg-[#13161e]">All Agents</option>
              <option value="VISION" className="bg-[#13161e]">VisionAgent</option>
              <option value="PLANNING" className="bg-[#13161e]">PlanningAgent</option>
              <option value="CONTROL" className="bg-[#13161e]">ControlAgent</option>
              <option value="ERRORS" className="bg-[#13161e]">Warnings & Errors</option>
            </select>
          </div>

          {/* Copy Button */}
          <button
            onClick={copyToClipboard}
            className="p-1 text-slate-400 hover:text-slate-200 rounded hover:bg-slate-800 transition-colors"
            title="Copy logs to clipboard"
          >
            {copied ? <Check className="w-3.5 h-3.5 text-emerald-400" /> : <Copy className="w-3.5 h-3.5" />}
          </button>
        </div>
      </div>

      {/* Log Entries Container — Fully scrollable up and down */}
      <div
        ref={scrollContainerRef}
        onScroll={handleScroll}
        className="flex-1 min-h-0 overflow-y-auto p-2.5 font-mono text-[11px] leading-relaxed flex flex-col gap-1 bg-[#090b0f]"
      >
        {filteredLogs.length === 0 ? (
          <div className="text-slate-600 text-center py-6">No matching logs in buffer.</div>
        ) : (
          filteredLogs.map((item, idx) => {
            const rawText = typeof item === 'string' ? item : item.text || '';
            const ts = item.time
              ? new Date(item.time * 1000).toISOString().substr(11, 12)
              : new Date().toISOString().substr(11, 12);
            const color = getAgentColor(rawText);

            return (
              <div key={idx} className="flex items-start gap-2 hover:bg-white/[0.02] px-1 py-0.5 rounded">
                <span className="text-slate-500 select-none shrink-0 font-mono text-[10px]">[{ts}]</span>
                <span className={`break-all ${color}`}>{rawText}</span>
              </div>
            );
          })
        )}
      </div>

      {/* Footer Status Bar with Auto-scroll resume */}
      <div className="h-6 px-3 border-t border-[#252a38] bg-[#0d0f14] flex items-center justify-between text-[10px] font-mono text-slate-500">
        <div className="flex items-center gap-2">
          <span>MAX BUFFER: 1000</span>
          <span>·</span>
          <span>RESONANCE: REAL-TIME</span>
        </div>
        {!autoScroll && (
          <button
            onClick={() => {
              setAutoScroll(true);
              if (scrollContainerRef.current) {
                scrollContainerRef.current.scrollTop = scrollContainerRef.current.scrollHeight;
              }
            }}
            className="flex items-center gap-1 text-cyan-400 hover:text-cyan-300 animate-bounce"
          >
            <ArrowDown className="w-3 h-3" />
            JUMP TO LIVE
          </button>
        )}
      </div>
    </div>
  );
}
