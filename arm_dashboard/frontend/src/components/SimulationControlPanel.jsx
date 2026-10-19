import React, { useState, useEffect, useRef } from 'react';
import { Play, Square, RefreshCw, Layers, Monitor, CheckCircle, AlertTriangle, Cpu, Power } from 'lucide-react';

const DEFAULT_WORLDS = [
  { id: 'aria_tester_workspace.world', name: 'Standard Tester Workspace', description: '80x60cm optical table with colorful benchmark workpieces' },
  { id: 'aria_industrial_workcell.world', name: 'Industrial Workcell', description: 'Conveyor feeding zone with sorting bins and inspection stands' },
  { id: 'aria_workspace.world', name: 'Basic Workspace', description: 'Minimal tabletop environment' },
];

export default function SimulationControlPanel() {
  const [worlds, setWorlds] = useState(DEFAULT_WORLDS);
  const [selectedWorld, setSelectedWorld] = useState('aria_tester_workspace.world');
  const [launchGui, setLaunchGui] = useState(false);
  const [launchSynthetic, setLaunchSynthetic] = useState(false);
  const [simStatus, setSimStatus] = useState({ running: false, world: '', uptime_s: 0, pid: null });
  const [actionLoading, setActionLoading] = useState(false);
  const [feedback, setFeedback] = useState(null);
  // Tracks whether the operator has manually picked a world. Until then the
  // 3-second status poll may sync the selection to the running world, but once
  // the operator chooses, polling must never yank the selection back.
  const userSelectedRef = useRef(false);

  const fetchStatus = async () => {
    try {
      const res = await fetch('/api/simulation/status');
      if (res.ok) {
        const data = await res.json();
        setSimStatus(data);
        if (data.world && !userSelectedRef.current) setSelectedWorld(data.world);
      }
    } catch (e) {
      // ignore
    }
  };

  const fetchWorlds = async () => {
    try {
      const res = await fetch('/api/simulation/worlds');
      if (res.ok) {
        const data = await res.json();
        if (data.worlds && data.worlds.length > 0) {
          setWorlds(data.worlds);
        }
      }
    } catch (e) {
      // ignore
    }
  };

  useEffect(() => {
    fetchWorlds();
    fetchStatus();
    const interval = setInterval(fetchStatus, 3000);
    return () => clearInterval(interval);
  }, []);

  const handleLaunch = async () => {
    setActionLoading(true);
    setFeedback(null);
    try {
      const res = await fetch('/api/simulation/launch', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ world: selectedWorld, gui: launchGui, synthetic: launchSynthetic }),
      });
      const data = await res.json();
      setFeedback(data.success ? `✓ ${data.message}` : `✗ ${data.message}`);
      await fetchStatus();
    } catch (err) {
      setFeedback(`✗ Failed to contact server`);
    } finally {
      setActionLoading(false);
    }
  };

  const handleStop = async () => {
    setActionLoading(true);
    setFeedback(null);
    try {
      const res = await fetch('/api/simulation/stop', { method: 'POST' });
      const data = await res.json();
      setFeedback(data.success ? '✓ Simulation cleanly terminated' : `✗ ${data.message}`);
      await fetchStatus();
    } catch (err) {
      setFeedback('✗ Termination error');
    } finally {
      setActionLoading(false);
    }
  };

  const handleKill = async () => {
    const ok = window.confirm(
      'FULL KILL SIMULATION\n\nThis terminates Gazebo and every ROS 2 node/agent, clears shared memory (/dev/shm) and frees RAM.\n\nContinue?'
    );
    if (!ok) return;
    setActionLoading(true);
    setFeedback(null);
    try {
      const res = await fetch('/api/simulation/kill', { method: 'POST' });
      const data = await res.json();
      setFeedback(data.success
        ? `✓ ${data.message} (${(data.shm_freed_bytes / 1e6).toFixed(1)} MB shm freed)`
        : `✗ ${data.message}`);
      await fetchStatus();
    } catch (err) {
      setFeedback('✗ Full-kill error');
    } finally {
      setActionLoading(false);
    }
  };

  return (
    <div className="panel-card h-full flex flex-col bg-[#13161e] border border-[#252a38] rounded-md overflow-hidden select-none p-4 font-mono">
      {/* Header */}
      <div className="flex items-center justify-between pb-3 border-b border-[#252a38] mb-4">
        <div className="flex items-center gap-2">
          <Layers className="w-4 h-4 text-cyan-400" />
          <span className="text-sm font-bold text-white uppercase tracking-wider">
            Gazebo World & Simulation Orchestrator
          </span>
        </div>
        <div className="flex items-center gap-2 text-xs">
          <span className={`w-2 h-2 rounded-full ${simStatus.running ? 'bg-emerald-400 animate-pulse' : 'bg-rose-500'}`} />
          <span className={simStatus.running ? 'text-emerald-400 font-bold' : 'text-slate-400'}>
            {simStatus.running ? `ACTIVE (PID ${simStatus.pid})` : 'SIMULATION INACTIVE'}
          </span>
        </div>
      </div>

      <div className="grid grid-cols-12 gap-4 flex-1 min-h-0">
        {/* Left Column: Launch Configuration */}
        <div className="col-span-12 md:col-span-7 flex flex-col space-y-4">
          <div>
            <label className="block text-xs font-bold text-slate-300 uppercase mb-2">
              Select Gazebo World Model
            </label>
            <div className="space-y-2">
              {worlds.map((w) => (
                <div
                  key={w.id}
                  onClick={() => { userSelectedRef.current = true; setSelectedWorld(w.id); }}
                  className={`p-3 rounded border cursor-pointer transition-all ${
                    selectedWorld === w.id
                      ? 'bg-cyan-950/40 border-cyan-500 text-white'
                      : 'bg-[#0a0c11] border-[#252a38] text-slate-400 hover:border-slate-600'
                  }`}
                >
                  <div className="flex items-center justify-between">
                    <span className="text-xs font-bold text-cyan-300">{w.name}</span>
                    <span className="text-[10px] text-slate-500">{w.id}</span>
                  </div>
                  <div className="text-[11px] text-slate-400 mt-1">{w.description}</div>
                </div>
              ))}
            </div>
          </div>

          {/* Options */}
          <div className="p-3 bg-[#0a0c11] rounded border border-[#252a38] flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Monitor className="w-4 h-4 text-purple-400" />
              <div>
                <div className="text-xs font-bold text-slate-200">Launch Gazebo GUI Client (gzclient)</div>
                <div className="text-[10px] text-slate-500">Uncheck for headless high-efficiency execution</div>
              </div>
            </div>
            <input
              type="checkbox"
              checked={launchGui}
              onChange={(e) => setLaunchGui(e.target.checked)}
              className="w-4 h-4 rounded accent-cyan-500 cursor-pointer"
            />
          </div>

          {/* Synthetic Demonstration Mode Option */}
          <div className="p-3 bg-[#0a0c11] rounded border border-[#252a38] flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Cpu className="w-4 h-4 text-cyan-400" />
              <div>
                <div className="text-xs font-bold text-cyan-300">Synthetic Runs Mode (Fast Demonstrator)</div>
                <div className="text-[10px] text-slate-400">Direct physical execution: sorting, stacking, gestures & drops</div>
              </div>
            </div>
            <input
              type="checkbox"
              checked={launchSynthetic}
              onChange={(e) => setLaunchSynthetic(e.target.checked)}
              className="w-4 h-4 rounded accent-cyan-500 cursor-pointer"
            />
          </div>

          {/* Action Buttons */}
          <div className="flex items-center gap-2 pt-2">
            <button
              onClick={handleLaunch}
              disabled={actionLoading}
              className="flex-1 py-2.5 px-4 bg-gradient-to-r from-emerald-600 to-cyan-600 hover:from-emerald-500 hover:to-cyan-500 text-white text-xs font-bold rounded flex items-center justify-center gap-2 shadow-md shadow-emerald-950/50 transition-all cursor-pointer"
            >
              <Play className="w-3.5 h-3.5 fill-current" />
              {actionLoading ? 'ORCHESTRATING...' : 'LAUNCH SIMULATION'}
            </button>
            <button
              onClick={handleStop}
              disabled={actionLoading || !simStatus.running}
              className="py-2.5 px-4 bg-rose-950/80 hover:bg-rose-900 border border-rose-700 text-rose-300 text-xs font-bold rounded flex items-center justify-center gap-2 transition-all cursor-pointer disabled:opacity-40"
            >
              <Square className="w-3.5 h-3.5 fill-current" />
              STOP SIMULATION
            </button>
            <button
              onClick={handleKill}
              disabled={actionLoading}
              title="Kill everything: Gazebo + all ROS 2 nodes, clear shared memory and free RAM"
              className="py-2.5 px-4 bg-orange-950/80 hover:bg-orange-900 border border-orange-700 text-orange-300 text-xs font-bold rounded flex items-center justify-center gap-2 transition-all cursor-pointer disabled:opacity-40"
            >
              <Power className="w-3.5 h-3.5" />
              KILL ALL
            </button>
          </div>

          {feedback && (
            <div className="text-xs font-bold px-3 py-2 rounded bg-[#0a0c11] border border-cyan-800 text-cyan-300">
              {feedback}
            </div>
          )}
        </div>

        {/* Right Column: Active Simulation Telemetry */}
        <div className="col-span-12 md:col-span-5 flex flex-col space-y-3 bg-[#0a0c11] p-3.5 rounded border border-[#252a38]">
          <span className="text-xs font-bold text-slate-300 uppercase tracking-wider pb-2 border-b border-[#252a38]">
            Workcell Simulation Runtime
          </span>

          <div className="space-y-2.5 text-xs">
            <div className="flex justify-between py-1 border-b border-[#181c26]">
              <span className="text-slate-500">Status</span>
              <span className={simStatus.running ? 'text-emerald-400 font-bold' : 'text-slate-400'}>
                {simStatus.running ? 'RUNNING' : 'STOPPED'}
              </span>
            </div>
            <div className="flex justify-between py-1 border-b border-[#181c26]">
              <span className="text-slate-500">Active World</span>
              <span className="text-cyan-300">{simStatus.world || selectedWorld}</span>
            </div>
            <div className="flex justify-between py-1 border-b border-[#181c26]">
              <span className="text-slate-500">Process PID</span>
              <span className="text-slate-300">{simStatus.pid || 'N/A'}</span>
            </div>
            <div className="flex justify-between py-1 border-b border-[#181c26]">
              <span className="text-slate-500">Session Uptime</span>
              <span className="text-purple-300">{simStatus.uptime_s ? `${simStatus.uptime_s}s` : '0s'}</span>
            </div>
            <div className="flex justify-between py-1 border-b border-[#181c26]">
              <span className="text-slate-500">Camera Resolution</span>
              <span className="text-emerald-400 font-bold">1080p Verification / 720p Wrist</span>
            </div>
            <div className="flex justify-between py-1">
              <span className="text-slate-500">Perception Models</span>
              <span className="text-cyan-400">Eye-in-Hand Only</span>
            </div>
          </div>

          <div className="mt-auto p-2.5 rounded bg-[#13161e] border border-[#252a38] text-[10px] text-slate-400">
            <div className="text-slate-300 font-bold mb-1">Autonomous Workcell Topology:</div>
            <div>• Top & Side cameras are streamed raw without inference overhead.</div>
            <div>• YOLOv8, Depth-Anything, and VLA run exclusively on the wrist camera.</div>
          </div>
        </div>
      </div>
    </div>
  );
}
