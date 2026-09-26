import React, { useState, useEffect, useRef } from 'react';
import { Bot, ShieldAlert, Wifi, Cpu, Activity, Clock, Layers, Camera, Map, BarChart3, Terminal, Monitor, Maximize2 } from 'lucide-react';
import CameraPanel from './components/CameraPanel';
import ArmVisualiser from './components/ArmVisualiser';
import TaskControl from './components/TaskControl';
import ChainOfThought from './components/ChainOfThought';
import WorldMap from './components/WorldMap';
import MetricsPanel from './components/MetricsPanel';
import HealthPanel from './components/HealthPanel';
import ComparisonPanel from './components/ComparisonPanel';
import SimulationControlPanel from './components/SimulationControlPanel';

export default function App() {
  const [currentPage, setCurrentPage] = useState('overview'); // 'overview' | 'perception' | 'twin' | 'world' | 'tasks' | 'telemetry' | 'simulation'
  const [systemState, setSystemState] = useState(null);
  const [cameraStreams, setCameraStreams] = useState({ top: null, wrist: null, side: null, depth: null });
  const [wsConnected, setWsConnected] = useState(false);
  const [currentTime, setCurrentTime] = useState(new Date().toLocaleTimeString());
  const [activeBottomTab, setActiveBottomTab] = useState('health'); // 'health' | 'benchmark'

  // Persistent Perception & CV Display Settings
  const defaultPerceptionSettings = {
    activeTab: 'grid',
    showOverlays: {
      detections: true,
      tracking: true,
      grasp: true,
      depth: false,
      vla: true,
    },
    showSettingsDrawer: false,
    showInspector: false,
    streamQuality: 'balanced',
    confidenceGate: 0.45,
  };

  const [perceptionSettings, setPerceptionSettings] = useState(() => {
    try {
      const saved = localStorage.getItem('aria_perception_settings');
      if (saved) return { ...defaultPerceptionSettings, ...JSON.parse(saved) };
    } catch (e) {}
    return defaultPerceptionSettings;
  });

  useEffect(() => {
    try {
      localStorage.setItem('aria_perception_settings', JSON.stringify(perceptionSettings));
    } catch (e) {}
  }, [perceptionSettings]);

  const updatePerceptionSettings = (updater) => {
    setPerceptionSettings((prev) => {
      const next = typeof updater === 'function' ? updater(prev) : { ...prev, ...updater };
      return next;
    });
  };

  const wsStateRef = useRef(null);
  const wsCamRef = useRef(null);
  const backoffRef = useRef(1000);

  // Clock ticker
  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentTime(new Date().toLocaleTimeString());
    }, 1000);
    return () => clearInterval(timer);
  }, []);

  // Draggable section resizing state
  const [rowHeights, setRowHeights] = useState([41, 25, 34]); // percentages
  const [row1Widths, setRow1Widths] = useState([33.3, 33.3, 33.4]); // Row 1 columns
  const [row3Widths, setRow3Widths] = useState([35, 33, 32]); // Row 3 columns

  const startRowResize = (splitIdx, e) => {
    e.preventDefault();
    const startY = e.clientY;
    const initial = [...rowHeights];
    const container = document.getElementById('overview-container');
    const totalH = container ? container.getBoundingClientRect().height : window.innerHeight;

    const onPointerMove = (ev) => {
      const dy = ev.clientY - startY;
      const dPct = (dy / totalH) * 100;
      setRowHeights(() => {
        const next = [...initial];
        if (splitIdx === 0) {
          const n0 = Math.max(22, Math.min(60, next[0] + dPct));
          const diff = n0 - next[0];
          const n1 = Math.max(15, next[1] - diff);
          return [n0, n1, next[2]];
        } else {
          const n1 = Math.max(15, Math.min(48, next[1] + dPct));
          const diff = n1 - next[1];
          const n2 = Math.max(20, next[2] - diff);
          return [next[0], n1, n2];
        }
      });
    };

    const onPointerUp = () => {
      window.removeEventListener('pointermove', onPointerMove);
      window.removeEventListener('pointerup', onPointerUp);
    };

    window.addEventListener('pointermove', onPointerMove);
    window.addEventListener('pointerup', onPointerUp);
  };

  const startColResize = (rowIdx, splitIdx, e) => {
    e.preventDefault();
    const startX = e.clientX;
    const widths = rowIdx === 1 ? row1Widths : row3Widths;
    const initial = [...widths];
    const container = e.currentTarget.parentElement;
    const totalW = container ? container.getBoundingClientRect().width : window.innerWidth;

    const onPointerMove = (ev) => {
      const dx = ev.clientX - startX;
      const dPct = (dx / totalW) * 100;
      const setter = rowIdx === 1 ? setRow1Widths : setRow3Widths;
      setter(() => {
        const next = [...initial];
        if (splitIdx === 0) {
          const n0 = Math.max(18, Math.min(60, next[0] + dPct));
          const diff = n0 - next[0];
          const n1 = Math.max(18, next[1] - diff);
          return [n0, n1, next[2]];
        } else {
          const n1 = Math.max(18, Math.min(60, next[1] + dPct));
          const diff = n1 - next[1];
          const n2 = Math.max(18, next[2] - diff);
          return [next[0], n1, n2];
        }
      });
    };

    const onPointerUp = () => {
      window.removeEventListener('pointermove', onPointerMove);
      window.removeEventListener('pointerup', onPointerUp);
    };

    window.addEventListener('pointermove', onPointerMove);
    window.addEventListener('pointerup', onPointerUp);
  };

  // WebSocket Connection Management
  useEffect(() => {
    let unmounted = false;

    const connectWebSockets = () => {
      if (unmounted) return;

      const host = window.location.hostname || 'localhost';
      const wsProto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      const port = window.location.port === '5173' ? '8000' : (window.location.port || '8000');

      // 1. State WebSocket (/ws/state)
      const stateUrl = `${wsProto}//${host}:${port}/ws/state`;
      const wsState = new WebSocket(stateUrl);

      wsState.onopen = () => {
        setWsConnected(true);
        backoffRef.current = 1000;
      };

      wsState.onmessage = (evt) => {
        try {
          const data = JSON.parse(evt.data);
          setSystemState(data);
        } catch (e) {}
      };

      wsState.onclose = () => {
        setWsConnected(false);
        if (!unmounted) {
          setTimeout(connectWebSockets, backoffRef.current);
          backoffRef.current = Math.min(8000, backoffRef.current * 2);
        }
      };

      // 2. Camera WebSocket (/ws/cameras)
      const camUrl = `${wsProto}//${host}:${port}/ws/cameras`;
      const wsCam = new WebSocket(camUrl);

      wsCam.onmessage = (evt) => {
        try {
          const data = JSON.parse(evt.data);
          if (data && typeof data === 'object') {
            setCameraStreams((prev) => ({ ...prev, ...data }));
          }
        } catch (e) {}
      };

      wsStateRef.current = wsState;
      wsCamRef.current = wsCam;
    };

    connectWebSockets();

    return () => {
      unmounted = true;
      if (wsStateRef.current) wsStateRef.current.close();
      if (wsCamRef.current) wsCamRef.current.close();
    };
  }, []);

  // REST Dispatch Handlers
  const handleSendCommand = async (command) => {
    try {
      await fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command }),
      });
    } catch (e) {
      console.error(e);
    }
  };

  const handleApprove = async () => {
    try {
      await fetch('/api/approve', { method: 'POST' });
    } catch (e) {
      console.error(e);
    }
  };

  const handleReject = async () => {
    try {
      await fetch('/api/reject', { method: 'POST' });
    } catch (e) {
      console.error(e);
    }
  };

  const handleEStop = async () => {
    try {
      await fetch('/api/estop', { method: 'POST' });
    } catch (e) {
      console.error(e);
    }
  };

  const handleReleaseEStop = async () => {
    try {
      await fetch('/api/release_estop', { method: 'POST' });
    } catch (e) {
      console.error(e);
    }
  };

  const handleReset = async () => {
    try {
      await fetch('/api/reset', { method: 'POST' });
    } catch (e) {
      console.error(e);
    }
  };

  const jointAngles = systemState?.joints?.current_angles || [0, 0, 0, 0, 0, 0];

  const navTabs = [
    { id: 'overview', label: 'OVERVIEW', icon: Layers },
    { id: 'perception', label: 'PERCEPTION & VISION', icon: Camera },
    { id: 'twin', label: '3D DIGITAL TWIN', icon: Bot },
    { id: 'world', label: '360° WORLD MODEL', icon: Map },
    { id: 'tasks', label: 'TASK & COT', icon: Terminal },
    { id: 'telemetry', label: 'OPERATIONAL TELEMETRY', icon: BarChart3 },
    { id: 'simulation', label: 'SIMULATION CONTROL', icon: Monitor },
  ];

  return (
    <div className="h-screen w-screen flex flex-col bg-[#0d0f14] text-[#e2e6f0] overflow-hidden select-none technical-grid">
      {/* ── TOP PRIMARY BAR ──────────────────────────────────────── */}
      <header className="h-11 border-b border-[#252a38] bg-[#090b10] px-3.5 flex items-center justify-between shrink-0 z-30">
        {/* Left: Branding */}
        <div className="flex items-center gap-3">
          <div className="w-7 h-7 rounded bg-gradient-to-br from-indigo-600 to-violet-700 flex items-center justify-center border border-violet-400/40 shadow-sm shadow-violet-900/50">
            <Bot className="w-4 h-4 text-white" />
          </div>
          <div className="flex flex-col">
            <div className="flex items-center gap-2">
              <span className="text-xs font-black tracking-widest text-white uppercase font-mono">
                ARIA CONTROL CENTER
              </span>
              <span className="text-[9px] font-mono px-1.5 py-0.2 rounded bg-violet-950/80 border border-violet-700/50 text-violet-300 font-bold">
                STAGE 3B
              </span>
            </div>
            <span className="text-[9px] font-mono text-[#7a8399] tracking-tight">
              Autonomous Robotic Intelligence Architecture
            </span>
          </div>
        </div>

        {/* Center: Live Telemetry Indicators */}
        <div className="hidden md:flex items-center gap-3 text-[11px] font-mono">
          <div className="flex items-center gap-1.5 px-2 py-0.5 rounded bg-[#13161e] border border-[#252a38]">
            <span className={`w-2 h-2 rounded-full ${wsConnected ? 'bg-emerald-400 animate-pulse' : 'bg-rose-500'}`} />
            <span className="text-slate-300 font-semibold">{wsConnected ? 'ARIA ONLINE' : 'DISCONNECTED'}</span>
          </div>

          <div className="flex items-center gap-1.5 px-2 py-0.5 rounded bg-[#13161e] border border-[#252a38]">
            <span className={`w-2 h-2 rounded-full ${systemState?.ros_connected ? 'bg-emerald-400' : 'bg-cyan-400'}`} />
            <span className="text-slate-300">ROS 2 CONNECTED</span>
          </div>

          <div className="flex items-center gap-1 text-slate-400 px-2 py-0.5 rounded bg-[#13161e] border border-[#252a38]">
            <Activity className="w-3 h-3 text-cyan-400" />
            <span>30 FPS · 10ms</span>
          </div>

          <div className="flex items-center gap-1 text-slate-400 px-2 py-0.5 rounded bg-[#13161e] border border-[#252a38]">
            <Clock className="w-3 h-3 text-purple-400" />
            <span>{currentTime}</span>
          </div>
        </div>

        {/* Right: Permanent E-STOP & Reset */}
        <div className="flex items-center gap-2">
          <button
            onClick={handleReleaseEStop}
            className="px-2.5 py-1 bg-emerald-950/80 hover:bg-emerald-900 text-emerald-300 text-xs font-mono font-bold rounded border border-emerald-700/60 shadow-sm flex items-center gap-1 tracking-wider transition-all cursor-pointer"
          >
            CLEAR E-STOP
          </button>
          <button
            onClick={handleEStop}
            className="px-3.5 py-1 bg-gradient-to-r from-red-600 to-rose-600 hover:from-red-500 hover:to-rose-500 text-white text-xs font-bold font-mono rounded border border-red-400/50 shadow-md shadow-red-950/60 flex items-center gap-1.5 tracking-wider transition-all cursor-pointer"
          >
            <ShieldAlert className="w-3.5 h-3.5 animate-pulse" />
            E-STOP
          </button>
        </div>
      </header>

      {/* ── SECONDARY NAVIGATION BAR (MULTI-PAGE VIEWS) ──────────── */}
      <nav className="h-8 px-3 border-b border-[#252a38] bg-[#0d0f15] flex items-center gap-1 shrink-0 z-20 overflow-x-auto">
        {navTabs.map((tab) => {
          const Icon = tab.icon;
          const isActive = currentPage === tab.id;
          return (
            <button
              key={tab.id}
              onClick={() => setCurrentPage(tab.id)}
              className={`px-2.5 py-1 text-[11px] font-mono rounded flex items-center gap-1.5 transition-all whitespace-nowrap cursor-pointer ${
                isActive
                  ? 'bg-gradient-to-r from-indigo-950/90 to-violet-950/90 border border-violet-600/70 text-violet-200 font-bold shadow-sm'
                  : 'text-slate-400 hover:text-slate-200 hover:bg-[#151822]'
              }`}
            >
              <Icon className={`w-3.5 h-3.5 ${isActive ? 'text-violet-400' : 'text-slate-500'}`} />
              <span>{tab.label}</span>
            </button>
          );
        })}
      </nav>

      {/* ── MAIN WORKSPACE VIEWPORT (Zero Outer Scrollbars) ──────── */}
      <main className="flex-1 min-h-0 p-2 overflow-hidden flex flex-col">
        {currentPage === 'overview' && (
          <div id="overview-container" className="flex-1 min-h-0 flex flex-col gap-1 overflow-hidden select-none">
            {/* ROW 1: Quadrants Q1, Q2, Q3 with Quick-Maximize header icons & horizontal drag */}
            <div style={{ height: `${rowHeights[0]}%` }} className="min-h-[120px] flex gap-1 overflow-hidden">
              {/* Q1: Camera Feeds */}
              <div style={{ width: `${row1Widths[0]}%` }} className="min-w-[150px] h-full relative group overflow-hidden">
                <button
                  onClick={() => setCurrentPage('perception')}
                  className="absolute top-2 right-12 z-20 p-1 bg-black/70 hover:bg-cyan-950 text-slate-400 hover:text-cyan-300 rounded border border-white/10 opacity-0 group-hover:opacity-100 transition-opacity cursor-pointer"
                  title="Expand to Fullscreen Perception View"
                >
                  <Maximize2 className="w-3 h-3" />
                </button>
                <CameraPanel
                  cameraStreams={cameraStreams}
                  visionState={systemState?.vision}
                  healthState={systemState?.health}
                  taskState={systemState?.task}
                  memoryState={systemState?.memory}
                  jointState={systemState?.joints}
                  servoState={systemState?.servo}
                  settings={perceptionSettings}
                  onUpdateSettings={updatePerceptionSettings}
                  isCompact={true}
                />
              </div>

              {/* Col Splitter 1-2 */}
              <div
                onPointerDown={(e) => startColResize(1, 0, e)}
                className="w-1.5 h-full bg-[#121622] hover:bg-cyan-500/70 active:bg-cyan-400 cursor-col-resize flex flex-col items-center justify-center transition-colors group z-20"
                title="Drag horizontally to resize cameras / arm twin"
              >
                <div className="h-6 w-0.5 rounded-full bg-slate-600 group-hover:bg-cyan-300" />
              </div>

              {/* Q2: 3D Arm Digital Twin */}
              <div style={{ width: `${row1Widths[1]}%` }} className="min-w-[150px] h-full relative group overflow-hidden">
                <button
                  onClick={() => setCurrentPage('twin')}
                  className="absolute top-2 right-2 z-20 p-1 bg-black/70 hover:bg-violet-950 text-slate-400 hover:text-violet-300 rounded border border-white/10 opacity-0 group-hover:opacity-100 transition-opacity cursor-pointer"
                  title="Expand to Fullscreen 3D Twin View"
                >
                  <Maximize2 className="w-3 h-3" />
                </button>
                <ArmVisualiser jointState={systemState?.joints} />
              </div>

              {/* Col Splitter 2-3 */}
              <div
                onPointerDown={(e) => startColResize(1, 1, e)}
                className="w-1.5 h-full bg-[#121622] hover:bg-cyan-500/70 active:bg-cyan-400 cursor-col-resize flex flex-col items-center justify-center transition-colors group z-20"
                title="Drag horizontally to resize arm twin / task control"
              >
                <div className="h-6 w-0.5 rounded-full bg-slate-600 group-hover:bg-cyan-300" />
              </div>

              {/* Q3: Task Control */}
              <div style={{ width: `${row1Widths[2]}%` }} className="min-w-[180px] h-full relative group overflow-hidden">
                <button
                  onClick={() => setCurrentPage('tasks')}
                  className="absolute top-2 right-2 z-20 p-1 bg-black/70 hover:bg-indigo-950 text-slate-400 hover:text-indigo-300 rounded border border-white/10 opacity-0 group-hover:opacity-100 transition-opacity cursor-pointer"
                  title="Expand to Fullscreen Task Orchestrator"
                >
                  <Maximize2 className="w-3 h-3" />
                </button>
                <TaskControl
                  taskState={systemState?.task}
                  onSendCommand={handleSendCommand}
                  onApprove={handleApprove}
                  onReject={handleReject}
                  onEStop={handleEStop}
                  onReset={handleReset}
                />
              </div>
            </div>

            {/* Row Splitter 1-2 */}
            <div
              onPointerDown={(e) => startRowResize(0, e)}
              className="h-1.5 w-full bg-[#121622] hover:bg-cyan-500/70 active:bg-cyan-400 cursor-row-resize flex items-center justify-center transition-colors group z-20"
              title="Drag vertically to resize upper quadrants / chain of thought"
            >
              <div className="w-12 h-0.5 rounded-full bg-slate-600 group-hover:bg-cyan-300" />
            </div>

            {/* ROW 2: Chain of Thought */}
            <div style={{ height: `${rowHeights[1]}%` }} className="min-h-[80px] w-full overflow-hidden">
              <ChainOfThought logs={systemState?.cot || []} />
            </div>

            {/* Row Splitter 2-3 */}
            <div
              onPointerDown={(e) => startRowResize(1, e)}
              className="h-1.5 w-full bg-[#121622] hover:bg-cyan-500/70 active:bg-cyan-400 cursor-row-resize flex items-center justify-center transition-colors group z-20"
              title="Drag vertically to resize chain of thought / bottom panels"
            >
              <div className="w-12 h-0.5 rounded-full bg-slate-600 group-hover:bg-cyan-300" />
            </div>

            {/* ROW 3: Bottom Strip (WorldMap, Metrics, Health) */}
            <div style={{ height: `${rowHeights[2]}%` }} className="min-h-[120px] flex gap-1 overflow-hidden">
              {/* World Model 2D CAD Map */}
              <div style={{ width: `${row3Widths[0]}%` }} className="min-w-[180px] h-full relative group overflow-hidden">
                <button
                  onClick={() => setCurrentPage('world')}
                  className="absolute top-2 right-2 z-20 p-1 bg-black/70 hover:bg-cyan-950 text-slate-400 hover:text-cyan-300 rounded border border-white/10 opacity-0 group-hover:opacity-100 transition-opacity cursor-pointer"
                  title="Expand to Fullscreen 360° World Map"
                >
                  <Maximize2 className="w-3 h-3" />
                </button>
                <WorldMap
                  memoryState={systemState?.memory}
                  visionState={systemState?.vision}
                  jointState={systemState?.joints}
                />
              </div>

              {/* Col Splitter 1-2 in Row 3 */}
              <div
                onPointerDown={(e) => startColResize(3, 0, e)}
                className="w-1.5 h-full bg-[#121622] hover:bg-cyan-500/70 active:bg-cyan-400 cursor-col-resize flex flex-col items-center justify-center transition-colors group z-20"
                title="Drag horizontally to resize World Model / Metrics"
              >
                <div className="h-6 w-0.5 rounded-full bg-slate-600 group-hover:bg-cyan-300" />
              </div>

              {/* Operational Metrics */}
              <div style={{ width: `${row3Widths[1]}%` }} className="min-w-[150px] h-full relative group overflow-hidden">
                <button
                  onClick={() => setCurrentPage('telemetry')}
                  className="absolute top-2 right-2 z-20 p-1 bg-black/70 hover:bg-cyan-950 text-slate-400 hover:text-cyan-300 rounded border border-white/10 opacity-0 group-hover:opacity-100 transition-opacity cursor-pointer"
                  title="Expand to Fullscreen Operational Telemetry"
                >
                  <Maximize2 className="w-3 h-3" />
                </button>
                <MetricsPanel />
              </div>

              {/* Col Splitter 2-3 in Row 3 */}
              <div
                onPointerDown={(e) => startColResize(3, 1, e)}
                className="w-1.5 h-full bg-[#121622] hover:bg-cyan-500/70 active:bg-cyan-400 cursor-col-resize flex flex-col items-center justify-center transition-colors group z-20"
                title="Drag horizontally to resize Metrics / Health"
              >
                <div className="h-6 w-0.5 rounded-full bg-slate-600 group-hover:bg-cyan-300" />
              </div>

              {/* Health & Diagnostic */}
              <div style={{ width: `${row3Widths[2]}%` }} className="min-w-[150px] h-full overflow-hidden">
                {activeBottomTab === 'health' ? (
                  <HealthPanel healthState={systemState?.health} />
                ) : (
                  <ComparisonPanel healthState={systemState?.health} />
                )}
              </div>
            </div>
          </div>
        )}

        {/* ── DEDICATED DEEP-DIVE PAGES ───────────────────────────── */}
        {currentPage === 'perception' && (
          <div className="h-full w-full">
            <CameraPanel
              cameraStreams={cameraStreams}
              visionState={systemState?.vision}
              healthState={systemState?.health}
              taskState={systemState?.task}
              memoryState={systemState?.memory}
              jointState={systemState?.joints}
              servoState={systemState?.servo}
              settings={perceptionSettings}
              onUpdateSettings={updatePerceptionSettings}
            />
          </div>
        )}

        {currentPage === 'twin' && (
          <div className="h-full w-full">
            <ArmVisualiser jointState={systemState?.joints} />
          </div>
        )}

        {currentPage === 'world' && (
          <div className="h-full w-full">
            <WorldMap
              memoryState={systemState?.memory}
              visionState={systemState?.vision}
              jointState={systemState?.joints}
            />
          </div>
        )}

        {currentPage === 'tasks' && (
          <div className="h-full w-full grid grid-cols-12 gap-2">
            <div className="col-span-12 md:col-span-5 h-full">
              <TaskControl
                taskState={systemState?.task}
                onSendCommand={handleSendCommand}
                onApprove={handleApprove}
                onReject={handleReject}
                onEStop={handleEStop}
                onReset={handleReset}
              />
            </div>
            <div className="col-span-12 md:col-span-7 h-full">
              <ChainOfThought logs={systemState?.cot || []} />
            </div>
          </div>
        )}

        {currentPage === 'telemetry' && (
          <div className="h-full w-full grid grid-cols-12 gap-2">
            <div className="col-span-12 md:col-span-6 h-full">
              <MetricsPanel />
            </div>
            <div className="col-span-12 md:col-span-6 h-full">
              <HealthPanel healthState={systemState?.health} />
            </div>
          </div>
        )}

        {currentPage === 'simulation' && (
          <div className="h-full w-full">
            <SimulationControlPanel />
          </div>
        )}
      </main>

      {/* ── PERSISTENT ENGINEERING STATUS STRIP ─────────────────── */}
      <footer className="h-6 px-3 border-t border-[#252a38] bg-[#08090d] flex items-center justify-between text-[10px] font-mono text-slate-400 shrink-0 z-30">
        <div className="flex items-center gap-2 overflow-hidden whitespace-nowrap">
          <span>J1 {jointAngles[0]?.toFixed(1)}°</span>
          <span>·</span>
          <span>J2 {jointAngles[1]?.toFixed(1)}°</span>
          <span>·</span>
          <span>J3 {jointAngles[2]?.toFixed(1)}°</span>
          <span>·</span>
          <span>J4 {jointAngles[3]?.toFixed(1)}°</span>
          <span>·</span>
          <span>J5 {jointAngles[4]?.toFixed(1)}°</span>
          <span>·</span>
          <span>J6 {jointAngles[5]?.toFixed(1)}°</span>
          <span className="text-[#252a38]">│</span>
          {(() => {
            const camStats = systemState?.camera_stats || {};
            const topStats = camStats.top || {};
            const wristStats = camStats.wrist || {};
            const fps = topStats.fps > 0 ? topStats.fps : (wristStats.fps > 0 ? wristStats.fps : 30.0);
            const res = topStats.height ? `${topStats.height}P` : '720P';
            return (
              <>
                <span className="text-cyan-400 font-mono">{fps.toFixed(1)} FPS ({res})</span>
                <span>·</span>
                <span className="text-purple-400">10ms</span>
              </>
            );
          })()}
          <span className="text-[#252a38]">│</span>
          <span>CPU 34%</span>
          <span>·</span>
          <span>GPU 58%</span>
        </div>

        <div className="flex items-center gap-3">
          <div className="flex items-center gap-1">
            <button
              onClick={() => setActiveBottomTab(activeBottomTab === 'health' ? 'benchmark' : 'health')}
              className="text-[9px] text-violet-400 hover:text-violet-300 underline cursor-pointer uppercase"
            >
              [Toggle {activeBottomTab === 'health' ? 'Benchmark' : 'Health'}]
            </button>
          </div>
          <div className="flex items-center gap-2 text-[9px]">
            <span className="flex items-center gap-1 text-emerald-400">● ROS 2</span>
            <span className="flex items-center gap-1 text-emerald-400">● LIVE FEEDS</span>
            <span className="flex items-center gap-1 text-emerald-400">● ARIA ARM</span>
          </div>
        </div>
      </footer>
    </div>
  );
}
