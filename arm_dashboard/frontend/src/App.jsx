import React, { useState, useEffect, useRef, useCallback } from 'react';
import CameraPanel from './components/CameraPanel';
import ArmVisualization from './components/ArmVisualization';
import TaskControl from './components/TaskControl';
import ChainOfThought from './components/ChainOfThought';
import WorldMap from './components/WorldMap';
import HealthPanel from './components/HealthPanel';
import JointSliders from './components/JointSliders';
import AIModelsPanel from './components/AIModelsPanel';
import CVPipelinePanel from './components/CVPipelinePanel';
import './App.css';

const WS_URL = `ws://${window.location.hostname}:8080`;
const API_URL = `http://${window.location.hostname}:8080`;

function App() {
  const [state, setState] = useState(null);
  // GRIPPER CAMERA ONLY — top camera intentionally not displayed
  const [cameraFrames, setCameraFrames] = useState({ wrist: null });
  const [connected, setConnected] = useState(false);
  const [activeTab, setActiveTab] = useState('main');
  const wsRef = useRef(null);
  const camWsRef = useRef(null);

  // ── WebSocket: State ────────────────────────────────────
  useEffect(() => {
    const connect = () => {
      const ws = new WebSocket(`${WS_URL}/ws/state`);
      ws.onopen = () => setConnected(true);
      ws.onclose = () => {
        setConnected(false);
        setTimeout(connect, 2000);
      };
      ws.onmessage = (e) => {
        try { setState(JSON.parse(e.data)); } catch {}
      };
      wsRef.current = ws;
    };
    connect();
    return () => wsRef.current?.close();
  }, []);

  // ── WebSocket: Cameras (GRIPPER ONLY — wrist_camera) ──
  useEffect(() => {
    const connect = () => {
      const ws = new WebSocket(`${WS_URL}/ws/cameras`);
      ws.onmessage = (e) => {
        try {
          const frames = JSON.parse(e.data);
          // Only use wrist (gripper) camera — top camera ignored
          setCameraFrames({ wrist: frames.wrist || null });
        } catch {}
      };
      ws.onclose = () => setTimeout(connect, 2000);
      camWsRef.current = ws;
    };
    connect();
    return () => camWsRef.current?.close();
  }, []);

  // ── API calls ───────────────────────────────────────────
  const sendCommand = useCallback(async (cmd) => {
    await fetch(`${API_URL}/api/command`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command: cmd }),
    });
  }, []);

  const approve = useCallback(async () => {
    await fetch(`${API_URL}/api/approve`, { method: 'POST' });
  }, []);

  const reject = useCallback(async () => {
    await fetch(`${API_URL}/api/reject`, { method: 'POST' });
  }, []);

  const estop = useCallback(async () => {
    await fetch(`${API_URL}/api/estop`, { method: 'POST' });
  }, []);

  const setJoint = useCallback(async (name, deg) => {
    await fetch(`${API_URL}/api/joint`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ joint_name: name, angle_deg: deg }),
    });
  }, []);

  const joints = state?.joints || { names: [], positions_deg: [], positions_rad: [] };
  const task = state?.task || {};
  const health = state?.health || {};
  const vision = state?.vision || {};
  const memory = state?.memory || {};
  const cot = state?.cot || [];

  const tabs = [
    { id: 'main', label: '🖥 Dashboard' },
    { id: 'ai_models', label: '🤖 AI Models' },
    { id: 'cv_pipeline', label: '🔬 CV Pipeline' },
    { id: 'joints', label: '🦾 Joint Control' },
  ];

  return (
    <div className="app">
      {/* Header */}
      <header className="app-header">
        <div className="header-left">
          <span className="logo">🤖</span>
          <div className="header-brand">
            <h1>ARIA CONTROL CENTER</h1>
            <span className="header-subtitle">Industrial Workcell · Gripper Camera Mode</span>
          </div>
        </div>
        <div className="header-center">
          <div className="tab-bar">
            {tabs.map(tab => (
              <button
                key={tab.id}
                id={`tab-${tab.id}`}
                className={`tab ${activeTab === tab.id ? 'active' : ''}`}
                onClick={() => setActiveTab(tab.id)}
              >
                {tab.label}
              </button>
            ))}
          </div>
        </div>
        <div className="header-right">
          <div className="status-group">
            <span className={`status-dot ${connected ? 'connected' : 'disconnected'}`} />
            <span className="status-text">
              {connected ? 'ROS2 Connected' : 'Disconnected'}
            </span>
          </div>
          <div className="header-metrics">
            <span className="hm-item">
              <span className="hm-val">{health.fps_wrist?.toFixed(1) || '—'}</span>
              <span className="hm-label">wrist fps</span>
            </span>
            <span className="hm-item">
              <span className="hm-val">{health.inference_ms?.toFixed(0) || '—'}</span>
              <span className="hm-label">ms</span>
            </span>
            <span className="hm-item">
              <span className="hm-val">{(vision.detected_objects || []).length}</span>
              <span className="hm-label">obj</span>
            </span>
          </div>
          <button id="estop-btn" className="estop-btn" onClick={estop}>
            ⛔ E-STOP
          </button>
        </div>
      </header>

      {/* ── Dashboard Tab ──────────────────────────────── */}
      {activeTab === 'main' && (
        <main className="dashboard-grid">
          {/* Row 1: Gripper Camera (large) | Task Control */}
          <section className="panel cameras-panel">
            <CameraPanel
              topFrame={null}            /* Top camera disabled — gripper only */
              wristFrame={cameraFrames.wrist}
              detections={vision.detected_objects || []}
              health={health}
            />
          </section>

          <section className="panel task-panel">
            <TaskControl
              task={task}
              onCommand={sendCommand}
              onApprove={approve}
              onReject={reject}
            />
          </section>

          {/* Row 2: Chain of Thought */}
          <section className="panel cot-panel">
            <ChainOfThought entries={cot} />
          </section>

          {/* Row 3: World Map | Arm Viz | Health */}
          <section className="panel world-panel">
            <WorldMap
              objects={memory.known_objects || []}
              relations={memory.spatial_relations || []}
            />
          </section>

          <section className="panel arm-panel">
            <ArmVisualization joints={joints} />
          </section>

          <section className="panel health-panel">
            <HealthPanel health={health} />
          </section>
        </main>
      )}

      {/* ── AI Models Tab ──────────────────────────────── */}
      {activeTab === 'ai_models' && (
        <main className="full-tab">
          <AIModelsPanel health={health} vision={vision} />
        </main>
      )}

      {/* ── CV Pipeline Tab ────────────────────────────── */}
      {activeTab === 'cv_pipeline' && (
        <main className="full-tab cv-tab-layout">
          {/* Live gripper feed alongside pipeline steps */}
          <div className="cv-tab-camera">
            <CameraPanel
              topFrame={null}
              wristFrame={cameraFrames.wrist}
              detections={vision.detected_objects || []}
              health={health}
            />
          </div>
          <div className="cv-tab-steps">
            <CVPipelinePanel vision={vision} health={health} memory={memory} />
          </div>
        </main>
      )}

      {/* ── Joint Control Tab ──────────────────────────── */}
      {activeTab === 'joints' && (
        <main className="joints-page">
          <JointSliders joints={joints} onSetJoint={setJoint} />
        </main>
      )}
    </div>
  );
}

export default App;
