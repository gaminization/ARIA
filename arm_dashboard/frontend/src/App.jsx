import React, { useState, useEffect, useRef, useCallback } from 'react';
import CameraPanel from './components/CameraPanel';
import ArmVisualization from './components/ArmVisualization';
import TaskControl from './components/TaskControl';
import ChainOfThought from './components/ChainOfThought';
import WorldMap from './components/WorldMap';
import HealthPanel from './components/HealthPanel';
import JointSliders from './components/JointSliders';
import './App.css';

const WS_URL = `ws://${window.location.hostname}:8080`;
const API_URL = `http://${window.location.hostname}:8080`;

function App() {
  const [state, setState] = useState(null);
  const [cameraFrames, setCameraFrames] = useState({ top: null, wrist: null });
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

  // ── WebSocket: Cameras ──────────────────────────────────
  useEffect(() => {
    const connect = () => {
      const ws = new WebSocket(`${WS_URL}/ws/cameras`);
      ws.onmessage = (e) => {
        try { setCameraFrames(JSON.parse(e.data)); } catch {}
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

  return (
    <div className="app">
      {/* Header */}
      <header className="app-header">
        <div className="header-left">
          <span className="logo">🤖</span>
          <h1>ARIA CONTROL CENTER</h1>
        </div>
        <div className="header-center">
          <div className="tab-bar">
            <button
              className={`tab ${activeTab === 'main' ? 'active' : ''}`}
              onClick={() => setActiveTab('main')}>
              Dashboard
            </button>
            <button
              className={`tab ${activeTab === 'joints' ? 'active' : ''}`}
              onClick={() => setActiveTab('joints')}>
              Joint Control
            </button>
          </div>
        </div>
        <div className="header-right">
          <span className={`status-dot ${connected ? 'connected' : 'disconnected'}`} />
          <span className="status-text">
            {connected ? 'Connected' : 'Disconnected'}
          </span>
          <button className="estop-btn" onClick={estop}>
            E-STOP 🔴
          </button>
        </div>
      </header>

      {activeTab === 'main' ? (
        <main className="dashboard-grid">
          {/* Row 1: Cameras | 3D Arm | Task Control */}
          <section className="panel cameras-panel">
            <CameraPanel
              topFrame={cameraFrames.top}
              wristFrame={cameraFrames.wrist}
              detections={vision.detected_objects || []}
            />
          </section>

          <section className="panel arm-panel">
            <ArmVisualization joints={joints} />
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

          {/* Row 3: World Map | Health */}
          <section className="panel world-panel">
            <WorldMap
              objects={memory.known_objects || []}
              relations={memory.spatial_relations || []}
            />
          </section>

          <section className="panel health-panel">
            <HealthPanel health={health} />
          </section>
        </main>
      ) : (
        <main className="joints-page">
          <JointSliders joints={joints} onSetJoint={setJoint} />
        </main>
      )}
    </div>
  );
}

export default App;
