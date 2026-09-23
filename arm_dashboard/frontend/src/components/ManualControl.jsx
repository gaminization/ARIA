import React, { useState } from 'react';
import { Send, Sliders, Play, RotateCcw, ShieldCheck } from 'lucide-react';

const JOINTS = [
  { name: 'waist_joint', label: 'J1 Waist', min: -180, max: 180 },
  { name: 'shoulder_joint', label: 'J2 Shoulder', min: -90, max: 90 },
  { name: 'elbow_joint', label: 'J3 Elbow', min: -90, max: 90 },
  { name: 'wrist_pitch_joint', label: 'J4 Wrist Pitch', min: -90, max: 90 },
  { name: 'gripper_joint', label: 'J5 Gripper', min: 0, max: 60 },
];

const PRESETS = {
  Home: [0, 0, 0, 0, 0],
  Ready: [0, 35, -55, 20, 55],
  Folded: [0, -30, 60, -30, 0],
};

export default function ManualControl({ isTaskExecuting = false }) {
  const [angles, setAngles] = useState([0, 35, -55, 20, 55]);
  const [statusMsg, setStatusMsg] = useState('');

  const handleSliderChange = (idx, val) => {
    const next = [...angles];
    next[idx] = parseFloat(val);
    setAngles(next);
  };

  const handleSliderRelease = (idx, val) => {
    const angleVal = parseFloat(val);
    sendJoint(JOINTS[idx].name, angleVal);
  };

  const applyPreset = (name) => {
    if (name === 'Open Gripper') {
      const next = [...angles];
      next[4] = 55;
      setAngles(next);
      sendJoint('gripper_joint', 55);
      return;
    }
    if (name === 'Close Gripper') {
      const next = [...angles];
      next[4] = 2;
      setAngles(next);
      sendJoint('gripper_joint', 2);
      return;
    }
    if (PRESETS[name]) {
      setAngles(PRESETS[name]);
      sendAll(PRESETS[name]);
    }
  };

  const releaseEstop = async () => {
    try {
      const res = await fetch('/api/release_estop', { method: 'POST' });
      const data = await res.json();
      if (data.success) {
        setStatusMsg('E-STOP released — joints active');
      } else {
        setStatusMsg(data.message || 'Error releasing E-STOP');
      }
    } catch (e) {
      setStatusMsg('Error calling /api/release_estop');
    }
  };

  const sendJoint = async (jointName, angle) => {
    try {
      const res = await fetch('/api/joint', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ joint: jointName, angle_deg: angle, speed: 35.0 }),
      });
      if (res.ok) setStatusMsg(`Sent ${jointName} -> ${angle.toFixed(1)}°`);
    } catch (e) {
      setStatusMsg('Error sending joint command');
    }
  };

  const sendAll = async (targetAngles = angles) => {
    try {
      const res = await fetch('/api/joints', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ angles_deg: targetAngles, speed_deg_per_s: 35.0 }),
      });
      if (res.ok) setStatusMsg('All 5 joint commands dispatched');
    } catch (e) {
      setStatusMsg('Error sending joint trajectory');
    }
  };

  return (
    <div className="flex flex-col gap-2 h-full">
      {isTaskExecuting && (
        <div className="bg-amber-950/60 border border-amber-800 text-amber-300 text-[10px] font-mono px-2 py-1 rounded">
          ⚠ Manual teleoperation disabled while autonomous task is executing.
        </div>
      )}

      {/* Release E-Stop bar - ALWAYS ACTIVE to allow emergency overrides */}
      <button
        onClick={releaseEstop}
        className="w-full py-1.5 bg-emerald-950/80 hover:bg-emerald-900 active:bg-emerald-800 border border-emerald-600/70 text-emerald-300 text-[10px] font-mono font-bold rounded flex items-center justify-center gap-1.5 transition-colors cursor-pointer shadow-sm shadow-emerald-950"
      >
        <ShieldCheck className="w-3.5 h-3.5 text-emerald-400" />
        RELEASE E-STOP / UNLOCK ARM
      </button>

      {/* Preset Pose Buttons */}
      <div className="grid grid-cols-3 gap-1">
        {['Home', 'Ready', 'Folded'].map((name) => (
          <button
            key={name}
            onClick={() => applyPreset(name)}
            disabled={isTaskExecuting}
            className="py-1 bg-[#1a1f2c] hover:bg-[#252a38] text-slate-200 rounded text-[10px] font-mono font-semibold border border-[#252a38] transition-colors disabled:opacity-50"
          >
            {name}
          </button>
        ))}
      </div>
      <div className="grid grid-cols-2 gap-1">
        <button
          onClick={() => applyPreset('Open Gripper')}
          disabled={isTaskExecuting}
          className="py-1 bg-cyan-950/60 hover:bg-cyan-900 text-cyan-300 rounded text-[10px] font-mono font-semibold border border-cyan-800/50 transition-colors disabled:opacity-50"
        >
          Open Gripper (55°)
        </button>
        <button
          onClick={() => applyPreset('Close Gripper')}
          disabled={isTaskExecuting}
          className="py-1 bg-purple-950/60 hover:bg-purple-900 text-purple-300 rounded text-[10px] font-mono font-semibold border border-purple-800/50 transition-colors disabled:opacity-50"
        >
          Close Gripper (2°)
        </button>
      </div>

      {/* 5 Real Joint Sliders (Interactive live drag and release) */}
      <div className="flex-1 overflow-y-auto flex flex-col gap-1.5 pr-1">
        {JOINTS.map((j, idx) => (
          <div key={j.name} className="flex flex-col text-[10px] font-mono bg-[#0c0e14] border border-[#252a38] rounded px-2 py-1">
            <div className="flex justify-between items-center text-slate-300">
              <span className="font-semibold">{j.label}</span>
              <span className="text-cyan-400 font-bold">{angles[idx].toFixed(1)}°</span>
            </div>
            <input
              type="range"
              min={j.min}
              max={j.max}
              step={1}
              value={angles[idx]}
              disabled={isTaskExecuting}
              onChange={(e) => handleSliderChange(idx, e.target.value)}
              onPointerUp={(e) => handleSliderRelease(idx, e.target.value)}
              className="w-full h-1 bg-[#1a1f2c] rounded-lg appearance-none cursor-pointer accent-cyan-400 mt-1"
            />
          </div>
        ))}
      </div>

      {/* Send All Command Button */}
      <button
        onClick={() => sendAll(angles)}
        disabled={isTaskExecuting}
        className="w-full py-1.5 bg-purple-600 hover:bg-purple-500 disabled:bg-slate-800 disabled:text-slate-600 text-white font-bold text-xs rounded flex items-center justify-center gap-1 transition-colors"
      >
        <Send className="w-3.5 h-3.5" />
        SEND ALL 5 JOINTS
      </button>

      {statusMsg && (
        <div className="text-[10px] font-mono text-cyan-400 text-center">
          {statusMsg}
        </div>
      )}
    </div>
  );
}
