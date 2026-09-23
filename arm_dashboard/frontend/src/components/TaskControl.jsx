import React, { useState } from 'react';
import { Play, CheckCircle2, ArrowRightCircle, Circle, AlertTriangle, ShieldAlert, Sliders, Cpu } from 'lucide-react';
import ManualControl from './ManualControl';

export default function TaskControl({ taskState, onSendCommand, onApprove, onReject, onEStop, onReset }) {
  const [inputCmd, setInputCmd] = useState('');
  const [controlMode, setControlMode] = useState('auto'); // 'auto' | 'manual'
  const [showEstopConfirm, setShowEstopConfirm] = useState(false);

  const goal = taskState?.current_goal || 'Standby — Ready for autonomous task execution';
  const confidence = taskState?.confidence ?? 0.92;
  const subgoals = taskState?.subgoals || [];
  const awaitingApproval = Boolean(taskState?.awaiting_approval);
  const status = taskState?.task_status || 'IDLE';

  const handleSend = (e) => {
    e.preventDefault();
    if (!inputCmd.trim()) return;
    onSendCommand(inputCmd.trim());
    setInputCmd('');
  };

  const getConfidenceColor = (c) => {
    if (c >= 0.8) return 'bg-emerald-500 text-emerald-400';
    if (c >= 0.6) return 'bg-amber-500 text-amber-400';
    return 'bg-red-500 text-red-400';
  };

  return (
    <div className="panel-card h-full flex flex-col bg-[#13161e] border border-[#252a38] rounded-md overflow-hidden select-none">
      {/* Mode Switch Header */}
      <div className="h-9 px-3 border-b border-[#252a38] flex items-center justify-between bg-[#0e1017]">
        <div className="flex items-center gap-2">
          <Cpu className="w-3.5 h-3.5 text-cyan-400" />
          <span className="text-xs font-semibold tracking-wider text-slate-200 uppercase">
            Task Orchestrator & Control
          </span>
        </div>

        {/* Tab Toggle: Auto vs Manual */}
        <div className="flex rounded border border-[#252a38] overflow-hidden text-[10px] font-mono">
          <button
            onClick={() => setControlMode('auto')}
            className={`px-2.5 py-0.5 uppercase transition-colors ${
              controlMode === 'auto' ? 'bg-cyan-500/20 text-cyan-300 font-bold' : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            Autonomous
          </button>
          <button
            onClick={() => setControlMode('manual')}
            className={`px-2.5 py-0.5 uppercase transition-colors ${
              controlMode === 'manual' ? 'bg-purple-500/20 text-purple-300 font-bold' : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            Manual Teleop
          </button>
        </div>
      </div>

      {/* Main Content Area */}
      <div className="flex-1 min-h-0 overflow-y-auto p-2.5 flex flex-col gap-2">
        {controlMode === 'manual' ? (
          <ManualControl isTaskExecuting={status === 'EXECUTING'} />
        ) : (
          <>
            {/* Command Dispatch Field */}
            <form onSubmit={handleSend} className="flex gap-1.5">
              <input
                type="text"
                value={inputCmd}
                onChange={(e) => setInputCmd(e.target.value)}
                placeholder="Enter autonomous command (e.g. 'Pick the yellow banana')..."
                disabled={awaitingApproval}
                className="flex-1 bg-[#0a0c11] border border-[#252a38] rounded px-2.5 py-1.5 text-xs text-slate-100 placeholder-slate-500 focus:outline-none focus:border-cyan-500 font-mono"
              />
              <button
                type="submit"
                disabled={awaitingApproval || !inputCmd.trim()}
                className="px-3 py-1.5 bg-cyan-600 hover:bg-cyan-500 disabled:bg-slate-800 disabled:text-slate-600 text-white rounded text-xs font-semibold flex items-center gap-1 transition-colors"
              >
                <Play className="w-3 h-3 fill-current" />
                SEND
              </button>
            </form>

            {/* Current Mission & Goal Display */}
            <div className="rounded border border-[#252a38] bg-[#0c0e14] p-2">
              <div className="flex justify-between items-center text-[10px] font-mono text-slate-400 mb-1">
                <span>ACTIVE MISSION GOAL</span>
                <span className={`px-1.5 py-0.2 rounded font-bold uppercase ${
                  status === 'EXECUTING' ? 'bg-cyan-950 text-cyan-400 border border-cyan-800' : 'bg-slate-800 text-slate-400'
                }`}>
                  ● {status}
                </span>
              </div>
              <div className="text-xs font-semibold text-slate-100 line-clamp-2">
                {goal}
              </div>
            </div>

            {/* Confidence Metric */}
            <div className="flex flex-col gap-1 rounded border border-[#252a38] bg-[#0c0e14] p-2">
              <div className="flex justify-between items-center text-[10px] font-mono">
                <span className="text-slate-400">PLANNER CONFIDENCE</span>
                <span className="font-bold text-slate-200">{(confidence * 100).toFixed(0)}%</span>
              </div>
              <div className="w-full bg-[#1a1f2c] h-1.5 rounded-full overflow-hidden">
                <div
                  className={`h-full rounded-full transition-all duration-300 ${getConfidenceColor(confidence).split(' ')[0]}`}
                  style={{ width: `${confidence * 100}%` }}
                />
              </div>
              <div className="text-[10px] font-mono text-slate-400 mt-0.5">
                {confidence >= 0.8 ? (
                  <span className="text-emerald-400">✓ Proceeding autonomously</span>
                ) : (
                  <span className="text-amber-400">⚠ Low confidence — Operator oversight recommended</span>
                )}
              </div>
            </div>

            {/* Subgoal Progress Checklist */}
            <div className="flex-1 min-h-[90px] rounded border border-[#252a38] bg-[#0c0e14] p-2 flex flex-col">
              <div className="text-[10px] font-mono text-slate-400 mb-1.5 uppercase tracking-wider">
                Action Decompositions & Subgoals
              </div>
              <div className="flex-1 overflow-y-auto flex flex-col gap-1.5">
                {subgoals.map((sg, i) => (
                  <div key={i} className="flex items-center gap-2 text-xs font-mono">
                    {sg.status === 'completed' ? (
                      <CheckCircle2 className="w-3.5 h-3.5 text-emerald-400 shrink-0" />
                    ) : sg.status === 'active' ? (
                      <ArrowRightCircle className="w-3.5 h-3.5 text-cyan-400 shrink-0 animate-pulse" />
                    ) : (
                      <Circle className="w-3.5 h-3.5 text-slate-600 shrink-0" />
                    )}
                    <span className={sg.status === 'completed' ? 'text-slate-400 line-through' : sg.status === 'active' ? 'text-cyan-300 font-bold' : 'text-slate-500'}>
                      {sg.name}
                    </span>
                  </div>
                ))}
              </div>
            </div>

            {/* Approval Panel (Modal/Banner when awaiting operator review) */}
            {awaitingApproval && (
              <div className="rounded border border-amber-600/80 bg-amber-950/70 p-2.5 flex flex-col gap-2 animate-pulse">
                <div className="flex items-center gap-1.5 text-amber-300 text-xs font-bold">
                  <AlertTriangle className="w-4 h-4 text-amber-400" />
                  SAFETY GATE: OPERATOR APPROVAL REQUIRED
                </div>
                <div className="text-[11px] font-mono text-amber-200">
                  Target grasp trajectory requires operator confirmation prior to actuation.
                </div>
                <div className="grid grid-cols-2 gap-2 mt-1">
                  <button
                    onClick={onApprove}
                    className="py-1.5 bg-emerald-600 hover:bg-emerald-500 text-white rounded text-xs font-bold transition-colors"
                  >
                    ✓ APPROVE
                  </button>
                  <button
                    onClick={onReject}
                    className="py-1.5 bg-rose-600 hover:bg-rose-500 text-white rounded text-xs font-bold transition-colors"
                  >
                    ✗ REJECT
                  </button>
                </div>
              </div>
            )}
          </>
        )}
      </div>

      {/* Persistent Industrial E-STOP Button */}
      <div className="p-2 border-t border-[#252a38] bg-[#0e1017]">
        <button
          onClick={() => setShowEstopConfirm(true)}
          className="w-full py-2 bg-gradient-to-r from-red-700 via-rose-600 to-red-700 hover:from-red-600 hover:to-rose-500 text-white font-bold text-xs rounded border border-red-500/50 shadow-lg shadow-red-950/50 flex items-center justify-center gap-2 tracking-wider transition-all"
        >
          <ShieldAlert className="w-4 h-4 animate-pulse" />
          EMERGENCY STOP (E-STOP)
        </button>
      </div>

      {/* E-STOP Confirmation Modal */}
      {showEstopConfirm && (
        <div className="absolute inset-0 z-50 bg-black/85 flex items-center justify-center p-4">
          <div className="bg-[#181116] border border-red-500/80 rounded-lg p-4 max-w-xs w-full text-center flex flex-col gap-3 shadow-2xl">
            <div className="w-10 h-10 rounded-full bg-red-950 text-red-400 flex items-center justify-center mx-auto border border-red-700">
              <ShieldAlert className="w-6 h-6" />
            </div>
            <div className="text-sm font-bold text-white uppercase tracking-wider">
              Trigger Hardware E-STOP?
            </div>
            <p className="text-[11px] font-mono text-slate-300">
              This will immediately halt all 5 joint actuators and lock the gripper in place.
            </p>
            <div className="grid grid-cols-2 gap-2 mt-1">
              <button
                onClick={() => {
                  onEStop();
                  setShowEstopConfirm(false);
                }}
                className="py-2 bg-red-600 hover:bg-red-500 text-white rounded text-xs font-bold uppercase tracking-wider transition-colors"
              >
                CONFIRM E-STOP
              </button>
              <button
                onClick={() => setShowEstopConfirm(false)}
                className="py-2 bg-slate-800 hover:bg-slate-700 text-slate-300 rounded text-xs font-bold transition-colors"
              >
                CANCEL
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
