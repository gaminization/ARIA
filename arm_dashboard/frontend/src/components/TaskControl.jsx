import React, { useState, useRef, useEffect } from 'react';

export default function TaskControl({ task, onCommand, onApprove, onReject, onReset }) {
  const [input, setInput] = useState('');
  const [feedback, setFeedback] = useState(null);
  const [loading, setLoading] = useState(false);
  const inputRef = useRef(null);

  const presets = [
    { label: '🍌 Pick Banana', cmd: 'pick up the banana' },
    { label: '☕ Pick Red Mug', cmd: 'pick up the red mug' },
    { label: '🦆 Pick Duck', cmd: 'pick up the duck' },
    { label: '🍾 Pick Bottle', cmd: 'pick up the bottle' },
    { label: '🔍 Active Scan', cmd: 'locate all objects on the table' },
    { label: '🔄 Home Arm', cmd: 'reset arm to home' },
  ];

  const handleSubmit = async (e) => {
    e.preventDefault();
    const cmd = input.trim();
    if (!cmd) return;
    setLoading(true);
    setFeedback({ type: 'info', msg: `Sending command: "${cmd}"...` });
    try {
      const res = await onCommand(cmd);
      if (res && res.success) {
        setFeedback({ type: 'success', msg: `✓ Accepted: "${cmd}"` });
        setInput('');
      } else {
        setFeedback({ type: 'warn', msg: res?.message || 'Command rejected or system busy' });
      }
    } catch (err) {
      setFeedback({ type: 'error', msg: `Failed to send: ${err.message}` });
    } finally {
      setLoading(false);
    }
  };

  const handlePreset = (cmd) => {
    setInput(cmd);
    if (inputRef.current) inputRef.current.focus();
  };

  const handleReset = async () => {
    setLoading(true);
    setFeedback({ type: 'info', msg: 'Aborting active task...' });
    try {
      if (onReset) await onReset();
      setFeedback({ type: 'success', msg: '✓ Task reset to IDLE' });
    } catch (err) {
      setFeedback({ type: 'error', msg: 'Reset failed' });
    } finally {
      setLoading(false);
    }
  };

  const statusColors = {
    IDLE: '#6366f1',
    PLANNING: '#f59e0b',
    EXECUTING: '#22c55e',
    PAUSED: '#ef4444',
    RECOVERY: '#f97316',
    COMPLETE: '#10b981',
    FAILED: '#ef4444',
    ESTOP: '#dc2626',
  };

  const statusEmoji = {
    IDLE: '⏸',
    PLANNING: '🤔',
    EXECUTING: '🚀',
    PAUSED: '⏯',
    RECOVERY: '🔧',
    COMPLETE: '✅',
    FAILED: '❌',
    ESTOP: '🛑',
  };

  const status = task.status || 'IDLE';
  const confidence = task.confidence || 0;
  const confPct = Math.round(confidence * 100);

  return (
    <div className="task-control">
      <div className="task-control-header">
        <h3>🎯 Task Control</h3>
        <button
          type="button"
          className="reset-task-btn"
          onClick={handleReset}
          title="Abort current task and reset system to IDLE"
        >
          🔄 Reset Task
        </button>
      </div>

      {/* Feedback Banner */}
      {feedback && (
        <div className={`feedback-banner ${feedback.type}`}>
          <span>{feedback.msg}</span>
          <button className="feedback-close" onClick={() => setFeedback(null)}>×</button>
        </div>
      )}

      {/* Quick Action Presets */}
      <div className="command-presets">
        <span className="presets-label">Quick Commands:</span>
        <div className="preset-chips">
          {presets.map((p, idx) => (
            <button
              key={idx}
              type="button"
              className="preset-chip"
              onClick={() => handlePreset(p.cmd)}
              disabled={loading}
            >
              {p.label}
            </button>
          ))}
        </div>
      </div>

      {/* Command Input */}
      <form className="command-form" onSubmit={handleSubmit}>
        <label>Natural Language Command:</label>
        <div className="input-row">
          <input
            ref={inputRef}
            type="text"
            value={input}
            onChange={(e) => setInput(e.target.value)}
            placeholder="Type any command (e.g. pick up the banana)..."
            className="command-input"
            disabled={loading}
          />
          <button type="submit" className="send-btn" disabled={loading || !input.trim()}>
            {loading ? 'Sending...' : 'Send'}
          </button>
        </div>
      </form>

      {/* Current Goal */}
      {task.goal && (
        <div className="info-row">
          <span className="info-label">Goal:</span>
          <span className="info-value">{task.goal}</span>
        </div>
      )}

      {/* Confidence Bar */}
      <div className="confidence-row">
        <span className="info-label">Confidence:</span>
        <div className="confidence-bar">
          <div
            className="confidence-fill"
            style={{
              width: `${confPct}%`,
              backgroundColor: confPct > 75 ? '#22c55e' :
                confPct > 50 ? '#f59e0b' : '#ef4444',
            }}
          />
        </div>
        <span className="confidence-value">{confPct}%</span>
      </div>

      {/* Status Badge */}
      <div className="status-row">
        <span className="info-label">Status:</span>
        <span
          className="status-badge"
          style={{ backgroundColor: statusColors[status] || '#666' }}
        >
          {statusEmoji[status] || ''} {status}
        </span>
      </div>

      {/* Subgoals */}
      {task.subgoals && task.subgoals.length > 0 && (
        <div className="subgoals">
          <span className="info-label">Subgoals:</span>
          <ul className="subgoal-list">
            {task.subgoals.map((sg, i) => {
              const action = task.action_queue?.[i];
              const actionStatus = action?.status || 'PENDING';
              const icon = actionStatus === 'COMPLETE' ? '✓' :
                actionStatus === 'EXECUTING' ? '→' :
                actionStatus === 'FAILED' ? '✗' : '○';
              return (
                <li key={i} className={`subgoal-item ${actionStatus.toLowerCase()}`}>
                  <span className="subgoal-icon">{icon}</span>
                  {sg}
                </li>
              );
            })}
          </ul>
        </div>
      )}

      {/* Approval Buttons */}
      {task.awaiting_approval && (
        <div className="approval-buttons">
          <p className="approval-prompt">
            ⚠ Low confidence — your approval is needed
          </p>
          <button className="approve-btn" onClick={onApprove}>
            ✅ APPROVE
          </button>
          <button className="reject-btn" onClick={onReject}>
            ❌ REJECT
          </button>
        </div>
      )}
    </div>
  );
}
