import React, { useState, useRef, useEffect } from 'react';

export default function TaskControl({ task, onCommand, onApprove, onReject }) {
  const [input, setInput] = useState('');
  const inputRef = useRef(null);

  const handleSubmit = (e) => {
    e.preventDefault();
    if (input.trim()) {
      onCommand(input.trim());
      setInput('');
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
      <h3>🎯 Task Control</h3>

      {/* Command Input */}
      <form className="command-form" onSubmit={handleSubmit}>
        <label>Command:</label>
        <div className="input-row">
          <input
            ref={inputRef}
            type="text"
            value={input}
            onChange={(e) => setInput(e.target.value)}
            placeholder="e.g. Pick up the red cube"
            className="command-input"
            disabled={status === 'EXECUTING' || status === 'PLANNING'}
          />
          <button type="submit" className="send-btn"
            disabled={status === 'EXECUTING' || status === 'PLANNING'}>
            Send
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
