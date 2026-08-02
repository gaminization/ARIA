import React, { useState, useEffect } from 'react';

/**
 * ═══════════════════════════════════════════════════════════
 * ARIA Bag Recorder Panel
 * Status bar component showing recording state, controls,
 * and a bag search popup.
 * ═══════════════════════════════════════════════════════════
 */

function formatDuration(seconds) {
  const h = Math.floor(seconds / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  const s = Math.floor(seconds % 60);
  return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
}

function formatSize(mb) {
  if (mb >= 1024) return `${(mb / 1024).toFixed(1)} GB`;
  return `${mb.toFixed(1)} MB`;
}

export default function BagRecorderPanel({ wsUrl }) {
  const [status, setStatus] = useState({
    recording: false,
    mode: 'standard',
    duration_s: 0,
    size_mb: 0,
    tasks: 0,
    failures: 0,
  });
  const [showSearch, setShowSearch] = useState(false);
  const [searchResults, setSearchResults] = useState([]);
  const [searchQuery, setSearchQuery] = useState({
    object: '',
    failureOnly: false,
    dateFrom: '',
  });

  // Subscribe to recording status
  useEffect(() => {
    const fetchStatus = async () => {
      try {
        const res = await fetch(`${wsUrl || ''}/api/bag/status`);
        if (res.ok) {
          setStatus(await res.json());
        }
      } catch {
        // WebSocket fallback or demo mode
      }
    };
    const interval = setInterval(fetchStatus, 2000);
    return () => clearInterval(interval);
  }, [wsUrl]);

  const handleModeSwitch = async () => {
    const newMode = status.mode === 'standard' ? 'full' : 'standard';
    try {
      await fetch(`${wsUrl || ''}/api/bag/set_mode`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: newMode }),
      });
    } catch {
      setStatus(prev => ({ ...prev, mode: newMode }));
    }
  };

  const handleToggleRecording = async () => {
    const endpoint = status.recording ? 'stop' : 'start';
    try {
      await fetch(`${wsUrl || ''}/api/bag/${endpoint}`, { method: 'POST' });
    } catch {
      setStatus(prev => ({ ...prev, recording: !prev.recording }));
    }
  };

  const handleSearch = async () => {
    try {
      const params = new URLSearchParams();
      if (searchQuery.object) params.set('object', searchQuery.object);
      if (searchQuery.failureOnly) params.set('failure_only', 'true');
      if (searchQuery.dateFrom) params.set('date_from', searchQuery.dateFrom);

      const res = await fetch(`${wsUrl || ''}/api/bag/search?${params}`);
      if (res.ok) {
        const data = await res.json();
        setSearchResults(data.bags || []);
      }
    } catch {
      setSearchResults(DEMO_BAGS);
    }
  };

  const modeColor = status.mode === 'full' ? '#FF9800'
    : status.mode === 'investigation' ? '#F44336' : '#4CAF50';

  return (
    <>
      {/* Status Bar */}
      <div style={{
        display: 'flex',
        alignItems: 'center',
        gap: 16,
        background: 'rgba(20,20,40,0.9)',
        borderRadius: 10,
        padding: '10px 18px',
        fontFamily: "'Inter', sans-serif",
        fontSize: '0.88em',
        color: '#E0E0E0',
        border: '1px solid rgba(255,255,255,0.06)',
      }}>
        {/* Recording indicator */}
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <div style={{
            width: 10, height: 10, borderRadius: '50%',
            background: status.recording ? '#F44336' : '#616161',
            animation: status.recording ? 'pulse 1.5s infinite' : 'none',
            boxShadow: status.recording ? '0 0 8px rgba(244,67,54,0.6)' : 'none',
          }} />
          <span style={{ fontWeight: 600 }}>
            {status.recording ? 'Recording' : 'Stopped'}
          </span>
        </div>

        {/* Mode badge */}
        <span style={{
          background: `${modeColor}20`,
          color: modeColor,
          padding: '2px 10px',
          borderRadius: 8,
          fontSize: '0.85em',
          fontWeight: 600,
          textTransform: 'uppercase',
          letterSpacing: '0.04em',
        }}>
          {status.mode}
        </span>

        {/* Size + Duration */}
        {status.recording && (
          <>
            <span style={{ color: '#78909C' }}>|</span>
            <span style={{ fontFamily: 'monospace' }}>
              {formatSize(status.size_mb)}
            </span>
            <span style={{ fontFamily: 'monospace' }}>
              {formatDuration(status.duration_s)}
            </span>
            <span style={{ color: '#78909C' }}>
              {status.tasks} task{status.tasks !== 1 ? 's' : ''}
              {status.failures > 0 && (
                <span style={{ color: '#F44336', marginLeft: 4 }}>
                  ({status.failures} fail)
                </span>
              )}
            </span>
          </>
        )}

        {/* Spacer */}
        <div style={{ flex: 1 }} />

        {/* Controls */}
        <button
          onClick={handleToggleRecording}
          style={{
            background: status.recording
              ? 'rgba(244,67,54,0.15)' : 'rgba(76,175,80,0.15)',
            color: status.recording ? '#F44336' : '#4CAF50',
            border: `1px solid ${status.recording ? 'rgba(244,67,54,0.3)' : 'rgba(76,175,80,0.3)'}`,
            borderRadius: 8, padding: '4px 14px',
            cursor: 'pointer', fontSize: '0.9em',
            transition: 'all 0.2s',
          }}
        >
          {status.recording ? '⏹ Stop' : '⏺ Start'}
        </button>

        {status.recording && (
          <button
            onClick={handleModeSwitch}
            style={{
              background: 'rgba(255,255,255,0.05)',
              color: '#B0BEC5',
              border: '1px solid rgba(255,255,255,0.1)',
              borderRadius: 8, padding: '4px 14px',
              cursor: 'pointer', fontSize: '0.9em',
              transition: 'all 0.2s',
            }}
          >
            {status.mode === 'standard' ? '↑ FULL' : '↓ STD'}
          </button>
        )}

        <button
          onClick={() => { setShowSearch(!showSearch); if (!showSearch) handleSearch(); }}
          style={{
            background: showSearch
              ? 'rgba(100,181,246,0.15)' : 'rgba(255,255,255,0.05)',
            color: showSearch ? '#64B5F6' : '#B0BEC5',
            border: `1px solid ${showSearch ? 'rgba(100,181,246,0.3)' : 'rgba(255,255,255,0.1)'}`,
            borderRadius: 8, padding: '4px 14px',
            cursor: 'pointer', fontSize: '0.9em',
            transition: 'all 0.2s',
          }}
        >
          🔍 Bags
        </button>
      </div>

      {/* Search Panel */}
      {showSearch && (
        <div style={{
          background: 'rgba(20,20,40,0.95)',
          borderRadius: 10,
          padding: 20,
          marginTop: 8,
          border: '1px solid rgba(255,255,255,0.06)',
          fontFamily: "'Inter', sans-serif",
          color: '#E0E0E0',
        }}>
          <div style={{ display: 'flex', gap: 10, marginBottom: 16 }}>
            <input
              type="text"
              placeholder="Object class..."
              value={searchQuery.object}
              onChange={e => setSearchQuery(prev => ({ ...prev, object: e.target.value }))}
              style={{
                background: 'rgba(255,255,255,0.06)',
                border: '1px solid rgba(255,255,255,0.1)',
                borderRadius: 8, padding: '6px 12px',
                color: '#E0E0E0', outline: 'none', flex: 1,
              }}
            />
            <input
              type="date"
              value={searchQuery.dateFrom}
              onChange={e => setSearchQuery(prev => ({ ...prev, dateFrom: e.target.value }))}
              style={{
                background: 'rgba(255,255,255,0.06)',
                border: '1px solid rgba(255,255,255,0.1)',
                borderRadius: 8, padding: '6px 12px',
                color: '#E0E0E0', outline: 'none',
              }}
            />
            <label style={{
              display: 'flex', alignItems: 'center', gap: 6,
              fontSize: '0.85em', color: '#78909C',
            }}>
              <input
                type="checkbox"
                checked={searchQuery.failureOnly}
                onChange={e => setSearchQuery(prev => ({
                  ...prev, failureOnly: e.target.checked
                }))}
              />
              Failures only
            </label>
            <button
              onClick={handleSearch}
              style={{
                background: 'rgba(100,181,246,0.15)',
                color: '#64B5F6',
                border: '1px solid rgba(100,181,246,0.3)',
                borderRadius: 8, padding: '6px 16px',
                cursor: 'pointer',
              }}
            >
              Search
            </button>
          </div>

          {searchResults.length > 0 ? (
            <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.85em' }}>
              <thead>
                <tr style={{
                  borderBottom: '2px solid rgba(255,255,255,0.08)',
                  color: '#78909C',
                }}>
                  <th style={{ padding: 6, textAlign: 'left' }}>Date</th>
                  <th style={{ padding: 6, textAlign: 'left' }}>Mode</th>
                  <th style={{ padding: 6, textAlign: 'center' }}>Tasks</th>
                  <th style={{ padding: 6, textAlign: 'center' }}>Failures</th>
                  <th style={{ padding: 6, textAlign: 'center' }}>Size</th>
                  <th style={{ padding: 6, textAlign: 'left' }}>Objects</th>
                </tr>
              </thead>
              <tbody>
                {searchResults.map((bag, i) => (
                  <tr key={i} style={{
                    borderBottom: '1px solid rgba(255,255,255,0.04)',
                    cursor: 'pointer',
                  }}>
                    <td style={{ padding: 6 }}>{bag.start_time?.slice(0, 10)}</td>
                    <td style={{ padding: 6 }}>{bag.recording_mode}</td>
                    <td style={{ padding: 6, textAlign: 'center' }}>{bag.task_count}</td>
                    <td style={{
                      padding: 6, textAlign: 'center',
                      color: bag.failure_count > 0 ? '#F44336' : '#4CAF50',
                    }}>
                      {bag.failure_count}
                    </td>
                    <td style={{ padding: 6, textAlign: 'center' }}>
                      {formatSize(bag.size_mb || 0)}
                    </td>
                    <td style={{ padding: 6, fontSize: '0.9em', color: '#78909C' }}>
                      {bag.objects?.join(', ') || '—'}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          ) : (
            <div style={{ textAlign: 'center', padding: 20, color: '#616161' }}>
              No bags found. Start recording to build history.
            </div>
          )}
        </div>
      )}

      <style>{`
        @keyframes pulse {
          0%, 100% { opacity: 1; }
          50% { opacity: 0.4; }
        }
      `}</style>
    </>
  );
}

const DEMO_BAGS = [
  { start_time: '2026-07-06T14:30:22', recording_mode: 'standard',
    task_count: 12, failure_count: 1, size_mb: 45.2,
    objects: ['red_cube', 'blue_cup'] },
  { start_time: '2026-07-05T10:15:00', recording_mode: 'full',
    task_count: 8, failure_count: 0, size_mb: 320.5,
    objects: ['red_bottle', 'screwdriver'] },
  { start_time: '2026-07-05T09:22:11', recording_mode: 'investigation',
    task_count: 1, failure_count: 1, size_mb: 180.0,
    objects: ['blue_cup'] },
];
