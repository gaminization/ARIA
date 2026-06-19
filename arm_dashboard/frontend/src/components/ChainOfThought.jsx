import React, { useRef, useEffect } from 'react';

export default function ChainOfThought({ entries }) {
  const scrollRef = useRef(null);

  useEffect(() => {
    if (scrollRef.current) {
      scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
    }
  }, [entries]);

  const formatTime = (ts) => {
    if (!ts) return '';
    const d = new Date(ts * 1000);
    return d.toLocaleTimeString('en-US', {
      hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit',
    });
  };

  const getEntryStyle = (text) => {
    if (!text) return {};
    if (text.includes('FAILED') || text.includes('✗') || text.includes('❌'))
      return { color: '#ef4444' };
    if (text.includes('SUCCESS') || text.includes('✓') || text.includes('✅') || text.includes('COMPLETE'))
      return { color: '#22c55e' };
    if (text.includes('⚠') || text.includes('WARNING') || text.includes('PAUSED'))
      return { color: '#f59e0b' };
    if (text.includes('[USER]'))
      return { color: '#818cf8' };
    if (text.includes('EXECUTING') || text.includes('→'))
      return { color: '#38bdf8' };
    if (text.includes('═══'))
      return { color: '#a78bfa', fontWeight: 'bold' };
    return { color: '#c0c0e0' };
  };

  return (
    <div className="cot-container">
      <h3>🧠 Chain of Thought</h3>
      <div className="cot-scroll" ref={scrollRef}>
        {entries.length === 0 ? (
          <div className="cot-empty">Waiting for activity...</div>
        ) : (
          entries.map((entry, i) => {
            const text = typeof entry === 'string' ? entry : entry.text || '';
            const ts = typeof entry === 'object' ? entry.time : null;
            return (
              <div key={i} className="cot-entry" style={getEntryStyle(text)}>
                {ts && <span className="cot-time">{formatTime(ts)}</span>}
                <span className="cot-text">{text}</span>
              </div>
            );
          })
        )}
      </div>
    </div>
  );
}
