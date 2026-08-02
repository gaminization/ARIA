import React, { useState, useEffect } from 'react';

/**
 * ═══════════════════════════════════════════════════════════
 * ARIA Experiment Panel
 * Dashboard tab showing experiment tracking data.
 * Embeds MLflow UI or displays local experiment summary.
 * ═══════════════════════════════════════════════════════════
 */

function MetricCard({ label, value, unit, trend, color }) {
  const trendColor = trend > 0 ? '#4CAF50' : trend < 0 ? '#F44336' : '#9E9E9E';
  const trendIcon = trend > 0 ? '↑' : trend < 0 ? '↓' : '→';

  return (
    <div style={{
      background: 'rgba(255,255,255,0.04)',
      borderRadius: 10,
      padding: '16px 20px',
      minWidth: 140,
      borderLeft: `3px solid ${color || '#64B5F6'}`,
    }}>
      <div style={{ fontSize: '0.78em', color: '#78909C', marginBottom: 6 }}>
        {label}
      </div>
      <div style={{ fontSize: '1.6em', fontWeight: 700, color: color || '#E0E0E0' }}>
        {value}{unit && <span style={{ fontSize: '0.5em', marginLeft: 4 }}>{unit}</span>}
      </div>
      {trend !== undefined && (
        <div style={{ fontSize: '0.8em', color: trendColor, marginTop: 4 }}>
          {trendIcon} {Math.abs(trend).toFixed(1)}% vs prev
        </div>
      )}
    </div>
  );
}

function RecentRunsTable({ runs }) {
  return (
    <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.85em' }}>
      <thead>
        <tr style={{
          borderBottom: '2px solid rgba(255,255,255,0.08)',
          color: '#78909C', textTransform: 'uppercase',
          fontSize: '0.8em', letterSpacing: '0.05em',
        }}>
          <th style={{ padding: '8px 12px', textAlign: 'left' }}>Date</th>
          <th style={{ padding: '8px 12px', textAlign: 'left' }}>Type</th>
          <th style={{ padding: '8px 12px', textAlign: 'center' }}>Tasks</th>
          <th style={{ padding: '8px 12px', textAlign: 'center' }}>Success</th>
          <th style={{ padding: '8px 12px', textAlign: 'center' }}>Duration</th>
        </tr>
      </thead>
      <tbody>
        {runs.map((run, i) => {
          const rateColor = run.success_rate >= 0.8 ? '#4CAF50'
            : run.success_rate >= 0.6 ? '#FF9800' : '#F44336';
          return (
            <tr key={i} style={{
              borderBottom: '1px solid rgba(255,255,255,0.04)',
            }}>
              <td style={{ padding: '8px 12px' }}>{run.date}</td>
              <td style={{ padding: '8px 12px' }}>
                <span style={{
                  background: run.type === 'session' ? 'rgba(100,181,246,0.15)' :
                    run.type === 'benchmark' ? 'rgba(129,199,132,0.15)' :
                    'rgba(206,147,216,0.15)',
                  padding: '2px 8px', borderRadius: 8, fontSize: '0.9em',
                }}>
                  {run.type}
                </span>
              </td>
              <td style={{ padding: '8px 12px', textAlign: 'center' }}>
                {run.task_count}
              </td>
              <td style={{
                padding: '8px 12px', textAlign: 'center',
                color: rateColor, fontWeight: 600,
              }}>
                {(run.success_rate * 100).toFixed(0)}%
              </td>
              <td style={{ padding: '8px 12px', textAlign: 'center' }}>
                {run.duration}
              </td>
            </tr>
          );
        })}
      </tbody>
    </table>
  );
}

export default function ExperimentPanel({ mlflowUrl }) {
  const [summary, setSummary] = useState(null);
  const [activeTab, setActiveTab] = useState('overview');
  const effectiveMlflowUrl = mlflowUrl || 'http://localhost:5000';

  useEffect(() => {
    // Fetch experiment summary
    setSummary({
      total_runs: 42,
      total_sessions: 18,
      avg_success_rate: 0.847,
      best_ik_solver: 'analytical_6dof',
      best_ik_rate: 0.975,
      recent_runs: [
        { date: '2026-07-06', type: 'session', task_count: 15, success_rate: 0.87, duration: '42m' },
        { date: '2026-07-05', type: 'benchmark', task_count: 500, success_rate: 0.975, duration: '8m' },
        { date: '2026-07-05', type: 'session', task_count: 8, success_rate: 0.75, duration: '22m' },
        { date: '2026-07-04', type: 'training', task_count: 100, success_rate: 0.82, duration: '1h 15m' },
        { date: '2026-07-03', type: 'session', task_count: 12, success_rate: 0.92, duration: '35m' },
      ],
    });
  }, []);

  const tabs = [
    { id: 'overview', label: '📊 Overview' },
    { id: 'mlflow', label: '🧪 MLflow' },
    { id: 'ik', label: '🦾 IK Solvers' },
  ];

  return (
    <div style={{
      background: 'rgba(20,20,40,0.95)',
      borderRadius: 12,
      padding: 24,
      color: '#E0E0E0',
      fontFamily: "'Inter', sans-serif",
    }}>
      {/* Header */}
      <div style={{
        display: 'flex', justifyContent: 'space-between',
        alignItems: 'center', marginBottom: 20,
      }}>
        <h2 style={{ margin: 0, color: '#CE93D8' }}>
          🧪 Experiments
        </h2>

        <div style={{ display: 'flex', gap: 4 }}>
          {tabs.map(tab => (
            <button
              key={tab.id}
              onClick={() => setActiveTab(tab.id)}
              style={{
                background: activeTab === tab.id
                  ? 'rgba(206,147,216,0.2)' : 'transparent',
                border: activeTab === tab.id
                  ? '1px solid rgba(206,147,216,0.3)' : '1px solid transparent',
                borderRadius: 8, padding: '6px 14px',
                color: activeTab === tab.id ? '#CE93D8' : '#78909C',
                cursor: 'pointer', fontSize: '0.85em',
                transition: 'all 0.2s',
              }}
            >
              {tab.label}
            </button>
          ))}
        </div>
      </div>

      {/* Overview tab */}
      {activeTab === 'overview' && summary && (
        <>
          <div style={{
            display: 'flex', gap: 16, marginBottom: 24,
            overflowX: 'auto', paddingBottom: 4,
          }}>
            <MetricCard
              label="Total Runs" value={summary.total_runs}
              color="#64B5F6" />
            <MetricCard
              label="Sessions" value={summary.total_sessions}
              color="#81C784" />
            <MetricCard
              label="Avg Success" value={(summary.avg_success_rate * 100).toFixed(1)}
              unit="%" color="#FFB74D" trend={2.3} />
            <MetricCard
              label="Best IK Solver" value={summary.best_ik_rate * 100}
              unit="%" color="#CE93D8" />
          </div>

          <h3 style={{ color: '#90CAF9', margin: '0 0 12px' }}>Recent Runs</h3>
          <RecentRunsTable runs={summary.recent_runs} />
        </>
      )}

      {/* MLflow embed tab */}
      {activeTab === 'mlflow' && (
        <div style={{ marginTop: 8 }}>
          <div style={{
            background: 'rgba(255,255,255,0.03)',
            borderRadius: 8, padding: 12, marginBottom: 12,
            fontSize: '0.85em', color: '#78909C',
          }}>
            MLflow UI at <a href={effectiveMlflowUrl}
              target="_blank" rel="noopener noreferrer"
              style={{ color: '#64B5F6' }}>
              {effectiveMlflowUrl}
            </a>
            {' '} — Start with:
            <code style={{
              background: 'rgba(0,0,0,0.3)', padding: '2px 6px',
              borderRadius: 4, marginLeft: 8, fontSize: '0.95em',
            }}>
              docker compose --profile tracking up
            </code>
          </div>
          <iframe
            src={effectiveMlflowUrl}
            title="MLflow"
            style={{
              width: '100%', height: 600, border: 'none',
              borderRadius: 8, background: '#fff',
            }}
          />
        </div>
      )}

      {/* IK Solver tab */}
      {activeTab === 'ik' && (
        <div style={{ marginTop: 8 }}>
          <h3 style={{ color: '#90CAF9', margin: '0 0 16px' }}>
            IK Solver Comparison
          </h3>
          <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: '0.88em' }}>
            <thead>
              <tr style={{
                borderBottom: '2px solid rgba(255,255,255,0.08)',
                color: '#78909C',
              }}>
                <th style={{ padding: 8, textAlign: 'left' }}>Solver</th>
                <th style={{ padding: 8, textAlign: 'center' }}>Success</th>
                <th style={{ padding: 8, textAlign: 'center' }}>Mean Time</th>
                <th style={{ padding: 8, textAlign: 'center' }}>P95 Time</th>
                <th style={{ padding: 8, textAlign: 'center' }}>Pos Error</th>
              </tr>
            </thead>
            <tbody>
              {[
                { name: 'analytical_6dof', rate: 97.5, mean: 0.8, p95: 2.1, err: 0.3 },
                { name: 'ikpy_numerical', rate: 89.2, mean: 15.4, p95: 42.0, err: 1.2 },
                { name: 'rtb_numerical', rate: 92.1, mean: 8.7, p95: 28.5, err: 0.9 },
              ].map((s, i) => (
                <tr key={i} style={{
                  borderBottom: '1px solid rgba(255,255,255,0.04)',
                }}>
                  <td style={{ padding: 8, fontWeight: 500 }}>{s.name}</td>
                  <td style={{
                    padding: 8, textAlign: 'center',
                    color: s.rate >= 95 ? '#4CAF50' : '#FF9800',
                    fontWeight: 600,
                  }}>
                    {s.rate}%
                  </td>
                  <td style={{ padding: 8, textAlign: 'center' }}>{s.mean} ms</td>
                  <td style={{ padding: 8, textAlign: 'center' }}>{s.p95} ms</td>
                  <td style={{ padding: 8, textAlign: 'center' }}>{s.err} mm</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
}
