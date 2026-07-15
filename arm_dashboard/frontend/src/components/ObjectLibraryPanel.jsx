import React, { useState, useEffect, useMemo } from 'react';

/**
 * ═══════════════════════════════════════════════════════════
 * ARIA Object Library Panel
 * Dashboard tab showing registered objects, success rates,
 * reference images, and affordance maps.
 * ═══════════════════════════════════════════════════════════
 */

const MATERIAL_COLORS = {
  ceramic: '#64B5F6',
  rigid_plastic: '#81C784',
  rigid_metal: '#B0BEC5',
  cardboard: '#FFB74D',
  wood: '#A1887F',
  rubber: '#CE93D8',
  glass: '#80DEEA',
  unknown: '#BDBDBD',
};

const APPROACH_ICONS = {
  top_down: '⬇️',
  horizontal: '➡️',
  side: '↗️',
};

function ObjectRow({ obj, isExpanded, onToggle }) {
  const rate = obj.times_grasped > 0
    ? `${(obj.grasp_success_rate * 100).toFixed(0)}%`
    : '—';

  const rateColor = obj.grasp_success_rate >= 0.8 ? '#4CAF50'
    : obj.grasp_success_rate >= 0.6 ? '#FF9800'
    : obj.times_grasped > 0 ? '#F44336' : '#9E9E9E';

  const materialColor = MATERIAL_COLORS[obj.material] || MATERIAL_COLORS.unknown;

  return (
    <>
      <tr
        onClick={onToggle}
        style={{
          cursor: 'pointer',
          background: isExpanded ? 'rgba(100,181,246,0.08)' : 'transparent',
          transition: 'background 0.2s',
        }}
      >
        <td style={{ padding: '10px 14px', fontWeight: 500 }}>
          {obj.display_name || obj.class_name}
        </td>
        <td style={{ padding: '10px 14px' }}>
          <span style={{
            background: materialColor,
            color: '#1a1a2e',
            padding: '2px 10px',
            borderRadius: 12,
            fontSize: '0.82em',
            fontWeight: 600,
          }}>
            {obj.material}
          </span>
        </td>
        <td style={{ padding: '10px 14px', textAlign: 'center' }}>
          {obj.times_grasped}
        </td>
        <td style={{
          padding: '10px 14px',
          textAlign: 'center',
          color: rateColor,
          fontWeight: 600,
        }}>
          {rate}
          {obj.times_grasped > 5 && obj.grasp_success_rate < 0.6 && ' ⚠️'}
        </td>
        <td style={{ padding: '10px 14px', textAlign: 'center' }}>
          {obj.grip_force_scale !== undefined
            ? `${(obj.grip_force_scale * 100).toFixed(0)}%`
            : '100%'}
        </td>
        <td style={{ padding: '10px 14px', textAlign: 'center' }}>
          {APPROACH_ICONS[obj.approach_style] || '⬇️'} {obj.approach_style}
        </td>
      </tr>

      {isExpanded && (
        <tr>
          <td colSpan={6} style={{
            padding: '16px 20px',
            background: 'rgba(30,30,60,0.5)',
            borderBottom: '1px solid rgba(255,255,255,0.05)',
          }}>
            <ObjectDetail obj={obj} />
          </td>
        </tr>
      )}
    </>
  );
}

function ObjectDetail({ obj }) {
  const affordance = obj.affordance || {};

  return (
    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 20 }}>
      <div>
        <h4 style={{ margin: '0 0 8px', color: '#90CAF9' }}>Physical</h4>
        <div style={{ fontSize: '0.9em', lineHeight: 1.8 }}>
          <div>Mass: <strong>{obj.mass_kg?.toFixed(3)} kg</strong></div>
          <div>Fragile: <strong>{obj.fragile ? 'Yes ⚠️' : 'No'}</strong></div>
          <div>Bounding Box: <strong>
            {obj.bounding_box?.map(v => `${(v * 100).toFixed(0)}cm`).join(' × ')}
          </strong></div>
          {obj.typical_colors?.length > 0 && (
            <div>Colors: <strong>{obj.typical_colors.join(', ')}</strong></div>
          )}
        </div>
      </div>

      <div>
        <h4 style={{ margin: '0 0 8px', color: '#A5D6A7' }}>Affordance</h4>
        <div style={{ fontSize: '0.9em', lineHeight: 1.8 }}>
          <div>Grasp Region: <strong>{affordance.grasp || 'top'}</strong></div>
          <div>Has Handle: <strong>{affordance.has_handle ? 'Yes' : 'No'}</strong></div>
          <div>Approach: <strong>{affordance.approach || 'top_down'}</strong></div>
          {affordance.avoid?.length > 0 && (
            <div>Avoid: <strong style={{ color: '#EF9A9A' }}>
              {affordance.avoid.join(', ')}
            </strong></div>
          )}
        </div>
      </div>

      {obj.notes && (
        <div style={{ gridColumn: '1 / -1', marginTop: 4 }}>
          <span style={{ color: '#9E9E9E', fontSize: '0.85em' }}>
            📝 {obj.notes}
          </span>
        </div>
      )}
    </div>
  );
}

export default function ObjectLibraryPanel({ wsUrl }) {
  const [objects, setObjects] = useState([]);
  const [expandedId, setExpandedId] = useState(null);
  const [searchTerm, setSearchTerm] = useState('');
  const [sortBy, setSortBy] = useState('class_name');

  // Fetch object library from backend
  useEffect(() => {
    const fetchObjects = async () => {
      try {
        const res = await fetch(`${wsUrl || ''}/api/objects`);
        if (res.ok) {
          const data = await res.json();
          setObjects(data.objects || []);
        }
      } catch {
        // Fallback: try WebSocket
        setObjects(DEMO_OBJECTS);
      }
    };
    fetchObjects();
    const interval = setInterval(fetchObjects, 10000);
    return () => clearInterval(interval);
  }, [wsUrl]);

  const filtered = useMemo(() => {
    let list = objects.filter(o =>
      o.class_name?.toLowerCase().includes(searchTerm.toLowerCase()) ||
      o.material?.toLowerCase().includes(searchTerm.toLowerCase())
    );

    list.sort((a, b) => {
      if (sortBy === 'success') return (b.grasp_success_rate || 0) - (a.grasp_success_rate || 0);
      if (sortBy === 'grasps') return (b.times_grasped || 0) - (a.times_grasped || 0);
      return (a.class_name || '').localeCompare(b.class_name || '');
    });

    return list;
  }, [objects, searchTerm, sortBy]);

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
        <h2 style={{ margin: 0, color: '#90CAF9' }}>
          📦 Object Library
          <span style={{
            fontSize: '0.6em', marginLeft: 12, color: '#78909C',
          }}>
            {objects.length} object{objects.length !== 1 ? 's' : ''}
          </span>
        </h2>

        <div style={{ display: 'flex', gap: 10 }}>
          <input
            type="text"
            placeholder="Search objects..."
            value={searchTerm}
            onChange={e => setSearchTerm(e.target.value)}
            style={{
              background: 'rgba(255,255,255,0.06)',
              border: '1px solid rgba(255,255,255,0.1)',
              borderRadius: 8, padding: '6px 14px',
              color: '#E0E0E0', outline: 'none',
            }}
          />

          <select
            value={sortBy}
            onChange={e => setSortBy(e.target.value)}
            style={{
              background: 'rgba(255,255,255,0.06)',
              border: '1px solid rgba(255,255,255,0.1)',
              borderRadius: 8, padding: '6px 10px',
              color: '#E0E0E0', outline: 'none',
            }}
          >
            <option value="class_name">Sort: Name</option>
            <option value="grasps">Sort: Most Used</option>
            <option value="success">Sort: Success Rate</option>
          </select>
        </div>
      </div>

      {/* Table */}
      <table style={{
        width: '100%', borderCollapse: 'collapse',
        fontSize: '0.9em',
      }}>
        <thead>
          <tr style={{
            borderBottom: '2px solid rgba(255,255,255,0.1)',
            color: '#78909C', textTransform: 'uppercase',
            fontSize: '0.8em', letterSpacing: '0.05em',
          }}>
            <th style={{ padding: '8px 14px', textAlign: 'left' }}>Object</th>
            <th style={{ padding: '8px 14px', textAlign: 'left' }}>Material</th>
            <th style={{ padding: '8px 14px', textAlign: 'center' }}>Grasps</th>
            <th style={{ padding: '8px 14px', textAlign: 'center' }}>Success</th>
            <th style={{ padding: '8px 14px', textAlign: 'center' }}>Force</th>
            <th style={{ padding: '8px 14px', textAlign: 'center' }}>Approach</th>
          </tr>
        </thead>
        <tbody>
          {filtered.map(obj => (
            <ObjectRow
              key={obj.class_name}
              obj={obj}
              isExpanded={expandedId === obj.class_name}
              onToggle={() => setExpandedId(
                expandedId === obj.class_name ? null : obj.class_name
              )}
            />
          ))}
        </tbody>
      </table>

      {filtered.length === 0 && (
        <div style={{
          textAlign: 'center', padding: 40, color: '#616161',
        }}>
          {searchTerm ? 'No matching objects' : 'No objects registered'}
        </div>
      )}
    </div>
  );
}

// Demo data for when backend is unavailable
const DEMO_OBJECTS = [
  { class_name: 'red_bottle', display_name: 'Red Bottle', material: 'rigid_plastic',
    mass_kg: 0.35, fragile: false, bounding_box: [0.07, 0.07, 0.22],
    grip_force_scale: 0.8, approach_style: 'side', times_grasped: 47,
    grasp_success_rate: 0.89, typical_colors: ['red'],
    affordance: { grasp: 'neck', avoid: ['cap'], approach: 'side', has_handle: false } },
  { class_name: 'blue_cup', display_name: 'Blue Cup', material: 'ceramic',
    mass_kg: 0.25, fragile: true, bounding_box: [0.08, 0.10, 0.09],
    grip_force_scale: 0.5, approach_style: 'horizontal', times_grasped: 32,
    grasp_success_rate: 0.78, typical_colors: ['blue'],
    affordance: { grasp: 'handle', avoid: ['rim', 'interior'], approach: 'horizontal', has_handle: true } },
  { class_name: 'red_cube', display_name: 'Red Cube', material: 'rigid_plastic',
    mass_kg: 0.05, fragile: false, bounding_box: [0.04, 0.04, 0.04],
    grip_force_scale: 0.8, approach_style: 'top_down', times_grasped: 120,
    grasp_success_rate: 0.95, typical_colors: ['red'],
    affordance: { grasp: 'top', avoid: [], approach: 'top_down', has_handle: false } },
];
