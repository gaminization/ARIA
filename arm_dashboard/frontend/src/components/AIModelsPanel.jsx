import React from 'react';

/**
 * AIModelsPanel — Live status of every AI model in the ARIA pipeline.
 * Shows model name, type, current state, inference latency, and
 * which camera feed each model consumes.
 */

const MODEL_CATALOG = [
  {
    id: 'yolo',
    name: 'YOLOv8m',
    type: 'Object Detection',
    file: 'yolov8m.pt',
    source: 'Ultralytics',
    camera: '/wrist_camera/image_raw',
    topic: '/detection/objects',
    desc: 'Real-time multi-class object detection. 640px input, 80 COCO classes + industrial objects.',
    icon: '🎯',
    color: '#f59e0b',
  },
  {
    id: 'depth_anything',
    name: 'Depth-Anything v2',
    type: 'Monocular Depth',
    file: 'depth_anything_v2_vits (torch.hub)',
    source: 'HuggingFace / ByteDance',
    camera: '/wrist_camera/image_raw',
    topic: '/depth/image_depth_anything',
    desc: 'Monocular depth estimation at 518px. Outputs metric depth using table-height anchoring. Primary depth source.',
    icon: '📐',
    color: '#06b6d4',
  },
  {
    id: 'midas',
    name: 'MiDaS v3 DPT-Hybrid',
    type: 'Monocular Depth (Fallback)',
    file: 'DPT_Hybrid (torch.hub)',
    source: 'Intel ISL',
    camera: '/wrist_camera/image_raw',
    topic: '/depth/image_midas',
    desc: 'Secondary depth model. Runs alongside DA-v2 for benchmarking. Inverse depth → metric conversion.',
    icon: '📏',
    color: '#8b5cf6',
  },
  {
    id: 'sam2',
    name: 'SAM 2',
    type: 'Instance Segmentation',
    file: 'sam2_hiera_small.pt',
    source: 'Meta AI',
    camera: '/wrist_camera/image_raw',
    topic: '/sam2/masks_json',
    desc: 'Segment-Anything v2 for precise object masks. Used by GraspNode v2 for mask-aligned gripper poses.',
    icon: '✂️',
    color: '#10b981',
  },
  {
    id: 'graspnet',
    name: 'GraspNet v2',
    type: 'Grasp Planning',
    file: 'Built-in (rule+SAM2+6D)',
    source: 'ARIA arm_vision',
    camera: '/wrist_camera/image_raw',
    topic: '/aria/grasp/plan_v2',
    desc: '3-tier grasp: 6D-pose-aware → SAM2-mask-aligned → BBox fallback. Material-specific force/speed scaling.',
    icon: '🤏',
    color: '#f97316',
  },
  {
    id: 'openvla',
    name: 'OpenVLA 7B',
    type: 'Vision-Language-Action',
    file: 'openvla/openvla-7b (HF)',
    source: 'Stanford / HuggingFace',
    camera: '/wrist_camera/image_raw',
    topic: '/aria/vla/action',
    desc: 'Language-conditioned manipulation policy. Takes natural language + gripper image → joint commands. 7B param.',
    icon: '🧠',
    color: '#a78bfa',
  },
  {
    id: 'lerobot',
    name: 'LeRobot ACT',
    type: 'Action Chunking Transformer',
    file: 'ACT policy (custom trained)',
    source: 'HuggingFace LeRobot',
    camera: '/wrist_camera/image_raw',
    topic: '/aria/vla/action',
    desc: 'Action Chunking Transformer trained on ARIA recorded demonstrations. Outputs 10-step action chunks.',
    icon: '🎬',
    color: '#ec4899',
  },
  {
    id: 'cot_planner',
    name: 'LLM CoT Planner',
    type: 'Chain-of-Thought Planning',
    file: 'Ollama / local LLM',
    source: 'ARIA arm_planner',
    camera: 'None (text only)',
    topic: '/aria/planning/llm_plan',
    desc: 'Decomposes NL commands into action graphs with full chain-of-thought reasoning. Validates JSON schema.',
    icon: '💭',
    color: '#38bdf8',
  },
  {
    id: 'world_model',
    name: 'World Model DB',
    type: 'Persistent Scene Model',
    file: 'world_model.db (SQLite)',
    source: 'ARIA arm_planner',
    camera: 'Vision stream input',
    topic: '/aria/state/memory',
    desc: 'SQLite-backed 3D scene model. Lifecycle: DETECTED→TRACKED→LOST→RECOVERED. Spatial relations engine.',
    icon: '🌍',
    color: '#22c55e',
  },
];

export default function AIModelsPanel({ health, vision }) {
  const infMs = health?.inference_ms || 0;
  const activeDetections = (vision?.detected_objects || []).length;

  return (
    <div className="ai-models-panel">
      <div className="panel-header">
        <span className="panel-icon">🤖</span>
        <span className="panel-title">AI Models Status</span>
        <span className="panel-badge">{MODEL_CATALOG.length} models</span>
      </div>

      <div className="ai-models-grid">
        {MODEL_CATALOG.map(model => {
          // Heuristic: is this model "active"?
          const isActive = (
            (model.id === 'yolo' && activeDetections > 0) ||
            (model.id === 'depth_anything' && health?.fps_wrist > 0) ||
            (model.id === 'midas' && health?.fps_wrist > 0) ||
            (model.id === 'world_model' && activeDetections > 0) ||
            (model.id === 'cot_planner') ||
            false
          );

          return (
            <div
              key={model.id}
              className={`ai-model-card ${isActive ? 'active' : 'standby'}`}
              style={{ '--model-color': model.color }}
            >
              <div className="ai-model-header">
                <span className="ai-model-icon">{model.icon}</span>
                <div className="ai-model-title">
                  <span className="ai-model-name">{model.name}</span>
                  <span className="ai-model-type">{model.type}</span>
                </div>
                <span className={`ai-model-status ${isActive ? 'active' : 'standby'}`}>
                  {isActive ? '● ACTIVE' : '○ STANDBY'}
                </span>
              </div>

              <div className="ai-model-details">
                <div className="ai-detail-row">
                  <span className="ai-detail-key">Camera:</span>
                  <code className="ai-detail-val cam">{model.camera}</code>
                </div>
                <div className="ai-detail-row">
                  <span className="ai-detail-key">Topic:</span>
                  <code className="ai-detail-val">{model.topic}</code>
                </div>
                <div className="ai-detail-row">
                  <span className="ai-detail-key">File:</span>
                  <code className="ai-detail-val file">{model.file}</code>
                </div>
              </div>

              <div className="ai-model-desc">{model.desc}</div>

              {model.id === 'yolo' && (
                <div className="ai-model-metrics">
                  <span className="metric">
                    <span className="metric-val">{activeDetections}</span>
                    <span className="metric-label">detections</span>
                  </span>
                  <span className="metric">
                    <span className="metric-val">{health?.fps_wrist?.toFixed(1) || '—'}</span>
                    <span className="metric-label">fps</span>
                  </span>
                  <span className="metric">
                    <span className="metric-val">{infMs > 0 ? `${infMs.toFixed(0)}ms` : '—'}</span>
                    <span className="metric-label">latency</span>
                  </span>
                </div>
              )}
            </div>
          );
        })}
      </div>
    </div>
  );
}
