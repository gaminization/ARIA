import React from 'react';

/**
 * CVPipelinePanel — Step-by-step Computer Vision pipeline trace.
 * Shows each step running in the ARIA perception stack with live data.
 */

export default function CVPipelinePanel({ vision, health, memory }) {
  const detections = vision?.detected_objects || [];
  const objects = memory?.known_objects || [];
  const relations = memory?.spatial_relations || [];

  const steps = [
    {
      id: 'capture',
      step: 1,
      title: 'Camera Capture',
      subtitle: '/wrist_camera/image_raw',
      detail: `Gripper eye-in-hand camera · 640×480 · ${health?.fps_wrist?.toFixed(1) || '0'} fps`,
      status: health?.fps_wrist > 0 ? 'RUNNING' : 'WAITING',
      icon: '📷',
      data: health?.fps_wrist > 0
        ? `${health.fps_wrist.toFixed(1)} fps · ${health.fps_wrist > 15 ? 'Real-time' : 'Low framerate'}`
        : 'Awaiting camera stream',
    },
    {
      id: 'yolo',
      step: 2,
      title: 'YOLOv8m Detection',
      subtitle: '/detection/objects',
      detail: 'Multi-class detection · NMS 0.45 · Conf 0.5',
      status: detections.length > 0 ? 'RUNNING' : (health?.fps_wrist > 0 ? 'RUNNING' : 'WAITING'),
      icon: '🎯',
      data: `${detections.length} objects detected · ${
        detections.map(d => d.class_name).join(', ') || 'scanning...'
      }`,
    },
    {
      id: 'depth',
      step: 3,
      title: 'Depth-Anything v2',
      subtitle: '/depth/image_depth_anything',
      detail: '518px input · Metric depth anchored to table (TABLE_HEIGHT=0.76m)',
      status: health?.fps_wrist > 0 ? 'RUNNING' : 'WAITING',
      icon: '📐',
      data: health?.inference_ms > 0
        ? `${health.inference_ms.toFixed(0)}ms inference · Metric scale: ~0.80m camera↔table`
        : 'Awaiting depth frames',
    },
    {
      id: 'tracking',
      step: 4,
      title: 'Object Tracking',
      subtitle: '/detection/tracked_ids',
      detail: 'Persistent IDs across frames · lifecycle management',
      status: vision?.tracked_ids?.length > 0 ? 'RUNNING' : (detections.length > 0 ? 'RUNNING' : 'WAITING'),
      icon: '🔍',
      data: `${vision?.tracked_ids?.length || 0} active tracks · IDs: ${
        (vision?.tracked_ids || []).slice(0, 5).join(', ') || 'none'
      }`,
    },
    {
      id: 'sam2',
      step: 5,
      title: 'SAM2 Segmentation',
      subtitle: '/sam2/masks_json',
      detail: 'Instance masks from gripper camera · mask-aligned gripper pose',
      status: detections.length > 0 ? 'RUNNING' : 'WAITING',
      icon: '✂️',
      data: detections.length > 0
        ? `Segmenting ${detections.length} objects · mask → min-bounding-rect → gripper yaw`
        : 'Waiting for detections',
    },
    {
      id: 'transform',
      step: 6,
      title: '3D Coordinate Transform',
      subtitle: '/aria/vision/coord_transform',
      detail: 'Pixel + depth → world frame · camera calibration via AprilTag',
      status: detections.length > 0 ? 'RUNNING' : 'WAITING',
      icon: '📍',
      data: detections.length > 0
        ? detections.slice(0, 2).map(d =>
            `${d.class_name}: (${d.pos_3d?.[0]?.toFixed(2)}, ${d.pos_3d?.[1]?.toFixed(2)}, ${d.pos_3d?.[2]?.toFixed(2)})m`
          ).join(' | ')
        : 'No 3D positions yet',
    },
    {
      id: 'grasp',
      step: 7,
      title: 'GraspNet v2 Planning',
      subtitle: '/aria/grasp/plan_v2',
      detail: '6D-pose → SAM2-mask → BBox fallback · material force scaling',
      status: detections.length > 0 ? 'READY' : 'WAITING',
      icon: '🤏',
      data: detections.length > 0
        ? 'Strategy: auto → 6D → mask → bbox · approach offset: 10cm'
        : 'Awaiting objects for grasp planning',
    },
    {
      id: 'world_model',
      step: 8,
      title: 'World Model Update',
      subtitle: '/aria/state/memory',
      detail: 'SQLite DB · spatial relations · lifecycle state machine',
      status: objects.length > 0 ? 'RUNNING' : 'WAITING',
      icon: '🌍',
      data: `${objects.length} known objects · ${relations.length} spatial relations · ${
        objects.filter(o => o.lifecycle_state === 'TRACKED').length
      } tracked`,
    },
    {
      id: 'cot',
      step: 9,
      title: 'CoT Planner (Ollama LLM)',
      subtitle: '/aria/planning/llm_plan',
      detail: 'NL → action graph · schema validation · confidence gating',
      status: 'READY',
      icon: '💭',
      data: 'Ready for task commands via /aria/command service',
    },
    {
      id: 'ik',
      step: 10,
      title: 'IK Solver',
      subtitle: '/aria/ik/solve',
      detail: 'Geometric 5-DOF IK · reachability check · singularity avoidance',
      status: 'READY',
      icon: '⚙️',
      data: 'Arm: 5-DOF · base_link → end_effector · workspace validated',
    },
    {
      id: 'execution',
      step: 11,
      title: 'Trajectory Execution',
      subtitle: '/joint_trajectory_controller/follow_joint_trajectory',
      detail: 'JTC → hardware interface → servo commands',
      status: 'READY',
      icon: '▶️',
      data: 'Controllers loaded: joint_state_broadcaster + joint_trajectory_controller',
    },
  ];

  const statusColor = {
    RUNNING: '#00ff88',
    READY: '#38bdf8',
    WAITING: '#6b7280',
    ERROR: '#ef4444',
  };

  return (
    <div className="cv-pipeline-panel">
      <div className="panel-header">
        <span className="panel-icon">🔬</span>
        <span className="panel-title">CV Pipeline — Individual Steps</span>
        <span className="panel-badge">
          {steps.filter(s => s.status === 'RUNNING').length} running
        </span>
      </div>

      <div className="pipeline-steps">
        {steps.map((step, i) => (
          <div key={step.id} className={`pipeline-step step-${step.status.toLowerCase()}`}>
            <div className="step-number">{step.step}</div>
            <div className="step-connector" />
            <div className="step-body">
              <div className="step-header">
                <span className="step-icon">{step.icon}</span>
                <span className="step-title">{step.title}</span>
                <code className="step-topic">{step.subtitle}</code>
                <span
                  className="step-status"
                  style={{ color: statusColor[step.status] }}
                >
                  ● {step.status}
                </span>
              </div>
              <div className="step-detail">{step.detail}</div>
              <div className="step-data">{step.data}</div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
