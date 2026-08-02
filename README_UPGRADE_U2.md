# ARIA Upgrade U2 — Advanced Perception

> **Upgrade U2** adds pixel-accurate segmentation, 6D object pose, transparent object handling, material-aware grasping, and a persistent 3D workspace model. Every module has graceful fallbacks — the system works even without optional dependencies.

---

## Architecture

```
  ┌─────────────────────────────────────────────────────────────┐
  │                  Perception Orchestrator                     │
  │  Monitors task type + detected objects → selects mode       │
  │  SIMPLE_PICK │ PRECISION │ TRANSPARENT │ FULL │ SCENE      │
  └──────────┬───────────┬────────────┬───────────┬────────────┘
             │           │            │           │
  ┌──────────▼──┐ ┌──────▼──────┐ ┌──▼─────────┐ │
  │ SAM2 Masks  │ │FoundnPose  │ │ ClearGrasp  │ │
  │ (always on) │ │ (on demand) │ │ (on demand) │ │
  │ 0.5GB VRAM  │ │ 2.0GB VRAM  │ │ 0.6GB VRAM  │ │
  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ │
         │               │               │        │
  ┌──────▼──────┐ ┌──────▼──────┐ ┌──────▼──────┐ │
  │ Material    │ │ 6D Pose     │ │ Corrected   │ │
  │ Recognition │ │ Estimation  │ │ Depth Map   │ │
  │ (always on) │ │             │ │             │ │
  │ 0.2GB VRAM  │ └─────────────┘ └─────────────┘ │
  └──────┬──────┘                                  │
         │                                  ┌──────▼──────┐
         │                                  │ Gaussian    │
         │                                  │ Splatting   │
         │                                  │ (on demand) │
         │                                  │ 0.3GB VRAM  │
         │                                  └──────┬──────┘
  ┌──────▼──────────────────────────────────────────▼──────┐
  │              Unified Scene Output                       │
  │  /perception/unified_scene (all objects + metadata)     │
  └────────────────────┬───────────────────────────────────┘
                       │
  ┌────────────────────▼───────────────────────────────────┐
  │     Grasp Node v2 + Affordance Agent v2                 │
  │  mask-aligned │ 6D-aware │ material-scaled grasps       │
  └─────────────────────────────────────────────────────────┘
```

---

## VRAM Modes

| Mode | Modules | VRAM | Trigger |
|------|---------|------|---------|
| SIMPLE_PICK | YOLO+Depth+SAM2+Material | ~2.8GB | Default, "pick up X" |
| PRECISION_PICK | + FoundationPose | ~4.8GB | Tools, cups, asymmetric objects |
| TRANSPARENT_PICK | + ClearGrasp | ~3.4GB | Glass/transparent detected |
| FULL_PRECISION | + FoundationPose + ClearGrasp | ~5.4GB | Assembly, insertion tasks |
| SCENE_UNDERSTANDING | + Gaussian Splatting | ~3.1GB | Organize, scan, inspect |

---

## New Nodes

| Node | Package | VRAM | Always On |
|------|---------|------|-----------|
| `sam2_node` | arm_vision | 0.5GB | Yes |
| `pose_6d_node` | arm_vision | 2.0GB | No (on demand) |
| `transparent_object_node` | arm_vision | 0.6GB | Yes (detection only) |
| `material_recognition_node` | arm_vision | 0.2GB | Yes |
| `gaussian_splatting_node` | arm_vision | 0.3GB | No (on demand) |
| `perception_orchestrator` | arm_vision | - | Yes (coordinator) |
| `grasp_node_v2` | arm_vision | - | Yes |
| `affordance_agent_v2` | arm_agents | - | Yes |

---

## Quick Start

```bash
# Install dependencies
bash arm_bringup/scripts/upgrade_u2_install.sh

# Source and launch
source install/setup.bash
ros2 launch arm_bringup aria_full_u2.launch.py

# Monitor
ros2 topic echo /perception/mode
ros2 topic echo /perception/vram_usage
ros2 topic echo /perception/unified_scene
ros2 topic echo /sam2/masks_json
ros2 topic echo /material/predictions

# Validate
python3 arm_bringup/scripts/validate_upgrade_u2.py
```

---

## Perception Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/sam2/masks_json` | String | Per-object mask properties |
| `/sam2/image_masked` | Image | RGB with colored mask overlays |
| `/pose_6d/objects` | PoseArray | 6D poses for detected objects |
| `/pose_6d/visualization` | MarkerArray | Coordinate axes in RViz |
| `/depth/image_completed` | Image | Corrected depth (transparent regions) |
| `/material/predictions` | String | Material + grasp params per object |
| `/gaussian/depth_rendered` | Image | Depth from 3D workspace model |
| `/gaussian/change_mask` | Image | New/moved object detection |
| `/perception/unified_scene` | String | Combined scene output |
| `/perception/mode` | String | Current perception mode |
| `/perception/vram_usage` | String | VRAM breakdown |

---

## Fallback Chain

Every module has graceful degradation:

| Module | Preferred | Fallback 1 | Fallback 2 |
|--------|-----------|------------|------------|
| Segmentation | SAM2-tiny | GrabCut | Bbox mask |
| 6D Pose | FoundationPose | PCA on depth cloud | XYZ only |
| Depth Correction | ClearGrasp | Plane fitting | Original depth |
| Material | MobileNetV3 | CLIP zero-shot | Class→material heuristic |
| 3D Model | gsplat | Depth fusion | No model |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| SAM2 not loading | Check `sam2_hiera_tiny.pt` in `arm_vision/models/` |
| VRAM overflow | Lower perception mode: `ros2 topic pub /aria/state/task ...` |
| Poor masks | Check camera image quality on `/top_camera/image_raw` |
| Wrong material | Material cache expires in 5s, re-detect automatically |
| Reconstruction fails | Need ≥3 views. Clear workspace first |
