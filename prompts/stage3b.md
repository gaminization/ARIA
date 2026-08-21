<context>
STAGE 3 (Part B) of 4 for ARIA.
Prerequisites: Stage 3a complete.
All 15 agents are running. Now add:
- Full world model and memory persistence
- Complete skill implementations
- VLA model interface + benchmark
- In-hand manipulation
- Real-time dashboard
</context>

<task>

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 1 — WORLD MODEL (Persistent)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/arm_planner/world_model_db.py

SQLite-backed world model. Persists between sessions.

Schema:
  CREATE TABLE objects (
    id INTEGER PRIMARY KEY,
    class_name TEXT,
    display_name TEXT,
    pos_x REAL, pos_y REAL, pos_z REAL,
    rot_x REAL, rot_y REAL, rot_z REAL, rot_w REAL,
    color TEXT,
    material TEXT,
    size_x REAL, size_y REAL, size_z REAL,
    lifecycle_state TEXT,
    last_seen_ts TEXT,
    detection_count INTEGER,
    affordance_scores TEXT  -- JSON blob
  );
  
  CREATE TABLE spatial_relations (
    id INTEGER PRIMARY KEY,
    subject_id INTEGER,
    relation_type TEXT,   -- "is_left_of", "is_inside", "is_on", "is_near"
    object_id INTEGER,
    distance_m REAL,
    confidence REAL,
    last_updated TEXT
  );
  
  CREATE TABLE task_history (
    id INTEGER PRIMARY KEY,
    command TEXT,
    success INTEGER,
    duration_s REAL,
    failure_reason TEXT,
    timestamp TEXT
  );

Object lifecycle transitions:
  NONE → DETECTED: first detection with confidence > 0.7
  DETECTED → TRACKED: detected in 3+ consecutive frames
  TRACKED → LOST: not seen for > 30 frames (1 second at 30fps)
  LOST → RECOVERED: re-detected within 60 seconds
  TRACKED → MOVED: position changed > 5cm since last update
  TRACKED → REMOVED: not seen for > 300 frames (10 seconds)

Methods:
  upsert_object(detection: ObjectDetection) → int (object_id)
  get_object(id) → WorldObject
  get_objects_by_class(class_name) → List[WorldObject]
  update_lifecycle(id, new_state)
  update_spatial_relations(subject_id)  # recompute for this object
  get_related_objects(id, relation_type)
  export_to_yaml(path)   # for visualization/debugging
  import_from_yaml(path) # for loading pre-mapped workspace

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 2 — IN-HAND MANIPULATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_agents/arm_agents/in_hand_manipulation.py

class InHandManipulation:
  """
  Reorients objects while holding them.
  Uses MPU6050 for orientation feedback.
  Uses wrist camera for visual verification.
  """
  
  def rotate_in_hand(target_angle_deg: float,
                     axis: str = 'z') -> bool:
    """
    Rotate held object by rotating wrist_roll joint.
    Monitors MPU6050 to verify rotation.
    Step: rotate 5° increments, verify after each.
    """
    
  def reposition_grip(offset_mm: float,
                      direction: str) -> bool:
    """
    Slide object in gripper by partially opening,
    nudging, then re-closing.
    Use wrist camera to verify new grip position.
    """
    
  def flip_object() -> bool:
    """
    180° rotation. Place on table, regrasp from other end.
    Sequence: place → reposition gripper → regrasp
    """
    
  def slide_to_tip(target_extension_mm: float) -> bool:
    """
    Let object slide to tip of gripper fingers.
    Used for tools: screwdriver, pen, etc.
    Visual feedback from wrist camera.
    """

  Use cases:
    Screwdriver: rotate_in_hand to align tip with screw
    Paintbrush: reposition_grip for correct holding angle
    Key: rotate_in_hand to correct insertion angle

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 3 — VLA INTERFACE + BENCHMARK
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_vla/arm_vla/vla_interface.py

  Interface to Vision-Language-Action models.
  One unified interface, multiple backend implementations.
  
  Backends (implement all, benchmark to select):
  
  1. LeRobot (Hugging Face):
     from lerobot.common.policies.act.modeling_act import ACTPolicy
     Load policy trained on arm's own recorded data
     Input:  camera images + joint states
     Output: joint velocity commands
     
  2. OpenVLA:
     Standard VLA API
     Input:  image + text instruction + joint states  
     Output: delta joint positions
     
  3. Placeholder interfaces for Pi0, Gr00t N1:
     (Include interface skeleton, full integration when models available)
     Clear TODO comment on integration steps
     
  VLA Benchmark (runs once after learn mode provides data):
    For each VLA backend:
      - Execute 10 standardized tasks in sim
      - Measure: task success rate, execution time, smoothness
      - Compare to rule-based planning baseline
    Report: "VLA Benchmark — LeRobot: 72% | OpenVLA: 68% | Baseline: 89%"
    
    Note: VLA typically needs MORE training data to surpass baseline.
          Recommend: use rule-based planning + VLA for novel tasks.
          
  Service /aria/vla/run_task:
    Request: {instruction: str, image: Image}
    Response: {joint_commands: Float64MultiArray[], confidence: float}

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 4 — ARIA DASHBOARD
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_dashboard/app.py  (FastAPI backend)
File: arm_dashboard/frontend/src/App.jsx (React frontend)

FastAPI backend:
  Serves React app at /
  WebSocket at /ws/state  → streams state at 10Hz
  WebSocket at /ws/cameras → streams camera frames at 30fps (MJPEG)
  
  REST endpoints:
    POST /api/command    → forward to /aria/command
    POST /api/approve    → forward to /aria/approve
    POST /api/reject     → forward to /aria/reject
    POST /api/estop      → forward to /aria/estop
    GET  /api/metrics    → EvaluationAgent summary
    GET  /api/logs       → last 100 chain-of-thought entries
    POST /api/joint      → manual joint control (Stage 1 integration)

React dashboard layout:

┌─────────────────────────────────────────────────────────────────┐
│                      ARIA CONTROL CENTER                        │
├─────────────┬───────────────────────────┬───────────────────────┤
│  TOP CAM    │    3D ARM VISUALIZATION    │  TASK CONTROL         │
│  [live feed]│  [Three.js robot model]   │  ┌──────────────────┐ │
│             │  [joint angle readout]    │  │ Command input:   │ │
│  WRIST CAM  │  [workspace bounds]       │  │ > ___________    │ │
│  [live feed]│                           │  └──────────────────┘ │
│             │   J1: ████████░░  90°     │  Goal: [...]          │
│  YOLO boxes │   J2: ██████████  89°     │  Confidence: ██░░ 92% │
│  overlay    │   J3: ████████░░  90°     │  Status: EXECUTING    │
│             │   ...                     │  [APPROVE] [REJECT]   │
│  DEPTH view │                           │  [E-STOP 🔴]         │
├─────────────┴───────────────────────────┴───────────────────────┤
│  CHAIN OF THOUGHT (scrolling, live):                            │
│  [ARIA] EXECUTING SUBGOAL 3: Plan grasp for red_cube            │
│  [ARIA]   ACTION: AffordanceAgent.get_grasp_strategy(red_cube)  │
│  [ARIA]   RESULT: top-down grasp. Confidence 0.92               │
├───────────────────────────┬─────────────────────────────────────┤
│  WORLD MAP (top-down)     │  HEALTH + METRICS                   │
│  [Canvas: table view]     │  Servos: J1🟢 J2🟢 J3🟡 J4🟢 J5🟢 │
│  [Object positions]       │  FPS: 28.4 fps | Latency: 42ms      │
│  [Workspace boundary]     │  Pick rate: 94% | Tasks: 127/134    │
│  [Semantic relations]     │  Last calib: 2h ago ✅               │
└───────────────────────────┴─────────────────────────────────────┘

Camera feeds: MJPEG streams from FastAPI, no encoding overhead
3D arm: Three.js cylinder/box meshes, updates on /joint_states topic
World map: Canvas 2D drawing, objects as colored circles with labels
Health: colored indicators, 🟢🟡🔴 based on thresholds

Also includes:
  Manual joint control tab (links to Stage 1 sliders — now web-based):
    Same functionality as joint_slider_gui.py but in browser

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 5 — MEMORY MANAGER + SKILL MANAGER
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/arm_planner/memory_manager.py
  
  Manages all persistent memory systems:
  - WorldModel database (arm_planner/data/world_model.db)
  - Skill performance data (arm_planner/data/skill_perf.yaml)
  - Calibration offsets (arm_ik/config/calibration_offsets.yaml)
  - Task history (for evaluation)
  
  Startup: load all from disk
  Shutdown: flush all to disk
  
  Service /aria/memory/reset_object(object_id): remove from model
  Service /aria/memory/export: dump world model to YAML
  Service /aria/memory/import(yaml_path): load scene

File: arm_planner/arm_planner/skill_manager.py

  Registry and dispatcher for all skills.
  
  class SkillManager:
    skills: Dict[str, Skill]
    
    register_skill(name, impl, version)
    execute_skill(name, **kwargs) → SkillResult
    get_performance(name) → PerformanceStats
    rollback_skill(name, to_version)  # if new version performs worse
    
    Built-in skills: pick, place, push, pull, stack, sort,
                     inspect, slide, roll, sweep
    Extended skills: rotate_in_hand, reposition_grip, flip, slide_to_tip

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 6 — STAGE 3 VALIDATION + FULL LAUNCH
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_bringup/launch/aria_full.launch.py

  Complete system launch (all stages):
  sim.launch.py → Stage 2 nodes → Stage 3 agents → dashboard
  
  Launch order:
    1. Gazebo + robot
    2. Camera node
    3. Detection node (YOLO)
    4. Depth node
    5. All 15 agents (lifecycle start)
    6. Task manager
    7. Dashboard (FastAPI)
    
  Health gate: TaskManager waits until all agents are ACTIVE
               before accepting commands.

File: arm_bringup/scripts/validate_stage3.py

  Full pipeline tests:
  
  1. All 15 agents active
     → check lifecycle states
  
  2. Natural language planning
     Command: "Put the red cube in the white box"
     Assert: subgoals generated correctly (4+ subgoals)
     Assert: chain of thought logged at each step
     
  3. Full task execution
     Assert: cube in box, task_status=COMPLETE
     Assert: evaluation metrics logged
     
  4. Confidence threshold
     Lower YOLO confidence artificially to 0.5
     Assert: DialogueAgent asks for approval
     Assert: task pauses until approved
     
  5. Failure recovery
     Remove object mid-task
     Assert: MISSED_OBJECT classified
     Assert: recovery attempted
     Assert: DialogueAgent explains failure
     
  6. World model persistence
     Run task, shutdown, restart
     Assert: world model loaded from DB with previous objects
     
  7. Learn mode
     Start learn mode, execute 3 manual picks
     Stop learn mode
     Assert: HDF5 dataset with 3 episodes created
     
  8. Moving target
     Start rolling_object simulation
     Command: "Pick up the rolling cylinder"
     Assert: arm intercepts moving target
     
  Output:
  STAGE 3 COMPLETE — All systems operational.
  Ready for Stage 4 (hardware) when SCORE > 85%
</task>

<output_order>
1.  arm_planner/arm_planner/world_model_db.py
2.  arm_agents/arm_agents/in_hand_manipulation.py
3.  arm_vla/arm_vla/vla_interface.py
4.  arm_vla/arm_vla/vla_benchmark.py
5.  arm_dashboard/app.py
6.  arm_dashboard/frontend/src/App.jsx
7.  arm_dashboard/frontend/src/components/ (all components)
8.  arm_planner/arm_planner/memory_manager.py
9.  arm_planner/arm_planner/skill_manager.py
10. arm_bringup/launch/aria_full.launch.py
11. arm_bringup/scripts/validate_stage3.py
12. arm_vla/package.xml + CMakeLists.txt
13. arm_agents/package.xml + CMakeLists.txt
14. requirements_stage3.txt
15. README_STAGE3.md

[STAGE3B CHECKPOINT] Files: X/15 — Resume: <next_file>
</output_order>