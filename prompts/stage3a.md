<context>
STAGE 3 (Part A) of 4 for ARIA.
Prerequisites: Stage 1 ✅, Stage 2 ✅

STAGE 3a GOAL:
All 15 agents running.
Natural language task planning with chain of thought.
Confidence-based decisions.
Failure classification and recovery.
Unified state system.

Architecture principle:
  Every agent is a ROS2 LifecycleNode.
  Agents communicate through the unified state bus.
  No agent directly calls another agent.
  The TaskManager orchestrates execution order.
</context>

<task>
Generate all Stage 3a files.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 1 — UNIFIED STATE SYSTEM
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/msg/VisionState.msg
  ObjectDetection[] detected_objects    # current detections
  int32[] tracked_ids                   # active tracking IDs
  bool active_perception_mode           # searching for better view
  bool search_mode                      # scanning workspace
  float32 scene_confidence              # overall confidence 0-1
  
File: arm_planner/msg/MemoryState.msg
  WorldObject[] known_objects           # all known objects
  SemanticRelation[] spatial_relations  # object-to-object relations
  string[] recent_tasks                 # last 10 tasks
  
File: arm_planner/msg/TaskState.msg
  string current_command                # raw user command
  string current_goal                   # parsed goal
  string[] subgoals                     # decomposed subgoals
  Action[] action_queue                 # pending actions
  string[] chain_of_thought             # reasoning trace
  FailureEvent[] failure_log            # all failures this task
  float32 confidence                    # 0-1
  bool awaiting_user_approval           # paused for user
  string task_status                    # IDLE/PLANNING/EXECUTING/PAUSED/COMPLETE/FAILED
  
File: arm_planner/msg/HealthState.msg
  ServoHealth[] servo_health            # per servo
  float32 fps_top_camera
  float32 fps_wrist_camera
  float32 inference_latency_ms
  bool calibration_valid
  string[] active_alerts
  
File: arm_planner/msg/ObjectDetection.msg (extends vision_msgs/Detection2D)
  int32 tracking_id
  string class_name
  float32 confidence
  geometry_msgs/PoseStamped pose_3d     # from coordinate transform
  string lifecycle_state                # Detected/Tracked/Lost/Recovered/Moved/Removed
  string[] affordance_regions           # e.g. ["handle", "rim", "base"]
  
File: arm_planner/msg/WorldObject.msg
  int32 id
  string name
  string class_name
  geometry_msgs/PoseStamped last_known_pose
  builtin_interfaces/Time last_seen
  string color                          # "red", "blue", etc.
  string material                       # "plastic", "metal", etc.
  string lifecycle_state
  string[] spatial_relations            # "is_left_of:box", "is_inside:shelf"
  float32[] affordance_scores           # per affordance region

File: arm_planner/msg/FailureEvent.msg
  string failure_type                   # MISSED_OBJECT, IK_FAILURE, etc.
  string cause                          # what caused it
  string recovery_attempted             # what was tried
  bool recovery_succeeded
  builtin_interfaces/Time timestamp
  string action_at_failure

File: arm_planner/arm_planner/state_bus.py
  """
  Central state bus — all agents read/write here.
  Publisher/subscriber pattern via ROS2 topics.
  Thread-safe access via asyncio.
  """
  
  # Single instance, shared by all agents
  # Publishes to:
  #   /aria/state/vision  (VisionState,  10Hz)
  #   /aria/state/memory  (MemoryState,  2Hz)
  #   /aria/state/joints  (JointState,   50Hz, from ros2_control)
  #   /aria/state/task    (TaskState,    10Hz)
  #   /aria/state/health  (HealthState,  1Hz)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 2 — TASK MANAGER (Orchestrator)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/arm_planner/task_manager.py

class TaskManager(LifecycleNode):
  """
  Top-level orchestrator. Receives NL commands.
  Coordinates all agents. Never executes directly.
  Everything flows through TaskManager.
  """
  
  Service /aria/command (SendCommand.srv):
    Request:  {command: str}
    Response: {accepted: bool, task_id: str, message: str}
    
  Lifecycle:
    IDLE      → waiting for command
    PLANNING  → PlanningAgent decomposing task
    EXECUTING → SkillAgent + ControlAgent running
    PAUSED    → awaiting user approval
    RECOVERY  → handling failure
    COMPLETE  → task done, logging results
    
  Main pipeline for each command:
  
    Step 1: PlanningAgent.decompose(command)
      → returns: goal, subgoals[], action_queue[]
      → chain_of_thought entries created
      
    Step 2: For each action in queue:
        ReachabilityAgent.check(action.target_pose)
        SafetyAgent.validate(action)
        SkillAgent.execute(action)
        
        If confidence < CONFIDENCE_THRESHOLD (0.75):
          DialogueAgent.request_approval(action, reason)
          PAUSE execution
          Wait for /aria/approve or /aria/reject
          
        Monitor execution via ControlAgent feedback
        
    Step 3: On action complete:
        EvaluationAgent.log_result(action, outcome)
        Update TaskState
        
    Step 4: On action failure:
        FailureClassifier.classify(error)
        RecoveryAgent.select_strategy(failure)
        Execute recovery
        
  Complete example chain of thought (logged to /aria/state/task):
  
  [ARIA] ═══════════════════════════════════════════
  [ARIA] Command: "Put the red cube in the white box"
  [ARIA] ═══════════════════════════════════════════
  [ARIA] Parsing goal...
  [ARIA] GOAL: Transport red_cube to white_box interior
  [ARIA] Confidence in goal interpretation: 0.93
  [ARIA]
  [ARIA] Decomposing into subgoals:
  [ARIA]   SUBGOAL 1: Locate red_cube in workspace
  [ARIA]   SUBGOAL 2: Locate white_box in workspace
  [ARIA]   SUBGOAL 3: Plan grasp for red_cube
  [ARIA]   SUBGOAL 4: Execute grasp sequence
  [ARIA]   SUBGOAL 5: Transport red_cube to white_box
  [ARIA]   SUBGOAL 6: Release and verify placement
  [ARIA]
  [ARIA] EXECUTING SUBGOAL 1: Locate red_cube
  [ARIA]   ACTION: Run YOLO detection on workspace
  [ARIA]   REASON: Need to find red_cube before grasping.
  [ARIA]           Using top camera for full workspace view.
  [ARIA]   RESULT: red_cube detected at pixel (342, 156), conf=0.94
  [ARIA]   RESULT: World position: X=0.23m Y=0.15m Z=0.05m
  [ARIA]   REASON: Z=0.05m from cube height (cube is 5cm tall).
  [ARIA]           Using primary geometric method (table height known).
  [ARIA]
  [ARIA] EXECUTING SUBGOAL 3: Plan grasp for red_cube
  [ARIA]   ACTION: AffordanceAgent.get_grasp_strategy(red_cube)
  [ARIA]   REASON: red_cube is a rigid cube → top-down grasp preferred.
  [ARIA]           Approach from above, descend vertically.
  [ARIA]   RESULT: Grasp pose computed. Approach vector: [0,0,-1]
  [ARIA]   ACTION: IK.solve(grasp_pose)
  [ARIA]   RESULT: IK success, analytical solver, error=0.4mm, time=0.05ms
  [ARIA]   ACTION: ReachabilityAgent.check(grasp_pose)
  [ARIA]   RESULT: Within workspace. Cables: safe. Proceed.
  [ARIA]   CONFIDENCE: 0.92 → Executing autonomously (threshold: 0.75)
  [ARIA]   ...
  [ARIA] ═══════════════════════════════════════════
  [ARIA] Task COMPLETE in 28.4 seconds
  [ARIA] Pick success rate (all time): 94.1%
  [ARIA] ═══════════════════════════════════════════

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 3 — ALL 15 AGENTS (Full Implementation)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Each agent: LifecycleNode, async spin, state bus reader/writer.
Each has: on_configure, on_activate, on_deactivate.
Each logs its reasoning for every decision.

File: arm_agents/arm_agents/vision_agent.py
  Manages camera pipeline.
  Triggers active perception when object poorly seen.
  Active perception: if confidence < 0.6 OR bbox near edge:
    → Move wrist camera to 3 viewpoints to get better estimate
    → Fuse multi-view observations
    → Update VisionState with improved estimate
  Monitors frame rates and camera health.

File: arm_agents/arm_agents/depth_agent.py
  Runs depth models (from Stage 2 DepthNode).
  Selects Depth-Anything vs MiDaS based on benchmark results.
  Publishes metric depth to state bus.
  Fuses depth with geometric model (coordinate_transformer).
  Monitors: model inference time, depth confidence.

File: arm_agents/arm_agents/tracking_agent.py
  ByteTrack implementation for consistent object IDs.
  Kalman filter for trajectory prediction.
  Moving target handling:
    Tracks object velocity from position history
    Predicts position Δt in future (configurable, default 0.5s)
    Publishes predicted intercept position
  Detects TRACKING_LOST (no detection for N frames).
  Publishes: /tracking/tracked_objects with predicted future positions.

File: arm_agents/arm_agents/affordance_agent.py
  Object affordance learning and inference.
  
  Pre-loaded affordance database:
    cup:          {grasp: "handle", avoid: ["rim"], approach: "horizontal"}
    bottle:       {grasp: "neck", avoid: ["cap"], approach: "top_down"}
    screwdriver:  {grasp: "shaft", tool_use: "pinch", approach: "horizontal"}
    paintbrush:   {grasp: "handle", tool_use: "grasp", approach: "horizontal"}
    cube:         {grasp: "top", avoid: [], approach: "top_down"}
    cylinder:     {grasp: "body", avoid: [], approach: "side"}
    scissors:     {grasp: "handle_holes", avoid: ["blade"], approach: "top"}
    
  Runtime learning:
    After each grasp, log: (object_class, grasp_region, success)
    Update affordance scores: bayesian update on success rate
    Save updated database to arm_agents/data/affordance_db.yaml
    
  Service /aria/affordance/get_grasp (GetAffordanceGrasp.srv):
    Request: {object_class: str, object_pose: PoseStamped}
    Response: {grasp_pose: PoseStamped, confidence: float, region: str}

File: arm_agents/arm_agents/planning_agent.py
  NLP task decomposition.
  
  Approach (no LLM required — rule-based + templates):
    Parse command → extract (action_verb, object, target, conditions)
    Map to canonical task template
    Expand template to subgoals + action queue
    
  Command patterns:
    "pick up X"           → [locate_X, plan_grasp_X, execute_grasp_X, lift_X]
    "put X in Y"          → [locate_X, locate_Y, ...pick_X, move_to_Y, place_in_Y]
    "stack X on Y"        → [locate_X, locate_Y, ...pick_X, align_over_Y, place_X_on_Y]
    "sort [color] objects" → [locate_all, group_by_color, pick_each, place_in_zone]
    "push X to Y"         → [locate_X, plan_push_path_X_to_Y, execute_push]
    "find X"              → [search_workspace_for_X, report_position]
    
  Confidence scoring:
    score = parse_confidence * object_localization_confidence
    If score < 0.75: include {require_approval: true} in TaskState
    
  VLA integration path (optional, when enabled):
    If use_vla=true: forward to VLAAgent for richer decomposition

File: arm_agents/arm_agents/skill_agent.py
  Executes all predefined skills.
  
  10 core skills:
  
  pick(object_id):
    Approach → descend → grasp → lift
    Uses: AffordanceAgent, GraspExecutor, VisualServoing
    
  place(object_id, target_pose):
    Transport → align → descend → release → retract
    
  push(object_id, direction, distance):
    Plan push path → approach → push slowly
    Contact detection via force_estimator
    
  pull(object_id, direction, distance):
    Grip lightly → pull → verify moved
    
  stack(object_id, base_id):
    Pick object → align precisely above base
    → place gently → verify stack stable (IMU)
    
  sort(object_ids, zone_map):
    Sequential: pick each object → classify → place in correct zone
    
  inspect(object_id):
    Move wrist camera around object → capture multiple views
    Report: "Object appears to be [class]. Color: [color]. Size: ~[dim]"
    
  slide(object_id, target_xy):
    Gentle push to slide object to target position
    
  roll(object_id, target_xy):
    For cylindrical/spherical objects: roll to target
    
  sweep(target_area):
    Push multiple objects to clear an area
    
  Skill improvement: each skill logs (attempt, success, failure_reason)
  Periodically: compute success_rate per skill, adjust parameters
  (e.g. if pick success rate drops → adjust grasp_offset by +2mm)

File: arm_agents/arm_agents/control_agent.py
  Executes joint-level commands.
  Interfaces with MoveIt2 and joint_trajectory_controller.
  Monitors execution feedback (position tracking error).
  Triggers visual servoing for final approach.
  Reports step-by-step execution status to TaskState.

File: arm_agents/arm_agents/safety_agent.py
  Intercepts ALL motion commands before execution.
  Checks:
    Joint limits (hard + soft)
    Self-collision (via MoveIt2 collision checking)
    Workspace boundary
    Cable zone clearance
    Force estimate threshold (unexpected contact)
    Servo temperature
  Emergency stop: /aria/estop (responds in < 5ms)
  Publishes: /safety/status (SafetyStatus, 10Hz)
  Logs every safety intervention with reason.

File: arm_agents/arm_agents/memory_agent.py
  Persistent storage manager.
  
  On detection of object:
    If new: create WorldObject entry, assign ID
    If known: update last_known_pose, last_seen
    
  Object coordinate management:
    Stores: {id, class, pose, last_seen, color, material, lifecycle}
    
  When object not at expected location:
    → DialogueAgent: "I cannot find [object]. It was last seen at
                      [location] on [timestamp]. Has it been moved?"
    → If user confirms moved: update WorldObject.last_known_pose
    → If user says "no": re-scan from current position
    
  Persistence: SQLite database at arm_planner/data/world_model.db
  Load on startup, save on every change.

File: arm_agents/arm_agents/world_model_agent.py
  Maintains 3D scene understanding.
  
  Semantic map:
    Tracks spatial relations between objects:
      "cup is_left_of bottle" (distance < 10cm and bearing ~270°)
      "cube is_inside box" (center of cube within box bounds)
      "bottle is_near edge" (within 5cm of table boundary)
    
  Updates relations on every object position change.
  
  Object attributes:
    Color: from YOLO detection + HSV analysis
    Material: inferred from object class + context
    Size: estimated from depth + bbox
    
  Publishes:
    /world_model/objects (MarkerArray: bounding boxes in RViz)
    /world_model/relations (MarkerArray: lines between related objects)

File: arm_agents/arm_agents/learning_agent.py
  Handles all learning and improvement.
  
  Learn mode (teleoperation):
    Activated by: ros2 service call /aria/learn_mode/start std_srvs/Trigger
    
    Records synchronized:
      /top_camera/image_raw        @ 30fps
      /wrist_camera/image_raw      @ 15fps  
      /joint_states                @ 50Hz
      /gripper/command             @ 50Hz
      /mpu6050/imu_raw             @ 100Hz
      task_label (user input)
      
    Format: HDF5, LeRobot-compatible
    Path: arm_learning/datasets/episode_YYYYMMDD_HHMMSS.hdf5
    
    Stop: ros2 service call /aria/learn_mode/stop
    
    Auto-analysis after stop:
      - Smooth joint trajectories detected
      - Successful grasp patterns identified
      - Dataset quality report generated
      
  Skill performance tracking:
    After every skill execution:
      Update skill_performance[skill_name].append(
        {success, duration, attempts, failure_reason})
      
    Periodically compute: running_success_rate, trend
    If trending downward: log alert "Skill [X] performance degrading"

File: arm_agents/arm_agents/evaluation_agent.py
  Performance logging for self-supervised improvement.
  
  Metrics tracked (all logged to arm_planner/logs/metrics.csv):
    localization_error_mm:  || detected_pose - ground_truth_pose ||
    ik_error_mm:           FK(IK(pose)) - target_pose
    pick_success_rate:     rolling window, last 100 picks
    task_completion_rate:  tasks completed / tasks attempted
    planning_time_ms:      NL command → action queue ready
    inference_time_ms:     YOLO + depth per frame
    recovery_rate:         recoveries succeeded / total failures
    
  Reports (generated on demand or every 50 tasks):
    Daily summary: mean/std of all metrics
    Trend analysis: improving or degrading?
    Failure breakdown: pie chart of failure types
    
  Service /aria/evaluation/report: generate summary

File: arm_agents/arm_agents/dialogue_agent.py
  Natural language interface.
  
  Incoming: /aria/command (user text commands)
  
  For each user command:
    Parse and forward to TaskManager
    
  User approval flow:
    When TaskState.awaiting_user_approval == True:
      Publish explanation: "I'm not confident about this action.
                           [Reason]. Should I proceed? [yes/no]"
    Wait for /aria/approve (std_srvs/Trigger)
            or /aria/reject (std_srvs/Trigger)
            
  Failure explanations:
    On task failure: generate human-readable summary
    "The task failed because I couldn't detect the bottle after
     3 attempts. The confidence was 0.43. Try improving lighting."
     
  Status updates (proactive):
    "Picking up red cube... ✓"
    "Moving to white box... ✓"
    "Placement complete! Task done in 28 seconds."
    
  Subscribes: /aria/state/task → generates text updates
  Publishes:  /aria/dialogue/output (String: for dashboard display)
              /aria/dialogue/requires_input (Bool)

File: arm_agents/arm_agents/attention_agent.py
  Selects what to focus perception resources on.
  
  Priority system:
    ACTIVE_TASK objects: highest priority (current target)
    RECENTLY_SEEN objects: medium priority
    BACKGROUND objects: low priority
    
  Drives:
    /detection/focus_region (ROI: where to run YOLO first)
    /depth/focus_region     (where to compute depth more carefully)
    
  When searching: expand focus to full workspace
  When grasping:  focus tightly on target object + gripper

File: arm_agents/arm_agents/reachability_agent.py
  Checks if target pose is reachable before planning.
  
  Checks:
    1. Within theoretical workspace bounds (computed from DH params)
    2. IK has a solution (quick test solve)
    3. MoveIt2 can plan a path (optional, slower)
    4. Cable constraints not violated
    
  Service /aria/reachability/check (CheckReachability.srv):
    Request:  {target_pose: PoseStamped}
    Response: {reachable: bool, reason: str, alternative_pose: PoseStamped}
    
    If not reachable: suggest nearest reachable alternative
    Log: "Pose (x,y,z) not reachable: [reason]. Nearest: (x',y',z')"

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 4 — FAILURE CLASSIFICATION + RECOVERY
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/arm_planner/failure_classifier.py

FAILURE_TYPES = {
  "MISSED_OBJECT":     "Gripper closed but no contact detected",
  "OBJECT_SLIPPED":    "Contact lost during transport",
  "IK_FAILURE":        "No IK solution found for target pose",
  "COLLISION":         "Path planner could not find collision-free path",
  "PERCEPTION_ERROR":  "Object detection confidence too low",
  "TRACKING_LOST":     "Tracking ID lost for N frames",
  "LOW_CONFIDENCE":    "Confidence below threshold, user approval needed",
  "JOINT_LIMIT":       "Motion would exceed joint limit",
  "CABLE_VIOLATION":   "Path passes through cable zone",
  "SERVO_FAULT":       "Servo not responding or overheating",
  "TIMEOUT":           "Action exceeded time limit",
}

def classify(error_context: dict) -> FailureEvent:
  # Pattern matching on error type + context
  # Returns FailureEvent with type, cause, recommended_recovery

File: arm_planner/arm_planner/recovery_manager.py

Recovery strategies per failure type:

  MISSED_OBJECT:
    → Re-detect object (force fresh YOLO inference)
    → If found: adjust grasp pose by observed error + retry
    → Retry up to 3 times
    → If still failing: PERCEPTION_ERROR
    
  OBJECT_SLIPPED:
    → Lower arm immediately (keep object close to table)
    → Re-grasp from current position
    → Increase grip force estimate threshold for this object
    
  IK_FAILURE:
    → Try fallback IK solver
    → Try approaching from different angle (+/-15° rotation)
    → Check if ReachabilityAgent can suggest alternative
    
  COLLISION:
    → Inflate collision margins by 1cm, replan
    → Try different planner (PRM vs RRTConnect)
    → Report if workspace is too cluttered
    
  PERCEPTION_ERROR:
    → Trigger active perception (move for better view)
    → Wait 2 frames for fresh detection
    → If still low confidence: ask user
    
  TRACKING_LOST:
    → Re-detect from scratch (full workspace scan)
    → Check WorldModel for last known position
    → Trigger search behavior

File: arm_planner/arm_planner/search_behavior.py

  Multi-phase search when object not found:
  
  Phase 1: Check last known location (from WorldModel)
    → Move arm to viewpoint of last_known_pose
    → Run detection for 2 seconds
    
  Phase 2: Systematic workspace scan
    → Grid pattern: scan 9 positions across workspace
    → Top camera at each position (arm moves to different heights/angles)
    
  Phase 3: Check behind obstacles
    → Use semantic map to identify occluding objects
    → Move to angle that sees around them
    
  Phase 4: Ask user
    → "I've searched the entire workspace for [object].
       It was last seen at [location] on [time].
       Has it been moved? If yes, where is it?"
       
  Phase 5: If user provides new location
    → Update WorldModel
    → Re-run search at new location

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 5 — HEALTH MONITORING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/arm_planner/health_monitor.py

  Monitors system health at 1Hz.
  
  Servo health (per servo):
    temperature_estimate: from force_estimator + operation time model
    drift_deg: |commanded - actual| when holding still
    Alert if temp > 60°C, shutdown if > 75°C
    Alert if drift > 5° (worn servo)
    
  Camera health:
    fps_top:   smoothed frame rate, alert if < 20fps
    fps_wrist: alert if < 8fps
    Alert if topic silent for > 2 seconds
    
  Inference health:
    latency_ms: rolling average of YOLO + depth time
    Alert if > 200ms (may drop below real-time)
    
  Calibration health:
    Tracks time since last calibration
    Alert after 500 operations without recalibration
    
  Publishes: /aria/state/health at 1Hz
  Logs alerts to arm_planner/logs/health_log.csv
  Emergency actions:
    OVERTEMP → immediate safe shutdown
    CAMERA_FAIL → trigger active perception pause
    IK_DEGRADED → force recalibration

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 6 — CAUSE AND EFFECT UNDERSTANDING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/arm_planner/cause_effect_model.py

  Knowledge base of how objects interact.
  Used by PlanningAgent to anticipate effects of actions.
  
  INTERACTION_MODEL = {
    "push(object, direction)": {
      "expected": "object moves in direction",
      "side_effects": ["may_move_adjacent_objects", "may_fall_off_edge"]
    },
    "place_on(A, B)": {
      "expected": "A is_on B",
      "side_effects": ["A.pose changes", "B may slide if not stable"]
    },
    "stack(A, on=B)": {
      "preconditions": ["B.surface is flat", "A.base is flat", 
                        "A.width < B.surface.width"],
      "expected": "A is_on B, stable",
      "risk": "stack may topple if not centered"
    }
  }
  
  def predict_effects(action, current_world_model):
    """
    Before executing: predict what will change.
    After executing: compare prediction to actual.
    Discrepancy → log as learning data.
    """
  
  def check_preconditions(action, world_model):
    """Check if preconditions met before executing."""
    # e.g. before stack: verify B.surface is flat from depth map

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 7 — MOVING TARGET TRACKING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_agents/arm_agents/moving_target_handler.py

  Extends TrackingAgent for dynamic targets.
  
  class MovingTargetHandler:
    
    Velocity estimation:
      position_history: deque(maxlen=10)  # last 10 positions
      velocity = (pos_t - pos_{t-5}) / (5 * dt)  # 5-frame difference
      
    Kalman filter:
      State: [x, y, vx, vy]
      Predicts future position given current velocity
      
    Intercept prediction:
      given arm_speed and object_velocity:
      compute intercept_pose where arm can reach before object moves away
      
      intercept_time = solve: arm_motion_time(current → intercept) 
                            = object_travel_time(current → intercept)
      intercept_position = object_pos + velocity * intercept_time
      
    Re-planning rate: 30Hz (continuous re-planning)
    
    Use case: rolling pencil
      1. Detect pencil rolling
      2. TrackingAgent maintains ID
      3. Velocity estimated: vx=0.05 m/s
      4. Intercept computed: arm moves to predicted position
      5. Arm arrives before pencil → grasp
      6. If miss: immediately predict next intercept

</task>

<output_order>
1.  arm_planner/msg/VisionState.msg  (+ all other .msg files)
2.  arm_planner/arm_planner/state_bus.py
3.  arm_planner/arm_planner/task_manager.py
4.  arm_agents/arm_agents/vision_agent.py
5.  arm_agents/arm_agents/depth_agent.py
6.  arm_agents/arm_agents/tracking_agent.py
7.  arm_agents/arm_agents/affordance_agent.py
8.  arm_agents/arm_agents/planning_agent.py
9.  arm_agents/arm_agents/skill_agent.py
10. arm_agents/arm_agents/control_agent.py
11. arm_agents/arm_agents/safety_agent.py
12. arm_agents/arm_agents/memory_agent.py
13. arm_agents/arm_agents/world_model_agent.py
14. arm_agents/arm_agents/learning_agent.py
15. arm_agents/arm_agents/evaluation_agent.py
16. arm_agents/arm_agents/dialogue_agent.py
17. arm_agents/arm_agents/attention_agent.py
18. arm_agents/arm_agents/reachability_agent.py
19. arm_planner/arm_planner/failure_classifier.py
20. arm_planner/arm_planner/recovery_manager.py
21. arm_planner/arm_planner/search_behavior.py
22. arm_planner/arm_planner/health_monitor.py
23. arm_planner/arm_planner/cause_effect_model.py
24. arm_agents/arm_agents/moving_target_handler.py
25. All package.xml and CMakeLists.txt

[STAGE3A CHECKPOINT] Files: X/25 — Resume: <next_file>
</output_order>