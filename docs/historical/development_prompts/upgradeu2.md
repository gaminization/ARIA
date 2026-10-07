<context>
This is UPGRADE U2 for ARIA — the Advanced Perception upgrade.

Prerequisites: All 4 main ARIA stages complete and validated.
U1 (LLM Planning) does NOT need to be complete first.
U2 is fully independent of U1 — can run in parallel or in any order.

CURRENT PERCEPTION STATE:
  VisionAgent:    YOLO v8 → class + bounding box (2D)
  DepthAgent:     Depth-Anything v2 → metric depth map
  CoordTransform: pixel + table height → world XYZ (position only)
  AffordanceAgent: rule-based affordance lookup
  GraspNode:      geometric grasp from 3D position (no orientation)
  
  Current limitations:
    - Object POSITION known, ORIENTATION unknown
      → Can pick up a cube but cannot pick up a key by its handle
    - Depth fails on transparent/reflective surfaces
      → Glass cup depth is wrong or missing
    - YOLO gives bounding box, not shape
      → Grasp plan based on class, not actual object shape
    - Single camera view, no persistent 3D model of workspace
    - No material awareness → same grip force for plastic vs metal

UPGRADE ADDS:
    6D Pose:     full position + orientation per object
    SAM2:        pixel-accurate masks instead of bounding boxes
    Transparent: special handling for glass, metal, plastic wrap
    Material:    recognize material → adjust grasp parameters
    Gaussian Splatting: persistent photorealistic 3D workspace model

VRAM BUDGET (RTX 5060, 8GB):
  YOLO v8m fp16:          0.8 GB  (always loaded)
  Depth-Anything v2 small: 0.5 GB  (always loaded)
  SAM2 tiny:              0.5 GB  (always loaded — tiny model)
  FoundationPose:         2.0 GB  (loaded on demand, for precision tasks)
  ClearGrasp:             0.6 GB  (loaded when transparent detected)
  Material classifier:    0.2 GB  (always loaded — small ResNet)
  CUDA overhead:          0.8 GB
  Total at max:           ~5.4 GB (fits within 8GB)
  
  Gaussian Splatting reconstruction: offline process, not real-time
  Gaussian Splatting rendering:      0.3 GB  (small model)

Hardware: Lenovo LOQ — i7-13700HX, RTX 5060 8GB
Stack: Pop!_OS 22.04, ROS2 Humble, ARIA full stack
</context>

<task>
Generate the complete advanced perception upgrade for ARIA.
All files production-ready. No stubs. No pseudocode.
Each new perception module integrates into the existing
VisionState and publishes to existing ARIA topics.
Existing nodes are NOT deleted — they are extended.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 1 — 6D OBJECT POSE ESTIMATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_vision/arm_vision/pose_6d_node.py

class Pose6DNode(Node):
  """
  6D pose estimation using FoundationPose.
  Adds full orientation to existing XYZ position.
  
  FoundationPose: 
    GitHub: NVlabs/FoundationPose
    Input:  RGB image + depth map + object mask + reference
    Output: 4×4 SE(3) transformation matrix (position + orientation)
    
  Two operating modes:
    MODE A — Texture-based (reference images):
      Requires: 5-10 reference photos of each object
      Works for: any textured object
      Better for: real-world deployment on known objects
      
    MODE B — Geometry-based (object point cloud from depth):
      Requires: nothing extra (uses depth + mask already available)
      Works for: any object with distinctive 3D shape
      Better for: quick setup on new objects
      
  Integration with existing pipeline:
    Subscribes:
      /top_camera/image_raw          (from CameraNode)
      /depth/image_depth_anything    (from DepthNode)
      /detection/objects             (from DetectionNode — for masks trigger)
      /sam2/masks                    (from SAM2Node — pixel masks)
      
    Triggers:
      Run FoundationPose only when:
        1. Object requires orientation knowledge (from ObjectModelDB)
        2. OR task type is: insertion, assembly, tool_use
        3. OR user requests high-precision mode
      Skip for: simple pick-place of symmetric objects
      
    Publishes:
      /pose_6d/objects (geometry_msgs/PoseArray):
        Full 6D pose for each detected object
        Pose.orientation is now meaningful
      /pose_6d/visualization (MarkerArray):
        Coordinate axes at each object pose for RViz verification
        
  Reference object setup:
    Service /aria/pose_6d/register_object:
      Request: {class_name: str, n_reference_images: int}
      Process:
        → Prompt user: "Show me the [object] from [n] different angles"
        → Capture frames at arm's active perception viewpoints
        → Store as reference for FoundationPose
        → Save to arm_vision/config/object_references/{class_name}/
        
  Updating world model:
    When 6D pose estimated with confidence > 0.8:
      Update WorldObject.pose with full 6D pose
      Update AffordanceAgent: now has correct approach axis
      Update GraspNode: can use orientation-aware grasps
      
  Performance:
    FoundationPose at ~5fps on RTX 5060 (acceptable for planning phase)
    Not run every frame — triggered for specific objects before grasp
    Log inference time to EvaluationAgent

File: arm_vision/scripts/capture_reference_images.py

  CLI tool for capturing reference images for new objects:
  
  Usage: python capture_reference_images.py --object cup --views 8
  
  Process:
    1. Moves arm to 8 viewpoints around the object
    2. Captures top_camera image at each viewpoint
    3. Prompts user to verify each capture
    4. Saves to arm_vision/config/object_references/cup/
    5. Generates reference_manifest.yaml with poses
  
  After capture: "cup registered. FoundationPose ready for cup."

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 2 — SAM2 SEGMENTATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_vision/arm_vision/sam2_node.py

class SAM2Node(Node):
  """
  Segment Anything Model v2 — pixel-accurate object masks.
  
  Model: SAM2-tiny (smallest, fastest, 0.5GB VRAM)
  GitHub: facebookresearch/sam2
  
  Input prompts (from YOLO detections):
    Box prompt: [x1, y1, x2, y2] bounding box → SAM2 refines to mask
    Point prompt: object center pixel → SAM2 finds surrounding mask
    
  Output:
    Per-object binary mask (same resolution as input image)
    Mask confidence score
    
  Integration:
    Subscribes: /detection/objects (YOLO detections as box prompts)
    Publishes:  /sam2/masks (custom SAM2Masks.msg)
                /sam2/image_masked (RGB with colored overlays)
                
  SAM2Masks.msg:
    int32[] object_ids
    sensor_msgs/Image[] masks  (binary, mono8)
    float32[] confidences
    
  How masks improve the existing pipeline:
  
    1. BETTER GRASP PLANNING (feeds GraspNode):
       Old: grasp at bounding box center ± class-based heuristic
       New: compute grasp from actual object silhouette
            find minimum bounding rectangle of mask
            orient gripper to object's actual major axis
            
    2. BETTER DEPTH ESTIMATION (feeds DepthNode):
       Old: depth at center pixel of bounding box
       New: mean depth within mask boundary (more accurate)
            exclude mask edge pixels (depth artifacts at boundaries)
            
    3. BETTER AFFORDANCE (feeds AffordanceAgent):
       Old: "cup → use handle" (class-level rule)
       New: locate handle sub-region within cup mask
            handle detection: mask + color gradient analysis
            → precise handle grasp point, not just general strategy
            
    4. OCCLUSION HANDLING:
       When object partially occluded:
         Visible mask → estimate full shape via symmetric completion
         "I can see the right half of the cup, estimate left half"
         
  Performance:
    SAM2-tiny: ~15ms per prompt on RTX 5060
    Run on every YOLO detection update (30fps → acceptable)
    Cache masks between frames if object not moved
    Use tracking IDs: if object stationary, reuse last mask

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 3 — TRANSPARENT AND REFLECTIVE OBJECTS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_vision/arm_vision/transparent_object_node.py

class TransparentObjectNode(Node):
  """
  Handles the depth estimation failure on glass, shiny plastic,
  metallic surfaces, and clear wrap.
  
  Problem:
    Depth-Anything v2 trained on opaque objects.
    On transparent surfaces: depth output is WRONG or MISSING.
    Monocular model sees background through glass → estimates
    background depth instead of glass surface depth.
    
  Detection:
    Identify transparent/reflective objects automatically:
      1. YOLO class check: if class in TRANSPARENT_CLASSES → transparent
         TRANSPARENT_CLASSES = ["glass", "bottle", "wine_glass", "cup",
                                 "vase", "window", "mirror"]
      2. Depth discontinuity check:
           If depth within YOLO bbox has bimodal distribution
           (some pixels at object range, others at background range)
           → likely transparent
      3. Specular highlight detection:
           Bright saturated patches on object → metallic or glossy
           
  Solution — ClearGrasp approach:
    Model: fine-tuned ResNet on transparent object dataset
    Input: RGB image + initial (broken) depth map
    Output: completed/corrected depth map for transparent regions
    
    GitHub: NVlabs/cleargrasp (or TransCG as alternative)
    
    If ClearGrasp not available: geometric fallback:
      Use object surface normal estimation (from surrounding depth)
      Fit a plane to neighboring non-transparent depth values
      Extrapolate plane into the transparent region
      → Approximate but better than wrong depth
      
  Integration:
    Subscribes:
      /top_camera/image_raw
      /depth/image_depth_anything (potentially incorrect)
      /detection/objects
      /sam2/masks
      
    Pipeline:
      1. Detect transparent objects in current frame
      2. If any detected: run ClearGrasp on those regions
      3. Merge corrected depth into main depth map
      4. Publish merged result
      
    Publishes:
      /depth/image_completed (32FC1):
        Full depth map with transparent regions corrected
      /perception/transparent_objects (ObjectList):
        Which detected objects are transparent
      /perception/depth_quality (Float32MultiArray):
        Per-object depth confidence after completion
        
  Grasp strategy adjustment:
    For transparent objects:
      Increase approach slowness (visual servoing more important)
      Adjust expected depth: glass has near-zero thickness
        → grasp at detected surface, not center of perceived volume
      Flag to GraspNode: "transparent → use gentle grip force"
      Publish: /perception/material_flags with transparency flag

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 4 — MATERIAL RECOGNITION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_vision/arm_vision/material_recognition_node.py

class MaterialRecognitionNode(Node):
  """
  Identifies object material from visual appearance.
  Maps material → grasp parameter adjustments.
  
  Material categories:
    RIGID_PLASTIC:    most common, standard grasp force
    RIGID_METAL:      heavier than appears, firm grip
    GLASS:            fragile, gentle grip, reduce speed
    SOFT_FOAM:        compressible, anticipate compression
    RUBBER:           high friction, less grip force needed
    CARDBOARD:        can deform, avoid point pressure
    FABRIC:           non-rigid, special handling
    WOOD:             rigid, medium friction
    
  Model architecture:
    Backbone: MobileNetV3-Small (0.2GB VRAM, fast, accurate enough)
    Input: object crop from SAM2 mask (not full image)
    Output: material class + confidence
    
    Training data: Materials in Context Database (MINC)
      5.6M patches, 23 material categories
      Pretrained weights available
      
  Zero-shot fallback (if pretrained model not available):
    Use CLIP embeddings:
      Compute CLIP embedding of object crop
      Compare to text embeddings of material descriptions:
        "shiny metallic surface", "transparent glass",
        "rough wooden surface", "smooth plastic", etc.
      Nearest text embedding → predicted material
      
  Integration:
    Subscribes:
      /top_camera/image_raw
      /sam2/masks
      /detection/objects
      
    Publishes:
      /material/predictions (MaterialPredictions.msg):
        Per object: {object_id, material_class, confidence}
        
  Grasp parameter mapping:
    MATERIAL_TO_GRASP_PARAMS = {
      "glass": {
        "grip_force_scale": 0.4,    # 40% of normal force
        "approach_speed_scale": 0.5, # 50% of normal speed
        "visual_servo_extra_iters": 5
      },
      "metal": {
        "grip_force_scale": 1.2,    # 20% extra force (slippery)
        "approach_speed_scale": 0.8
      },
      "soft_foam": {
        "grip_force_scale": 0.3,
        "grasp_offset_mm": -3       # reach further (foam compresses)
      },
      "cardboard": {
        "grip_force_scale": 0.5,
        "approach_style": "flat_palm"  # avoid point pressure
      },
      ...
    }
    
    Service /aria/material/get_grasp_params (GetMaterialGraspParams.srv):
      Request:  {object_id: int}
      Response: {material: str, confidence: float,
                 grip_force_scale: float, approach_speed_scale: float,
                 special_notes: str}
                 
    MaterialAgent (or updates to AffordanceAgent):
      Before any grasp: query material
      Apply material-specific scaling to GraspNode parameters
      Log: "Grasping [object] (material: glass, confidence: 0.87)
            → applying gentle grip (40% force, 50% speed)"
            
    Updates WorldObject:
      Store material in WorldObject.material field
      Next time same object seen: use stored material
      → faster lookup, no re-inference needed

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 5 — 3D GAUSSIAN SPLATTING WORKSPACE MODEL
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_vision/arm_vision/gaussian_splatting_node.py

class GaussianSplattingNode(Node):
  """
  Persistent photorealistic 3D model of the workspace.
  
  Library: gsplat (fast, minimal dependency, Hugging Face backed)
  GitHub:  nerfstudio-project/gsplat
  
  Two phases:
  
  PHASE A — Reconstruction (offline, runs once or when workspace changes):
    1. Move arm to 12-20 viewpoints around workspace
    2. Capture RGB + depth at each viewpoint
    3. Run gsplat training (~5-10 minutes on RTX 5060)
    4. Save trained splat model to arm_vision/models/workspace_splat.ckpt
    
  PHASE B — Runtime inference (real-time, replaces depth for static scene):
    1. Given current camera pose → render depth from splat model
    2. Splat depth is MUCH more accurate than monocular for:
        Static elements (table surface, fixed objects)
        Areas that were well-sampled during reconstruction
    3. Use splat depth for: world model background, obstacle positions
    4. Use Depth-Anything for: NEW objects not in splat (dynamic elements)
    
    Hybrid depth strategy:
      Change detection:
        splat_depth vs live_depth → significant difference → new object
      Static elements:      use splat_depth (accurate, fast)
      New/moved objects:    use Depth-Anything (handles novel content)
      
  Service /aria/gaussian_splatting/reconstruct:
    Triggers PHASE A reconstruction
    User must clear workspace of objects first
    → "Please clear all movable objects from the workspace.
       I will reconstruct the background model."
       
  Service /aria/gaussian_splatting/update_region (x, y, radius):
    Partial update of a workspace region
    Useful when furniture is rearranged
    
  Publishes:
    /gaussian/depth_rendered (32FC1):
      Depth map from current camera viewpoint
    /gaussian/rgb_rendered (RGB):
      Photorealistic render of workspace (for debugging)
    /gaussian/change_mask (mono8):
      Pixels where live depth significantly differs from splat
      (these are new/moved objects)
    /gaussian/reconstruction_status (String):
      "READY", "RECONSTRUCTING", "STALE", "NONE"

  Note on VRAM during reconstruction:
    gsplat training uses ~4-6GB VRAM
    Unload YOLO + depth models during reconstruction
    Reconstruction is offline — no robot motion during this time
    
  Fallback:
    If no splat model available: use Depth-Anything only (original behavior)
    If splat model stale (> 7 days): warn but still use it

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 6 — PERCEPTION PIPELINE ORCHESTRATOR
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_vision/arm_vision/perception_orchestrator.py

class PerceptionOrchestrator(Node):
  """
  Manages all perception modules.
  Decides which modules to run based on task context and VRAM.
  
  Active module sets by task type:
  
    SIMPLE_PICK: (default)
      YOLO + DepthAnything + SAM2 + MaterialRecognition
      VRAM: ~2.0GB
      
    PRECISION_PICK: (orientation matters)
      + FoundationPose
      VRAM: ~4.0GB
      
    TRANSPARENT_PICK: (glass/metal detected)
      + ClearGrasp
      VRAM: ~2.6GB
      
    FULL_PRECISION: (assembly, tool use)
      + FoundationPose + ClearGrasp
      VRAM: ~4.6GB
      
    SCENE_UNDERSTANDING: (complex sorting, planning)
      + GaussianSplatting rendering + SAM2
      VRAM: ~3.3GB
      
  Service /aria/perception/set_mode (SetPerceptionMode.srv):
    Called by TaskManager based on task type
    Loads/unloads models as needed
    Reports: {active_modules, vram_used_gb, ready: bool}
    
  Automatic mode selection:
    PlanningAgent includes task_type in action plan
    Orchestrator maps task_type → perception mode
    No manual intervention needed for normal operation
    
  Publishes unified perception output:
    /perception/unified_scene (UnifiedScene.msg):
      Combines output from all active modules:
      {
        objects: [
          {
            id, class_name, confidence,
            position_xyz: [x,y,z],
            orientation_quat: [x,y,z,w],   # from FoundationPose if active
            mask: binary_image,             # from SAM2
            material: string,              # from MaterialRecognition
            depth_confidence: float,       # quality of depth estimate
            pose_6d_available: bool
          }
        ],
        scene_type: str,
        depth_source: str,  # "splat" | "depth_anything" | "hybrid"
        active_modules: [str]
      }

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 7 — UPDATES TO EXISTING NODES
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_vision/arm_vision/grasp_node_v2.py

  Extension of original grasp_node.py.
  Now uses 6D pose and SAM2 masks:
  
  def compute_grasp_from_mask(mask, pose_6d, material):
    """
    Old method:   top_down at center, or side at center
    New method:   orientation-aware grasp using actual object shape
    
    Steps:
      1. Find minimum bounding rectangle of SAM2 mask
      2. Major axis = gripper alignment axis
      3. Grasp width = minor axis of mask (actual object width)
      4. If 6D pose available: approach along object's local Z
      5. Apply material-specific force scaling
    """
    
  Backward compatible:
    If SAM2 masks unavailable: use original bbox-based method
    If 6D pose unavailable: use XYZ position only (original)
    Graceful degradation at every level

File: arm_agents/arm_agents/affordance_agent_v2.py

  Extension of original affordance_agent.py.
  Now uses SAM2 mask + material for precise affordance:
  
  def locate_handle_in_mask(object_mask, class_name):
    """
    Find handle sub-region within object mask.
    Method: connected component analysis on mask
    For cup: handle is the protruding connected region
             to the side of the main circular body
    Returns: handle_mask, handle_center, handle_approach_axis
    """
    
  def material_aware_affordance(class_name, material):
    """
    Combines class-based affordance with material:
      glass_cup: handle grasp + gentle force
      metal_mug: handle grasp + firm force (heavier)
      paper_cup:  body grasp (handles tear) + gentle force
    """

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 8 — VALIDATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_bringup/scripts/validate_upgrade_u2.py

  Test 1: SAM2 masks
    Spawn 4 objects in sim
    Run SAM2 on YOLO detections
    Assert: masks generated for all 4
    Assert: mask IoU vs ground truth bounding polygon > 0.85
    
  Test 2: 6D pose accuracy
    Spawn cube at known 30° rotation
    Run FoundationPose
    Assert: orientation error < 10°
    Assert: position error < 8mm
    
  Test 3: Transparent object depth
    Spawn "glass bottle" (transparent material in Gazebo)
    Compare raw depth vs ClearGrasp corrected
    Assert: corrected depth within 15mm of ground truth
    (raw depth expected to be significantly worse)
    
  Test 4: Material recognition
    Spawn objects with different material properties
    Run MaterialRecognitionNode
    Assert: metal object predicted as metal (confidence > 0.7)
    Assert: grasp force scale applied correctly
    
  Test 5: Gaussian Splatting workspace model
    Run reconstruction service (clear workspace first)
    Wait for reconstruction to complete
    Assert: rendered depth within 5mm of Gazebo ground truth
    Assert: change_mask lights up when new object placed
    
  Test 6: Full integration
    Task: "Pick up the glass cup by its handle"
    Assert: SAM2 generates cup mask
    Assert: handle located within mask
    Assert: FoundationPose gives cup orientation
    Assert: approach axis aligns with handle orientation
    Assert: grip force reduced (glass material detected)
    Assert: grasp succeeds
    
  Output:
  ══════════════════════════════════════════
  ARIA Upgrade U2 — Advanced Perception
  ══════════════════════════════════════════
  ✅ SAM2: masks accurate (IoU 0.91 avg)
  ✅ FoundationPose: orientation error 7.2°
  ✅ ClearGrasp: transparent depth 11mm MAE
  ✅ Material recognition: 84% accuracy
  ✅ Gaussian Splatting: 3.8mm MAE
  ✅ Full integration: handle grasp SUCCESS
  ══════════════════════════════════════════
  U2 UPGRADE COMPLETE
  ══════════════════════════════════════════
</task>

<output_order>
1.  arm_vision/arm_vision/pose_6d_node.py
2.  arm_vision/scripts/capture_reference_images.py
3.  arm_vision/arm_vision/sam2_node.py
4.  arm_vision/arm_vision/transparent_object_node.py
5.  arm_vision/arm_vision/material_recognition_node.py
6.  arm_vision/arm_vision/gaussian_splatting_node.py
7.  arm_vision/arm_vision/perception_orchestrator.py
8.  arm_vision/arm_vision/grasp_node_v2.py
9.  arm_agents/arm_agents/affordance_agent_v2.py
10. arm_vision/msg/SAM2Masks.msg
11. arm_vision/msg/MaterialPredictions.msg
12. arm_vision/msg/UnifiedScene.msg
13. arm_bringup/scripts/upgrade_u2_install.sh
14. arm_bringup/scripts/validate_upgrade_u2.py
15. arm_bringup/launch/aria_full_u2.launch.py
16. README_UPGRADE_U2.md

[U2 CHECKPOINT] Files: X/16 — Resume: <next_file>
</output_order>