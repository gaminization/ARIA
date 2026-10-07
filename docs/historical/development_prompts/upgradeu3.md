<context>
This is UPGRADE U3 for ARIA — the Infrastructure upgrade.

Prerequisites: All 4 main ARIA stages complete and validated.
U3 is fully independent of U1 and U2.
Can be implemented in parallel with U1 and U2.

CURRENT STATE:
  No Docker — setup is manual and brittle
  No CI/CD — regressions discovered by accident
  No experiment tracking — training runs are not compared
  No systematic bag recording — debugging relies on memory
  No object database — object properties hardcoded in agents
  
UPGRADE GOAL:
  Docker: entire ARIA stack runs with one command
  CI/CD:  every commit tested automatically in headless sim
  Tracking: every learning run and benchmark logged to W&B or MLflow
  Bags: automatic session recording, organized, indexed
  Object DB: central database of known objects with CAD, appearance,
             weight, material, affordances — grows over time

Hardware: Lenovo LOQ — i7-13700HX, RTX 5060 8GB
Stack: Pop!_OS 22.04, ROS2 Humble, NVIDIA Container Toolkit
</context>

<task>
Generate the complete infrastructure upgrade for ARIA.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 1 — DOCKER CONTAINERIZATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: docker/Dockerfile.base

  Base image with ROS2 + system dependencies:
  
  FROM nvidia/cuda:12.8.0-devel-ubuntu22.04
  
  Build stages (multi-stage for layer caching):
  
  STAGE 1 — system:
    Ubuntu 22.04 base packages
    ROS2 Humble desktop-full
    Gazebo Harmonic + ros_gz bridge
    MoveIt2
    All ros2_control packages
    apriltag_ros, camera_calibration, usb_cam
    
  STAGE 2 — python_base:
    Miniconda
    conda environment "aria" from environment.yaml
    PyTorch 2.5+ with CUDA 12.8 (Blackwell wheels)
    All pip packages from requirements.txt
    
  STAGE 3 — ml_models:
    Clone + install: Depth-Anything V2
    Clone + install: GraspNet-baseline (with MinkowskiEngine)
    Clone + install: ByteTrack
    Clone + install: LeRobot
    Clone + install: gsplat
    Pre-download model weights to /models/
      (YOLO v8m, DA2-small, SAM2-tiny, MiDaS)
    
  STAGE 4 — aria_runtime (final):
    FROM stage 3
    Copy source: /aria_ws/src/
    RUN colcon build --symlink-install
    ENTRYPOINT: /aria_ws/docker/entrypoint.sh

File: docker/Dockerfile.sim
  Extends base. Adds Gazebo GUI support.
  Configures DISPLAY passthrough for GUI forwarding.
  Used for: development and testing.

File: docker/Dockerfile.headless
  Extends base. No GUI packages.
  Used for: CI/CD, automated testing.

File: docker/Dockerfile.dashboard
  Lightweight Node.js container for React dashboard.
  No ROS2, no GPU needed.
  FROM node:20-alpine
  Serves dashboard at port 3000.
  Connects to ROS2 container via WebSocket.

File: docker/entrypoint.sh
  #!/bin/bash
  source /opt/ros/humble/setup.bash
  source /aria_ws/install/setup.bash
  conda activate aria
  export PYTHONPATH=...
  exec "$@"

File: docker-compose.yml

  services:
  
    ros2_core:
      image: aria:runtime
      runtime: nvidia
      environment:
        - NVIDIA_VISIBLE_DEVICES=all
        - DISPLAY=${DISPLAY}
        - ARIA_MODE=${ARIA_MODE:-sim}   # sim | hardware
      volumes:
        - /aria_ws/src:/aria_ws/src     # live source mount (dev mode)
        - /aria_ws/datasets:/datasets
        - /aria_ws/logs:/logs
        - /aria_ws/models:/models
        - /tmp/.X11-unix:/tmp/.X11-unix # GUI passthrough
      network_mode: host                 # ROS2 needs host network for DDS
      devices:
        - /dev/dri                       # GPU for rendering
      command: >
        bash -c "ros2 launch arm_bringup ${ARIA_MODE}.launch.py"
        
    dashboard:
      image: aria:dashboard
      ports:
        - "3000:3000"
      environment:
        - ROS_BRIDGE_HOST=localhost
        - ROS_BRIDGE_PORT=9090
      depends_on:
        - ros2_core
        
    experiment_tracker:
      image: ghcr.io/mlflow/mlflow:latest
      ports:
        - "5000:5000"
      volumes:
        - /aria_ws/mlflow:/mlflow
      command: mlflow server --host 0.0.0.0 --backend-store-uri /mlflow
      profiles:
        - tracking   # only starts with: docker compose --profile tracking up
        
    ollama:                              # for U1 LLM upgrade
      image: ollama/ollama:latest
      runtime: nvidia
      ports:
        - "11434:11434"
      volumes:
        - ~/.ollama:/root/.ollama
      profiles:
        - llm

File: docker/Makefile

  build:
    docker compose build
    
  sim:
    ARIA_MODE=sim docker compose up ros2_core dashboard
    
  hardware:
    ARIA_MODE=hardware docker compose up ros2_core dashboard
    
  test:
    docker compose run --rm ros2_core \
      bash -c "python arm_bringup/scripts/validate_stage3.py"
      
  ci:
    docker compose -f docker-compose.ci.yml run aria_tests
    
  shell:
    docker compose run --rm ros2_core bash
    
  clean:
    docker compose down --volumes --remove-orphans

File: docker/NVIDIA_SETUP.md

  Instructions for installing NVIDIA Container Toolkit
  (prerequisite for GPU access in Docker):
  
  distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
  curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
    sudo apt-key add -
  curl -s -L \
    https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list
  sudo apt update
  sudo apt install -y nvidia-container-toolkit
  sudo systemctl restart docker
  
  Verify: docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 \
    nvidia-smi

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 2 — CI/CD PIPELINE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: .github/workflows/aria_ci.yml

  name: ARIA CI

  on:
    push:
      branches: [main, develop, 'stage*', 'upgrade*']
    pull_request:
      branches: [main]
    schedule:
      - cron: '0 2 * * *'  # nightly at 2am

  env:
    ROS_DISTRO: humble

  jobs:
  
    build:
      name: Build all packages
      runs-on: ubuntu-22.04
      steps:
        - uses: actions/checkout@v4
        - name: Setup ROS2
          uses: ros-tooling/setup-ros@v0.7
          with:
            required-ros-distributions: humble
        - name: Install dependencies
          run: |
            rosdep update
            rosdep install --from-paths src --ignore-src -r -y
        - name: Build
          run: |
            source /opt/ros/humble/setup.bash
            colcon build --symlink-install \
              --cmake-args -DCMAKE_BUILD_TYPE=Release
        - name: Check build artifacts
          run: ls install/
          
    unit_tests:
      name: Unit tests (no ROS, no GPU)
      runs-on: ubuntu-22.04
      needs: build
      steps:
        - uses: actions/checkout@v4
        - name: Install Python deps (CPU only for CI)
          run: |
            pip install pytest pytest-cov pytest-asyncio
            pip install ikpy roboticstoolbox-python scipy numpy
            pip install sqlalchemy pyyaml
        - name: Run pure Python tests
          run: |
            pytest arm_ik/tests/test_analytical_ik.py -v
            pytest arm_planner/tests/test_failure_classifier.py -v
            pytest arm_planner/tests/test_world_model_db.py -v
            pytest arm_planner/tests/test_episodic_memory.py -v
            pytest arm_control/tests/test_trajectory_generator.py -v
        - name: Coverage report
          run: pytest --cov=. --cov-report=xml
        - uses: codecov/codecov-action@v3
          
    integration_tests:
      name: Integration tests (headless sim, Docker)
      runs-on: ubuntu-22.04
      needs: build
      # Note: full GPU sim not available in GitHub Actions free tier
      # Run lightweight tests (no Gazebo physics, mock sensors)
      steps:
        - uses: actions/checkout@v4
        - name: Build headless image
          run: docker build -f docker/Dockerfile.headless -t aria:ci .
        - name: Run node launch tests
          run: |
            docker run --rm aria:ci bash -c "
              source /opt/ros/humble/setup.bash
              source /aria_ws/install/setup.bash
              ros2 launch arm_bringup sim_headless.launch.py &
              sleep 15
              python arm_bringup/scripts/validate_stage1.py \
                --timeout 30 --skip-camera
            "
        - name: Run IK unit tests (no GPU)
          run: |
            docker run --rm aria:ci bash -c "
              python arm_ik/tests/test_ik_solvers_cpu.py
            "
            
    benchmark:
      name: Performance benchmarks
      runs-on: self-hosted   # requires a self-hosted runner with RTX 5060
      needs: integration_tests
      if: github.event_name == 'schedule'  # nightly only
      steps:
        - uses: actions/checkout@v4
        - name: Run IK benchmark
          run: |
            conda activate aria
            python arm_ik/arm_ik/ik_benchmark_node.py \
              --output benchmark_ik_$(date +%Y%m%d).csv
        - name: Run perception benchmark
          run: |
            python arm_bringup/benchmarks/run_full_benchmark.py \
              --output benchmark_full_$(date +%Y%m%d).html
        - name: Compare to baseline
          run: |
            python .github/scripts/compare_benchmarks.py \
              --current benchmark_ik_*.csv \
              --baseline benchmarks/baseline_ik.csv \
              --threshold 0.10   # fail if >10% regression
        - uses: actions/upload-artifact@v4
          with:
            name: benchmark-results
            path: benchmark_*.html

File: .github/scripts/compare_benchmarks.py

  Compares benchmark results to stored baseline.
  Fails CI if any metric regresses by > threshold %.
  Generates comparison report.
  
  Metrics tracked across commits:
    IK success rate per solver
    IK mean solve time
    IK position error
    Perception inference time
    Task success rate
    
  If regression detected:
    Print: "⚠️ REGRESSION: IK success rate dropped from 97.5% to 91.2%
            Last passing commit: abc1234
            Likely cause: check arm_ik changes in this PR"

File: .github/workflows/docker_build.yml

  Builds and pushes Docker image on merge to main.
  Tags: aria:latest, aria:humble, aria:YYYY.MM.DD
  Stores image in GitHub Container Registry (ghcr.io).
  
  Allows: any machine to run ARIA with docker pull.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 3 — EXPERIMENT TRACKING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_learning/arm_learning/experiment_tracker.py

class ExperimentTracker:
  """
  Unified interface supporting both W&B and MLflow.
  Config selects backend. Same API either way.
  
  Backend: "wandb" | "mlflow" | "none" (for offline work)
  
  W&B: requires account + API key, but excellent UI
  MLflow: fully local, self-hosted, no account needed
  
  Default recommendation: MLflow (no account, no data leaves machine)
  W&B: opt-in for users who want cloud dashboard
  """
  
  def __init__(self, backend: str, project: str = "aria"):
    if backend == "wandb":
      import wandb
      wandb.init(project=project, config=self.get_system_config())
    elif backend == "mlflow":
      import mlflow
      mlflow.set_tracking_uri("http://localhost:5000")
      mlflow.set_experiment(project)
      
  def log_ik_benchmark(self, results: IKBenchmarkResults):
    """
    Logs IK benchmark run:
      metrics: success_rate, mean_time_ms, p95_time_ms, position_error_mm
      params:  solver_name, n_test_poses, timestamp
    """
    
  def log_skill_performance(self, skill: str, result: SkillResult):
    """
    Logs every skill execution:
      metrics: success (0/1), duration_s, attempts
      params:  skill_name, object_class, material, task_id
    Enables: track skill improvement over time, detect degradation
    """
    
  def log_training_run(self, config: dict, metrics: dict):
    """
    Logs VLA training runs:
      params:  model_name, n_episodes, learning_rate, batch_size
      metrics: train_loss, val_loss per epoch, task_success_rate
    """
    
  def log_calibration(self, before_error_mm: float, after_error_mm: float):
    """Tracks calibration quality over sessions."""
    
  def log_session_summary(self, session_metrics: SessionMetrics):
    """End-of-session summary: tasks, success rate, top failures."""
    
  def compare_runs(self, run_ids: List[str]) -> ComparisonReport:
    """Compare multiple runs side-by-side (IK solvers, VLA models)."""
    
  Configuration (arm_learning/config/tracking_config.yaml):
    backend: mlflow            # or "wandb" or "none"
    wandb_api_key: ""          # set if using W&B
    mlflow_uri: "http://localhost:5000"
    log_every_skill: true
    log_every_ik_solve: false  # too frequent, log per-benchmark only
    log_images: true           # log annotated frames on failure
    auto_log_session: true

Integration points (modify existing files):

File: arm_ik/arm_ik/ik_benchmark_node.py — ADD:
  At end of benchmark:
    tracker.log_ik_benchmark(benchmark_results)
    # Each solver becomes a separate run for comparison

File: arm_agents/arm_agents/evaluation_agent.py — ADD:
  After each task:
    tracker.log_skill_performance(skill, result)
  After each session:
    tracker.log_session_summary(session_metrics)

File: arm_learning/arm_learning/learning_agent.py — ADD:
  After each training run:
    tracker.log_training_run(config, training_metrics)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 4 — ROS2 BAG RECORDING PIPELINE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_learning/arm_learning/bag_recorder_node.py

class BagRecorderNode(Node):
  """
  Automatic ROS2 bag recording for all sessions.
  Smart topic selection, compression, indexing.
  
  Starts automatically with system. Records everything.
  Organized by date/session with metadata.
  """
  
  Recording modes:
  
    STANDARD (default — always active):
      Topics recorded:
        /joint_states                    @ 50Hz
        /aria/state/task                 @ 10Hz
        /aria/state/health               @ 1Hz
        /detection/objects               @ 10Hz
        /depth/object_positions          @ 10Hz
        /aria/servo_states               @ 50Hz (hardware mode)
        /aria/planning/llm_plan          @ event-driven
        
    FULL (activated for important sessions or debugging):
      Adds to standard:
        /top_camera/image_raw            @ 5fps (reduced rate)
        /wrist_camera/image_raw          @ 5fps
        /detection/image_annotated       @ 5fps
        /depth/image_colorized           @ 2fps
        /grasp/candidates_viz            @ event-driven
        
    INVESTIGATION (activated after failure):
      Adds to full:
        All remaining topics at full rate
        Triggered automatically when FAILED task detected
        Records 30s before failure (circular buffer) + 60s after
        
  Service /aria/bag/start_recording (mode: str)
  Service /aria/bag/stop_recording
  Service /aria/bag/set_mode (mode: str)
  
  Circular pre-buffer:
    Maintains 30-second buffer in memory
    On failure detection: flush buffer → bag
    Ensures failure context is always captured
    
  File organization:
    ~/aria_bags/
      YYYY-MM-DD/
        session_HHMMSS_standard.db3   (small, always)
        session_HHMMSS_full.db3       (medium, when enabled)
        session_HHMMSS_metadata.yaml
        failure_HHMMSS_TASKNAME.db3   (auto, on failure)
        
  Metadata per session:
    {
      start_time, end_time,
      mode: "sim" | "hardware",
      tasks_attempted, tasks_completed,
      objects_in_scene, [list]
      failure_events: [{type, timestamp, task}],
      system_version: git_sha,
      calibration_age_hours
    }

File: arm_learning/arm_learning/bag_indexer.py

  Indexes all recorded bags for search and retrieval.
  
  Database: arm_learning/data/bag_index.db
  
  Schema:
    CREATE TABLE bag_files (
      id INTEGER PRIMARY KEY,
      filepath TEXT,
      start_time TEXT,
      duration_s REAL,
      mode TEXT,
      task_count INTEGER,
      failure_count INTEGER,
      topics TEXT,         -- JSON list
      objects TEXT,        -- JSON list of objects in scene
      metadata_json TEXT
    )
    
  Service /aria/bag/search (BagSearch.srv):
    Request:
      object_class: optional str   # "find all bags with cup"
      failure_type: optional str   # "find all MISSED_OBJECT failures"
      date_from:    optional str   # "2025-06-01"
      date_to:      optional str
      success_only: bool
      failure_only: bool
      
    Response:
      matching_bags: [BagInfo]
      total_found: int
      
  CLI tool:
    aria bag search --object cup --failure MISSED_OBJECT
    aria bag search --date-from 2025-06-01 --failure-only
    aria bag replay <bag_id>
    aria bag export <bag_id> --format hdf5  # for training dataset

File: arm_learning/arm_learning/bag_to_dataset.py

  Converts recorded bags to LeRobot-format training datasets.
  
  def convert_bag_to_hdf5(bag_path: str,
                           output_path: str,
                           success_only: bool = True):
    """
    Extracts from bag:
      - Camera images (top + wrist) at 30fps
      - Joint states at 50Hz
      - Task labels (from chain of thought)
      - Grasp success/fail labels (from EvaluationAgent)
      
    Synchronizes all streams by timestamp.
    Saves in LeRobot HDF5 format.
    
    Filters: success_only=True removes failed episodes by default
             (don't want to learn from failures — unless explicitly training
             failure recovery policy)
    """
    
  CLI: aria bag to-dataset --input session_143022.db3 \
                            --output dataset_cups.hdf5 \
                            --success-only

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 5 — OBJECT MODEL DATABASE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/arm_planner/object_model_db.py

class ObjectModelDB:
  """
  Central database of known objects.
  Grows as ARIA encounters and registers new objects.
  
  Difference from WorldModelDB (Stage 3):
    WorldModelDB: tracks WHERE objects are (position, lifecycle)
    ObjectModelDB: stores WHAT objects are (properties, CAD, appearance)
    
  They are linked: WorldObject.class_name → ObjectModelDB.class_name
  """
  
  Schema:
    CREATE TABLE object_models (
      id INTEGER PRIMARY KEY,
      class_name TEXT UNIQUE,
      display_name TEXT,
      
      -- Physical properties
      mass_kg REAL,
      material TEXT,
      fragile INTEGER,          -- boolean
      
      -- Geometry
      mesh_path TEXT,           -- path to .obj or .stl file
      urdf_path TEXT,           -- for Gazebo sim integration
      bounding_box TEXT,        -- JSON [x, y, z] in meters
      
      -- Grasp properties
      affordance_json TEXT,     -- full affordance definition JSON
      grasp_points_json TEXT,   -- list of named grasp poses
      grip_force_scale REAL,    -- relative to default
      approach_style TEXT,      -- top_down | side | handle | custom
      
      -- Appearance
      typical_colors TEXT,      -- JSON list of common colors
      reference_images_path TEXT, -- directory of reference images
      clip_embedding BLOB,      -- for visual similarity search
      
      -- FoundationPose
      pose_6d_reference_path TEXT, -- reference images for FoundationPose
      
      -- Sim properties (for Gazebo object files)
      sdf_path TEXT,
      friction REAL,
      restitution REAL,
      
      -- Metadata
      date_added TEXT,
      times_grasped INTEGER,
      grasp_success_rate REAL,
      notes TEXT
    )
    
  Pre-populated entries (from Stage 2 known objects):
    red_bottle, blue_cup, white_box, yellow_pencil,
    screwdriver, paintbrush, scissors, cube_variants
    
  Methods:
    get_model(class_name) → ObjectModel
    get_grasp_params(class_name) → GraspParams
    get_affordances(class_name) → AffordanceMap
    search_by_appearance(clip_embedding) → List[ObjectModel]
    
    register_new_object(
      class_name: str,
      sample_images: List[np.ndarray],
      physical_props: dict
    ) → int:
      """
      Register a new object ARIA has never seen before.
      1. Generate CLIP embedding from sample images
      2. Check similarity: if close match to existing → confirm with user
      3. Create database entry
      4. Save reference images for FoundationPose
      5. Generate Gazebo SDF from bounding box estimate
      """
      
    update_success_rate(class_name: str, success: bool):
      """
      Called after every grasp attempt.
      Updates rolling success rate per object class.
      If success_rate drops below 0.6:
        Flag for review: "consider updating grasp parameters for {class}"
      """

File: arm_planner/scripts/object_model_cli.py

  CLI for managing the object database:
  
  aria objects list
    → Table of all known objects with grasp success rates
    
  aria objects show cup
    → Full details: mass, material, affordances, success rate
    
  aria objects register
    → Interactive wizard to register a new object:
       "Place the object in the workspace. I'll capture reference images."
       Arm moves to 8 viewpoints, captures images, measures bounding box
       User provides: name, mass (estimate), fragile? (y/n)
       CLIP embedding generated automatically
       
  aria objects update cup --mass 0.3 --grip-force 0.4
    → Update specific properties
    
  aria objects export --format yaml
    → Export all models to YAML for backup/sharing
    
  aria objects import objects_library.yaml
    → Import object library (e.g. from another ARIA installation)
    
  aria objects stats
    → Table: object class | times grasped | success rate | last seen
    → Sorted by most-used
    → Highlights objects with degrading success rate

File: arm_planner/data/object_library_default.yaml

  Pre-populated object definitions for the default simulation objects.
  Format matches ObjectModelDB schema.
  Include complete entries for:
    red_bottle, blue_cup, white_box, yellow_pencil,
    screwdriver, paintbrush, scissors, red_cube,
    blue_cylinder, green_sphere
    
  Each entry is a complete reference that can be imported
  into any ARIA installation.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 6 — DASHBOARD ADDITIONS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_dashboard/frontend/src/components/ObjectLibraryPanel.jsx

  New dashboard tab: "Object Library"
  
  Shows:
    Table of all registered objects
    Per-object: image thumbnail, grasp success rate, last seen
    Click to expand: full properties, reference images, affordance map
    "Register New Object" button (triggers wizard)
    Grasp success rate chart over time per object
    
File: arm_dashboard/frontend/src/components/ExperimentPanel.jsx

  New dashboard tab: "Experiments"
  
  Shows:
    Embedded MLflow UI (iframe to localhost:5000)
    OR W&B charts if configured
    Recent runs table: date, task count, success rate
    IK solver comparison chart
    Skill performance trends (line chart, last 30 days)
    
File: arm_dashboard/frontend/src/components/BagRecorderPanel.jsx

  New dashboard section in status bar:
  
  Shows:
    [●] Recording (STANDARD mode) | 1.2 GB | 00:42:15
    [Stop] [Switch to FULL] [Search Bags]
    
  Search Bags popup:
    Filter by: date, object, failure type, task
    Click result: play bag in RViz

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 7 — VALIDATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_bringup/scripts/validate_upgrade_u3.py

  Test 1: Docker builds
    docker build -f docker/Dockerfile.base -t aria:base .
    docker build -f docker/Dockerfile.sim -t aria:sim .
    docker build -f docker/Dockerfile.headless -t aria:headless .
    Assert: all build without errors
    
  Test 2: Container runs
    docker run --rm aria:headless ros2 --version
    docker run --rm --gpus all aria:sim nvidia-smi
    Assert: ROS2 available, GPU accessible inside container
    
  Test 3: Experiment tracking
    Start MLflow: docker compose --profile tracking up experiment_tracker
    Run mock training run: tracker.log_training_run(config, metrics)
    Assert: run appears in MLflow UI at localhost:5000
    Assert: metrics queryable via MLflow Python API
    
  Test 4: Bag recording
    Start bag recorder in STANDARD mode
    Run 3 tasks
    Stop recorder
    Assert: bag file created with expected topics
    Assert: metadata YAML created alongside bag
    Assert: bag indexed in bag_index.db
    
  Test 5: Bag search
    aria bag search --object red_cube
    Assert: finds bags from test 4
    Assert: metadata matches
    
  Test 6: Bag to dataset conversion
    Convert recorded bag to HDF5
    Assert: HDF5 has correct keys (observation.images, action, etc.)
    Assert: LeRobot can load the dataset
    
  Test 7: Object model database
    aria objects list → shows default objects
    Run pick task on red_cube
    Assert: success rate updated in ObjectModelDB
    aria objects register (interactive — skip in CI, test via mock)
    
  Test 8: CI pipeline
    Push a trivial commit to a test branch
    Assert: GitHub Actions triggers
    Assert: build job passes
    Assert: unit tests pass
    (Integration test and benchmark only fully testable with self-hosted runner)
    
  Output:
  ══════════════════════════════════════════
  ARIA Upgrade U3 — Infrastructure
  ══════════════════════════════════════════
  ✅ Docker: all images build
  ✅ Container: ROS2 + GPU accessible
  ✅ MLflow: tracking run logged
  ✅ Bag recording: 3-task session recorded
  ✅ Bag search: query returns results
  ✅ Dataset conversion: LeRobot format valid
  ✅ Object DB: success rate updated
  ✅ CI: pipeline triggers on push
  ══════════════════════════════════════════
  U3 UPGRADE COMPLETE
  ══════════════════════════════════════════
</task>

<output_order>
1.  docker/Dockerfile.base
2.  docker/Dockerfile.sim
3.  docker/Dockerfile.headless
4.  docker/Dockerfile.dashboard
5.  docker/entrypoint.sh
6.  docker-compose.yml
7.  docker-compose.ci.yml
8.  docker/Makefile
9.  docker/NVIDIA_SETUP.md
10. .github/workflows/aria_ci.yml
11. .github/workflows/docker_build.yml
12. .github/scripts/compare_benchmarks.py
13. arm_learning/arm_learning/experiment_tracker.py
14. arm_learning/config/tracking_config.yaml
15. arm_learning/arm_learning/bag_recorder_node.py
16. arm_learning/arm_learning/bag_indexer.py
17. arm_learning/arm_learning/bag_to_dataset.py
18. arm_planner/arm_planner/object_model_db.py
19. arm_planner/scripts/object_model_cli.py
20. arm_planner/data/object_library_default.yaml
21. arm_dashboard/frontend/src/components/ObjectLibraryPanel.jsx
22. arm_dashboard/frontend/src/components/ExperimentPanel.jsx
23. arm_dashboard/frontend/src/components/BagRecorderPanel.jsx
24. arm_bringup/scripts/upgrade_u3_install.sh
25. arm_bringup/scripts/validate_upgrade_u3.py
26. arm_bringup/launch/aria_full_u3.launch.py
27. README_UPGRADE_U3.md

[U3 CHECKPOINT] Files: X/27 — Resume: <next_file>
</output_order>