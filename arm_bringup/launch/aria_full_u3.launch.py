#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Full System Launch — Upgrade U3 (Infrastructure)
All Stages + Bag Recording + Object DB + Experiment Tracking

Adds to aria_full.launch.py:
  - Bag recorder node (automatic session recording)
  - rosbridge WebSocket (for dashboard connection)
═══════════════════════════════════════════════════════════════
"""
import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    TimerAction, ExecuteProcess, LogInfo, GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Arguments ──────────────────────────────────────────
    use_sim = DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='Use Gazebo simulation')
    dashboard_port = DeclareLaunchArgument(
        'dashboard_port', default_value='8000',
        description='Dashboard server port')
    bag_mode = DeclareLaunchArgument(
        'bag_mode', default_value='standard',
        description='Bag recording mode: standard, full, or none')
    tracking_backend = DeclareLaunchArgument(
        'tracking_backend', default_value='mlflow',
        description='Experiment tracking backend: mlflow, wandb, none')

    bringup_dir = get_package_share_directory('arm_bringup')

    # ── World and GUI overrides ─────────────────────────────
    world = DeclareLaunchArgument(
        'world', default_value='aria_tester_workspace.world',
        description='World file in arm_bringup/worlds')
    world_conf = LaunchConfiguration('world')

    gui = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Launch Gazebo Classic GUI (set true for GUI window)')
    gui_conf = LaunchConfiguration('gui')

    synthetic = DeclareLaunchArgument(
        'synthetic', default_value='false',
        description='Launch in fast synthetic mode (skips heavy agents, executes synthetic demonstration suite)')
    synthetic_conf = LaunchConfiguration('synthetic')

    # CPU-only hosts (no CUDA) cannot run the heavy learned-perception stack
    # (DepthAnything, SAM2, grasp synthesis, OpenVLA) in real time. `lite:=true`
    # starts the full 15-agent state machine while omitting those GPU-bound nodes,
    # letting the skill agent resolve poses from simulator ground truth instead.
    lite = DeclareLaunchArgument(
        'lite', default_value='false',
        description='Skip GPU-dependent perception/inference nodes (depth, SAM2, grasp, OpenVLA, learning, evaluation)')
    lite_conf = LaunchConfiguration('lite')

    # ═══════════════════════════════════════════════════════
    # STAGE 1: Gazebo + Robot + Controllers (0s)
    # ═══════════════════════════════════════════════════════
    stage1_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'sim.launch.py')),
        launch_arguments={'world': world_conf, 'gui': gui_conf}.items(),
    )

    # ═══════════════════════════════════════════════════════
    # STAGE 2: Camera relay + Detection + Depth + IK (2s)
    # ═══════════════════════════════════════════════════════
    # Camera relay — republishes the Gazebo camera feeds on the ARIA
    # namespace (/aria/top_camera/image_raw, /aria/wrist_camera/image_raw)
    # that the dashboard bridge subscribes to. Without this relay the
    # dashboard's wrist and overhead feeds have no publisher in the sim stack.
    camera_node = TimerAction(
        period=1.5,
        actions=[Node(
            package='arm_vision', executable='camera_node',
            name='camera_node', output='screen',
            parameters=[{'mode': 'sim', 'use_sim_time': True}],
        )],
    )

    # GRIPPER CAMERA ONLY — both nodes subscribe to /wrist_camera/image_raw
    yolo_detection = TimerAction(
        period=2.0,
        actions=[Node(
            package='arm_vision', executable='detection_node',
            name='detection_node', output='screen',
            parameters=[{'camera_topic': '/top_camera/image_raw'}],
        )],
    )

    depth_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='arm_vision', executable='depth_node',
            name='depth_node', output='screen',
            parameters=[{'camera_topic': '/wrist_camera/image_raw'}],
        )],
    )

    ik_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='arm_ik', executable='ik_node',
            name='ik_node', output='screen',
        )],
    )

    grasp_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='arm_vision', executable='grasp_node',
            name='grasp_node', output='screen',
        )],
    )

    sam2_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='arm_vision', executable='sam2_node',
            name='sam2_node', output='screen',
        )],
    )

    visual_servo_node = TimerAction(
        period=2.5,
        actions=[Node(
            package='arm_control',
            executable='visual_servo_node.py',
            name='visual_servo_node',
            output='screen',
        )],
    )

    # ═══════════════════════════════════════════════════════
    # STAGE 3: 15 Agents (5s delay)
    # ═══════════════════════════════════════════════════════
    vision_agent = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='vision_agent',
        name='vision_agent', output='screen')])

    depth_agent = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='depth_agent',
        name='depth_agent', output='screen')])

    tracking_agent = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='tracking_agent',
        name='tracking_agent', output='screen')])

    affordance_agent = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='affordance_agent',
        name='affordance_agent', output='screen')])

    attention_agent = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='attention_agent',
        name='attention_agent', output='screen')])

    planning_agent = TimerAction(period=5.5, actions=[Node(
        package='arm_agents', executable='llm_planning_agent',
        name='llm_planning_agent', output='screen')])

    reachability_agent = TimerAction(period=5.5, actions=[Node(
        package='arm_agents', executable='reachability_agent',
        name='reachability_agent', output='screen')])

    skill_agent = TimerAction(period=6.0, actions=[Node(
        package='arm_agents', executable='skill_agent',
        name='skill_agent', output='screen')])

    control_agent = TimerAction(period=6.0, actions=[Node(
        package='arm_agents', executable='control_agent',
        name='control_agent', output='screen')])

    safety_agent = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='safety_agent',
        name='safety_agent', output='screen')])

    memory_agent = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='memory_agent',
        name='memory_agent', output='screen')])

    world_model_agent = TimerAction(period=5.5, actions=[Node(
        package='arm_agents', executable='world_model_agent',
        name='world_model_agent', output='screen')])

    learning_agent = TimerAction(period=6.0, actions=[Node(
        package='arm_agents', executable='learning_agent',
        name='learning_agent', output='screen')])

    evaluation_agent = TimerAction(period=6.0, actions=[Node(
        package='arm_agents', executable='evaluation_agent',
        name='evaluation_agent', output='screen')])

    dialogue_agent = TimerAction(period=6.0, actions=[Node(
        package='arm_agents', executable='dialogue_agent',
        name='dialogue_agent', output='screen')])

    # OpenVLA Executor — VLA inference from gripper camera
    # Model loads lazily in background (15GB — ~30s on first activation)
    openvla_executor = TimerAction(period=7.0, actions=[Node(
        package='arm_agents', executable='openvla_executor',
        name='openvla_executor', output='screen')])

    # Orchestration
    task_manager = TimerAction(period=7.0, actions=[Node(
        package='arm_planner', executable='task_manager',
        name='task_manager', output='screen')])

    memory_manager = TimerAction(period=7.0, actions=[Node(
        package='arm_planner', executable='memory_manager',
        name='memory_manager', output='screen')])

    health_monitor = TimerAction(period=7.0, actions=[Node(
        package='arm_planner', executable='health_monitor',
        name='health_monitor', output='screen')])

    # ═══════════════════════════════════════════════════════
    # U3: Infrastructure (8s delay)
    # ═══════════════════════════════════════════════════════

    # Bag recorder — automatic session recording
    bag_recorder = TimerAction(
        period=8.0,
        actions=[Node(
            package='arm_learning', executable='bag_recorder_node',
            name='bag_recorder_node', output='screen',
            parameters=[{
                'auto_start': True,
                'default_mode': LaunchConfiguration('bag_mode'),
            }],
        )],
    )

    # rosbridge for dashboard WebSocket (optional — dashboard WebSocket works without it)
    # Only include if package is available
    try:
        from ament_index_python.packages import get_package_share_directory as _gpsd
        _gpsd('rosbridge_server')  # will raise if not installed
        rosbridge = TimerAction(
            period=8.0,
            actions=[Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge',
                parameters=[{'port': 9090}],
                output='screen',
            )],
        )
    except Exception:
        import warnings
        warnings.warn(
            "[ARIA U3] rosbridge_server not found — skipping. "
            "Install with: sudo apt install ros-humble-rosbridge-server"
        )
        rosbridge = LogInfo(msg='rosbridge_server not installed — skipped')

    # Dashboard (9s)
    dashboard = TimerAction(
        period=9.0,
        actions=[ExecuteProcess(
            cmd=['python3', '-m', 'arm_dashboard.app'],
            name='dashboard', output='screen',
            additional_env={
                'ARIA_DASHBOARD_PORT': LaunchConfiguration('dashboard_port'),
            },
        )],
    )

    # Agent Activator (8.5s) — automatically configures and activates all LifecycleNode agents
    activate_agents = TimerAction(
        period=8.5,
        actions=[ExecuteProcess(
            cmd=['python3', os.path.join(
                os.environ.get('ARIA_ROOT', '/home/gaminizer/Projects/ARIA'),
                'arm_bringup', 'scripts', 'activate_agents.py'
            )],
            name='activate_agents', output='screen',
        )],
    )

    startup_msg = TimerAction(
        period=10.0,
        actions=[LogInfo(msg='\n'
            '═══════════════════════════════════════════════════════\n'
            '  🤖 ARIA FULL SYSTEM ONLINE — U3 INFRASTRUCTURE\n'
            '  Dashboard:  http://localhost:8080\n'
            '  rosbridge:  ws://localhost:9090\n'
            '  MLflow:     docker compose --profile tracking up\n'
            '  Bags:       ~/aria_bags/<date>/\n'
            '  Objects:    python3 arm_planner/scripts/object_model_cli.py list\n'
            '  Command:    ros2 service call /aria/command ...\n'
            '═══════════════════════════════════════════════════════\n'
        )],
    )

    # ── Autonomous 15-Agent Multi-Agent Stack (active when synthetic=false) ──
    heavy_perception = GroupAction(
        condition=UnlessCondition(lite_conf),
        actions=[
            depth_node,
            sam2_node,
            grasp_node,
        ],
    )

    heavy_inference = GroupAction(
        condition=UnlessCondition(lite_conf),
        actions=[
            openvla_executor,
            learning_agent,
            evaluation_agent,
        ],
    )

    autonomous_stack = GroupAction(
        condition=UnlessCondition(synthetic_conf),
        actions=[
            # Stage 2: Detection + Depth + IK (heavy nodes gated by `lite`)
            yolo_detection,
            ik_node,
            visual_servo_node,
            heavy_perception,

            # Stage 3: 15 Agents
            vision_agent, depth_agent, tracking_agent,
            affordance_agent, attention_agent,
            planning_agent, reachability_agent,
            skill_agent, control_agent,
            safety_agent, memory_agent,
            world_model_agent,
            heavy_inference,
            dialogue_agent,

            # Orchestration
            task_manager, memory_manager, health_monitor,

            # U3 infrastructure
            activate_agents,
        ],
    )

    lite_notice = LogInfo(
        msg='╔═════════════════════════════════════════════════════════════╗\n'
            '║  🤖 LITE MODE: heavy GPU perception nodes disabled          ║\n'
            '║  Skill agent will use simulator ground truth for poses      ║\n'
            '╚═════════════════════════════════════════════════════════════╝',
        condition=IfCondition(lite_conf),
    )

    # ── Synthetic Demonstration Stack (active when synthetic=true) ───────────
    aria_root = os.environ.get('ARIA_ROOT', '/home/gaminizer/Projects/ARIA')
    synthetic_script = os.path.join(aria_root, 'scripts', 'synthetic_server.py')
    synthetic_stack = GroupAction(
        condition=IfCondition(synthetic_conf),
        actions=[
            LogInfo(msg='╔═════════════════════════════════════════════════════════════╗'),
            LogInfo(msg='║  🤖 ARIA SYNTHETIC DEMONSTRATION MODE ACTIVE                ║'),
            LogInfo(msg='║  Skipping 15 slow agents for rapid physical demonstration   ║'),
            LogInfo(msg='╚═════════════════════════════════════════════════════════════╝'),
            TimerAction(
                period=12.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'python3', synthetic_script,
                            '-w', PythonExpression([
                                "'industrial' if 'industrial' in '", world_conf, "' else 'tester'"
                            ]),
                        ],
                        name='synthetic_runner',
                        output='screen',
                    ),
                ],
            ),
        ],
    )

    return LaunchDescription([
        use_sim,
        dashboard_port,
        bag_mode,
        tracking_backend,
        world,
        gui,
        synthetic,
        lite,

        # Stage 1: Gazebo + Robot + Controllers
        stage1_sim,

        # Camera relay (always on) — required by the dashboard in every mode
        camera_node,

        # Either Autonomous Multi-Agent Stack OR Synthetic Demonstration Suite
        autonomous_stack,
        synthetic_stack,
        lite_notice,

        # Infrastructure
        bag_recorder,
        rosbridge,
        startup_msg,
    ])
