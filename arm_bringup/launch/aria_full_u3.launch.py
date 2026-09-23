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
    TimerAction, ExecuteProcess, LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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

    # ── World override (tester workspace by default) ───────
    world = DeclareLaunchArgument(
        'world', default_value='aria_tester_workspace.world',
        description='World file in arm_bringup/worlds')

    world_conf = LaunchConfiguration('world')
    # ═══════════════════════════════════════════════════════
    # STAGE 1: Gazebo + Robot + Controllers (0s)
    # ═══════════════════════════════════════════════════════
    stage1_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'sim.launch.py')),
        launch_arguments={'world': world_conf}.items(),
    )

    # ═══════════════════════════════════════════════════════
    # STAGE 2: Detection + Depth + IK (2s)
    # ═══════════════════════════════════════════════════════
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
        package='arm_agents', executable='planning_agent',
        name='planning_agent', output='screen')])

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

    return LaunchDescription([
        use_sim,
        dashboard_port,
        bag_mode,
        tracking_backend,
        world,   # industrial workcell world arg

        # Stage 1
        stage1_sim,

        # Stage 2
        yolo_detection,
        depth_node,
        ik_node,

        # Stage 3 — agents
        vision_agent, depth_agent, tracking_agent,
        affordance_agent, attention_agent,
        planning_agent, reachability_agent,
        skill_agent, control_agent,
        safety_agent, memory_agent,
        world_model_agent, learning_agent,
        evaluation_agent, dialogue_agent,
        openvla_executor,

        # Orchestration
        task_manager, memory_manager, health_monitor,

        # U3 — infrastructure
        activate_agents,
        bag_recorder,
        rosbridge,
        # dashboard,  # Managed in aria_dash session for dedicated logging & control
        startup_msg,
    ])
