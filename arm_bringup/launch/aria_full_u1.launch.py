#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Full System Launch — Upgrade U1 (LLM Planning)
All Stages + LLM Planning Agent + LLM Dialogue Agent

Replaces rule-based planning_agent and dialogue_agent with
LLM-backed versions. Original agents remain in codebase
as fallback (activated automatically if Ollama is down).

Changes from aria_full.launch.py:
  - planning_agent → llm_planning_agent
  - dialogue_agent → llm_dialogue_agent
  - Added /aria/planning/mode topic for monitoring
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
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Arguments ──────────────────────────────────────────
    use_sim = DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='Use Gazebo simulation')
    dashboard_port = DeclareLaunchArgument(
        'dashboard_port', default_value='8080',
        description='Dashboard server port')
    use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 visualization')
    use_gz_gui = DeclareLaunchArgument(
        'use_gz_gui', default_value='true',
        description='Launch Gazebo with GUI')
    paused = DeclareLaunchArgument(
        'paused', default_value='false',
        description='Start simulation paused')

    # ── Package paths ──────────────────────────────────────
    bringup_dir = get_package_share_directory('arm_bringup')

    # ═══════════════════════════════════════════════════════
    # STAGE 1: Gazebo + Robot + Cameras (0s)
    # ═══════════════════════════════════════════════════════
    stage1_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'sim.launch.py')),
    )

    # ═══════════════════════════════════════════════════════
    # STAGE 2: Detection + Depth + IK (2s delay)
    # ═══════════════════════════════════════════════════════
    yolo_detection = TimerAction(
        period=2.0,
        actions=[Node(
            package='arm_vision',
            executable='detection_node',
            name='detection_node',
            output='screen',
        )],
    )

    depth_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='arm_vision',
            executable='depth_node',
            name='depth_node',
            output='screen',
        )],
    )

    ik_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='arm_ik',
            executable='ik_node',
            name='ik_node',
            output='screen',
        )],
    )

    # ═══════════════════════════════════════════════════════
    # STAGE 3: 15 Agents (4s delay for Stage 2 startup)
    # ═══════════════════════════════════════════════════════

    # -- Perception agents --
    vision_agent = TimerAction(period=4.0, actions=[Node(
        package='arm_agents', executable='vision_agent',
        name='vision_agent', output='screen')])

    depth_agent = TimerAction(period=4.0, actions=[Node(
        package='arm_agents', executable='depth_agent',
        name='depth_agent', output='screen')])

    tracking_agent = TimerAction(period=4.0, actions=[Node(
        package='arm_agents', executable='tracking_agent',
        name='tracking_agent', output='screen')])

    affordance_agent = TimerAction(period=4.0, actions=[Node(
        package='arm_agents', executable='affordance_agent',
        name='affordance_agent', output='screen')])

    attention_agent = TimerAction(period=4.0, actions=[Node(
        package='arm_agents', executable='attention_agent',
        name='attention_agent', output='screen')])

    # -- Planning agents --
    # ═══ U1 UPGRADE: LLM Planning Agent replaces rule-based ═══
    llm_planning_agent = TimerAction(period=4.5, actions=[Node(
        package='arm_agents', executable='llm_planning_agent',
        name='llm_planning_agent', output='screen')])

    reachability_agent = TimerAction(period=4.5, actions=[Node(
        package='arm_agents', executable='reachability_agent',
        name='reachability_agent', output='screen')])

    # -- Execution agents --
    skill_agent = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='skill_agent',
        name='skill_agent', output='screen')])

    control_agent = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='control_agent',
        name='control_agent', output='screen')])

    safety_agent = TimerAction(period=4.0, actions=[Node(
        package='arm_agents', executable='safety_agent',
        name='safety_agent', output='screen')])

    # -- Support agents --
    memory_agent = TimerAction(period=4.0, actions=[Node(
        package='arm_agents', executable='memory_agent',
        name='memory_agent', output='screen')])

    world_model_agent = TimerAction(period=4.5, actions=[Node(
        package='arm_agents', executable='world_model_agent',
        name='world_model_agent', output='screen')])

    learning_agent = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='learning_agent',
        name='learning_agent', output='screen')])

    evaluation_agent = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='evaluation_agent',
        name='evaluation_agent', output='screen')])

    # ═══ U1 UPGRADE: LLM Dialogue Agent replaces template-based ═══
    llm_dialogue_agent = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='llm_dialogue_agent',
        name='llm_dialogue_agent', output='screen')])

    # ═══════════════════════════════════════════════════════
    # Task Manager + Memory Manager + Health Monitor (6s)
    # ═══════════════════════════════════════════════════════
    task_manager = TimerAction(period=6.0, actions=[Node(
        package='arm_planner', executable='task_manager',
        name='task_manager', output='screen')])

    memory_manager = TimerAction(period=6.0, actions=[Node(
        package='arm_planner', executable='memory_manager',
        name='memory_manager', output='screen')])

    health_monitor = TimerAction(period=6.0, actions=[Node(
        package='arm_planner', executable='health_monitor',
        name='health_monitor', output='screen')])

    # ═══════════════════════════════════════════════════════
    # Dashboard (8s delay — after all nodes started)
    # ═══════════════════════════════════════════════════════
    dashboard = TimerAction(
        period=8.0,
        actions=[ExecuteProcess(
            cmd=['python3', '-m', 'arm_dashboard.app'],
            name='dashboard',
            output='screen',
            additional_env={
                'ARIA_DASHBOARD_PORT': LaunchConfiguration('dashboard_port'),
            },
        )],
    )

    startup_msg = TimerAction(
        period=9.0,
        actions=[LogInfo(msg='\n'
            '═══════════════════════════════════════════════════════\n'
            '  🤖 ARIA FULL SYSTEM ONLINE — U1 LLM UPGRADE\n'
            '  Planning:  LLM-backed (Ollama) with rule-based fallback\n'
            '  Dialogue:  LLM-enhanced natural language\n'
            '  Dashboard: http://localhost:8080\n'
            '  Command:   ros2 service call /aria/command ...\n'
            '  Mode:      ros2 topic echo /aria/planning/mode\n'
            '  E-Stop:    ros2 service call /aria/estop ...\n'
            '═══════════════════════════════════════════════════════\n'
        )],
    )

    return LaunchDescription([
        use_sim,
        dashboard_port,
        use_rviz,
        use_gz_gui,
        paused,

        # Stage 1
        stage1_sim,

        # Stage 2
        yolo_detection,
        depth_node,
        ik_node,

        # Stage 3 — 15 agents (U1 upgraded)
        vision_agent,
        depth_agent,
        tracking_agent,
        affordance_agent,
        attention_agent,
        llm_planning_agent,    # ← U1: replaces planning_agent
        reachability_agent,
        skill_agent,
        control_agent,
        safety_agent,
        memory_agent,
        world_model_agent,
        learning_agent,
        evaluation_agent,
        llm_dialogue_agent,    # ← U1: replaces dialogue_agent

        # Orchestration
        task_manager,
        memory_manager,
        health_monitor,

        # Dashboard
        dashboard,
        startup_msg,
    ])
