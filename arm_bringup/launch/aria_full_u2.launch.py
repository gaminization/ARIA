#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Full System Launch — Upgrade U2 (Advanced Perception)
All Stages + Enhanced Perception Pipeline

Adds to aria_full.launch.py:
  - SAM2 segmentation node
  - FoundationPose 6D node (on-demand)
  - Transparent object handler
  - Material recognition
  - Gaussian splatting node
  - Perception orchestrator
  - Grasp node v2
  - Affordance agent v2
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
        'dashboard_port', default_value='8080',
        description='Dashboard server port')
    use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 visualization')

    bringup_dir = get_package_share_directory('arm_bringup')

    # ═══════════════════════════════════════════════════════
    # STAGE 1: Gazebo + Robot + Controllers (0s)
    # ═══════════════════════════════════════════════════════
    stage1_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'sim.launch.py')),
    )

    # ═══════════════════════════════════════════════════════
    # STAGE 2: Original Detection + Depth + IK (2s)
    # ═══════════════════════════════════════════════════════
    yolo_detection = TimerAction(
        period=2.0,
        actions=[Node(
            package='arm_vision', executable='detection_node',
            name='detection_node', output='screen',
            parameters=[{'model_path': 'yolov8n.pt',
                         'confidence_threshold': 0.5}],
        )],
    )

    depth_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='arm_vision', executable='depth_node',
            name='depth_node', output='screen',
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
    # U2: Advanced Perception Pipeline (3s delay)
    # ═══════════════════════════════════════════════════════

    # SAM2 — always on, needs YOLO detections
    sam2_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='arm_vision', executable='sam2_node',
            name='sam2_node', output='screen',
        )],
    )

    # Material recognition — always on, needs SAM2 + YOLO
    material_node = TimerAction(
        period=3.5,
        actions=[Node(
            package='arm_vision', executable='material_recognition_node',
            name='material_recognition_node', output='screen',
        )],
    )

    # Transparent object handler — always on, monitors depth
    transparent_node = TimerAction(
        period=3.5,
        actions=[Node(
            package='arm_vision', executable='transparent_object_node',
            name='transparent_object_node', output='screen',
        )],
    )

    # 6D pose — on-demand (activated by orchestrator)
    pose_6d_node = TimerAction(
        period=3.5,
        actions=[Node(
            package='arm_vision', executable='pose_6d_node',
            name='pose_6d_node', output='screen',
            parameters=[{'auto_trigger': True}],
        )],
    )

    # Gaussian splatting — on-demand
    gaussian_node = TimerAction(
        period=3.5,
        actions=[Node(
            package='arm_vision', executable='gaussian_splatting_node',
            name='gaussian_splatting_node', output='screen',
        )],
    )

    # Grasp node v2 (replaces original)
    grasp_node_v2 = TimerAction(
        period=4.0,
        actions=[Node(
            package='arm_vision', executable='grasp_node_v2',
            name='grasp_node_v2', output='screen',
        )],
    )

    # Perception orchestrator — coordinates all modules
    perception_orchestrator = TimerAction(
        period=4.5,
        actions=[Node(
            package='arm_vision', executable='perception_orchestrator',
            name='perception_orchestrator', output='screen',
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

    # U2: Affordance Agent v2
    affordance_agent_v2 = TimerAction(period=5.0, actions=[Node(
        package='arm_agents', executable='affordance_agent_v2',
        name='affordance_agent_v2', output='screen')])

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

    startup_msg = TimerAction(
        period=10.0,
        actions=[LogInfo(msg='\n'
            '═══════════════════════════════════════════════════════\n'
            '  🤖 ARIA FULL SYSTEM ONLINE — U2 ADVANCED PERCEPTION\n'
            '  Perception: SAM2 + FoundationPose + Material + GS\n'
            '  Dashboard:  http://localhost:8080\n'
            '  Perception: ros2 topic echo /perception/mode\n'
            '  VRAM:       ros2 topic echo /perception/vram_usage\n'
            '  Scene:      ros2 topic echo /perception/unified_scene\n'
            '  Command:    ros2 service call /aria/command ...\n'
            '═══════════════════════════════════════════════════════\n'
        )],
    )

    return LaunchDescription([
        use_sim,
        dashboard_port,
        use_rviz,

        # Stage 1
        stage1_sim,

        # Stage 2 — base perception
        yolo_detection,
        depth_node,
        ik_node,

        # U2 — advanced perception
        sam2_node,
        material_node,
        transparent_node,
        pose_6d_node,
        gaussian_node,
        grasp_node_v2,
        perception_orchestrator,

        # Stage 3 — agents (U2 upgraded)
        vision_agent,
        depth_agent,
        tracking_agent,
        affordance_agent_v2,  # ← U2 replaces affordance_agent
        attention_agent,
        planning_agent,
        reachability_agent,
        skill_agent,
        control_agent,
        safety_agent,
        memory_agent,
        world_model_agent,
        learning_agent,
        evaluation_agent,
        dialogue_agent,

        # Orchestration
        task_manager,
        memory_manager,
        health_monitor,

        # Dashboard
        dashboard,
        startup_msg,
    ])
