#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA — Single-File Master Launch
═══════════════════════════════════════════════════════════════

Runs the ENTIRE Project ARIA stack from one file:

  Stage 1  Gazebo sim + robot + controllers
  Stage 2  Full perception: camera, YOLO, Depth-Anything, SAM2,
           6D pose, transparent-object handling, material
           recognition, Gaussian-splatting workspace model,
           perception orchestrator, grasp synthesis
  Stage 3  MoveIt2 move_group + IK + control additions
           (force estimator, visual servo, grasp executor,
           kinematic calibration, cable scene updater)
  Stage 4  All 15 multi-agent nodes + OpenVLA (optional)
  Stage 5  Orchestration (task / memory / health)
  Stage 6  Infrastructure (bag recorder, agent activator,
           rosbridge, dashboard)

Optional AI components (OpenVLA, FoundationPose, ClearGrasp,
trained ACT) are honestly gated — they report UNAVAILABLE rather
than fabricating actions when weights are absent, so launching
without them is safe.

Usage:
  ros2 launch arm_bringup aria.launch.py
  ros2 launch arm_bringup aria.launch.py gui:=true use_vla:=true
═══════════════════════════════════════════════════════════════
"""
import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    TimerAction, ExecuteProcess, LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index.python.packages import get_package_share_directory


def generate_launch_description():
    # ── Arguments ──────────────────────────────────────────
    use_sim = DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='Use Gazebo simulation')
    world = DeclareLaunchArgument(
        'world', default_value='aria_tester_workspace.world',
        description='World file in arm_bringup/worlds')
    gui = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Launch Gazebo Classic GUI')
    dashboard_port = DeclareLaunchArgument(
        'dashboard_port', default_value='8000',
        description='Dashboard server port')
    bag_mode = DeclareLaunchArgument(
        'bag_mode', default_value='standard',
        description='Bag recording mode: standard, full, or none')
    tracking_backend = DeclareLaunchArgument(
        'tracking_backend', default_value='mlflow',
        description='Experiment tracking backend: mlflow, wandb, none')
    use_moveit = DeclareLaunchArgument(
        'use_moveit', default_value='true',
        description='Launch the MoveIt2 move_group')
    use_advanced_perception = DeclareLaunchArgument(
        'use_advanced_perception', default_value='true',
        description='Launch SAM2, 6D pose, transparent, material, '
                    'Gaussian-splatting and the orchestrator')
    use_vla = DeclareLaunchArgument(
        'use_vla', default_value='false',
        description='Launch the OpenVLA executor (requires model weights)')
    use_bag = DeclareLaunchArgument(
        'use_bag', default_value='true',
        description='Launch the ROS 2 bag recorder')

    world_conf = LaunchConfiguration('world')
    gui_conf = LaunchConfiguration('gui')
    use_moveit_conf = LaunchConfiguration('use_moveit')
    use_adv_conf = LaunchConfiguration('use_advanced_perception')
    use_vla_conf = LaunchConfiguration('use_vla')
    use_bag_conf = LaunchConfiguration('use_bag')

    bringup_dir = get_package_share_directory('arm_bringup')
    moveit_dir = get_package_share_directory('arm_moveit_config')

    # ═══════════════════════════════════════════════════════
    # STAGE 1: Gazebo + Robot + Controllers
    # ═══════════════════════════════════════════════════════
    stage1_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'sim.launch.py')),
        launch_arguments={'world': world_conf, 'gui': gui_conf}.items(),
    )

    # ═══════════════════════════════════════════════════════
    # STAGE 2: Base perception (2s)
    # ═══════════════════════════════════════════════════════
    # Camera relay — republishes the Gazebo camera feeds on the ARIA
    # namespace that the dashboard bridge subscribes to. Do NOT remap its
    # /aria/* publishers back onto the raw topics: that would redirect the
    # relay onto its own inputs, so no /aria feed is ever produced and the
    # dashboard has nothing to display.
    camera_node = TimerAction(period=2.0, actions=[Node(
        package='arm_vision', executable='camera_node',
        name='camera_node', output='screen',
        parameters=[{'mode': 'sim', 'use_sim_time': True}])])

    detection_node = TimerAction(period=2.0, actions=[Node(
        package='arm_vision', executable='detection_node',
        name='detection_node', output='screen',
        parameters=[{
            'model': 'yolov8n.pt',
            'confidence_threshold': 0.4,
            'camera_topic': '/top_camera/image_raw',
        }])])

    depth_node = TimerAction(period=2.0, actions=[Node(
        package='arm_vision', executable='depth_node',
        name='depth_node', output='screen',
        parameters=[{'camera_topic': '/wrist_camera/image_raw'}])])

    ik_node = TimerAction(period=2.0, actions=[Node(
        package='arm_ik', executable='ik_node',
        name='ik_node', output='screen')])

    grasp_node = TimerAction(period=2.0, actions=[Node(
        package='arm_vision', executable='grasp_node',
        name='grasp_node', output='screen')])

    visual_servo_node = TimerAction(period=2.5, actions=[Node(
        package='arm_control', executable='visual_servo_node.py',
        name='visual_servo_node', output='screen')])

    # ═══════════════════════════════════════════════════════
    # STAGE 2b: Advanced perception stack (3s) — SAM2 + 6D pose
    # + transparent handling + material + Gaussian splatting.
    # ═══════════════════════════════════════════════════════
    sam2_node = TimerAction(period=3.0, actions=[Node(
        package='arm_vision', executable='sam2_node',
        name='sam2_node', output='screen',
        condition=IfCondition(use_adv_conf))])

    material_node = TimerAction(period=3.0, actions=[Node(
        package='arm_vision', executable='material_recognition_node',
        name='material_recognition_node', output='screen',
        condition=IfCondition(use_adv_conf))])

    transparent_node = TimerAction(period=3.0, actions=[Node(
        package='arm_vision', executable='transparent_object_node',
        name='transparent_object_node', output='screen',
        condition=IfCondition(use_adv_conf))])

    pose6d_node = TimerAction(period=3.0, actions=[Node(
        package='arm_vision', executable='pose_6d_node',
        name='pose_6d_node', output='screen',
        parameters=[{'auto_trigger': True}],
        condition=IfCondition(use_adv_conf))])

    gaussian_node = TimerAction(period=3.0, actions=[Node(
        package='arm_vision', executable='gaussian_splatting_node',
        name='gaussian_splatting_node', output='screen',
        condition=IfCondition(use_adv_conf))])

    perception_orchestrator = TimerAction(period=4.0, actions=[Node(
        package='arm_vision', executable='perception_orchestrator',
        name='perception_orchestrator', output='screen',
        condition=IfCondition(use_adv_conf))])

    # ═══════════════════════════════════════════════════════
    # STAGE 3: MoveIt2 + control additions (4s)
    # ═══════════════════════════════════════════════════════
    move_group = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_dir, 'launch', 'move_group.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        )],
        condition=IfCondition(use_moveit_conf),
    )

    force_estimator = TimerAction(period=4.0, actions=[Node(
        package='arm_control', executable='force_estimator_node.py',
        name='force_estimator_node', output='screen')])

    grasp_executor = TimerAction(period=4.0, actions=[Node(
        package='arm_control', executable='grasp_executor.py',
        name='grasp_executor', output='screen')])

    kinematic_cal = TimerAction(period=4.0, actions=[Node(
        package='arm_control', executable='kinematic_calibration_node.py',
        name='kinematic_calibration_node', output='screen')])

    cable_updater = TimerAction(period=4.0, actions=[Node(
        package='arm_moveit_config', executable='cable_scene_updater.py',
        name='cable_scene_updater', output='screen')])

    # ═══════════════════════════════════════════════════════
    # STAGE 4: 15 agents (5s) + optional OpenVLA
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

    # OpenVLA — only when requested (needs weights + pinned deps)
    openvla_executor = TimerAction(period=7.0, actions=[Node(
        package='arm_agents', executable='openvla_executor',
        name='openvla_executor', output='screen',
        condition=IfCondition(use_vla_conf))])

    # ═══════════════════════════════════════════════════════
    # STAGE 5: Orchestration (7s)
    # ═══════════════════════════════════════════════════════
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
    # STAGE 6: Infrastructure (8s)
    # ═══════════════════════════════════════════════════════
    bag_recorder = TimerAction(period=8.0, actions=[Node(
        package='arm_learning', executable='bag_recorder_node',
        name='bag_recorder_node', output='screen',
        parameters=[{
            'auto_start': True,
            'default_mode': LaunchConfiguration('bag_mode'),
        }],
        condition=IfCondition(use_bag_conf))])

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

    # rosbridge (optional — dashboard WebSocket works without it)
    try:
        get_package_share_directory('rosbridge_server')
        rosbridge = TimerAction(period=8.0, actions=[Node(
            package='rosbridge_server', executable='rosbridge_websocket',
            name='rosbridge', parameters=[{'port': 9090}],
            output='screen')])
    except Exception:
        rosbridge = LogInfo(msg='rosbridge_server not installed — skipped')

    dashboard = TimerAction(period=9.0, actions=[ExecuteProcess(
        cmd=['python3', '-m', 'arm_dashboard.app'],
        name='dashboard', output='screen',
        additional_env={
            'ARIA_DASHBOARD_PORT': LaunchConfiguration('dashboard_port'),
        })])

    startup_msg = TimerAction(
        period=10.0,
        actions=[LogInfo(msg='\n'
            '═══════════════════════════════════════════════════════\n'
            '  🤖 ARIA FULL SYSTEM ONLINE\n'
            '  Dashboard:  http://localhost:8080\n'
            '  rosbridge:  ws://localhost:9090\n'
            '  Command:    ros2 service call /aria/command ...\n'
            '═══════════════════════════════════════════════════════\n')],
    )

    return LaunchDescription([
        use_sim, world, gui, dashboard_port, bag_mode, tracking_backend,
        use_moveit, use_advanced_perception, use_vla, use_bag,

        stage1_sim,

        camera_node, detection_node, depth_node, ik_node,
        grasp_node, visual_servo_node,

        sam2_node, material_node, transparent_node, pose6d_node,
        gaussian_node, perception_orchestrator,

        move_group, force_estimator, grasp_executor,
        kinematic_cal, cable_updater,

        vision_agent, depth_agent, tracking_agent, affordance_agent,
        attention_agent, planning_agent, reachability_agent,
        skill_agent, control_agent, safety_agent, memory_agent,
        world_model_agent, learning_agent, evaluation_agent,
        dialogue_agent, openvla_executor,

        task_manager, memory_manager, health_monitor,

        bag_recorder, activate_agents, rosbridge, dashboard, startup_msg,
    ])
