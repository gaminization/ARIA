#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Agents-Only Launch
Launches robot_state_publisher + all ARIA nodes WITHOUT Gazebo.
Used by aria_start.sh which starts Gazebo manually (gzserver).

Includes:
  - robot_state_publisher
  - joint_state_broadcaster + joint_trajectory_controller
  - detection_node (YOLO — gripper cam)
  - depth_node (Depth-Anything v2 + MiDaS)
  - sam2_node, grasp_node_v2, coordinate_transformer
  - All 15 arm_agents nodes
  - task_manager, memory_manager, health_monitor (arm_planner)
  - bag_recorder_node (arm_learning)
  - Dashboard (arm_dashboard)
═══════════════════════════════════════════════════════════════
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, ExecuteProcess, LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    # ── Arguments ──────────────────────────────────────────
    dashboard_port = DeclareLaunchArgument(
        'dashboard_port', default_value='8000',
        description='Dashboard server port')

    # ── Robot description ───────────────────────────────────
    desc_pkg = get_package_share_directory('arm_description')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'aria_arm.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    import re
    urdf_xml = doc.toxml()
    urdf_xml = re.sub(r'<!--.*?-->', '', urdf_xml, flags=re.DOTALL)
    urdf_xml = urdf_xml.replace('package://arm_description', f'file://{desc_pkg}')
    urdf_xml = re.sub(r'\s+', ' ', urdf_xml).strip()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_xml, 'use_sim_time': True}],
        output='screen',
    )

    # ── Stage 2: Perception nodes (2s) ────────────────────
    # GRIPPER CAMERA ONLY — both subscribe to /wrist_camera/image_raw
    detection_node = TimerAction(period=2.0, actions=[Node(
        package='arm_vision', executable='detection_node',
        name='detection_node', output='screen',
        parameters=[{'camera_topic': '/wrist_camera/image_raw'}],
    )])

    depth_node = TimerAction(period=2.0, actions=[Node(
        package='arm_vision', executable='depth_node',
        name='depth_node', output='screen',
        parameters=[{'camera_topic': '/wrist_camera/image_raw'}],
    )])

    ik_node = TimerAction(period=2.0, actions=[Node(
        package='arm_ik', executable='ik_node',
        name='ik_node', output='screen',
    )])

    visual_servo_node = TimerAction(period=2.5, actions=[Node(
        package='arm_control', executable='visual_servo_node.py',
        name='visual_servo_node', output='screen',
    )])

    # ── Stage 3: 15 Agents (5s) ────────────────────────────
    vision_agent      = TimerAction(period=5.0, actions=[Node(package='arm_agents', executable='vision_agent',      name='vision_agent',      output='screen')])
    depth_agent       = TimerAction(period=5.0, actions=[Node(package='arm_agents', executable='depth_agent',       name='depth_agent',       output='screen')])
    tracking_agent    = TimerAction(period=5.0, actions=[Node(package='arm_agents', executable='tracking_agent',    name='tracking_agent',    output='screen')])
    affordance_agent  = TimerAction(period=5.0, actions=[Node(package='arm_agents', executable='affordance_agent',  name='affordance_agent',  output='screen')])
    attention_agent   = TimerAction(period=5.0, actions=[Node(package='arm_agents', executable='attention_agent',   name='attention_agent',   output='screen')])
    planning_agent    = TimerAction(period=5.5, actions=[Node(package='arm_agents', executable='planning_agent',    name='planning_agent',    output='screen')])
    reachability_agent= TimerAction(period=5.5, actions=[Node(package='arm_agents', executable='reachability_agent',name='reachability_agent',output='screen')])
    skill_agent       = TimerAction(period=6.0, actions=[Node(package='arm_agents', executable='skill_agent',       name='skill_agent',       output='screen')])
    control_agent     = TimerAction(period=6.0, actions=[Node(package='arm_agents', executable='control_agent',     name='control_agent',     output='screen')])
    safety_agent      = TimerAction(period=5.0, actions=[Node(package='arm_agents', executable='safety_agent',      name='safety_agent',      output='screen')])
    memory_agent      = TimerAction(period=5.0, actions=[Node(package='arm_agents', executable='memory_agent',      name='memory_agent',      output='screen')])
    world_model_agent = TimerAction(period=5.5, actions=[Node(package='arm_agents', executable='world_model_agent', name='world_model_agent', output='screen')])
    learning_agent    = TimerAction(period=6.0, actions=[Node(package='arm_agents', executable='learning_agent',    name='learning_agent',    output='screen')])
    evaluation_agent  = TimerAction(period=6.0, actions=[Node(package='arm_agents', executable='evaluation_agent',  name='evaluation_agent',  output='screen')])
    dialogue_agent    = TimerAction(period=6.0, actions=[Node(package='arm_agents', executable='dialogue_agent',    name='dialogue_agent',    output='screen')])

    # ── Orchestration (7s) ─────────────────────────────────
    task_manager   = TimerAction(period=7.0, actions=[Node(package='arm_planner', executable='task_manager',   name='task_manager',   output='screen')])
    memory_manager = TimerAction(period=7.0, actions=[Node(package='arm_planner', executable='memory_manager', name='memory_manager', output='screen')])
    health_monitor = TimerAction(period=7.0, actions=[Node(package='arm_planner', executable='health_monitor', name='health_monitor', output='screen')])

    # ── Bag recorder (8s) ──────────────────────────────────
    bag_recorder = TimerAction(period=8.0, actions=[Node(
        package='arm_learning', executable='bag_recorder_node',
        name='bag_recorder_node', output='screen',
        parameters=[{'auto_start': True, 'default_mode': 'standard'}],
    )])

    # ── Dashboard (9s) ─────────────────────────────────────
    dashboard = TimerAction(
        period=9.0,
        actions=[ExecuteProcess(
            cmd=['python3', os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                '..', '..', '..', '..', 'arm_dashboard', 'app.py'
            )],
            name='dashboard', output='screen',
            additional_env={'ARIA_DASHBOARD_PORT': LaunchConfiguration('dashboard_port')},
        )],
    )

    # ── Agent Activator (8.5s) ─────────────────────────────
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
            '  🤖 ARIA AGENTS ONLINE\n'
            '  Dashboard:  http://localhost:8000\n'
            '  Send cmd:   ros2 service call /aria/command ...\n'
            '═══════════════════════════════════════════════════════\n'
        )],
    )

    return LaunchDescription([
        dashboard_port,
        robot_state_publisher,
        detection_node, depth_node, ik_node, visual_servo_node,
        vision_agent, depth_agent, tracking_agent,
        affordance_agent, attention_agent,
        planning_agent, reachability_agent,
        skill_agent, control_agent,
        safety_agent, memory_agent,
        world_model_agent, learning_agent,
        evaluation_agent, dialogue_agent,
        task_manager, memory_manager, health_monitor,
        activate_agents,
        bag_recorder,
        dashboard,
        startup_msg,
    ])
