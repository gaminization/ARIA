#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Full Stage 2 Launch File
Simulation + control + IK + MoveIt2 + perception + grasp executor
═══════════════════════════════════════════════════════════════
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('arm_bringup')

    # ── Stage 1: Simulation + Manual Control ───────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'use_rviz': 'true'}.items(),
    )

    # ── IK Nodes ───────────────────────────────────────────
    ik_node = Node(
        package='arm_ik',
        executable='ik_node',
        name='ik_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    ik_benchmark = Node(
        package='arm_ik',
        executable='ik_benchmark_node',
        name='ik_benchmark_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Cable Scene Updater ────────────────────────────────
    cable_updater = Node(
        package='arm_moveit_config',
        executable='cable_scene_updater.py',
        name='cable_scene_updater',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Control Additions ──────────────────────────────────
    force_estimator = Node(
        package='arm_control',
        executable='force_estimator_node.py',
        name='force_estimator_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    visual_servo = Node(
        package='arm_control',
        executable='visual_servo_node.py',
        name='visual_servo_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    grasp_executor = Node(
        package='arm_control',
        executable='grasp_executor.py',
        name='grasp_executor',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    kinematic_cal = Node(
        package='arm_control',
        executable='kinematic_calibration_node.py',
        name='kinematic_calibration_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Perception Pipeline ────────────────────────────────
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'perception.launch.py')
        ),
    )

    # ── Sequenced Launch ───────────────────────────────────

    # IK nodes after sim is up (12s from sim.launch.py)
    delayed_ik = TimerAction(
        period=14.0,
        actions=[ik_benchmark, ik_node],
    )

    # Control additions after IK
    delayed_control = TimerAction(
        period=18.0,
        actions=[
            force_estimator, visual_servo,
            grasp_executor, kinematic_cal, cable_updater,
        ],
    )

    # Perception after everything else
    delayed_perception = TimerAction(
        period=22.0,
        actions=[
            perception_launch,
            LogInfo(msg="═══════════════════════════════════════"),
            LogInfo(msg="  ARIA Stage 2 — Full Pipeline Ready"),
            LogInfo(msg="  IK + Perception + Grasp Execution"),
            LogInfo(msg="═══════════════════════════════════════"),
        ],
    )

    return LaunchDescription([
        sim_launch,
        delayed_ik,
        delayed_control,
        delayed_perception,
    ])
