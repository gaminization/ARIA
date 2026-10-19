#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
PROJECT ARIA: SYNTHETIC SYSTEM LAUNCH FILE
═══════════════════════════════════════════════════════════════
Launches Gazebo simulation + controllers + synthetic demonstration
runner. Skips the 15 heavy autonomous agents for immediate, fast,
low-latency demonstration to third parties.
═══════════════════════════════════════════════════════════════
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory("arm_bringup")

    # ── Arguments ──────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        "world", default_value="aria_tester_workspace.world",
        description="World file name in arm_bringup/worlds ('aria_tester_workspace.world' or 'aria_industrial_workcell.world')"
    )
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="false",
        description="Launch Gazebo Classic 3D GUI client"
    )
    auto_run_arg = DeclareLaunchArgument(
        "auto_run", default_value="true",
        description="Automatically execute synthetic maneuver suite after controllers initialize"
    )
    dashboard_port_arg = DeclareLaunchArgument(
        "dashboard_port", default_value="8000",
        description="Web dashboard port"
    )

    world_conf = LaunchConfiguration("world")
    gui_conf = LaunchConfiguration("gui")
    auto_run_conf = LaunchConfiguration("auto_run")

    # ── Stage 1: Gazebo Sim + Controllers ───────────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "world": world_conf,
            "gui": gui_conf,
        }.items(),
    )

    # ── Stage 2: Synthetic Server Execution (after 12s bringup)
    aria_root = os.environ.get("ARIA_ROOT", "/home/gaminizer/Projects/ARIA")
    synthetic_script = os.path.join(aria_root, "scripts", "synthetic_server.py")

    # Determine world target flag
    synthetic_process = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg=">>> Launching ARIA Synthetic Fast Demonstration Suite <<<"),
            ExecuteProcess(
                cmd=[
                    "python3", synthetic_script,
                    "-w", PythonExpression([
                        "'industrial' if 'industrial' in '", world_conf, "' else 'tester'"
                    ]),
                ],
                name="synthetic_runner",
                output="screen",
                condition=IfCondition(auto_run_conf),
            ),
        ],
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        auto_run_arg,
        dashboard_port_arg,
        LogInfo(msg="╔═════════════════════════════════════════════════════════════╗"),
        LogInfo(msg="║  🤖 ARIA SYNTHETIC LAUNCHER (RAPID DEMONSTRATION MODE)     ║"),
        LogInfo(msg="║  Skipping 15 slow agents — Direct Physical Execution Online ║"),
        LogInfo(msg="╚═════════════════════════════════════════════════════════════╝"),
        sim_launch,
        synthetic_process,
    ])
