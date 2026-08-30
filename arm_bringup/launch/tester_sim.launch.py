#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Tester Simulation Launch File
# Launches Gazebo Classic 11 with the advanced tester workspace
# (IAI Optical Table + Bullet3 Benchmark Suite + RGB-D HandyBot Perception)
# ═══════════════════════════════════════════════════════════════
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_pkg = get_package_share_directory("arm_bringup")

    # ── Arguments ──────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false",
        description="Launch RViz2 visualization"
    )
    use_rviz = LaunchConfiguration("use_rviz")

    # ── Include core sim launch with tester world ───────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "world": "aria_tester_workspace.world",
            "use_rviz": use_rviz,
            "gui": "true",
        }.items(),
    )

    return LaunchDescription([
        use_rviz_arg,
        LogInfo(msg=">>> Launching ARIA Ultimate Tester Workspace (Bullet3 + IAI Table + RGB-D) <<<"),
        sim_launch,
    ])
