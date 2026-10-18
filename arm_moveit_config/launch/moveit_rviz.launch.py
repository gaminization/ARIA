#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA MoveIt2 — RViz launch
# Starts RViz2 with the ARIA MoveIt config (config/moveit.rviz).
# ═══════════════════════════════════════════════════════════════
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("aria_arm", package_name="arm_moveit_config")
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
