#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA MoveIt2 — static virtual joint TFs
# Publishes the fixed world -> base_link transform declared as a
# virtual joint in config/aria_arm.srdf.
# ═══════════════════════════════════════════════════════════════
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("aria_arm", package_name="arm_moveit_config")
        .to_moveit_configs()
    )
    return generate_static_virtual_joint_tfs_launch(moveit_config)
