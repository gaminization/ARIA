#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA MoveIt2 — robot_state_publisher launch
# Publishes the robot description and TF. Start this only when the
# rest of the system is not already running a robot_state_publisher.
# ═══════════════════════════════════════════════════════════════
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("aria_arm", package_name="arm_moveit_config")
        .to_moveit_configs()
    )
    return generate_rsp_launch(moveit_config)
