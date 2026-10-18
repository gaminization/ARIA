#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA MoveIt2 — spawn move_group's controllers
# Spawns the joint_state_broadcaster plus every controller listed in
# config/moveit_controllers.yaml through the ros2_control
# controller_manager. Use this when the controller_manager is already
# running (e.g. after sim.launch.py / hardware.launch.py).
# ═══════════════════════════════════════════════════════════════
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("aria_arm", package_name="arm_moveit_config")
        .to_moveit_configs()
    )
    return generate_spawn_controllers_launch(moveit_config)
