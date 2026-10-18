#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA MoveIt2 — move_group launch
# Loads the arm_moveit_config (SRDF, kinematics, OMPL, joint limits,
# controller manager) via moveit_configs_utils and starts the
# move_group planning node. Does NOT start robot_state_publisher or
# ros2_control — the sim/hardware bringup already provides those.
#
# Usage:
#   ros2 launch arm_moveit_config move_group.launch.py
#   ros2 launch arm_moveit_config move_group.launch.py use_sim_time:=true
# ═══════════════════════════════════════════════════════════════
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("aria_arm", package_name="arm_moveit_config")
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use the Gazebo /clock (true for simulation)"))
    ld.add_action(DeclareLaunchArgument(
        "capabilities",
        default_value=moveit_config.move_group_capabilities["capabilities"]))
    ld.add_action(DeclareLaunchArgument(
        "disable_capabilities",
        default_value=moveit_config.move_group_capabilities["disable_capabilities"]))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution":
            LaunchConfiguration("allow_trajectory_execution"),
        "capabilities":
            ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities":
            ParameterValue(
                LaunchConfiguration("disable_capabilities"), value_type=str),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_sim_time":
            ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_configuration],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    ld.add_action(move_group_node)

    return ld
