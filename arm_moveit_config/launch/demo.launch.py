#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA MoveIt2 — self-contained demo
# robot_state_publisher + static virtual joint TF + move_group +
# optional RViz. Controllers are provided by the sim/hardware bringup.
#
# Usage:
#   ros2 launch arm_moveit_config demo.launch.py
#   ros2 launch arm_moveit_config demo.launch.py use_rviz:=false
# ═══════════════════════════════════════════════════════════════
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_launch = os.path.join(
        get_package_share_directory("arm_moveit_config"), "launch")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz2 with the ARIA MoveIt config")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_launch, "rsp.launch.py")))

    static_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_launch, "static_virtual_joint_tfs.launch.py")))

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_launch, "move_group.launch.py")))

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_launch, "moveit_rviz.launch.py")),
        condition=IfCondition(LaunchConfiguration("use_rviz")))

    return LaunchDescription([use_rviz_arg, rsp, static_tfs, move_group, rviz])
