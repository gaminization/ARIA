#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Manual Control Launch
# RECOMMENDED FIRST-LAUNCH FILE
# Includes sim.launch.py + keyboard_control + joint_slider_gui
# ═══════════════════════════════════════════════════════════════
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory("arm_bringup")

    # ── Include full simulation launch ─────────────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "use_rviz": "true",
            "use_gz_gui": "true",
        }.items(),
    )

    # ── Joint Slider GUI (Tkinter, no terminal needed) ─────
    slider_gui = Node(
        package="arm_control",
        executable="joint_slider_gui.py",
        name="joint_slider_gui",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Delay GUI launch until simulation is fully up
    delayed_gui = TimerAction(
        period=15.0,
        actions=[
            slider_gui,
            LogInfo(msg="═══ ARIA Manual Control ready. ═══"),
            LogInfo(msg="═══ For keyboard control, run in a separate terminal: ═══"),
            LogInfo(msg="═══   ros2 run arm_control keyboard_control.py         ═══"),
        ],
    )

    return LaunchDescription([
        sim_launch,
        delayed_gui,
    ])
