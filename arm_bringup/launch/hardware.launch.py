#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Hardware Launch File — SKELETON
# Complete implementation in Stage 4 hardware prompt.
# DO NOT RUN until ESP32 firmware and serial protocol are ready.
# ═══════════════════════════════════════════════════════════════
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package paths ──────────────────────────────────────
    desc_pkg = get_package_share_directory("arm_description")
    ctrl_pkg = get_package_share_directory("arm_control")

    # ── Launch arguments ───────────────────────────────────
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyUSB0",
        description="Serial port for ESP32 connection"
    )
    calibrate_arg = DeclareLaunchArgument(
        "calibrate", default_value="false",
        description="Run servo calibration routine on startup"
    )

    port = LaunchConfiguration("port")
    calibrate = LaunchConfiguration("calibrate")

    # ── Robot description ──────────────────────────────────
    xacro_file = os.path.join(desc_pkg, "urdf", "aria_arm.urdf.xacro")
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file,
        # Override hardware plugin for real hardware:
        # " use_sim:=false",
    ])

    # ── Robot state publisher ──────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": False,
        }],
        output="screen",
    )

    # ── Controller manager (loads AriaHardwareInterface) ───
    # TODO(Stage 4): Configure controller_manager with hardware interface
    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         {"robot_description": robot_description_content},
    #         os.path.join(ctrl_pkg, "config", "aria_controllers.yaml"),
    #     ],
    #     output="screen",
    # )

    # ── Manual control node ────────────────────────────────
    manual_control = Node(
        package="arm_control",
        executable="manual_control_node.py",
        name="manual_control_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    return LaunchDescription([
        port_arg,
        calibrate_arg,

        LogInfo(msg="═══════════════════════════════════════════════════"),
        LogInfo(msg="  ARIA Hardware Launch — SKELETON (Stage 4)"),
        LogInfo(msg="  DO NOT RUN until ESP32 is connected and"),
        LogInfo(msg="  serial protocol is implemented."),
        LogInfo(msg="═══════════════════════════════════════════════════"),

        robot_state_publisher,
        # controller_manager,  # Uncomment in Stage 4
        manual_control,
    ])
