#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Simulation Launch File
# Launches Gazebo Classic 11 + robot + controllers
# Uses separate ros2_control_node to avoid gazebo_ros2_control
# parameter parsing bug in v0.4.x
# ═══════════════════════════════════════════════════════════════
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # ── Package paths ──────────────────────────────────────
    desc_pkg = get_package_share_directory("arm_description")
    ctrl_pkg = get_package_share_directory("arm_control")
    bringup_pkg = get_package_share_directory("arm_bringup")

    # ── Launch arguments ───────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false",
        description="Launch RViz2 visualization"
    )
    use_rviz = LaunchConfiguration("use_rviz")

    # ── Robot description (URDF via xacro) ──
    xacro_file = os.path.join(desc_pkg, "urdf", "aria_arm.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    urdf_xml = doc.toxml()

    # Write URDF to a temp file so it can be referenced without
    # passing through RCL parameter parsing (which chokes on long XML)
    urdf_tmp = os.path.join(tempfile.gettempdir(), "aria_arm.urdf")
    with open(urdf_tmp, "w") as f:
        f.write(urdf_xml)

    robot_description = {"robot_description": urdf_xml}

    # ── Paths ──────────────────────────────────────────────
    world_file = os.path.join(bringup_pkg, "worlds", "aria_workspace.world")
    controllers_file = os.path.join(ctrl_pkg, "config", "aria_controllers.yaml")
    rviz_config = os.path.join(bringup_pkg, "config", "aria_rviz.rviz")

    # ═══════════════════════════════════════════════════════
    # 1. GAZEBO CLASSIC 11
    # ═══════════════════════════════════════════════════════
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world": world_file,
            "verbose": "true",
        }.items(),
    )

    # ═══════════════════════════════════════════════════════
    # 2. ROBOT STATE PUBLISHER
    # ═══════════════════════════════════════════════════════
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ═══════════════════════════════════════════════════════
    # 3. SPAWN ROBOT IN GAZEBO CLASSIC
    # ═══════════════════════════════════════════════════════
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "aria_arm",
        ],
        output="screen",
    )

    # ═══════════════════════════════════════════════════════
    # 4. CONTROLLER LOADING (chained via OnProcessExit)
    # ═══════════════════════════════════════════════════════
    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"],
        output="screen",
    )

    load_jtc = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_trajectory_controller"],
        output="screen",
    )

    load_gripper = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "gripper_action_controller"],
        output="screen",
    )

    # ═══════════════════════════════════════════════════════
    # 5. MANUAL CONTROL NODE
    # ═══════════════════════════════════════════════════════
    manual_control = Node(
        package="arm_control",
        executable="manual_control_node.py",
        name="manual_control_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ═══════════════════════════════════════════════════════
    # 6. RVIZ2 (optional)
    # ═══════════════════════════════════════════════════════
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    # ═══════════════════════════════════════════════════════
    # EVENT CHAIN: spawn -> JSB -> JTC -> gripper -> manual
    # ═══════════════════════════════════════════════════════
    return LaunchDescription([
        use_rviz_arg,

        # Core launch
        gazebo,
        robot_state_publisher,
        spawn_robot,

        # Chain: after spawn completes, load JSB
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_jsb],
            )
        ),
        # Chain: after JSB, load JTC
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_jtc],
            )
        ),
        # Chain: after JTC, load gripper controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jtc,
                on_exit=[load_gripper],
            )
        ),
        # Chain: after gripper controller, start manual control
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_gripper,
                on_exit=[
                    manual_control,
                    LogInfo(msg="=== ARIA Simulation ready ==="),
                ],
            )
        ),

        # Optional RViz
        rviz2,
    ])
