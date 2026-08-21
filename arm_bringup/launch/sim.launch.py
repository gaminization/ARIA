#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Simulation Launch File
# Launches Gazebo Classic 11 + robot + controllers
# Uses separate ros2_control_node to avoid gazebo_ros2_control
# parameter parsing bug in v0.4.x
# ═══════════════════════════════════════════════════════════════
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

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

    # Strip XML comments and collapse whitespace.
    # gazebo_ros2_control v0.4.x internally passes robot_description
    # as a --param CLI arg; RCL parser fails on large XML with
    # special characters in comments. Stripping comments fixes this.
    import re
    urdf_xml = re.sub(r"<!--.*?-->", "", urdf_xml, flags=re.DOTALL)
    urdf_xml = re.sub(r"\s+", " ", urdf_xml).strip()

    robot_description = {"robot_description": urdf_xml}

    # ── Paths ──────────────────────────────────────────────
    world_file = os.path.join(bringup_pkg, "worlds", "aria_workspace.world")
    controllers_file = os.path.join(ctrl_pkg, "config", "aria_controllers.yaml")
    rviz_config = os.path.join(bringup_pkg, "config", "aria_rviz.rviz")

    # ── Set GAZEBO_MODEL_PATH so Gazebo resolves package:// mesh URIs ──
    # Gazebo Classic converts package://pkg_name/path to model://pkg_name/path
    # and looks in GAZEBO_MODEL_PATH for the pkg_name directory.
    install_share = os.path.dirname(desc_pkg)  # .../install/share
    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    new_model_path = install_share + (":" + existing_model_path if existing_model_path else "")
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=new_model_path,
    )

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
            "pause": "true",  # Start paused — unpause after controllers load
        }.items(),
    )

    # ═══════════════════════════════════════════════════════
    # 2. ROBOT STATE PUBLISHER
    # ═══════════════════════════════════════════════════════
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_xml, "use_sim_time": True}],
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

    # ═══════════════════════════════════════════════════════
    # 5. MANUAL CONTROL NODE
    # ═══════════════════════════════════════════════════════
    manual_control = Node(
        package="arm_control",
        executable="manual_control_node",
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
    # EVENT CHAIN: spawn -> JSB -> JTC -> manual
    # ═══════════════════════════════════════════════════════
    return LaunchDescription([
        use_rviz_arg,

        # Core launch
        set_gazebo_model_path,
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
        # Chain: after JTC, unpause physics then start manual control
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jtc,
                on_exit=[
                    # Unpause Gazebo — controllers are ready to hold the arm
                    ExecuteProcess(
                        cmd=["ros2", "service", "call",
                             "/unpause_physics",
                             "std_srvs/srv/Empty"],
                        output="screen",
                    ),
                    manual_control,
                    LogInfo(msg="=== ARIA Simulation ready ==="),
                ],
            )
        ),

        # Optional RViz
        rviz2,
    ])
