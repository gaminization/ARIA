#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Simulation Launch File
# Launches Gazebo Classic 11 + robot + controllers
# ═══════════════════════════════════════════════════════════════
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
    SetEnvironmentVariable,
    Shutdown,
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

    world_arg = DeclareLaunchArgument(
        "world", default_value="aria_tester_workspace.world",
        description="World file name or path in arm_bringup/worlds"
    )
    world_conf = LaunchConfiguration("world")

    gui_arg = DeclareLaunchArgument(
        "gui", default_value="false",
        description="Launch Gazebo Classic GUI (set true for GUI window)"
    )
    gui_conf = LaunchConfiguration("gui")

    # ── Robot description (URDF via xacro) ──
    # xacro is expanded before launch substitutions resolve, so the study
    # switches are environment variables, not launch arguments:
    #   ARIA_ENABLE_ATTACH=false  -> no oracle attach plugin
    #   ARIA_ENABLE_CAMERAS=false -> no wrist depth camera (faster control runs)
    xacro_file = os.path.join(desc_pkg, "urdf", "aria_arm.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={
        "enable_attach": os.environ.get("ARIA_ENABLE_ATTACH", "true"),
        "enable_cameras": os.environ.get("ARIA_ENABLE_CAMERAS", "true"),
    })
    urdf_xml = doc.toxml()

    # Strip XML comments and collapse whitespace for gazebo_ros2_control
    import re
    urdf_xml = re.sub(r"<!--.*?-->", "", urdf_xml, flags=re.DOTALL)
    urdf_xml = urdf_xml.replace("package://arm_description", f"file://{desc_pkg}")
    urdf_xml = re.sub(r"\s+", " ", urdf_xml).strip()

    robot_description = {"robot_description": urdf_xml}

    # ── Paths ──────────────────────────────────────────────
    from launch.substitutions import PathJoinSubstitution
    world_file = PathJoinSubstitution([bringup_pkg, "worlds", world_conf])
    controllers_file = os.path.join(ctrl_pkg, "config", "aria_controllers.yaml")
    rviz_config = os.path.join(bringup_pkg, "config", "aria_rviz.rviz")

    # ── Set GAZEBO_MODEL_PATH, GAZEBO_RESOURCE_PATH, and GAZEBO_PLUGIN_PATH ──
    desc_share = os.path.join(get_package_prefix("arm_description"), "share")
    bringup_share = os.path.join(get_package_prefix("arm_bringup"), "share")
    ctrl_lib = os.path.join(get_package_prefix("arm_control"), "lib")
    bringup_models = os.path.join(bringup_pkg, "models")

    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    new_model_path = (
        desc_share + ":" + bringup_models + ":" + bringup_share
        + (":" + existing_model_path if existing_model_path else "")
    )
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=new_model_path,
    )

    existing_resource_path = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    new_resource_path = desc_share + ":" + bringup_share + (":" + existing_resource_path if existing_resource_path else "")
    set_gazebo_resource_path = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=new_resource_path,
    )

    existing_plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    new_plugin_path = f"{ctrl_lib}:/opt/ros/humble/lib" + (f":{existing_plugin_path}" if existing_plugin_path else "")
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=new_plugin_path,
    )

    set_display = SetEnvironmentVariable(
        name="DISPLAY",
        value=os.environ.get("DISPLAY", ":1"),
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
            "verbose": "false",
            "gui": gui_conf,
            "server": "true",
            "extra_gazebo_args": "-s libgazebo_ros_state.so",
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
    # 4. CONTROLLER LOADING (robust, verified bring-up)
    #    Handles transient controller-manager stalls by converging the
    #    controller state machine and verifying via list_controllers,
    #    instead of aborting the launch on a single spawner timeout.
    # ═══════════════════════════════════════════════════════
    load_controllers = Node(
        package="arm_bringup",
        executable="ensure_controllers.py",
        name="ensure_controllers",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "--timeout", "60",
            "--max-attempts", "5",
        ],
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

    def _after_controllers_exit(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg="ERROR: Controllers failed to reach 'active'; refusing to report ARIA ready."),
                Shutdown(reason="controller bring-up failed"),
            ]
        return [
            manual_control,
            LogInfo(msg="=== ARIA Simulation ready ==="),
        ]

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
        world_arg,
        gui_arg,

        # Environment & Core launch
        set_display,
        set_gazebo_model_path,
        set_gazebo_resource_path,
        set_gazebo_plugin_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,

        # Chain: after spawn completes, bring up controllers (verified)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_controllers],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_controllers,
                on_exit=_after_controllers_exit,
            )
        ),

        # Optional RViz
        rviz2,
    ])
