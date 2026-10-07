#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Industrial Manufacturing Workcell Launch File
# Launches Gazebo Classic 11 with the Industrial Manufacturing World:
# Active Infeed Conveyor, Quality Control Station, Finished Goods Tray,
# Defect Reject Bin, Perimeter Safety Guarding & Control Cabinet.
# ═══════════════════════════════════════════════════════════════
import os
import re

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        description="Launch RViz2 visualization with camera and robot monitors"
    )
    use_rviz = LaunchConfiguration("use_rviz")

    world_arg = DeclareLaunchArgument(
        "world", default_value="aria_industrial_workcell.world",
        description="Industrial world file in arm_bringup/worlds"
    )
    world_conf = LaunchConfiguration("world")

    auto_cycle_arg = DeclareLaunchArgument(
        "auto_cycle", default_value="false",
        description="Automatically start the industrial pick-inspect-sort manufacturing coordinator"
    )
    auto_cycle = LaunchConfiguration("auto_cycle")

    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Launch Gazebo UI window (set false for headless simulation)"
    )
    gui_conf = LaunchConfiguration("gui")

    # ── Robot description (URDF via xacro) ─────────────────
    xacro_file = os.path.join(desc_pkg, "urdf", "aria_arm.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    urdf_xml = doc.toxml()

    # Clean XML comments and resolve package paths
    urdf_xml = re.sub(r"<!--.*?-->", "", urdf_xml, flags=re.DOTALL)
    urdf_xml = urdf_xml.replace("package://arm_description", f"file://{desc_pkg}")
    urdf_xml = re.sub(r"\s+", " ", urdf_xml).strip()

    robot_description = {"robot_description": urdf_xml}

    # ── File Paths ─────────────────────────────────────────
    world_file = PathJoinSubstitution([bringup_pkg, "worlds", world_conf])
    controllers_file = os.path.join(ctrl_pkg, "config", "aria_controllers.yaml")
    rviz_config = os.path.join(bringup_pkg, "config", "aria_rviz.rviz")
    models_dir = os.path.join(bringup_pkg, "models")

    # ── Gazebo Environment Setup ───────────────────────────
    desc_share = os.path.join(get_package_prefix("arm_description"), "share")
    bringup_share = os.path.join(get_package_prefix("arm_bringup"), "share")
    ctrl_lib = os.path.join(get_package_prefix("arm_control"), "lib")

    # GAZEBO_MODEL_PATH: arm_description + bringup models + bringup share
    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    new_model_path = f"{desc_share}:{models_dir}:{bringup_share}" + (f":{existing_model_path}" if existing_model_path else "")
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=new_model_path,
    )

    # GAZEBO_RESOURCE_PATH
    existing_resource_path = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    new_resource_path = f"{desc_share}:{bringup_share}" + (f":{existing_resource_path}" if existing_resource_path else "")
    set_gazebo_resource_path = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=new_resource_path,
    )

    # GAZEBO_PLUGIN_PATH: Ensure libaria_conveyor_plugin.so and libaria_gripper_plugin.so are discovered
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

    set_gazebo_ip = SetEnvironmentVariable(
        name="GAZEBO_IP",
        value="127.0.0.1",
    )

    set_gazebo_master = SetEnvironmentVariable(
        name="GAZEBO_MASTER_URI",
        value="http://127.0.0.1:11345",
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
    # 3. SPAWN ROBOT IN WORKCELL TABLE
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
    # 4. CONTROLLER LOADING (Joint State Broadcaster + JTC)
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
    # 5. MANUAL CONTROL NODE (With Industrial Poses)
    # ═══════════════════════════════════════════════════════
    manual_control = Node(
        package="arm_control",
        executable="manual_control_node",
        name="manual_control_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ═══════════════════════════════════════════════════════
    # 6. OPTIONAL INDUSTRIAL ORCHESTRATOR
    # ═══════════════════════════════════════════════════════
    industrial_coordinator = Node(
        package="arm_bringup",
        executable="industrial_workcell_node.py",
        name="industrial_workcell_node",
        output="screen",
        condition=IfCondition(auto_cycle),
    )

    # ═══════════════════════════════════════════════════════
    # 7. RVIZ2 (Optional)
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
    # EVENT CHAIN: spawn -> JSB -> JTC -> manual_control -> ready
    # ═══════════════════════════════════════════════════════
    return LaunchDescription([
        use_rviz_arg,
        world_arg,
        auto_cycle_arg,
        gui_arg,

        # Environment configuration
        set_display,
        set_gazebo_ip,
        set_gazebo_master,
        set_gazebo_model_path,
        set_gazebo_resource_path,
        set_gazebo_plugin_path,

        # Core simulator & robot nodes
        gazebo,
        robot_state_publisher,
        spawn_robot,

        # Chain: after spawn completes, load JSB with 2.0s settle for controller_manager
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[TimerAction(period=2.0, actions=[load_jsb])],
            )
        ),
        # Chain: after JSB, load JTC
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_jtc],
            )
        ),
        # Chain: after JTC, start manual control and log ready
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jtc,
                on_exit=[
                    manual_control,
                    LogInfo(msg="═════════════════════════════════════════════════════════"),
                    LogInfo(msg="★ ARIA Industrial Manufacturing Workcell Ready!"),
                    LogInfo(msg="  - Active Infeed Conveyor: /aria/conveyor/set_power"),
                    LogInfo(msg="  - Optical Inspection Cameras: /top_camera, /side_camera"),
                    LogInfo(msg="  - Assembly Tray: (0.08, -0.22, 0.614)"),
                    LogInfo(msg="  - Defect Reject Bin: (0.22, -0.18, 0.614)"),
                    LogInfo(msg="═════════════════════════════════════════════════════════"),
                ],
            )
        ),

        # Optional nodes
        industrial_coordinator,
        rviz2,
    ])
