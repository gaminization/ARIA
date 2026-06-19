#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Simulation Launch File
# Launches Gazebo Harmonic + robot + controllers + RViz
# ═══════════════════════════════════════════════════════════════
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Package paths ──────────────────────────────────────
    desc_pkg = get_package_share_directory("arm_description")
    ctrl_pkg = get_package_share_directory("arm_control")
    bringup_pkg = get_package_share_directory("arm_bringup")

    # ── Launch arguments ───────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz2 visualization"
    )
    use_gz_gui_arg = DeclareLaunchArgument(
        "use_gz_gui", default_value="true",
        description="Launch Gazebo with GUI"
    )
    paused_arg = DeclareLaunchArgument(
        "paused", default_value="false",
        description="Start simulation paused"
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_gz_gui = LaunchConfiguration("use_gz_gui")
    paused = LaunchConfiguration("paused")

    # ── Robot description (URDF via xacro) ─────────────────
    xacro_file = os.path.join(desc_pkg, "urdf", "aria_arm.urdf.xacro")
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file
    ])

    # ── Paths ──────────────────────────────────────────────
    world_file = os.path.join(bringup_pkg, "worlds", "aria_workspace.sdf")
    controllers_file = os.path.join(ctrl_pkg, "config", "aria_controllers.yaml")
    rviz_config = os.path.join(bringup_pkg, "config", "aria_rviz.rviz")

    # ═══════════════════════════════════════════════════════
    # 1. GAZEBO HARMONIC
    # ═══════════════════════════════════════════════════════
    gz_sim = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r", world_file,
            "--render-engine", "ogre2",
        ],
        output="screen",
        additional_env={"GZ_SIM_RESOURCE_PATH": os.path.dirname(world_file)},
    )

    # ═══════════════════════════════════════════════════════
    # 2. ROBOT STATE PUBLISHER
    # ═══════════════════════════════════════════════════════
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": True,
        }],
        output="screen",
    )

    # ═══════════════════════════════════════════════════════
    # 3. SPAWN ROBOT IN GAZEBO
    # ═══════════════════════════════════════════════════════
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "aria_arm",
            "-topic", "/robot_description",
            "-x", "-0.15",
            "-y", "0.0",
            "-z", "0.76",
        ],
        output="screen",
    )

    # ═══════════════════════════════════════════════════════
    # 4-7. CONTROLLER SPAWNERS
    # ═══════════════════════════════════════════════════════
    # Joint state broadcaster (must come first)
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Joint trajectory controller (5 arm joints)
    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Gripper action controller
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_action_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # ═══════════════════════════════════════════════════════
    # 8. MANUAL CONTROL NODE
    # ═══════════════════════════════════════════════════════
    manual_control = Node(
        package="arm_control",
        executable="manual_control_node.py",
        name="manual_control_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ═══════════════════════════════════════════════════════
    # 9. RVIZ2
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
    # 10. ROS-GZ BRIDGE (camera, IMU, contacts → ROS2)
    # ═══════════════════════════════════════════════════════
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/top_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/wrist_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/mpu6050/imu_raw@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/gripper/contact_left@sensor_msgs/msg/ContactsState[gz.msgs.Contacts",
            "/gripper/contact_right@sensor_msgs/msg/ContactsState[gz.msgs.Contacts",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # ═══════════════════════════════════════════════════════
    # LAUNCH SEQUENCE with wait conditions
    # ═══════════════════════════════════════════════════════

    # Delay spawn until Gazebo is up (3 second grace period)
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot],
    )

    # Delay controller spawners until after robot spawn
    delayed_controllers = TimerAction(
        period=6.0,
        actions=[
            jsb_spawner,
            gz_bridge,
        ],
    )

    # Delay arm controllers until JSB is up
    delayed_arm_controllers = TimerAction(
        period=9.0,
        actions=[
            jtc_spawner,
            gripper_spawner,
        ],
    )

    # Delay manual control until controllers are active
    delayed_manual = TimerAction(
        period=12.0,
        actions=[
            manual_control,
            LogInfo(msg="═══ ARIA Simulation ready. Launch keyboard or GUI control. ═══"),
        ],
    )

    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        use_gz_gui_arg,
        paused_arg,

        # Launch sequence
        gz_sim,
        robot_state_publisher,
        delayed_spawn,
        delayed_controllers,
        delayed_arm_controllers,
        delayed_manual,
        rviz2,
    ])
