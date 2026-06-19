#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Hardware Launch File (Complete — Stage 4)
# Launches real hardware: ESP32 + micro-ROS + servo sync +
# cameras + perception + all agents + dashboard
#
# Usage:
#   ros2 launch arm_bringup hardware.launch.py
#   ros2 launch arm_bringup hardware.launch.py port:=/dev/ttyUSB1
#   ros2 launch arm_bringup hardware.launch.py feedback:=commanded
#   ros2 launch arm_bringup hardware.launch.py teach_mode:=true
# ═══════════════════════════════════════════════════════════════
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── Package paths ──────────────────────────────────────
    desc_pkg = get_package_share_directory("arm_description")
    ctrl_pkg = get_package_share_directory("arm_control")
    bringup_pkg = get_package_share_directory("arm_bringup")

    # ── Launch arguments ───────────────────────────────────
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyUSB0",
        description="ESP32 USB serial port"
    )
    feedback_arg = DeclareLaunchArgument(
        "feedback", default_value="adc",
        description="Feedback mode: adc | commanded | none"
    )
    calibrate_arg = DeclareLaunchArgument(
        "calibrate", default_value="false",
        description="Run AprilTag calibration on startup"
    )
    teach_arg = DeclareLaunchArgument(
        "teach_mode", default_value="false",
        description="Start in teach mode (servos relaxed)"
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz2 visualization"
    )
    dashboard_port_arg = DeclareLaunchArgument(
        "dashboard_port", default_value="8080",
        description="Dashboard server port"
    )

    port = LaunchConfiguration("port")
    feedback = LaunchConfiguration("feedback")
    calibrate = LaunchConfiguration("calibrate")
    teach_mode = LaunchConfiguration("teach_mode")
    use_rviz = LaunchConfiguration("use_rviz")

    # ── Robot description ──────────────────────────────────
    xacro_file = os.path.join(desc_pkg, "urdf", "aria_arm.urdf.xacro")
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file
    ])

    rviz_config = os.path.join(bringup_pkg, "config", "aria_rviz.rviz")
    cal_file = os.path.join(ctrl_pkg, "config", "servo_calibration.yaml")

    # ═══════════════════════════════════════════════════════
    # 1. ROBOT STATE PUBLISHER
    # ═══════════════════════════════════════════════════════
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                robot_description_content, value_type=str
            ),
            "use_sim_time": False,
        }],
        output="screen",
    )

    # ═══════════════════════════════════════════════════════
    # 2. MICRO-ROS AGENT (bridges ESP32 ↔ ROS2)
    # ═══════════════════════════════════════════════════════
    micro_ros_agent = ExecuteProcess(
        cmd=[
            "ros2", "run", "micro_ros_agent", "micro_ros_agent",
            "serial", "--dev", port,
        ],
        name="micro_ros_agent",
        output="screen",
    )

    # ═══════════════════════════════════════════════════════
    # 3. SERVO SYNC NODE (the bridge)
    # ═══════════════════════════════════════════════════════
    servo_sync = TimerAction(
        period=3.0,  # Wait for micro-ROS agent
        actions=[Node(
            package="arm_control",
            executable="servo_sync_node",
            name="servo_sync_node",
            output="screen",
            parameters=[{
                "sync_mode": "SIM_TO_REAL",
                "calibration_file": cal_file,
                "record_teach": False,
            }],
        )],
    )

    # ═══════════════════════════════════════════════════════
    # 4. TEACH MODE NODE
    # ═══════════════════════════════════════════════════════
    teach_mode_node = TimerAction(
        period=3.0,
        actions=[Node(
            package="arm_control",
            executable="teach_mode_node",
            name="teach_mode_node",
            output="screen",
        )],
    )

    # ═══════════════════════════════════════════════════════
    # 5. RVIZ2
    # ═══════════════════════════════════════════════════════
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    # ═══════════════════════════════════════════════════════
    # 6. REAL CAMERAS (top + wrist)
    # ═══════════════════════════════════════════════════════
    top_camera = TimerAction(
        period=2.0,
        actions=[Node(
            package="arm_vision",
            executable="camera_node",
            name="top_camera_node",
            output="screen",
            parameters=[{
                "camera_id": 0,
                "width": 1280,
                "height": 720,
                "fps": 30,
                "topic": "/top_camera/image_raw",
            }],
        )],
    )

    wrist_camera = TimerAction(
        period=2.0,
        actions=[Node(
            package="arm_vision",
            executable="camera_node",
            name="wrist_camera_node",
            output="screen",
            parameters=[{
                "camera_id": 1,  # Or ESP32-CAM HTTP stream URL
                "width": 640,
                "height": 480,
                "fps": 15,
                "topic": "/wrist_camera/image_raw",
            }],
        )],
    )

    # ═══════════════════════════════════════════════════════
    # 7. PERCEPTION (YOLO + Depth + Coord Transform)
    # ═══════════════════════════════════════════════════════
    yolo_detection = TimerAction(
        period=4.0,
        actions=[Node(
            package="arm_vision",
            executable="detection_node",
            name="detection_node",
            output="screen",
            parameters=[{
                "model_path": "yolov8n.pt",
                "confidence_threshold": 0.5,
            }],
        )],
    )

    depth_node = TimerAction(
        period=4.0,
        actions=[Node(
            package="arm_vision",
            executable="depth_node",
            name="depth_node",
            output="screen",
        )],
    )

    # ═══════════════════════════════════════════════════════
    # 8. IK SERVICE
    # ═══════════════════════════════════════════════════════
    ik_node = TimerAction(
        period=4.0,
        actions=[Node(
            package="arm_ik",
            executable="ik_node",
            name="ik_service_node",
            output="screen",
        )],
    )

    # ═══════════════════════════════════════════════════════
    # 9. AGENTS (15 agents, staggered launch)
    # ═══════════════════════════════════════════════════════
    agent_names = [
        "vision_agent", "depth_agent", "tracking_agent",
        "affordance_agent", "attention_agent",
        "planning_agent", "reachability_agent",
        "skill_agent", "control_agent", "safety_agent",
        "memory_agent", "world_model_agent",
        "learning_agent", "evaluation_agent", "dialogue_agent",
    ]

    agent_nodes = []
    for i, agent_name in enumerate(agent_names):
        delay = 5.0 + (i // 5) * 0.5  # Stagger in batches
        agent_nodes.append(TimerAction(
            period=delay,
            actions=[Node(
                package="arm_agents",
                executable=agent_name,
                name=agent_name,
                output="screen",
            )],
        ))

    # ═══════════════════════════════════════════════════════
    # 10. ORCHESTRATION (Task Manager, Memory, Health)
    # ═══════════════════════════════════════════════════════
    task_manager = TimerAction(period=7.0, actions=[Node(
        package="arm_planner", executable="task_manager",
        name="task_manager", output="screen")])

    memory_manager = TimerAction(period=7.0, actions=[Node(
        package="arm_planner", executable="memory_manager",
        name="memory_manager", output="screen")])

    health_monitor = TimerAction(period=7.0, actions=[Node(
        package="arm_planner", executable="health_monitor",
        name="health_monitor", output="screen")])

    # ═══════════════════════════════════════════════════════
    # 11. DASHBOARD
    # ═══════════════════════════════════════════════════════
    dashboard = TimerAction(
        period=9.0,
        actions=[ExecuteProcess(
            cmd=["python3", "-m", "arm_dashboard.app"],
            name="dashboard",
            output="screen",
            additional_env={
                "ARIA_DASHBOARD_PORT":
                    LaunchConfiguration("dashboard_port"),
            },
        )],
    )

    # ═══════════════════════════════════════════════════════
    # 12. STARTUP MESSAGE
    # ═══════════════════════════════════════════════════════
    startup_msg = TimerAction(
        period=10.0,
        actions=[LogInfo(msg="\n"
            "═══════════════════════════════════════════════════════\n"
            "  🤖 ARIA HARDWARE SYSTEM ONLINE\n"
            "  Mode:      REAL HARDWARE\n"
            "  Dashboard: http://localhost:8080\n"
            "  Teach:     ros2 service call /aria/teach/start ...\n"
            "  Command:   aria command \"Pick up the red cube\"\n"
            "  E-Stop:    ros2 service call /aria/estop ...\n"
            "═══════════════════════════════════════════════════════\n"
        )],
    )

    # ═══════════════════════════════════════════════════════
    # ASSEMBLE LAUNCH DESCRIPTION
    # ═══════════════════════════════════════════════════════
    ld = LaunchDescription([
        # Arguments
        port_arg,
        feedback_arg,
        calibrate_arg,
        teach_arg,
        use_rviz_arg,
        dashboard_port_arg,

        # Core
        robot_state_publisher,
        micro_ros_agent,
        servo_sync,
        teach_mode_node,
        rviz2,

        # Cameras
        top_camera,
        wrist_camera,

        # Perception
        yolo_detection,
        depth_node,
        ik_node,

        # Orchestration
        task_manager,
        memory_manager,
        health_monitor,

        # Dashboard
        dashboard,
        startup_msg,
    ])

    # Add all agent nodes
    for agent_node in agent_nodes:
        ld.add_action(agent_node)

    return ld
