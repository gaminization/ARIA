#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Perception Launch File
Launches camera, detection, depth, and grasp nodes.
═══════════════════════════════════════════════════════════════
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch arguments ───────────────────────────────────
    use_depth_arg = DeclareLaunchArgument(
        'use_depth', default_value='true',
        description='Run depth estimation models'
    )
    use_detection_arg = DeclareLaunchArgument(
        'use_detection', default_value='true',
        description='Run YOLO detection'
    )
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model', default_value='yolov8m.pt',
        description='YOLO model to load'
    )

    # ── Camera Node ────────────────────────────────────────
    camera_node = Node(
        package='arm_vision',
        executable='camera_node',
        name='camera_node',
        parameters=[{'mode': 'sim', 'use_sim_time': True}],
        output='screen',
    )

    # ── Detection Node ─────────────────────────────────────
    detection_node = Node(
        package='arm_vision',
        executable='detection_node',
        name='detection_node',
        parameters=[{
            'model': 'yolov8m.pt',
            'confidence_threshold': 0.5,
            'device': 'cuda:0',
            'half_precision': True,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ── Depth Node ─────────────────────────────────────────
    depth_node = Node(
        package='arm_vision',
        executable='depth_node',
        name='depth_node',
        parameters=[{
            'device': 'cuda:0',
            'run_midas': True,
            'publish_colorized': True,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ── Grasp Node ─────────────────────────────────────────
    grasp_node = Node(
        package='arm_vision',
        executable='grasp_node',
        name='grasp_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── AprilTag Calibration ───────────────────────────────
    calibration_node = Node(
        package='arm_vision',
        executable='apriltag_calibration_node',
        name='apriltag_calibration_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Delay heavy GPU nodes to avoid memory spike
    delayed_gpu = TimerAction(
        period=3.0,
        actions=[
            detection_node,
            depth_node,
            LogInfo(msg="═══ GPU perception nodes launched ═══"),
        ],
    )

    return LaunchDescription([
        use_depth_arg,
        use_detection_arg,
        yolo_model_arg,

        camera_node,
        grasp_node,
        calibration_node,
        delayed_gpu,
    ])
