#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Perception Launch File
Launches camera, detection, depth, and grasp nodes, plus the
advanced perception stack (SAM2, 6D pose, transparent-object
handling, material recognition, Gaussian-splatting workspace
model and the perception orchestrator).

The advanced stack is gated by launch arguments so the file can
still be used for a lightweight launch, but every module is ON by
default so the full perception pipeline is present whenever the
simulation starts.
═══════════════════════════════════════════════════════════════
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.conditions import IfCondition
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

    # Advanced perception stack (all ON by default — these modules
    # are real implementations, not placeholders).
    use_sam2_arg = DeclareLaunchArgument(
        'use_sam2', default_value='true',
        description='Run SAM2 segmentation (publishes /sam2/masks)'
    )
    use_material_arg = DeclareLaunchArgument(
        'use_material', default_value='true',
        description='Run material recognition (publishes /material/predictions)'
    )
    use_transparent_arg = DeclareLaunchArgument(
        'use_transparent', default_value='true',
        description='Run transparent-object handling (geometric fallback)'
    )
    use_pose6d_arg = DeclareLaunchArgument(
        'use_pose6d', default_value='true',
        description='Run 6D pose estimation (geometry-based PCA)'
    )
    use_gaussian_arg = DeclareLaunchArgument(
        'use_gaussian', default_value='true',
        description='Run Gaussian-splatting workspace model (gated by /gaussian/activate)'
    )
    use_orchestrator_arg = DeclareLaunchArgument(
        'use_orchestrator', default_value='true',
        description='Run perception orchestrator (publishes /perception/unified_scene)'
    )

    use_sam2 = LaunchConfiguration('use_sam2')
    use_material = LaunchConfiguration('use_material')
    use_transparent = LaunchConfiguration('use_transparent')
    use_pose6d = LaunchConfiguration('use_pose6d')
    use_gaussian = LaunchConfiguration('use_gaussian')
    use_orchestrator = LaunchConfiguration('use_orchestrator')

    # ── Camera Node ────────────────────────────────────────
    # Republish the Gazebo camera feeds on the ARIA namespace used by the
    # dashboard. The /aria/* publisher names must NOT be remapped back onto
    # the raw topics, otherwise the relay publishes onto its own inputs and
    # the dashboard never receives a top/wrist frame.
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

    # ── SAM2 Segmentation ──────────────────────────────────
    sam2_node = Node(
        package='arm_vision',
        executable='sam2_node',
        name='sam2_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_sam2),
    )

    # ── Material Recognition ───────────────────────────────
    material_node = Node(
        package='arm_vision',
        executable='material_recognition_node',
        name='material_recognition_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_material),
    )

    # ── Transparent Object Handling ────────────────────────
    transparent_node = Node(
        package='arm_vision',
        executable='transparent_object_node',
        name='transparent_object_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_transparent),
    )

    # ── 6D Pose Estimation ─────────────────────────────────
    pose6d_node = Node(
        package='arm_vision',
        executable='pose_6d_node',
        name='pose_6d_node',
        parameters=[{'auto_trigger': True, 'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_pose6d),
    )

    # ── Gaussian Splatting Workspace Model ─────────────────
    gaussian_node = Node(
        package='arm_vision',
        executable='gaussian_splatting_node',
        name='gaussian_splatting_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_gaussian),
    )

    # ── Perception Orchestrator ────────────────────────────
    orchestrator_node = Node(
        package='arm_vision',
        executable='perception_orchestrator',
        name='perception_orchestrator',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_orchestrator),
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

    # Launch the advanced perception stack after the base GPU nodes.
    # Order matters: SAM2 first (needs YOLO boxes), then material /
    # transparent / pose-6D / gaussian, then the orchestrator.
    delayed_advanced = TimerAction(
        period=4.0,
        actions=[
            sam2_node,
            material_node,
            transparent_node,
            pose6d_node,
            gaussian_node,
            LogInfo(msg="═══ Advanced perception nodes launched ═══"),
        ],
    )

    delayed_orchestrator = TimerAction(
        period=5.0,
        actions=[
            orchestrator_node,
            LogInfo(msg="═══ Perception orchestrator launched ═══"),
        ],
    )

    return LaunchDescription([
        use_depth_arg,
        use_detection_arg,
        yolo_model_arg,
        use_sam2_arg,
        use_material_arg,
        use_transparent_arg,
        use_pose6d_arg,
        use_gaussian_arg,
        use_orchestrator_arg,

        camera_node,
        grasp_node,
        calibration_node,
        delayed_gpu,
        delayed_advanced,
        delayed_orchestrator,
    ])
