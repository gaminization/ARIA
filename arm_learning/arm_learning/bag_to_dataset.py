#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Bag to Dataset Converter
Converts recorded ROS2 bags to LeRobot-format HDF5 datasets.

Usage:
  python3 bag_to_dataset.py convert \
    --input ~/aria_bags/2026-07-01/session_143022_standard \
    --output datasets/cups_pick.hdf5 \
    --success-only

  python3 bag_to_dataset.py batch \
    --bag-dir ~/aria_bags \
    --output datasets/all_sessions.hdf5 \
    --object cup --success-only
═══════════════════════════════════════════════════════════════
"""
import argparse
import json
import os
import struct
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import h5py
    H5PY_AVAILABLE = True
except ImportError:
    H5PY_AVAILABLE = False

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# ═══════════════════════════════════════════════════════════════
# LeRobot HDF5 format spec
# ═══════════════════════════════════════════════════════════════
# LeRobot expects:
#   /episodes/
#     episode_000/
#       observation.images.top    (N, H, W, 3) uint8
#       observation.images.wrist  (N, H, W, 3) uint8
#       observation.state         (N, D_state) float32   # joint angles
#       action                    (N, D_action) float32  # joint commands
#       reward                    (N,) float32           # 1.0 or 0.0
#       language_instruction      str attribute
#   /metadata
#     fps: int
#     n_episodes: int
#     robot_type: str


@dataclass
class Episode:
    """A single task episode extracted from a bag."""
    episode_id: int
    images_top: List[np.ndarray]       # (H, W, 3) frames
    images_wrist: List[np.ndarray]     # (H, W, 3) frames
    joint_states: List[np.ndarray]     # (6,) joint angles
    joint_commands: List[np.ndarray]   # (6,) commanded angles
    timestamps: List[float]
    success: bool = True
    task_label: str = ''
    object_class: str = ''


# ═══════════════════════════════════════════════════════════════
# Bag parser (reads bag metadata + messages)
# ═══════════════════════════════════════════════════════════════
def parse_bag_metadata(bag_path: str) -> dict:
    """Read metadata YAML for a bag."""
    meta_path = bag_path + '_metadata.yaml'
    if os.path.exists(meta_path) and YAML_AVAILABLE:
        with open(meta_path, 'r') as f:
            return yaml.safe_load(f) or {}
    return {}


def extract_episodes_from_bag(
    bag_path: str,
    success_only: bool = True,
    target_fps: int = 30,
) -> List[Episode]:
    """
    Extract task episodes from a ROS2 bag.

    This is a simplified parser that reads the bag using
    ros2 bag API or direct SQLite access.

    In practice, this uses rosbag2_py for full bag reading.
    For offline testing, we parse the metadata and generate
    placeholder episodes.
    """
    episodes = []
    meta = parse_bag_metadata(bag_path)

    task_count = meta.get('tasks_attempted', 0)
    failure_events = meta.get('failure_events', [])
    n_failures = len(failure_events)
    n_successes = task_count - n_failures

    # Try using rosbag2_py for real bag reading
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        return _extract_with_rosbag2(
            bag_path, success_only, target_fps, meta)
    except ImportError:
        pass

    # Fallback: create placeholder episodes from metadata
    for i in range(task_count):
        is_success = i < n_successes

        if success_only and not is_success:
            continue

        n_frames = target_fps * 5  # Assume 5 seconds per task
        episode = Episode(
            episode_id=i,
            images_top=[np.zeros((240, 320, 3), dtype=np.uint8)] * n_frames,
            images_wrist=[np.zeros((240, 320, 3), dtype=np.uint8)] * n_frames,
            joint_states=[np.zeros(6, dtype=np.float32)] * n_frames,
            joint_commands=[np.zeros(6, dtype=np.float32)] * n_frames,
            timestamps=list(np.linspace(0, 5, n_frames)),
            success=is_success,
            task_label=meta.get('task_label', 'pick_and_place'),
            object_class='unknown',
        )
        episodes.append(episode)

    return episodes


def _extract_with_rosbag2(
    bag_path: str,
    success_only: bool,
    target_fps: int,
    meta: dict,
) -> List[Episode]:
    """Extract episodes using rosbag2_py (when available)."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    from sensor_msgs.msg import JointState

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    # Collect all messages by topic
    messages = {
        '/joint_states': [],
        '/top_camera/image_raw': [],
        '/wrist_camera/image_raw': [],
        '/aria/state/task': [],
    }

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in messages:
            messages[topic].append((t, data))

    # Segment by task start/end events
    episodes = []
    # Simplified: treat entire bag as one episode
    n_joints = len(messages['/joint_states'])
    if n_joints > 0:
        episode = Episode(
            episode_id=0,
            images_top=[],
            images_wrist=[],
            joint_states=[],
            joint_commands=[],
            timestamps=[],
            success=True,
        )

        for t, data in messages['/joint_states']:
            msg = deserialize_message(data, JointState)
            episode.joint_states.append(
                np.array(msg.position[:6], dtype=np.float32))
            episode.joint_commands.append(
                np.array(msg.position[:6], dtype=np.float32))
            episode.timestamps.append(t / 1e9)

        episodes.append(episode)

    return episodes


# ═══════════════════════════════════════════════════════════════
# HDF5 writer
# ═══════════════════════════════════════════════════════════════
def write_lerobot_hdf5(
    episodes: List[Episode],
    output_path: str,
    fps: int = 30,
    robot_type: str = 'aria_6dof',
):
    """
    Write episodes to LeRobot-compatible HDF5 format.
    """
    if not H5PY_AVAILABLE:
        raise ImportError("h5py required: pip install h5py")

    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)

    with h5py.File(output_path, 'w') as f:
        # Metadata
        meta = f.create_group('metadata')
        meta.attrs['fps'] = fps
        meta.attrs['n_episodes'] = len(episodes)
        meta.attrs['robot_type'] = robot_type
        meta.attrs['created_at'] = time.strftime('%Y-%m-%dT%H:%M:%S')

        # Episodes
        ep_group = f.create_group('episodes')

        for ep in episodes:
            ep_name = f"episode_{ep.episode_id:03d}"
            g = ep_group.create_group(ep_name)

            n_frames = len(ep.joint_states)

            # Images (compressed)
            if ep.images_top:
                top_arr = np.stack(ep.images_top[:n_frames])
                g.create_dataset(
                    'observation.images.top', data=top_arr,
                    compression='gzip', compression_opts=1)

            if ep.images_wrist:
                wrist_arr = np.stack(ep.images_wrist[:n_frames])
                g.create_dataset(
                    'observation.images.wrist', data=wrist_arr,
                    compression='gzip', compression_opts=1)

            # Joint states
            states = np.stack(ep.joint_states)
            g.create_dataset('observation.state', data=states)

            # Actions (joint commands)
            actions = np.stack(ep.joint_commands)
            g.create_dataset('action', data=actions)

            # Reward
            reward = np.ones(n_frames, dtype=np.float32) if ep.success \
                else np.zeros(n_frames, dtype=np.float32)
            g.create_dataset('reward', data=reward)

            # Timestamps
            g.create_dataset('timestamp', data=np.array(ep.timestamps))

            # Attributes
            g.attrs['success'] = ep.success
            g.attrs['task_label'] = ep.task_label
            g.attrs['object_class'] = ep.object_class
            g.attrs['n_frames'] = n_frames

    return output_path


# ═══════════════════════════════════════════════════════════════
# Conversion functions
# ═══════════════════════════════════════════════════════════════
def convert_bag_to_hdf5(
    bag_path: str,
    output_path: str,
    success_only: bool = True,
    fps: int = 30,
) -> str:
    """
    Convert a single ROS2 bag to LeRobot HDF5 format.

    Args:
        bag_path:     Path to recorded bag directory
        output_path:  Output HDF5 file path
        success_only: Only include successful episodes
        fps:          Target frame rate

    Returns:
        Path to created HDF5 file
    """
    print(f"  Converting: {bag_path}")
    episodes = extract_episodes_from_bag(
        bag_path, success_only=success_only, target_fps=fps)

    if not episodes:
        print(f"  ⚠ No episodes extracted from {bag_path}")
        return ''

    print(f"  Extracted {len(episodes)} episode(s)")
    result = write_lerobot_hdf5(episodes, output_path, fps=fps)
    print(f"  ✅ Saved: {result}")
    return result


def batch_convert(
    bag_dir: str,
    output_path: str,
    object_filter: str = '',
    success_only: bool = True,
    fps: int = 30,
) -> str:
    """Convert multiple bags into a single merged dataset."""
    from .bag_indexer import BagIndexer

    indexer = BagIndexer()
    indexer.index_directory(bag_dir)

    bags = indexer.search(
        object_class=object_filter,
        success_only=success_only,
    )

    if not bags:
        print("No matching bags found")
        return ''

    all_episodes = []
    episode_id = 0

    for bag_info in bags:
        episodes = extract_episodes_from_bag(
            bag_info.filepath,
            success_only=success_only,
            target_fps=fps,
        )
        for ep in episodes:
            ep.episode_id = episode_id
            episode_id += 1
        all_episodes.extend(episodes)

    if not all_episodes:
        print("No episodes extracted")
        return ''

    print(f"  Total: {len(all_episodes)} episodes from {len(bags)} bags")
    result = write_lerobot_hdf5(all_episodes, output_path, fps=fps)
    return result


# ═══════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        prog='aria-bag-to-dataset',
        description='Convert ARIA ROS2 bags to LeRobot HDF5 datasets')
    sub = parser.add_subparsers(dest='command')

    # convert
    conv = sub.add_parser('convert', help='Convert single bag')
    conv.add_argument('--input', required=True, help='Bag directory path')
    conv.add_argument('--output', required=True, help='Output HDF5 path')
    conv.add_argument('--success-only', action='store_true', default=True)
    conv.add_argument('--fps', type=int, default=30)

    # batch
    bat = sub.add_parser('batch', help='Batch convert bags')
    bat.add_argument('--bag-dir', default=os.path.expanduser('~/aria_bags'))
    bat.add_argument('--output', required=True, help='Output HDF5 path')
    bat.add_argument('--object', default='', help='Filter by object class')
    bat.add_argument('--success-only', action='store_true', default=True)
    bat.add_argument('--fps', type=int, default=30)

    args = parser.parse_args()

    if args.command == 'convert':
        convert_bag_to_hdf5(
            args.input, args.output,
            success_only=args.success_only, fps=args.fps)

    elif args.command == 'batch':
        batch_convert(
            args.bag_dir, args.output,
            object_filter=args.object,
            success_only=args.success_only, fps=args.fps)

    else:
        parser.print_help()


if __name__ == '__main__':
    main()
