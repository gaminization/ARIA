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

    Reads the bag with ``rosbag2_py`` (real observations, states and
    commands). When ``rosbag2_py`` is unavailable no episodes can be
    extracted and an empty list is returned rather than fabricating data.
    """
    episodes = []
    meta = parse_bag_metadata(bag_path)

    task_count = meta.get('tasks_attempted', 0)
    failure_events = meta.get('failure_events', [])
    n_failures = len(failure_events)
    n_successes = task_count - n_failures

    # Real bag reading requires rosbag2_py. If it is unavailable we
    # cannot extract real observations/actions, so we return nothing
    # rather than fabricating placeholder episodes.
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        return _extract_with_rosbag2(
            bag_path, success_only, target_fps, meta)
    except ImportError:
        print(
            "  ⚠ rosbag2_py unavailable — cannot read the bag. "
            "No episodes extracted (refusing to fabricate data). "
            "Install the ROS 2 rosbag2 stack to convert real bags.")
        return []


# Canonical ARIA joint order (must match the URDF / control stack).
ARIA_JOINT_NAMES = [
    'waist_joint', 'shoulder_joint', 'elbow_joint',
    'wrist_pitch_joint', 'gripper_joint',
]

# Topic that carries *commanded* joint targets (the action signal).
COMMAND_TOPIC = '/aria/joint_stream'
TOP_IMAGE_TOPIC = '/top_camera/image_raw'
WRIST_IMAGE_TOPIC = '/wrist_camera/image_raw'


def _order_joints(msg) -> Optional[np.ndarray]:
    """Return joint positions reordered into canonical ARIA order.

    Falls back to the raw ``position`` order when the message carries no
    joint names. Returns ``None`` when the message does not contain a full
    canonical joint set — we never guess missing joints.
    """
    position = getattr(msg, 'position', None)
    if position is None or len(position) == 0:
        return None
    names = list(getattr(msg, 'name', []) or [])
    if not names:
        if len(position) < len(ARIA_JOINT_NAMES):
            return None
        return np.asarray(
            position[:len(ARIA_JOINT_NAMES)], dtype=np.float32)
    index = {n: i for i, n in enumerate(names)}
    if not all(name in index for name in ARIA_JOINT_NAMES):
        return None
    return np.asarray(
        [position[index[name]] for name in ARIA_JOINT_NAMES],
        dtype=np.float32)


def _imgmsg_to_rgb(msg) -> Optional[np.ndarray]:
    """Convert a ``sensor_msgs/Image`` to an ``(H, W, 3)`` uint8 RGB array.

    Uses ``cv_bridge`` when available and falls back to a manual decode of
    the common raw encodings. Returns ``None`` on failure — no fabricating
    an image.
    """
    try:
        import cv_bridge
        bridge = cv_bridge.CvBridge()
        rgb = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        return np.ascontiguousarray(rgb)
    except Exception:
        pass

    try:
        enc = (getattr(msg, 'encoding', '') or '').lower()
        h, w = int(msg.height), int(msg.width)
        step = int(msg.step) if msg.step else w * 3
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in ('rgb8', 'bgr8'):
            arr = raw.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
            if enc == 'bgr8':
                arr = arr[:, :, ::-1]
            return np.ascontiguousarray(arr)
        if enc in ('mono8', '8uc1'):
            gray = raw.reshape(h, step)[:, :w]
            return np.repeat(gray[:, :, None], 3, axis=2)
    except Exception:
        return None
    return None


def _decode_images(
    records: List[Tuple[int, bytes]],
    msg_type,
    deserialize,
) -> List[Tuple[float, np.ndarray]]:
    """Deserialize image records into ``(timestamp_s, (H, W, 3) rgb)``."""
    out: List[Tuple[float, np.ndarray]] = []
    dropped = 0
    for t, data in records:
        try:
            rgb = _imgmsg_to_rgb(deserialize(data, msg_type))
        except Exception:
            rgb = None
        if rgb is None:
            dropped += 1
            continue
        out.append((t / 1e9, rgb))
    if dropped:
        print(f"  ⚠ Skipped {dropped} undecodable image frame(s)")
    out.sort(key=lambda x: x[0])
    return out


def _nearest_image(
    images: List[Tuple[float, np.ndarray]],
    t: float,
    tolerance: float,
) -> Optional[np.ndarray]:
    """Return the image closest in time to ``t`` within ``tolerance``.

    ``images`` must be sorted by timestamp. Returns ``None`` when no frame
    is close enough (caller decides whether to drop the frame).
    """
    if not images:
        return None
    # Binary search for the insertion point.
    import bisect
    times = [x[0] for x in images]
    i = bisect.bisect_left(times, t)
    best = None
    best_dt = None
    for j in (i - 1, i):
        if 0 <= j < len(images):
            dt = abs(images[j][0] - t)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best = images[j][1]
    if best_dt is not None and best_dt <= tolerance:
        return best
    return None


def _extract_with_rosbag2(
    bag_path: str,
    success_only: bool,
    target_fps: int,
    meta: dict,
) -> List[Episode]:
    """Extract one episode per bag using rosbag2_py.

    Reads:
      * observation state  ← ``/joint_states`` (real measured positions)
      * action             ← ``/aria/joint_stream`` (real recorded commands)
      * observations       ← ``/top_camera/image_raw`` + ``/wrist_camera/image_raw``

    When the bag contains no command topic we fall back to the *next
    observed state* as the action target (the standard imitation-from-
    observation target). That target is a real measurement — never a
    fabricated or zero-filled command — and we say so explicitly so it can
    never be mistaken for a recorded command stream.
    """
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, JointState

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    wanted = {
        '/joint_states': [],
        COMMAND_TOPIC: [],
        TOP_IMAGE_TOPIC: [],
        WRIST_IMAGE_TOPIC: [],
    }
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in wanted:
            wanted[topic].append((t, data))

    # ── Real observed joint states ─────────────────────────
    states: List[Tuple[float, np.ndarray]] = []
    for t, data in wanted['/joint_states']:
        try:
            pos = _order_joints(deserialize_message(data, JointState))
        except Exception:
            pos = None
        if pos is not None:
            states.append((t / 1e9, pos))
    states.sort(key=lambda x: x[0])
    if len(states) < 2:
        print("  ⚠ Bag has no usable /joint_states sequence — "
              "no episodes extracted.")
        return []

    # ── Recorded commands (action) ─────────────────────────
    commands: List[Tuple[float, np.ndarray]] = []
    for t, data in wanted[COMMAND_TOPIC]:
        try:
            pos = _order_joints(deserialize_message(data, JointState))
        except Exception:
            pos = None
        if pos is not None:
            commands.append((t / 1e9, pos))
    commands.sort(key=lambda x: x[0])
    use_recorded_commands = len(commands) > 0
    if not use_recorded_commands:
        print("  ⚠ No recorded command topic "
              f"({COMMAND_TOPIC}) in bag — using next observed state as the "
              "action target (state-derived, NOT a recorded command).")

    # ── Observations ───────────────────────────────────────
    top_images = _decode_images(wanted[TOP_IMAGE_TOPIC], Image,
                                deserialize_message)
    wrist_images = _decode_images(wanted[WRIST_IMAGE_TOPIC], Image,
                                  deserialize_message)
    image_tol = max(0.1, 3.0 / max(target_fps, 1))
    if top_images:
        print(f"  Decoded {len(top_images)} top-camera frame(s)")
    if wrist_images:
        print(f"  Decoded {len(wrist_images)} wrist-camera frame(s)")

    # ── Align frames onto the joint-state timeline ─────────
    episode = Episode(
        episode_id=0,
        images_top=[],
        images_wrist=[],
        joint_states=[],
        joint_commands=[],
        timestamps=[],
        success=len(meta.get('failure_events', [])) == 0,
        task_label=str(meta.get('recording_mode', '')),
        object_class=','.join(meta.get('objects_in_scene', []) or []),
    )

    cmd_i = 0
    for i, (t, state) in enumerate(states):
        # Resolve the action for this frame.
        if use_recorded_commands:
            # Zero-order hold: the controller keeps the last command until
            # a new one arrives.
            while (cmd_i + 1 < len(commands)
                   and commands[cmd_i + 1][0] <= t):
                cmd_i += 1
            if commands[cmd_i][0] > t:
                # No command has been issued yet at this instant.
                continue
            action = commands[cmd_i][1]
        else:
            if i + 1 >= len(states):
                # Last frame has no successor to act toward.
                continue
            action = states[i + 1][1]

        # Resolve observations (only enforced for cameras present in bag).
        top_img = _nearest_image(top_images, t, image_tol) \
            if top_images else None
        wrist_img = _nearest_image(wrist_images, t, image_tol) \
            if wrist_images else None
        if top_images and top_img is None:
            continue
        if wrist_images and wrist_img is None:
            continue

        episode.joint_states.append(state)
        episode.joint_commands.append(action)
        episode.timestamps.append(t)
        if top_img is not None:
            episode.images_top.append(top_img)
        if wrist_img is not None:
            episode.images_wrist.append(wrist_img)

    if not episode.joint_states:
        print("  ⚠ No aligned frames could be extracted from the bag.")
        return []

    if success_only and not episode.success:
        print("  Skipping failed episode (--success-only)")
        return []

    return [episode]


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
