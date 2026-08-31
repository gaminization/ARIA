#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Learning Agent — LifecycleNode
Teleoperation recording (HDF5/LeRobot). Skill performance tracking.
═══════════════════════════════════════════════════════════════
"""
import os, time, datetime
from collections import defaultdict
from typing import Dict
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Image, Imu
from std_srvs.srv import Trigger
from arm_planner.state_bus import StateBus

class LearningAgent(LifecycleNode):
    """
    Learn mode: records teleoperation demonstrations.
    Skill tracking: logs success/failure per skill, detects degradation.

    Recording format: HDF5 (LeRobot-compatible)
    Channels: top_camera, wrist_camera, joint_states, gripper, IMU, task_label
    """

    def __init__(self):
        super().__init__('learning_agent')
        self.bus = StateBus(self)
        self.recording = False
        self.episode_data: Dict[str, list] = {}
        self.episode_start = 0.0
        self.episode_count = 0
        self.skill_performance: Dict[str, list] = defaultdict(list)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("LearningAgent: CONFIGURING")
        self.declare_parameter('dataset_dir', '')
        dd = self.get_parameter('dataset_dir').value
        if not dd:
            dd = os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), '..', 'arm_learning', 'datasets')
        self.dataset_dir = dd
        os.makedirs(self.dataset_dir, exist_ok=True)

        self.start_srv = self.create_service(
            Trigger, '/aria/learn_mode/start', self._start_cb)
        self.stop_srv = self.create_service(
            Trigger, '/aria/learn_mode/stop', self._stop_cb)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=5)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 50)
        self.top_sub = self.create_subscription(
            Image, '/top_camera/image_raw', self._top_cb, qos)
        self.wrist_sub = self.create_subscription(
            Image, '/wrist_camera/image_raw', self._wrist_cb, qos)
        self.imu_sub = self.create_subscription(
            Imu, '/mpu6050/imu_raw', self._imu_cb, qos)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("LearningAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self.recording:
            self._stop_recording()
        return TransitionCallbackReturn.SUCCESS

    def _start_cb(self, request, response):
        if self.recording:
            response.success = False
            response.message = "Already recording"
            return response
        self._start_recording()
        response.success = True
        response.message = f"Recording started (episode #{self.episode_count})"
        return response

    def _stop_cb(self, request, response):
        if not self.recording:
            response.success = False
            response.message = "Not recording"
            return response
        path = self._stop_recording()
        response.success = True
        response.message = f"Episode saved to {path}"
        return response

    def _start_recording(self):
        self.recording = True
        self.episode_start = time.time()
        self.episode_count += 1
        self.episode_data = {
            'timestamps': [], 'joint_positions': [],
            'joint_velocities': [], 'top_camera_frames': 0,
            'wrist_camera_frames': 0, 'imu_samples': 0,
        }
        self.bus.add_chain_of_thought(
            f"LEARNING: 🎬 Recording episode #{self.episode_count}")
        self.get_logger().info("🎬 Learn mode STARTED")

    def _stop_recording(self) -> str:
        self.recording = False
        duration = time.time() - self.episode_start
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"episode_{timestamp}.hdf5"
        filepath = os.path.join(self.dataset_dir, filename)

        # Save as HDF5
        n_frames = len(self.episode_data['timestamps'])
        try:
            import h5py
            with h5py.File(filepath, 'w') as f:
                f.attrs['duration_s'] = duration
                f.attrs['n_frames'] = n_frames
                f.attrs['episode_id'] = self.episode_count
                if self.episode_data['timestamps']:
                    f.create_dataset('timestamps',
                        data=np.array(self.episode_data['timestamps']))
                if self.episode_data['joint_positions']:
                    f.create_dataset('joint_positions',
                        data=np.array(self.episode_data['joint_positions']))
                if self.episode_data['joint_velocities']:
                    f.create_dataset('joint_velocities',
                        data=np.array(self.episode_data['joint_velocities']))
        except ImportError:
            # Fallback: save as numpy
            filepath = filepath.replace('.hdf5', '.npz')
            np.savez(filepath,
                timestamps=np.array(self.episode_data.get('timestamps', [])),
                joint_positions=np.array(self.episode_data.get('joint_positions', [])))

        self.bus.add_chain_of_thought(
            f"LEARNING: 🎬 Episode #{self.episode_count} saved: "
            f"{duration:.1f}s, {n_frames} frames, "
            f"top={self.episode_data['top_camera_frames']}f, "
            f"wrist={self.episode_data['wrist_camera_frames']}f → {filepath}")

        # Auto-analysis
        if self.episode_data['joint_positions']:
            positions = np.array(self.episode_data['joint_positions'])
            smoothness = np.mean(np.abs(np.diff(positions, axis=0)))
            self.bus.add_chain_of_thought(
                f"LEARNING: Analysis — joint smoothness: {smoothness:.4f} rad/sample")

        self.get_logger().info(f"🎬 Episode saved: {filepath}")
        return filepath

    def _joint_cb(self, msg: JointState):
        if not self.recording:
            return
        self.episode_data['timestamps'].append(time.time())
        self.episode_data['joint_positions'].append(list(msg.position))
        self.episode_data['joint_velocities'].append(
            list(msg.velocity) if msg.velocity else [0.0] * len(msg.position))

    def _top_cb(self, msg: Image):
        if self.recording:
            self.episode_data['top_camera_frames'] += 1

    def _wrist_cb(self, msg: Image):
        if self.recording:
            self.episode_data['wrist_camera_frames'] += 1

    def _imu_cb(self, msg: Imu):
        if self.recording:
            self.episode_data['imu_samples'] += 1

    def log_skill_outcome(self, skill_name: str, success: bool,
                          duration: float, failure_reason: str = ''):
        """Log skill execution outcome for performance tracking."""
        self.skill_performance[skill_name].append({
            'success': success, 'duration': duration,
            'failure_reason': failure_reason, 'time': time.time()})

        # Check for degradation (last 20 attempts)
        history = self.skill_performance[skill_name][-20:]
        if len(history) >= 10:
            recent_rate = sum(1 for h in history if h['success']) / len(history)
            if recent_rate < 0.6:
                self.bus.add_chain_of_thought(
                    f"LEARNING: ⚠ Skill '{skill_name}' degrading: "
                    f"{recent_rate:.0%} success (last {len(history)} attempts)")

def main(args=None):
    rclpy.init(args=args)
    node = LearningAgent()
    node.trigger_configure()
    node.trigger_activate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
