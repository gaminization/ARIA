#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA OpenVLA Executor Node — LifecycleNode
Real-time VLA inference using the gripper (wrist) camera.

This node replaces YOLO-based detection in the manipulation
pipeline when OpenVLA is selected as the active backend.

Architecture:
  /wrist_camera/image_raw ──→ OpenVLA 7B ──→ /aria/vla/joint_deltas
  /aria/state/task         ──→ (instruction)
  /joint_states            ──→ (proprioception)

The node runs at ~2-4 Hz when active (VLA inference ~250-500ms
on RTX 5060 with 8-bit quantization).

When the task status is EXECUTING and vla_mode=True:
  - Each wrist frame + current instruction → OpenVLA → joint deltas
  - Deltas are gated by the ControlAgent before execution
  - CoT stream updated with VLA reasoning per step

Topics:
  Subs: /wrist_camera/image_raw, /aria/state/task, /joint_states,
        /aria/vla/activate (Bool)
  Pubs: /aria/vla/joint_deltas (Float64MultiArray)
        /aria/vla/active (Bool)
        /aria/vla/reasoning (String)
═══════════════════════════════════════════════════════════════
"""
import time
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, Bool, String

from arm_planner.msg import TaskState
from arm_planner.state_bus import StateBus

try:
    import cv2
    CV2_OK = True
except ImportError:
    CV2_OK = False

try:
    from cv_bridge import CvBridge
    BRIDGE_OK = True
except ImportError:
    BRIDGE_OK = False


class OpenVLAExecutorNode(LifecycleNode):
    """
    Runs OpenVLA inference from the gripper camera.
    Acts as the VLA execution engine for ARIA's autonomous mode.
    """

    def __init__(self):
        super().__init__('openvla_executor')
        self.bus = StateBus(self)
        self.cb_group = ReentrantCallbackGroup()

        # VLA interface (lazy-loaded to avoid 15GB in memory at all times)
        self._vla = None
        self._vla_loaded = False
        self._vla_loading = False
        self._vla_lock = threading.Lock()

        # Camera frames
        self._wrist_frame: Optional[np.ndarray] = None
        self._bridge = CvBridge() if BRIDGE_OK else None

        # State
        self._active = False         # VLA mode enabled
        self._instruction = ""       # Current task instruction
        self._joint_positions = np.zeros(5)
        self._gripper_pos = 0.0
        self._last_inference_time = 0.0
        self._inference_hz = 2.0     # target rate (limited by model speed)

    # ── Lifecycle ──────────────────────────────────────────────

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("OpenVLAExecutor: CONFIGURING")

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=2,
        )

        # Subscriptions
        self.create_subscription(
            Image, '/wrist_camera/image_raw', self._wrist_cb, qos,
            callback_group=self.cb_group)
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 50,
            callback_group=self.cb_group)
        self.create_subscription(
            Bool, '/aria/vla/activate', self._activate_cb, 10,
            callback_group=self.cb_group)

        self.bus.on_change('task', self._task_cb)

        # Publishers
        self.delta_pub = self.create_publisher(
            Float64MultiArray, '/aria/vla/joint_deltas', 10)
        self.active_pub = self.create_publisher(
            Bool, '/aria/vla/active', 10)
        self.reasoning_pub = self.create_publisher(
            String, '/aria/vla/reasoning', 10)

        # Inference timer
        self._infer_timer = self.create_timer(
            1.0 / self._inference_hz, self._run_inference,
            callback_group=self.cb_group)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("OpenVLAExecutor: ACTIVATED — loading model in background")
        # Start model loading in background thread (non-blocking)
        t = threading.Thread(target=self._load_vla_background, daemon=True)
        t.start()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("OpenVLAExecutor: DEACTIVATING — unloading VLA")
        self._active = False
        if self._vla is not None:
            try:
                self._vla.unload()
            except Exception:
                pass
        return TransitionCallbackReturn.SUCCESS

    # ── Model loading ──────────────────────────────────────────

    def _load_vla_background(self):
        """Load OpenVLA in background thread (15GB — takes ~30s)."""
        with self._vla_lock:
            if self._vla_loaded or self._vla_loading:
                return
            self._vla_loading = True

        try:
            from arm_vla.vla_interface import VLAInterface
            vla = VLAInterface()
            vla.select_backend("OpenVLA")
            self.get_logger().info("OpenVLAExecutor: Starting OpenVLA-7B load (~30s)...")

            t0 = time.time()
            ok = vla.load()
            dt = time.time() - t0

            if ok:
                with self._vla_lock:
                    self._vla = vla
                    self._vla_loaded = True
                    self._vla_loading = False
                self.get_logger().info(
                    f"OpenVLAExecutor: ✅ OpenVLA-7B ready ({dt:.1f}s)")
                self.bus.add_chain_of_thought(
                    f"[OpenVLA] Model loaded in {dt:.1f}s — VLA inference active")
            else:
                with self._vla_lock:
                    self._vla_loading = False
                self.get_logger().warn(
                    "OpenVLAExecutor: ❌ OpenVLA load failed — model may not be downloaded yet. "
                    "Run: python3 -c \"from huggingface_hub import snapshot_download; "
                    "snapshot_download('openvla/openvla-7b', "
                    "local_dir='models/openvla-7b', local_dir_use_symlinks=False)\"")
        except Exception as e:
            with self._vla_lock:
                self._vla_loading = False
            self.get_logger().error(f"OpenVLAExecutor: Exception loading VLA: {e}")

    # ── Callbacks ──────────────────────────────────────────────

    def _wrist_cb(self, msg: Image):
        """Convert ROS image to numpy array for VLA inference."""
        try:
            if self._bridge:
                self._wrist_frame = self._bridge.imgmsg_to_cv2(
                    msg, desired_encoding='rgb8')
            else:
                # Fallback: manual decode
                import numpy as np
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                frame = arr.reshape((msg.height, msg.width, 3))
                if msg.encoding == 'bgr8':
                    frame = frame[:, :, ::-1]  # BGR→RGB
                self._wrist_frame = frame
        except Exception as e:
            self.get_logger().warn(f"OpenVLAExecutor: Frame decode error: {e}", once=True)

    def _joint_cb(self, msg: JointState):
        """Track current joint positions."""
        names = ["waist_joint", "shoulder_joint", "elbow_joint",
                 "wrist_pitch_joint", "wrist_roll_joint"]
        for i, name in enumerate(names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self._joint_positions[i] = msg.position[idx]

        if "gripper_joint" in msg.name:
            idx = msg.name.index("gripper_joint")
            if idx < len(msg.position):
                # Normalize gripper pos to [0, 1]
                self._gripper_pos = float(np.clip(
                    msg.position[idx] / 1.57, 0.0, 1.0))

    def _task_cb(self, msg: TaskState):
        """Extract instruction from task state."""
        if msg.current_command:
            self._instruction = msg.current_command

        # Auto-activate VLA when task is EXECUTING
        if msg.task_status == 'EXECUTING' and self._vla_loaded:
            if not self._active:
                self.get_logger().info(
                    "OpenVLAExecutor: Auto-activating VLA for EXECUTING task")
                self._active = True

        elif msg.task_status in ('IDLE', 'DONE', 'FAILED'):
            self._active = False

    def _activate_cb(self, msg: Bool):
        """Manual VLA mode toggle."""
        was_active = self._active
        self._active = bool(msg.data)
        if self._active and not was_active:
            self.get_logger().info("OpenVLAExecutor: VLA mode ACTIVATED")
            if not self._vla_loaded and not self._vla_loading:
                t = threading.Thread(target=self._load_vla_background, daemon=True)
                t.start()
        elif not self._active and was_active:
            self.get_logger().info("OpenVLAExecutor: VLA mode DEACTIVATED")

    # ── Inference ──────────────────────────────────────────────

    def _run_inference(self):
        """Run OpenVLA inference at target Hz when active."""
        if not self._active:
            self.active_pub.publish(Bool(data=False))
            return

        if not self._vla_loaded:
            # Still loading — publish status
            loading_msg = (
                "OpenVLA: loading model..." if self._vla_loading
                else "OpenVLA: not loaded (run aria_setup_check.sh)"
            )
            self.reasoning_pub.publish(String(data=loading_msg))
            self.active_pub.publish(Bool(data=False))
            return

        if self._wrist_frame is None:
            self.get_logger().warn("OpenVLAExecutor: No wrist camera frame yet", once=True)
            return

        # Rate limiting (VLA inference is slow — cap at inference_hz)
        now = time.time()
        if now - self._last_inference_time < (1.0 / self._inference_hz):
            return
        self._last_inference_time = now

        # Build VLA input
        from arm_vla.vla_interface import VLAInput
        vla_in = VLAInput(
            wrist_camera_image=self._wrist_frame.copy(),
            joint_positions=self._joint_positions.copy(),
            gripper_position=self._gripper_pos,
            instruction=self._instruction or "pick up the object",
            timestep=int(now * 10) % 10000,
        )

        try:
            vla_out = self._vla.predict(vla_in)
        except Exception as e:
            self.get_logger().error(f"OpenVLAExecutor: Inference error: {e}")
            return

        if vla_out.confidence < 0.1:
            self.get_logger().warn(
                f"OpenVLAExecutor: Low confidence ({vla_out.confidence:.2f}): "
                f"{vla_out.reasoning}")
            return

        # Publish joint deltas [5 joints + gripper]
        delta_msg = Float64MultiArray()
        delta_msg.data = list(vla_out.joint_commands) + [vla_out.gripper_command]
        self.delta_pub.publish(delta_msg)

        # Publish reasoning for CoT display
        reasoning = (
            f"[OpenVLA] {vla_out.reasoning} | "
            f"conf={vla_out.confidence:.2f} | "
            f"Δjoints=[{', '.join(f'{d:.3f}' for d in vla_out.joint_commands)}] "
            f"gripper={vla_out.gripper_command:.2f}"
        )
        self.reasoning_pub.publish(String(data=reasoning))
        self.active_pub.publish(Bool(data=True))

        # Add to CoT stream (every ~5 inferences to avoid spam)
        if int(now * 2) % 5 == 0:
            self.bus.add_chain_of_thought(reasoning)


def main(args=None):
    rclpy.init(args=args)
    node = OpenVLAExecutorNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
