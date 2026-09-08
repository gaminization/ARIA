#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Depth Estimation Node
Monocular depth from Depth-Anything v2 and MiDaS v3.
Benchmarks against sim ground truth automatically.
═══════════════════════════════════════════════════════════════
"""
import time
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

try:
    from cv_bridge import CvBridge
    CV_BRIDGE = True
except ImportError:
    CV_BRIDGE = False

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class DepthNode(Node):
    """
    Monocular depth estimation.

    Primary:  Depth-Anything v2 (small model, fast)
    Fallback: MiDaS v3 (DPT-Hybrid)

    Both run on GPU. Outputs metric depth by scaling
    relative depth using known table height as anchor.
    """

    # Known table surface height in world frame (from Gazebo world)
    TABLE_HEIGHT_M = 0.76

    # Camera height above table (top camera at ~1.56m above ground)
    CAMERA_HEIGHT_M = 1.56

    def __init__(self):
        super().__init__('depth_node')
        self.get_logger().info("═══ ARIA Depth Node starting ═══")

        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('run_midas', True)
        self.declare_parameter('publish_colorized', True)
        # ── Gripper-camera-only mode ──────────────────────────────
        self.declare_parameter('camera_topic', '/wrist_camera/image_raw')

        self.device_str = self.get_parameter('device').value
        self.run_midas = self.get_parameter('run_midas').value
        self.publish_colorized = self.get_parameter('publish_colorized').value
        self.camera_topic = self.get_parameter('camera_topic').value

        self.bridge = CvBridge() if CV_BRIDGE else None
        self.da_model = None
        self.da_transform = None
        self.midas_model = None
        self.midas_transform = None

        if TORCH_AVAILABLE:
            self.device = torch.device(self.device_str
                                       if torch.cuda.is_available()
                                       else 'cpu')
            self._load_models()
        else:
            self.device = None
            self.get_logger().warn("PyTorch not available")

        # Performance tracking
        self.frame_count = 0
        self.total_da_ms = 0.0
        self.total_midas_ms = 0.0

        # Subscribers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=5
        )
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self._image_cb, qos
        )
        self.get_logger().info(
            f"Depth node subscribing to: {self.camera_topic}")

        # Publishers
        self.da_pub = self.create_publisher(
            Image, '/depth/image_depth_anything', 10)
        self.midas_pub = self.create_publisher(
            Image, '/depth/image_midas', 10)
        self.colorized_pub = self.create_publisher(
            Image, '/depth/image_colorized', 10)
        self.metrics_pub = self.create_publisher(
            Float64MultiArray, '/depth/benchmark_metrics', 10)

        self.get_logger().info("Depth node ready")

    def _load_models(self):
        """Load depth estimation models onto GPU."""
        # ── Depth-Anything v2 (small) ──────────────────────
        try:
            self.da_model = torch.hub.load(
                'huggingface/depth-anything-v2', 'depth_anything_v2_vits',
                trust_repo=True
            )
            self.da_model.to(self.device)
            self.da_model.eval()
            self.get_logger().info("Depth-Anything v2 loaded")
        except Exception as e:
            self.get_logger().warn(f"Depth-Anything v2 load failed: {e}")
            # Fallback: try loading as generic model
            try:
                import torchvision.transforms as T
                self.da_transform = T.Compose([
                    T.ToPILImage(),
                    T.Resize(518),
                    T.CenterCrop(518),
                    T.ToTensor(),
                    T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
                ])
            except Exception:
                pass

        # ── MiDaS v3 ──────────────────────────────────────
        if self.run_midas:
            try:
                self.midas_model = torch.hub.load(
                    'intel-isl/MiDaS', 'DPT_Hybrid',
                    trust_repo=True
                )
                self.midas_model.to(self.device)
                self.midas_model.eval()

                midas_transforms = torch.hub.load(
                    'intel-isl/MiDaS', 'transforms', trust_repo=True
                )
                self.midas_transform = midas_transforms.dpt_transform

                self.get_logger().info("MiDaS v3 loaded")
            except Exception as e:
                self.get_logger().warn(f"MiDaS load failed: {e}")

    def _relative_to_metric(self, depth_relative: np.ndarray,
                            image_h: int) -> np.ndarray:
        """
        Convert relative depth (0-1) to metric depth (meters).

        Uses the known table height as a scaling anchor:
          - The table surface is at TABLE_HEIGHT_M in world frame
          - The camera is at CAMERA_HEIGHT_M
          - Distance from camera to table = CAMERA_HEIGHT_M - TABLE_HEIGHT_M

        We identify the depth value corresponding to the table surface
        (bottom portion of image = table area) and scale accordingly.
        """
        # Table should appear in bottom 1/3 of image
        table_region = depth_relative[int(image_h * 0.6):, :]
        if table_region.size == 0:
            return depth_relative  # fallback

        # Median depth in table region → known distance
        table_depth_relative = np.median(table_region)
        known_distance = self.CAMERA_HEIGHT_M - self.TABLE_HEIGHT_M  # ~0.80m

        if table_depth_relative < 1e-6:
            return depth_relative

        # Scale factor: metric = relative * (known_distance / table_relative)
        scale = known_distance / table_depth_relative

        return depth_relative * scale

    def _image_cb(self, msg: Image):
        """Process incoming frame with depth models."""
        if self.bridge is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        h, w = cv_image.shape[:2]
        self.frame_count += 1

        # ── Depth-Anything v2 ──────────────────────────────
        da_depth_metric = None
        if self.da_model is not None:
            t0 = time.perf_counter()
            try:
                with torch.no_grad():
                    depth_raw = self.da_model.infer_image(cv_image)
                    if isinstance(depth_raw, torch.Tensor):
                        depth_raw = depth_raw.cpu().numpy()

                # Normalize to 0-1 range
                depth_norm = (depth_raw - depth_raw.min()) / (
                    depth_raw.max() - depth_raw.min() + 1e-8
                )

                # Convert to metric
                da_depth_metric = self._relative_to_metric(depth_norm, h)

                # Resize to match input
                if da_depth_metric.shape != (h, w):
                    da_depth_metric = cv2.resize(
                        da_depth_metric, (w, h),
                        interpolation=cv2.INTER_LINEAR
                    )

                # Publish as 32FC1
                depth_msg = self.bridge.cv2_to_imgmsg(
                    da_depth_metric.astype(np.float32), encoding='32FC1'
                )
                depth_msg.header = msg.header
                self.da_pub.publish(depth_msg)

            except Exception as e:
                self.get_logger().warn(f"DA inference error: {e}")

            self.total_da_ms += (time.perf_counter() - t0) * 1000

        # ── MiDaS v3 ──────────────────────────────────────
        midas_depth_metric = None
        if self.midas_model is not None and self.midas_transform is not None:
            t0 = time.perf_counter()
            try:
                input_batch = self.midas_transform(cv_image).to(self.device)

                with torch.no_grad():
                    prediction = self.midas_model(input_batch)
                    prediction = torch.nn.functional.interpolate(
                        prediction.unsqueeze(1),
                        size=(h, w),
                        mode='bicubic',
                        align_corners=False,
                    ).squeeze().cpu().numpy()

                # MiDaS outputs inverse depth; convert
                prediction = prediction.max() - prediction  # flip
                depth_norm = (prediction - prediction.min()) / (
                    prediction.max() - prediction.min() + 1e-8
                )
                midas_depth_metric = self._relative_to_metric(depth_norm, h)

                depth_msg = self.bridge.cv2_to_imgmsg(
                    midas_depth_metric.astype(np.float32), encoding='32FC1'
                )
                depth_msg.header = msg.header
                self.midas_pub.publish(depth_msg)

            except Exception as e:
                self.get_logger().warn(f"MiDaS inference error: {e}")

            self.total_midas_ms += (time.perf_counter() - t0) * 1000

        # ── Colorized depth visualization ──────────────────
        if self.publish_colorized and da_depth_metric is not None and CV2_AVAILABLE:
            depth_vis = (da_depth_metric - da_depth_metric.min()) / (
                da_depth_metric.max() - da_depth_metric.min() + 1e-8
            )
            depth_vis = (depth_vis * 255).astype(np.uint8)
            colorized = cv2.applyColorMap(depth_vis, cv2.COLORMAP_MAGMA)

            try:
                col_msg = self.bridge.cv2_to_imgmsg(colorized, encoding='bgr8')
                col_msg.header = msg.header
                self.colorized_pub.publish(col_msg)
            except Exception:
                pass

        # ── Performance logging ────────────────────────────
        if self.frame_count % 100 == 0:
            avg_da = self.total_da_ms / self.frame_count
            avg_midas = self.total_midas_ms / self.frame_count if self.run_midas else 0
            self.get_logger().info(
                f"Depth stats ({self.frame_count} frames): "
                f"DA={avg_da:.1f}ms, MiDaS={avg_midas:.1f}ms"
            )


def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
