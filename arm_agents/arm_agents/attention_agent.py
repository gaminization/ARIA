#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Attention Agent — LifecycleNode
Selects perception focus based on task priority.
Drives ROI-based detection and depth computation.
═══════════════════════════════════════════════════════════════
"""
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from sensor_msgs.msg import RegionOfInterest
from arm_planner.msg import TaskState, VisionState
from arm_planner.state_bus import StateBus

# Image dimensions (top camera)
IMG_W, IMG_H = 1280, 720

class AttentionAgent(LifecycleNode):
    """
    Selects what to focus perception resources on.

    Priority:
      ACTIVE_TASK objects: highest (current target)
      RECENTLY_SEEN: medium
      BACKGROUND: low

    Drives:
      /detection/focus_region (ROI)
      /depth/focus_region (ROI)
    """

    def __init__(self):
        super().__init__('attention_agent')
        self.bus = StateBus(self)
        self.focus_target = ''
        self.searching = False

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("AttentionAgent: CONFIGURING")
        self.det_roi_pub = self.create_publisher(
            RegionOfInterest, '/detection/focus_region', 10)
        self.depth_roi_pub = self.create_publisher(
            RegionOfInterest, '/depth/focus_region', 10)
        self.bus.on_change('task', self._on_task)
        self.bus.on_change('vision', self._on_vision)
        self.create_timer(0.1, self._tick)  # 10Hz
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("AttentionAgent: ACTIVATED")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_task(self, msg: TaskState):
        """Update focus based on current task."""
        if msg.task_status == 'EXECUTING' and msg.action_queue:
            for action in msg.action_queue:
                if action.status == 'EXECUTING':
                    self.focus_target = action.target_object
                    self.searching = False
                    break
        elif msg.task_status == 'IDLE':
            self.focus_target = ''
            self.searching = False

    def _on_vision(self, msg: VisionState):
        """Update search mode from vision state."""
        self.searching = msg.search_mode

    def _tick(self):
        """Publish focus ROI at 10Hz."""
        if self.searching or not self.focus_target:
            # Full workspace — no ROI restriction
            roi = RegionOfInterest()
            roi.x_offset = 0
            roi.y_offset = 0
            roi.width = IMG_W
            roi.height = IMG_H
            roi.do_rectify = False
        else:
            # Focus on target object
            vision = self.bus.state.vision
            if vision:
                for det in vision.detected_objects:
                    if (det.class_name == self.focus_target or
                            self.focus_target in det.class_name):
                        # Expand bbox by 50% for context
                        cx, cy = det.bbox_x, det.bbox_y
                        w, h = det.bbox_w * 1.5, det.bbox_h * 1.5
                        roi = RegionOfInterest()
                        roi.x_offset = max(0, int(cx - w / 2))
                        roi.y_offset = max(0, int(cy - h / 2))
                        roi.width = min(IMG_W, int(w))
                        roi.height = min(IMG_H, int(h))
                        roi.do_rectify = False
                        self.det_roi_pub.publish(roi)
                        self.depth_roi_pub.publish(roi)
                        return

            # Object not found — full workspace
            roi = RegionOfInterest()
            roi.x_offset = 0
            roi.y_offset = 0
            roi.width = IMG_W
            roi.height = IMG_H
            roi.do_rectify = False

        self.det_roi_pub.publish(roi)
        self.depth_roi_pub.publish(roi)

def main(args=None):
    rclpy.init(args=args)
    node = AttentionAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
