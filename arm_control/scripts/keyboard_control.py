#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Keyboard Control Node
# Terminal-based keyboard control using curses for clean display.
# Sends commands to manual_control_node via services.
# ═══════════════════════════════════════════════════════════════
import curses
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from arm_interfaces.srv import SetJoint, SetAllJoints, GoNamedPose


class KeyboardControlNode(Node):
    """Terminal keyboard control for ARIA arm using curses."""

    JOINT_NAMES = ["waist", "shoulder", "elbow",
                   "wrist_pitch", "wrist_roll", "gripper"]
    JOINT_DISPLAY = ["J1 Waist", "J2 Shoulder", "J3 Elbow",
                     "J4 Wrist Pitch", "J5 Wrist Roll", "J6 Gripper"]

    # Key mappings: (increment_key, decrement_key) for each joint
    KEY_MAP = [
        (ord('q'), ord('a')),  # Joint 1: waist
        (ord('w'), ord('s')),  # Joint 2: shoulder
        (ord('e'), ord('d')),  # Joint 3: elbow
        (ord('r'), ord('f')),  # Joint 4: wrist pitch
        (ord('t'), ord('g')),  # Joint 5: wrist roll
        (ord('y'), ord('h')),  # Joint 6: gripper
    ]

    JOINT_LIMITS_DEG = [
        [-90.0,  90.0],
        [  0.0, 180.0],
        [  0.0, 150.0],
        [-90.0,  90.0],
        [-90.0,  90.0],
        [  0.0,  45.0],
    ]

    NAMED_POSE_ORDER = ["home", "ready", "folded", "inspect"]

    def __init__(self):
        super().__init__("keyboard_control_node")

        # ── State ──────────────────────────────────────────────
        self.current_angles_deg = [0.0, 90.0, 75.0, 0.0, 0.0, 20.0]
        self.target_angles_deg = list(self.current_angles_deg)
        self.step_deg = 5.0
        self.estop_active = False
        self.motion_complete = True
        self.status_text = "INITIALIZING"
        self.current_pose_idx = 0

        # ── Subscriber ────────────────────────────────────────
        self.status_sub = self.create_subscription(
            JointState, "/aria/manual_status",
            self._status_cb, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states",
            self._joint_state_cb, 10
        )

        # ── Service clients ───────────────────────────────────
        self.set_joint_client = self.create_client(
            SetJoint, "/aria/set_joint")
        self.set_all_client = self.create_client(
            SetAllJoints, "/aria/set_all_joints")
        self.named_pose_client = self.create_client(
            GoNamedPose, "/aria/go_named_pose")
        self.estop_client = self.create_client(
            Trigger, "/aria/estop")
        self.release_estop_client = self.create_client(
            Trigger, "/aria/release_estop")
        self.open_gripper_client = self.create_client(
            Trigger, "/aria/open_gripper")
        self.close_gripper_client = self.create_client(
            Trigger, "/aria/close_gripper")

        self.get_logger().info("Keyboard control node initialized")

    def _status_cb(self, msg: JointState):
        """Update status from manual_control_node."""
        if len(msg.position) >= 6:
            self.current_angles_deg = [
                p * 180.0 / math.pi for p in msg.position
            ]
        if len(msg.velocity) >= 6:
            self.target_angles_deg = [
                v * 180.0 / math.pi for v in msg.velocity
            ]
        if len(msg.effort) >= 3:
            self.motion_complete = msg.effort[0] > 0.5
            self.estop_active = msg.effort[1] > 0.5

        if self.estop_active:
            self.status_text = "🔴 E-STOP ACTIVE"
        elif self.motion_complete:
            self.status_text = "AT POSITION"
        else:
            self.status_text = "MOVING..."

    def _joint_state_cb(self, msg: JointState):
        """Fallback: update from direct joint states."""
        joint_names_full = [n + "_joint" for n in self.JOINT_NAMES]
        for i, name in enumerate(joint_names_full):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_angles_deg[i] = msg.position[idx] * 180.0 / math.pi

    def _call_set_joint(self, joint_name, angle_deg, speed=30.0):
        """Call /aria/set_joint service."""
        if not self.set_joint_client.wait_for_service(timeout_sec=0.5):
            return
        req = SetJoint.Request()
        req.joint_name = joint_name
        req.angle_deg = angle_deg
        req.speed_deg_per_s = speed
        self.set_joint_client.call_async(req)

    def _call_named_pose(self, pose_name):
        """Call /aria/go_named_pose service."""
        if not self.named_pose_client.wait_for_service(timeout_sec=0.5):
            return
        req = GoNamedPose.Request()
        req.pose_name = pose_name
        self.named_pose_client.call_async(req)

    def _call_estop(self):
        """Call /aria/estop service."""
        if not self.estop_client.wait_for_service(timeout_sec=0.5):
            return
        self.estop_client.call_async(Trigger.Request())

    def _call_release_estop(self):
        """Call /aria/release_estop service."""
        if not self.release_estop_client.wait_for_service(timeout_sec=0.5):
            return
        self.release_estop_client.call_async(Trigger.Request())

    def run_curses(self, stdscr):
        """Main curses loop."""
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(100)  # 10Hz refresh

        # Colors
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)   # Safe
        curses.init_pair(2, curses.COLOR_YELLOW, -1)   # Warning
        curses.init_pair(3, curses.COLOR_RED, -1)      # Danger/E-stop
        curses.init_pair(4, curses.COLOR_CYAN, -1)     # Header
        curses.init_pair(5, curses.COLOR_WHITE, -1)    # Normal

        running = True
        while running and rclpy.ok():
            try:
                key = stdscr.getch()
            except curses.error:
                key = -1

            # ── Handle key input ──────────────────────────────
            if key == ord('x') or key == 27:  # 'x' or ESC: e-stop
                self._call_estop()
            elif key == ord('c'):  # Clear e-stop
                self._call_release_estop()
            elif key == ord(' '):  # Space: go home
                self._call_named_pose("home")
            elif key == ord('p'):  # Cycle named poses
                pose_name = self.NAMED_POSE_ORDER[self.current_pose_idx]
                self._call_named_pose(pose_name)
                self.current_pose_idx = (
                    (self.current_pose_idx + 1) % len(self.NAMED_POSE_ORDER)
                )
            elif key == ord('1'):
                self.step_deg = 1.0
            elif key == ord('5'):
                self.step_deg = 5.0
            elif key == ord('0'):
                self.step_deg = 15.0
            elif key == ord('Q'):  # Capital Q to quit
                running = False
                continue
            else:
                # Check joint key mappings
                for i, (inc_key, dec_key) in enumerate(self.KEY_MAP):
                    if key == inc_key:
                        new_angle = self.current_angles_deg[i] + self.step_deg
                        self._call_set_joint(
                            self.JOINT_NAMES[i], new_angle, 60.0)
                        break
                    elif key == dec_key:
                        new_angle = self.current_angles_deg[i] - self.step_deg
                        self._call_set_joint(
                            self.JOINT_NAMES[i], new_angle, 60.0)
                        break

            # ── Draw UI ───────────────────────────────────────
            stdscr.clear()
            h, w = stdscr.getmaxyx()
            if h < 20 or w < 42:
                stdscr.addstr(0, 0, "Terminal too small! Need 42x20+")
                stdscr.refresh()
                continue

            box_w = 42

            # Top border
            stdscr.addstr(0, 0, "╔" + "═" * (box_w - 2) + "╗", curses.color_pair(4))

            # Title
            estop_tag = " [E-STOP]" if self.estop_active else " [RUNNING]"
            title = f"  ARIA Keyboard Control{estop_tag}"
            color = curses.color_pair(3) if self.estop_active else curses.color_pair(4)
            stdscr.addstr(1, 0, "║" + title.ljust(box_w - 2) + "║", color)

            # Divider
            stdscr.addstr(2, 0, "╠" + "═" * (box_w - 2) + "╣", curses.color_pair(4))

            # Joint display
            row = 3
            key_labels = ["q/a", "w/s", "e/d", "r/f", "t/g", "y/h"]
            for i in range(6):
                angle = self.current_angles_deg[i]
                limits = self.JOINT_LIMITS_DEG[i]

                # Color based on proximity to limits
                margin = min(angle - limits[0], limits[1] - angle)
                if margin <= 1.0:
                    col = curses.color_pair(3)  # Red: at limit
                elif margin <= 10.0:
                    col = curses.color_pair(2)  # Orange: near limit
                else:
                    col = curses.color_pair(1)  # Green: safe

                line = f"  {self.JOINT_DISPLAY[i]:16s}[{angle:7.1f}°] ← {key_labels[i]}"
                stdscr.addstr(row + i, 0, "║", curses.color_pair(4))
                stdscr.addstr(row + i, 1, line.ljust(box_w - 2), col)
                stdscr.addstr(row + i, box_w - 1, "║", curses.color_pair(4))

            row += 6
            # Divider
            stdscr.addstr(row, 0, "╠" + "═" * (box_w - 2) + "╣", curses.color_pair(4))
            row += 1

            # Step size
            step_line = f"  Step: {self.step_deg:.0f}° [1=fine|5=med|0=coarse]"
            stdscr.addstr(row, 0, "║" + step_line.ljust(box_w - 2) + "║", curses.color_pair(5))
            row += 1

            # Controls
            ctrl_line = "  SPACE:Home  p:Poses  ESC/x:E-Stop"
            stdscr.addstr(row, 0, "║" + ctrl_line.ljust(box_w - 2) + "║", curses.color_pair(5))
            row += 1
            ctrl_line2 = "  c:Release  Q:Quit"
            stdscr.addstr(row, 0, "║" + ctrl_line2.ljust(box_w - 2) + "║", curses.color_pair(5))
            row += 1

            # Divider
            stdscr.addstr(row, 0, "╠" + "═" * (box_w - 2) + "╣", curses.color_pair(4))
            row += 1

            # Status
            status_col = curses.color_pair(3) if self.estop_active else curses.color_pair(1)
            status_line = f"  Status: {self.status_text}"
            stdscr.addstr(row, 0, "║", curses.color_pair(4))
            stdscr.addstr(row, 1, status_line.ljust(box_w - 2), status_col)
            stdscr.addstr(row, box_w - 1, "║", curses.color_pair(4))
            row += 1

            # Bottom border
            stdscr.addstr(row, 0, "╚" + "═" * (box_w - 2) + "╝", curses.color_pair(4))

            stdscr.refresh()

            # Spin ROS2 briefly
            rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()

    # Run ROS2 spinner in background thread
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    spin_thread.start()

    try:
        curses.wrapper(node.run_curses)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Keyboard control shutting down")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
