#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
# ARIA Joint Slider GUI
# Tkinter GUI for joint control. NO web dependencies.
# Features: sliders, named pose buttons, 2D arm visualization,
# live status, e-stop button. Thread-safe ROS2 integration.
# ═══════════════════════════════════════════════════════════════
import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from arm_interfaces.srv import SetAllJoints, GoNamedPose


class JointSliderGUI(Node):
    """Tkinter GUI with sliders, 2D arm view, and live status."""

    JOINT_NAMES = ["waist", "shoulder", "elbow",
                   "wrist_pitch", "wrist_roll", "gripper"]
    JOINT_DISPLAY = ["Waist", "Shoulder", "Elbow",
                     "Wrist Pitch", "Wrist Roll", "Gripper"]

    JOINT_LIMITS_DEG = [
        (-90.0,  90.0),
        (  0.0, 180.0),
        (  0.0, 150.0),
        (-90.0,  90.0),
        (-90.0,  90.0),
        (  0.0,  45.0),
    ]

    # Link lengths for 2D visualization (meters, for FK)
    LINK_LENGTHS = [0.070, 0.035, 0.145, 0.115, 0.055, 0.040]

    def __init__(self):
        super().__init__("joint_slider_gui")

        # ── State ──────────────────────────────────────────
        self.current_angles_deg = [0.0, 90.0, 75.0, 0.0, 0.0, 20.0]
        self.target_angles_deg = list(self.current_angles_deg)
        self.estop_active = False
        self.motion_complete = True
        self.status_text = "READY"

        # ── ROS2 subscribers ───────────────────────────────
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states",
            self._joint_state_cb, 10
        )
        self.status_sub = self.create_subscription(
            JointState, "/aria/manual_status",
            self._status_cb, 10
        )

        # ── ROS2 clients ──────────────────────────────────
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

        self.get_logger().info("Joint Slider GUI initialized")

    def _joint_state_cb(self, msg: JointState):
        """Update current positions from joint states."""
        full_names = [n + "_joint" for n in self.JOINT_NAMES]
        for i, name in enumerate(full_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_angles_deg[i] = (
                        msg.position[idx] * 180.0 / math.pi
                    )

    def _status_cb(self, msg: JointState):
        """Update status from manual control node."""
        if len(msg.effort) >= 3:
            self.motion_complete = msg.effort[0] > 0.5
            self.estop_active = msg.effort[1] > 0.5

        if self.estop_active:
            self.status_text = "🔴 E-STOP ACTIVE"
        elif self.motion_complete:
            self.status_text = "AT POSITION"
        else:
            self.status_text = "MOVING..."

    def _send_all_joints(self, angles_deg):
        """Send all joint angles to manual control node."""
        if not self.set_all_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("set_all_joints service not available")
            return
        req = SetAllJoints.Request()
        req.angles_deg = [float(a) for a in angles_deg]
        req.speed_deg_per_s = 30.0
        self.set_all_client.call_async(req)

    def _call_named_pose(self, pose_name):
        """Go to named pose."""
        if not self.named_pose_client.wait_for_service(timeout_sec=0.5):
            return
        req = GoNamedPose.Request()
        req.pose_name = pose_name
        self.named_pose_client.call_async(req)

    def _call_service(self, client):
        """Call a Trigger service."""
        if not client.wait_for_service(timeout_sec=0.5):
            return
        client.call_async(Trigger.Request())


class SliderApp:
    """Tkinter application wrapping the ROS2 node."""

    def __init__(self, ros_node: JointSliderGUI):
        self.node = ros_node

        # ── Root window ────────────────────────────────────
        self.root = tk.Tk()
        self.root.title("ARIA Joint Control")
        self.root.geometry("520x780")
        self.root.resizable(True, True)
        self.root.configure(bg="#2b2b2b")

        # ── Style ──────────────────────────────────────────
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#2b2b2b")
        style.configure("TLabel", background="#2b2b2b", foreground="#e0e0e0",
                         font=("Monospace", 10))
        style.configure("Header.TLabel", font=("Monospace", 12, "bold"),
                         foreground="#00bcd4")
        style.configure("TButton", font=("Monospace", 9))
        style.configure("Estop.TButton", font=("Monospace", 11, "bold"),
                         foreground="#ff0000")
        style.configure("TScale", background="#2b2b2b")

        # ── Main frame ────────────────────────────────────
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # ── Title ──────────────────────────────────────────
        ttk.Label(main_frame, text="─ ARIA Joint Control ─",
                  style="Header.TLabel").pack(pady=(0, 5))

        # ── Sliders ────────────────────────────────────────
        slider_frame = ttk.LabelFrame(main_frame, text="Joints",
                                       padding=5)
        slider_frame.pack(fill=tk.X, pady=5)

        self.sliders = []
        self.slider_vars = []
        self.angle_labels = []

        for i in range(6):
            row = ttk.Frame(slider_frame)
            row.pack(fill=tk.X, pady=2)

            ttk.Label(row, text=f"{self.node.JOINT_DISPLAY[i]:12s}",
                      width=14).pack(side=tk.LEFT)

            var = tk.DoubleVar(value=self.node.current_angles_deg[i])
            self.slider_vars.append(var)

            lo, hi = self.node.JOINT_LIMITS_DEG[i]
            slider = ttk.Scale(row, from_=lo, to=hi,
                               variable=var, orient=tk.HORIZONTAL,
                               length=250)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            self.sliders.append(slider)

            lbl = ttk.Label(row, text=f"{var.get():6.1f}°", width=8)
            lbl.pack(side=tk.LEFT)
            self.angle_labels.append(lbl)

        # ── Buttons row 1: Send + Poses ───────────────────
        btn_frame1 = ttk.Frame(main_frame)
        btn_frame1.pack(fill=tk.X, pady=3)

        ttk.Button(btn_frame1, text="Send to Robot",
                   command=self._on_send).pack(side=tk.LEFT, padx=2)
        for pose in ["Home", "Ready", "Folded", "Inspect"]:
            ttk.Button(btn_frame1, text=pose,
                       command=lambda p=pose.lower(): self._on_pose(p)
                       ).pack(side=tk.LEFT, padx=2)

        # ── Buttons row 2: Gripper + E-stop ───────────────
        btn_frame2 = ttk.Frame(main_frame)
        btn_frame2.pack(fill=tk.X, pady=3)

        ttk.Button(btn_frame2, text="Open Gripper",
                   command=self._on_open_gripper).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame2, text="Close Gripper",
                   command=self._on_close_gripper).pack(side=tk.LEFT, padx=2)

        self.estop_btn = tk.Button(
            btn_frame2, text="🔴 E-STOP", bg="#ff0000", fg="white",
            font=("Monospace", 11, "bold"), relief=tk.RAISED,
            command=self._on_estop, width=10
        )
        self.estop_btn.pack(side=tk.LEFT, padx=5)

        ttk.Button(btn_frame2, text="Release E-Stop",
                   command=self._on_release_estop).pack(side=tk.LEFT, padx=2)

        # ── Live Status ───────────────────────────────────
        status_frame = ttk.LabelFrame(main_frame, text="Live Status",
                                       padding=5)
        status_frame.pack(fill=tk.X, pady=5)

        self.actual_label = ttk.Label(status_frame, text="Actual:  --")
        self.actual_label.pack(anchor=tk.W)
        self.target_label = ttk.Label(status_frame, text="Target:  --")
        self.target_label.pack(anchor=tk.W)
        self.status_label = ttk.Label(status_frame, text="Status:  READY")
        self.status_label.pack(anchor=tk.W)

        # ── 2D Arm View ───────────────────────────────────
        view_frame = ttk.LabelFrame(main_frame, text="2D Side View",
                                     padding=5)
        view_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.canvas = tk.Canvas(view_frame, bg="#1a1a2e", height=250,
                                highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # ── Start update loop ─────────────────────────────
        self._update_gui()

    def _on_send(self):
        """Send current slider values to robot."""
        angles = [var.get() for var in self.slider_vars]
        self.node._send_all_joints(angles)

    def _on_pose(self, pose_name):
        """Go to named pose and update sliders."""
        self.node._call_named_pose(pose_name)

    def _on_open_gripper(self):
        self.node._call_service(self.node.open_gripper_client)

    def _on_close_gripper(self):
        self.node._call_service(self.node.close_gripper_client)

    def _on_estop(self):
        self.node._call_service(self.node.estop_client)

    def _on_release_estop(self):
        self.node._call_service(self.node.release_estop_client)

    def _compute_2d_arm_points(self, angles_deg):
        """
        Compute 2D (side view) arm joint positions using forward kinematics.
        Returns list of (x, y) tuples for each joint in canvas coordinates.
        """
        # Link lengths in pixels (scale from meters)
        scale = 500.0
        base_h = 0.070 * scale
        waist_h = 0.035 * scale
        upper_l = 0.145 * scale
        fore_l = 0.115 * scale
        wrist_l = 0.055 * scale
        grip_l = 0.040 * scale

        canvas_w = self.canvas.winfo_width()
        canvas_h = self.canvas.winfo_height()

        # Base position (center-bottom of canvas)
        bx = canvas_w // 2
        by = canvas_h - 10

        points = [(bx, by)]

        # Top of base
        y = by - base_h
        points.append((bx, y))

        # Top of waist bracket
        y -= waist_h
        points.append((bx, y))

        # Shoulder angle (0°=up, positive=tilt forward)
        angle_sum = -math.radians(angles_deg[1]) + math.pi / 2

        # Upper arm endpoint
        x3 = bx + upper_l * math.cos(angle_sum)
        y3 = y - upper_l * math.sin(angle_sum)
        points.append((x3, y3))

        # Elbow
        angle_sum -= math.radians(angles_deg[2]) - math.pi / 2
        x4 = x3 + fore_l * math.cos(angle_sum)
        y4 = y3 - fore_l * math.sin(angle_sum)
        points.append((x4, y4))

        # Wrist
        angle_sum -= math.radians(angles_deg[3])
        x5 = x4 + wrist_l * math.cos(angle_sum)
        y5 = y4 - wrist_l * math.sin(angle_sum)
        points.append((x5, y5))

        # Gripper
        x6 = x5 + grip_l * math.cos(angle_sum)
        y6 = y5 - grip_l * math.sin(angle_sum)
        points.append((x6, y6))

        return points

    def _draw_2d_view(self):
        """Draw 2D side view of the arm on the canvas."""
        self.canvas.delete("all")

        canvas_w = self.canvas.winfo_width()
        canvas_h = self.canvas.winfo_height()

        if canvas_w < 50 or canvas_h < 50:
            return

        # Draw workspace circle (approximate reach)
        center_x = canvas_w // 2
        center_y = canvas_h - 10 - 0.105 * 500  # top of waist
        reach = 0.330 * 500  # approximate max reach
        self.canvas.create_oval(
            center_x - reach, center_y - reach,
            center_x + reach, center_y + reach,
            outline="#333355", dash=(4, 4), width=1
        )

        # Compute arm points
        try:
            points = self._compute_2d_arm_points(self.node.current_angles_deg)
        except (ValueError, ZeroDivisionError):
            return

        # Colors for link segments
        colors = ["#666666", "#f08c00", "#f08c00",
                  "#f08c00", "#f08c00", "#cccccc"]
        widths = [8, 6, 5, 4, 3, 3]

        # Draw links
        for i in range(len(points) - 1):
            x1, y1 = points[i]
            x2, y2 = points[i + 1]
            color = colors[min(i, len(colors) - 1)]
            width = widths[min(i, len(widths) - 1)]
            self.canvas.create_line(x1, y1, x2, y2,
                                    fill=color, width=width,
                                    capstyle=tk.ROUND)

        # Draw joints
        joint_colors = self._get_joint_colors()
        for i, (x, y) in enumerate(points):
            if i == 0:
                continue  # Skip base bottom
            r = 5 if i < len(points) - 1 else 4
            color = joint_colors[min(i - 1, 5)]
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r,
                fill=color, outline="#ffffff", width=1
            )

        # Draw gripper fingers (simplified)
        if len(points) >= 7:
            gx, gy = points[-1]
            gripper_deg = self.node.current_angles_deg[5]
            spread = gripper_deg * 0.3  # visual spread factor
            for side in [-1, 1]:
                fx = gx + 15 * math.cos(0)
                fy = gy + side * (3 + spread)
                self.canvas.create_line(gx, gy + side * 2, fx, fy,
                                        fill="#3366cc", width=2)

        # Label
        self.canvas.create_text(
            10, 10, text="Side View (YZ plane)",
            fill="#555577", anchor=tk.NW, font=("Monospace", 8)
        )

    def _get_joint_colors(self):
        """Get color for each joint based on proximity to limits."""
        colors = []
        for i in range(6):
            angle = self.node.current_angles_deg[i]
            lo, hi = self.node.JOINT_LIMITS_DEG[i]
            margin = min(angle - lo, hi - angle)
            if margin <= 1.0:
                colors.append("#ff3333")    # Red: at limit
            elif margin <= 10.0:
                colors.append("#ff9933")    # Orange: near limit
            else:
                colors.append("#33cc33")    # Green: safe
        return colors

    def _update_gui(self):
        """Update GUI at ~10Hz. Thread-safe: only reads from node state."""
        # Update slider labels
        for i in range(6):
            angle = self.node.current_angles_deg[i]
            self.angle_labels[i].config(text=f"{angle:6.1f}°")

        # Update status
        actual_str = ", ".join(
            f"{a:5.1f}" for a in self.node.current_angles_deg
        )
        self.actual_label.config(text=f"Actual:  [{actual_str}]")

        target_str = ", ".join(
            f"{a:5.1f}" for a in self.node.target_angles_deg
        )
        self.target_label.config(text=f"Target:  [{target_str}]")

        self.status_label.config(text=f"Status:  {self.node.status_text}")

        # Update E-stop button appearance
        if self.node.estop_active:
            self.estop_btn.config(bg="#aa0000", text="🔴 E-STOP ON")
        else:
            self.estop_btn.config(bg="#ff0000", text="🔴 E-STOP")

        # Update 2D view
        self._draw_2d_view()

        # Schedule next update
        self.root.after(100, self._update_gui)

    def run(self):
        """Start the Tkinter mainloop."""
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    ros_node = JointSliderGUI()

    # Run ROS2 spinner in daemon thread
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(ros_node), daemon=True
    )
    spin_thread.start()

    # Run Tkinter in main thread
    app = SliderApp(ros_node)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.get_logger().info("Joint Slider GUI shutting down")
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
