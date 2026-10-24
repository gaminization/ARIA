"""ROS 2 helpers for the study. TF failures raise; they never return zeros."""
from __future__ import annotations

import math
import time
from typing import Callable, Dict, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import ModelStates, LinkStates, ContactsState
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


class TFUnavailable(RuntimeError):
    pass


class StudyNode(Node):
    def __init__(self, name: str = "aria_study_node"):
        super().__init__(name)
        self.joint_positions: Dict[str, float] = {}
        self.joint_velocities: Dict[str, float] = {}
        self.joint_efforts: Dict[str, float] = {}
        self.model_poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        self.link_poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        self.link_twists: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        self.last_link_stamp: Optional[float] = None
        self.last_joint_stamp: Optional[float] = None
        self.trajectory_publishes: list = []
        self.left_contacts: list = []
        self.right_contacts: list = []

        self.create_subscription(JointState, "/joint_states", self._joint_cb, 50)
        self.create_subscription(ModelStates, "/gazebo/model_states", self._model_cb, 20)
        self.create_subscription(LinkStates, "/gazebo/link_states", self._link_cb, 20)
        self.create_subscription(ContactsState, "/gripper/contact_left", self._left_contact_cb, 20)
        self.create_subscription(ContactsState, "/gripper/contact_right", self._right_contact_cb, 20)

        import tf2_ros
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_traj = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        # Observe every command published on the trajectory topic, including
        # commands from SafetyAgent. Used for dispatch-time measurement.
        self.create_subscription(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            self._traj_cb,
            50,
        )
        self.pub_cmd_vel = self.create_publisher(Twist, "/aria/conveyor/cmd_vel", 10)
        self.cli_set_state = self.create_client(SetEntityState, "/gazebo/set_entity_state")

    # ── callbacks ──────────────────────────────────────────────────────────
    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = float(msg.position[i])
            if i < len(msg.velocity):
                self.joint_velocities[name] = float(msg.velocity[i])
            if i < len(msg.effort):
                self.joint_efforts[name] = float(msg.effort[i])
        self.last_joint_stamp = time.monotonic()

    def _model_cb(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            p = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            q = np.array(
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                dtype=float,
            )
            self.model_poses[name] = (p, q)

    def _link_cb(self, msg: LinkStates):
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            p = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            q = np.array(
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                dtype=float,
            )
            v = np.array([twist.linear.x, twist.linear.y, twist.linear.z], dtype=float)
            w = np.array([twist.angular.x, twist.angular.y, twist.angular.z], dtype=float)
            self.link_poses[name] = (p, q)
            self.link_twists[name] = (v, w)
        self.last_link_stamp = time.monotonic()

    def _traj_cb(self, msg: JointTrajectory):
        self.trajectory_publishes.append((time.monotonic(), msg))

    def _left_contact_cb(self, msg: ContactsState):
        self.left_contacts = list(msg.states)

    def _right_contact_cb(self, msg: ContactsState):
        self.right_contacts = list(msg.states)

    def contact_forces(self) -> Tuple[float, float]:
        """Sum of contact-force magnitudes on each pad, in newtons."""
        def total(states):
            f = 0.0
            for s in states:
                for w in s.wrenches:
                    f += math.sqrt(w.force.x ** 2 + w.force.y ** 2 + w.force.z ** 2)
            return f
        return total(self.left_contacts), total(self.right_contacts)

    def contact_force_on(self, partner_substr: str) -> Tuple[float, float]:
        """Contact force on each pad from collisions whose name contains partner_substr."""
        def total(states, own):
            f = 0.0
            for s in states:
                a, b = s.collision1_name, s.collision2_name
                partner = b if own in a else a
                if partner_substr not in partner:
                    continue
                for w in s.wrenches:
                    f += math.sqrt(w.force.x ** 2 + w.force.y ** 2 + w.force.z ** 2)
            return f
        return (
            total(self.left_contacts, "left_1_link"),
            total(self.right_contacts, "right_1_link"),
        )

    def contact_partners(self) -> Tuple[list, list]:
        def names(states, own):
            out = []
            for s in states:
                a, b = s.collision1_name, s.collision2_name
                out.append(b if own in a else a)
            return out
        return names(self.left_contacts, "left_1_link"), names(self.right_contacts, "right_1_link")

    # ── spinning ───────────────────────────────────────────────────────────
    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.02, max(0.0, end - time.monotonic())))

    def wait_until(self, predicate: Callable[[], bool], timeout: float, poll: float = 0.02) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=poll)
            if predicate():
                return True
        return False

    def wait_for_joint_states(self, joints: Sequence[str], timeout: float = 20.0) -> bool:
        return self.wait_until(lambda: all(j in self.joint_positions for j in joints), timeout)

    def wait_for_models(self, names: Sequence[str], timeout: float = 20.0) -> bool:
        return self.wait_until(lambda: all(n in self.model_poses for n in names), timeout)

    # ── state access ───────────────────────────────────────────────────────
    def q(self, joints: Sequence[str]) -> np.ndarray:
        missing = [j for j in joints if j not in self.joint_positions]
        if missing:
            raise RuntimeError(f"joint state missing: {missing}")
        return np.array([self.joint_positions[j] for j in joints], dtype=float)

    def qdot(self, joints: Sequence[str]) -> np.ndarray:
        missing = [j for j in joints if j not in self.joint_velocities]
        if missing:
            raise RuntimeError(f"joint velocity missing: {missing}")
        return np.array([self.joint_velocities[j] for j in joints], dtype=float)

    def model_pose(self, name: str) -> Tuple[np.ndarray, np.ndarray]:
        if name not in self.model_poses:
            raise RuntimeError(f"model '{name}' not present in /gazebo/model_states")
        return self.model_poses[name]

    def link_pose(self, name: str) -> Tuple[np.ndarray, np.ndarray]:
        if name not in self.link_poses:
            raise RuntimeError(f"link '{name}' not present in /gazebo/link_states")
        return self.link_poses[name]

    def link_twist(self, name: str) -> Tuple[np.ndarray, np.ndarray]:
        if name not in self.link_twists:
            raise RuntimeError(f"link '{name}' not present in /gazebo/link_states")
        return self.link_twists[name]

    def wait_for_links(self, names: Sequence[str], timeout: float = 20.0) -> bool:
        return self.wait_until(lambda: all(n in self.link_poses for n in names), timeout)

    def tf_pose(self, parent: str, child: str, timeout: float = 1.0) -> Tuple[np.ndarray, np.ndarray, float]:
        """Look up a transform. Raises TFUnavailable instead of returning zeros."""
        try:
            t = self.tf_buffer.lookup_transform(
                parent, child, Time(), timeout=Duration(seconds=timeout)
            )
        except Exception as exc:  # noqa: BLE001 - re-raise with context
            raise TFUnavailable(f"TF {parent}->{child} unavailable: {exc}") from exc
        p = np.array(
            [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z],
            dtype=float,
        )
        q = np.array(
            [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ],
            dtype=float,
        )
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        return p, q, stamp

    # ── actuation ──────────────────────────────────────────────────────────
    def send_trajectory(
        self,
        positions: Sequence[float],
        duration: float,
        joint_names: Optional[Sequence[str]] = None,
    ) -> float:
        names = list(joint_names or [
            "waist_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_pitch_joint",
            "gripper_joint",
        ])
        traj = JointTrajectory()
        traj.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in positions]
        # The controller uses spline interpolation.  A single-point trajectory
        # without an explicit velocity is not reliably executed; supply zero
        # end velocities so the goal is always honoured.
        pt.velocities = [0.0] * len(names)
        pt.accelerations = [0.0] * len(names)
        pt.time_from_start = Duration(seconds=float(duration)).to_msg()
        traj.points.append(pt)
        t0 = time.monotonic()
        self.pub_traj.publish(traj)
        return t0

    def send_trajectory_until(
        self,
        positions: Sequence[float],
        duration: float,
        joint_names: Optional[Sequence[str]] = None,
        tol: float = 0.02,
        timeout: float = 10.0,
        attempts: int = 3,
    ) -> Tuple[float, int]:
        """Send a goal and verify that the joints actually arrive.

        The controller can silently drop the first trajectory after start-up
        (observed in this workcell), so resend until the measured joint
        positions are within ``tol`` or the attempts are exhausted.
        """
        names = list(joint_names or [
            "waist_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_pitch_joint",
            "gripper_joint",
        ])
        t0 = 0.0
        used = 0
        for _ in range(max(1, attempts)):
            used += 1
            t0 = self.send_trajectory(positions, duration, names)
            # A single publish can be lost before the controller's subscription
            # is fully matched; a second identical goal is harmless.
            self.spin_for(0.05)
            self.send_trajectory(positions, duration, names)
            ok = self.wait_until(
                lambda: all(
                    abs(self.joint_positions.get(n, 1e9) - float(p)) <= tol
                    for n, p in zip(names, positions)
                ),
                timeout,
            )
            if ok:
                return t0, used
        return t0, used

    def send_conveyor_speed(self, speed_m_s: float) -> None:
        """Single source of truth: Twist only, never the service, to avoid a race."""
        tw = Twist()
        tw.linear.x = float(speed_m_s)
        self.pub_cmd_vel.publish(tw)

    def set_model_pose(
        self, name: str, xyz: Sequence[float], quat: Sequence[float] = (0, 0, 0, 1), timeout: float = 3.0
    ) -> bool:
        if not self.cli_set_state.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/gazebo/set_entity_state not available")
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(xyz[0])
        req.state.pose.position.y = float(xyz[1])
        req.state.pose.position.z = float(xyz[2])
        req.state.pose.orientation.x = float(quat[0])
        req.state.pose.orientation.y = float(quat[1])
        req.state.pose.orientation.z = float(quat[2])
        req.state.pose.orientation.w = float(quat[3])
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0
        fut = self.cli_set_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() and fut.result().success)

    def call_trigger(self, service: str, timeout: float = 5.0) -> bool:
        cli = self.create_client(Trigger, service)
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service {service} not available")
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() and fut.result().success)


def quat_to_rpy(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return np.array([roll, pitch, yaw], dtype=float)
