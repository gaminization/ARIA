#!/usr/bin/env python3
"""Drive the arm to a list of joint poses (deg) and save a side-camera montage.
Usage: pose_probe.py out.png "0,35,-55,20,20" "0,0,0,0,0" ...
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

from arm_interfaces.srv import SetAllJoints


class P(Node):
    def __init__(self):
        super().__init__('pose_probe')
        self.b = CvBridge()
        self.side = None
        self.j = {}
        self.create_subscription(Image, '/side_camera/image_raw', lambda m: setattr(self, 'side', self.b.imgmsg_to_cv2(m, 'bgr8')), qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.js, qos_profile_sensor_data)
        self.cli = self.create_client(SetAllJoints, '/aria/set_all_joints')

    def js(self, m):
        self.j = dict(zip(m.name, m.position))

    def spin(self, s):
        t = time.time()
        while time.time() - t < s:
            rclpy.spin_once(self, timeout_sec=0.05)


rclpy.init()
n = P()
n.cli.wait_for_service(10)
out = sys.argv[1]
tiles = []
for spec in sys.argv[2:]:
    q = [float(v) for v in spec.split(',')]
    r = SetAllJoints.Request()
    r.angles_deg = q
    r.speed_deg_per_s = 60.0
    f = n.cli.call_async(r)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    n.spin(5.0)
    got = [round(np.degrees(n.j.get(k, 0)), 1) for k in
           ('waist_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'gripper_joint')]
    print(spec, '->', got)
    img = cv2.resize(n.side, (640, 360)).copy()
    cv2.putText(img, spec, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    tiles.append(img)
rows = [np.hstack(tiles[i:i + 3] + [np.zeros_like(tiles[0])] * (3 - len(tiles[i:i + 3]))) for i in range(0, len(tiles), 3)]
cv2.imwrite(out, np.vstack(rows))
