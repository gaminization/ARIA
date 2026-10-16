#!/usr/bin/env python3
"""
ARIA end-to-end task runner with visual + ground-truth verification.

Usage:  e2e_task_runner.py "<command>" [--name NAME] [--timeout S] [--track model1,model2]

Sends the command to /aria/command, then records:
  * every TaskState change (status, action queue statuses, new chain-of-thought lines)
  * camera frames (top / wrist / side) at start, periodically, and at the end
  * Gazebo ground-truth model poses before/after (for strict pick/place verification)
Writes a timeline log, a JSON report, PNG frames and a composite contact sheet to
runs/e2e/<name>/.
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from std_srvs.srv import Empty
from arm_interfaces.srv import SendCommand
from arm_planner.msg import TaskState

ROOT = os.environ.get('ARIA_ROOT', '/home/gaminizer/Projects/ARIA')
CAMS = {'top': '/top_camera/image_raw', 'wrist': '/wrist_camera/image_raw',
        'side': '/side_camera/image_raw'}
TERMINAL = {'COMPLETE', 'FAILED', 'CANCELLED', 'IDLE'}


class Runner(Node):
    def __init__(self, a):
        super().__init__('e2e_runner')
        self.a = a
        self.out = os.path.join(ROOT, 'runs', 'e2e', a.name)
        os.makedirs(self.out, exist_ok=True)
        self.bridge = CvBridge()
        self.frames = {}
        self.models = {}
        self.timeline = []
        self.seen_cot = 0
        self.last_sig = None
        self.status = None
        self.task_id = None
        self.t0 = time.time()
        self.snaps = []
        for k, t in CAMS.items():
            self.create_subscription(Image, t, lambda m, k=k: self._img(k, m), qos_profile_sensor_data)
        self.create_subscription(ModelStates, '/gazebo/model_states', self._ms, 1)
        self.create_subscription(TaskState, '/aria/state/task', self._task, 10)
        self.cli = self.create_client(SendCommand, '/aria/command')

    def _img(self, k, m):
        try:
            arr = np.frombuffer(m.data, dtype=np.uint8).reshape((m.height, m.width, -1))
            if m.encoding in ('rgb8', 'RGB8'):
                self.frames[k] = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif m.encoding in ('bgr8', 'BGR8'):
                self.frames[k] = arr.copy()
            else:
                self.frames[k] = self.bridge.imgmsg_to_cv2(m, 'bgr8')
        except Exception as e:
            pass

    def _ms(self, m):
        self.models = {n: (p.position.x, p.position.y, p.position.z)
                       for n, p in zip(m.name, m.pose)}

    def log(self, s):
        line = f'[{time.time() - self.t0:6.1f}s] {s}'
        print(line, flush=True)
        self.timeline.append(line)

    def _task(self, m):
        if self.task_id is None or m.task_id != self.task_id:
            return
        self.status = m.task_status
        sig = (m.task_status, tuple((x.action_type, x.target_object, x.destination, x.status)
                                    for x in m.action_queue))
        if sig != self.last_sig:
            self.last_sig = sig
            self.log(f'STATE {m.task_status}  actions=' + ' | '.join(
                f'{x.action_type}({x.target_object}->{x.destination or "-"}):{x.status}'
                for x in m.action_queue))
        for line in list(m.chain_of_thought)[self.seen_cot:]:
            self.log('  CoT> ' + line)
        self.seen_cot = len(m.chain_of_thought)
        self.failures = [f for f in m.failure_log]

    def snap(self, tag):
        imgs = []
        for k in CAMS:
            f = self.frames.get(k)
            if f is None:
                f = np.zeros((360, 480, 3), np.uint8)
            f = cv2.resize(f, (480, 360)).copy()
            cv2.putText(f, f'{k} {tag}', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
            imgs.append(f)
            cv2.imwrite(os.path.join(self.out, f'{tag}_{k}.png'), self.frames.get(k, f))
        self.snaps.append(np.hstack(imgs))

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def reset_world(self):
        c = self.create_client(Empty, '/reset_world')
        if c.wait_for_service(timeout_sec=5.0):
            f = c.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, f, timeout_sec=10)
            self.log('world reset')
            self.spin_for(4.0)

    def run(self):
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.log('waiting for /aria/command')
        if self.a.reset:
            self.reset_world()
        self.spin_for(2.0)
        track = [t for t in self.a.track.split(',') if t]
        before = {t: self.models.get(t) for t in track}
        self.snap('00_start')
        req = SendCommand.Request()
        req.command = self.a.command
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        r = fut.result()
        if r is None or not r.accepted:
            self.log(f'COMMAND REJECTED: {r}')
            return self.finish(before, track, 'REJECTED')
        self.task_id = r.task_id
        self.log(f'ACCEPTED {r.task_id}: {self.a.command!r}')
        last_snap = time.time()
        n = 1
        while time.time() - self.t0 < self.a.timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - last_snap > self.a.snap_every:
                self.snap(f'{n:02d}_t{int(time.time() - self.t0)}')
                n += 1
                last_snap = time.time()
            if self.status in ('COMPLETE', 'FAILED', 'CANCELLED'):
                self.spin_for(2.0)
                break
        else:
            self.log('TIMEOUT')
            self.status = 'TIMEOUT'
        self.snap('99_end')
        return self.finish(before, track, self.status)

    def finish(self, before, track, status):
        after = {t: self.models.get(t) for t in track}
        moved = {}
        for t in track:
            b, a = before.get(t), after.get(t)
            if b and a:
                moved[t] = {'before': b, 'after': a,
                            'dist': float(np.linalg.norm(np.array(a) - np.array(b)))}
        self.log(f'FINAL status={status}')
        for t, v in moved.items():
            self.log(f'GROUND-TRUTH {t}: {tuple(round(x, 3) for x in v["before"])} -> '
                     f'{tuple(round(x, 3) for x in v["after"])}  moved {v["dist"]:.3f} m')
        if self.snaps:
            rows = self.snaps[:8] if len(self.snaps) <= 8 else \
                self.snaps[:1] + self.snaps[1:-1][::max(1, len(self.snaps) // 6)][:6] + self.snaps[-1:]
            cv2.imwrite(os.path.join(self.out, 'contact_sheet.png'), np.vstack(rows))
        rep = {'command': self.a.command, 'status': status, 'moved': moved,
               'timeline': self.timeline}
        json.dump(rep, open(os.path.join(self.out, 'report.json'), 'w'), indent=1)
        open(os.path.join(self.out, 'timeline.log'), 'w').write('\n'.join(self.timeline))
        return status


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('command')
    ap.add_argument('--name', default=None)
    ap.add_argument('--timeout', type=float, default=300)
    ap.add_argument('--track', default='')
    ap.add_argument('--snap-every', type=float, default=15)
    ap.add_argument('--reset', action='store_true', help='reset Gazebo world before the task')
    a = ap.parse_args()
    a.name = a.name or time.strftime('%H%M%S')
    rclpy.init()
    n = Runner(a)
    s = n.run()
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if s == 'COMPLETE' else 1)


if __name__ == '__main__':
    main()
