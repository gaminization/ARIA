#!/usr/bin/env python3
"""
Compare Gazebo ground truth with ARIA's world model (/aria/state/memory).
Reports: matched, missing (GT with no model object within tol), ghosts (model objects with no GT),
duplicates (>1 model object per GT object). Exit code 0 if no ghosts/duplicates.

Usage: world_model_audit.py [--tol 0.06] [--wait 3] [--ignore substr,substr]
"""
import argparse
import math
import sys
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node

from arm_planner.msg import MemoryState

STATIC = ('ground', 'table', 'plate_', 'mounting', 'calibration', 'camera', 'inspection', 'pedestal',
          'aria_arm', 'fence', 'floor', 'cabinet', 'conveyor', 'optical', 'pad')


class A(Node):
    def __init__(self):
        super().__init__('wm_audit')
        self.gt = {}
        self.mem = None
        self.create_subscription(ModelStates, '/gazebo/model_states', self.ms, 1)
        self.create_subscription(MemoryState, '/aria/state/memory', self.mm, 1)

    def ms(self, m):
        self.gt = {n: (p.position.x, p.position.y, p.position.z) for n, p in zip(m.name, m.pose)}

    def mm(self, m):
        self.mem = m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tol', type=float, default=0.06)
    ap.add_argument('--wait', type=float, default=3)
    ap.add_argument('--ignore', default='')
    a = ap.parse_args()
    rclpy.init()
    n = A()
    end = time.time() + a.wait
    while time.time() < end or n.mem is None or not n.gt:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() > end + 10:
            break
    ign = STATIC + tuple(x for x in a.ignore.split(',') if x)
    gt = {k: v for k, v in n.gt.items() if not any(s in k for s in ign)}
    objs = [(o.name, o.class_name, o.lifecycle_state,
             (o.last_known_pose.pose.position.x, o.last_known_pose.pose.position.y,
              o.last_known_pose.pose.position.z)) for o in (n.mem.known_objects if n.mem else [])]
    d = lambda p, q: math.hypot(p[0] - q[0], p[1] - q[1])
    per_gt = {k: [] for k in gt}
    ghosts = []
    for name, cls, st, p in objs:
        best = min(gt.items(), key=lambda kv: d(kv[1], p), default=None)
        if best and d(best[1], p) <= a.tol:
            per_gt[best[0]].append((name, cls, d(best[1], p)))
        else:
            ghosts.append((name, cls, st, p))
    print(f'GT objects: {len(gt)}   model objects: {len(objs)}')
    missing, dups = [], []
    for k, v in sorted(per_gt.items()):
        tag = 'OK     ' if len(v) == 1 else ('MISSING' if not v else 'DUP    ')
        if not v:
            missing.append(k)
        if len(v) > 1:
            dups.append(k)
        print(f'  {tag} {k:22s} -> ' + ', '.join(f'{x[0]}({x[1]}) err={x[2]*100:.1f}cm' for x in v))
    for g in ghosts:
        print(f'  GHOST   {g[0]} class={g[1]} state={g[2]} at ({g[3][0]:.3f},{g[3][1]:.3f})')
    print(f'SUMMARY matched={len(gt)-len(missing)-len(dups)} missing={len(missing)} '
          f'duplicates={len(dups)} ghosts={len(ghosts)}')
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if not ghosts and not dups else 1)


if __name__ == '__main__':
    main()
