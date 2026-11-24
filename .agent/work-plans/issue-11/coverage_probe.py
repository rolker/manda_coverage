#!/usr/bin/env python3
"""Closed-loop probe for manda_coverage's compute_sonar_coverage_path action.

Sends one goal (a 200 x 100 m rectangle rotated 30 degrees, in either vertex
order) to a running, active manda_coverage_action_server, then plays a fake
boat: for every line the server sends back as feedback it drives the line from
its first to its last pose at 1 m per step, publishing `odom` (pose and yaw
along the segment) and then a flat `soundings` cloud 40 m wide (y from -20 to
+20, z 10 m) at each step, until the server returns its result.

Writes <out_dir>/lines_<order>.txt (one line per returned survey line, x,y per
pose to 1e-6 m, for diffing between builds), <out_dir>/summary_<order>.txt (the
checks below) and prints both.

The checks are a sanity check for a flat swath, not a check of the planner's
adaptive behaviour (with a constant swath the next line is a plain offset):
  - the result is success within 10 lines;
  - each line's vertices lie inside the polygon (the 15 m lead-in and 5 m
    lead-out excepted, 0.5 m tolerance);
  - each line is parallel to the polygon's first edge (within 5 degrees);
  - consecutive lines are spaced between (1 - swath_overlap) x 40 = 32 m and
    40 m apart;
  - the union of the 40 m strips of the driven lines covers most (>= 85%) of
    the polygon;
  - no pose orientation is NaN (reported, not judged: see the follow-up on
    marine_nav_utilities::adjustPathOrientations).

usage: coverage_probe.py <fwd|rev> <out_dir> [timeout_s]
  fwd: vertices counter-clockwise (the server's first line is on Port);
  rev: the same list reversed, clockwise (first line on Stbd).
"""

import math
import os
import struct
import sys
import time

from geometry_msgs.msg import Point32
from marine_nav_interfaces.action import ComputeSonarCoveragePath
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

SWATH_HALF = 20.0
OVERLAP = 0.2
LEAD_IN = 15.0
LEAD_OUT = 5.0
STEP = 1.0
PERIOD = 0.02  # s between steps
MAX_LINES = 10


def rectangle(order):
    c, s = math.cos(math.pi / 6), math.sin(math.pi / 6)
    base = [(0, 0), (200, 0), (200, 100), (0, 100)]
    pts = [(x * c - y * s, x * s + y * c) for x, y in base]
    return pts if order == 'fwd' else list(reversed(pts))


def make_cloud(stamp):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = 'sonar'
    ys = [-SWATH_HALF + i * 0.5 for i in range(int(2 * SWATH_HALF / 0.5) + 1)]
    msg.height = 1
    msg.width = len(ys)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(ys)
    msg.is_dense = True
    msg.data = b''.join(struct.pack('<fff', 0.0, y, 10.0) for y in ys)
    return msg


def inside(p, poly, tol=0.0):
    # Point in polygon, or within tol of its boundary.
    x, y = p
    n = len(poly)
    hit = False
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            hit = not hit
    if hit:
        return True
    return min(seg_dist(p, poly[i], poly[(i + 1) % n]) for i in range(n)) <= tol


def seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def move(a, b, d):
    n = math.hypot(b[0] - a[0], b[1] - a[1])
    return (a[0] + (b[0] - a[0]) / n * d, a[1] + (b[1] - a[1]) / n * d)


class Probe(Node):

    def __init__(self, order):
        super().__init__('coverage_probe')
        self.order = order
        self.polygon = rectangle(order)
        self.client = ActionClient(self, ComputeSonarCoveragePath, 'compute_sonar_coverage_path')
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        self.ping_pub = self.create_publisher(PointCloud2, 'soundings', qos_profile_sensor_data)
        self.lines = []  # (line_number, [(x, y, qx, qy, qz, qw)])
        self.pending = []
        self.result = None
        self.drive = None  # (points, index)
        self.timer = self.create_timer(PERIOD, self.tick)

    def send(self):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('action server not available')
        goal = ComputeSonarCoveragePath.Goal()
        goal.survey_area.header.frame_id = 'map'
        goal.survey_area.polygon.points = [Point32(x=x, y=y, z=0.0) for x, y in self.polygon]
        fut = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('goal rejected')
        self.result_future = handle.get_result_async()

    def feedback(self, msg):
        fb = msg.feedback
        poses = [(p.pose.position.x, p.pose.position.y, p.pose.orientation.x,
                  p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)
                 for p in fb.current_line.poses]
        self.lines.append((fb.line_number, poses))
        self.pending.append(poses)

    def tick(self):
        if self.drive is None:
            if not self.pending:
                return
            poses = self.pending.pop(0)
            pts = [(p[0], p[1]) for p in poses]
            steps = []  # (x, y, yaw)
            for i in range(len(pts) - 1):
                a, b = pts[i], pts[i + 1]
                length = math.hypot(b[0] - a[0], b[1] - a[1])
                if length == 0:
                    continue
                yaw = math.atan2(b[1] - a[1], b[0] - a[0])
                n = max(1, math.ceil(length / STEP))
                for k in range(0 if not steps else 1, n + 1):
                    t = min(1.0, k * STEP / length)
                    steps.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, yaw))
            self.drive = (steps, 0)
        steps, i = self.drive
        if i >= len(steps):
            self.drive = None
            return
        x, y, yaw = steps[i]
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        self.odom_pub.publish(odom)
        time.sleep(PERIOD / 4)
        self.ping_pub.publish(make_cloud(now))
        self.drive = (steps, i + 1)


def judge(probe):
    poly = probe.polygon
    out = []
    ok = True
    res = probe.result
    n = len(probe.lines)
    success = res is not None and res.success
    out.append(f'result: {"none (timeout)" if res is None else f"success={res.success}"}; '
               f'lines: {n}')
    if not (success and n <= MAX_LINES):
        ok = False
        out.append(f'FAIL: success within {MAX_LINES} lines')
    edge = math.atan2(poly[1][1] - poly[0][1], poly[1][0] - poly[0][0])
    normal = (-math.sin(edge), math.cos(edge))
    offsets = []
    nan_lines = []
    for num, poses in probe.lines:
        pts = [(p[0], p[1]) for p in poses]
        if any(any(math.isnan(v) for v in p[2:]) for p in poses):
            nan_lines.append(num)
        core = list(pts)
        if len(core) >= 2:
            core[0] = move(core[0], core[1], LEAD_IN)
            core[-1] = move(core[-1], core[-2], LEAD_OUT)
        outside = [p for p in core if not inside(p, poly, 0.5)]
        if outside:
            ok = False
            out.append(f'FAIL: line {num}: {len(outside)} vertices outside the polygon')
        ang = math.degrees(math.atan2(pts[-1][1] - pts[0][1], pts[-1][0] - pts[0][0]) - edge)
        ang = (ang + 90) % 180 - 90
        if abs(ang) > 5:
            ok = False
            out.append(f'FAIL: line {num} not parallel to the first edge ({ang:.2f} deg)')
        offsets.append(sum(p[0] * normal[0] + p[1] * normal[1] for p in core) / len(core))
    spacings = [abs(offsets[i + 1] - offsets[i]) for i in range(len(offsets) - 1)]
    lo, hi = (1 - OVERLAP) * 2 * SWATH_HALF, 2 * SWATH_HALF
    for i, s in enumerate(spacings):
        if not lo - 0.5 <= s <= hi + 0.5:
            ok = False
            out.append(f'FAIL: spacing lines {i + 1}-{i + 2} {s:.2f} m not in [{lo}, {hi}]')
    out.append('line offsets across the first edge (m): ' +
               ', '.join(f'{o:.2f}' for o in offsets))
    out.append('spacings (m): ' + ', '.join(f'{s:.2f}' for s in spacings))
    # Coverage: 1 m grid over the polygon, a cell covered if within SWATH_HALF of
    # a driven line.
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    segs = []
    for _, poses in probe.lines:
        pts = [(p[0], p[1]) for p in poses]
        segs += [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    total = covered = 0
    y = min(ys) + 0.5
    while y < max(ys):
        x = min(xs) + 0.5
        while x < max(xs):
            if inside((x, y), poly):
                total += 1
                if any(seg_dist((x, y), a, b) <= SWATH_HALF for a, b in segs):
                    covered += 1
            x += 1.0
        y += 1.0
    frac = covered / total if total else 0.0
    out.append(f'coverage of the polygon by the 40 m strips: {100 * frac:.1f}% '
               f'({covered} of {total} 1 m cells)')
    if frac < 0.85:
        ok = False
        out.append('FAIL: coverage below 85%')
    out.append('NaN orientations in lines: ' + (', '.join(map(str, nan_lines)) or 'none'))
    out.append(f'verdict: {"PASS" if ok else "FAIL"} (sanity check for a flat swath)')
    return '\n'.join(out)


def main():
    order = sys.argv[1]
    out_dir = sys.argv[2]
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 300.0
    if order not in ('fwd', 'rev'):
        sys.exit('order must be fwd or rev')
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    probe = Probe(order)
    probe.send()
    deadline = time.monotonic() + timeout
    gone_since = None
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
        if probe.result_future.done():
            probe.result = probe.result_future.result().result
            break
        # Stop early if the action server has gone (e.g. the node died).
        if probe.client.server_is_ready():
            gone_since = None
        elif gone_since is None:
            gone_since = time.monotonic()
        elif time.monotonic() - gone_since > 10.0:
            print('action server gone for 10 s: giving up')
            break
    with open(os.path.join(out_dir, f'lines_{order}.txt'), 'w') as f:
        f.write(f'# polygon ({order}): ' +
                ' '.join(f'{x:.6f},{y:.6f}' for x, y in probe.polygon) + '\n')
        for num, poses in probe.lines:
            f.write(f'line {num} ({len(poses)} poses):' +
                    ''.join(f' {p[0]:.6f},{p[1]:.6f}' for p in poses) + '\n')
    summary = judge(probe)
    with open(os.path.join(out_dir, f'summary_{order}.txt'), 'w') as f:
        f.write(summary + '\n')
    print(open(os.path.join(out_dir, f'lines_{order}.txt')).read())
    print(summary)
    probe.destroy_node()
    rclpy.shutdown()
    return 0 if summary.endswith('(sanity check for a flat swath)') and 'PASS' in \
        summary.splitlines()[-1] else 1


if __name__ == '__main__':
    sys.exit(main())
