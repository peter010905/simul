"""Simulation-only walker that follows the robot from a distance (a person trailing it).

Started by trial.sh when the world contains a model named "follower". It reads the
robot's ground-truth pose from Gazebo and moves the kinematic follower along the
robot's own trail, 1.5-3 m behind it, at walking speed. It never talks to the policy.
"""
import math
import os
import re
import subprocess
import sys
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

WORLD = "osp_studio"
SPEED = 0.7          # m/s, sim time
MIN_GAP = 1.0        # never closer to the robot than this
HIDDEN = (0.0, 0.0, -5.0)


def gz_partition(timeout=300):
    end = time.time() + timeout
    while time.time() < end:
        out = subprocess.run(["pgrep", "-f", "gz sim"], capture_output=True, text=True).stdout.split()
        for pid in out:
            try:
                env = open(f"/proc/{pid}/environ", "rb").read().split(b"\0")
            except OSError:
                continue
            for item in env:
                if item.startswith(b"GZ_PARTITION="):
                    return item.split(b"=", 1)[1].decode()
        time.sleep(1)
    return None


class Follower:
    def __init__(self, gap_seed):
        self.trail = []
        self.position = None
        self.last_t = None
        self.last_sent = -1.0
        self.phase = (gap_seed % 97) / 97 * 2 * math.pi
        self.node = Node()
        self.robot = None
        self.pending = None

    def gap(self, t):
        return 2.25 + 0.75 * math.sin(t / 40.0 + self.phase)

    def target(self, t):
        need, walked = self.gap(t), 0.0
        for a, b in zip(reversed(self.trail), list(reversed(self.trail))[1:]):
            step = math.dist(a, b)
            if walked + step >= need:
                f = (need - walked) / step
                return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
            walked += step
        return None

    def on_poses(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        robot = next((p for p in msg.pose if p.name == "turtlebot4"), None)
        if robot is None:
            return
        xy = (robot.position.x, robot.position.y)
        if not self.trail or math.dist(self.trail[-1], xy) >= 0.1:
            self.trail.append(xy)
            self.trail = self.trail[-400:]
        if t - self.last_sent < 0.1:
            return
        dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t
        goal = self.target(t)
        if goal is None:
            if self.position is None:
                self.send(HIDDEN, 0.0, t)
            return
        if self.position is None:
            self.position = goal        # enters behind the robot on its trail
        dx, dy = goal[0] - self.position[0], goal[1] - self.position[1]
        dist = math.hypot(dx, dy)
        step = min(dist, SPEED * dt)
        nxt = self.position if dist < 1e-6 else (self.position[0] + dx / dist * step,
                                                 self.position[1] + dy / dist * step)
        if math.dist(nxt, xy) >= MIN_GAP:
            self.position = nxt
        yaw = math.atan2(xy[1] - self.position[1], xy[0] - self.position[0])
        self.send((*self.position, 0.85), yaw, t)

    def send(self, xyz, yaw, t):
        # Requests go out from the main thread; the subscriber callback only records.
        self.pending = (xyz, yaw)
        self.last_sent = t

    def flush(self):
        if self.pending is None:
            return
        xyz, yaw = self.pending
        self.pending = None
        req = Pose()
        req.name = "follower"
        req.position.x, req.position.y, req.position.z = xyz
        req.orientation.z, req.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.node.request(f"/world/{WORLD}/set_pose", req, Pose, Boolean, 200)


def main():
    part = gz_partition()
    if part is None:
        print("follower: no gz sim process", file=sys.stderr)
        return 1
    os.environ["GZ_PARTITION"] = part
    seed = int(re.sub(r"\D", "", sys.argv[1]) or 0) if len(sys.argv) > 1 else 0
    f = Follower(seed)
    f.node.subscribe(Pose_V, f"/world/{WORLD}/pose/info", f.on_poses)
    print(f"follower: partition {part}", flush=True)
    while True:
        f.flush()
        time.sleep(0.05)


if __name__ == "__main__":
    sys.exit(main())
