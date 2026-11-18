"""Evaluator-only ground truth checks; never imported by the robot policy."""
import csv
import math
from pathlib import Path


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class NativeAudit:
    def __init__(self, scenario):
        self.targets = {int(t['id']): (float(t['x']), float(t['y'])) for t in scenario['targets']}
        self.world_to_map = None
        self.trajectory = []
        self.nearest = {i: float('inf') for i in self.targets}

    def calibrate(self, world_pose, map_pose):
        wx, wy, wa = world_pose
        mx, my, ma = map_pose
        a = ma - wa
        c, s = math.cos(a), math.sin(a)
        self.world_to_map = (mx - c * wx + s * wy, my - s * wx - c * wy, a)

    def observe(self, seconds, x, y, angle):
        if self.trajectory and seconds - self.trajectory[-1][0] < .2:
            return
        self.trajectory.append((seconds, x, y, angle))
        for ident, (tx, ty) in self.targets.items():
            self.nearest[ident] = min(self.nearest[ident], math.hypot(tx - x, ty - y))

    def evaluate(self, marker_file, tolerance=.5):
        estimates = {}
        if marker_file and Path(marker_file).exists():
            with Path(marker_file).open() as stream:
                estimates = {int(r['id']): (float(r['x']), float(r['y'])) for r in csv.DictReader(stream)}
        errors = {}
        if self.world_to_map is not None:
            ox, oy, a = self.world_to_map
            c, s = math.cos(a), math.sin(a)
            for ident, (wx, wy) in self.targets.items():
                if ident in estimates:
                    mx, my = estimates[ident]
                    errors[ident] = math.hypot(mx - (ox + c * wx - s * wy), my - (oy + s * wx + c * wy))
        valid_ids = set(estimates) == set(self.targets)
        valid_positions = len(errors) == len(self.targets) and all(e <= tolerance for e in errors.values())
        return dict(available=self.world_to_map is not None, correct_ids=valid_ids,
                    correct_positions=valid_positions, position_tolerance_m=tolerance,
                    position_errors_m=errors, expected_ids=sorted(self.targets), found_ids=sorted(estimates),
                    closest_approach_m={i: d if math.isfinite(d) else None for i, d in self.nearest.items()},
                    approached_within_1_5m=sum(d <= 1.5 for d in self.nearest.values()),
                    approach_radius_m=1.5, all_approached=bool(self.targets) and all(d <= 1.5 for d in self.nearest.values()),
                    approached_within_2m=sum(d <= 2. for d in self.nearest.values()),
                    world_to_map=self.world_to_map, trajectory_samples=len(self.trajectory))

    def save(self, path):
        with Path(path).open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['sim_seconds', 'world_x', 'world_y', 'world_yaw'])
            writer.writerows(self.trajectory)
