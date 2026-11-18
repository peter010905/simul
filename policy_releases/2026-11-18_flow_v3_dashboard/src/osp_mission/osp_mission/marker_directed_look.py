"""Keep a selected marker in view; no recognition or approach credit is created."""
import json
import math
from pathlib import Path

from rclpy.time import Time


class MarkerDirectedLook:
    def __init__(self, owner):
        self.owner = owner
        self.pending = None
        self.selected_count = 0
        self.executed_count = 0
        self.unavailable_count = 0

    def event(self, event, **values):
        owner = self.owner
        row = dict(event=event, stamp=owner.now(), **values)
        try:
            path = Path(owner.p('output_dir')) / 'marker_directed_looks.jsonl'
            with path.open('a') as stream:
                stream.write(json.dumps(row, allow_nan=False) + '\n')
        except (OSError, ValueError) as exc:
            owner.get_logger().warning(f'Could not save marker look audit: {exc}')

    def clear(self, reason):
        if self.pending is not None:
            self.event('cleared', reason=reason, id=self.pending['id'])
        self.pending = None

    def select(self, selected, map_to_odom):
        self.clear('new_selection')
        if selected is None:
            return
        owner = self.owner
        # Approach.choose copies candidates: an approach may retain focus_id
        # for a different marker. The final approach selection takes precedence.
        if 'approach_id' in selected:
            ident = selected['approach_id']
            point = owner.targets.get(ident)
            kind = 'approach'
        elif 'focus_id' in selected:
            ident = selected['focus_id']
            point = owner.marker_focus.hints.get(ident, {}).get('point')
            kind = 'focus'
        else:
            return
        task = dict(id=int(ident), kind=kind, point_odom=None,
                    selected_scan_angle_rad=float(selected.get('scan_angle', 0.)))
        # Do not keep a map-frame point across a possible SLAM correction.
        # Missing TF still reserves the marker look, so generic sweep cannot
        # silently replace it. consume() safely returns a stationary dwell.
        try:
            x, y = map(float, point[:2])
            tx, ty, angle = map(float, map_to_odom)
            if not all(math.isfinite(v) for v in (x, y, tx, ty, angle)):
                raise ValueError('Nonfinite marker transform')
            c, s = math.cos(angle), math.sin(angle)
            task['point_odom'] = [tx + c*x - s*y, ty + s*x + c*y]
        except (TypeError, ValueError, IndexError):
            pass
        self.pending = task
        self.selected_count += 1
        self.event('selected', **task)

    def consume(self):
        """None permits exploration; any marker task returns its own signed turn."""
        task, self.pending = self.pending, None
        if task is None:
            return None
        owner = self.owner
        try:
            if task['point_odom'] is None:
                raise ValueError('Marker point has no valid odom transform')
            tr = owner.tf.lookup_transform('odom', owner.inspection_camera_frame, Time()).transform
            q, t = tr.rotation, tr.translation
            heading = math.atan2(2*(q.y*q.z-q.w*q.x), 2*(q.x*q.z+q.w*q.y))
            dx, dy = task['point_odom'][0]-t.x, task['point_odom'][1]-t.y
            if not all(math.isfinite(v) for v in (dx, dy, heading)) or math.hypot(dx, dy) < .05:
                raise ValueError('Invalid camera-to-marker geometry')
            delta = math.atan2(dy, dx)-heading
            turn = math.atan2(math.sin(delta), math.cos(delta))
        except Exception as exc:
            self.unavailable_count += 1
            self.event('unavailable', reason=str(exc), **task)
            owner.get_logger().debug(f'Marker-directed look unavailable: {exc}')
            return 0.
        self.executed_count += 1
        self.event('consumed', turn_rad=turn, camera_odom=[t.x, t.y, heading], **task)
        owner.get_logger().info(f"Marker-directed {task['kind']} look ID {task['id']}: {math.degrees(turn):.2f} deg")
        return turn

    def report(self):
        return dict(version=1, target_frame='odom', camera_axis='optical_z',
                    maximum_turn_rad=math.pi, selected=self.selected_count,
                    executed=self.executed_count, unavailable=self.unavailable_count,
                    creates_recognition_or_approach_credit=False, ground_truth_inputs=False)
