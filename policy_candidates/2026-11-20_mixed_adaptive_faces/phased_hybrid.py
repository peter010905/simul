"""Preserve marker priority while scheduling inspection of interior and wall surfaces."""
import json
from pathlib import Path

import numpy as np
from rclpy.time import Time

from osp_mission.center_wall import CenterWall
from osp_mission.box_views import local_boxes, odom_point
from osp_mission.hybrid import Hybrid
from osp_mission.r32_drive import grid_from_message
from osp_mission import _r32_navigation_core as r32


class PhasedHybrid(Hybrid):
    def __init__(self):
        super().__init__()
        self.schedule = CenterWall()
        # Base interrupt logic can still request verified print follow-ups.
        self.next_box_probe = float('inf')

    def select(self, owner, selected, data, viewed, pose, transform, *,
               print_only=False, box_probe=False, choices=()):
        if selected and any(k in selected for k in ('approach_id','focus_id')):
            return selected
        msg = self.drive.costmap
        clock = owner.get_clock().now().nanoseconds/1e9
        if msg is None or msg.header.frame_id != owner.p('map_frame') or transform is None:
            return selected
        age = clock-Time.from_msg(msg.header.stamp).nanoseconds/1e9
        if not -.1 <= age <= 3.:
            return selected
        cost = grid_from_message(msg)
        mapped = grid_from_message(owner.map)
        grid = r32.Grid(data,mapped.resolution,mapped.origin)
        now = owner.now()

        def veto(goal):
            return ((owner.navigation_flow and owner.navigation_flow.blocked(goal,transform,now))
                    or (not goal.get('stationary') and any(
                        np.hypot(goal['x']-x,goal['y']-y)<.45 for x,y in owner.excluded[-8:])))

        cue = self.memory.choose(grid,cost,pose,transform,owner.targets,clock,veto)
        if cue is not None or (print_only and not box_probe):
            return cue
        if print_only:
            # Reuse the navigation owner's interrupt, cancel and replan sequence.
            # Check coverage only for a wall transit, with the same map geometry.
            viewed = owner.coverage.grid(data, mapped.resolution, mapped.origin[:2])
        fitted = []
        if any(t.confirmed for t in self.box_views.tracks):
            for center, faces in local_boxes(grid, pose):
                track = self.box_views.nearest(odom_point(center, transform))
                if track is not None and track.confirmed:
                    fitted.append((center, faces, track.seen))
        goal = self.schedule.decide(grid,cost,pose,transform,viewed,
                                    now-owner.started,owner.p('duration'),veto,choices,fitted,probe_only=print_only)
        if goal is not None:
            return goal
        return selected

    def dispatched(self,owner,goal):
        super().dispatched(owner,goal)
        elapsed = owner.now()-owner.started
        self.schedule.dispatched(goal,elapsed)
        self.next_box_probe = owner.now()+3. if goal.get('r32_kind')=='wall_surface' else float('inf')
        row = dict(elapsed=elapsed,phase=self.schedule.phase,reason=self.schedule.reason,
                   available=self.schedule.counts,kind=goal.get('r32_kind','flow_or_marker'),
                   surface=goal.get('surface_map'), fitted_face=goal.get('fitted_face'),
                   standoff_m=goal.get('standoff_m'), clearance_m=goal.get('clearance_m'))
        try:
            with (Path(owner.p('output_dir'))/'center_wall.jsonl').open('a') as out:
                out.write(json.dumps(row,allow_nan=False)+'\n')
        except OSError as exc:
            owner.get_logger().warning(str(exc))
