"""Preserve marker priority while scheduling inspection of interior and wall surfaces."""
import json
from pathlib import Path

import numpy as np
from rclpy.time import Time

from osp_mission.center_wall import CenterWall
from osp_mission.auto_chair_strip import ChairCueFilter
from osp_mission.chair_band import ChairBandTracker
from osp_mission.end_walls import EndWalls
from osp_mission.box_views import local_boxes, odom_point
from osp_mission.hybrid import Hybrid
from osp_mission.r32_drive import grid_from_message
from osp_mission.region_transit import route_connected
from osp_mission import _r32_navigation_core as r32


class PhasedHybrid(Hybrid):
    def __init__(self):
        super().__init__()
        self.schedule = CenterWall()
        self.local_cost = None
        # Base interrupt logic can still request verified print follow-ups.
        self.next_box_probe = float('inf')
        # .51: chair clusters on the glass ends become committed virtual walls.
        self.chair_walls = ChairBandTracker()
        self.chair_wall_errors = 0

    def update_local_cost(self, msg):
        self.local_cost = msg

    def select(self, owner, selected, data, viewed, pose, transform, *,
               print_only=False, box_probe=False, choices=()):
        self.band_rejected_selected = False
        if selected and any(k in selected for k in ('approach_id','focus_id')):
            self.schedule.reason = 'verified_marker_approach' if 'approach_id' in selected else 'accepted_marker_followup'
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
        try:
            self.chair_walls.update(data, mapped.resolution, mapped.origin)
        except Exception:  # never let the detector stop navigation
            self.chair_wall_errors += 1
        walls = self.chair_walls
        self.schedule.wall_lap.sparse = lambda point: walls.contains(point, 1.6)
        self.schedule.wall_lap.project = lambda point: walls.project(point, 1.0)

        def veto(goal):
            # .53: a band detected once is avoided until the next update confirms or drops it.
            if walls.contains((goal['x'], goal['y']), .3) or walls.suspected((goal['x'], goal['y']), .3):
                return True
            return ((owner.navigation_flow and owner.navigation_flow.blocked(goal,transform,now))
                    or (not goal.get('stationary') and any(
                        np.hypot(goal['x']-x,goal['y']-y)<.45 for x,y in owner.excluded[-8:])))

        chair_strip = ChairCueFilter(grid)
        small_returns = self.schedule.small_returns
        small_returns.update(grid)

        def white_filter(point):
            # .65: white-leg cues are off. Physical runs read no marker from 19 white-leg looks
            # (.50) and spent about a minute on them on 2026-11-27; that time goes to the lap.
            return False

        cue = self.memory.choose(grid,cost,pose,transform,owner.targets,clock,veto,white_filter=white_filter,
                                 excluded_zone=small_returns.matches)
        lap_active = self.schedule.wall_lap.phase == 'perimeter'
        protected = self.schedule.continuity.protecting(now-owner.started) or lap_active
        if cue is not None and cue.get('white_leg') and (
                protected or selected is not None and selected.get('r32_kind') == 'interior_surface'):
            cue = None
        if print_only and protected and cue is None:
            return None
        if print_only and (cue is not None or not box_probe or self.schedule.region_transit.active):
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
        centers = [track.center for track in self.box_views.tracks if track.confirmed]
        # An identified print completes object service; 1.5m mission proof is separate.
        identified = [odom_point(point, transform) for point in owner.targets.values()]
        self.schedule.weak_known = [odom_point(point,transform) for point in owner.targets.values()]
        self.schedule.weak_clock = clock
        self.schedule.survey.travel_memory = getattr(owner, 'exploration_memory', None)
        self.schedule.survey.camera = getattr(owner, 'inspection_camera', None)
        sample = getattr(owner, 'wall_camera_sample', None)
        self.schedule.wall_camera = sample[1:] if sample and 0 <= clock-sample[0] <= .25 else None
        current_ids = set(owner.targets)
        if current_ids - self.schedule.survey.known:
            self.schedule.survey.last_progress = now-owner.started
            self.schedule.survey.loop_center = None
        self.schedule.survey.known = current_ids
        self.schedule.batch.update(centers, identified, odom_point(pose[:2], transform))
        self.schedule.cluster.observed = [track.center for track in self.box_views.tracks
            if track.confirmed and (len(track.seen) >= 2 or
                                    tuple(track.center) in self.schedule.batch.completed)]
        ends = EndWalls(grid)
        self.schedule.end_wall_surface = ends.wall_surface
        def reject_exploration(point):
            return (chair_strip.band_contains(point) or ends.contains(point)
                    or walls.contains(point, .3))
        self.schedule.chair_band_contains = reject_exploration
        self.schedule.chair_surface_contains = chair_strip.includes
        self.schedule.local_box_cost = None
        local = self.local_cost
        if local is not None and local.header.frame_id in ('odom', owner.p('map_frame')):
            local_age = clock-Time.from_msg(local.header.stamp).nanoseconds/1e9
            if -.1 <= local_age <= 1.5:
                self.schedule.local_box_cost = (local.header.frame_id, grid_from_message(local))
        if selected is not None and reject_exploration((selected['x'], selected['y'])):
            self.band_rejected_selected = True
            selected = None
        goal = self.schedule.decide(grid,cost,pose,transform,viewed,
                                    now-owner.started,owner.p('duration'),veto,choices,fitted,probe_only=print_only,
                                    pending_approach=(owner.approach, owner.targets, clock))
        if goal is not None and 'routed_approach_m' in goal:
            return goal
        if cue is not None and cue.get('white_leg') and goal is not None and (
                goal.get('wall_lap') or goal.get('r32_kind') == 'interior_surface' or any(
                key in goal for key in ('bearing_advance_id','distant_hint_id','weak_wall'))):
            return goal
        if cue is not None:
            self.schedule.reason = 'white_leg_candidate' if cue.get('white_leg') else 'local_print_candidate'
            return cue
        if print_only:
            return goal
        if goal is None and self.schedule.wall_lap.phase == 'perimeter':
            self.band_rejected_selected = True
            return None
        if goal is None and selected is not None and selected['features'][1] < .15:
            self.band_rejected_selected = True
            selected = None
        routes = self.schedule.exploration_routes
        if goal is not None and not route_connected(grid, routes, goal):
            goal = None
        if selected is not None and not route_connected(grid, routes, selected):
            self.band_rejected_selected = True
            selected = None
        if goal is None and selected is not None and selected['features'][1] >= .15 and not selected.get('stationary'):
            self.schedule.reason = 'safe_frontier_fallback'
            goal = dict(selected, r32_kind='interior_frontier')
        if goal is not None:
            if goal.get('region_action') == 'resume':
                goal['region_destination_odom'] = list(self.schedule.region_transit.anchor)
            elif (not print_only and (goal.get('r32_kind') in ('interior_frontier', 'unvisited_region') or goal.get('region_action') == 'leave')
                    and np.hypot(goal['x']-pose[0], goal['y']-pose[1]) >= 1.5):
                goal = dict(goal, region_destination_odom=odom_point((goal['x'],goal['y']), transform).tolist())
            return goal
        return selected

    def dispatched(self,owner,goal):
        super().dispatched(owner,goal)
        self.active_print_priority = any(k in goal for k in (
            'approach_id', 'focus_id', 'print_point', 'routed_approach_m'))
        elapsed = owner.now()-owner.started
        self.schedule.dispatched(goal,elapsed)
        events = self.schedule.continuity.events
        if events:
            try:
                with (Path(owner.p('output_dir'))/'box_continuity.jsonl').open('a') as stream:
                    for event in events:
                        stream.write(json.dumps(event)+'\n')
                events.clear()
            except OSError as exc:
                owner.get_logger().warning(str(exc))
        self.next_box_probe = owner.now()+3. if (goal.get('r32_kind')=='wall_surface' or goal.get('cluster_exit')) else float('inf')
        if goal.get('survey_escape') or 'distant_hint_id' in goal or 'survey_wall' in goal:
            self.next_box_probe = float('inf')
        row = dict(face_work=self.schedule.work.summary(), marked_box_ids=sorted(self.schedule.marked_boxes.markers),
                   suppressed_box_face_groups=self.schedule.marked_boxes.skipped,
                   bearing_attempt_number=goal.get('bearing_attempt_number'),
                   bearing_attempt_limit=goal.get('bearing_attempt_limit'), routed_approach_m=goal.get('routed_approach_m'), region_arrived_s=self.schedule.joint.entered if self.schedule.joint.anchor is not None else None,
                   region_stops=self.schedule.joint.stops, face_frontier_gap_m=goal.get('face_frontier_gap_m'),
                   original_wall_route_m=goal.get('original_wall_route_m'), survey_wall=goal.get('survey_wall'),survey_visible_sections=goal.get('survey_visible_sections'),
                   survey_escape=bool(goal.get('survey_escape')), distant_hint_id=goal.get('distant_hint_id'),
                   elapsed=elapsed,phase=self.schedule.phase,reason=self.schedule.reason,
                   available=self.schedule.counts,kind=goal.get('r32_kind','flow_or_marker'),
                   white_leg=bool(goal.get('white_leg')), white_point=goal.get('print_point') if goal.get('white_leg') else None,
                   region_action=goal.get('region_action'), region_destination_odom=goal.get('region_destination_odom'),
                   cluster_exit=bool(goal.get('cluster_exit')),
                   crowded_centers=[list(c) for c in self.schedule.cluster.crowded_centers(elapsed)],
                   weak_wall=bool(goal.get('weak_wall')),local_batch=goal.get('local_batch'), completed_box=goal.get('completed_box'),
                   batch_box=goal.get('batch_box'), marker_id=goal.get('approach_id',goal.get('focus_id')),
                   bearing_advance_id=goal.get('bearing_advance_id'), bearing_pixels=goal.get('bearing_pixels'),
                   bearing_progress_m=goal.get('bearing_progress_m'),
                   surface=goal.get('surface_map'), fitted_face=goal.get('fitted_face'),
                   face_aim_map=goal.get('face_aim_map'),
                   wall_lap=self.schedule.wall_lap.report(),
                   chair_walls=[[float(v) for v in (*p, *n)] for p, n in self.chair_walls.walls.values()],
                   lap_chair_face=bool(goal.get('lap_chair_face')),
                   interior_priority=bool(goal.get('interior_priority')),
                   known_wall_distance_m=goal.get('known_wall_distance_m'),
                   interior_priority_score=goal.get('interior_priority_score'),
                   box_face_resume=bool(goal.get('box_face_resume')),
                   box_face_continuation=bool(goal.get('box_face_continuation')),
                   local_box_cost_used=self.schedule.local_box_cost is not None,
                   box_recovery_walk_m=goal.get('box_recovery_walk_m'),
                   box_recovery_same_face=goal.get('box_recovery_same_face'),
                   joint_views=goal.get('joint_views'), joint_walk_m=goal.get('joint_walk_m'),
                   joint_route_m=goal.get('joint_route_m'), joint_score=goal.get('joint_score'),
                   region_anchor=goal.get('region_anchor'), region_face=goal.get('region_face'),
                   new_region_distance_m=goal.get('new_region_distance_m'), new_region_path_m=goal.get('new_region_path_m'),
                   joint_continuation=goal.get('joint_continuation'),
                   visit_novelty_m=goal.get('visit_novelty_m'),
                   visit_novelty_gain=goal.get('visit_novelty_gain'),
                   visit_return_cost=goal.get('visit_return_cost'),
                   standoff_m=goal.get('standoff_m'), clearance_m=goal.get('clearance_m'))
        try:
            with (Path(owner.p('output_dir'))/'center_wall.jsonl').open('a') as out:
                out.write(json.dumps(row,allow_nan=False)+'\n')
        except OSError as exc:
            owner.get_logger().warning(str(exc))
