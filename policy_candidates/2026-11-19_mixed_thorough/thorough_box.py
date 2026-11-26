"""Inspect a verified object's reachable faces, including incomplete rings."""
import math

from osp_mission.full_box import FullBox
from osp_mission.two_views import TwoViews


class ThoroughBox(FullBox):
    def __init__(self):
        super().__init__()
        self.corner_half_fov = math.radians(20)

    def choose(self, grid, costmap, pose, transform, now, targets, veto):
        if now - self.last_choice < 3.0:
            return None
        full = super().choose(grid, costmap, pose, transform, now, targets, veto)
        if full is not None:
            full['box_view_scope'] = 'full_ring'
            return full
        # A mapped wall, chair, or unfinished map may block the ring while a
        # confirmed box still has reachable, camera-visible faces.
        partial = TwoViews.choose(self, grid, costmap, pose, transform, now, targets, veto)
        if partial is not None:
            partial['box_view_scope'] = 'reachable_face'
        return partial
