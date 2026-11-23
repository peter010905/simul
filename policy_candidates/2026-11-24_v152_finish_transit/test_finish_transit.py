from types import SimpleNamespace
import unittest

from osp_mission.mission import Mission
from osp_mission.navigation_flow import NavigationFlow


class TransitTests(unittest.TestCase):
    def test_periodic_camera_sweep_cannot_cancel_disabled_transit_but_cue_can(self):
        events = []
        owner = SimpleNamespace(
            state='running', targets={}, collision_count=0, started=1.,
            published={}, status=SimpleNamespace(publish=lambda msg: None),
            now=lambda: 100., p=lambda key: 600., ready=lambda: True,
            goal_handle=object(), pending=False, cancel_started=None, cancel_reason='',
            kind='nav', action_start=99., spin_blocked_until=0.,
            contact_pending=False, recovery_pending=False, hybrid=None,
            navigation_flow=NavigationFlow(False), cancel_action=events.append,
            contact_inspection=SimpleNamespace(phase=None, bearing=None, due=lambda _: True),
        )
        Mission.tick(owner)
        self.assertEqual(events, [])
        owner.contact_inspection.bearing = (99.9, .4)
        Mission.tick(owner)
        self.assertEqual(events, ['inspection_due'])


if __name__ == '__main__':
    unittest.main()
