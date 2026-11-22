"""Exercise the installed mission callback with its real marker tracker."""
import json
from types import SimpleNamespace
import unittest

from osp_mission.marker_focus import MarkerFocus
from osp_mission.mission import Mission
from std_msgs.msg import String


def owner(look):
    stops=[]
    o=SimpleNamespace(marker_focus=MarkerFocus(),marker_look=SimpleNamespace(pending=look),
        p=lambda _: 'map', state='running',kind='nav',goal_handle=object(),pending=False,
        cancel_reason='',targets={},get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_200_000_000)),
        get_logger=lambda:SimpleNamespace(info=lambda _:None,debug=lambda _:None),
        cancel_action=stops.append,zero=lambda:stops.append('zero'))
    return o,stops


def send(o,ident,stamp=10.1,provisional=True,point=(3.,0.)):
    msg=String(data=json.dumps(dict(id=ident,stamp=stamp,provisional=provisional,map_point=point,
        camera_origin_map=[0.,0.],observation_frame='map',min_edge_pixels=13.)))
    Mission.on_marker_hint(o,msg)


class FocusContinuityTests(unittest.TestCase):
    def test_matching_hint_updates_evidence_while_navigation_keeps_its_view(self):
        for ident in (0,43,1023):
            for kind in ('focus','approach'):
                with self.subTest(ident=ident,kind=kind):
                    o,stops=owner(dict(id=ident,kind=kind))
                    send(o,ident,10.0)
                    send(o,ident,10.1,point=(3.1,0.))
                    self.assertEqual(stops,[])
                    self.assertEqual(o.marker_focus.hints[ident]['count'],2)
                    self.assertAlmostEqual(o.marker_focus.hints[ident]['point'][0],3.1)
                    self.assertNotIn(ident,o.marker_focus.interruptions)
                    self.assertEqual(o.targets,{})

    def test_different_id_or_ordinary_exploration_can_still_interrupt(self):
        for look in (None,dict(id=43,kind='focus'),dict(id=None,kind='anonymous_print'),dict(id=7,kind='passed_box')):
            with self.subTest(look=look):
                o,stops=owner(look)
                send(o,7)
                self.assertEqual(stops,['marker_hint','zero'])

    def test_accepted_nonprovisional_frames_on_pending_same_id_preserve_the_goal(self):
        o,stops=owner(dict(id=7,kind='focus'))
        o.goal_handle=None;o.pending=True
        send(o,7,10.0,False)
        send(o,7,10.1,False)
        self.assertEqual(stops,[])
        self.assertIn(7,o.marker_focus.hints)

    def test_pending_hazard_cancel_is_respected_and_stale_cannot_interrupt(self):
        o,stops=owner(None)
        send(o,9,8.)
        self.assertEqual(stops,[])
        self.assertNotIn(9,o.marker_focus.hints)
        o.cancel_reason='bumper'
        send(o,9)
        self.assertEqual(stops,[])
        self.assertEqual(o.cancel_reason,'bumper')
