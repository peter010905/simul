import math
from types import SimpleNamespace
import unittest

import numpy as np

from osp_mission._r32_navigation_core import Grid
from osp_mission.enclosed_surface import EnclosedSurface
from osp_mission.followthrough import FollowThrough
from osp_mission.marker_approach import MarkerApproach


def fixture():
    data=np.zeros((160,160),dtype=np.int8)
    data[:2,:]=data[-2:,:]=100;data[:,:2]=data[:,-2:]=100
    data[57:70,54:95]=100
    data[69:90,54:64]=100
    return data


def goal(face, yaw, point, seconds=11, **flags):
    return SimpleNamespace(seconds=seconds,goal=dict(x=point[0],y=point[1],surface_map=face,
                           normal_odom=yaw,previous_attempts=0,**flags))


class FollowTest(unittest.TestCase):
    def setUp(self):
        self.data=fixture();self.robot=(2.,3.15,0.);self.anchor=(2.70,3.15)
        self.state=FollowThrough()
        self.state.dispatched(dict(r32_kind='interior_surface',normal_odom=math.pi,
                              surface_odom=self.anchor,survey_nav_odom=self.robot[:2],survey_expected_s=18.),0.)
        self.north=goal((2.95,4.50),math.pi/2,(2.95,5.10))
        self.back=goal(self.anchor,math.pi,(2.0,3.15))

    def select(self,options,wait=None,data=None,robot=None,now=20.):
        grid=Grid(self.data if data is None else data,.05,(0.,0.,0.))
        return self.state.choose(grid,robot or self.robot,(0.,0.,0.),
                                 dict(interior=options,**({'wall':[1]} if wait is not None else {})),
                                 {'wall':wait} if wait is not None else {},90.,now,400.)

    def test_large_irregular_body_after_arrival(self):
        out=self.select([self.back,self.north])
        self.assertIsNotNone(out)
        self.assertEqual(out[1]['survey_follow_geometry'],'enclosed_surface')
        self.assertIs(out[0],self.north)
        self.assertIsNone(self.select([self.north],robot=(.65,2.,0.)))

    def test_external_wall_and_unknown_gap_are_deferred(self):
        extended=self.data.copy();extended[58:63,:56]=100
        self.assertIsNone(self.select([self.north],data=extended))
        uncertain=self.data.copy();uncertain[58:63,:56]=-1
        self.assertIsNone(self.select([self.north],data=uncertain))

    def test_other_object_repeat_and_occlusion(self):
        griddata=self.data.copy();griddata[80:87,95:108]=100
        other=goal((4.85,4.35),math.pi/2,(4.85,5.0))
        self.assertIsNone(self.select([other],data=griddata))
        self.assertIsNone(self.select([self.back]))
        griddata[95:98,57:62]=100
        self.assertIsNone(self.select([self.north],data=griddata))

    def test_budget_and_stored_sides(self):
        self.assertIsNone(self.select([self.north],wait=-65.,now=20.))
        first=self.select([self.north]);self.assertIsNotNone(first)
        self.state.dispatched(first[1],20.)
        self.assertIsNone(self.select([self.north],robot=(*first[1]['survey_follow_nav'],0.),now=25.))
        self.assertIsNone(self.select([goal((2.95,4.50),math.pi/2,(2.95,5.1),tagged_box=True)]))

    def test_rotated_raster_owners_and_poles(self):
        g=Grid(self.data,.05,(1.2,-1.,math.pi/3))
        point=g.world(62.5,53.5); d=g.world(62.5,52.5)-point;d/=np.linalg.norm(d)
        robot=tuple(g.world(62.5,40.))+(.1,)
        island=EnclosedSurface(g,robot,point,3.8)
        self.assertIsNotNone(island.owner(point,d))
        pole=np.zeros((100,100),dtype=np.int8);pole[49,48]=100
        g=Grid(pole,.05,(0.,0.,0.));island=EnclosedSurface(g,(2.,2.,0.),(2.4,2.45),3.)
        owner=island.owner((2.4,2.475),np.array([-1.,0.]));self.assertIsNotNone(owner)
        self.assertIsNone(island.center(owner))

    def test_island_center_in_large_rotated_map(self):
        data=np.zeros((300,350),dtype=np.int8)
        data[185:205,235:267]=100
        grid=Grid(data,.05,(.9,1.1,math.pi/4))
        face=grid.world(194.5,234.5)
        normal=grid.world(194.5,233.5)-face
        normal/=np.linalg.norm(normal)
        robot=(*grid.world(194.5,217.),0.)
        island=EnclosedSurface(grid,robot,face,3.8)
        owner=island.owner(face,normal)
        self.assertIsNotNone(owner)
        np.testing.assert_allclose(island.center(owner),grid.world(194.5,250.5))


class RosterTest(unittest.TestCase):
    def test_known_and_undisclosed_counts_with_arbitrary_ids(self):
        for known in (0,1,3,4):
            a=MarkerApproach(known);ids=[0,156,702,1023]
            a.confirmed_ids=set(ids);a.range_witnesses={i:{} for i in ids}
            roster={i:(0.,0.) for i in ids}
            self.assertEqual(a.complete(roster),known==4)
            self.assertEqual(a.complete({i:roster[i] for i in ids[:3]}),known==3)
        a=MarkerApproach(3);self.assertFalse(a.complete({i:(0.,0.) for i in (1,2,3)}))
        for bad in (-1,1025,1.5,True):
            with self.assertRaises(ValueError):MarkerApproach(bad)


if __name__=='__main__':unittest.main()
