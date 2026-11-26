"""Run identical small behavioral counterexamples in parent and candidate imports."""
from concurrent.futures import Future
from types import SimpleNamespace
import json

import numpy as np
from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall
from osp_mission.interior_priority import InteriorPriority
from osp_mission.marker_focus import MarkerFocus
from osp_mission.mission import Mission


f=MarkerFocus()
f.observe(dict(provisional=True,id=42,stamp=10.,map_point=[3.,3.,.2],
              observation_frame='map',min_edge_pixels=8.),[1.,3.],10.1)
for _ in range(8):f.choose(np.zeros((80,80),np.int8),.1,(0.,0.),(1.,3.),0.,[],{},10.15)
rows=dict(unexecuted_focus_attempts=f.attempts.get(42,0))
owner=SimpleNamespace(action_generation=1,goal_handle=object(),cancel_started=None,
    state='running',kind='nav',cancel_reason='',now=lambda:2.,started=0.,
    contact_inspection=None,contact_pending=False,hybrid=None,near_pass=None,
    navigation_flow=None,exploration_memory=None,excluded=[],destination=(1.,2.))
result=Future();result.set_result(SimpleNamespace(status=4));Mission.completed(owner,result)
rows['successful_destination_hard_exclusions']=len(owner.excluded)
data=np.zeros((90,110),np.int8);data[[0,-1],:]=100;data[:,[0,-1]]=100
grid=Grid(data,.1,(0.,0.,0.));_,dist=grid.reachable((2.,3.),.32)
box=dict(x=3.,y=3.,yaw=0.,r32_kind='interior_surface',batch_box=[3.,3.5],
         surface_map=[3.,3.],previous_attempts=0,completed_box=False,marker_gain=.18)
frontier=dict(x=8.,y=3.,yaw=0.,distance=6.,features=np.array([2.,8.,6.,0.,0.]))
chosen=InteriorPriority().choose(grid,dist,(0.,0.,0.),[(1.,box)],[frontier],[],
        [(2.,3.),(2.1,3.),(2.2,3.),(2.3,3.)],500.,lambda _:False)
rows['competing_close_box_and_new_frontier']=chosen['r32_kind']
s=CenterWall();s.wall_lap.finish('comparison')
s.region_transit.mode=lambda *args:'resume'
s.region_transit.choose=lambda *args:dict(x=6.,y=5.,r32_kind='interior_frontier',features=np.zeros(5))
s.survey.choose=lambda *args,**kwargs:None
chosen=s.decide(grid,grid,(2.,3.,0.),(0.,0.,0.),np.zeros((4,)+data.shape,bool),200.,600.,lambda _:False)
rows['region_resume_selected']=bool(chosen is not None and s.reason=='region_resume')
print(json.dumps(rows,indent=2))
