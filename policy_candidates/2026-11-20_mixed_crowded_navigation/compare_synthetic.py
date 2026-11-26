"""Compare controlled failure lifecycles and map counterexamples, without ROS motion."""
from concurrent.futures import Future
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np

from osp_mission._r32_navigation_core import Grid, connected_nav_cells
from osp_mission.center_wall import CenterWall
from osp_mission.crowded_navigation import CommonRoutes, CrowdedFlow
from osp_mission.mission import Mission
from osp_mission.navigation_flow import NavigationFlow


def legacy(name, source):
    spec = importlib.util.spec_from_file_location('baseline_'+name,source/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def failed_visits(cls,flow):
    node = NS(state='running',kind='nav',cancel_reason='navigation_stalled',
              goal_handle=object(),started=0.,now=lambda:10.,action_start=0.,plan_after=0.,
              destination=(1.,2.),excluded=[],navigation_flow=flow,exploration_memory=Mock(),
              marker_look=Mock(),face_look=Mock(),get_logger=lambda:Mock())
    for _ in range(3):
        future=Future();future.set_result(NS(status=5))
        cls.completed(node,future)
    return len(node.excluded)


def surface_attempts(cls):
    policy=cls();policy.phase='interior'
    anchor=np.array([1.,1.])
    goal=dict(surface_odom=anchor,normal_odom=0.)
    for stamp in (10.,20.):
        policy.dispatched(goal,stamp)
        if hasattr(policy,'navigation_result'):policy.navigation_result(6,'',stamp+1.)
    policy.elapsed=100.
    return len(policy.attempts('interior',anchor,0.,.5))


def gazebo_running():
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            if b'gz sim' in path.read_bytes():return True
        except OSError:pass
    return False


def compare(source):
    old_mission=legacy('mission',source).Mission
    old_center=legacy('center_wall',source).CenterWall
    cells=np.zeros((100,100),dtype=np.int16)
    cells[[0,-1],:]=100;cells[:,[0,-1]]=100
    nav=cells.copy();cells[18:82,50]=100;nav[:,50]=100;nav[25:75,50]=0
    grid,cost=Grid(cells,.05,(0.,0.,0.)),Grid(nav,.05,(0.,0.,0.))
    pose=(*grid.world(50,20),0.)
    x,y=grid.world(50,80);target=dict(x=x,y=y)
    routes=CommonRoutes(grid,cost,pose)
    alternate=routes.reposition(lambda _:False,[])
    rows=[]
    large=np.zeros((300,400),dtype=np.int16)
    large[[0,-1],:]=100;large[:,[0,-1]]=100
    large[70:100,100:115]=100;large[150:165,150:250]=100
    lg=Grid(large,.05,(-10.,-7.5,0.))
    for _ in range(12):
        before=time.perf_counter();CommonRoutes(lg,lg,(-3.,-1.,0.))
        rows.append((time.perf_counter()-before)*1000)
    return dict(validation='synthetic map and real Mission callback execution; no physical robot',
        failed_goals_added_to_permanent_visit_history=dict(baseline=failed_visits(old_mission,NavigationFlow()),candidate=failed_visits(Mission,CrowdedFlow())),
        failed_surface_attempts_still_counting_after_80s=dict(baseline=surface_attempts(old_center),candidate=surface_attempts(CenterWall)),
        incompatible_passages=dict(slam_individually_reachable=bool(grid.reachable(pose[:2],.32)[0][50,80]),
            nav2_individually_reachable=bool(connected_nav_cells(cost,pose[:2])[50,80]),
            candidate_accepts_target=bool(routes.allows(target)),
            alternate_common_goal_exists=alternate is not None),
        common_graph_extra_computation=dict(shape=list(large.shape),resolution_m=.05,samples=len(rows),
            median_ms=float(np.median(rows)),p95_ms=float(np.percentile(rows,95)),
            measured_while_gazebo_running=gazebo_running()),
        preserved_baseline_source_sha256={p:hashlib.sha256((source/p).read_bytes()).hexdigest()
            for p in ('mission.py','center_wall.py','phased_hybrid.py','transit_control.py')})


if __name__=='__main__':
    result=compare(Path(sys.argv[1]))
    Path(sys.argv[2]).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
