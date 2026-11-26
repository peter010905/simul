"""Map-based route exercise. The robot follows safe grid paths; no physics or camera is simulated."""
import json
from pathlib import Path
import numpy as np

from osp_mission.wall_lap import WallLap
from osp_mission._r32_navigation_core import Grid


def run(chairs=False):
    cells=np.zeros((100,150),np.int8)
    cells[:3]=100;cells[-3:]=100;cells[:,:3]=100;cells[:,-3:]=100
    cells[44:53,68:77]=100
    if chairs:
        # 64 small supports in two rows, with non-passable gaps.
        for row in (12,86):
            for c in range(10,138,4):
                cells[row:row+1,c:c+2]=100
    grid=Grid(cells,.1,(0.,0.,0.))
    pose=(3.5,3.5)
    lap=WallLap()
    lap.position(*pose,9.)
    elapsed=9.;trail=[pose];goals=[]
    for _ in range(80):
        safe,dist=grid.reachable(pose,.32)
        goal=lap.choose(grid,dist,(*pose,0.),(0.,0.,0.),elapsed,600.,lambda _:False)
        if lap.phase=='interior':
            break
        if goal is None:
            elapsed+=3
            continue
        lap.dispatched(goal)
        goals.append([goal['x'],goal['y']])
        start=grid.index(*pose);cell=grid.index(goal['x'],goal['y'])
        route=[cell]
        while cell!=start:
            r,c=cell
            candidates=[(r-1,c),(r+1,c),(r,c-1),(r,c+1)]
            candidates=[p for p in candidates if grid.inside(*p)]
            previous=min(candidates,key=lambda p:dist[p])
            if dist[previous]>=dist[cell]:
                raise AssertionError('Cannot reconstruct safe route')
            cell=previous;route.append(cell)
        for cell in reversed(route[:-1]):
            assert safe[cell]
            pose=tuple(grid.world(*cell))
            trail.append(pose);elapsed+=.1/.30
            lap.position(*pose,elapsed)
        elapsed+=.4
    return dict(chairs=64 if chairs else 0,elapsed_s=round(elapsed,2),goals=len(goals),**lap.report(),
                trace=trail,waypoints=goals,scope='safe-grid route following, no physics or ArUco evaluation')

if __name__=='__main__':
    results=[run(False),run(True)]
    for row in results:
        assert row['phase']=='interior'
        print({k:v for k,v in row.items() if k not in ('trace','waypoints')})
    Path('/tmp/lap46-route-results.json').write_text(json.dumps(results,indent=2))
