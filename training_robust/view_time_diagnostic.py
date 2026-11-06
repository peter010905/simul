"""Measure motion, rotation and dwell time without changing policy decisions."""

from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import numpy as np
import yaml

from osp_mission.fast_sim import SearchEnv


class TimedEnvironment(SearchEnv):
    def __init__(self,*args,**kwargs):
        self.rotation_seconds=0.;self.dwell_seconds=0.
        super().__init__(*args,**kwargs)

    def rotate(self,*args,**kwargs):
        before=self.elapsed
        super().rotate(*args,**kwargs)
        self.rotation_seconds+=self.elapsed-before

    def hold(self,*args,**kwargs):
        before=self.elapsed
        super().hold(*args,**kwargs)
        self.dwell_seconds+=self.elapsed-before


def run(arguments):
    seed,planner=arguments
    cfg=yaml.safe_load(Path(f'scenarios_robust/v2/train/{seed}.yaml').read_text())
    settings=dict(resolution=.05,lidar_samples=640,lidar_body_frame=True,raycast_backend='native',
                  continuous_shortcuts=True,frame_quarter_turns=(seed//20)%4)
    if planner=='edge_glance':settings['travel_glance_threshold']=1.5
    env=TimedEnvironment(cfg,seed=seed+123456,planner=planner,camera_width=960,camera_hz=5,
                         detection_dropout=cfg['sensor_stress']['detection_dropout'],**settings)
    env.speed=min(env.speed,cfg['sensor_stress']['speed_limit'])
    weights=json.loads(Path('src/osp_mission/config/surface_endpoint_policy.json').read_text())['parameters']
    result=env.run(weights)
    source=f'training_robust/view_route_screen/{planner}.json'
    if planner=='view_tour':source='training_robust/view_tour_screen/view_tour.json'
    if planner=='edge_glance':source='training_robust/travel_glance_screen/glance_1.5.json'
    reference=next(r for r in json.loads(Path(source).read_text())['rows'] if r['seed']==seed)
    assert all(result[k]==v for k,v in reference.items() if k in result)
    moving=env.distance/env.speed
    return dict(seed=seed,planner=planner,exact_metrics=True,found=result['found'],success=result['success'],
                seconds=result['seconds'],motion_seconds=moving,rotation_seconds=env.rotation_seconds,
                dwell_seconds=env.dwell_seconds,other_seconds=env.elapsed-moving-env.rotation_seconds-env.dwell_seconds,
                decisions=env.steps,glances=env.travel_glances)


if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:
        rows=list(pool.map(run,[(seed,planner) for seed in [8500003,8500019]
                               for planner in ['surface_endpoint','edge','view_route','view_tour','edge_glance']]))
    report=dict(test_loaded=False,validation_loaded=False,diagnostic_only=True,rows=rows)
    Path('training_robust/view_time_diagnostic.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)
