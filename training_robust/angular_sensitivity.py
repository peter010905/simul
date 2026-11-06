"""Sensitivity to angular dynamics; this is not a learned policy improvement."""

from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil

import yaml

from evaluate import task,summarize


def main():
    out=Path('training_robust/angular_sensitivity');out.mkdir(exist_ok=False)
    snapshot=out/'source_snapshot';snapshot.mkdir()
    for source in Path('src/osp_mission/osp_mission').iterdir():
        if source.suffix in ('.py','.cpp','.so'):shutil.copy2(source,snapshot/source.name)
    shutil.copy2(__file__,snapshot/'driver.py');shutil.copy2(Path(__file__).parent/'evaluate.py',snapshot/'evaluate.py')
    reference=Path('src/osp_mission/config/nav2_assignment.yaml')
    behavior=yaml.safe_load(reference.read_text())['behavior_server']['ros__parameters']
    speed=behavior['max_rotational_vel'];acceleration=behavior['rotational_acc_lim']
    common=dict(resolution=.05,lidar_samples=640,lidar_body_frame=True,raycast_backend='native',continuous_shortcuts=True)
    variants=[('legacy',common),('derated',dict(common,angular_profile='trapezoid',max_turn_speed=.7,max_turn_acceleration=1.6)),
              ('configured_limits',dict(common,angular_profile='trapezoid',max_turn_speed=speed,max_turn_acceleration=acceleration))]
    cfgs=[yaml.safe_load(Path(f'scenarios_robust/v2/train/{8500100+i}.yaml').read_text()) for i in range(20)]
    weights=json.loads(Path('src/osp_mission/config/surface_endpoint_policy.json').read_text())['parameters']
    meta=dict(environment='2d_surrogate',split='v2/train',test_loaded=False,validation_loaded=False,
              planner='surface_endpoint',parameters=weights,seeds=[c['seed'] for c in cfgs],variants=variants,
              camera_width=960,camera_hz=5,reference=str(reference),reference_sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),
              actual_achieved_velocity_measured=False,default_angular_model_changed=False,
              assumption='Trapezoidal turns start and end at rest and reach their declared bounds when the angle allows; no Gazebo calibration',
              source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir()})
    (out/'configuration.json').write_text(json.dumps(meta,indent=2))
    results=[]
    with ProcessPoolExecutor(max_workers=2) as pool:
        for name,settings in variants:
            rows=[]
            for row in pool.map(task,[(c,'surface_endpoint',weights,5.,3,dict(settings,frame_quarter_turns=(c['seed']//20)%4)) for c in cfgs]):
                rows.append(row)
                if len(rows)%10==0:print(json.dumps(dict(name=name,completed=len(rows),total=len(cfgs))),flush=True)
            result=dict(name=name,settings=settings,summary=summarize(rows),rows=rows)
            (out/f'{name}.json').write_text(json.dumps(result,indent=2));results.append(result)
            print(json.dumps(dict(name=name,success=result['summary']['success_rate'],seconds=result['summary']['mean_seconds'],contacts=result['summary']['collisions'])),flush=True)
    (out/'completed.json').write_text(json.dumps(dict(episodes=60,unique_training_layouts=20,test_loaded=False,
        validation_loaded=False,gazebo_verified=False,policy_improvement_claimed=False,results=results),indent=2))


if __name__=='__main__':main()
