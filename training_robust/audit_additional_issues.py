"""Read-only reproductions of additional route/observation issues in v14."""
import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--campaign',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    source=args.workspace.resolve()/'src/osp_mission'
    sys.path.insert(0,str(source))
    from osp_mission.edge_policy import EdgeCoverage
    from osp_mission.landmark_identity import resolved_ids
    from osp_mission.marker_approach import MarkerApproach

    focal=960/(2*math.tan(1.25/2))
    grid=np.zeros((50,80),dtype=np.int8);grid[:,50]=100
    coverage=EdgeCoverage()
    for stamp in (1.,1.2,1.4):
        coverage.observe(grid,.1,(0.,0.),(1.5,2.05,0.),np.array([[5.02,2.05]]),stamp,focal,1.25)
    pixels=focal*.08/3.5
    assert coverage.viewed.sum()>0 and pixels<18
    small=dict(kind='controlled_code_reproduction',credited_faces=int(coverage.viewed.sum()),
               marker_size_m=.08,distance_m=3.5,approx_projected_pixels=pixels,detector_min_pixels=18,
               interpretation='Coverage uses a0.12m prior even when the new development scenes contain0.08m markers. Native impact not yet measured.')

    histories={645:[(k,np.array([1.,1.,.45])) for k in range(30)],
               1023:[(k,np.array([1.05,1.,.45])) for k in range(11)]}
    ids,conflicts=resolved_ids(histories)
    assert not ids
    for k in range(100):
        histories[645].append((100+k,np.array([1.,1.,.45])))
        del histories[645][:-30]
    final_ids,_=resolved_ids(histories)
    assert not final_ids
    identity=dict(kind='controlled_code_reproduction',initial_conflict=conflicts,
                  additional_correct_frames=100,resolved_ids_after=sorted(final_ids),
                  interpretation='30-frame cap prevents a3:1 winner against11 stale conflicting observations; no age-based release exists. This condition has not been established in completed native trials.')

    approach=MarkerApproach()
    targets={1:(0.,0.),2:(3.,0.),3:(6.,0.)}
    for ident,point in targets.items():approach.observe_pose({ident:point},(point[0]+1.45,0.),1.,1.)
    corrected={1:(-.1,0.),2:(3.,0.),3:(6.,0.)}
    displayed=approach.report()['verified_ids'];complete=approach.complete(corrected)
    assert displayed==[1,2,3] and not complete
    status=dict(kind='controlled_code_reproduction',displayed_verified_ids=displayed,actual_complete_predicate=complete,
                interpretation='Status uses historical target coordinates, while completion checks current coordinates. This is a reporting inconsistency, not evidence of false completion.')

    loss=[]
    for episode in sorted((args.campaign/'episodes').iterdir()):
        files=sorted((episode/'policy_decisions').glob('*.npz'))
        for first,second in zip(files,files[1:]):
            with np.load(first) as a,np.load(second) as b:
                old=int(a['viewed'].sum())
                if old<20:continue
                replay=EdgeCoverage();replay.grid(a['data'],float(a['resolution']),a['origin'])
                replay.viewed=a['viewed'].copy();replay.counts=replay.viewed.astype(np.uint8)*3
                replay.grid(b['data'],float(b['resolution']),b['origin'])
                retained=int(replay.viewed.sum())
                if retained<old*.5:
                    loss.append(dict(episode=episode.name,before_seconds=float(a['elapsed']),after_seconds=float(b['elapsed']),
                        previous_credited_faces=old,retained_from_previous=retained,actual_later_credited_faces=int(b['viewed'].sum()),
                        first_snapshot=str(first),second_snapshot=str(second)))
    report=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        workspace=str(args.workspace),small_marker_coverage=small,identity_recovery=identity,status_consistency=status,
        coverage_memory_loss=dict(kind='native_snapshot_replay',severe_pairs=len(loss),pairs=loss,
            limitation='Endpoint replay omits intermediate updates. Some invalidation is necessary after SLAM corrections; route/time penalty and safe retention rules remain unproven.'),
        source_hashes={name:hashlib.sha256((source/'osp_mission'/name).read_bytes()).hexdigest()
                       for name in ['edge_policy.py','coverage_resize.py','landmark_identity.py','marker_approach.py']},
        runtime_sources_changed=False)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(controlled_reproductions=3,native_memory_loss_pairs=len(loss),output=str(args.output))))


if __name__=='__main__':main()
