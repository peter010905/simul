"""Measure recorded motor-stall episodes; this does not establish their physical cause."""
import argparse
import json
from pathlib import Path

from inspect_recorded_stalls import messages


def inspect(run):
    episodes=[]
    startup=[json.loads(line) for line in (run/'artifacts/startup.jsonl').read_text().splitlines()]
    stops=[r['wall_epoch_s'] for r in startup if r['event']=='stopped']
    stopped=min(stops) if stops else float('inf')
    current=None
    for _,message,stamp in messages(run/'rosbag',{'/hazard_detection'}):
        if stamp > stopped:break
        types={d.type for d in message.detections}
        if 3 in types:
            if current is None:
                current=dict(first=stamp,last=stamp,samples=0,initial_types=sorted(types),types=set(),frames=set())
            if types & {2,4}:current.setdefault('first_critical',stamp)
            current['last']=stamp
            current['samples']+=1
            current['types'].update(types)
            current['frames'].update(d.header.frame_id for d in message.detections)
        elif current is not None:
            current.update(clear=stamp,duration_to_clear_s=stamp-current['first'])
            episodes.append(current);current=None
    if current is not None:episodes.append(current)
    for episode in episodes:
        episode['types']=sorted(episode['types'])
        episode['frames']=sorted(episode['frames'])
    return dict(run=run.name,episodes=episodes)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=[inspect(run) for run in args.runs]
    args.output.write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
