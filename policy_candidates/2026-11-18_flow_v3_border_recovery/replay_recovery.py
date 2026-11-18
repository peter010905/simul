"""Compare with the frozen square-priority replay under the same four recorded routes."""
import argparse
import json
import math
import sys
from pathlib import Path

import cv2

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'2026-11-18_flow_v3_square_face'))
from compare import compare
from recovery import borders, plane_hit, can_confirm


def evaluate(run, output, baseline, *, target_calibration=None):
    trace=[]
    def frame(t,candidates,tracks,looks,readable,camera,cal):
        if not candidates and not readable:
            return
        choices=[]
        for look in looks:
            xy=look['point']
            bearing=math.atan2(xy[1]-camera[1],xy[0]-camera[0])-camera[2]
            choices.append(dict(action=look['action'],u=round(cal.k[2]-cal.k[0]*math.tan(bearing),1)))
        trace.append(dict(t=round(t,3),image=candidates,attached=len(tracks),choices=choices,readable=readable))
    result=compare(run,output,baseline,detect_borders=borders,trace=frame,associate_target=plane_hit,confirm_border=can_confirm,target_calibration=target_calibration)
    (output/(run.name+'-trace.json')).write_text(json.dumps(trace))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('baseline',type=Path)
    p.add_argument('output',type=Path)
    p.add_argument('runs',nargs='+',type=Path)
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(1)
    results=[]
    for run in args.runs:
        results.append(evaluate(run,args.output,args.baseline))
        print(json.dumps(results[-1]),flush=True)
        (args.output/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
