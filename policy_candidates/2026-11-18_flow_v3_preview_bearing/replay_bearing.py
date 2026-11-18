"""Paired pre-decode target localization on frozen camera recordings."""
import argparse
import json
from pathlib import Path
import sys

import cv2
from preview_geometry import preview_geometry

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'2026-11-18_flow_v3_border_recovery'))
from replay_recovery import evaluate

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('baseline',type=Path);p.add_argument('output',type=Path)
    p.add_argument('runs',nargs='+',type=Path)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(1)
    results=[]
    for run in args.runs:
        r=evaluate(run,args.output,args.baseline,target_calibration=preview_geometry)
        results.append(r)
        (args.output/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
        e=next(iter(r['marker_checks'].values()))
        print(run.name,e.get('before_either_decoder_or_log',e['before_first_observation'])['immediate'],flush=True)
