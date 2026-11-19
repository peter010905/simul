"""Create one post-run report from a completed matched pair."""
import argparse
import json
from pathlib import Path
import sys

from inspection_time import summarize
from map_errors import measure

# Reuse the same clocks and definitions as the original Mixed comparison.
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'2026-11-19_mixed_two_views'))
from compare import compare


def report(baseline, candidate):
    results=compare(baseline,candidate)
    for key,folder in [('baseline',baseline),('candidate',candidate)]:
        inspection=summarize(folder)
        if abs(sum(inspection['seconds'].values())-results[key]['box_goal_to_next_dispatch_s'])>.01:
            raise ValueError('Inspection durations do not reconcile')
        results[key]['inspection_by_host']=inspection
        try:
            results[key]['map_alignment']=measure(folder)
        except ValueError as exc:
            results[key]['map_alignment']=dict(unavailable_reason=str(exc))
    return results


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('baseline',type=Path)
    parser.add_argument('candidate',type=Path)
    args=parser.parse_args()
    print(json.dumps(report(args.baseline,args.candidate),allow_nan=False,indent=2))
