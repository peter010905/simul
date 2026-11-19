"""Compare only identical timestamps and report candidate-only omissions separately."""
from pathlib import Path
import json
from statistics import mean


def paired(before, after):
    if before['run']!=after['run'] or before['frames']!=after['frames']:
        raise ValueError('Input replay streams differ')
    old,new = ({row['t']:row for row in d['probes']} for d in (before,after))
    both = sorted(old.keys() & new.keys())
    result = dict(run=before['run'],frames=before['frames'],matched=len(both),
                  v2_only=len(old.keys()-new.keys()),v3_only=len(new.keys()-old.keys()))
    for name, rows in [('v2',old),('v3',new)]:
        lengths=[rows[t]['walking_m'] for t in both if rows[t]['walking_m'] is not None]
        result[name]=dict(two_face_poses=sum(rows[t]['faces']>=2 for t in both),
                          mean_grid_path_m=round(mean(lengths),3) if lengths else None,
                          measured_lengths=len(lengths))
    return result


if __name__=='__main__':
    import sys
    base=Path(sys.argv[1])
    print(json.dumps([paired(json.loads(a.read_text()),json.loads(a.with_name(a.name.replace('-v2.json','-v3.json')).read_text()))
                      for a in sorted(base.glob('mixed-corner-*-v2.json'))],indent=2))
