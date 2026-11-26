"""Recompute calibration from cached predictions without touching test frames."""
import argparse
import json
from pathlib import Path

import numpy as np

from evaluate import score


def main():
    p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,required=True)
    a=p.parse_args()
    rows=[json.loads(line) for line in (a.folder/'frames.jsonl').read_text().splitlines()]
    grid=list(np.arange(.05,.96,.05))+[.97,.98,.99,.995,.999,.9995,.9999,.99995,.99999]
    entries=[dict(threshold=float(t),**score(rows,float(t))) for t in grid]
    winner=max(entries,key=lambda x:x['overall']['f1'])
    old=json.loads((a.folder/'summary.json').read_text())
    old['selected']=winner
    (a.folder/'thresholds.json').write_text(json.dumps(entries,indent=2))
    (a.folder/'summary.json').write_text(json.dumps(old,indent=2))
    print(old['kind'],old['variant'],winner['threshold'],winner['overall'],flush=True)


if __name__=='__main__':main()
