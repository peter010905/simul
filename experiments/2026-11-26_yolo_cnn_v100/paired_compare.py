"""Paired frame bootstrap for two selected full-frame detectors."""
import argparse
import json
from pathlib import Path

import numpy as np

from evaluate import match


def load(folder):
    summary=json.loads((folder/'summary.json').read_text())
    threshold=summary['threshold']
    rows=[json.loads(x) for x in (folder/'frames.jsonl').read_text().splitlines()]
    counts=np.array([[v[k] for k in ('tp','fp','fn')] for row in rows
                     for v in [match(row['hits'],row['gt'],threshold)]],dtype=int)
    return summary,rows,counts


def f1(counts):
    tp,fp,fn=counts.sum(axis=-2).T
    return 2*tp/np.maximum(1,2*tp+fp+fn)


def main():
    p=argparse.ArgumentParser();p.add_argument('--first',type=Path,required=True);p.add_argument('--second',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    first,rows_a,count_a=load(a.first);second,rows_b,count_b=load(a.second)
    assert [r['file'] for r in rows_a]==[r['file'] for r in rows_b]
    rng=np.random.default_rng(261126);n=len(count_a)
    samples=rng.integers(0,n,size=(2000,n))
    differences=f1(count_b[samples])-f1(count_a[samples])
    point=float(f1(count_b)-f1(count_a))
    result=dict(first=first['kind']+'_'+first['variant'],second=second['kind']+'_'+second['variant'],
                frames=n,delta_f1_second_minus_first=point,
                frame_bootstrap_95_percentile_interval=[float(x) for x in np.percentile(differences,[2.5,97.5])],
                interpretation='paired image bootstrap on this supplied synthetic test set; not independent real-scene uncertainty')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
