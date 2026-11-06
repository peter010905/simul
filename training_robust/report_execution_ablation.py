"""Report paired observation-execution results without selecting test layouts."""

import json
from pathlib import Path
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def compact(result):
    rows=result['rows'];summary=result['summary']
    out={key:summary[key] for key in ('success_rate','mean_seconds','successful_mean_seconds','collisions','mean_found')}
    for key in ('by_mount','by_obstacles'):out[key]={k:v['success_rate'] for k,v in summary[key].items()}
    out['by_frame']={str(frame):sum(r['success'] for r in rows if (r['seed']//20)%4==frame)/sum((r['seed']//20)%4==frame for r in rows) for frame in range(4)}
    for key in ('observation_alignment_seconds','observation_sweep_seconds','avoidable_alignment_radians'):
        out[key]=statistics.mean(r[key] for r in rows)
    return out


def main():
    root=Path('training_robust/execution_ablation')
    variants=json.loads((root/'variants.json').read_text())
    runs={name:json.loads((root/f'{name}.json').read_text()) for name,_,_ in variants}
    summaries={name:compact(run) for name,run in runs.items()};paired={}
    for planner in ('surface_endpoint','symmetric_edge'):
        before={r['seed']:r for r in runs[planner+'_baseline']['rows']}
        after={r['seed']:r for r in runs[planner+'_efficient']['rows']}
        assert before.keys()==after.keys()
        paired[planner]=dict(gained=sum(after[s]['success'] and not before[s]['success'] for s in before),
                             lost=sum(before[s]['success'] and not after[s]['success'] for s in before),
                             both_failed=sum(not before[s]['success'] and not after[s]['success'] for s in before))
    report=dict(environment='2d_surrogate',split='v2/train',evaluations=320,unique_layouts=80,
                validation_loaded=False,test_loaded=False,gazebo_verified=False,
                summaries=summaries,paired=paired)
    (root/'comparison.json').write_text(json.dumps(report,indent=2))
    names=list(runs);labels=['endpoint\nbaseline','endpoint\nshort alignment','symmetric\nbaseline','symmetric\nshort alignment']
    fig,axes=plt.subplots(1,2,figsize=(11,4.4));x=np.arange(4)
    rates=[summaries[n]['success_rate']*100 for n in names]
    axes[0].bar(x,rates,color=['#4975a5','#76a5c9','#4975a5','#76a5c9'])
    for i,value in enumerate(rates):axes[0].text(i,value+1,f'{value:g}%',ha='center')
    axes[0].set_ylim(0,105);axes[0].set_ylabel('3-marker completion (%)')
    bottom=np.zeros(4)
    parts=[('observation_alignment_seconds','Alignment'),('observation_sweep_seconds','Observation sweep'),('other','Other modeled time')]
    for key,label in parts:
        values=np.array([s['mean_seconds']-s['observation_alignment_seconds']-s['observation_sweep_seconds'] if key=='other' else s[key] for s in summaries.values()])
        axes[1].bar(x,values,bottom=bottom,label=label);bottom+=values
    axes[1].set_ylabel('Mean seconds, failures included');axes[1].legend(fontsize=8)
    for ax in axes:ax.set_xticks(x,labels);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    fig.suptitle('Paired training comparison: 80 layouts, unchanged sensors and motion limits\nContinuous observation arc preserved; scan direction/start may change',fontsize=11)
    fig.tight_layout();fig.savefig(root/'comparison.png',dpi=150);plt.close(fig)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
