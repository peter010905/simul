"""Apply the .50 white-leg rule to logged white-leg decisions (timeline not re-simulated)."""
import json, math, sys
for run in sys.argv[1:]:
    rows=[json.loads(l) for l in open(run+'/artifacts/center_wall.jsonl')]
    white=[(r['elapsed'],r.get('white_point')) for r in rows if r.get('reason')=='white_leg_candidate']
    used=[];streak=0;block=-1e9;kept=[]
    for t,p in white:
        if t<block: continue
        if p and any(math.dist(p,u)<1.0 for u in used): continue
        kept.append(t); used.append(p) if p else None
        streak+=1
        if streak>=3: block=t+150; streak=0
    print(run.split('/')[-1], 'logged white-leg decisions', len(white), '-> allowed by .50', len(kept), 'of', len(rows), 'decisions; allowed at', [round(x) for x in kept])
