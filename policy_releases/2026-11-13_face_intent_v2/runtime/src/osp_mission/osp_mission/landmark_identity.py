"""Resolve conflicting IDs at one observed location without an ID allowlist."""
import numpy as np


def resolved_ids(histories, confirmations=3, conflict_radius=.12):
    points={i:np.median([row[1] for row in rows],axis=0)
            for i,rows in histories.items() if len(rows)>=confirmations}
    suppressed=set()
    conflicts=[]
    ids=sorted(points)
    for n,a in enumerate(ids):
        for b in ids[n+1:]:
            distance=float(np.linalg.norm(points[a]-points[b]))
            if distance>conflict_radius:
                continue
            na,nb=len(histories[a]),len(histories[b])
            if na>=max(6,3*nb):
                suppressed.add(b);winner=a
            elif nb>=max(6,3*na):
                suppressed.add(a);winner=b
            else:
                # Do not guess which of two equally supported IDs is correct.
                suppressed.update((a,b));winner=None
            conflicts.append(dict(ids=[a,b],distance_m=distance,counts=[na,nb],winner=winner))
    return set(points)-suppressed,conflicts
