"""Cover observed faces with a small set of views, then shorten their open tour."""

import math
import numpy as np
from scipy.sparse.csgraph import dijkstra

from osp_mission.view_route_policy import options
from osp_mission.surface_endpoint_policy import DEFAULT


def shorten_open_tour(start_distances, pair_distances):
    """Nearest-neighbour initialization followed by open-path 2-opt."""
    pending=set(range(len(start_distances)));order=[]
    while pending:
        distances=start_distances if not order else pair_distances[order[-1]]
        nearest=min(pending,key=lambda j:(distances[j],j))
        order.append(nearest);pending.remove(nearest)
    def cost(path):
        return start_distances[path[0]]+sum(pair_distances[a,b] for a,b in zip(path,path[1:]))
    best=cost(order)
    for _ in range(12):
        improved=False
        for first in range(len(order)):
            for last in range(first+1,len(order)):
                candidate=order[:first]+order[first:last+1][::-1]+order[last+1:]
                fresh=cost(candidate)
                if fresh<best-1e-9:
                    order,best=candidate,fresh;improved=True
        if not improved:break
    return order


def choose(candidates,weights=DEFAULT):
    if not candidates:return None
    context=candidates[0]['_route_context'];coverage=context['coverage']
    # A complete local turn records both sides that are visible from the stop.
    eligible=np.array([abs(c['scan_angle'])>=2*math.pi-1e-6 or c['features'][0]==0 for c in candidates])
    ids=np.flatnonzero(eligible)
    if not len(ids):return None
    views=coverage[ids];remaining=np.ones(views.shape[1])
    frontier=np.array([candidates[i]['features'][1] for i in ids])
    positions=np.array([[candidates[i]['x'],candidates[i]['y']] for i in ids])
    revisits=np.array([candidates[i]['features'][4] for i in ids])
    active=np.ones(len(ids),dtype=bool);selected=[]
    for _ in range(12):
        surface=np.asarray(views@remaining).reshape(-1)*context['resolution']
        mass=weights[0]*surface+weights[1]*frontier
        score=np.where(active,mass/(1.+weights[4]*revisits*.1),-np.inf)
        best=int(np.argmax(score))
        if not np.isfinite(score[best]) or mass[best]<.25:break
        selected.append(int(ids[best]));remaining[candidates[ids[best]]['face_ids']]=0.
        separation=np.linalg.norm(positions-positions[best],axis=1)
        frontier*=1-np.exp(-separation**2/9.)
        active &= separation>.45
    if not selected:return None
    cells=context['cells'][selected]
    pair=dijkstra(context['network'],directed=False,indices=cells)[:,cells]
    start=np.array([candidates[i]['distance'] for i in selected])
    tour=shorten_open_tour(start,pair)
    result=dict(candidates[selected[tour[0]]])
    result['planned_view_count']=len(selected)
    return result
