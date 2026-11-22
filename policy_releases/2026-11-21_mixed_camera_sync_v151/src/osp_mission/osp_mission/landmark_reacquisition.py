"""Recover an unconfirmed landmark seed using three fresh readable positions."""
import numpy as np


class ObservationConsensus:
    def __init__(self):
        self.pending={}
        self.last_recovery=None

    def update(self,ident,history,stamp,point,pixels,minimum=3):
        if minimum<3:raise ValueError('At least three readable frames are required')
        self.last_recovery=None
        point=np.asarray(point,dtype=float)
        if point.shape!=(3,) or not np.isfinite(point).all() or not np.isfinite(pixels) or pixels<18:
            self.pending.pop(ident,None);return 'invalid'
        if history and stamp-history[-1][0]<150_000_000:return 'duplicate'
        consistent=not history or np.linalg.norm(point-np.median([r[1] for r in history],axis=0))<=.5
        if consistent:
            history.append((stamp,point.copy()));del history[:-30]
            self.pending.pop(ident,None);return 'accepted'
        # Already confirmed landmarks retain their original strict anchor.
        # Only a sub-threshold initial seed can be replaced by a new consensus.
        if len(history)>=minimum:
            self.pending.pop(ident,None);return 'inconsistent'
        rows=self.pending.setdefault(ident,[])
        if rows and stamp<=rows[-1][0]:return 'duplicate'
        if rows and (stamp-rows[-1][0]>600_000_000 or
                     max(np.linalg.norm(point-r[1]) for r in rows)>.25):rows.clear()
        if not rows or stamp-rows[-1][0]>=150_000_000:
            rows.append((stamp,point.copy(),float(pixels)))
        if len(rows)<minimum:return 'inconsistent'
        chosen=rows[-minimum:]
        self.last_recovery=dict(id=int(ident),old_seed=[dict(stamp_ns=int(t),point=p.tolist()) for t,p in history],
            readable_consensus=[dict(stamp_ns=int(t),point=p.tolist(),min_edge_pixels=pixels) for t,p,pixels in chosen])
        history[:]=[(t,p.copy()) for t,p,_ in chosen]
        self.pending.pop(ident,None);return 'recovered'
