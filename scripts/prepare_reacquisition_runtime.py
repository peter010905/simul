"""Stage landmark seed recovery separately from every frozen live runtime."""
import argparse
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[1]


def replace_once(code,old,new):
    if code.count(old)!=1:raise ValueError(f'Review changed source anchor:{old[:80]}')
    return code.replace(old,new)


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Create a fresh runtime stage')
    original=(source/'src/osp_mission/osp_mission/detector.py').read_text()
    code=replace_once(original,'from osp_mission.provisional_marker_cues import ProvisionalMarkerCues',
        'from osp_mission.provisional_marker_cues import ProvisionalMarkerCues\nfrom osp_mission.landmark_reacquisition import ObservationConsensus')
    code=replace_once(code,'        self.observations={}; self.pub=',
        '        self.confirmation_guard=ObservationConsensus()\n        self.observations={}; self.pub=')
    old='''                if history and np.linalg.norm(pos-np.median([v[1] for v in history],axis=0))>.5: self.reject(ident,'inconsistent');continue
                if history and stamp.nanoseconds-history[-1][0]<150_000_000: continue
                history.append((stamp.nanoseconds,pos)); del history[:-30]
'''
    new='''                if not hasattr(self,'confirmation_guard'):self.confirmation_guard=ObservationConsensus()
                verdict=self.confirmation_guard.update(ident,history,stamp.nanoseconds,pos,float(edges.min()),self.p('confirmations'))
                if verdict in ('inconsistent','invalid'):self.reject(ident,'inconsistent');continue
                if verdict=='duplicate':continue
                if verdict=='recovered':
                    event=self.confirmation_guard.last_recovery
                    self.diagnostics.setdefault('seed_reacquisitions',[]).append(event)
                    recovered=len(event['readable_consensus'])
                    self.diagnostics['accepted']+=recovered-1
                    detail['accepted']=detail.get('accepted',0)+recovered-1
'''
    code=replace_once(code,old,new)
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    (package/'osp_mission/detector.py').write_text(code)
    shutil.copy2(ROOT/'training_robust/policy_candidates/landmark_reacquisition.py',package/'osp_mission/landmark_reacquisition.py')
    shutil.copy2(ROOT/'scripts/test_reacquisition_runtime.py',package/'test/test_reacquisition_runtime.py')
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--destination',type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
