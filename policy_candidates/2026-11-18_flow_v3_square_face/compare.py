"""Paired offline replay; marker identity, observation times and locations are audit-only."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent/'2026-11-18_flow_v3_revisit_replay'))
sys.path.insert(0, str(HERE.parent/'2026-11-18_flow_v3_visual_cue'/'overlay'/'osp_mission'))
from replay import replay
from square_face import SquarePriority, associate, borders
from surface_memory import local_observations
from osp_mission.marker_pair import decode


def compare(run, output, baseline_dir):
    memory = SquarePriority()
    recorded, decoded = {}, {}
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    startup = (run/'artifacts/startup.jsonl').read_text().splitlines()
    start = next(e['wall_epoch_s'] for e in map(json.loads, startup) if e['event']=='mission_started')
    counts = Counter()

    def reference(ident, t, point, base):
        recorded.setdefault(int(ident), (t, point.copy(), np.array(base[:2])))

    def frame(t, stamp, laser, origin, cam, cal, image):
        counts['synchronized_frames'] += 1
        candidates = borders(image)
        # Separate evaluation channel. Its values cannot affect tracking or choices.
        corners, ids = decode(image, dictionary)
        readable = []
        for quad, ident in zip(corners, () if ids is None else ids.ravel()):
            edges = np.linalg.norm(quad[0] - np.roll(quad[0], 1, axis=0), axis=1)
            if min(edges) >= 18:
                readable.append((int(ident), float(np.mean(quad[0,:,0])), float(min(edges))))
        samples = list(local_observations(laser, origin, cam, cal, image)) if candidates or readable else []
        tracks = []
        for u, _, size in candidates:
            hit = associate(u, size, samples, cam, cal)
            if hit is not None:
                tracks.append((*hit, size))
        counts['frames_with_border'] += bool(candidates)
        counts['frames_with_attached_border'] += bool(tracks)
        memory.track_frame(tracks, stamp, t, cam)
        for ident, u, size in readable:
            entry = decoded.setdefault(ident, {'t': t, 'points': [], 'observers': []})
            if t-entry['t'] > 1:
                continue
            hit = associate(u, size, samples, cam, cal)
            if hit:
                entry['points'].append(hit[0])
                entry['observers'].append(cam[:2])

    replay(run, output, memory=memory, inspect_frame=frame, record_reference=reference)
    baseline = json.loads((baseline_dir/(run.name+'.json')).read_text())
    raw = json.loads((output/(run.name+'.json')).read_text())
    assert baseline['summary']['counts']['baseline_decisions'] == raw['summary']['counts']['baseline_decisions']
    def proposal(t, kind, source, face):
        return (t, kind, source, face.xy, face.normal)
    ours = [proposal(t, d['kind'], d['source'], memory.faces[d['face_id']]) for t,d in memory.events]
    base = []
    for d in baseline['proposals']:
        f = baseline['faces'][d['face_id']]
        base.append((start+d['mission_s'],d['kind'],'geometry',np.array([f['x'],f['y']]),np.array([f['nx'],f['ny']])))
    def audit(point, observer, cutoff):
        def same(p, n):
            return np.linalg.norm(p-point)<=.30 and n@(observer-p)>0
        relevant = [t for t,p,n,_ in memory.signals if t<cutoff and same(p,n)]
        def selected(events, source=None, covered=False):
            times = [t for t,k,s,p,n in events if t<cutoff and same(p,n) and
                     (k=='covered')==covered and (source is None or source==s)]
            return None if not times else round(min(times)-start,3)
        pre_looks = [look for look in memory.looks if look['t'] < cutoff]
        matched = [look for look in pre_looks if same(look['point'],look['normal'])]
        first = matched[0] if matched else None
        immediate = dict(all_choices_before=len(pre_looks), matching_choices=len(matched),
                         first_choice_matched=bool(pre_looks and same(pre_looks[0]['point'],pre_looks[0]['normal'])),
                         matched_s=round(first['t']-start,3) if first else None,
                         action=first['action'] if first else None,
                         lead_s=round(cutoff-first['t'],3) if first else None,
                         yaw_delta_deg=first['yaw_delta_deg'] if first else None)
        return dict(immediate=immediate,eligible_decisions_between_signal_and_reference=
                    sum(min(relevant)<t<cutoff for t in memory.decision_times) if relevant else None,
                    first_same_face_signal_s=round(min(relevant)-start,3) if relevant else None,
                    old_selected_s=selected(base),new_selected_s=selected(ours),
                    square_selected_s=selected(ours,'square'),
                    existing_goal_already_covers_square_s=selected(ours,'square',True),
                    all_square_choices_before=sum(t<cutoff and s=='square' and k!='covered' for t,k,s,_,_ in ours))
    audits = {}
    for ident in sorted(set(decoded)|set(recorded)):
        rec, video = recorded.get(ident), decoded.get(ident)
        entry = {'first_recorded_s':round(rec[0]-start,3) if rec else None,
                 'first_video_decode_s':round(video['t']-start,3) if video else None}
        if rec:
            cutoff, point, observer = rec
            entry['reference']='recorded marker odometry'
        elif video['points']:
            cutoff = video['t']
            points = np.stack(video['points'])
            point,observer = np.median(points,axis=0),np.median(video['observers'],axis=0)
            entry.update(reference='video bearing plus lidar proxy',proxy_frames=len(points),
                         proxy_max_spread_m=round(float(np.linalg.norm(points-point,axis=1).max()),3))
        else:
            entry['reference']='no usable surface location'
            audits[str(ident)] = entry
            continue
        entry['before_first_observation']=audit(point,observer,cutoff)
        if video and rec:
            entry['before_either_decoder_or_log']=audit(point,observer,min(cutoff,video['t']))
        audits[str(ident)] = entry
    return dict(run=run.name,eligible_decisions=raw['summary']['counts']['baseline_decisions'],
                old_choices=len(base),frame_counts=dict(counts),
                repeat_frame_signals=len({t for t,_,_,_ in memory.signals}),
                mapped_signal_decisions=memory.stats['decisions_with_mapped_signal'],
                decisions_by_source_and_action=dict(Counter(s+'_'+k for _,k,s,_,_ in ours)),
                immediate_actions=dict(Counter(d["action"] for d in memory.looks)),
                marker_checks=audits)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline_dir', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('runs', nargs='+', type=Path)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(1)
    results=[]
    for path in args.runs:
        results.append(compare(path,args.output,args.baseline_dir))
        print(json.dumps(results[-1]),flush=True)
        (args.output/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
