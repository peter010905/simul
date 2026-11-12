"""Prepare a separate small-marker inspection candidate from a frozen runtime."""
import argparse
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]


def replace_once(code, old, new):
    if code.count(old) != 1:
        raise ValueError(f'Review changed patch anchor: {old[:100]}')
    return code.replace(old, new)


def detector_code(code):
    code = replace_once(code, 'from osp_mission.landmark_identity import resolved_ids',
        'from osp_mission.landmark_identity import resolved_ids\nfrom osp_mission.provisional_marker_cues import ProvisionalMarkerCues')
    code = replace_once(code, '        self.observations={}; self.pub=',
        "        self.provisional=ProvisionalMarkerCues()\n        self.candidate_pub=self.create_publisher(String,'mission/marker_candidates',10)\n        self.observations={}; self.pub=")
    code = replace_once(code,
        "                if edges.min()<self.p('min_marker_pixels'): self.reject(ident,'small');continue",
        """                provisional = edges.min()<self.p('min_marker_pixels')
                if provisional:
                    self.reject(ident,'small')
                    if edges.min()<10.:continue""")
    code = replace_once(code, '                history=self.observations.setdefault(ident,[])',
        """                if provisional:
                    # A candidate requests a closer view. It never enters the
                    # recognition history, targets, map output or approach log.
                    if self.provisional.observe(ident,pos,stamp.nanoseconds/1e9,float(edges.min())):
                        evidence=dict(stamp=stamp.nanoseconds/1e9,id=ident,map_point=pos.tolist(),
                            min_edge_pixels=float(edges.min()),provisional=True,
                            camera_origin_map=[tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z],
                            observation_frame=self.p('map_frame'))
                        self.candidate_pub.publish(String(data=json.dumps(evidence)))
                    continue
                history=self.observations.setdefault(ident,[])""")
    return code


def focus_code(code):
    code = replace_once(code, "                                count=1 if old is None else old['count']+1)",
        "                                count=1 if old is None or old.get('provisional',False)!=bool(evidence.get('provisional',False)) else old['count']+1,\n                                provisional=bool(evidence.get('provisional',False)))")
    code = replace_once(code, "        if hint['count'] >= 3 or not 0. <= now-hint['stamp'] <= .75:",
        "        if (hint['count'] >= 3 and not hint.get('provisional',False)) or not 0. <= now-hint['stamp'] <= .75:")
    code = replace_once(code, "            if ident in confirmed or hint['count'] >= 3 or not 1. <= age <= 600.:",
        "            if ident in confirmed or (hint['count'] >= 3 and not hint.get('provisional',False)) or not 1. <= age <= 600.:")
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    package = args.destination / 'src/osp_mission'
    if args.destination.exists():
        raise SystemExit('Use a new destination. Frozen runtimes are preserved.')
    shutil.copytree(args.source / 'src/osp_mission', package,
                    ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    detector = package / 'osp_mission/detector.py'
    detector.write_text(detector_code(detector.read_text()))
    focus = package / 'osp_mission/marker_focus.py'
    focus.write_text(focus_code(focus.read_text()))
    mission = package / 'osp_mission/mission.py'
    mission.write_text(replace_once(mission.read_text(),
        "self.create_subscription(String,'mission/marker_observations',self.on_marker_hint,10)",
        "self.create_subscription(String,'mission/marker_observations',self.on_marker_hint,10)\n            self.create_subscription(String,'mission/marker_candidates',self.on_marker_hint,10)"))
    shutil.copy2(ROOT/'src/osp_mission/osp_mission/provisional_marker_cues.py', package/'osp_mission')
    shutil.copy2(ROOT/'scripts/test_small_marker_runtime.py', package/'test/test_small_marker_inspection.py')
    print(f'Staged small-marker inspection candidate: {args.destination}')


if __name__ == '__main__':
    main()
