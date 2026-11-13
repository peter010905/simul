"""Use observed image resolution to choose a reliable marker follow-up view."""
import argparse
from pathlib import Path
import shutil

from prepare_approach_witness_runtime import replace_once

ROOT=Path(__file__).resolve().parents[1]


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Use a new immutable candidate')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    path=package/'osp_mission/marker_focus.py';code=path.read_text()
    code=replace_once(code,'class MarkerFocus:\n',
        'class MarkerFocus:\n    desired_edge_pixels = 36.\n    uncertain_view_penalty_s = 8.\n')
    code=replace_once(code,"        self.hints[ident] = dict(point=point, observer=observer, stamp=stamp,",
        "        try: pixels=float(evidence.get('min_edge_pixels',0))\n"
        "        except (TypeError,ValueError): pixels=0.\n"
        "        pixels=pixels if math.isfinite(pixels) and pixels>0 else None\n"
        "        self.hints[ident] = dict(point=point, observer=observer, stamp=stamp, min_edge_pixels=pixels,")
    code=replace_once(code,"                features = candidate['features'].copy()",'''                # A short, low-resolution glimpse is fragile even when decoded.
                # Estimate closer-view resolution from this image, not true size.
                observed_pixels=hint.get('min_edge_pixels')
                predicted_pixels=(observed_pixels*observed_range/distance
                                  if observed_pixels is not None else None)
                resolution_penalty=(self.uncertain_view_penalty_s*max(0.,1.-predicted_pixels/self.desired_edge_pixels)
                                    if predicted_pixels is not None else 0.)
                cost+=resolution_penalty
                features = candidate['features'].copy()''')
    code=replace_once(code,"selected = dict(candidate, yaw=yaw, scan_angle=0., focus_id=ident, features=features)",
        "selected = dict(candidate, yaw=yaw, scan_angle=0., focus_id=ident, features=features,\n"
        "                                observed_min_edge_pixels=observed_pixels,\n"
        "                                predicted_min_edge_pixels=predicted_pixels,\n"
        "                                view_resolution_penalty_s=resolution_penalty)")
    path.write_text(code)
    path=package/'osp_mission/mission.py';code=path.read_text()
    code=replace_once(code,"attempt=self.marker_focus.attempts[focus['focus_id']]))+'\\n')",
        "attempt=self.marker_focus.attempts[focus['focus_id']],\n"
        "                            observed_min_edge_pixels=focus.get('observed_min_edge_pixels'),\n"
        "                            predicted_min_edge_pixels=focus.get('predicted_min_edge_pixels'),\n"
        "                            view_resolution_penalty_s=focus.get('view_resolution_penalty_s')))+'\\n')")
    code=replace_once(code,"report={'approach':self.approach.report(),'reason':self.reason,",
        "report={'marker_focus_profile':dict(\n"
        "                    desired_edge_pixels=getattr(getattr(self,'marker_focus',None),'desired_edge_pixels',None),\n"
        "                    uncertain_view_penalty_s=getattr(getattr(self,'marker_focus',None),'uncertain_view_penalty_s',None)),\n"
        "                'approach':self.approach.report(),'reason':self.reason,")
    path.write_text(code)
    shutil.copy2(ROOT/'scripts/test_pixel_focus_runtime.py',package/'test/test_pixel_focus_runtime.py')
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','destination'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
