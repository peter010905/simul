"""Align native exploration reward with the actual camera's small-marker readable range."""
import argparse
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[1]


def replace_once(code,old,new):
    if code.count(old)!=1:raise ValueError(f'Review changed camera-gain anchor:{old[:90]}')
    return code.replace(old,new)


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Use a new immutable runtime')
    code=(source/'src/osp_mission/osp_mission/edge_policy.py').read_text()
    if 'if len(indices)>counts[best]*1.5:' in code:
        raise ValueError('Require the bounded-sweep source;do not restore full-circle reward')
    code=replace_once(code,'class EdgeCoverage:', '''def readable_distance(focal):
    # Fixed minimum-size prior; never obtain a scenario marker's true size.
    if focal is None or not math.isfinite(focal) or focal<=0:return 0.
    return .85*focal*.08/18


class EdgeCoverage:''')
    code=replace_once(code,'lengths*resolution<.85*focal*.08*incidence/18',
        'lengths*resolution<readable_distance(focal)*incidence')
    code=replace_once(code,'collect_faces=False,full_scans=False):',
        'collect_faces=False,full_scans=False,inspection_focal=None):')
    code=replace_once(code,'    free=data==0;occupied=data>50;unknown=data<0',
        '    # Native mission always supplies calibration (zero until available).\n'
        '    # Keep the old bare options API for offline callers.\n'
        '    max_readable=3.7 if inspection_focal is None else readable_distance(inspection_focal)\n'
        '    free=data==0;occupied=data>50;unknown=data<0')
    code=replace_once(code,'lengths*resolution<3.7*incidence',
        'lengths*resolution<max_readable*incidence')
    mission=(source/'src/osp_mission/osp_mission/mission.py').read_text()
    anchor="        option_settings={'preferred_clearance':self.surface_settings['view_standoff']} if self.surface_settings else {}"
    mission=replace_once(mission,anchor,anchor+'''
        if self.edge_mode:
            calibration=getattr(self,'inspection_camera',None)
            option_settings['inspection_focal']=calibration[0] if calibration is not None else 0.''')
    mission=replace_once(mission,"                        marker_override='approach_id' in selected or 'focus_id' in selected,",
        "                        inspection_focal_px=option_settings.get('inspection_focal'),\n"
        "                        readable_distance_m=edge_policy.readable_distance(option_settings.get('inspection_focal')),\n"
        "                        marker_override='approach_id' in selected or 'focus_id' in selected,")
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    (package/'osp_mission/edge_policy.py').write_text(code)
    (package/'osp_mission/mission.py').write_text(mission)
    shutil.copy2(ROOT/'scripts/test_calibrated_gain_runtime.py',package/'test/test_calibrated_gain_runtime.py')
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','destination']:parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
