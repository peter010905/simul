"""Stage the single loop-response gate supported by recorded SLAM replays.

No native completion or generalization is established by creating this stage.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[1]


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Preserve frozen and partial runtime stages')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    config=package/'config/slam.yaml';code=config.read_text()
    before='    loop_match_minimum_response_fine: 0.45'
    if code.count(before)!=1:raise ValueError('Unexpected predecessor loop gate')
    config.write_text(code.replace(before,'    loop_match_minimum_response_fine: 0.55'))
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    changed=[]
    original=source/'src/osp_mission'
    for path in package.rglob('*'):
        if path.is_file() and digest(path)!=digest(original/path.relative_to(package)):
            changed.append(str(path.relative_to(package)))
    if changed!=['config/slam.yaml']:raise RuntimeError(f'Unexpected source changes: {changed}')
    (destination.parent/'preparation.json').write_text(json.dumps(dict(
        source=str(source),destination=str(destination),changes=changed,
        parameter='loop_match_minimum_response_fine',before=.45,after=.55,
        native_queued=False,native_result_verified=False,production_promoted=False,
        evidence='repository_snapshot/second_L_localization_failure.json',
        limit='Broader recorded-space comparison and fresh native tests are still required.'),indent=2)+'\n')
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','destination'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
