"""Resume a recorded training experiment using its frozen original package."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run',required=True)
    parser.add_argument('--generation-limit',type=int,default=2)
    parser.add_argument('--workers',type=int,default=4)
    args=parser.parse_args()
    run=Path(args.run).resolve()
    if (run/'completed.json').exists():
        print('Training already completed; nothing to resume',flush=True)
        return
    configuration=json.loads((run/'configuration.json').read_text())
    snapshot=run/'source_snapshot'
    frozen=run/'resume_source'/'osp_mission';frozen.mkdir(parents=True,exist_ok=True)
    for source in snapshot.glob('*.py'):
        if source.name in ('evaluate.py','training_driver.py','package_train.py'):continue
        if source.name=='train.py' and (snapshot/'package_train.py').exists():
            source=snapshot/'package_train.py';destination=frozen/'train.py'
        else:destination=frozen/source.name
        shutil.copy2(source,destination)
    # The first snapshots predate state restoration. Their sampler algorithm
    # is unchanged; the current module adds the validated restore operation.
    upgrade='def restore(' not in (frozen/'curriculum.py').read_text()
    if upgrade:
        shutil.copy2(Path(__file__).resolve().parents[1]/'src/osp_mission/osp_mission/curriculum.py',frozen/'curriculum.py')
    driver=run/'resume_driver';driver.mkdir(exist_ok=True)
    shutil.copy2(Path(__file__).parent/'train.py',driver/'train.py')
    shutil.copy2(snapshot/'evaluate.py',driver/'evaluate.py')
    (run/'recovery_package.json').write_text(json.dumps(dict(
        source_snapshot=str(snapshot),curriculum_restore_added=upgrade,
        files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in frozen.glob('*.py')}
    ),indent=2))
    command=[sys.executable,str(driver/'train.py'),'--resume','--output',str(run),
        '--planner',configuration['planner'],'--camera-hz',str(configuration['camera_hz']),
        '--train-dir',configuration['train_dir'],'--validation-dir',configuration['validation_dir'],
        '--generations',str(configuration['generations']),'--seed',str(configuration['seed']),
        '--generation-limit',str(args.generation_limit),'--workers',str(args.workers)]
    environment=os.environ.copy();environment['PYTHONPATH']=str(frozen.parent)
    environment['OPENBLAS_NUM_THREADS']='1'
    result=subprocess.run(command,env=environment,cwd=Path(__file__).resolve().parents[1])
    raise SystemExit(result.returncode)


if __name__=='__main__':main()
