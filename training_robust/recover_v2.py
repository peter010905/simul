"""Resume a v2 checkpoint without importing edited workspace simulation code."""

import argparse
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run',required=True)
    parser.add_argument('--generation-limit',type=int,default=2);parser.add_argument('--workers',type=int,default=6)
    args=parser.parse_args();run=Path(args.run).resolve()
    if (run/'completed.json').exists():
        print('Already completed',flush=True);return
    env=dict(os.environ,PYTHONPATH=str(run/'source_snapshot'),OPENBLAS_NUM_THREADS='1')
    result=subprocess.run([sys.executable,str(run/'source_snapshot/driver.py'),'--resume','--output',str(run),
        '--generation-limit',str(args.generation_limit),'--workers',str(args.workers)],env=env,
        cwd=Path(__file__).resolve().parents[1])
    raise SystemExit(result.returncode)


if __name__=='__main__':main()
