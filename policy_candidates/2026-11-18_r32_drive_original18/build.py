"""Build a separate R32 drive overlay using the authorized local reference checkout."""
import argparse
import os
from pathlib import Path
import runpy

if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--reference-core', type=Path, required=True)
    cli.add_argument('--workspace', type=Path, required=True)
    cli.add_argument('--test', action='store_true')
    args = cli.parse_args()
    here = Path(__file__).resolve().parent
    os.environ['OSP_R32_CORE'] = str(args.reference_core.resolve())
    parent = here.parent / '2026-11-17_face_intent_flow_v3'
    flow = here.parent / '2026-11-18_face_intent_flow_v4'
    runpy.run_path(str(parent/'build.py'))['build'](args.workspace.resolve(), args.test, [flow, here])
