"""Build the Flow v3 surface-inspection and route-history candidate."""
import argparse
import runpy
from pathlib import Path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--test', action='store_true')
    args = parser.parse_args()
    candidate = Path(__file__).resolve().parent
    parent = candidate.parent / '2026-11-17_face_intent_flow_v3' / 'build.py'
    runpy.run_path(str(parent))['build'](args.workspace.resolve(), args.test, extra_stages=[candidate.parent / '2026-11-18_flow_v3_mppi', candidate.parent / '2026-11-18_flow_v3_contact', candidate])
