"""Run an isolated TF configuration regression after the perception comparison."""
import argparse
import json
from pathlib import Path
import signal
import sys
import time

from native_tournament import service_active
from run_small_marker_regression import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign', type=Path, required=True)
    parser.add_argument('--perception-regression', type=Path, required=True)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(143))
    predecessor = 'osp-small-marker-regression-v2.service'
    while service_active(predecessor):
        time.sleep(5.)
    previous = json.loads((args.perception_regression/'regression_state.json').read_text())
    if previous['status'] != 'completed_regression_requires_fresh_map_validation':
        raise RuntimeError('Perception comparison did not finish. Inspect it before launching another regression.')
    # Wait for confirmed live CEM jobs, so stopping the coordinator cannot race
    # its initial systemd dispatch. Those native jobs finish normally as well.
    while True:
        state = json.loads((args.campaign/'state.json').read_text())
        if state['status'] != 'running':
            raise RuntimeError(f'CEM needs attention: {state["status"]}')
        active = [job for job in state['jobs'] if job['status'] == 'running'
                  and service_active(job['unit'])]
        if len(active) == 2:
            break
        time.sleep(5.)
    run(args.campaign.resolve(), args.workspace.resolve(), args.output.resolve())


if __name__ == '__main__':
    main()
