"""Fresh-space feedback for camera-gain candidate if the approach-only trial fails.

Do not launch alongside the gated seven-shape dispatcher: its terminal decision
must be observed first. The shared one-slot owner preserves running CEM jobs.
"""
import argparse
import json
from pathlib import Path
import time

from continual_native_curriculum import digest, save
from run_fresh_variant_probe import prepare as prepare_case
from run_spatial_calibrated_screen import qualified_regression
from run_strict_approach_regression import run as run_profiled_case

UNIT = 'osp-calibrated-triangle-probe-v1'


def prepare(regression, workspace, output):
    if '253 passed' not in (workspace.parent/'build_and_tests.log').read_text():
        raise ValueError('Require the tested combined camera/approach runtime')
    prior = json.loads((regression/'state.json').read_text())['jobs'][0]
    prepare_case(argparse.Namespace(output=output, workspace=workspace,
        policy=Path(prior['output'])/'policy.json', seed=1171400101,
        obstacles=16, shape='triangle', domain=121, unit=UNIT))
    state = json.loads((output/'state.json').read_text())
    state.update(status='running', phase='waiting_approach_only_regression',
        engineering_screen=True, training_samples=True, prerequisite=str(regression),
        driver_sha256=digest(Path(__file__)))
    state['jobs'][0]['generation'] = 0
    save(output/'state.json', state)


def run(parent, regression, screen, workspace, output):
    if not (output/'state.json').exists(): prepare(regression, workspace, output)
    path = output/'state.json'; state = json.loads(path.read_text())
    if state['status'] != 'running': return
    terminal_seen = None
    while True:
        if digest(Path(__file__)) != state['driver_sha256']: raise RuntimeError('Frozen probe driver changed')
        prior = json.loads((regression/'state.json').read_text()); job = prior['jobs'][0]
        if job['status'] == 'done' and prior['status'].startswith('completed_'):
            if terminal_seen is None: terminal_seen = time.monotonic()
            report = Path(job['output'])/'observer_analysis.json'
            decision = json.loads((screen/'state.json').read_text())
            if report.exists():
                analysis = json.loads(report.read_text())
                if qualified_regression(prior, analysis):
                    state.update(status='not_needed_seven_shape_screen_proceeds',
                        prerequisite_outcome=job['outcome']); save(path, state); return
                if decision['status'] == 'not_started_prerequisite_failed': break
            if time.monotonic()-terminal_seen > 150:
                state.update(status='prerequisite_evidence_requires_attention',
                    original_native_evidence_preserved=True); save(path, state); return
        time.sleep(5)
    state.update(phase='fresh_triangle_camera_gain_engineering_probe',
        prerequisite_outcome=job['outcome'], seven_shape_screen_terminal=True)
    save(path, state)
    # Reuse the proven single-case reservation and read-only velocity-profile
    # observer on this already-prepared fresh scene. It checks source hashes,
    # native sensor/profile gates and keeps the original native score intact.
    run_profiled_case(parent, regression, workspace, output)
    state = json.loads(path.read_text()); state['status'] = 'completed_calibrated_triangle_probe'
    save(path, state)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('parent', 'regression', 'screen', 'workspace', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    run(args.parent.resolve(), args.regression.resolve(), args.screen.resolve(), args.workspace.resolve(), args.output.resolve())
