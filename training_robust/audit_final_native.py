"""Strict final gate across one or more frozen Gazebo benchmark shards.

Development successes, duplicate layouts, source changes and partial campaigns
cannot satisfy the reserved 40-scene / 36-success requirement.
"""
import argparse
import hashlib
import json
from pathlib import Path

from audit_native_campaign import audit_episode


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(campaigns, reserved):
    errors = []
    rows = []
    planned = []
    attempted = []
    policy_digest = None
    frozen_source = None
    expected = {e['seed']: e for e in reserved['episodes']}
    if len(expected) != 40 or reserved.get('planned') != 40 or reserved.get('required_successes') != 36:
        errors.append('Reserved protocol must contain 40 distinct layouts and require 36 successes')
    if reserved.get('approach_radius_m') != 1.5:
        errors.append('Reserved protocol must require actual 1.5 m approaches')
    for campaign in campaigns:
        campaign = Path(campaign).resolve()
        try:
            manifest = json.loads((campaign/'manifest.json').read_text())
            report = json.loads((campaign/'report.json').read_text())
            hashes = json.loads((campaign/'source_hashes.json').read_text())
            actual_policy = digest(campaign/'policy.json')
        except (OSError, ValueError) as exc:
            errors.append(f'{campaign}: missing or invalid campaign metadata: {exc}')
            continue
        if manifest.get('environment') != 'gazebo_harmonic' or report.get('environment') != 'gazebo_harmonic':
            errors.append(f'{campaign}: not a native Gazebo campaign')
        if manifest.get('duration') != 600 or manifest.get('camera_width') != 960:
            errors.append(f'{campaign}: mission duration or camera configuration differs from the protocol')
        if manifest.get('approach_radius_m') != 1.5:
            errors.append(f'{campaign}: missing required 1.5 m approach protocol')
        if not report.get('record_video'):
            errors.append(f'{campaign}: actual simulation recording was not enabled')
        if actual_policy != manifest.get('policy_sha256'):
            errors.append(f'{campaign}: frozen policy changed')
        if policy_digest is None:
            policy_digest = actual_policy
            frozen_source = hashes
        if actual_policy != policy_digest or hashes != frozen_source:
            errors.append(f'{campaign}: campaigns do not share one frozen policy and source')
        snapshot = campaign/'source_snapshot'
        actual_files = {str(p.relative_to(snapshot)) for p in snapshot.rglob('*')
                        if p.is_file() and not {'__pycache__', '.pytest_cache'}.intersection(p.parts)}
        if not hashes or actual_files != set(hashes):
            errors.append(f'{campaign}: source snapshot inventory differs from its hash manifest')
        for relative, expected_digest in hashes.items():
            path = snapshot/relative
            if not path.is_file() or digest(path) != expected_digest:
                errors.append(f'{campaign}: source snapshot changed: {relative}')
        entries = {e['seed']: e for e in manifest['scenarios']}
        planned.extend(e['seed'] for e in manifest['scenarios'])
        for entry in manifest['scenarios']:
            reference = expected.get(entry['seed'])
            if reference is None or reference['sha256'] != entry['sha256']:
                errors.append(f'{campaign}: layout {entry["seed"]} is not the reserved frozen layout')
        for row in report['rows']:
            entry = row['scenario']
            seed = entry['seed']
            attempted.append(seed)
            if seed not in entries or entry['sha256'] != entries[seed]['sha256']:
                errors.append(f'{campaign}: attempted layout {seed} differs from the manifest')
            try:
                directory = campaign/'episodes'/str(seed)
                result = audit_episode(directory, entry)
                evaluation = json.loads((directory/'evaluation.json').read_text())
                if evaluation.get('environment') != 'gazebo_harmonic':
                    result['errors'].append('Episode does not identify native Gazebo execution')
                if not evaluation.get('runtime_metrics', {}).get('complete_sensor_coverage'):
                    result['errors'].append('Actual runtime sensor streams were not measured')
                result['success'] = not result['errors']
                if result['success'] != bool(evaluation.get('success')):
                    errors.append(f'{campaign}: independent audit and evaluator disagree for {seed}')
                result['campaign'] = str(campaign)
                reference = expected.get(seed)
                result['reserved_layout'] = bool(reference and reference['sha256'] == entry['sha256'])
                rows.append(result)
            except (OSError, ValueError, KeyError) as exc:
                rows.append(dict(seed=seed, campaign=str(campaign), success=False,
                                 errors=[f'Incomplete episode evidence: {exc}']))
    if len(planned) != 40 or len(set(planned)) != 40 or set(planned) != set(expected):
        errors.append('Planned layouts are not exactly the 40 distinct reserved layouts')
    if len(attempted) != 40 or len(set(attempted)) != 40 or set(attempted) != set(expected):
        errors.append('Attempted layouts are not exactly the 40 distinct reserved layouts')
    successes = sum(row['success'] and row.get('reserved_layout', False) for row in rows)
    complete_layout_set = (len(attempted) == 40 and len(set(attempted)) == 40
                          and set(attempted) == set(expected))
    if successes < 36:
        errors.append(f'Only {successes} independently audited successes; at least 36 are required')
    return dict(achieved=not errors, required_successes=36, planned_layouts=len(planned),
                attempted_layouts=len(attempted), audited_reserved_successes=successes,
                success_rate_over_reserved_40=successes/40. if complete_layout_set else None,
                errors=errors, rows=rows,
                policy_sha256=policy_digest, frozen_source_files=len(frozen_source or {}),
                note='This gate checks reserved identities, artifacts and shared frozen sources. '
                     'The experiment record must additionally show these layouts were not used for tuning.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('campaigns', nargs='+', type=Path)
    parser.add_argument('--reserved-manifest', type=Path, default=Path(__file__).parent/'native90/reserved_holdout_manifest.json')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = audit(args.campaigns, json.loads(args.reserved_manifest.read_text()))
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'rows'}, indent=2))
    raise SystemExit(0 if result['achieved'] else 2)


if __name__ == '__main__':
    main()
