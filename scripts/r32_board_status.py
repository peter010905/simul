"""Translate upstream osp_project status messages for the shared session and board."""


def normalize(status):
    phase = status.get('state')
    state = {'READY': 'idle', 'FINISHING': 'saving', 'FINISHED': 'done'}.get(phase, 'running' if phase else '')
    confirmed = [target for target in status.get('targets', []) if target.get('confirmed')]
    approached = sorted({target['id'] for target in confirmed if target.get('approached')})
    reason = status.get('reason', '')
    return dict(state=state, runtime='upstream-r32', run_id=status.get('run_id'),
                active_action='nav' if phase == 'NAVIGATE' else phase,
                elapsed_seconds=status.get('elapsed'), readiness_reason=status.get('readiness'),
                termination_reason={'targets_complete': 'all_targets_found', 'time_limit': 'timeout'}.get(reason, reason)
                if state in ('saving', 'done') else '',
                task_success=bool(status.get('stop_confirmed') and reason == 'targets_complete' and len(approached) >= 3),
                approached_ids=approached,
                confirmed_targets=[(target['id'], target['x'], target['y']) for target in confirmed])
