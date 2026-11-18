"""Separate task success from a finished process and freeze its elapsed time."""


def outcome_status(owner):
    ended = owner.state in ('saving', 'done')
    reason = getattr(owner, 'reason', '') if ended else ''
    elapsed = (getattr(owner, 'elapsed', 0.) if ended else
               max(0., owner.now()-owner.started) if owner.started else 0.)
    return dict(termination_reason=reason, task_success=ended and reason=='all_targets_found',
                elapsed_seconds=elapsed)
