"""Balanced layout batches without replacement inside each difficulty stratum."""

import numpy as np

from osp_mission.robust_scenarios import COUNTS, PROFILES


class LayoutCurriculum:
    """Visit every layout in a stratum before reshuffling that stratum."""

    def __init__(self, scenarios, seed):
        seeds = [int(c['seed']) for c in scenarios]
        if len(seeds) != len(set(seeds)):
            raise ValueError('Training scenario seeds must be unique')
        self.rng = np.random.default_rng(seed)
        self.groups = []
        for mount in PROFILES:
            for count in COUNTS:
                group = sorted(
                    [c for c in scenarios
                     if c['mount_profile'] == mount and c['difficulty'] == count],
                    key=lambda c: c['seed'])
                if not group:
                    raise ValueError(f'Missing training stratum: {mount}/{count}')
                self.groups.append(group)
        if sum(map(len, self.groups)) != len(scenarios):
            raise ValueError('Unsupported training stratum')
        self.queues = [[] for _ in self.groups]
        self.cycles = [0 for _ in self.groups]

    def next_batch(self):
        batch = []
        for i, group in enumerate(self.groups):
            if not self.queues[i]:
                self.queues[i] = self.rng.permutation(len(group)).tolist()
                self.cycles[i] += 1
            batch.append(group[self.queues[i].pop()])
        return batch

    def state(self):
        """JSON-serializable schedule evidence for the training checkpoint."""
        return dict(
            rng_state=self.rng.bit_generator.state,
            remaining_seeds=[[self.groups[i][j]['seed'] for j in queue]
                             for i, queue in enumerate(self.queues)],
            cycles=self.cycles.copy())

    def restore(self, state):
        """Restore the exact remaining layout order after a process interruption."""
        queues=state['remaining_seeds'];cycles=state['cycles']
        if len(queues)!=len(self.groups) or len(cycles)!=len(self.groups):
            raise ValueError('Checkpoint strata differ from training scenarios')
        restored=[]
        for group,seeds in zip(self.groups,queues):
            indices={c['seed']:i for i,c in enumerate(group)}
            if len(seeds)!=len(set(seeds)) or any(seed not in indices for seed in seeds):
                raise ValueError('Invalid remaining layout checkpoint')
            restored.append([indices[seed] for seed in seeds])
        if any(not isinstance(c,int) or c<0 for c in cycles):
            raise ValueError('Invalid curriculum cycle checkpoint')
        self.rng.bit_generator.state=state['rng_state']
        self.queues=restored;self.cycles=cycles.copy()


def validate_split_separation(training, validation):
    """Reject accidental reuse of a held-out layout before optimization starts."""
    import hashlib
    import json

    def geometry(c):
        # A new seed or new targets must not hide identical obstacle geometry.
        objects = sorted(json.dumps(o, sort_keys=True) for o in c['obstacles'])
        return hashlib.sha256(json.dumps(objects).encode()).hexdigest()

    if {c['seed'] for c in training} & {c['seed'] for c in validation}:
        raise ValueError('Training and validation seeds overlap')
    if {geometry(c) for c in training} & {geometry(c) for c in validation}:
        raise ValueError('Training and validation obstacle geometry overlaps')
