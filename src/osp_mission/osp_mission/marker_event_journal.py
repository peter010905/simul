"""Append camera-confirmed identity and estimated approach transitions to disk."""
import json


class MarkerEventJournal:
    def __init__(self, path):
        self.path = path
        self.confirmed = set()
        self.approached = set()
        self.state = None

    def write(self, event, status, ident=None):
        row = dict(event=event, elapsed_seconds=status.get('elapsed_seconds', 0.),
                   state=status.get('state'), confirmed_ids=sorted(self.confirmed),
                   approached_ids=sorted(self.approached))
        if ident is not None:
            row['id'] = ident
        with self.path.open('a') as stream:
            stream.write(json.dumps(row) + '\n')

    def detector(self, detector, status):
        current = set(int(ident) for ident in detector.get('resolved_ids', []))
        previous = self.confirmed
        self.confirmed = current
        for ident in sorted(current - previous):
            self.write('id_confirmed', status, ident)
        for ident in sorted(previous - current):
            self.write('id_retracted', status, ident)

    def mission(self, status):
        current = set(int(ident) for ident in status.get('approached_ids', []))
        previous = self.approached
        self.approached = current
        for ident in sorted(current - previous):
            self.write('approach_estimated_within_1_5m', status, ident)
        for ident in sorted(previous - current):
            self.write('approach_retracted', status, ident)
        if status.get('state') != self.state:
            self.state = status.get('state')
            self.write('mission_state', status)
