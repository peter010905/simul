"""Measure substantial consecutive turns in opposite directions from the actual base trajectory."""
import argparse
import csv
import math
from pathlib import Path


def reversals(headings):
    """Reject slow drift, tiny corrections, and pairs separated by a pause or recording gap."""
    reversals_at = []
    prev = None
    window = None
    angle = 0.
    direction = 0
    first = last = 0.
    previous_turn = None

    def finish():
        nonlocal previous_turn
        if abs(angle) >= math.radians(15):
            if previous_turn and previous_turn[0] != direction and first - previous_turn[1] <= 1.5:
                reversals_at.append(first)
            previous_turn = direction, last

    for stamp, yaw in headings:
        if prev is None or not 0 < stamp - prev[0] <= .55:
            if prev is not None:
                finish()
            angle, direction, previous_turn = 0., 0, None
            prev = window = (stamp, yaw)
            continue
        prev = (stamp, yaw)
        elapsed = stamp - window[0]
        if elapsed < .25:
            continue
        delta = math.remainder(yaw - window[1], math.tau)
        sign = (1 if delta > 0 else -1) if abs(delta) / elapsed >= .15 else 0
        if sign != direction:
            finish()
            first, angle, direction = window[0], 0., sign
        if sign:
            angle += delta
        last = stamp
        window = (stamp, yaw)
    finish()
    return reversals_at


def measured(run):
    with (run / 'headings.csv').open() as stream:
        return reversals((float(row['elapsed_sim_s']), float(row['yaw_rad'])) for row in csv.DictReader(stream))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    args = parser.parse_args()
    for manifest in sorted(args.archive.glob('run*-*/complete.json')):
        turns = measured(manifest.parent)
        print(manifest.parent.name, len(turns), ','.join(f'{t:.1f}' for t in turns), sep='\t')
