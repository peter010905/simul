# v1.5.2.70 · box orbit (every side of a box)

Parent: v1.5.2.69 thin obstacles (`stage-v15269-thin-obstacles`). Changes only `center_wall.py`
(hook) and adds `box_orbit.py`.

## Why
Gazebo, scene16-room-slam1128, 11 runs of .69 (evaluator geometry, ground truth only offline):
56/131 viewable box faces seen, every face of a box in 4/33 boxes. Two of the three failed runs
(3805, 3811) had a marker side that was never a target. Face tasks exist only for mapped
surfaces (four-face fits only within 2.7 m), one box gets 55 s, then a 90 s cooldown.

## Rule
Each confirmed box keeps the bearings from which the camera actually looked at it (0.45-2.0 m,
heading within 28 deg, line of sight clear). While the largest gap between those bearings exceeds
100 deg, a viewpoint on the far side of the gap is dispatched, mapped side or not. The served box
first; at most two attempts per 45 deg sector and 120 s per box; a box with a registered marker is
done; a sector with no safe viewpoint counts as covered.

## Install
    python3 install.py <.69 workspace> <new workspace> [--copy-staged-parent]

Not driven on the physical robot. Inherits the .69 thin-obstacle memory, which made the robot spin
in place in the 2026-11-28 physical run (local costmap filled with remembered small blobs).
