# v1.5.2.63 unswept sides (staged candidate, not production)

Parent: v1.5.2.62 same view. Stage `stage-v15263-unswept-sides` on V100 was made by copying the
.62 stage and dropping in the two overlay files. The parent hashes in `parent_sources.json` were
checked before the copy.

## What .60/.62 showed (5 seeds both failed: 3502, 3507, 3517, 3613, 3615)
- All 10 runs timed out (600 s).
- The missed marker face had open floor in front.
- In 7 of the 8 runs with good logs, the face was a PENDING candidate (a safe viewpoint existed),
  but it never won the ranking. Meanwhile other faces of the same box were swept again and again.
  3517-.60 swept the opposite face 12 times; 3613-.60 swept the other faces 31 times.
- Each run also spent 57-322 s on face goals that were not target boxes.
  Measured on the logged SLAM snapshots, 167 of about 1,300 such dispatches sit within 0.20 m of
  a wall-sized occupied structure (2 m or longer; no box is).
  On those snapshots, no dispatch on a target box between or beside desks, or in the open, fell
  within that 0.20 m. 2 of 17 dispatches on boxes standing against a wall did.

## Rules
1. **Finish a box** (`box_continuity.py`). Every swept box side is remembered: tracked box
   centre plus face direction, from any standoff. While the same box still offers a face
   direction that was never swept, its swept directions are dropped from the ranking.
   Inside the box visit, an unswept side may be retried after one failed try.
2. **Wall surface is not a box face** (`center_wall.py`). A generic interior sample within
   0.20 m of a wall-sized occupied structure is skipped. Faces from the box fitter are exempt,
   so a target box against a wall stays a task. Count: `counts.on_wall_skipped`.

## Test
`python3 test_unswept_sides.py` (run in the sim image against the stage source).

## Install
`python3 install.py <.62 workspace> <new workspace>` then `colcon build`.
Physical drive: none. Board default unchanged.
