# v1.5.2.62 same view (staged candidate, not production)

Parent: v1.5.2.61 swept face (`swept_face_manifest.json` version `1.5.2.61-swept-face`).

## What .61 showed (demo-room twin, seeds 3501-3516)
.61 removed every 30 s stall, and its median finish time was the fastest (313 s). But it found
all three prints in only 10/16 runs, against 13/15 for .60. In 3509 and 3511 the missed box print
had been swept once from a poor standoff. .61 then held that face for 145 s. .60 later found the
same prints by standing about 1 m in front of the face.

## Rule
Hold a swept face only against a repeat from the same standoff (within 0.35 m). A different
standoff for the same face stays available. One file: `box_continuity.py`.

## Install
`python3 install.py <.61 workspace> <new workspace>` then `colcon build`.
Physical drive: none. Board default unchanged.
