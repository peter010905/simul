# v1.5.2.61 swept face (staged candidate, not production)

Parent: v1.5.2.60 box angle (`box_angle_manifest.json` version `1.5.2.60-box-angle`).

## Problem seen in the demo-room twin (seeds 3501-3506 and follower seeds 3601-3608, .57-.60)
Most stall windows (>=30 s with <0.15 m of motion; 11/15 demo, 8/8 follower) were one loop:
the robot re-dispatched a face it had just swept from its standoff, because the face stayed
unobserved. "finish current box" repeated it every ~4 s until the 55 s box visit expired, and
then "regional camera gain" could pick the same face again (3607: 33 s more).

## Rule
Once a face is swept from its standoff, it leaves the shared interior candidate list for
`box_visit_s + box_return_cooldown_s` (145 s). That list feeds the box tour, regional gain and
interior priority alike. Interrupted faces (marker hint etc.) are not swept, so they still resume.
Files: `box_continuity.py` (memory), `center_wall.py` (one filter on the candidate list).

## Install
`python3 install.py <.60 workspace> <new workspace>` then `colcon build`.
Physical drive: none. Board default unchanged.
