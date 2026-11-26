# v1.5.2.61 swept face (staged candidate, not production)

Parent: v1.5.2.60 box angle (`box_angle_manifest.json` version `1.5.2.60-box-angle`).

## Problem seen in the demo-room twin (seeds 3501-3506, no floor debris)
11 of 15 stall windows (>=30 s with <0.15 m of motion) were the same loop: at one box the
"finish current box" tour re-dispatched a face the robot had just swept from its standoff,
because the face stayed unobserved. It repeated every ~4 s until the 55 s box visit expired.

## Rule
Once a face is swept from its standoff, the same-box tour does not choose that face again for
`box_visit_s + box_return_cooldown_s` (145 s). Interrupted faces (marker hint etc.) still resume.
One file: `box_continuity.py`.

## Install
`python3 install.py <.60 workspace> <new workspace>` then `colcon build`.
Physical drive: none. Board default unchanged.
