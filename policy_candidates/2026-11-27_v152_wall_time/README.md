# v1.5.2.65 wall time (staged candidate, not production)

Parent: v1.5.2.64 lap resume. Files: `phased_hybrid.py`, `wall_lap.py`.

## Why (physical run 2026-11-27 21:11 KST, .62)
- At 150 s the wall lap hit its time budget about three quarters of the way round:
  261 deg turned, 24.7 m, 11 stops. The rest of the wall was never lapped.
- From 152 s, white-leg cues were dispatched repeatedly (decisions at about 152, 266, 290, 478 and
  494 s).
- An earlier physical record (.50 comment) read no marker from 19 white-leg looks in 600 s.
- Gazebo: 3 of 7 missed wall prints were on a section the lap never reached before 150 s.

## Rules
1. White-leg cues are off. `white_filter` returns False, so no white patch becomes a goal.
   The thin-obstacle skip for box faces is unchanged.
2. The wall lap may continue past its budget, up to 210 s, while two things hold:
   - it has covered at least 60% of a turn or of the measured perimeter;
   - a lap stop was credited in the last 25 s.

   A lap that stops advancing ends as before. The `duration - 240 s` cap still applies.
   Report fields: `wall_lap.extended`, `wall_lap.progress`.

## Test
`python3 test_wall_time.py`

## Install
`python3 install.py <.64 workspace> <new workspace>` then `colcon build`.
Physical drive: none. Board default unchanged.
