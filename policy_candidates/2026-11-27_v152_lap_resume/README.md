# v1.5.2.64 lap resume (staged candidate, not production)

Parent: v1.5.2.63 unswept sides. One file: `wall_lap.py`.

## What .63 showed (5 seeds where .60 and .62 both failed)
- .63 found all three prints in 3/5 runs (3502, 3517, 3615). 0 bumps.
- 3507 missed wall print 714. At 100 s the lap was heading to the stop 1.9 m in front of 714.
  At 105 s a different print (304) was seen, and its follow-up and approach took over.
  At 116 s the lap resumed, but it chose the next section ahead and skipped the unreached stop.
  .60 failed the same way on the same seed.
- 3613 missed box print 188. Its face was never a candidate; .64 does not address that.

## Rule
A lap stop that was interrupted before arrival is resumed once, when the lap continues.
It is dropped if the interruption is older than 90 s, the stop is farther than 4 m from the robot,
or the stop is vetoed. A stop the robot reached (within 0.55 m) is never resumed.
Count: `wall_lap.resumed_stops`; reason `resume_interrupted_lap_stop`.

## Test
`python3 test_lap_resume.py`

## Install
`python3 install.py <.63 workspace> <new workspace>` then `colcon build`.
Physical drive: none. Board default unchanged.
