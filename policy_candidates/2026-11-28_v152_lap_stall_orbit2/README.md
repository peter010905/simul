# v1.5.2.74 · lap stall cut + box orbit v2 (on .71)

Overlay: `wall_lap.py`, `box_orbit.py`.

1. Lap stall: after entry, a lap whose best progress does not rise by 0.03 during 45 s of lap
   dispatching ends ('perimeter_no_progress'). Physical 17:56 run: progress 0.39 from 57 s to 161 s.
2. Box orbit v2: a confirmed box never seen by the camera is served (its side facing the robot first,
   route <= 4 m); viewpoints may stand 0.55 m away and up to 50 deg off the side normal. Gazebo
   between-desk misses: robot 1.3-1.6 m away, never in front of the marker side.

Not driven on the physical robot.
