# v1.5.2.72.1 · one lap direction from the contour orientation (on .71)

.72 fixed the sense from the entry tangent; in Gazebo 3801/3804 it disagreed with the contour
and the lap ended at 81/97 s. .72.1 takes the sense from the signed area of the measured contour.

Overlay: `wall_lap.py` only.

## Evidence (four physical .67 runs, 2026-11-28)
Around the measured room centre the lap goals changed direction mid-lap, for example
17:56 +23,+20 then -5,-28,-31,-12,-16 deg; 18:09 +22,+35 then -2..-21 deg. Robot paths covered
56-173 deg in 150-218 s, net -56, +7, +10, -41 deg. In Gazebo (.71, 11 runs) net 115-306 deg, rare
reversals. The lap "forward" comes from the floor contour, which zig-zags through the desk/chair-leg
band along the physical walls, so it flips; the stop-normal turn also jumped (progress 1.01 after
half a lap in the 18:21 run).

## Rule
The lap direction (clockwise or counter-clockwise around the centre of the measured floor) is fixed
at the first continuation after entry. Every later lap stop must advance 3-75 deg around that centre
from the robot. Progress = max(travelled share of the perimeter, credited angle / 360 deg).

Not driven on the physical robot.
