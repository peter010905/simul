# v1.5.2.71 · far marker follow + box orbit

Parent: v1.5.2.67 wall jogs (the policy run on the physical robot on 2026-11-28).
Overlay: `bearing_advance.py` (far prints), `box_orbit.py` + `center_wall.py` hook (from .70).

## Evidence (five physical .67 runs, 2026-11-28 17:56-19:06)
- Every run ended on the 600 s timeout with 2, 2, 1, 0, 1 of 3 prints.
- The missed print was decoded but never registered: ID 7 decoded 12-20 times at 10-12 px
  (`small`/`no_range`), about 4-6 m away. Distant hints: 0 (off on the board, and the
  triangulated hint needs five views over a 15 deg baseline).
- The bearing look moved at most 1.4 m along the ray (route <= 2.4 m, line of sight through mapped
  cells only). In the 19:06 run it went once toward ID 7 and the print stayed too small.
- Box faces: camera diagnostics show 0-3 observed box faces per run.

## Changes
1. Bearing look: endpoint up to 3.0 m along the ray, route up to 4 m, unknown cells do not block the
   ray (measured obstacles do), three looks per ID, six per mission. Prints under 14 px prefer the far end.
2. Box orbit (.70): unseen sides of a tracked box get a viewpoint, mapped or not.

Not driven on the physical robot. No thin-obstacle memory (.69), so no spin-in-place regression.
