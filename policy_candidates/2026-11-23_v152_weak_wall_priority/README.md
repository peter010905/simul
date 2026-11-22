# Mixed v1.5.2.11 Weak Wall Priority (simulation candidate)

Parent: v1.5.2.10 Distant Wall Gain. Previous candidates remain available. No physical robot policy or camera setting is changed.

The same archived exam fixture exposed a specific regression. The successful v1.5.2.2 Wall Spread run selected `persistent_small_wall_candidate` at 250 seconds, reached the west-wall marker and verified 3/3 by 282.79 seconds. Later candidates selected generic box/wall goals before calling `WallBearings.choose`, so a strong distant print cue could be starved. This candidate checks the existing multi-frame, map-matched 6–10 px bearing before a local box continuation, frontier or generic wall goal. `WallBearings.choose` already requires at least three temporally separated samples and either repeated 8 px evidence or two independent views; it returns only feasible wall viewpoints and limits attempts. Pending obstacle faces stay pending, with the same camera-confirmation and Nav2 safety rules.

This restores the cue's scheduling opportunity. A cue being selected is not proof that Nav2 reached it or that ArUco registered the marker.

## Archived Gazebo result (2026-11-23)

In the frozen random exam fixture `map-417608489`, with the same spawn, three markers, and a 600 s limit, this candidate registered and approached all 3/3 markers within 1.5 m by 271.496 s. It traveled 39.42 m and recorded 0 bumps. The previously missed west-wall marker ID 768 was registered at 254.98 s and verified at 270.996 s. The decision log selected a `persistent_small_wall_candidate` at 233 s. The immediate parent v1.5.2.10 found only 2/3 by 600 s, traveled 92.44 m, and had 0 bumps. This is one fixed layout, not a general success claim; other random layouts are being tested separately.

Archived result: `/home/ubuntu/.local/share/simul-team/results/team-v152_weak_wall_priority_1123_ws-track-m417608489-1795361605204141997` (VM).

Two additional frozen, independently randomized exam fixtures also completed all 3/3 within 1.5 m with no bumps: `map-550866253` at 243.816 s and 36.96 m (north wall plus two boxes), and `map-1398584497` at 396.136 s and 62.91 m (south recess wall plus two boxes). Their spawn positions and headings differ. All three runs processed 480×480 images at 5 fps in Gazebo; this does not establish real robot USB or DDS capacity. The third run first passed the last box from its unprinted side, then viewed its printed side on return. Physical proximity alone would have wrongly counted that face as inspected.

Two additional untuned stress fixtures also completed 3/3 with zero bumps: seed 865942137 at 389.492 s included elbow-shaped obstacles and three wall/column markers; seed 246813579 at 137.436 s had no extra obstacles and three wall markers. They are different conditions from the three box-only comparison fixtures and are reported separately in `experiments/2026-11-23_weak_wall_priority/README.md`.

These five fixtures are a limited validation set. The code has not been installed for physical driving; the previous working policies remain intact.
