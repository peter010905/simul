# Mixed v1.5.2.6 Twelve Wall Views (simulation candidate)

Parent: v1.5.2.5 Four Wall Sections. Every preceding candidate is preserved.

This candidate schedules up to three **spatially separated** camera turns per north/east/south/west wall, for 12 planned turns, interleaved with box-face work. The next wall turn becomes eligible after 35 seconds while box work remains or 22 seconds otherwise. A 36-degree scan is requested at each selected viewpoint. New positions must be at least 1.25 m apart along that wall; the long-wall sampler considers both endpoints and a high-gain local section. Existing camera coverage evidence and marker candidate/ID pipelines remain intact. Already mapped walls remain eligible until their three-turn quota is met. Nav2, reachability, line-of-sight, collision and time-budget checks are unchanged. Inaccessible directions are deferred; finding and approaching all three markers ends the mission even if the turn quota has not been used.

The three-turn counters record **dispatched distinct viewpoints**, not proof that every camera frame arrived or that every Nav2 goal succeeded. The native run log and camera frames must be audited before claiming a wall was actually inspected three times. Camera stays 480×480 at 5 fps; 640 is excluded from this policy test because real reception has not been established across varied runs.

## Same-fixture result (2026-11-22)

Archived random exam seed 417608489, fixed desks, random extra obstacles and three markers, 600 simulated seconds: **2/3 markers registered and verified**, 111.30 m travel, zero bumper contacts. The west-wall marker (ID 768, x=-10.0 m) was never approached closer than 5.53 m. Eleven sparse wall-view goals were selected, but their target surfaces ranged from x=-3.30 to x=11.85 m; none directed the robot to the west-wall marker's section. This is a failed candidate for the stated wall-coverage goal. Baseline v1.5.2.2 Wall Spread on the same fixture found and approached 3/3 by 282.79 s with 39.02 m travel and zero bumper contacts.

The failure shows that a quota of camera turns grouped by compass direction does not ensure coverage of the actual room perimeter. A mapped surface classified as `wall` can consume quota while a distant wall section remains unmapped. Keep this candidate for comparison; do not promote it to the physical robot.
