# Mixed v1.5.2.10 Distant Wall Gain (simulation candidate)

Parent: v1.5.2.9 Local Face Commit. All prior candidates remain available; no physical robot policy is changed.

The same-fixture v1.5.2.8 and v1.5.2.9 runs each registered and verified only two obstacle markers in 600 seconds. Neither approached the mapped, unobserved west-wall marker within 5.5 m. In the last saved map, the target wall surface was present, `viewed` was false for its sampled cells, and a geometric free-space replay found 50 candidate viewpoints within 1 m of that wall section with map reachability and line of sight. This does **not** prove Nav2 would execute a path to any of those points.

The earlier selector evaluated only the two ends and one preferred sample per compass direction, then divided wall gain by travel cost. This can exclude a far middle section of a long wall entirely. This candidate retains one goal per physical wall section, considers up to 64 sections, and raises the value of mapped wall areas whose camera-valid viewpoint is far from the robot's recent trail. It still uses camera-confirmed unseen cells, the readable-size and line-of-sight checks, Nav2 costmap connectivity, and the existing obstacle-face inspection rules. No map location or marker truth is hardcoded into the policy.

Inspect actual Nav2 goal outcomes and the full 600-second exam result before using this version. The offline map replay only establishes that the missing wall is mapped and geometrically reachable; it is not a substitute for a Gazebo run.
