# Mixed v1.5.2.7 Twelve Wall Coverage (simulation candidate)

Parent: v1.5.2.6 Twelve Wall Views. Older candidates remain available.

Three spatially separated looks per north/east/south/west wall are requested. A box-facing turn also counts toward a wall's three zones if the live EdgeCoverage pipeline confirms that wall region from processed camera frames, map-aligned LiDAR endpoints, readable pixel size, incidence and line of sight. One planning interval contributes at most one zone per wall direction. Weak 6–8 px print-like cues remain revisit hints rather than proof that the wall is fully inspected. Dispatched wall-view goals are recorded separately as attempts and do not count as camera-confirmed looks. The scheduler and existing Nav2 safety constraints are otherwise unchanged.

Camera is still 480×480 at 5 fps; no physical robot camera or drive configuration changed.

The camera-confirmed counting logic passed two local tests and the same-fixture runtime precheck. Its full simulation was stopped at 47 simulated seconds after the parent twelve-turn scheduler failed the same fixture and the user rejected fixed turn quotas. No detection result is claimed for this unfinished run. This version remains a separate prototype of the counting logic, not a recommended navigation policy.
