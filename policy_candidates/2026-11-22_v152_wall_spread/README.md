# Mixed v1.5.2.2 Wall Spread (simulation candidate)

This is a separate candidate based on v1.5.2.1 Focus Continuity. The parent is preserved.

The archived west-wall miss placed the marker 7.42 m from the closest robot pose. Ten scheduled wall views were dispatched, but none approached that side of the room. The running `edge` planner uses `features[1]` for unknown-space gain and `features[0]` for camera-visible surface gain.

This candidate periodically gives a reachable, not recently visited unknown region a chance even if local object faces remain. It does not change the camera detector, marker confirmation, robot footprint, costmap clearance, Nav2 collision checks, or urgent marker approach priority. Goals still pass the existing connected free-space and Nav2 costmap checks; the actual Nav2 result is recorded by the simulator.

When evaluating wall goals, it samples at least one candidate per wall-facing direction before filling the eight-goal shortlist. Dense samples on one nearby wall can no longer consume the entire shortlist.

The region task has a 100-second dispatch cooldown, begins after 120 seconds, and requires at least 55 seconds of budget. It selects a previously unvisited safe target with unknown-space gain or camera-unseen surfaces. This also works after the LiDAR map frontier has vanished. If no such target exists it leaves the normal policy unchanged.

The candidate is experimental until matched Gazebo runs show its effect on wall marker registration, 1.5 m approach, time, and bumper events. A wider LiDAR scan can map more of the room but does not itself prove a wall was camera-inspected.
