# Mixed v1.5.2.4 Four Wall Glance (simulation candidate)

This candidate preserves v1.5.2.3 Balanced Glance and its parent policies.

The reservation still gives box faces most of the schedule and periodically selects a sparse camera view of an unseen wall. When a wall slot is due, wall candidates are grouped by their world-fixed outward normal into north, east, south, and west. Up to three sections per side are evaluated. Among sides with a safe, reachable, camera-readable viewpoint, the least attempted side wins; within that side, the view covering more unseen sections with less travel wins. Sides without a safe viewpoint are deferred and can re-enter on a later map update. A dispatched viewpoint is only an **attempt**; camera coverage continues to depend on actual `viewed` evidence. Marker approach remains higher priority at mission level.

No camera or collision parameter changed. Simulation trials use 480×480 at 5 fps and 2.5/3.0 m Nav2 LiDAR marking/clearing. A successful 480 fps window on the real robot does not prove intermittent frame reception is absent, so 640 is excluded until a separate idle camera test checks processed FPS, dropped frames, image age and the robot's other ROS traffic.
