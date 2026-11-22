# Mixed v1.5.2.9 Local Face Commit (simulation candidate)

Parent: v1.5.2.8 Wall Marginal Gain. Earlier versions remain available. This candidate is not installed on the physical robot.

The box's camera-confirmed face state and Nav2 safety checks are unchanged. A box face is never considered complete simply because the robot drove through its area. Both fitted rectangles and ordinary mapped occupied islands can own a local face batch. If a new, safely reachable face of the active nearby object remains, it is selected before a wall or frontier trip. An incidental wall glance keeps the active box batch; leaving to a different region explicitly ends it. Thus the robot can finish nearby box faces and credit any wall surfaces seen along the way, then leave for a genuinely unseen region. A face with no safe viewpoint is still deferred by the existing view generator and failure history.

This is a local route-continuity fix, not a proven global optimum. It must be compared against v1.5.2.8 on identical exam fixtures for marker detection, within-1.5m approaches, wall discovery, distance, bumper contacts and large region switches.
