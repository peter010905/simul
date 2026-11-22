# Mixed v1.5.2.3 Balanced Glance (simulation candidate)

Parent: Mixed v1.5.2.2 Wall Spread. The parent code and real robot profile are unchanged.

The new scheduler grants a sparse wall observation when a wall section remains camera-unseen for at least 75 seconds without pending box work, or 115 seconds while box faces remain. It evaluates viewpoints that cover multiple unobserved wall sections, penalizes very close wall stops, and retains the original camera pixel, angle, line-of-sight, free-space, costmap, and time-budget checks. Existing marker approach and box-face batching retain priority outside this bounded wall reservation. Missed weak wall cues and distant ID hints remain in their original revisit pipeline.

Camera and LiDAR profiles are unchanged: simulation trials use 480×480 at 5 fps and Nav2's 2.5 m marking / 3.0 m clearing. The candidate does not claim that a wall is observed simply because LiDAR mapped it. Gazebo comparisons and real OAK-D frame-reception checks are required before promotion.
