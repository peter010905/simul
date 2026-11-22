# Mixed v1.5.2.8 Wall Marginal Gain (simulation candidate)

Parent: v1.5.2.7. Earlier versions are retained for comparison. This candidate is not installed on the physical robot.

The fixed three-turns-per-wall schedule is removed. A wall section already confirmed by processed camera frames is not reinserted merely to fill a turn quota. A remaining unobserved part can still be revisited, but wall-view scoring uses the length of genuinely unobserved visible surface divided by travel cost. A wall glance gained while inspecting a box updates the same camera coverage and removes that region from subsequent wall gain. Distant weak marker hints can still trigger a closer revisit. Existing Nav2 safety, wall line-of-sight and readable-size checks remain in effect.

This addresses redundant returns to already observed mapped walls. It does not by itself guarantee discovery of a currently unmapped external wall; that requires frontier exploration to carry the robot into the missing region. Validate registered and approached markers, wall coverage, path length, bumper contacts and goal repetition before promotion.

The wall and object selectors are still separate, and a wall or frontier goal can interrupt a local box-face batch. `BoxBatch.dispatched()` also clears its active box when given a non-box goal. This version isolates camera-coverage scoring; it does not yet solve the observed wall/object backtracking. A future candidate should score the *union* of visible new box faces and wall sections from each viewpoint against one Nav2 path cost, preserve a nearby unfinished batch across incidental wall glances, and add an explicit cost for switching distant regions. A marker cue, unsafe route, or failed goal can override that commitment.
