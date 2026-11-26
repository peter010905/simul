# Mixed v1.5.1 object follow-through (R3)

R3 starts from the frozen adaptive survey R2. A long move to an anonymous interior surface can reveal a compact object for the first time upon arrival. R2 may immediately send the camera toward a distant wall. R3 checks whether an unobserved side of the same object already has a safe, short observation goal.

## Selection

1. Remember the last dispatched interior surface, normal and approach goal in odometry. A named marker approach, another kind of goal, or expiration clears this opportunity.
2. Require the robot to arrive within 0.65 m of that goal and within 2.3 m of the original surface. Match the old surface to one current rectangular fit. For a previously anonymous surface, require an isolated compact occupied component too. Ambiguous fits and components that extend into a wall are rejected.
3. Consider existing eligible interior viewpoints with a distinct fitted surface normal. The existing geometry, camera-coverage filtering, Nav2 costmap connectivity, clearance and failure veto remain active. The camera must be on the viewing side and have a clear line to the face. Skip previously attempted, tagged, completed or deferred/reopened candidates.
4. Allow at most two such goals in a 30-second local window measured from the first extra dispatch. Each remaining action estimate must fit the mission time and the time left until the next wall or frontier service deadline. Persistent visual clues and named marker work retain precedence. A 90-second local cooldown prevents immediate reseeding.
5. Return to the ordinary R2 selector when these checks fail. Each choice includes the inferred host, normal, step number and remaining local time in the scheduler log.

## Development evidence

The R2 run on map 112601 selected an interior surface at 186.7 s. By 223.7 s it had reached that observation pose within 5 cm and was 1.05 m from the missed box center. The original selection had no attached box track. R2 then chose a wall goal. Its wall service wait was 50.5 s at this decision, below the 90-second overdue threshold. Across that run six adaptive interior selections and one late fallback were dispatched. None carried the local-batch flag.

The archived SLAM snapshot at 223.7 s independently fits one compact object 0.355 m from the original surface. Six generic viewpoints around it survive the replay's proxy costmap. R3 selects a distinct side with a 1.60 m grid path, 0.763 m clearance and 11.69 s predicted completion. Only 14.5 s remained before frontier service came due. On the six recorded arrivals after an adaptive interior goal, four snapshot replays produced a local continuation.

Ground-truth geometry provides an important qualification. This first alternate is on the back of marker 32, with printed-side cosine −0.99. One other candidate faces the print from about 1.02 m, but its 16.79 s predicted cost exceeds the 14.5 s deadline. Closed-loop driving must establish whether a follow-up view or subsequent exploration reads the ID. The controller has no access to marker locations or printed-side ground truth.

The original balanced run on 112601 had 25 exposure frames both physically within 1.5 m and geometrically compatible with an 18-pixel front view of marker 32. R2 had 55 physically close frames but zero of those front-view opportunities. On balanced 112605 the corresponding figures were 48 close frames and zero compatible front views. The projection ignores occlusion, pitch and roll. [Development replay and source data fields](../../experiments/2026-11-22_v151_adaptive_survey/development/object_arrival_results.json)

## Validation

Run `test_followthrough.py` with the staged R3 package on `PYTHONPATH`. Six geometry/state tests cover delayed arrival, time limits, different normals, ambiguous components, wall adjacency, uncertain sightlines, host drift, caps and cooldowns. The independent one-step replay uses saved SLAM occupancy, directional RGB coverage, pose, selected-goal history and frontier proposals. Recorded runs omit per-option live Nav2 costmaps, veto history and full fitted-face tracks. Their reconstruction substitutes a conservative clearance grid and identity map-to-odometry alignment. It cannot establish actual progress, collision rate, ID acquisition or approach success. [Reproduction script](../../experiments/2026-11-22_v151_adaptive_survey/development/object_arrival_replay.py)
