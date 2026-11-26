# Mixed v1.5.2.32 · Glass/Chair Exclusion

This is an opt-in fork of 1.5.2.31-start-sweep; the parent workspace is not edited.
The runtime workspace is /home/ubuntu/v15232_glass_wall_640_1124_ws.

## Operator workflow

On the physical board at http://192.168.252.2:8767/, select
**Mixed v1.5.2.32 · Glass Wall + Marker Cooldown**, press **주행 준비**,
and wait for a fresh SLAM map. Before pressing **출발**, select
**탐색 제외 구역 지정** and click the four corners of each chair/glass strip,
in boundary order. The two strips are independent annotations. Check that
the yellow overlay covers only the known marker-free strip, then start.
**제외 지정 지우기** resets both strips and line annotations. These map-frame
annotations are scoped to this run because the SLAM origin can change.

Each marked polygon removes wall and interior inspection targets whose
surface lies inside it, including chair fragments fitted as box faces.
Nearby box surfaces outside the polygon remain candidates. A line
annotation is available for a narrow glass wall; it does not remove nearby
fitted boxes. Navigation costmaps, robot footprint, collision checking,
and the route through the area are unchanged. A registered marker approach
still has first priority even if its estimated position lies in a marked
region. Normal exploration gain within 1.8 m of a registered marker is
reduced to 18%, recovering continuously to full gain at 2 m.

## Why the region must be marked

A trial geometric detector joined nearby small occupied islands whose
gaps were narrower than the robot's clearance. On a saved real SLAM map
it selected two unrelated side-wall groups, one close to marker 42, and
missed the operator's upper chair/glass strip. The automatic classification
therefore is **not** used to exclude any area. The explicit polygon is the
source of truth for this candidate.

## Checks and limits

- Unit tests: four mask/cooldown cases passed.
- Board control tests in the VM: 53 passed without moving the robot.
- ROS 2 package build of the separate workspace: passed.
- Read-only replay of saved decision snapshot 0036.npz at mission
  time 540.0 s with approximate polygons drawn over the operator's
  screenshot: wall groups 238 → 182; interior groups 498 → 293. This
  checks filtering, not marker success or driving performance.
- No new physical drive or matched Gazebo comparison has been run.
- The source screenshots alone do not provide map-frame coordinates for
  future sessions; mark the two regions on each fresh map.

The previous real drive selected seven fitted-face targets in 38 recorded
decisions and logged two Nav2 stalls. The saved decision metadata does
not identify table legs as the cause, so that remains a hypothesis.
