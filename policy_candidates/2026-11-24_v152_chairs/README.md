# Mixed v1.5.2.31 Chair Clutter Candidate

Parent: `Mixed v1.5.2.31 · Start Sweep`. The parent is copied, never edited.

The previous 4101 chair run found 1/3 markers in 600 simulated seconds. Its goal
trace included repeated generic surface inspections near dense furniture; this
candidate reduces the priority and continuation of those goals. Explicit box
faces, mapped wall goals, marker decoding, Nav2 safety and approach thresholds
remain as in the parent. In the same fixture, it verified 3/3 IDs at 453.0 s,
but hit the bumper once. A subsequent physical trial contacted the bumper four
times. The operator stopped it and confirmed zero wheel speed. At the user's
request, the 8767 board lists this separate candidate as selectable again,
with the four-contact warning in its name. Selecting it alone does not move
the robot; preparation and departure remain separate actions with the existing
sensor, clock and Nav2 safety checks. The original Start Sweep workspace
remains available.

[Matched chair run and safety finding](../../experiments/2026-11-24_chair_clutter/README.md)

Install on the ROS VM:

```
python3 install.py /home/ubuntu/v15231_start_sweep_640_1124_ws /home/ubuntu/v15231_chair_clutter_640_1124_ws
cd /home/ubuntu/v15231_chair_clutter_640_1124_ws
colcon build --symlink-install --packages-select osp_mission
```

Compare against the parent using the same chair fixture, seed, initial pose,
markers, 640×480 at 5 fps, 600-second limit, and recorded full trajectory.
Report marker IDs registered, stationary time, surface goal repetitions,
navigation stalls, distance, collisions, and wall/box coverage. A second seed
should be used before a performance conclusion.
