# Mixed v1.5.2.21 · Bearing Advance

This separate simulation candidate builds on Wall Evidence. Run `python3 install.py FOCUS_WORKSPACE NEW_WORKSPACE` and build the staged ROS package as for the parent.

In the full Finish Transit run on seed 550866253 the detector decoded two box IDs on 16 and 26 frames. All 42 lacked a usable range. The ground-truth evaluator measured minimum distances of 4.57 and 4.63 m. The vehicle spent 43 of its 51 goals on wall surfaces. A short turn to face the print did not create a translation goal.

Three native decodes of the same ID with consistent pixels, origin and world bearing can now propose a short move into visible, mapped free space. The proposal runs at an ordinary goal boundary. Collision checks, confirmed-target approach, bounded local box work and recovery retain their prior behavior. A ray supplies a direction only. The resulting goal never enters the marker registry or counts toward successful approach.

The forward samples are 0.65, 1.0 and 1.4 m from the observing position. The planner rejects a move that advances less than 0.4 m along the ray, exceeds 2.4 m of estimated walking, crosses unknown cells, or fails the existing Nav2 and failed-goal checks. A second move for the same ID requires a new three-frame observation from at least 0.65 m away and a 12% increase in measured edge size. The cap is two dispatched looks per ID and four per mission. The final judgement comes from a separate evaluator with the actual marker identities, 1.5 m approach evidence, map position error, false IDs and bumper events.
