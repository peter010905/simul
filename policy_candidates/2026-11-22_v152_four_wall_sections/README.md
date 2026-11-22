# Mixed v1.5.2.5 Four Wall Sections (simulation candidate)

Parent: v1.5.2.4 Four Wall Glance. Both older versions are preserved.

This corrects 45-degree wall-normal ties in the north/east/south/west scheduler. It evaluates the far end, near end and strongest local section of each unobserved wall direction. A later wall view receives credit for moving to a new tangential section, so one obstacle-heavy corner cannot consume all wall appointments. The viewed mask still comes from actual camera observations; a dispatched goal is only an attempt. Geometry, Nav2 safety, marker confirmation, camera 480×480 at 5 fps, and real robot settings are unchanged.
