# Mixed v1.5.2.19 · Calibrated Routes

This simulation candidate builds on Finish Transit from the frozen Focus Continuity workspace. Run `python3 install.py FOCUS_WORKSPACE NEW_WORKSPACE` from this directory. Source Jazzy and build the new workspace with `colcon build --symlink-install --packages-select osp_mission`.

A visibility check in `WallBearings.choose` overwrote the camera calibration variable with the observer's map coordinates. Starting with the next track, the horizontal map coordinate was used as the focal length. The range estimate now uses a separate calibrated focal length for every track. The local detour and time checks use reachable walking metres. The previous wall score also contained a preference for continuing along the last wall and penalties for prior attempts.

A supported weak wall cue is eligible at the next selection unless the current bounded visit still has a local, uncompleted box face. A lower-priority anonymous print can no longer replace the selected supported bearing or a committed local continuation. The short-range corroboration rule, RGB confirmation and 1.5 m verification continue to apply.

Four-neighbour walking costs use SciPy on the same traversable cells, including the parent's monotone start connector. The clearance transform is reused in the same planning pass. In 88 comparisons on 44 saved SLAM grids the full masks and distances were bit-identical. On the simulation VM the old/new medians for this one subroutine were 438/38 ms. The measured whole decision on 32 physical-run snapshots was 271/198 ms (medians), with other simulation load present.
