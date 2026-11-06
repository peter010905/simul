import numpy as np
from osp_mission.landmark_identity import resolved_ids


def history(n,point):
    return [(k*200000000,np.array(point,dtype=float)) for k in range(n)]


def test_weak_same_location_id_does_not_become_fourth_target():
    rows={645:history(30,(6.673,1.731,.449)),1023:history(5,(6.628,1.703,.497)),
          249:history(5,(4.,2.,.45)),961:history(5,(6.8,.6,.45))}
    ids,conflicts=resolved_ids(rows)
    assert ids=={645,249,961} and conflicts[0]['winner']==645


def test_ambiguous_identity_is_withheld_and_can_recover():
    rows={4:history(3,(1,1,.45)),5:history(3,(1.05,1,.45))}
    assert resolved_ids(rows)[0]==set()
    rows[5]=history(12,(1.05,1,.45))
    assert resolved_ids(rows)[0]=={5}


def test_legitimate_1023_and_separate_markers_are_retained():
    rows={1023:history(3,(0,0,.45)),0:history(3,(1,0,.45)),1:history(3,(2,0,.45))}
    assert resolved_ids(rows)[0]=={0,1,1023}
