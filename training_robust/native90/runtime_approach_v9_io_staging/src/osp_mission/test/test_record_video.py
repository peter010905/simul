"""Validate media timing/decodability with synthetic fixtures, not Gazebo results."""
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest

from osp_mission.record_video import SimulationVideo,map_panel
from osp_mission.world import generate


def test_recording_camera_has_no_collision_or_scenario_mutation(tmp_path):
    scenario=Path(__file__).parents[1]/'config/robust_demo.yaml'
    normal,_=generate(scenario,tmp_path/'normal')
    recording,_=generate(scenario,tmp_path/'recorded',recording=True)
    a=ET.parse(normal).getroot();b=ET.parse(recording).getroot()
    observer=b.find(".//model[@name='mission_observer']")
    assert observer is not None and observer.find('.//collision') is None
    assert observer.find('.//sensor').attrib['type']=='camera'
    b.find('world').remove(observer)
    assert ET.tostring(a)==ET.tostring(b)


def test_h264_is_decodable_and_preserves_simulation_time_gaps(tmp_path):
    first=np.zeros((720,1280,3),np.uint8);first[:]=(20,30,40)
    last=first.copy();last[:]=(150,160,170)
    video=SimulationVideo(tmp_path)
    video.write(first,7.8);video.write(last,9.0);video.close({'fixture':True})
    cap=cv2.VideoCapture(str(video.path));means=[]
    while True:
        ok,image=cap.read()
        if not ok:break
        assert image.shape==(720,1280,3)
        means.append(image.mean())
    cap.release()
    assert len(means)==7 and all(value<40 for value in means[:-1]) and means[-1]>140
    metadata=json.loads((tmp_path/'recording.json').read_text())
    assert metadata['held_frames']==5 and metadata['duration_seconds']==pytest.approx(1.4)
    rows=list(csv.DictReader((tmp_path/'frames.csv').open()))
    assert float(rows[-1]['video_seconds'])==pytest.approx(1.2)
    assert float(rows[-1]['source_sim_seconds'])==9.


def test_no_frames_does_not_claim_a_video_and_never_overwrites(tmp_path):
    video=SimulationVideo(tmp_path);video.close()
    assert json.loads((tmp_path/'recording.json').read_text())['video'] is None
    assert not (tmp_path/'mission.mp4').exists()
    (tmp_path/'mission.mp4').write_bytes(b'existing')
    with pytest.raises(FileExistsError):SimulationVideo(tmp_path)


def test_map_panel_handles_negative_origins_and_rotations_without_truth_input():
    grid=np.full((100,160),-1,np.int8);grid[10:70,40:90]=0;grid[10,40:90]=100
    image=map_panel(grid,.05,(-2.,-1.,.3),robot=(0.,0.,1.),
        scan=((0.,0.),np.array([[.5,1.],[-.5,.5]])),path=[(0.,0.),(.1,0.)],targets=[(7,1.,1.)])
    assert image.shape==(626,460,3) and image.dtype==np.uint8
