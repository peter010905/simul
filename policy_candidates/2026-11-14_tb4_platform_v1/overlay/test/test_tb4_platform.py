import copy
from pathlib import Path
from types import SimpleNamespace as S
import xml.dom.minidom

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from osp_mission.tb4_platform import configure_model,load_profile,contract_errors
from osp_mission.mission import Mission

PACKAGE=Path(__file__).resolve().parents[1]


def observation():
    p=load_profile(PACKAGE)
    value={key:dict(count=20,age_s=.1,rate_hz=rate) for key,rate in [('odom',20),('scan',5.5),('rgb',30),('imu',100),('hazards',62)]}
    value['odom'].update(frame='odom',child_frame='base_link')
    value['rgb'].update(width=250,height=250,frame='oakd_rgb_camera_optical_frame')
    value.update(camera_info=dict(width=250,height=250,frame='oakd_rgb_camera_optical_frame',valid_intrinsics=True),
        odom_publishers=1,odom_tf_authorities=1,velocity_types=[p['velocity_type']],velocity_subscribers=1,sensor_tf_connected=True)
    return p,value


def test_good_hardware_interface_is_ready():
    p,value=observation();assert contract_errors(value,p)==[]


@pytest.mark.parametrize('key,changes',[
    ('odom',{'age_s':3.}),('rgb',{'width':960}),('imu',{'count':0}),('scan',{'rate_hz':0}),
    ('odom',{'child_frame':'world'}),('camera_info',{'valid_intrinsics':False}),('hazards',{'count':0})])
def test_missing_stale_or_wrong_input_cannot_start(key,changes):
    p,value=observation();value[key].update(changes);assert contract_errors(value,p)


@pytest.mark.parametrize('key,value', [('velocity_types',['geometry_msgs/msg/Twist']),('velocity_subscribers',0),
    ('odom_publishers',2),('odom_tf_authorities',2),('unexpected_tf_publishers',['/state_estimator']),('sensor_tf_connected',False)])
def test_conflicting_or_disconnected_graph_cannot_start(key,value):
    p,observed=observation();observed[key]=value;assert contract_errors(observed,p)


def test_original_robot_geometry_and_odom_are_preserved():
    path=Path(get_package_share_directory('turtlebot4_description'))/'urdf/standard/turtlebot4.urdf.xacro'
    doc=xacro.process_file(str(path),mappings={'gazebo':'ignition','namespace':''})
    original=[node.toxml() for node in doc.getElementsByTagName('inertial')]+[node.toxml() for node in doc.getElementsByTagName('joint')]
    configure_model(doc,load_profile(PACKAGE),PACKAGE/'config/tb4_control.yaml')
    assert original==[node.toxml() for node in doc.getElementsByTagName('inertial')]+[node.toxml() for node in doc.getElementsByTagName('joint')]
    assert '~/odom:=odom' in doc.toxml() and 'wheel/odom' not in doc.toxml()
    sensors={node.getAttribute('name'):node for node in doc.getElementsByTagName('sensor')}
    def value(name,tag):return sensors[name].getElementsByTagName(tag)[0].firstChild.data
    assert value('rgbd_camera','width')==value('rgbd_camera','height')=='250'
    assert float(value('rgbd_camera','update_rate'))==30
    assert float(value('rplidar','update_rate'))==5.5
    control=yaml.safe_load((PACKAGE/'config/tb4_control.yaml').read_text())['/**']['diffdrive_controller']['ros__parameters']
    assert control['enable_odom_tf'] is True and control['open_loop'] is False
    assert control['left_wheel_radius_multiplier']==control['right_wheel_radius_multiplier']==1.


def test_platform_gate_rejects_stale_status_and_reports_reason():
    owner=S(require_platform_contract=True,platform_contract={'passed':True},platform_contract_received=1.,now=lambda:5.)
    assert Mission.readiness_reason(owner)=='platform_contract_unverified'
    owner.platform_contract_received=5.;owner.platform_contract={'passed':False}
    assert Mission.readiness_reason(owner)=='platform_contract_unverified'
