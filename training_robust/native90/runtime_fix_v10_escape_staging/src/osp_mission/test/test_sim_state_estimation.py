"""Simulated state estimation uses real sensor messages, not world truth."""
from pathlib import Path
from xml.dom.minidom import parseString
import yaml
from osp_mission.sensor_model import configure_sensors


def test_imu_rate_and_raw_wheel_odometry_are_preserved_for_fusion():
    doc=parseString('<robot><remapping>~/odom:=odom</remapping><sensor type="imu"><update_rate>62</update_rate></sensor><sensor type="gpu_lidar"><update_rate>62</update_rate></sensor></robot>')
    configure_sensors(doc,fuse_imu=True)
    assert doc.getElementsByTagName('remapping')[0].firstChild.data=='~/odom:=wheel/odom'
    assert [float(e.firstChild.data) for e in doc.getElementsByTagName('update_rate')]==[62.,10.]


def test_fusion_never_uses_world_position_or_absolute_imu_orientation():
    config=Path(__file__).parents[1]/'config'
    fusion=yaml.safe_load((config/'sim_ekf.yaml').read_text())['/**']['ros__parameters']
    assert fusion['odom0']=='wheel/odom' and fusion['imu0']=='imu'
    assert [i for i,v in enumerate(fusion['odom0_config']) if v]==[6,7]
    assert [i for i,v in enumerate(fusion['imu0_config']) if v]==[11]
    control=yaml.safe_load((config/'sim_control.yaml').read_text())['/**']['diffdrive_controller']['ros__parameters']
    assert control['enable_odom_tf'] is False and fusion['publish_tf'] is True
