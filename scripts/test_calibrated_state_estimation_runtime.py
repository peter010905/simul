"""State estimation uses calibrated encoder velocity and measured IMU angular velocity only."""
import ast
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
    package=Path(__file__).parents[1]
    fusion=yaml.safe_load((package/'config/sim_ekf.yaml').read_text())['/**']['ros__parameters']
    assert fusion['twist0']=='wheel/twist_calibrated' and fusion['imu0']=='imu' and 'odom0' not in fusion
    assert [i for i,v in enumerate(fusion['twist0_config']) if v]==[6,7]
    assert [i for i,v in enumerate(fusion['imu0_config']) if v]==[11]
    source=ast.parse((package/'osp_mission/calibrated_wheel_twist.py').read_text())
    subscriptions=[call.args[1].value for call in ast.walk(source) if isinstance(call,ast.Call)
        and isinstance(call.func,ast.Attribute) and call.func.attr=='create_subscription']
    assert subscriptions==['wheel/odom']
    publishers=[call.args[1].value for call in ast.walk(source) if isinstance(call,ast.Call)
        and isinstance(call.func,ast.Attribute) and call.func.attr=='create_publisher']
    assert publishers==['wheel/twist_calibrated']
    control=yaml.safe_load((package/'config/sim_control.yaml').read_text())['/**']['diffdrive_controller']['ros__parameters']
    assert control['enable_odom_tf'] is False and fusion['publish_tf'] is True
